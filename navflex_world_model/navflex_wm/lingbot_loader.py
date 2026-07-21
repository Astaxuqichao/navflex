#!/usr/bin/env python3
"""Build WanI2VCausal so that an 18.5B DiT fits on a 32 GB card.

The upstream constructor cannot do it. `wan/image2video.py` reads:

    self.model = self._configure_model(..., convert_model_dtype=...).to(self.device)

`_configure_model` honours `init_on_cpu` and leaves the model on the host -- and
then the caller unconditionally `.to(self.device)`s it anyway. So the whole DiT
(18.54B params, 37.1 GB in bf16) must land on the GPU at construction, which a
32 GB 5090 cannot hold. `--offload_model` does not save you: `generate()` never
moves the model onto the device, it only calls `self.model.cpu()` *after* the
rollout. It is a post-hoc release, not layer streaming.

What actually works: diffusers group offloading. `WanModelFast` is a ModelMixin
with `_no_split_modules = ['WanAttentionBlock']` and
`_supports_group_offloading = True`, so its 40 blocks can live in pinned host
memory and be prefetched onto the GPU a group at a time, on a side CUDA stream.
Peak VRAM becomes a few blocks plus activations.

This module patches the load path rather than editing the vendored clone, so
`lingbot-world-v2` stays a pristine checkout.
"""

from __future__ import annotations

import sys
import types
from typing import Any, Optional

DEFAULT_REPO = '/home/jiangbz/project/world_model/lingbot-world-v2'


class LingBotLoadError(RuntimeError):
    pass


def _neutralise_device_moves(model):
    """Make `.to(device)` a no-op while still allowing dtype casts.

    Once group-offloading hooks are attached, moving the module wholesale to
    CUDA would undo them (and OOM). The constructor calls `.to(self.device)`
    regardless, so the instance has to refuse.
    """
    import torch

    original_to = model.to

    def guarded_to(self, *args, **kwargs):
        del self
        target = kwargs.get('device')
        moves_to_device = target is not None or any(
            isinstance(a, (str, torch.device)) for a in args)
        if moves_to_device:
            return model
        return original_to(*args, **kwargs)

    model.to = types.MethodType(guarded_to, model)
    return model


def build_pipeline(
    ckpt_dir: str,
    lingbot_repo: str = DEFAULT_REPO,
    device_id: int = 0,
    blocks_per_group: int = 2,
    local_attn_size: int = 18,
    sink_size: int = 6,
    use_stream: bool = False,
    offload_to_disk_path: Optional[str] = None,
    task: str = 'i2v-A14B',
):
    """Return ``(pipe, cfg)`` with the DiT group-offloaded off the GPU.

    ``use_stream`` defaults to False on purpose. Stream prefetch pins the
    offloaded weights in page-locked host memory, which is a *second* copy: with
    a 37 GB bf16 DiT the kernel OOM-killed the process at
    ``anon-rss 30 GB + shmem-rss 27 GB``. Without streams the weights sit in
    ordinary pageable memory (~37 GB) and umT5 still fits beside them.

    Set ``offload_to_disk_path`` on a host with less RAM: weights are paged from
    NVMe instead, which trades throughput for a flat memory profile.
    """
    if lingbot_repo not in sys.path:
        sys.path.insert(0, lingbot_repo)

    import torch

    import wan
    import wan.image2video as image2video
    from wan.configs import WAN_CONFIGS

    if not torch.cuda.is_available():
        raise LingBotLoadError('no CUDA device')

    onload = torch.device(f'cuda:{device_id}')
    offload = torch.device('cpu')
    original_from_pretrained = image2video.WanModelFast.from_pretrained

    def loading_with_group_offload(*args: Any, **kwargs: Any):
        model = original_from_pretrained(*args, **kwargs)
        # block_level: whole WanAttentionBlocks move as a unit. leaf_level would
        # stream every Linear and is far slower.
        offload_kwargs = dict(
            onload_device=onload,
            offload_device=offload,
            offload_type='block_level',
            num_blocks_per_group=blocks_per_group,
            use_stream=use_stream,
            non_blocking=use_stream,
            # Only meaningful with streams; it re-pins lazily instead of holding
            # a full pinned copy.
            low_cpu_mem_usage=use_stream,
        )
        if offload_to_disk_path:
            offload_kwargs['offload_to_disk_path'] = offload_to_disk_path
        model.enable_group_offload(**offload_kwargs)
        return _neutralise_device_moves(model)

    image2video.WanModelFast.from_pretrained = loading_with_group_offload
    try:
        cfg = WAN_CONFIGS[task]
        pipe = wan.WanI2VCausal(
            config=cfg,
            checkpoint_dir=ckpt_dir,
            device_id=device_id,
            rank=0,
            t5_fsdp=False,
            dit_fsdp=False,
            use_sp=False,
            # umT5 is ~11 GB and runs once per plan; the GPU budget belongs to
            # the DiT and the VAE decode.
            t5_cpu=True,
            # from_pretrained already loaded bf16; a second cast would move the
            # offloaded weights back to one device.
            convert_model_dtype=False,
            local_attn_size=local_attn_size,
            sink_size=sink_size,
            infer_mode='causal_fast',
        )
    finally:
        image2video.WanModelFast.from_pretrained = original_from_pretrained

    return pipe, cfg


def vram_report(device_id: int = 0) -> str:
    import torch

    free, total = torch.cuda.mem_get_info(device_id)
    allocated = torch.cuda.memory_allocated(device_id)
    peak = torch.cuda.max_memory_allocated(device_id)
    gib = 1024 ** 3
    return (f'VRAM allocated {allocated / gib:.1f} GiB, peak {peak / gib:.1f} GiB, '
            f'free {free / gib:.1f} / {total / gib:.1f} GiB')
