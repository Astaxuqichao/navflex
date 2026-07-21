#!/usr/bin/env bash
# Build the conda env the LingBot-World-v2 sidecar runs in.
#
# Python 3.10 is not arbitrary: the only flash-attn build we have for this box
# is a cp310 wheel (sm_120 / torch2.8 / cu12). Compiling flash-attn from source
# takes over an hour, and lingbot needs it -- wan/modules/model.py calls
# flash_attention() directly, past the scaled_dot_product_attention fallback.
set -euo pipefail

ENV_NAME="${ENV_NAME:-lingbot_wm}"
FLASH_ATTN_WHEEL="${FLASH_ATTN_WHEEL:-/home/jiangbz/project/vla/Isaac-GR00T/flash_attn-2.8.3+cu12torch2.8cxx11abiTRUE-cp310-cp310-linux_x86_64.whl}"
LINGBOT_REPO="${LINGBOT_REPO:-/home/jiangbz/project/world_model/lingbot-world-v2}"

source "$(conda info --base)/etc/profile.d/conda.sh"

echo "== creating $ENV_NAME (python 3.10) =="
conda create -n "$ENV_NAME" python=3.10 -y
conda activate "$ENV_NAME"

# download.pytorch.org resolves without the proxy, whose quota is metered.
echo "== torch 2.8.0 + cu128 (Blackwell sm_120) =="
pip install torch==2.8.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128

echo "== lingbot deps (flash_attn installed separately from a local wheel) =="
grep -v '^flash_attn' "${LINGBOT_REPO}/requirements.txt" > /tmp/lingbot_reqs.txt
# ConfigMixin/ModelMixin moved; lingbot's own floor of 0.31 is the real one.
pip install -r /tmp/lingbot_reqs.txt

echo "== flash-attn from local wheel =="
if [ ! -f "$FLASH_ATTN_WHEEL" ]; then
  echo "missing $FLASH_ATTN_WHEEL; falling back to a source build (slow)" >&2
  pip install flash-attn --no-build-isolation
else
  pip install "$FLASH_ATTN_WHEEL"
fi

echo "== verify =="
python - <<'PY'
import torch, flash_attn
print('torch       ', torch.__version__)
print('cuda        ', torch.version.cuda, '| device', torch.cuda.get_device_name(0))
print('capability  ', torch.cuda.get_device_capability(0))
print('flash_attn  ', flash_attn.__version__)

# causal_fast runs sliding-window attention (local_attn_size=18). Prove the
# kernel accepts it on this architecture before trusting a 37 GB model load.
from flash_attn.flash_attn_interface import flash_attn_func
q = torch.randn(1, 64, 8, 64, device='cuda', dtype=torch.bfloat16)
out = flash_attn_func(q, q, q, causal=True, window_size=(18, 0))
print('sliding-window flash_attn on this GPU: OK', tuple(out.shape))
PY

echo
echo "done. activate with:  conda activate $ENV_NAME"
