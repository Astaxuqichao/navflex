#!/usr/bin/env python3
"""World-model generator backends.

The generator answers one question: given the frame the robot sees now and the
camera trajectory it is about to fly, what does the near future look like? It
returns frames; judging them is the critic's job (see :mod:`critic`).

LingBot-World-v2 needs ~37 GB in bf16 and a torch/flash-attn stack that has no
business inside an rclpy process, so it lives behind HTTP in a sidecar
(``scripts/lingbot_server.py``). :class:`NullBackend` keeps the rest of the gate
runnable -- and testable -- without any of that.
"""

from __future__ import annotations

import base64
import io
import json
import urllib.error
import urllib.request
from abc import ABC, abstractmethod
from typing import List

import numpy as np


class WorldModelError(RuntimeError):
    pass


def _encode_npy(array: np.ndarray) -> str:
    buffer = io.BytesIO()
    np.save(buffer, array, allow_pickle=False)
    return base64.b64encode(buffer.getvalue()).decode('ascii')


def decode_npy(encoded: str) -> np.ndarray:
    return np.load(io.BytesIO(base64.b64decode(encoded)), allow_pickle=False)


class Rollout:
    """An imagined future: the frames, and where the video was written."""

    def __init__(self, frames: List[bytes], video_path: str = '', note: str = ''):
        self.frames = frames
        self.video_path = video_path
        self.note = note

    @property
    def frame_count(self) -> int:
        return len(self.frames)


class WorldModelBackend(ABC):
    name = 'abstract'

    @abstractmethod
    def imagine(
        self,
        image_jpeg: bytes,
        c2ws: np.ndarray,
        intrinsics: np.ndarray,
        prompt: str,
    ) -> Rollout:
        """Roll the world forward along ``c2ws`` starting from ``image_jpeg``."""

    def health(self) -> str:
        return 'ok'


class NullBackend(WorldModelBackend):
    """Runs the gate without a world model.

    Every plan is imagined as "no frames", which the critic reads as no
    evidence. Useful for wiring up and exercising the ROS surface before the
    86 GB of weights land, and as the fallback when the sidecar is down.
    """

    name = 'null'

    def imagine(self, image_jpeg, c2ws, intrinsics, prompt) -> Rollout:
        del image_jpeg, intrinsics, prompt
        return Rollout(
            frames=[],
            note=f'null backend: would have imagined {len(c2ws)} frames')

    def health(self) -> str:
        return 'ok (null backend, no generation)'


class LingBotHttpBackend(WorldModelBackend):
    """Talks to the persistent LingBot-World-v2 sidecar.

    The sidecar holds the pipeline warm across calls: constructing it costs
    ~100 s and a prewarm another ~7 s, which we refuse to pay per plan.
    """

    name = 'lingbot_http'

    def __init__(self, url: str, timeout: float = 600.0, sample_stride: int = 8):
        self.url = url.rstrip('/')
        self.timeout = timeout
        # The critic reads a handful of frames, not the whole rollout; sending
        # 81 base64 JPEGs over HTTP for nothing is wasteful.
        self.sample_stride = max(1, sample_stride)

    def imagine(self, image_jpeg, c2ws, intrinsics, prompt) -> Rollout:
        # Ship the control signal inline rather than as a path: the sidecar may
        # run in another container, and it needs no shared filesystem.
        payload = {
            'image_b64': base64.b64encode(image_jpeg).decode('ascii'),
            'poses_npy_b64': _encode_npy(c2ws.astype(np.float32)),
            'intrinsics_npy_b64': _encode_npy(intrinsics.astype(np.float32)),
            'prompt': prompt,
            'frame_num': int(len(c2ws)),
            'sample_stride': self.sample_stride,
        }
        data = self._post('/imagine', payload)

        frames = [base64.b64decode(f) for f in data.get('frames_b64', [])]
        if not frames:
            raise WorldModelError(
                f"lingbot sidecar returned no frames: {data.get('message', '')}")
        return Rollout(
            frames=frames,
            video_path=str(data.get('video_path', '')),
            note=str(data.get('message', '')))

    def health(self) -> str:
        try:
            data = self._get('/health')
        except WorldModelError as exc:
            return f'unavailable: {exc}'
        return str(data.get('status', 'unknown'))

    def _post(self, route: str, payload: dict) -> dict:
        request = urllib.request.Request(
            f'{self.url}{route}',
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST')
        return self._send(request)

    def _get(self, route: str) -> dict:
        return self._send(urllib.request.Request(f'{self.url}{route}', method='GET'))

    def _send(self, request) -> dict:
        # The sidecar is local; an inherited proxy would black-hole the request.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')[:400]
            raise WorldModelError(f'sidecar HTTP {exc.code}: {detail}') from exc
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise WorldModelError(f'sidecar unreachable at {self.url}: {exc}') from exc


def build_backend(
    kind: str,
    lingbot_url: str = 'http://127.0.0.1:8100',
    timeout: float = 600.0,
    sample_stride: int = 8,
) -> WorldModelBackend:
    kind = (kind or 'null').strip().lower()
    if kind in ('null', 'none', 'off'):
        return NullBackend()
    if kind in ('lingbot_http', 'lingbot', 'http'):
        return LingBotHttpBackend(lingbot_url, timeout=timeout, sample_stride=sample_stride)
    raise WorldModelError(f"unknown world model backend '{kind}'")
