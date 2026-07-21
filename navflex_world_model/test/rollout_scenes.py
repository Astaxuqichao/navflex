#!/usr/bin/env python3
"""Synthetic ego-view rollouts for benchmarking a critic.

The photos shipped with LingBot are cinematic (a rider on a dragon, a low-angle
building) -- nothing a ground robot would ever see, so a good critic rejects
them for the wrong reason and the benchmark measures nothing. These scenes are
deterministic, in-domain, and differ from each other in exactly one respect,
which is what makes a verdict attributable.

Forward motion is drawn, not cropped: the corridor's end wall grows as the
camera advances, so the frames really are a translation along the optical axis.
"""

import io
import os
import math
from typing import List

from PIL import Image, ImageDraw

W, H = 768, 432
CX, CY = W // 2, int(H * 0.46)  # vanishing point, slightly above centre


def _jpeg(image: Image.Image) -> bytes:
    buffer = io.BytesIO()
    image.save(buffer, format='JPEG', quality=88)
    return buffer.getvalue()


def _corridor(progress: float, obstacle: float = 0.0) -> Image.Image:
    """One frame. ``progress`` in [0,1] is how far the camera has advanced."""
    img = Image.new('RGB', (W, H), (28, 30, 34))
    d = ImageDraw.Draw(img)

    # The end wall subtends a larger angle as we approach it. Keep the growth
    # modest: if it fills the frame the perspective cues vanish and the scene
    # stops reading as a corridor at all.
    half_w = (0.085 + 0.125 * progress) * W
    half_h = (0.115 + 0.125 * progress) * H
    l, r = CX - half_w, CX + half_w
    t, b = CY - half_h, CY + half_h

    d.polygon([(0, H), (W, H), (r, b), (l, b)], fill=(78, 74, 70))       # floor
    d.polygon([(0, 0), (W, 0), (r, t), (l, t)], fill=(196, 198, 202))    # ceiling
    d.polygon([(0, 0), (l, t), (l, b), (0, H)], fill=(150, 152, 156))    # left wall
    d.polygon([(W, 0), (r, t), (r, b), (W, H)], fill=(132, 134, 138))    # right wall

    # Floor seams and ceiling lights: perspective cues, and something for the
    # model to notice moving between frames.
    for k in range(1, 7):
        f = (k - progress * 1.6) / 6.0
        if not 0.02 < f < 1.0:
            continue
        y = b + (H - b) * (f ** 1.7)
        d.line([(l - (l - 0) * (f ** 1.7), y), (r + (W - r) * (f ** 1.7), y)],
               fill=(96, 92, 88), width=2)
        yc = t - t * (f ** 1.7)
        d.rectangle([CX - 26 / max(f, 0.08), yc - 3, CX + 26 / max(f, 0.08), yc + 3],
                    fill=(238, 240, 235))

    d.rectangle([l, t, r, b], fill=(112, 116, 120), outline=(60, 62, 66), width=2)

    # An open doorway straight ahead: the corridor continues, nothing blocks it.
    dw, dh = half_w * 0.42, half_h * 0.66
    d.rectangle([CX - dw, b - 2 * dh, CX + dw, b], fill=(34, 38, 44))

    if obstacle > 0.0:
        # A crate on the floor between the camera and the end wall, closing
        # faster than the wall does. By the last frame it blocks the path but
        # does not swallow the frame -- the corridor must stay legible.
        s = obstacle * (0.035 + 0.145 * progress)
        ow, oh = s * W, s * H * 1.35
        oy = b + (H - b) * (0.72 - 0.42 * progress)  # rides toward the camera
        d.rectangle([CX - ow, oy - oh, CX + ow, oy], fill=(84, 62, 44),
                    outline=(30, 22, 16), width=3)
        d.line([(CX - ow, oy - oh), (CX + ow, oy)], fill=(52, 38, 26), width=2)
        d.line([(CX - ow, oy), (CX + ow, oy - oh)], fill=(52, 38, 26), width=2)

    return img


def clear_corridor(n: int = 6) -> List[bytes]:
    """Driving straight down an empty corridor toward an open doorway."""
    return [_jpeg(_corridor(i / (n - 1))) for i in range(n)]


def blocked_corridor(n: int = 6) -> List[bytes]:
    """Same corridor, same motion, but a crate fills the path by the last frame."""
    return [_jpeg(_corridor(i / (n - 1), obstacle=1.0)) for i in range(n)]


def blank(n: int = 6) -> List[bytes]:
    """No information at all. Approving here is disqualifying."""
    return [_jpeg(Image.new('RGB', (W, H), (92, 108, 124))) for _ in range(n)]


def real_segment(directory: str, segment: int, n: int = 6) -> List[bytes]:
    """Frames cut from a MATRiX rosbag by test/extract_real_rollout.py.

    Preferred over the synthetic corridor: these are the robot's own camera,
    which is the distribution LingBot conditions on and therefore the one the
    critic will actually see. Downscaled to keep the image-token bill sane.
    """
    import glob

    paths = sorted(glob.glob(os.path.join(directory, f'seg{segment}_*.jpg')))
    if not paths:
        raise FileNotFoundError(f'no seg{segment}_*.jpg under {directory}')
    step = max(1, len(paths) // n)
    frames = []
    for path in paths[::step][:n]:
        with Image.open(path) as im:
            frames.append(_jpeg(im.convert('RGB').resize((W, H), Image.LANCZOS)))
    return frames


if __name__ == '__main__':
    import os
    out = os.environ.get('OUT', '/tmp/navflex_scenes')
    os.makedirs(out, exist_ok=True)
    for name, frames in (('clear', clear_corridor()),
                         ('blocked', blocked_corridor())):
        for i, frame in enumerate(frames):
            with open(f'{out}/{name}_{i}.jpg', 'wb') as fh:
                fh.write(frame)
        print(f'{name}: {len(frames)} frames -> {out}/{name}_*.jpg')
