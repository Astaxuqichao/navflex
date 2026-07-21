"""Find where an imagined rollout stops being a single continuous shot.

The world model has no physics of contact. Driving a plan straight into a
cabinet, it renders the cabinet faithfully -- filling more and more of the view
-- and then, at the moment the camera would be inside it, it has nothing
plausible left to draw and cuts to an invented scene. Measured on exactly that
plan: frame-to-frame difference sits at ~14 for fourteen frames, spikes to 42
and 67 at frames 15-16, and settles into a different room containing a blue
humanoid robot that does not exist.

That tear is not evidence. A critic shown those frames rejected the plan -- but
its stated reason was the hallucinated robot, not the cabinet. It was right by
luck. The same model just as easily invents an empty corridor, and then the
critic approves a plan that drives into furniture.

So the gate measures its own rollout before anyone judges it, keeps the
continuous prefix, and says plainly where the imagination gave out.
"""

import io
from typing import List, Sequence, Tuple

import numpy as np

# A tear is a frame that differs from its predecessor far more than the clip's
# own typical motion. 3x the running median is comfortably above the noise:
# coherent rollouts measure 1.1-1.6 by this ratio, the torn one measured 4.8.
TEAR_RATIO = 3.0

# Below this the prefix carries no motion worth judging.
MIN_COHERENT_FRAMES = 3

# The median is meaningless until a few frames have established what "typical"
# motion looks like for this clip.
WARMUP = 4


def _luma(jpeg: bytes, width: int = 96) -> np.ndarray:
    from PIL import Image

    img = Image.open(io.BytesIO(jpeg)).convert('L')
    height = max(1, round(img.height * width / img.width))
    return np.asarray(img.resize((width, height)), dtype=np.float32)


def frame_diffs(frames: Sequence[bytes]) -> np.ndarray:
    """Mean absolute luma difference between consecutive frames."""
    if len(frames) < 2:
        return np.zeros(0, dtype=np.float32)
    lumas = [_luma(f) for f in frames]
    return np.array([np.abs(a - b).mean() for a, b in zip(lumas, lumas[1:])],
                    dtype=np.float32)


def first_tear(frames: Sequence[bytes],
               tear_ratio: float = TEAR_RATIO,
               warmup: int = WARMUP) -> int:
    """Index of the first frame that broke continuity, or -1 if none did.

    Compares each step against the median of the steps *before* it, so a tear
    cannot raise the very threshold meant to catch it -- which is what happens
    if you take the median of the whole clip.
    """
    diffs = frame_diffs(frames)
    if len(diffs) <= warmup:
        return -1
    for i in range(warmup, len(diffs)):
        baseline = float(np.median(diffs[:i]))
        if baseline > 1e-6 and diffs[i] > tear_ratio * baseline:
            return i + 1        # diffs[i] is the step INTO frame i+1
    return -1


def coherent_prefix(frames: Sequence[bytes],
                    tear_ratio: float = TEAR_RATIO) -> Tuple[List[bytes], int]:
    """Return ``(frames up to the tear, tear_index)``; tear_index -1 if intact."""
    tear = first_tear(frames, tear_ratio=tear_ratio)
    if tear < 0:
        return list(frames), -1
    return list(frames[:tear]), tear
