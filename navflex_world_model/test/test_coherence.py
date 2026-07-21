#!/usr/bin/env python3
"""Offline checks for tear detection in an imagined rollout.

Synthetic frames, so this runs without a GPU. The numbers it asserts against
come from real rollouts: coherent clips measure a max/median frame-difference
ratio of 1.1-1.6, and the clip that drove into a cabinet measured 5.65 with the
break at frame 16.
"""

import io
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from navflex_wm.coherence import (  # noqa: E402
    MIN_COHERENT_FRAMES,
    coherent_prefix,
    first_tear,
    frame_diffs,
)

failures = []


def check(name, condition, detail=''):
    if condition:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        failures.append(name)


def jpeg(arr):
    from PIL import Image
    buf = io.BytesIO()
    Image.fromarray(arr.astype(np.uint8)).save(buf, format='JPEG', quality=92)
    return buf.getvalue()


WIN = 160
# Big blocks, not fine noise. A room is low spatial frequency: nudging the
# camera one step changes few pixels, so a scene cut stands out. Fine noise
# makes every step look like a cut and the fixture proves nothing. At this
# blockiness a synthetic cut measures a ratio of 5.5 -- the real rollout that
# drove into a cabinet measured 5.65.
BLOCK = 32


def pan(n, step=6, seed=0):
    """A camera drifting steadily across a textured scene: continuous motion."""
    rng = np.random.default_rng(seed)
    width = WIN + step * n + 8          # never pan off the canvas
    coarse = rng.integers(40, 215, size=(120 // BLOCK + 2, width // BLOCK + 2),
                          dtype=np.uint8)
    scene = np.repeat(np.repeat(coarse, BLOCK, 0), BLOCK, 1)[:120, :width]
    return [jpeg(scene[:, i * step:i * step + WIN]) for i in range(n)]


print('== a continuous pan has no tear ==')
smooth = pan(20)
d = frame_diffs(smooth)
check('diffs computed for every step', len(d) == len(smooth) - 1, len(d))
check('no tear reported', first_tear(smooth) == -1, first_tear(smooth))
kept, tear = coherent_prefix(smooth)
check('every frame kept', len(kept) == len(smooth) and tear == -1)

print('== a cut to another scene is caught, at the right frame ==')
torn = pan(16) + pan(13, seed=99)          # frame 16 belongs to a different room
t = first_tear(torn)
check('tear detected', t > 0, t)
check('tear located at the cut (frame 16)', t == 16, t)
kept, t2 = coherent_prefix(torn)
check('prefix stops before the invented frames', len(kept) == 16, len(kept))
check('prefix is the continuous part',
      all(a == b for a, b in zip(kept, torn[:16])))

print('== the tear must not raise the threshold that catches it ==')
# Taking the median of the WHOLE clip lets a long invented tail drag the
# baseline up until the cut looks ordinary. Baseline must use only the past.
long_tail = pan(8) + pan(30, step=20, seed=7)    # tail moves much faster
check('a fast invented tail is still a tear', first_tear(long_tail) == 8,
      first_tear(long_tail))

print('== degenerate inputs do not crash or lie ==')
check('empty rollout has no diffs', len(frame_diffs([])) == 0)
check('single frame has no diffs', len(frame_diffs(pan(1))) == 0)
check('single frame reports no tear', first_tear(pan(1)) == -1)
check('a clip shorter than the warmup reports no tear', first_tear(pan(3)) == -1)
kept, tear = coherent_prefix(pan(2))
check('two frames survive intact', len(kept) == 2 and tear == -1)

print('== a still clip has no motion to divide by ==')
still = [jpeg(np.full((120, 160), 128)) for _ in range(10)]
check('identical frames report no tear (no divide-by-zero)',
      first_tear(still) == -1, first_tear(still))

print('== the constant the node relies on ==')
check('MIN_COHERENT_FRAMES leaves room for motion', MIN_COHERENT_FRAMES >= 2)

print()
if failures:
    print(f'{len(failures)} FAILED: {failures}')
    sys.exit(1)
print('all checks passed')
