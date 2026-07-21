#!/usr/bin/env python3
"""Turn a nav2 plan into the camera-trajectory control signal LingBot-World reads.

LingBot-World-v2 (``causal_fast``) conditions generation on two arrays it loads
from ``action_path``:

  poses.npy       (T, 4, 4) float32  camera-to-world, OpenCV axes (x right, y down, z forward)
  intrinsics.npy  (T, 4)    float32  [fx, fy, cx, cy]

``wan/image2video.py`` hardcodes ``wasd_action = None`` on the causal_fast path,
so the camera trajectory is the *only* control signal. It also rescales the
intrinsics from a fixed 832x480 reference (``get_Ks_transformed(height_org=480,
width_org=832, ...)``), so they must be expressed at that size no matter what
the real camera resolution is. Only ``Ks[0]`` is read, then broadcast, so every
row has to be identical anyway.

The pipeline runs ``compute_relative_poses`` on the trajectory, rebasing it onto
frame 0 and normalizing translation. Absolute map coordinates and camera height
therefore drop out; only the frame-to-frame deltas and the mounting rotation
survive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

# Frame count is not free. wan/image2video.py:498-500 does
#
#     lat_f = (F - 1) // vae_stride + 1     # vae_stride == 4
#     lat_f = lat_f - (lat_f % chunk_size)  # floor to a whole chunk
#     F     = (lat_f - 1) * 4 + 1
#
# and then *silently* truncates c2ws to the resulting F. Ask for a frame count
# off that grid and the tail of the trajectory is dropped without a word: a
# 21-frame request becomes a 13-frame rollout covering only the first 60% of
# the plan. For a gate that is the worst possible failure -- it would judge a
# path whose far end it never rendered. So quantize onto the real grid:
#
#     lat_f in {chunk, 2*chunk, ...}  =>  F in {13, 29, 45, 61, 77, ...}
#
# for the deployed chunk_size of 4.
VAE_TEMPORAL_STRIDE = 4
DEFAULT_CHUNK_SIZE = 4

# LingBot rescales intrinsics from this reference resolution.
REFERENCE_WIDTH = 832
REFERENCE_HEIGHT = 480

# Optical axes (columns) expressed in the ROS body frame (REP-103: x forward,
# y left, z up):  x_opt = -y_body,  y_opt = -z_body,  z_opt = +x_body.
ROS_BODY_TO_OPTICAL = np.array(
    [[0.0, 0.0, 1.0],
     [-1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0]], dtype=np.float64)


class PoseConversionError(ValueError):
    pass


def level_for_conditioning(extrinsic: 'CameraExtrinsic') -> 'CameraExtrinsic':
    """Strip the mount's tilt before the pose trajectory is handed to LingBot.

    The model never sees an absolute pose: `compute_relative_poses` forces frame
    0 to identity, so a constant mounting tilt is *unobservable* to it. What it
    does see is the tilt's consequences, and both are ruinous:

    * A camera pitched 15 deg down, driving horizontally, translates along
      -y_opt by sin(15 deg): 25.9% of every step points "up" in the camera's own
      image axes. That is geometrically correct -- but the model, unable to know
      about the tilt, reads a persistent up-component as *ascending*. Measured
      on a 29-frame straight rollout: vertical optical flow of +3.12 px/frame
      (87 px cumulative, a fifth of the image) until the frame is pure ceiling.
      Levelling the camera drops it to +1.02 px/frame, which is just the honest
      parallax of driving forward.
    * Yawing about the world z-axis with a tilted camera conjugates the rotation
      into the camera frame, which puts *roll* on the optical axis: 6.24 deg
      cumulative over a 25 deg turn. Roll tilts the horizon and shatters the
      rollout.

    LingBot's own `examples/*/poses.npy` average a vertical step share of about
    zero, so a constant 25.9% is out of distribution. Keep the mount's
    translation and its yaw -- neither is pathological -- and drop roll/pitch.
    The conditioning image still shows the true tilted view; we simply stop
    telling the model that the robot is climbing.
    """
    return CameraExtrinsic(x=extrinsic.x, y=extrinsic.y, z=extrinsic.z,
                           roll=0.0, pitch=0.0, yaw=extrinsic.yaw)


@dataclass(frozen=True)
class CameraExtrinsic:
    """Rigid transform from base_link to the camera body frame."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def matrix(self) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rpy_to_rotation(self.roll, self.pitch, self.yaw)
        T[:3, 3] = (self.x, self.y, self.z)
        return T


def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise PoseConversionError('quaternion has zero norm')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_rotation(yaw: float) -> np.ndarray:
    return rpy_to_rotation(0.0, 0.0, yaw)


def _grid(lat_f: int) -> int:
    return (lat_f - 1) * VAE_TEMPORAL_STRIDE + 1


def quantize_frame_num(frame_num: int,
                       chunk_size: int = DEFAULT_CHUNK_SIZE) -> int:
    """Largest renderable frame count <= ``frame_num`` (never below one chunk).

    This is the *conservative* direction, for ceilings and for an explicitly
    requested frame count: it never renders more than the caller asked for.
    """
    if frame_num < 1:
        raise PoseConversionError('frame_num must be >= 1')
    if chunk_size < 1:
        raise PoseConversionError('chunk_size must be >= 1')
    lat_f = (frame_num - 1) // VAE_TEMPORAL_STRIDE + 1
    lat_f = max(lat_f - (lat_f % chunk_size), chunk_size)
    return _grid(lat_f)


def frame_num_at_least(frame_num: int,
                       chunk_size: int = DEFAULT_CHUNK_SIZE) -> int:
    """Smallest renderable frame count >= ``frame_num``.

    Used for the rotation budget, where rounding the wrong way is not merely
    imprecise but wrong: `frames_for_plan` computes how many frames are needed
    to keep the yaw under `max_deg_per_frame`, so dropping to the grid point
    *below* it silently raises the rate back above the limit. A 22.4 deg turn
    needs 23 frames; flooring to 13 makes it 1.87 deg/frame.
    """
    if frame_num < 1:
        raise PoseConversionError('frame_num must be >= 1')
    if chunk_size < 1:
        raise PoseConversionError('chunk_size must be >= 1')
    lat_f = (frame_num - 1) // VAE_TEMPORAL_STRIDE + 1
    if frame_num > _grid(lat_f):  # frame_num sat between two latent frames
        lat_f += 1
    remainder = lat_f % chunk_size
    if remainder:
        lat_f += chunk_size - remainder
    return _grid(max(lat_f, chunk_size))


# How fast the rollout may turn, per frame. This used to be 1.0, inferred from
# rollouts that broke -- but they were breaking because the camera's mount tilt
# was being fed to the model (see level_for_conditioning), and turning made the
# tilt's roll injection worse, so turns looked like the culprit.
#
# Re-measured with a level conditioning camera, 29 frames, arclength held at
# 5.26 m so the turn is the only variable. Every one of these is coherent, and
# every one looks like a correctly-rotated interior with a level horizon:
#
#   turn   deg/frame   radius    scene-cut ratio
#     14        0.50    21.5 m   1.12
#     28        1.00    10.8 m   1.57
#     56        2.00     5.4 m   1.20
#     90        3.21     3.4 m   1.38
#    120        4.29     2.5 m   1.36
#    150        5.36     2.0 m   1.49
#
# (The ratio is noisy -- 28 deg scores worse than 56 deg. Trust the frames.)
# 1.0 deg/frame implied a 10.8 m minimum turn radius, which would have refused
# every indoor corner. 4.0 sits below the highest verified rate with margin and
# admits a 90 deg corner. LingBot's own examples turn ~0.65 deg/frame, so this
# is well outside their demonstrated range: keep the check, keep the margin.
MAX_DEG_PER_FRAME = 4.0

# The rollout stops being trustworthy once the imagined robot walks further than
# the space in front of it. With the mount tilt stripped (level_for_conditioning)
# and a dead-straight plan through a MATRiX living room, scene-cut ratio =
# max frame-to-frame pixel change / median:
#
#   13 frames (~2.3 m)   1.12   coherent
#   29 frames (~5.3 m)   1.17   coherent
#   45 frames (~8.3 m)   3.11   breaks -- it walks past the kitchen island and
#                               invents a robot to fill the view
#
# This is a property of the *room*, not of the model: 45 frames breaks because
# 8.3 m is further than that house affords. A tighter space breaks sooner. Both
# constants are on the grid quantize_frame_num() enforces; the old value of 41
# was off-grid and had been rendering as 29 all along.
MIN_COHERENT_FRAMES = 13
MAX_COHERENT_FRAMES = 29

# Metres the imagined robot advances per frame. `normalize_trans` divides every
# step by the largest one, so a uniformly-resampled plan gives every step the
# value 1.0 and the rollout always advances one model-unit per frame, whatever
# the plan's real length. The frame count IS the distance.
#
# Measured against ground truth: seed a rollout from the first image of a 3.09 m
# straight stretch of a MATRiX bag, then find which recorded image best matches
# each rollout frame (normalised cross-correlation) and read the odometry.
# Rollout frame 28 lands 5.19 m along; least squares over the whole clip gives
# 0.188 m/frame. One bag, one segment, NCC on downsampled grey -- treat as
# +/- 20%, not a physical constant.
METRES_PER_FRAME = 0.188

# What the two limits mean in metres, for error messages.
MIN_COHERENT_METRES = (MIN_COHERENT_FRAMES - 1) * METRES_PER_FRAME   # ~2.3 m
MAX_COHERENT_METRES = (MAX_COHERENT_FRAMES - 1) * METRES_PER_FRAME   # ~5.3 m

# The two limits together cap how much a plan may turn before the world model
# can no longer imagine it: turn faster than 1 deg/frame and it tumbles, turn
# slower and it needs more frames than it can hold coherently.
MAX_COHERENT_TURN_DEG = (MAX_COHERENT_FRAMES - 1) * MAX_DEG_PER_FRAME


def plan_rotation_deg(
    positions: Sequence[Sequence[float]],
    final_yaw: Optional[float] = None,
) -> float:
    """Total heading change along the plan, in degrees.

    Sums each segment's turn rather than taking the net start-to-end angle: an
    S-bend that ends on its original heading still costs the model two turns.
    """
    points = np.asarray(positions, dtype=np.float64)
    if len(points) < 2:
        return 0.0
    yaws = tangent_yaws(points, final_yaw=final_yaw)
    return float(np.abs(np.diff(yaws)).sum() * 180.0 / math.pi)


def plan_length_m(positions: Sequence[Sequence[float]]) -> float:
    """Arclength of the plan, in metres."""
    points = np.asarray(positions, dtype=np.float64)
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


@dataclass(frozen=True)
class RolloutBudget:
    """What the world model will actually imagine, versus what was planned.

    Frame count is the *only* distance knob LingBot exposes, and it also sets
    the yaw rate. So the budget is not a free choice -- it is read off the plan
    and then checked. Two ways it can come back unusable, and the gate must say
    which:

    ``truncated``  the plan is longer than the ceiling can depict. The rollout
                   shows the first ``covered_m`` metres and nothing beyond, so
                   any obstacle further along was never imagined.
    ``too_curvy``  holding the yaw under ``max_deg_per_frame`` would take more
                   frames than the ceiling allows. Since frames are distance,
                   there is no way to buy them.
    """

    frame_num: int
    plan_m: float
    covered_m: float
    turn_deg: float
    deg_per_frame: float
    max_deg_per_frame: float

    @property
    def truncated(self) -> bool:
        return self.plan_m > self.covered_m + 0.05

    @property
    def too_curvy(self) -> bool:
        return self.deg_per_frame > self.max_deg_per_frame * 1.05

    @property
    def usable(self) -> bool:
        return not (self.truncated or self.too_curvy)

    @property
    def frames_to_goal(self) -> int:
        """How many rollout frames the plan itself accounts for.

        The renderable frame counts are sparse (13, 29, 45, ...) so a rollout
        almost never depicts exactly the plan's length -- 4.00 m lands between
        13 frames (2.26 m) and 29 (5.26 m), and we round up, because seeing
        further than you will drive is the safe error. But the overshoot is not
        evidence about *this* plan: the imagined robot carries on 1.26 m past
        the goal, and an obstacle it meets there must not veto the task.

        The critic gets frames up to this index. Everything after it is the
        model imagining a journey nobody asked for.
        """
        if self.covered_m <= 0.0:
            return self.frame_num
        span = self.covered_m / max(self.frame_num - 1, 1)
        reached = int(round(self.plan_m / span)) + 1
        return max(2, min(self.frame_num, reached))


def budget_for_plan(
    positions: Sequence[Sequence[float]],
    final_yaw: Optional[float] = None,
    max_deg_per_frame: float = MAX_DEG_PER_FRAME,
    min_frames: int = MIN_COHERENT_FRAMES,
    max_frames: int = MAX_COHERENT_FRAMES,
    metres_per_frame: float = METRES_PER_FRAME,
) -> RolloutBudget:
    """Size the rollout from the plan's *distance*, then check its curvature.

    Distance first, because that is what the frame count actually buys: the
    rollout advances ``metres_per_frame`` per frame no matter how long the plan
    is, so sizing from the turn (as this used to) would imagine 2.3 m of a 20 m
    corridor and cheerfully approve it.

    The turn is then a *check*, not an input: frames cannot be added to slow a
    turn without also walking further, so there is no budget to spend. At
    4.0 deg/frame a 29-frame rollout admits up to ~112 deg of turning, which
    covers an indoor corner.
    """
    if max_deg_per_frame <= 0.0:
        raise PoseConversionError('max_deg_per_frame must be > 0')
    if metres_per_frame <= 0.0:
        raise PoseConversionError('metres_per_frame must be > 0')

    length = plan_length_m(positions)
    turn = plan_rotation_deg(positions, final_yaw=final_yaw)

    # Round the requirement up onto the grid, then clip by a ceiling rounded
    # down. Both bounds are snapped so the arithmetic below is not a fiction.
    ceiling = quantize_frame_num(max_frames)
    floor = quantize_frame_num(min_frames)
    wanted = frame_num_at_least(int(math.ceil(length / metres_per_frame)) + 1)
    frame_num = max(floor, min(ceiling, wanted))

    covered = (frame_num - 1) * metres_per_frame
    return RolloutBudget(
        frame_num=frame_num,
        plan_m=length,
        covered_m=covered,
        turn_deg=turn,
        deg_per_frame=turn / max(frame_num - 1, 1),
        max_deg_per_frame=max_deg_per_frame,
    )


def frames_for_plan(
    positions: Sequence[Sequence[float]],
    final_yaw: Optional[float] = None,
    max_deg_per_frame: float = MAX_DEG_PER_FRAME,
    min_frames: int = MIN_COHERENT_FRAMES,
    max_frames: int = MAX_COHERENT_FRAMES,
) -> int:
    """Frame count only. Prefer ``budget_for_plan``, which also reports coverage."""
    return budget_for_plan(positions, final_yaw=final_yaw,
                           max_deg_per_frame=max_deg_per_frame,
                           min_frames=min_frames, max_frames=max_frames).frame_num


def resample_by_arclength(points: np.ndarray, count: int) -> Tuple[np.ndarray, np.ndarray]:
    """Sample ``count`` points spaced evenly along the polyline ``points``.

    Returns the resampled positions and the fractional source indices they came
    from, so callers can interpolate other per-pose quantities the same way.
    A nav2 plan is spaced by costmap resolution rather than by time, so walking
    it at constant arclength is what makes the imagined rollout move at a
    constant speed.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise PoseConversionError(f'expected (N, 3) points, got {points.shape}')
    n = len(points)
    if n == 0:
        raise PoseConversionError('cannot resample an empty path')
    if n == 1:
        return np.repeat(points, count, axis=0), np.zeros(count)

    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    total = cumulative[-1]
    if total < 1e-9:
        # Degenerate plan: the robot is already at the goal.
        return np.repeat(points[:1], count, axis=0), np.zeros(count)

    targets = np.linspace(0.0, total, count)
    src_index = np.interp(targets, cumulative, np.arange(n, dtype=np.float64))
    resampled = np.stack(
        [np.interp(targets, cumulative, points[:, axis]) for axis in range(3)],
        axis=1)
    return resampled, src_index


def tangent_yaws(points: np.ndarray, final_yaw: Optional[float] = None) -> np.ndarray:
    """Heading at each point, taken from the direction of travel.

    nav2 planners do not all populate intermediate pose orientations -- NavFn
    leaves them at identity -- so the tangent is the only heading we can trust
    mid-path. ``final_yaw`` overrides the last sample when the goal orientation
    is meaningful.
    """
    n = len(points)
    if n < 2:
        return np.zeros(n)
    deltas = np.diff(points[:, :2], axis=0)
    yaws = np.arctan2(deltas[:, 1], deltas[:, 0])
    # Each point inherits the heading of the segment leaving it; the last point
    # keeps the heading of the segment arriving at it.
    yaws = np.concatenate([yaws, yaws[-1:]])

    # A zero-length segment yields a meaningless atan2(0, 0) == 0; carry the
    # previous heading through instead of snapping to +x.
    stationary = np.concatenate([np.linalg.norm(deltas, axis=1) < 1e-9, [False]])
    for i in range(1, n):
        if stationary[i - 1]:
            yaws[i - 1] = yaws[i - 2] if i >= 2 else yaws[i]
    if final_yaw is not None:
        yaws[-1] = final_yaw
    return unwrap_angles(yaws)


def unwrap_angles(angles: np.ndarray) -> np.ndarray:
    return np.unwrap(angles)


def slew_limit_yaws(
    yaws: np.ndarray,
    max_step_rad: float,
    max_iters: int = 20000,
) -> np.ndarray:
    """Spread the turning out until no single frame turns more than ``max_step_rad``.

    Budgeting the *total* rotation is not enough. A plan given as a handful of
    waypoints has all of its turn concentrated at the polyline corners, and
    overriding the last sample with the goal heading concentrates another jump
    at the end: a 90 deg plan sampled to 89 frames averaged 1.03 deg/frame but
    peaked at 29.6 deg, and the rollout tumbled at exactly that frame.

    Endpoints are pinned -- the rollout must still start at the robot's current
    heading and end at the goal's -- and the interior is diffused. This is what
    the robot does anyway: it cannot rotate instantaneously.
    """
    if len(yaws) < 3 or max_step_rad <= 0.0:
        return yaws
    smoothed = np.array(yaws, dtype=np.float64)
    first, last = smoothed[0], smoothed[-1]
    for _ in range(max_iters):
        if np.abs(np.diff(smoothed)).max() <= max_step_rad:
            break
        padded = np.concatenate([smoothed[:1], smoothed, smoothed[-1:]])
        smoothed = 0.25 * padded[:-2] + 0.5 * padded[1:-1] + 0.25 * padded[2:]
        smoothed[0], smoothed[-1] = first, last
    return smoothed


def base_poses_to_camera_trajectory(
    positions: np.ndarray,
    yaws: np.ndarray,
    extrinsic: CameraExtrinsic,
) -> np.ndarray:
    """Compose base_link poses with the camera mount into OpenCV c2w matrices."""
    if len(positions) != len(yaws):
        raise PoseConversionError('positions and yaws must have equal length')

    base_to_cam = extrinsic.matrix()
    body_to_optical = np.eye(4, dtype=np.float64)
    body_to_optical[:3, :3] = ROS_BODY_TO_OPTICAL

    c2ws = np.empty((len(positions), 4, 4), dtype=np.float64)
    for i, (pos, yaw) in enumerate(zip(positions, yaws)):
        map_to_base = np.eye(4, dtype=np.float64)
        map_to_base[:3, :3] = yaw_to_rotation(float(yaw))
        map_to_base[:3, 3] = pos
        c2ws[i] = map_to_base @ base_to_cam @ body_to_optical
    return c2ws.astype(np.float32)


def reference_intrinsics(
    horizontal_fov_deg: float,
    source_width: int,
    source_height: int,
) -> Tuple[float, float, float, float]:
    """Intrinsics of the source camera, expressed at LingBot's 832x480 reference.

    The source frame is center-cropped to the reference aspect ratio and then
    scaled, which is what an aspect-preserving resize of the image itself does.
    Cropping narrows the field of view, so fx ends up larger than the naive
    ``(832/2) / tan(fov/2)``.
    """
    if source_width <= 0 or source_height <= 0:
        raise PoseConversionError('source resolution must be positive')
    if not 0.0 < horizontal_fov_deg < 180.0:
        raise PoseConversionError('horizontal_fov_deg must be in (0, 180)')

    fx = (source_width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)
    fy = fx  # square pixels
    cx = source_width / 2.0
    cy = source_height / 2.0

    reference_aspect = REFERENCE_WIDTH / REFERENCE_HEIGHT
    source_aspect = source_width / source_height
    if source_aspect > reference_aspect:
        cropped_width = source_height * reference_aspect
        cx -= (source_width - cropped_width) / 2.0
        scale = REFERENCE_WIDTH / cropped_width
    else:
        cropped_height = source_width / reference_aspect
        cy -= (source_height - cropped_height) / 2.0
        scale = REFERENCE_HEIGHT / cropped_height

    return fx * scale, fy * scale, cx * scale, cy * scale


def plan_to_control_signal(
    path_positions: Sequence[Sequence[float]],
    frame_num: int,
    extrinsic: CameraExtrinsic,
    horizontal_fov_deg: float,
    source_width: int,
    source_height: int,
    final_yaw: Optional[float] = None,
    max_deg_per_frame: float = MAX_DEG_PER_FRAME,
    level_camera: bool = True,
) -> Tuple[np.ndarray, np.ndarray]:
    """Full nav2-plan -> (poses, intrinsics) conversion.

    Returns ``(T, 4, 4)`` float32 camera-to-world matrices and the matching
    ``(T, 4)`` float32 intrinsics.

    ``level_camera`` drops the mount's tilt from the conditioning trajectory --
    see ``level_for_conditioning``. Pass False only to reproduce the drift.
    """
    points = np.asarray(path_positions, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise PoseConversionError(
            f'expected path of (N, 3) positions, got {points.shape}')

    if level_camera:
        extrinsic = level_for_conditioning(extrinsic)
    frames = quantize_frame_num(frame_num)
    resampled, _ = resample_by_arclength(points, frames)
    yaws = tangent_yaws(resampled, final_yaw=final_yaw)
    yaws = slew_limit_yaws(yaws, math.radians(max_deg_per_frame))
    c2ws = base_poses_to_camera_trajectory(resampled, yaws, extrinsic)

    fx, fy, cx, cy = reference_intrinsics(
        horizontal_fov_deg, source_width, source_height)
    intrinsics = np.tile(
        np.array([fx, fy, cx, cy], dtype=np.float32), (frames, 1))
    return c2ws, intrinsics
