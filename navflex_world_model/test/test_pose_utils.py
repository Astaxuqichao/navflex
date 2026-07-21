#!/usr/bin/env python3
"""Offline checks for the nav2-plan -> LingBot control-signal conversion.

Run with the real LingBot code on sys.path so the trajectory is validated
through the same ``compute_relative_poses`` the model applies at inference:

    LINGBOT_REPO=/home/jiangbz/project/world_model/lingbot-world-v2 \
        python3 test/test_pose_utils.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from navflex_wm.pose_utils import (  # noqa: E402
    MAX_COHERENT_FRAMES,
    MIN_COHERENT_FRAMES,
    CameraExtrinsic,
    budget_for_plan,
    frame_num_at_least,
    frames_for_plan,
    level_for_conditioning,
    plan_length_m,
    plan_rotation_deg,
    plan_to_control_signal,
    quantize_frame_num,
    reference_intrinsics,
    resample_by_arclength,
    tangent_yaws,
)

# The xgb camera as configured in MATRiX config/config.json.
MATRIX_CAMERA = CameraExtrinsic(x=0.29, y=0.0, z=0.01, pitch=math.radians(15.0))
MATRIX_FOV = 90.0
MATRIX_W, MATRIX_H = 1920, 1080

failures = []


def check(name, condition, detail=''):
    if condition:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        failures.append(name)


print('== frame quantization (wan/image2video.py:498-500) ==')
# lat_f = (F-1)//4 + 1, floored to a multiple of chunk_size, then F = (lat_f-1)*4+1.
# For chunk_size 4 the only renderable counts are 16k-3. Anything else is
# floored AND the pose trajectory is truncated to match, without a warning.
GRID = [13, 29, 45, 61, 77]
for f in GRID:
    check(f'{f} is on the grid and survives', quantize_frame_num(f) == f,
          quantize_frame_num(f))
check('21 -> 13 (the bug that shipped: 21 rendered as 13)',
      quantize_frame_num(21) == 13, quantize_frame_num(21))
check('41 -> 29 (the old "max_frame_num" was really 29)',
      quantize_frame_num(41) == 29, quantize_frame_num(41))
check('81 -> 77', quantize_frame_num(81) == 77, quantize_frame_num(81))
check('floor never rounds up',
      all(quantize_frame_num(f) <= f for f in range(13, 200)))
check('floor bottoms out at one chunk, not zero',
      quantize_frame_num(1) == 13, quantize_frame_num(1))
check('quantized values are fixed points',
      all(quantize_frame_num(quantize_frame_num(f)) == quantize_frame_num(f)
          for f in range(1, 200)))

# The rotation budget must round the other way: fewer frames means a faster yaw.
check('ceil never rounds down',
      all(frame_num_at_least(f) >= f for f in range(1, 78)))
check('ceil lands on the grid',
      all(frame_num_at_least(f) in GRID for f in range(1, 78)))
check('23 -> 29 (a 22 deg turn must not be crushed into 13 frames)',
      frame_num_at_least(23) == 29, frame_num_at_least(23))
for f in GRID:
    check(f'ceil({f}) is a fixed point', frame_num_at_least(f) == f,
          frame_num_at_least(f))
check('ceil is tight (one below a grid point lands on it)',
      all(frame_num_at_least(f - 1) == f for f in GRID))


def _renders_in_full(frame_num, chunk_size=4):
    """Replay wan/image2video.py's own arithmetic on the quantized value."""
    lat_f = (frame_num - 1) // 4 + 1
    lat_f -= lat_f % chunk_size
    return (lat_f - 1) * 4 + 1 == frame_num


check('every quantized count renders in full (no silent truncation)',
      all(_renders_in_full(quantize_frame_num(f)) for f in range(1, 200)))

print('== arclength resampling ==')
line = np.array([[0, 0, 0], [1, 0, 0], [4, 0, 0]], dtype=float)
pts, _ = resample_by_arclength(line, 5)
check('endpoints preserved', np.allclose(pts[0], [0, 0, 0]) and np.allclose(pts[-1], [4, 0, 0]))
spacing = np.linalg.norm(np.diff(pts, axis=0), axis=1)
check('uniform spacing', np.allclose(spacing, spacing[0]), spacing)
degenerate, _ = resample_by_arclength(np.zeros((3, 3)), 5)
check('degenerate plan does not divide by zero', degenerate.shape == (5, 3))

print('== tangent headings ==')
check('straight +x path -> yaw 0', np.allclose(tangent_yaws(line), 0.0))
turn = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]], dtype=float)
check('left turn ends at +pi/2', math.isclose(tangent_yaws(turn)[-1], math.pi / 2, abs_tol=1e-6))
check('final_yaw override wins',
      math.isclose(tangent_yaws(line, final_yaw=1.23)[-1], 1.23, abs_tol=1e-9))

print('== frame budget is paid for by rotation, not distance ==')
# Measured: LingBot's own examples/03/poses.npy turns 0.65 deg/frame, and a
# 90 deg turn at 4.3 deg/frame produced a rollout that cut to a different scene.
straight_10m = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
turn_90 = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.5, 1.0, 0.0], [4.0, 2.0, 0.0]]
check('straight plan needs no rotation', plan_rotation_deg(straight_10m) < 1e-6)
check('90 deg turn measured as ~90 deg',
      math.isclose(plan_rotation_deg(turn_90, final_yaw=math.pi / 2), 90.0, abs_tol=1.0),
      plan_rotation_deg(turn_90, final_yaw=math.pi / 2))
# An S-bend ends on its original heading but still costs the model two turns.
s_bend = [[0, 0, 0], [1, 0, 0], [2, 1, 0], [3, 0, 0], [4, 0, 0]]
check('S-bend costs more than its net heading change',
      plan_rotation_deg(s_bend) > 100.0, plan_rotation_deg(s_bend))

print('== the frame budget is bought with DISTANCE, and the turn is a check ==')
# 10 m is far beyond the ~5.3 m the ceiling can depict. The old code sized the
# rollout from the turn, so this straight plan got the 13-frame floor: it
# imagined 2.3 m of a 10 m corridor and would have approved on that.
long_straight = budget_for_plan(straight_10m)
check('a 10 m plan saturates the ceiling', long_straight.frame_num == MAX_COHERENT_FRAMES,
      long_straight.frame_num)
check('...and reports itself truncated', long_straight.truncated)
check('...and is therefore unusable', not long_straight.usable)
check('...covering only ~5.3 m',
      math.isclose(long_straight.covered_m, 5.264, abs_tol=0.01), long_straight.covered_m)

short = budget_for_plan([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
check('a 1 m plan sits at the frame floor', short.frame_num == MIN_COHERENT_FRAMES,
      short.frame_num)
check('a 1 m plan is covered in full', not short.truncated)
check('a 1 m plan is usable', short.usable)

fitting = budget_for_plan([[x * 0.05, 0.0, 0.0] for x in range(81)])   # 4.0 m
check('a 4 m plan gets enough frames to cover it',
      fitting.covered_m >= fitting.plan_m - 0.05,
      (fitting.frame_num, fitting.covered_m, fitting.plan_m))
check('a 4 m plan is not truncated', not fitting.truncated)
check('budget frame counts are on the grid',
      all(quantize_frame_num(b.frame_num) == b.frame_num
          for b in (short, fitting, long_straight)))

# Turning: frames cannot be added to slow it, because frames are distance. The
# only question is the per-frame rate. Verified coherent up to 5.36 deg/frame
# with a level camera; the limit is 4.0, so a 90 deg corner (3.21 deg/frame in
# a 29-frame rollout) is allowed -- it was measured, and it renders cleanly.
corner = budget_for_plan(turn_90, final_yaw=math.pi / 2)
check('a 90 deg corner is within the turn budget', not corner.too_curvy,
      corner.deg_per_frame)
check('...at 3.21 deg/frame, the rate that was measured coherent',
      math.isclose(corner.deg_per_frame, 3.214, abs_tol=1e-3), corner.deg_per_frame)

# Past the verified range the gate must still refuse: a 135 deg turn in a
# 29-frame rollout is 4.8 deg/frame, beyond anything that was rendered.
hairpin = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 1.5, 0.0], [2.2, 2.6, 0.0]]
sharp = budget_for_plan(hairpin, final_yaw=math.radians(135.0))
check('a hairpin exceeds even the relaxed limit', sharp.too_curvy,
      sharp.deg_per_frame)
check('...and is unusable', not sharp.usable)

gentle_plan = [[math.sin(t) * 12, (1 - math.cos(t)) * 12, 0.0]
               for t in np.linspace(0, math.radians(20.0), 120)]
gentle = budget_for_plan(gentle_plan)
check('a 12 m-radius arc is within the turn budget', not gentle.too_curvy,
      gentle.deg_per_frame)
check('...covers its own 4.2 m length', not gentle.truncated,
      (gentle.plan_m, gentle.covered_m))
check('a gentle arc is usable', gentle.usable,
      (gentle.frame_num, gentle.plan_m, gentle.covered_m, gentle.deg_per_frame))

# The grid is sparse, so a rollout almost never depicts exactly the plan. It
# rounds up (seeing further than you drive is the safe error) and the critic is
# then shown only the frames the plan accounts for.
check('a 4 m plan overshoots to 5.26 m', fitting.covered_m > fitting.plan_m,
      (fitting.covered_m, fitting.plan_m))
check('...so the critic sees only the frames up to the goal',
      fitting.frames_to_goal < fitting.frame_num,
      (fitting.frames_to_goal, fitting.frame_num))
check('...which is the plan\'s share of the rollout',
      fitting.frames_to_goal == 22, fitting.frames_to_goal)
check('a truncated plan shows every frame it has',
      long_straight.frames_to_goal == long_straight.frame_num,
      long_straight.frames_to_goal)
check('frames_to_goal never exceeds the rollout',
      all(b.frames_to_goal <= b.frame_num
          for b in (short, fitting, long_straight, gentle)))
check('frames_to_goal keeps at least two frames (motion needs a pair)',
      budget_for_plan([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]]).frames_to_goal >= 2)

check('plan_length_m measures arclength, not displacement',
      math.isclose(plan_length_m(s_bend), 2 + 2 * math.sqrt(2), abs_tol=1e-9),
      plan_length_m(s_bend))

check('frames_for_plan still returns just the count',
      frames_for_plan(straight_10m) == long_straight.frame_num)
check('min_frames raises the floor',
      budget_for_plan(short_plan := [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]],
                      min_frames=29, max_frames=45).frame_num == 29)
check('an off-grid floor snaps down, never up',
      budget_for_plan(short_plan, min_frames=33, max_frames=45).frame_num == 29,
      budget_for_plan(short_plan, min_frames=33, max_frames=45).frame_num)

print('== the turn must be spread out, not just budgeted ==')


def per_frame_rotation_deg(c2ws_):
    out = []
    for i in range(len(c2ws_) - 1):
        R_ = c2ws_[i, :3, :3].T @ c2ws_[i + 1, :3, :3]
        out.append(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_) - 1) / 2)))))
    return np.array(out)


# The worst case is ordinary: nav2 drives straight to the goal, and the goal's
# orientation differs from the direction of travel. Without a slew limit the
# whole difference lands on the final frame. 25 deg is inside the 28 deg the
# 29-frame ceiling can pay for; a 90 deg goal is refused upstream by the node.
GOAL_TURN = math.radians(25.0)
dense_straight = [[i * 0.05, 0.0, 0.0] for i in range(101)]
n_frames = frames_for_plan(dense_straight, final_yaw=GOAL_TURN)
check('a 25 deg goal turn fits inside the ceiling',
      n_frames <= MAX_COHERENT_FRAMES, n_frames)
unlimited, _ = plan_to_control_signal(
    dense_straight, n_frames, MATRIX_CAMERA, MATRIX_FOV, MATRIX_W, MATRIX_H,
    final_yaw=GOAL_TURN, max_deg_per_frame=1e9)
limited, _ = plan_to_control_signal(
    dense_straight, n_frames, MATRIX_CAMERA, MATRIX_FOV, MATRIX_W, MATRIX_H,
    final_yaw=GOAL_TURN, max_deg_per_frame=1.0)
check('unlimited dumps the whole goal turn into one frame',
      per_frame_rotation_deg(unlimited).max() > 24.0,
      per_frame_rotation_deg(unlimited).max())
check('slew limit holds every frame under ~1 deg',
      per_frame_rotation_deg(limited).max() <= 1.05,
      per_frame_rotation_deg(limited).max())

# A sparse polyline concentrates its turn at the corners instead.
sparse_gentle = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.8, 0.84, 0.0]]
sparse_frames = frames_for_plan(sparse_gentle)
sparse_raw, _ = plan_to_control_signal(
    sparse_gentle, sparse_frames, MATRIX_CAMERA, MATRIX_FOV, MATRIX_W, MATRIX_H,
    max_deg_per_frame=1e9)
check('a sparse corner concentrates its turn',
      per_frame_rotation_deg(sparse_raw).max() > 20.0,
      per_frame_rotation_deg(sparse_raw).max())
sparse_turn, _ = plan_to_control_signal(
    sparse_gentle, sparse_frames, MATRIX_CAMERA, MATRIX_FOV, MATRIX_W, MATRIX_H,
    max_deg_per_frame=1.0)
check('polyline corners are diffused too',
      per_frame_rotation_deg(sparse_turn).max() <= 1.05,
      per_frame_rotation_deg(sparse_turn).max())

check('endpoints are pinned: start heading preserved',
      np.allclose(limited[0, :3, :3], unlimited[0, :3, :3], atol=1e-6))
check('endpoints are pinned: goal heading preserved',
      np.allclose(limited[-1, :3, :3], unlimited[-1, :3, :3], atol=1e-4))

# A yaw-only plan must not induce roll: with a pitched-down camera, a body-frame
# roll would tilt the horizon and the rollout falls apart.
roll = [math.degrees(math.asin(max(-1.0, min(1.0, -limited[i, :3, 0][2]))))
        for i in range(len(limited))]
check('slew-limited turn introduces no roll', max(abs(r) for r in roll) < 1e-3,
      max(abs(r) for r in roll))

# A real curved nav2 path needs no limiting at all: it already turns smoothly.
# The limiter exists for sparse plans and for the goal-heading step. Radius 12 m
# so the arc is inside the curvature budget; a 5 m radius is refused upstream.
arc = [[12 * math.sin(t), 12 * (1 - math.cos(t)), 0.0]
       for t in np.linspace(0, GOAL_TURN, 120)]
check('a smooth arc is left essentially alone',
      per_frame_rotation_deg(plan_to_control_signal(
          arc, frames_for_plan(arc), MATRIX_CAMERA, MATRIX_FOV, MATRIX_W,
          MATRIX_H, max_deg_per_frame=1.0)[0]).max() <= 1.05,
      per_frame_rotation_deg(plan_to_control_signal(
          arc, frames_for_plan(arc), MATRIX_CAMERA, MATRIX_FOV, MATRIX_W,
          MATRIX_H, max_deg_per_frame=1.0)[0]).max())

print('== intrinsics at the 832x480 reference ==')
fx, fy, cx, cy = reference_intrinsics(MATRIX_FOV, MATRIX_W, MATRIX_H)
# 1920x1080 @ 90 deg hfov -> fx=960. Center-crop to 832:480 keeps 1872 px of
# width, then scales by 832/1872, so fx = 960 * 832/1872 = 426.67.
check('fx == 426.67', math.isclose(fx, 426.666, abs_tol=1e-2), fx)
check('principal point is the reference center',
      math.isclose(cx, 416.0, abs_tol=1e-3) and math.isclose(cy, 240.0, abs_tol=1e-3), (cx, cy))
check('square pixels', math.isclose(fx, fy))

print('== control signal shape/dtype matches examples/*/ ==')
straight = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
# 81 is off-grid: it must come back as 77 poses, the count LingBot renders.
# One pose per rendered frame, or the trajectory's tail is dropped in silence.
c2ws, K = plan_to_control_signal(
    straight, 81, MATRIX_CAMERA, MATRIX_FOV, MATRIX_W, MATRIX_H)
check('an off-grid 81 yields 77 poses', c2ws.shape == (77, 4, 4) and c2ws.dtype == np.float32, (c2ws.shape, c2ws.dtype))
check('intrinsics (77,4) float32', K.shape == (77, 4) and K.dtype == np.float32, (K.shape, K.dtype))
check('homogeneous bottom row', np.allclose(c2ws[:, 3, :], [0, 0, 0, 1]))
check('rotations are orthonormal',
      np.allclose(c2ws[:, :3, :3] @ c2ws[:, :3, :3].transpose(0, 2, 1),
                  np.eye(3)[None], atol=1e-5))
check('identical intrinsic rows (only Ks[0] is read)',
      len(np.unique(K, axis=0)) == 1)

print('== OpenCV axis convention ==')
# Camera at yaw 0, no pitch: optical +z must point along map +x (robot forward),
# optical +y must point along map -z (down).
flat = CameraExtrinsic()
c2ws_flat, _ = plan_to_control_signal(
    straight, 5, flat, MATRIX_FOV, MATRIX_W, MATRIX_H)
R = c2ws_flat[0, :3, :3]
check('optical +z == map +x (forward)', np.allclose(R[:, 2], [1, 0, 0], atol=1e-6), R[:, 2])
check('optical +y == map -z (down)', np.allclose(R[:, 1], [0, 0, -1], atol=1e-6), R[:, 1])
check('optical +x == map -y (right)', np.allclose(R[:, 0], [0, -1, 0], atol=1e-6), R[:, 0])

print('== pitch sign: +15 deg must look downward ==')
# level_camera=False: this checks the extrinsic's own geometry, not the
# trajectory that gets conditioned on.
pitched = CameraExtrinsic(pitch=math.radians(15.0))
c2ws_p, _ = plan_to_control_signal(straight, 5, pitched, MATRIX_FOV, MATRIX_W,
                                   MATRIX_H, level_camera=False)
forward_axis = c2ws_p[0, :3, 2]
check('optical +z has negative map z (tilted down)', forward_axis[2] < -0.2, forward_axis)
check('tilt magnitude is 15 deg',
      math.isclose(math.degrees(math.asin(-forward_axis[2])), 15.0, abs_tol=1e-3),
      math.degrees(math.asin(-forward_axis[2])))

print('== the tilt must not reach the model ==')
check('levelling keeps the mount offset',
      (level_for_conditioning(MATRIX_CAMERA).x,
       level_for_conditioning(MATRIX_CAMERA).z) == (MATRIX_CAMERA.x, MATRIX_CAMERA.z))
check('levelling keeps the mount yaw',
      level_for_conditioning(CameraExtrinsic(yaw=0.5)).yaw == 0.5)
check('levelling drops roll and pitch',
      (level_for_conditioning(MATRIX_CAMERA).pitch,
       level_for_conditioning(MATRIX_CAMERA).roll) == (0.0, 0.0))

# ---------------------------------------------------------------------------
# Validate through LingBot's own relative-pose transform, which is what the
# DiT actually sees. A straight plan must produce pure forward translation in
# the camera frame and no rotation.
# ---------------------------------------------------------------------------
repo = os.environ.get('LINGBOT_REPO', '/home/jiangbz/project/world_model/lingbot-world-v2')
if os.path.isdir(repo):
    sys.path.insert(0, repo)
    try:
        import importlib.util

        import torch

        # Load cam_utils by path: importing it as `wan.utils.cam_utils` would run
        # wan/__init__.py, which drags in the whole inference stack (easydict,
        # flash_attn, ...) that this offline check does not need.
        _spec = importlib.util.spec_from_file_location(
            'lingbot_cam_utils', os.path.join(repo, 'wan', 'utils', 'cam_utils.py'))
        _cam_utils = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_cam_utils)
        compute_relative_poses = _cam_utils.compute_relative_poses

        print('== round-trip through wan.utils.cam_utils.compute_relative_poses ==')
        rel = compute_relative_poses(torch.from_numpy(c2ws.astype(np.float64)), framewise=True)
        rel = rel.numpy()
        check('frame 0 is identity', np.allclose(rel[0], np.eye(4), atol=1e-5))

        c2ws_straight, _ = plan_to_control_signal(
            straight, 81, flat, MATRIX_FOV, MATRIX_W, MATRIX_H)
        rel_s = compute_relative_poses(
            torch.from_numpy(c2ws_straight.astype(np.float64)), framewise=False).numpy()
        trans = rel_s[:, :3, 3]
        # Straight-ahead driving: motion lives on the optical z axis only.
        check('straight plan -> no lateral drift in camera frame',
              np.abs(trans[:, 0]).max() < 1e-4 and np.abs(trans[:, 1]).max() < 1e-4,
              (np.abs(trans[:, 0]).max(), np.abs(trans[:, 1]).max()))
        check('straight plan -> monotonic forward translation',
              np.all(np.diff(trans[:, 2]) > -1e-6) and trans[-1, 2] > 0, trans[-1])
        check('straight plan -> no rotation',
              np.allclose(rel_s[:, :3, :3], np.eye(3)[None], atol=1e-5))

        left = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]]
        c2ws_left, _ = plan_to_control_signal(
            left, 81, flat, MATRIX_FOV, MATRIX_W, MATRIX_H)
        rel_l = compute_relative_poses(
            torch.from_numpy(c2ws_left.astype(np.float64)), framewise=False).numpy()
        # Turning left in the map rotates the camera about its optical -y axis.
        final_yaw_cam = math.atan2(rel_l[-1, 0, 2], rel_l[-1, 2, 2])
        check('left turn -> ~ -90 deg about optical y',
              math.isclose(math.degrees(final_yaw_cam), -90.0, abs_tol=1.0),
              math.degrees(final_yaw_cam))

        # The checks above all use a `flat` camera, which is how the real
        # camera's 15 deg tilt slipped through for so long. Run the two
        # quantities the model punishes against the ACTUAL MATRiX mount.
        print('== the tilted MATRiX mount, as the DiT sees it ==')

        def conditioned(plan, **kw):
            c, _ = plan_to_control_signal(
                plan, 29, MATRIX_CAMERA, MATRIX_FOV, MATRIX_W, MATRIX_H, **kw)
            return compute_relative_poses(torch.from_numpy(c.astype(np.float64)),
                                          framewise=True, normalize_trans=True).numpy()

        def upward_share(rel_):
            steps = rel_[1:, :3, 3]
            return float(np.mean(-steps[:, 1] / np.linalg.norm(steps, axis=1)))

        def total_roll_deg(rel_):
            return float(sum(math.degrees(math.atan2(R[1, 0], R[0, 0]))
                             for R in rel_[1:, :3, :3]))

        dense = [[i * 0.04, 0.0, 0.0] for i in range(101)]
        tilted_straight = conditioned(dense, level_camera=False)
        level_straight = conditioned(dense)
        check('tilt injects a 25.9% upward component (the ceiling drift)',
              math.isclose(upward_share(tilted_straight), math.sin(math.radians(15.0)),
                           abs_tol=1e-3), upward_share(tilted_straight))
        check('levelled straight plan has NO vertical component',
              abs(upward_share(level_straight)) < 1e-4, upward_share(level_straight))

        curve = [[math.sin(t) * 5, (1 - math.cos(t)) * 5, 0.0]
                 for t in np.linspace(0, math.radians(25.0), 120)]
        tilted_turn = conditioned(curve, level_camera=False)
        level_turn = conditioned(curve)
        check('tilt injects roll on a turn (shatters the rollout)',
              abs(total_roll_deg(tilted_turn)) > 5.0, total_roll_deg(tilted_turn))
        check('levelled turn introduces NO roll about the optical axis',
              abs(total_roll_deg(level_turn)) < 1e-3, total_roll_deg(level_turn))

        # Scale is normalised away: frame count, not metres, is the distance.
        short = [[i * 0.01, 0.0, 0.0] for i in range(101)]
        check('a 1 m and a 4 m plan are the same input to the model',
              np.allclose(conditioned(short), conditioned(dense), atol=1e-5))
    except ImportError as exc:
        print(f'  SKIP  LingBot round-trip ({exc})')
else:
    print(f'  SKIP  LingBot round-trip (repo not found at {repo})')

print()
if failures:
    print(f'{len(failures)} FAILED: {failures}')
    sys.exit(1)
print('all checks passed')
