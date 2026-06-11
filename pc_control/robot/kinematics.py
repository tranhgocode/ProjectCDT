import math
import numpy as np


BASE_HEIGHT = 1.27
L1 = 1.2
L2 = 1.2
L3 = 1.0
LINK_LENGTHS = (L1, L2, L3)

GROUND_LEVEL = 0.0
TOUCH_TOLERANCE = 0.05
GROUND_TARGET_MAX_RADIUS = math.sqrt(max(0.0, (L1 + L2) ** 2 - (L3 - BASE_HEIGHT) ** 2))


def _wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _joint_positions_from_radians(angles):
    """
    Return [base, elbow, wrist, end_effector] using the same convention as
    robot_builder.apply_joint_rotations:
      - angles are radians
      - yaw rotates around Y
      - pitch 0 means each link points horizontally along +Z
      - positive pitch raises the link
    """
    yaw, shoulder, elbow, wrist = np.asarray(angles, dtype=float)

    base_pos = np.array([0.0, BASE_HEIGHT, 0.0], dtype=float)
    positions = [base_pos]
    pos = base_pos.copy()
    total_pitch = 0.0

    for length, joint_angle in zip(LINK_LENGTHS, (shoulder, elbow, wrist)):
        total_pitch += joint_angle
        horizontal = length * math.cos(total_pitch)
        pos = pos + np.array(
            [
                horizontal * math.sin(yaw),
                length * math.sin(total_pitch),
                horizontal * math.cos(yaw),
            ],
            dtype=float,
        )
        positions.append(pos)

    return positions


def forward_kinematics_direct(angles, degrees=True):
    """
    Direct FK based on the physical render geometry.

    By default angles are accepted in degrees for compatibility with the
    older iterative IK function. Pass degrees=False for radians.
    """
    q = np.radians(angles) if degrees else np.asarray(angles, dtype=float)
    positions = _joint_positions_from_radians(q)
    return positions[-1], None


def check_ground_collision(angles, degrees=False):
    """
    Return True only when the arm stays above the ground and the suction cup
    end-effector touches y=0.

    The interactive robot stores joint angles in radians, so radians are the
    default here.
    """
    _, p1, p2, p3 = _joint_positions_from_radians(
        np.radians(angles) if degrees else angles
    )

    joints_safe = bool(p1[1] >= GROUND_LEVEL and p2[1] >= GROUND_LEVEL)
    effector_touching = bool(abs(p3[1] - GROUND_LEVEL) <= TOUCH_TOLERANCE)
    safe = joints_safe and effector_touching

    return safe, {"p1": float(p1[1]), "p2": float(p2[1]), "p3": float(p3[1])}


def dh_matrix(theta, d, a, alpha):
    """Create a 4x4 homogeneous DH transform matrix."""
    return np.array(
        [
            [
                math.cos(theta),
                -math.sin(theta) * math.cos(alpha),
                math.sin(theta) * math.sin(alpha),
                a * math.cos(theta),
            ],
            [
                math.sin(theta),
                math.cos(theta) * math.cos(alpha),
                -math.cos(theta) * math.sin(alpha),
                a * math.sin(theta),
            ],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


def forward_kinematics(angles, dh_params):
    """Calculate end-effector position (x, y, z) from joint angles."""
    return forward_kinematics_direct(angles)


def jacobian_numeric(angles, dh_params=None, eps=1e-5):
    """Calculate the position Jacobian using finite differences."""
    base_pos, _ = forward_kinematics(angles, dh_params)
    n = len(angles)
    J = np.zeros((3, n), dtype=float)

    for i in range(n):
        perturbed = angles.copy()
        perturbed[i] += eps
        pos_pert, _ = forward_kinematics(perturbed, dh_params)
        J[:, i] = (pos_pert - base_pos) / eps

    return J


def inverse_kinematics(target, init_angles, dh_params=None, max_iters=1000, tol=1e-4, alpha=0.1):
    """
    Iterative IK kept for compatibility with older demos.

    This function uses degrees internally.
    """
    angles = np.array(init_angles, dtype=float)
    target = np.array(target, dtype=float)

    target[1] = max(0.1, target[1])

    best_angles = angles.copy()
    best_error = float("inf")

    for restart in range(3):
        if restart > 0:
            angles = best_angles + np.random.normal(0, 5, 4)

        for _ in range(max_iters):
            pos, _ = forward_kinematics(angles.tolist(), dh_params)
            error = target - pos
            err_norm = np.linalg.norm(error)

            if err_norm < best_error:
                best_error = err_norm
                best_angles = angles.copy()

            if err_norm < tol:
                break

            J = jacobian_numeric(angles.tolist(), dh_params, eps=1e-4)
            lambda_sq = 0.001 + 0.01 * err_norm
            J_T = J.T

            try:
                J_inv = J_T @ np.linalg.inv(J @ J_T + lambda_sq * np.eye(3))
                dq = J_inv @ error
                angles += alpha * dq
            except np.linalg.LinAlgError:
                lambda_sq = 0.1
                J_inv = J_T @ np.linalg.inv(J @ J_T + lambda_sq * np.eye(3))
                dq = J_inv @ error
                angles += alpha * 0.1 * dq

            angles[0] = np.clip(angles[0], -180, 180)
            angles[1] = np.clip(angles[1], -90, 90)
            angles[2] = np.clip(angles[2], -180, 180)
            angles[3] = np.clip(angles[3], -180, 180)

    return best_angles.tolist()


def inverse_kinematics_optimized(target, current_angles):
    """
    Analytical IK for the ground-pick robot.

    Returns joint angles in radians. The suction cup is constrained to point
    straight down and touch the ground at y=0.
    """
    tx, _, tz = np.asarray(target, dtype=float)
    current_angles = np.asarray(current_angles, dtype=float)

    q_yaw = current_angles[0] + _wrap_to_pi(math.atan2(tx, tz) - current_angles[0])
    r_target = math.hypot(tx, tz)

    # With the cup vertical, the wrist must be L3 above the ground and has
    # the same horizontal radius as the target.
    r_rel = r_target
    y_rel = (GROUND_LEVEL + L3) - BASE_HEIGHT
    d_sq = r_rel**2 + y_rel**2
    d = math.sqrt(d_sq)

    if d > L1 + L2 + 1e-9 or d < abs(L1 - L2) - 1e-9 or d <= 1e-12:
        return None

    cos_elbow = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_elbow = float(np.clip(cos_elbow, -1.0, 1.0))
    elbow_abs = math.acos(cos_elbow)
    alpha = math.atan2(y_rel, r_rel)

    candidates = []
    for elbow in (-elbow_abs, elbow_abs):
        shoulder = alpha - math.atan2(
            L2 * math.sin(elbow),
            L1 + L2 * math.cos(elbow),
        )
        wrist = -math.pi / 2 - shoulder - elbow
        candidate = np.array([q_yaw, shoulder, elbow, wrist], dtype=float)
        safe, heights = check_ground_collision(candidate)

        if safe:
            motion_cost = sum(
                abs(_wrap_to_pi(candidate[i] - current_angles[i])) for i in range(4)
            )
            min_height = min(heights["p1"], heights["p2"])
            candidates.append((motion_cost, -min_height, candidate))

    if not candidates:
        return None

    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][2].tolist()
