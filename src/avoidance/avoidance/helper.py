import numpy as np
import collision_geom as cg

__all__ = ["quaternion_slerp", "rollout_spheres", "quat_to_R"]

TOLERANCE = 1e-8  # rad - treat anything smaller as zero

def skew(w: np.ndarray) -> np.ndarray:
    """Return the 3x3 skew-symmetric matrix of a 3-vector.

    Args:
        w: (3,) array_like - the vector.

    Returns:
        3x3 ndarray - the matrix such that (skew(w) @ v) = w x v.
    """
    wx, wy, wz = w
    return np.array([[0.0, -wz,  wy],
                     [wz,  0.0, -wx],
                     [-wy, wx,  0.0]], dtype=w.dtype)

def quat_to_R(q) -> np.ndarray:
    """Convert a quaternion **(x, y, z, w)** to a 3x3 rotation matrix
    Returns:
        3x3 ndarray - the corresponding rotation matrix in **SO(3)**.
    """
    q = np.asarray(q, dtype=float)
    assert q.shape == (4,), "Quaternion must be length-4 (x, y, z, w)"
    x, y, z, w = q / np.linalg.norm(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),     2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),     1.0 - 2.0 * (xx + yy)]
    ], dtype=np.float64)

def gamma_matrices(w, dt):
    """
    gamma_0, gamma_1, gamma_2 for constant body-frame angular rate w (rad/s) over dt.
    """
    w = np.asarray(w, dtype=np.float64)
    theta = np.linalg.norm(w) * dt
    I3 = np.eye(3, dtype=np.float64)
    A = skew(w)

    if theta < 1e-8:                                      # series expansion
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        A2 = A @ A
        G0 = I3 + A * dt + 0.5 * A2 * dt2
        G1 = I3 * dt + 0.5 * A * dt2 + (1.0 / 6.0) * A2 * dt3
        G2 = 0.5 * I3 * dt2 + (1.0 / 6.0) * A * dt3 + (1.0 / 24.0) * A2 * dt4
        return G0, G1, G2

    # closed-form coefficients
    sin_t, cos_t = np.sin(theta), np.cos(theta)
    A2 = A @ A
    w2 = np.dot(w, w)
    w3 = w2 * np.sqrt(w2)

    G0 = I3 + (sin_t / theta) * A + ((1.0 - cos_t) / (theta ** 2)) * A2
    G1 = (dt * I3 +
          ((1.0 - cos_t) / w2) * A +
          ((theta - sin_t) / w3) * A2)
    G2 = (0.5 * dt * dt * I3 +
          ((theta - sin_t) / w3) * A +
          ((theta ** 2 + 2.0 * cos_t - 2.0) / (2.0 * w2 * w2)) * A2)
    return G0, G1, G2

def quaternion_slerp(q0, q1, t):
    """
    Spherical linear interpolation between quaternions q0 → q1.
    * q0, q1: iterable length (x, y, z, w), need not be normalised
    * t: float in [0, 1]
    Returns a normalised quaternion (x, y, z, w).
    """
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)

    # Normalise & make sure we take the short arc
    q0 /= np.linalg.norm(q0)
    q1 /= np.linalg.norm(q1)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1

    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)

    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q0 + s1 * q1)


def rollout_spheres(offsets, radii, pose, imu, v0,
                    horizon, dt_step, gravity=np.array([0, 0, -9.81])):
    """
    offsets : (NS,3) ndarray  - sphere centres in body frame
    radii   : (NS,)  ndarray  - matching radii
    pose    : (p0, q0)        - world position & quaternion at t₀
    imu     : (omega,  a)     - angular vel & lin accel in body frame
    v0      : (3,) ndarray    - world-frame velocity at t₀
    Returns : list[n_steps][Sphere]  (ready for collision_geom.check_collision)
    """
    p0, quat0 = pose
    ω, a      = imu

    R0 = quat_to_R(quat0)
    n_steps = int(np.ceil(horizon / dt_step))
    layers  = []

    for k in range(1, n_steps + 1):
        dt  = k * dt_step
        G0, G1, G2 = gamma_matrices(ω, dt)

        R   = R0 @ G0
        p   = (p0 + v0 * dt + (R0 @ G2 @ a) + 0.5 * gravity * dt ** 2)

        centres = p + (R @ offsets.T).T
        layer   = [cg.Sphere(centres[i], float(radii[i]))
                   for i in range(radii.size)]
        layers.append(layer)

    return layers
