"""Rotation helpers (numpy only). WebXR space: +X right, +Y up, -Z forward."""
import math
import numpy as np


def from_quaternion(value):
    x, y, z, w = np.asarray(value, dtype=float) / np.linalg.norm(value)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotvec(r):
    """Rotation matrix -> axis * angle (radians)."""
    angle = math.acos(max(-1., min(1., (np.trace(r) - 1) / 2)))
    if angle < 1e-9: return np.zeros(3)
    if angle > math.pi - 1e-4:  # near 180 degrees the antisymmetric part vanishes
        b = (r + np.eye(3)) / 2; k = int(np.argmax(np.diag(b)))
        return b[:, k] / math.sqrt(b[k, k]) * angle
    return np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]) * angle / (2 * math.sin(angle))


def from_rotvec(v):
    """Axis * angle -> rotation matrix (Rodrigues)."""
    angle = np.linalg.norm(v)
    if angle < 1e-12: return np.eye(3)
    x, y, z = np.asarray(v) / angle
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def user_frame(head_q):
    """Rows: the user's right, up, and forward (toward the robot), from head yaw only."""
    forward = from_quaternion(head_q) @ np.array([0., 0., -1.]); forward[1] = 0
    forward = forward / np.linalg.norm(forward) if np.linalg.norm(forward) > .1 else np.array([0., 0., -1.])
    up = np.array([0., 1., 0.])
    return np.array([np.cross(forward, up), up, forward])
