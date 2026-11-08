"""Rigid transforms. Metres for tracking; millimetres at the robot boundary."""
import math
import cv2
import numpy as np


def from_quaternion(value):
    q = np.asarray(value, dtype=float)
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def from_rpy(value):
    r, p, y = np.radians(np.asarray(value, dtype=float))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def to_rpy(r):
    p = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(p)) > 1e-7:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0., math.atan2(-r[0, 1], r[1, 1])
    return np.degrees([roll, p, yaw])


def mirrored_pose(source, neutral, home_pose, gain=1., rotation_gain=1.):
    """Apply the tracked motion since the neutral guard to the robot's home pose."""
    m = neutral['basis']
    p = np.asarray(home_pose[:3]) + gain * 1000 * (m @ (source['p'] - neutral['p']))
    turn = cv2.Rodrigues(m @ source['r'] @ neutral['r'].T @ m.T)[0] * rotation_gain
    r = cv2.Rodrigues(turn)[0] @ from_rpy(home_pose[3:])
    return np.r_[p, to_rpy(r)]


# Marker/board axes: +X right, +Y toward the arrow, +Z toward the camera.
MARKER_BASIS = np.array([[0., 0., -1.], [1., 0., 0.], [0., 1., 0.]])


def source_basis(kind, head_q=None):
    if kind == 'markers': return MARKER_BASIS
    forward = from_quaternion(head_q or [0, 0, 0, 1]) @ np.array([0., 0., -1.]); forward[1] = 0
    if np.linalg.norm(forward) < .1: forward = np.array([0., 0., -1.])
    forward /= np.linalg.norm(forward)
    axes = np.column_stack((np.cross(forward, [0, 1, 0]), [0, 1, 0], -forward))
    return MARKER_BASIS @ axes.T
