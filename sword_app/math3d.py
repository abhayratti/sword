"""Rigid transforms. Metres for tracking; millimetres at the robot boundary."""
import math
import cv2
import numpy as np


def vector(value, length):
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != length:
        raise ValueError(f"Expected {length} numbers")
    if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.number)) for v in value):
        raise ValueError("Expected finite numbers")
    a = np.asarray(value, dtype=float)
    if a.shape != (length,) or not np.isfinite(a).all():
        raise ValueError("Expected finite vector")
    return a


def rotation(value):
    a = np.asarray(value, dtype=float)
    if a.shape != (3, 3) or not np.isfinite(a).all() or not np.allclose(a.T @ a, np.eye(3), atol=1e-5) or not math.isclose(np.linalg.det(a), 1, abs_tol=1e-5):
        raise ValueError("Expected a proper rotation")
    return a


def from_quaternion(value):
    x, y, z, w = q = vector(value, 4)
    if abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError("Invalid orientation")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def from_rpy(value):
    r, p, y = np.radians(vector(value, 3))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def to_rpy(r):
    r = rotation(r)
    p = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(p)) > 1e-7:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0., math.atan2(-r[0, 1], r[1, 1])
    return np.degrees([roll, p, yaw])


def interpolate_rotation(a, b, fraction):
    rv = cv2.Rodrigues(b @ a.T)[0]
    return cv2.Rodrigues(rv * fraction)[0] @ a


def mirrored_pose(source, neutral, home_pose, gain, rotation_gain=1., position_deadband_m=0., angle_deadband_deg=0.):
    # Proper rotation remains proper when conjugated by a reflection.
    m = rotation(neutral['basis']) if neutral.get('basis_is_rotation') else np.asarray(neutral['basis'])
    delta=source['p']-neutral['p'];length=np.linalg.norm(delta)
    delta=delta*max(0.,1-position_deadband_m/max(length,1e-12))
    p = np.asarray(home_pose[:3]) + gain * 1000 * (m @ delta)
    turn=cv2.Rodrigues(m@source['r']@neutral['r'].T@m.T)[0];angle=np.linalg.norm(turn)
    turn*=rotation_gain*max(0.,1-math.radians(angle_deadband_deg)/max(angle,1e-12))
    r=cv2.Rodrigues(turn)[0]@from_rpy(home_pose[3:])
    return np.r_[p, to_rpy(r)]


def source_basis(kind, head_q=None, anchor_r=None):
    if kind == 'markers':
        # Anchor +X is right, +Y is toward arrow, +Z faces the camera.
        return np.array([[0.,0.,-1.],[1.,0.,0.],[0.,1.,0.]])
    head = from_quaternion(head_q or [0,0,0,1])
    forward = head @ np.array([0.,0.,-1.]); forward[1] = 0
    if np.linalg.norm(forward) < .1:
        raise ValueError('Look toward the robot when setting the guard')
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0,1,0])
    axes = np.column_stack((right, [0,1,0], -forward))
    return np.array([[0.,0.,-1.],[1.,0.,0.],[0.,1.,0.]]) @ axes.T
