"""Latest tracking state shared by the camera, UI, and robot worker."""
import copy
import math
import threading
import time
import cv2
import numpy as np
from .math3d import source_basis


def _rotvec(r): return cv2.Rodrigues(r)[0].ravel()


class PoseFilter:
    """One Euro filter on position and orientation.

    Heavy smoothing when the sword is still (removes jitter), little smoothing when it
    moves fast (keeps swings responsive). Also tracks a smoothed velocity."""

    def __init__(self, min_cutoff_hz, beta_position, beta_rotation, derivative_cutoff_hz=2.):
        self.min_cutoff, self.betas, self.d_cutoff = min_cutoff_hz, (beta_position, beta_rotation), derivative_cutoff_hz
        self.reset()

    def reset(self):
        self.p = self.r = self.at = self.raw_p = self.raw_r = None
        self.velocity, self.spin = np.zeros(3), np.zeros(3)

    @staticmethod
    def _alpha(dt, cutoff): return 1 / (1 + 1 / (2 * math.pi * cutoff * dt))

    def __call__(self, p, r, at):
        if self.p is None or not 0 < at - self.at <= .25:  # first sample or after a long gap
            self.reset(); self.p, self.r, self.at, self.raw_p, self.raw_r = p, r, at, p, r
            return p, r
        dt = at - self.at; self.at = at
        a_d = self._alpha(dt, self.d_cutoff)
        # Velocity from consecutive raw samples (unbiased), then smoothed.
        self.velocity = a_d * (p - self.raw_p) / dt + (1 - a_d) * self.velocity
        self.spin = a_d * _rotvec(r @ self.raw_r.T) / dt + (1 - a_d) * self.spin
        self.raw_p, self.raw_r = p, r
        dp, dr = p - self.p, _rotvec(r @ self.r.T)
        a_p = self._alpha(dt, self.min_cutoff + self.betas[0] * np.linalg.norm(self.velocity))
        a_r = self._alpha(dt, self.min_cutoff + self.betas[1] * np.linalg.norm(self.spin))
        self.p = self.p + a_p * dp
        self.r = cv2.Rodrigues(a_r * dr)[0] @ self.r
        return self.p, self.r


class State:
    def __init__(self, timeout=.5, prediction_s=.3, smoothing=(1., 10., 3.)):
        self.lock = threading.RLock()
        self.timeout = timeout  # how long a measurement counts as live (UI, setting the guard)
        self.prediction_s = prediction_s
        self.filter = PoseFilter(*smoothing)
        self.raw = None  # last accepted unfiltered measurement, for flip rejection
        self.pose = self.neutral = None
        self.epoch = None
        self.kind = 'markers'
        self.frames = 0
        self.reason = 'Start tracking'
        self.robot = {'state': 'disconnected', 'reason': 'Connect the robot when ready', 'joints': None}
        self.vision = {'objects': [], 'camera': {'calibrated': False, 'views': 0, 'needed': 12}}
        self.frame_jpeg = None
        self.last_frame_at = None

    def note(self, reason, *, reset=False):
        """Report a tracking problem. The last pose stays in use; only a reset discards it."""
        with self.lock:
            self.reason = reason
            if reset: self.pose = self.neutral = self.raw = None; self.filter.reset()

    def update(self, pose, epoch, now=None):
        kind, now = pose['kind'], time.monotonic() if now is None else now
        p, r = np.asarray(pose['p'], float), np.asarray(pose['r'], float)
        with self.lock:
            if self.epoch != epoch or self.kind != kind:  # new coordinate frame
                self.pose = self.neutral = self.raw = None; self.filter.reset()
            if self.raw is not None and now - self.raw['at'] < .1 and \
                    np.linalg.norm(_rotvec(r @ self.raw['r'].T)) > math.radians(60):
                # A >60° turn within one or two frames is a pose-solver flip, not a real swing.
                self.reason = 'Ignored a flipped marker pose'
                return
            self.raw = {'r': r, 'at': now}
            fp, fr = self.filter(p, r, now)
            self.epoch, self.kind = epoch, kind
            self.pose = {'p': fp, 'r': fr, 'at': now, 'kind': kind, 'head_q': pose.get('head_q')}
            self.frames += 1; self.reason = 'Tracking'

    def current(self, now=None):
        """The latest measurement if it is still live, else None."""
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.pose is None or now - self.pose['at'] > self.timeout: return None
            return copy.deepcopy(self.pose)

    def set_guard(self):
        with self.lock:
            pose = self.current()
            if pose is None: raise ValueError('A live pose is required to set the guard')
            pose['basis'] = source_basis(pose['kind'], pose.get('head_q'))
            self.neutral = pose
            return 'Guard set'

    def target_source(self, now=None):
        """Latest filtered pose. It never expires.

        Once the marker has been missing for 0.1 s, the pose is carried forward along its
        smoothed motion for up to prediction_s, then held until tracking returns. Slow
        motion is not extrapolated, so a still sword stays still."""
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.pose is None or self.neutral is None: return None, None
            pose = copy.deepcopy(self.pose)
            pose['age'] = now - pose['at']
            ahead = min(max(pose['age'] - .1, 0.), self.prediction_s)
            v, w = self.filter.velocity, self.filter.spin
            if ahead and np.linalg.norm(v) > .1: pose['p'] = pose['p'] + v * ahead
            if ahead and np.linalg.norm(w) > 1.: pose['r'] = cv2.Rodrigues(w * ahead)[0] @ pose['r']
            return pose, copy.deepcopy(self.neutral)

    def snapshot(self):
        with self.lock:
            pose = self.current()
            robot = copy.deepcopy(self.robot)
            at = robot.pop('feedback_at', None)
            robot['feedback_age_ms'] = None if at is None else round((time.monotonic() - at) * 1000)
            return {'tracking': {'kind': self.kind, 'fresh': pose is not None, 'reason': self.reason,
                                 'age_ms': None if self.pose is None else round((time.monotonic() - self.pose['at']) * 1000),
                                 'guard_set': self.neutral is not None, 'frames': self.frames},
                    'robot': robot, 'vision': copy.deepcopy(self.vision),
                    'frame_age_ms': None if self.last_frame_at is None else round((time.monotonic() - self.last_frame_at) * 1000)}
