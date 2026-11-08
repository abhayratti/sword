"""Latest tracking state shared by the camera, UI, and robot worker."""
import copy
import threading
import time
import cv2
import numpy as np
from .math3d import source_basis


class State:
    def __init__(self, timeout=.5, prediction_s=.3):
        self.lock = threading.RLock()
        self.timeout = timeout  # how long a measurement counts as live (UI, setting the guard)
        self.prediction_s = prediction_s
        self.velocity, self.spin = np.zeros(3), np.zeros(3)  # smoothed m/s and rad/s
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
            if reset: self.pose = self.neutral = None

    def update(self, pose, epoch, now=None):
        kind, now = pose['kind'], time.monotonic() if now is None else now
        p, r = np.asarray(pose['p'], float), np.asarray(pose['r'], float)
        with self.lock:
            if self.epoch != epoch or self.kind != kind: self.pose = self.neutral = None  # new coordinate frame
            last = self.pose
            if last is not None and 0 < now - last['at'] <= .25:
                dt = now - last['at']
                self.velocity = .5 * self.velocity + .5 * (p - last['p']) / dt
                self.spin = .5 * self.spin + .5 * cv2.Rodrigues(r @ last['r'].T)[0].ravel() / dt
            else:  # first sample, or after a long gap the old motion no longer applies
                self.velocity, self.spin = np.zeros(3), np.zeros(3)
            self.epoch, self.kind = epoch, kind
            self.pose = {'p': p, 'r': r, 'at': now, 'kind': kind, 'head_q': pose.get('head_q')}
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
        """Latest pose, carried forward along its recent motion for up to prediction_s.

        It never expires: when tracking drops out the robot continues on the predicted
        path, then holds that pose until tracking returns."""
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.pose is None or self.neutral is None: return None, None
            pose = copy.deepcopy(self.pose)
            pose['age'] = now - pose['at']
            ahead = min(max(pose['age'], 0.), self.prediction_s)
            pose['p'] = pose['p'] + self.velocity * ahead
            pose['r'] = cv2.Rodrigues(self.spin * ahead)[0] @ pose['r']
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
