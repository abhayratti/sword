"""Latest tracking state shared by the camera, UI, and robot worker."""
import copy
import threading
import time
import numpy as np
from .math3d import source_basis


class State:
    def __init__(self, timeout=.5):
        self.lock = threading.RLock()
        self.timeout = timeout
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
        """Report a tracking problem. The last pose stays usable until it times out."""
        with self.lock:
            self.reason = reason
            if reset: self.pose = self.neutral = None

    def update(self, pose, epoch, now=None):
        kind = pose['kind']
        with self.lock:
            if self.epoch != epoch or self.kind != kind: self.neutral = None  # new coordinate frame
            self.epoch, self.kind = epoch, kind
            self.pose = {'p': np.asarray(pose['p'], float), 'r': np.asarray(pose['r'], float),
                         'at': time.monotonic() if now is None else now, 'kind': kind, 'head_q': pose.get('head_q')}
            self.frames += 1; self.reason = 'Tracking'

    def current(self, now=None):
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

    def target_source(self):
        with self.lock:
            pose = self.current()
            if pose is None or self.neutral is None: return None, None
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
