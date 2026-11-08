"""Latest tracking state shared by the headset link, UI, and robot worker."""
import copy
import json
import math
import threading
import time
from pathlib import Path
import numpy as np
from .blocking import Blocker
from .math3d import from_rotvec, rotvec, user_frame


class PoseFilter:
    """One Euro filter on position and orientation: steady when still, responsive when fast."""

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
        self.velocity = a_d * (p - self.raw_p) / dt + (1 - a_d) * self.velocity
        self.spin = a_d * rotvec(r @ self.raw_r.T) / dt + (1 - a_d) * self.spin
        self.raw_p, self.raw_r = p, r
        a_p = self._alpha(dt, self.min_cutoff + self.betas[0] * np.linalg.norm(self.velocity))
        a_r = self._alpha(dt, self.min_cutoff + self.betas[1] * np.linalg.norm(self.spin))
        self.p = self.p + a_p * (p - self.p)
        self.r = from_rotvec(a_r * rotvec(r @ self.r.T)) @ self.r
        return self.p, self.r


class State:
    def __init__(self, cfg, storage):
        self.lock = threading.RLock()
        self.cfg = cfg  # [tracking] and [blocking] merged
        self.blade_file = Path(storage) / 'blade.json'
        self.filter = PoseFilter(cfg['smoothing_min_cutoff_hz'], cfg['smoothing_beta_position'], cfg['smoothing_beta_rotation'])
        self.blocker = Blocker(cfg)
        self.blade_axis = None  # blade direction in the controller's frame
        if self.blade_file.exists(): self.blade_axis = np.array(json.loads(self.blade_file.read_text())['axis'])
        self.pose = self.neutral = None
        self.epoch = None
        self.frames = 0
        self.reason = 'Start tracking'
        self.robot = {'state': 'disconnected', 'reason': 'Connect the robot when ready', 'joints': None}

    def note(self, reason, *, reset=False):
        with self.lock:
            self.reason = reason
            if reset: self._reset()

    def _reset(self):
        self.pose = self.neutral = None; self.filter.reset(); self.blocker.reset()

    def _tip(self, pose):
        return pose['p'] + pose['r'] @ self.blade_axis * self.cfg['blade_length_m']

    def update(self, p, r, head_q, epoch, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.epoch != epoch: self._reset()  # new tracking session or reference frame
            self.epoch = epoch
            fp, fr = self.filter(np.asarray(p, float), np.asarray(r, float), now)
            self.pose = {'p': fp, 'r': fr, 'at': now, 'head_q': head_q}
            if self.neutral is not None:
                # [opponent's right, up, toward robot]. Facing each other, the opponent's right is the robot's left.
                offset = self.neutral['frame'] @ (self._tip(self.pose) - self.neutral['tip'])
                self.blocker.update(offset, now)
            self.frames += 1; self.reason = 'Tracking'

    def current(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.pose is None or now - self.pose['at'] > self.cfg['tracking_timeout_s']: return None
            return copy.deepcopy(self.pose)

    def set_blade_up(self):
        """Hold the sword pointing straight up and call this to learn how the controller is taped on."""
        with self.lock:
            pose = self.current()
            if pose is None: raise ValueError('Controller not tracked')
            self.blade_axis = pose['r'].T @ np.array([0., 1., 0.])
            self.blade_file.parent.mkdir(parents=True, exist_ok=True)
            self.blade_file.write_text(json.dumps({'axis': self.blade_axis.tolist()}))
            self.neutral = None; self.blocker.reset()
            return 'Blade direction saved; now set your guard'

    def set_guard(self):
        with self.lock:
            pose = self.current()
            if pose is None: raise ValueError('Controller not tracked')
            if self.blade_axis is None: raise ValueError('Calibrate the blade direction first (point it straight up, press B)')
            self.neutral = {'tip': self._tip(pose), 'frame': user_frame(pose['head_q'])}
            self.blocker.reset()
            return 'Guard set'

    def block_zone(self):
        """The block the robot should hold now. Guard if tracking is lost or not set up."""
        with self.lock:
            if self.neutral is None or self.current() is None: return 'guard'
            return self.blocker.zone

    def snapshot(self):
        with self.lock:
            robot = copy.deepcopy(self.robot)
            at = robot.pop('feedback_at', None)
            robot['feedback_age_ms'] = None if at is None else round((time.monotonic() - at) * 1000)
            fresh = self.current() is not None
            return {'tracking': {'fresh': fresh, 'reason': self.reason, 'frames': self.frames,
                                 'age_ms': None if self.pose is None else round((time.monotonic() - self.pose['at']) * 1000),
                                 'blade_calibrated': self.blade_axis is not None, 'guard_set': self.neutral is not None},
                    'threat': {**self.blocker.snapshot(), 'zone': self.block_zone()}, 'robot': robot}
