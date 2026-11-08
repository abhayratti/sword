"""Single robot writer: native IK, joint velocity control, explicit Stop."""
import json
from pathlib import Path
import queue
import threading
import time
import numpy as np
from .math3d import mirrored_pose


def velocity_step(target, actual, previous, dt, cfg):
    """Proportional joint response with speed and acceleration caps."""
    accel, cap = np.array(cfg['max_accel_deg_s2']), np.array(cfg['max_speed_deg_s'])
    error = np.asarray(target, float) - np.asarray(actual, float)
    desired = np.sign(error) * np.minimum(np.abs(error) * cfg['kp'], np.sqrt(1.3 * accel * np.abs(error)))
    return np.clip(previous + np.clip(desired - previous, -accel * dt, accel * dt), -cap, cap)


class Robot:
    def __init__(self, state, config, storage):
        self.state, self.cfg, self.storage = state, config, Path(storage)
        self.jobs = queue.Queue()
        self.closed, self.stop_requested = threading.Event(), threading.Event()
        self.arm = None; self.active = False; self.moving = False; self.log = None
        self.low, self.high = np.full(6, -360.), np.full(6, 360.)
        self.velocity = np.zeros(6); self.target = None
        self.last_tick = self.last_feedback = 0.
        self.thread = threading.Thread(target=self._run, name='robot-writer', daemon=True)

    def start_worker(self): self.thread.start()

    def request(self, operation):
        if operation not in {'connect', 'home', 'start', 'disconnect'}: raise ValueError('Unknown robot operation')
        self.jobs.put(operation)

    def stop(self): self.stop_requested.set()

    def close(self):
        self.stop(); self.closed.set(); self.thread.join(timeout=4)

    def _publish(self, status, reason, **values):
        with self.state.lock:
            self.state.robot.update(state=status, reason=reason, **values)

    def _record(self, event, **values):
        if self.log:
            self.log.write(json.dumps({'at': time.monotonic(), 'event': event, **values}) + '\n')

    def _check(self, code, name):
        if code != 0: raise RuntimeError(f'{name} rejected by controller ({code})')

    def _connect(self):
        if self.arm: self.arm.disconnect()
        from xarm.wrapper import XArmAPI
        self.arm = arm = XArmAPI(self.cfg['ip'], is_radian=False)
        code, native = arm.get_reduced_states(is_radian=False)
        if code == 0 and isinstance(native, list) and len(native) >= 5:
            joints = np.reshape(native[4], (7, 2))[:6]
            self.low, self.high = joints[:, 0], joints[:, 1]
        self._publish('connected', 'Connected', joints=self._actual().tolist(),
                      speed_caps=self.cfg['max_speed_deg_s'], firmware=arm.version, feedback_at=time.monotonic())

    def _ready(self, mode):
        if not self.arm: raise ValueError('Connect the robot first')
        arm = self.arm
        arm.clean_warn(); arm.clean_error(); arm.motion_enable(True)
        self._check(arm.set_mode(mode), 'Set mode'); self._check(arm.set_state(0), 'Ready state')
        self.moving = True

    def _errors(self):
        if not self.arm.connected: raise RuntimeError('Robot disconnected')
        if self.arm.error_code: raise RuntimeError(f'Controller error {self.arm.error_code}')

    def _actual(self):
        code, angles = self.arm.get_servo_angle(is_radian=False)
        self._check(code, 'Joint query')
        return np.array(angles[:6], float)

    def _fk(self, joints):
        code, pose = self.arm.get_forward_kinematics([*joints, 0.], input_is_radian=False, return_is_radian=False)
        self._check(code, 'Forward kinematics')
        return np.array(pose[:6], float)

    def _halt(self, reason, status='stopped'):
        self.active = False; self.velocity[:] = 0; self.target = None
        if self.moving and self.arm and self.arm.connected:
            try: self.arm.set_state(4)
            except Exception as exc: reason += f'; stop not acknowledged: {exc}'
        self.moving = False
        self._record('stop', reason=reason)
        if self.log: self.log.close(); self.log = None
        self._publish(status if self.arm else 'disconnected', reason, command=[0.] * 6)

    def _home(self):
        self._ready(0)
        guard = np.array(self.cfg['guard_deg'])
        self._check(self.arm.set_servo_angle(angle=[*guard, 0.], speed=self.cfg['home_speed_deg_s'], mvacc=200,
                                             is_radian=False, wait=False), 'Home move')
        self._publish('homing', 'Returning to guard')
        deadline = time.monotonic() + np.max(np.abs(guard - self._actual())) / self.cfg['home_speed_deg_s'] * 2 + 5
        while time.monotonic() < deadline:
            if self.stop_requested.is_set(): return
            self._errors(); q = self._actual()
            self._publish('homing', 'Returning to guard', joints=q.tolist(), feedback_at=time.monotonic())
            if np.max(np.abs(q - guard)) < .5:
                self.moving = False
                self._publish('connected', 'At guard'); return
            time.sleep(.05)
        raise RuntimeError('Home move did not finish')

    def _start(self):
        self._ready(4)
        self.home_pose = self._fk(self._actual())
        self.velocity = np.zeros(6); self.target = None; self.last_tick = time.monotonic()
        self.storage.mkdir(parents=True, exist_ok=True)
        self.log = (self.storage / ('run-' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '.jsonl')).open('w')
        self._record('start', config=self.cfg)
        self.active = True; self._publish('running', 'Following tracked pose')

    def _tick(self):
        self._errors()
        q = self._actual(); now = time.monotonic()
        dt = min(now - self.last_tick, self.cfg['velocity_lease_s']); self.last_tick = now
        source, neutral = self.state.target_source()
        if source is not None:
            desired = mirrored_pose(source, neutral, self.home_pose, self.cfg['position_gain'], self.cfg['rotation_gain'])
            code, solution = self.arm.get_inverse_kinematics(desired.tolist(), input_is_radian=False, return_is_radian=False)
            if code == 0:
                t = np.array(solution[:6], float)
                for j in (0, 3, 5): t[j] += 360 * round((q[j] - t[j]) / 360)  # nearest wrap of periodic joints
                self.target = np.clip(t, self.low, self.high)
        target = q if self.target is None else self.target
        self.velocity = velocity_step(target, q, self.velocity, dt, self.cfg)
        self._check(self.arm.vc_set_joint_velocity([*self.velocity, 0.], is_radian=False, is_sync=False,
                                                   duration=self.cfg['velocity_lease_s']), 'Velocity')
        self._record('sample', actual_deg=q.tolist(), target_deg=target.tolist(), command_deg_s=self.velocity.tolist())
        tracking = source is not None
        self._publish('running' if tracking else 'holding', 'Following tracked pose' if tracking else 'Waiting for tracking',
                      joints=q.tolist(), target=target.tolist(), command=self.velocity.tolist(), feedback_at=now)

    def _run(self):
        try:
            while not self.closed.is_set():
                if self.stop_requested.is_set():
                    self.stop_requested.clear()
                    while not self.jobs.empty(): self.jobs.get_nowait()
                    self._halt('Operator Stop')
                try:
                    operation = None if self.jobs.empty() else self.jobs.get_nowait()
                    if operation:
                        if self.active or self.moving: self._halt('Interrupted by ' + operation)
                        self._publish('working', operation.capitalize())
                        if operation == 'connect': self._connect()
                        elif operation == 'home': self._home()
                        elif operation == 'start': self._start()
                        elif operation == 'disconnect':
                            if self.arm: self.arm.disconnect(); self.arm = None
                            self._publish('disconnected', 'Robot disconnected')
                    if self.active: self._tick()
                    elif self.arm and self.arm.connected and time.monotonic() - self.last_feedback > .2:
                        self.last_feedback = time.monotonic()
                        with self.state.lock:
                            self.state.robot.update(joints=self._actual().tolist(), feedback_at=self.last_feedback)
                except Exception as exc:
                    self._halt(str(exc), 'fault')
                self.closed.wait(.005 if self.active else .05)
        finally:
            self._halt('Application closed')
            if self.arm: self.arm.disconnect()
