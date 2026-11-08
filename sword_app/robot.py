"""Single robot writer: moves between taught block poses, teaching helpers, explicit Stop."""
import json
from pathlib import Path
import queue
import threading
import time
import numpy as np
from .blocking import BLOCKS

COLLISION_ERROR = 31  # C31: collision caused abnormal current


def joint_step(target, actual, previous, dt, cfg):
    """Coordinated joint velocity toward a target: all joints arrive together (straight line in
    joint space), within each joint's speed and acceleration caps."""
    accel, cap = np.array(cfg['max_accel_deg_s2']), np.array(cfg['max_speed_deg_s'])
    error = np.asarray(target, float) - np.asarray(actual, float)
    size = np.abs(error)
    desired = np.sign(error) * np.minimum.reduce([size * cfg['kp'], np.sqrt(1.3 * accel * size), cap])
    moving = size > 1e-3
    if moving.any(): desired = error * np.min(np.abs(desired[moving]) / size[moving])
    return np.clip(previous + np.clip(desired - previous, -accel * dt, accel * dt), -cap, cap)


class Robot:
    def __init__(self, state, config, storage, blocks):
        self.state, self.cfg, self.storage = state, config, Path(storage)
        self.blocks_file = self.storage / 'blocks.json'
        self.blocks = {name: list(map(float, pose)) for name, pose in blocks.items()}  # from config.toml
        if self.blocks_file.exists(): self.blocks.update(json.loads(self.blocks_file.read_text()))
        self.jobs = queue.Queue()
        self.closed, self.stop_requested = threading.Event(), threading.Event()
        self.arm = None; self.active = False; self.moving = False; self.log = None
        self.velocity = np.zeros(6); self.last_tick = self.last_feedback = 0.
        self.thread = threading.Thread(target=self._run, name='robot-writer', daemon=True)

    def start_worker(self): self.thread.start()

    def request(self, operation, arg=None):
        if operation not in {'connect', 'home', 'start', 'disconnect', 'freedrive', 'save_block', 'goto_block'}:
            raise ValueError('Unknown robot operation')
        if operation in {'save_block', 'goto_block'} and arg not in BLOCKS: raise ValueError('Unknown block')
        self.jobs.put((operation, arg))

    def stop(self): self.stop_requested.set()

    def close(self):
        self.stop(); self.closed.set(); self.thread.join(timeout=4)

    def _publish(self, status, reason, **values):
        with self.state.lock:
            self.state.robot.update(state=status, reason=reason, **values)

    def _record(self, event, **values):
        if self.log: self.log.write(json.dumps({'at': time.monotonic(), 'event': event, **values}) + '\n')

    def _check(self, code, name):
        if code != 0: raise RuntimeError(f'{name} rejected by controller ({code})')

    def _connect(self):
        if self.arm: self.arm.disconnect()
        from xarm.wrapper import XArmAPI
        self.arm = arm = XArmAPI(self.cfg['ip'], is_radian=False)
        # Runtime settings only (not saved to the controller): foam sword payload and collision sensitivity.
        self._check(arm.set_tcp_load(self.cfg['sword_weight_kg'], self.cfg['sword_center_of_gravity_mm']), 'Payload')
        self._check(arm.set_collision_sensitivity(self.cfg['collision_sensitivity']), 'Collision sensitivity')
        self._publish('connected', 'Connected', joints=self._actual().tolist(),
                      firmware=arm.version, feedback_at=time.monotonic())

    def _ready(self, mode):
        if not self.arm: raise ValueError('Connect the robot first')
        arm = self.arm
        arm.clean_warn(); arm.clean_error(); arm.motion_enable(True)
        self._check(arm.set_mode(mode), 'Set mode'); self._check(arm.set_state(0), 'Ready state')
        self.moving = True

    def _errors(self):
        if not self.arm.connected: raise RuntimeError('Robot disconnected')
        if self.arm.error_code: raise RuntimeError(f'Controller error C{self.arm.error_code}')

    def _actual(self):
        code, angles = self.arm.get_servo_angle(is_radian=False)
        self._check(code, 'Joint query')
        return np.array(angles[:6], float)

    def _halt(self, reason, status='stopped'):
        self.active = False; self.velocity[:] = 0
        if self.moving and self.arm and self.arm.connected:
            try: self.arm.set_state(4)
            except Exception as exc: reason += f'; stop not acknowledged: {exc}'
        self.moving = False
        self._record('stop', reason=reason)
        if self.log: self.log.close(); self.log = None
        self._publish(status if self.arm else 'disconnected', reason, command=[0.] * 6)

    def _move_to(self, joints, speed, label):
        """Position-mode move to a joint pose. Returns False if Stop interrupted it."""
        self._ready(0)
        joints = np.array(joints, float)
        self._check(self.arm.set_servo_angle(angle=[*joints, 0.], speed=speed, mvacc=200, is_radian=False, wait=False), label)
        deadline = time.monotonic() + np.max(np.abs(joints - self._actual())) / speed * 2 + 5
        while time.monotonic() < deadline:
            if self.stop_requested.is_set(): return False
            self._errors(); q = self._actual()
            self._publish('moving', label, joints=q.tolist(), feedback_at=time.monotonic())
            if np.max(np.abs(q - joints)) < .5:
                self.moving = False; return True
            time.sleep(.05)
        raise RuntimeError(f'{label} did not finish')

    def _freedrive(self, on):
        if on:
            self._ready(2)  # manual (teach) mode: the arm can be moved by hand
            self._publish('freedrive', 'Freedrive: move the arm by hand, then save a block')
        else:
            self._ready(0); self.moving = False
            self._publish('connected', 'Freedrive off')

    def _save_block(self, name):
        if not self.arm: raise ValueError('Connect the robot first')
        self.blocks[name] = [round(float(v), 2) for v in self._actual()]
        self.storage.mkdir(parents=True, exist_ok=True)
        self.blocks_file.write_text(json.dumps(self.blocks, indent=2))
        with self.state.lock: self.state.robot['reason'] = f'Saved {name}'

    def _start(self):
        self._ready(4)
        self.velocity = np.zeros(6); self.last_tick = time.monotonic()
        self.storage.mkdir(parents=True, exist_ok=True)
        self.log = (self.storage / ('run-' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '.jsonl')).open('w')
        self._record('start', config=self.cfg, blocks=self.blocks)
        self.active = True; self._publish('running', 'Blocking')

    def _recover(self):
        """Collision: clear it, go back to guard, and keep blocking."""
        self._record('collision')
        self._publish('recovering', 'Collision; returning to guard')
        if not self._move_to(self.blocks['guard'], self.cfg['home_speed_deg_s'], 'Returning to guard after collision'): return
        self._ready(4); self.velocity[:] = 0; self.last_tick = time.monotonic()

    def _tick(self):
        if self.arm.error_code == COLLISION_ERROR and self.cfg['collision_auto_home']: return self._recover()
        self._errors()
        q = self._actual(); now = time.monotonic()
        dt = min(now - self.last_tick, self.cfg['velocity_lease_s']); self.last_tick = now
        zone = self.state.block_zone()
        target = np.array(self.blocks.get(zone, self.blocks['guard']), float)
        self.velocity = joint_step(target, q, self.velocity, dt, self.cfg)
        self._check(self.arm.vc_set_joint_velocity([*self.velocity, 0.], is_radian=False, is_sync=False,
                                                   duration=self.cfg['velocity_lease_s']), 'Velocity')
        with self.state.lock: threat = self.state.blocker.snapshot()
        self._record('sample', zone=zone, actual_deg=q.tolist(), target_deg=target.tolist(),
                     command_deg_s=self.velocity.tolist(), threat=threat)
        taught = zone in self.blocks
        self._publish('running', f'Blocking: {zone}' + ('' if taught else ' (not taught; holding guard)'),
                      joints=q.tolist(), command=self.velocity.tolist(), feedback_at=now)

    def _run(self):
        try:
            while not self.closed.is_set():
                if self.stop_requested.is_set():
                    self.stop_requested.clear()
                    while not self.jobs.empty(): self.jobs.get_nowait()
                    self._halt('Operator Stop')
                try:
                    operation, arg = (None, None) if self.jobs.empty() else self.jobs.get_nowait()
                    if operation:
                        # Saving a pose must not end freedrive; anything else takes over the arm.
                        if operation != 'save_block' and (self.active or self.moving): self._halt('Interrupted by ' + operation)
                        if operation == 'connect': self._publish('working', 'Connecting'); self._connect()
                        elif operation == 'home':
                            if self._move_to(self.blocks['guard'], self.cfg['home_speed_deg_s'], 'Returning to guard'):
                                self._publish('connected', 'At guard')
                        elif operation == 'goto_block':
                            if self._move_to(self.blocks.get(arg, self.blocks['guard']), self.cfg['teach_speed_deg_s'], f'Moving to {arg}'):
                                self._publish('connected', f'At {arg}' + ('' if arg in self.blocks else ' (not taught; used guard)'))
                        elif operation == 'freedrive': self._freedrive(arg)
                        elif operation == 'save_block': self._save_block(arg)
                        elif operation == 'start': self._start()
                        elif operation == 'disconnect':
                            if self.arm: self.arm.disconnect(); self.arm = None
                            self._publish('disconnected', 'Robot disconnected')
                    if self.active: self._tick()
                    elif self.arm and self.arm.connected and time.monotonic() - self.last_feedback > .1:
                        self.last_feedback = time.monotonic()
                        with self.state.lock:
                            self.state.robot.update(joints=self._actual().tolist(), feedback_at=self.last_feedback)
                except Exception as exc:
                    self._halt(str(exc), 'fault')
                self.closed.wait(.005 if self.active else .05)
        finally:
            self._halt('Application closed')
            if self.arm: self.arm.disconnect()
