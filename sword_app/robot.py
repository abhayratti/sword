"""Single robot writer: blocks and attacks with taught poses, teaching helpers, explicit Stop."""
import json
from pathlib import Path
import queue
import random
import threading
import time
import numpy as np
from .blocking import BLOCKS

COLLISION_ERROR = 31  # C31: collision caused abnormal current
ONLINE_PLANNING = 6  # controller re-plans smoothly toward each new joint target, even mid-move


class Robot:
    def __init__(self, state, config, storage, blocks, attacks=None):
        self.state, self.cfg, self.storage = state, config, Path(storage)
        self.blocks_file = self.storage / 'blocks.json'
        self.blocks = {name: list(map(float, pose)) for name, pose in blocks.items()}  # from config.toml
        if self.blocks_file.exists(): self.blocks.update(json.loads(self.blocks_file.read_text()))
        self.jobs = queue.Queue()
        self.closed, self.stop_requested = threading.Event(), threading.Event()
        self.arm = None; self.active = False; self.moving = False; self.log = None
        self.sent = None  # block whose pose was last sent to the controller
        self.attacks = attacks or {}  # [attacks] from config.toml
        self.attack, self.next_attack_at = None, 0.
        self.last_feedback = 0.
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
        if code != 0:
            error = getattr(self.arm, 'error_code', 0) if self.arm else 0
            hint = ' — a joint is at its limit: in xArm Studio clear the error and move it back (Initial Position or manual mode)' if error == 23 else ''
            raise RuntimeError(f'{name} rejected by controller ({code})' + (f'; controller error C{error}' if error else '') + hint)

    def _connect(self):
        if self.arm: self.arm.disconnect()
        from xarm.wrapper import XArmAPI
        self.arm = arm = XArmAPI(self.cfg['ip'], is_radian=False)
        self._publish('connected', 'Connected', joints=self._actual().tolist(),
                      firmware=arm.version, feedback_at=time.monotonic())

    def _ready(self, mode):
        if not self.arm: raise ValueError('Connect the robot first')
        arm = self.arm
        arm.clean_warn(); arm.clean_error(); arm.motion_enable(True)
        self._check(arm.set_mode(mode), 'Set mode'); self._check(arm.set_state(0), 'Ready state')
        # Runtime settings only (not saved to the controller). The payload is rejected unless the
        # arm is ready, so it is applied here rather than at connect.
        # Not fatal: a rejected setting shouldn't block Home from getting the arm out of trouble.
        if arm.set_tcp_load(self.cfg['sword_weight_kg'], self.cfg['sword_center_of_gravity_mm']) or \
                arm.set_collision_sensitivity(self.cfg['collision_sensitivity']):
            self._record('warning', message='Payload or collision sensitivity not applied')
        self.moving = True

    def _errors(self):
        if not self.arm.connected: raise RuntimeError('Robot disconnected')
        if self.arm.error_code == 23:
            raise RuntimeError('Controller error C23 — a joint is at its limit: in xArm Studio clear the error and move it back (Initial Position or manual mode)')
        if self.arm.error_code: raise RuntimeError(f'Controller error C{self.arm.error_code}')

    def _actual(self):
        code, angles = self.arm.get_servo_angle(is_radian=False)
        self._check(code, 'Joint query')
        return np.array(angles[:6], float)

    def _halt(self, reason, status='stopped'):
        self.active = False
        if self.moving and self.arm and self.arm.connected:
            try: self.arm.set_state(4)
            except Exception as exc: reason += f'; stop not acknowledged: {exc}'
        self.moving = False
        self._record('stop', reason=reason)
        if self.log: self.log.close(); self.log = None
        self._publish(status if self.arm else 'disconnected', reason)

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
        self._ready(ONLINE_PLANNING); self.sent = None; self.attack = None; self._schedule_attack()
        self.storage.mkdir(parents=True, exist_ok=True)
        self.log = (self.storage / ('run-' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '.jsonl')).open('w')
        self._record('start', config=self.cfg, blocks=self.blocks)
        self.active = True; self._publish('running', 'Blocking')

    def _recover(self):
        """Collision: clear it, go back to guard, and keep blocking."""
        self._record('collision')
        self._publish('recovering', 'Collision; returning to guard')
        if not self._move_to(self.blocks['guard'], self.cfg['home_speed_deg_s'], 'Returning to guard after collision'): return
        self._ready(ONLINE_PLANNING); self.sent = None; self.attack = None; self._schedule_attack()

    def _send(self, pose, label, speed=None, accel=None):
        """Send a joint target; the controller plans a smooth move and lands exactly on it."""
        self._check(self.arm.set_servo_angle(angle=[*pose, 0.], speed=speed or self.cfg['block_speed_deg_s'],
                                             mvacc=accel or self.cfg['block_accel_deg_s2'], is_radian=False, wait=False), label)

    def _send_attack(self, pose, label):
        self._send(pose, label, self.attacks.get('speed_deg_s'), self.attacks.get('accel_deg_s2'))

    def _schedule_attack(self):
        low, high = self.attacks.get('interval_s', [2., 5.])
        self.next_attack_at = time.monotonic() + random.uniform(low, high)

    def _begin_strike(self, name, now, combo):
        moves = self.attacks['moves']
        self._send_attack(moves[name]['windup'], f'{name} wind-up')
        self.attack, self.sent = {'name': name, 'phase': 'windup', 'since': now, 'combo': combo}, None
        self._record('attack', name=name, phase='windup', remaining=len(combo))

    def _attack_step(self, now):
        """Run the current attack, or start one when the timer fires. An attack is a burst of 1-3
        strikes chained back to back. Returns True while an attack owns the arm."""
        moves = self.attacks.get('moves') or {}
        if self.attack is None:
            if not self.attacks.get('enabled') or not moves or now < self.next_attack_at or not self.state.engaged(): return False
            low, high = self.attacks.get('combo', [1, 1])
            names = [random.choice(list(moves)) for _ in range(random.randint(low, high))]
            self._begin_strike(names[0], now, names[1:])
            return True
        a, move, q = self.attack, moves[self.attack['name']], self._actual()
        if a['phase'] == 'windup':
            # Swing as soon as the wind-up is nearly reached, so wind-up and strike blend into one motion.
            if np.max(np.abs(q - move['windup'])) < self.attacks.get('blend_deg', 8.) or now - a['since'] > 2:
                self._send_attack(move['strike'], f"{a['name']} strike"); a.update(phase='strike', since=now)
                self._record('attack', name=a['name'], phase='strike')
            return True
        if np.max(np.abs(q - move['strike'])) < 3 or now - a['since'] > 2:  # strike landed
            if a['combo']:  # chain straight into the next strike
                self._begin_strike(a['combo'][0], now, a['combo'][1:]); return True
            self._record('attack', name=a['name'], phase='done')
            self.attack = None; self._schedule_attack(); return False  # back to blocking
        return True

    def _tick(self):
        if self.arm.error_code == COLLISION_ERROR and self.cfg['collision_auto_home']: return self._recover()
        self._errors()
        now = time.monotonic()
        attacking = self._attack_step(now)
        zone = self.state.block_zone()
        if not attacking and zone != self.sent:
            target = self.blocks.get(zone, self.blocks['guard'])
            self._send(target, f'Move to {zone}')
            self.sent = zone
            with self.state.lock: threat = self.state.blocker.snapshot()
            self._record('target', zone=zone, target_deg=list(target), threat=threat)
        if now - self.last_feedback > .03:
            self.last_feedback = now; q = self._actual()
            with self.state.lock: threat = self.state.blocker.snapshot()
            attack = self.attack and self.attack['name']
            self._record('sample', zone=zone, attack=attack, actual_deg=q.tolist(), threat=threat)
            self._publish('running', f'Attacking: {attack}' if attack else f'Blocking: {zone}', joints=q.tolist(), feedback_at=now)

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
