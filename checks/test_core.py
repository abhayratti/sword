"""Offline checks. Never connects to a robot or headset. Run: python -m unittest discover checks"""
from pathlib import Path
import tempfile
import time
import tomllib
import unittest
import numpy as np
from aiohttp.test_utils import TestClient, TestServer
from sword_app.__main__ import create_app
from sword_app.blocking import Blocker, band
from sword_app.math3d import from_rotvec, rotvec, user_frame
from sword_app.robot import Robot, joint_step
from sword_app.state import State

ROOT = Path(__file__).resolve().parents[1]
CFG = tomllib.loads((ROOT/'config.toml').read_text())
ZCFG = {**CFG['tracking'], **CFG['blocking']}
GUARD = CFG['blocks']['guard']


def run(blocker, points, start=0., rate=90):
    """Feed (x, y, z) offsets at `rate` Hz; return the zone after each sample."""
    return [blocker.update(p, start + i / rate) for i, p in enumerate(points)]


class Math(unittest.TestCase):
    def test_rotvec_roundtrip(self):
        for v in [[.1, -.2, .3], [0, 0, 3.1], [2., 1., -.5], [0, 0, 0]]:
            r = from_rotvec(np.array(v, float))
            np.testing.assert_allclose(from_rotvec(rotvec(r)), r, atol=1e-6)

    def test_user_frame_faces_forward(self):
        np.testing.assert_allclose(user_frame([0, 0, 0, 1]), [[1, 0, 0], [0, 1, 0], [0, 0, -1]], atol=1e-9)


class Blocking(unittest.TestCase):
    def test_band_hysteresis(self):
        self.assertEqual(band(.16, .15, .03, 0), 0)   # not far enough in to enter
        self.assertEqual(band(.19, .15, .03, 0), 1)
        self.assertEqual(band(.13, .15, .03, 1), 1)   # not far enough back to leave
        self.assertEqual(band(.11, .15, .03, 1), 0)

    def test_slow_move_switches_after_dwell(self):
        b = Blocker(ZCFG)
        zones = run(b, [[.3, .3, 0]] * 30)  # opponent's right and high = robot's high left
        self.assertEqual(zones[0], 'guard')  # one sample is not enough
        self.assertEqual(zones[-1], 'high_left')

    def test_hysteresis_holds_near_edges(self):
        b = Blocker(ZCFG)
        self.assertEqual(run(b, [[.16, 0, 0]] * 30)[-1], 'guard')
        self.assertEqual(run(b, [[.25, 0, 0]] * 30, 1)[-1], 'mid_left')
        self.assertEqual(run(b, [[.14, 0, 0]] * 30, 2)[-1], 'mid_left')
        self.assertEqual(run(b, [[.10, 0, 0]] * 30, 3)[-1], 'guard')

    def test_strike_locks_the_wind_up_block_then_releases(self):
        b = Blocker(ZCFG)
        run(b, [[.3, .3, 0]] * 30)  # wind-up: high left
        swing = [[.3 - .6 * i / 18, .3 - .6 * i / 18, 0] for i in range(1, 19)]  # 0.2 s diagonal, ~4 m/s
        self.assertTrue(all(z == 'high_left' for z in run(b, swing, 1)))
        self.assertTrue(b.strike)
        after = run(b, [[-.3, -.3, 0]] * 60, 2)
        self.assertEqual(after[5], 'high_left')  # still locked just after the strike
        self.assertEqual(after[-1], 'guard')  # then covers where the sword now is (low counts as guard)

    def test_low_and_centre_are_guard(self):
        b = Blocker(ZCFG)
        self.assertEqual(run(b, [[.3, -.3, 0]] * 30)[-1], 'guard')
        self.assertEqual(run(b, [[0, 0, i / 90] for i in range(30)], 1)[-1], 'guard')  # thrust: no block


class Tracking(unittest.TestCase):
    def test_controller_to_zone(self):
        with tempfile.TemporaryDirectory() as tmp:
            state, now = State(ZCFG, tmp), time.monotonic()
            up, ident = [0, 0, 0, 1], np.eye(3)
            state.update([0, 1, -.5], ident, up, 'e', now)  # held straight up, blade along the grip's +Y
            self.assertIn('Blade direction saved', state.set_blade_up())
            state.set_guard()
            for i in range(60):  # move the hilt 0.3 m to the opponent's right
                state.update([.3, 1, -.5], ident, up, 'e', now + (i + 1) / 72)
            self.assertEqual(state.block_zone(), 'mid_left')
            self.assertTrue((Path(tmp) / 'blade.json').exists())

    def test_lost_tracking_returns_to_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = State(ZCFG, tmp)
            state.blocker.zone = 'high'; state.neutral = {'tip': np.zeros(3), 'frame': np.eye(3)}
            state.pose = {'p': np.zeros(3), 'r': np.eye(3), 'at': time.monotonic() - 5, 'head_q': [0, 0, 0, 1]}
            self.assertEqual(state.block_zone(), 'guard')


class FakeArm:
    connected = True; version = 'fake'

    def __init__(self): self.q = np.array(GUARD, float); self.sent = []; self.states = []; self.error_code = 0
    def clean_warn(self): return 0
    def clean_error(self): self.error_code = 0; return 0
    def motion_enable(self, enable): return 0
    def set_mode(self, mode): return 0
    def set_state(self, state): self.states.append(state); return 0
    def get_servo_angle(self, is_radian=False): return 0, [*self.q, 0.]
    def set_servo_angle(self, angle, **kw): self.q = np.array(angle[:6], float); return 0
    def vc_set_joint_velocity(self, v, **kw): self.sent.append(np.array(v[:6])); return 0
    def disconnect(self): pass


class RobotLoop(unittest.TestCase):
    def test_joint_step_is_coordinated_and_capped(self):
        cfg = CFG['robot']; target = np.array([40, -20, 10, 0, 5, -30.])
        v = joint_step(target, np.zeros(6), np.zeros(6), 10, cfg)
        np.testing.assert_allclose(v / np.max(np.abs(v)), target / np.max(np.abs(target)))  # straight line
        self.assertTrue(np.all(np.abs(v) <= np.array(cfg['max_speed_deg_s']) + 1e-9))

    def _robot(self, tmp):
        state = State(ZCFG, tmp)
        state.neutral = {'tip': np.zeros(3), 'frame': np.eye(3)}
        state.pose = {'p': np.zeros(3), 'r': np.eye(3), 'at': time.monotonic() + 60, 'head_q': [0, 0, 0, 1]}
        robot = Robot(state, CFG['robot'], tmp, CFG['blocks']); robot.arm = FakeArm()
        return state, robot

    def test_follows_the_chosen_block_and_stops(self):
        with tempfile.TemporaryDirectory() as tmp:
            state, robot = self._robot(tmp)
            robot.blocks['mid_left'] = [g + 10 for g in GUARD]
            state.blocker.zone = 'mid_left'
            robot._start(); time.sleep(.01); robot._tick()
            self.assertTrue(np.all(robot.arm.sent[-1] > 0))
            robot._halt('Operator Stop')
            self.assertEqual(robot.arm.states[-1], 4); self.assertFalse(robot.active)

    def test_collision_returns_to_guard_and_keeps_blocking(self):
        with tempfile.TemporaryDirectory() as tmp:
            state, robot = self._robot(tmp)
            robot._start(); robot.arm.q = np.array(GUARD) + 15; robot.arm.error_code = 31
            robot._tick()
            np.testing.assert_allclose(robot.arm.q, GUARD); self.assertTrue(robot.active)
            robot._halt('done')

    def test_save_block(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, robot = self._robot(tmp)
            robot.arm.q = np.array(GUARD) + 1; robot._save_block('high')
            np.testing.assert_allclose(Robot(robot.state, CFG['robot'], tmp, CFG['blocks']).blocks['high'], np.array(GUARD) + 1)


class Server(unittest.IsolatedAsyncioTestCase):
    async def test_state_and_token(self):
        async with TestClient(TestServer(create_app())) as client:
            data = await (await client.get('/api/state')).json()
            self.assertIn('high_left', data['blocks'])
            self.assertEqual((await client.post('/api/action', json={'action': 'stop'})).status, 403)
            ok = await client.post('/api/action', json={'action': 'stop'}, headers={'X-Sword-Key': data['key']})
            self.assertEqual(ok.status, 200)


if __name__ == '__main__': unittest.main()
