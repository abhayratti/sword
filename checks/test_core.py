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
from sword_app.robot import Robot
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
        b = Blocker({**ZCFG, 'strike_lock': True})
        run(b, [[.3, .3, 0]] * 30)  # wind-up: high left
        swing = [[.3 - .6 * i / 18, .3 - .6 * i / 18, 0] for i in range(1, 19)]  # 0.2 s diagonal, ~4 m/s
        self.assertTrue(all(z == 'high_left' for z in run(b, swing, 1)))
        self.assertTrue(b.strike)
        after = run(b, [[-.3, -.3, 0]] * 60, 2)
        self.assertEqual(after[5], 'high_left')  # still locked just after the strike
        self.assertEqual(after[-1], 'mid_right')  # then covers where the sword now is (low right uses mid right)

    def test_fast_moves_switch_without_strike_lock(self):
        b = Blocker(ZCFG)  # strike_lock off in config
        run(b, [[.3, .3, 0]] * 30)
        swing = [[.3 - .6 * i / 18, .3 - .6 * i / 18, 0] for i in range(1, 19)]  # fast move to low right
        zones = run(b, swing + [[-.3, -.3, 0]] * 10, 1)
        self.assertEqual(zones[-1], 'mid_right')
        self.assertLess(zones.index('mid_right'), 22)  # switched within ~0.25 s of starting the move

    def test_low_sides_use_mid_blocks_and_centre_is_guard(self):
        b = Blocker(ZCFG)
        self.assertEqual(run(b, [[.3, -.3, 0]] * 30)[-1], 'mid_left')
        self.assertEqual(run(b, [[0, -.3, 0]] * 30, 2)[-1], 'guard')
        self.assertEqual(run(b, [[0, 0, i / 90] for i in range(30)], 4)[-1], 'guard')  # thrust: no block


    def test_calibrated_zones_pick_nearest_with_hysteresis(self):
        zones = {'guard': [0, 0], 'high': [0, .4], 'high_left': [.35, .3], 'mid_left': [.4, 0]}
        b = Blocker(ZCFG, zones)
        self.assertEqual(run(b, [[.02, .38, 0]] * 20)[-1], 'high')  # top centre really is high
        self.assertEqual(run(b, [[.18, .35, 0]] * 20, 1)[-1], 'high')  # about halfway: stays put
        self.assertEqual(run(b, [[.3, .3, 0]] * 20, 2)[-1], 'high_left')


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

    def test_zone_calibration_walkthrough(self):
        with tempfile.TemporaryDirectory() as tmp:
            state, now, ident, up = State(ZCFG, tmp), time.monotonic(), np.eye(3), [0, 0, 0, 1]
            state.update([0, 1, -.5], ident, up, 'e', now); state.set_blade_up(); state.press_a()  # guard
            state.start_zone_calibration()
            spots = {'guard': [0, 0], 'high': [0, .4], 'high_left': [.4, .3], 'high_right': [-.4, .3], 'mid_left': [.45, 0], 'mid_right': [-.45, 0]}
            t = now
            for name, (x, y) in spots.items():
                self.assertEqual(state.snapshot()['tracking']['calibrating'], name)
                self.assertEqual(state.block_zone(), 'guard')  # robot holds guard while calibrating
                for _ in range(40): t += 1 / 72; state.update([x, 1 + y, -.5], ident, up, 'e', t)
                message = state.press_a()
            self.assertEqual(message, 'Zones saved')
            self.assertTrue((Path(tmp) / 'zones.json').exists())
            np.testing.assert_allclose(state.blocker.zones['high'], [0, .4], atol=.02)
            self.assertEqual(State(ZCFG, tmp).blocker.zones.keys(), spots.keys())  # reloaded on restart
            for _ in range(40): t += 1 / 72; state.update([.38, 1.3, -.5], ident, up, 'e', t)
            self.assertEqual(state.block_zone(), 'high_left')

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
    def set_tcp_load(self, weight, cog): return 0
    def set_collision_sensitivity(self, value): return 0
    def get_servo_angle(self, is_radian=False): return 0, [*self.q, 0.]
    def set_servo_angle(self, angle, **kw): self.q = np.array(angle[:6], float); self.sent.append(self.q); return 0
    def vc_set_joint_velocity(self, v, **kw): self.sent.append(np.array(v[:6])); return 0
    def disconnect(self): pass


class RobotLoop(unittest.TestCase):
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
            robot._start(); robot._tick(); robot._tick()
            np.testing.assert_allclose(robot.arm.sent[-1], robot.blocks['mid_left'])
            self.assertEqual(len(robot.arm.sent), 1)  # sent once, not every tick
            state.blocker.zone = 'high'; state.blocker.velocity = np.array([2., 0, 0]); robot._tick()  # fast swing
            np.testing.assert_allclose(robot.arm.sent[-1], robot.blocks['high'])  # re-targets immediately
            robot._halt('Operator Stop')
            self.assertEqual(robot.arm.states[-1], 4); self.assertFalse(robot.active)

    def _attacker(self, tmp):
        state, robot = self._robot(tmp)
        robot.attacks = {**CFG['attacks'], 'combo': [1, 1], 'moves': {'chop': CFG['attacks']['moves']['chop']}}
        robot._start(); robot.next_attack_at = 0
        return state, robot

    def test_attack_winds_up_strikes_then_blocks_again(self):
        with tempfile.TemporaryDirectory() as tmp:
            state, robot = self._attacker(tmp)
            chop = CFG['attacks']['moves']['chop']
            robot._tick(); np.testing.assert_allclose(robot.arm.sent[-1], chop['windup'])
            robot.arm.q = np.array(chop['windup']); robot._tick()
            np.testing.assert_allclose(robot.arm.sent[-1], chop['strike'])
            robot.arm.q = np.array(chop['strike']); robot._tick()
            self.assertIsNone(robot.attack)
            np.testing.assert_allclose(robot.arm.sent[-1], robot.blocks['guard'])  # straight back to blocking
            self.assertGreater(robot.next_attack_at, time.monotonic())  # next one a few seconds later
            robot._halt('done')

    def test_combo_chains_strikes_even_while_opponent_swings(self):
        with tempfile.TemporaryDirectory() as tmp:
            state, robot = self._attacker(tmp)
            moves = CFG['attacks']['moves']; robot.attacks = {**robot.attacks, 'combo': [2, 2], 'moves': moves}
            state.blocker.zone = 'high'; state.blocker.velocity = np.array([3., 0, 0])  # opponent mid-swing
            robot._tick(); first = robot.attack['name']
            robot.arm.q = np.array(moves[first]['windup']); robot._tick()
            robot.arm.q = np.array(moves[first]['strike']); robot._tick()
            second = robot.attack['name']
            self.assertEqual(robot.attack['phase'], 'windup')  # chained straight into strike two
            np.testing.assert_allclose(robot.arm.sent[-1], moves[second]['windup'])
            robot.arm.q = np.array(moves[second]['windup']); robot._tick()
            robot.arm.q = np.array(moves[second]['strike']); robot._tick()
            self.assertIsNone(robot.attack)  # combo over: back to blocking
            robot._halt('done')

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
