"""Offline checks. Never connects to a robot or camera. Run: python -m unittest discover checks"""
from pathlib import Path
import tempfile
import time
import tomllib
import unittest
import numpy as np
from aiohttp.test_utils import TestClient, TestServer
from sword_app.__main__ import create_app
from sword_app.math3d import from_rpy, mirrored_pose, source_basis, to_rpy
from sword_app.robot import Robot, velocity_step
from sword_app.state import State

ROOT = Path(__file__).resolve().parents[1]
CFG = tomllib.loads((ROOT/'config.toml').read_text())


class FakeArm:
    connected = True; error_code = 0; version = 'fake'

    def __init__(self): self.q = np.array(CFG['robot']['guard_deg']); self.sent = []; self.states = []
    def clean_warn(self): return 0
    def clean_error(self): return 0
    def motion_enable(self, enable): return 0
    def set_mode(self, mode): return 0
    def set_state(self, state): self.states.append(state); return 0
    def get_servo_angle(self, is_radian=False): return 0, [*self.q, 0.]
    def get_forward_kinematics(self, joints, **kw): return 0, [300., 0., 400., 180., 0., 0.]
    def get_inverse_kinematics(self, pose, **kw): return 0, [*(self.q + 5), 0.]
    def vc_set_joint_velocity(self, v, **kw): self.sent.append(v[:6]); return 0
    def disconnect(self): pass


class Core(unittest.TestCase):
    def test_rpy_roundtrip(self):
        for angles in [[10, 20, 30], [-170, 89, 170], [30, -90, 0]]:
            np.testing.assert_allclose(from_rpy(to_rpy(from_rpy(angles))), from_rpy(angles), atol=1e-7)

    def test_mirrored_pose_applies_gain(self):
        neutral = {'p': np.zeros(3), 'r': np.eye(3), 'basis': source_basis('hand')}
        result = mirrored_pose({'p': np.array([.1, .2, -.3]), 'r': np.eye(3)}, neutral, [500, 0, 500, 0, 0, 0], .5)
        np.testing.assert_allclose(result[:3], [650, 50, 600])

    def test_velocity_moves_toward_target_within_caps(self):
        cfg = CFG['robot']; target = np.array([10, -10, 10, -10, 10, -10.])
        v = velocity_step(target, np.zeros(6), np.zeros(6), .01, cfg)
        self.assertTrue(np.all(np.sign(v) == np.sign(target)))
        v = velocity_step(target*100, np.zeros(6), np.zeros(6), 10, cfg)
        np.testing.assert_allclose(np.abs(v), cfg['max_speed_deg_s'])

    def test_state_guard_and_timeout(self):
        state = State(.5); pose = {'p': [0, 0, -.5], 'r': np.eye(3), 'kind': 'hand'}
        state.update(pose, 'a', 10)
        self.assertIsNotNone(state.current(10.4)); self.assertIsNone(state.current(10.6))
        state.update(pose, 'a'); state.set_guard(); self.assertIsNotNone(state.neutral)
        state.update(pose, 'b'); self.assertIsNone(state.neutral)

    def test_lost_marker_continues_on_recent_motion_then_holds(self):
        state = State(.5, .3)
        for i in range(5): state.update({'p': [.1*i/30, 0, 0], 'r': np.eye(3), 'kind': 'hand'}, 'a', 10+i/30)
        state.neutral = {'p': np.zeros(3), 'r': np.eye(3), 'basis': np.eye(3)}
        last = 10+4/30
        moving, _ = state.target_source(last+.2); held, _ = state.target_source(last+5)
        self.assertGreater(moving['p'][0], .4/30)  # still moving at ~0.1 m/s after the marker vanished
        np.testing.assert_allclose(held['p'][0], .4/30+.1*.3, atol=.003)  # stops after prediction_s
        self.assertIsNone(state.current(last+5))  # not live, but the robot still has a target

    def test_robot_follows_tracking_and_stops(self):
        state = State(.5); state.update({'p': [0, 0, 0], 'r': np.eye(3), 'kind': 'hand'}, 'a'); state.set_guard()
        with tempfile.TemporaryDirectory() as tmp:
            robot = Robot(state, CFG['robot'], tmp); robot.arm = FakeArm()
            robot._start()
            for _ in range(3): time.sleep(.01); robot._tick()
            self.assertTrue(np.all(robot.arm.sent[-1] > 0))
            self.assertEqual(state.robot['state'], 'running')
            robot._halt('Operator Stop')
            self.assertEqual(robot.arm.states[-1], 4); self.assertFalse(robot.active)


class Server(unittest.IsolatedAsyncioTestCase):
    async def test_state_and_token(self):
        async with TestClient(TestServer(create_app())) as client:
            data = await (await client.get('/api/state')).json()
            self.assertEqual((await client.post('/api/action', json={'action': 'stop'})).status, 403)
            ok = await client.post('/api/action', json={'action': 'stop'}, headers={'X-Sword-Key': data['key']})
            self.assertEqual(ok.status, 200)


if __name__ == '__main__': unittest.main()
