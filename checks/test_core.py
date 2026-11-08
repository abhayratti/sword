"""Small offline contract suite. It never connects to a robot or camera."""
import asyncio
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import tomllib
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from aiohttp.test_utils import TestClient, TestServer
from sword_app.__main__ import create_app
from sword_app.math3d import from_quaternion, from_rpy, mirrored_pose, source_basis, to_rpy
from sword_app.robot import Robot, velocity_step
from sword_app.state import State
from sword_app.vision import Vision

ROOT=Path(__file__).resolve().parents[1]
CFG=tomllib.loads((ROOT/'config.toml').read_text())


class Contracts(unittest.TestCase):
    def test_orientation_roundtrip_and_full_roll_are_preserved(self):
        for angles in [[10,20,30],[-170,89,170],[30,-90,0]]:
            r=from_rpy(angles);np.testing.assert_allclose(from_rpy(to_rpy(r)),r,atol=1e-7)
        neutral={'p':np.zeros(3),'r':np.eye(3),'basis':source_basis('hand')}
        source={'p':np.array([.1,.2,-.3]),'r':from_rpy([30,20,10])}
        result=mirrored_pose(source,neutral,[500,0,500,0,0,0],.5)
        np.testing.assert_allclose(result[:3],[650,50,600])
        self.assertAlmostEqual(np.linalg.det(from_rpy(result[3:])),1)
        self.assertGreater(np.linalg.norm(result[3:]),20)

    def test_stale_tracking_and_reference_change_clear_admission(self):
        state=State();pose={'p':[0,0,-.5],'r':np.eye(3),'kind':'hand'}
        state.update(pose,'first',10)
        self.assertIsNotNone(state.current(10.1));self.assertIsNone(state.current(10.16))
        with patch('sword_app.state.time.monotonic',return_value=10.3):
            for at in [10.1,10.2,10.3]:state.update(pose,'first',at)
            state.set_guard()
            self.assertIsNotNone(state.neutral)
            state.update(pose,'second',10.31);self.assertIsNone(state.neutral)
            self.assertEqual(len(state.history),1)

    def test_nonfinite_or_nonrigid_pose_is_rejected(self):
        state=State()
        for p,r in [([float('nan'),0,0],np.eye(3)),([0,0,0],np.diag([1,1,-1]))]:
            with self.assertRaises(ValueError):state.update({'p':p,'r':r,'kind':'hand'},'space')
        with self.assertRaises(ValueError):from_quaternion([0,0,0,0])

    def test_all_six_joints_respond_with_bounded_stopping_distance(self):
        cfg=CFG['robot'];q=np.zeros(6);target=np.array([10,-10,10,-10,10,-10]);lo=q-20;hi=q+20
        v=velocity_step(target,q,q,.02,cfg,lo,hi)
        self.assertTrue(np.all(v!=0));self.assertTrue(np.all(np.sign(v)==np.sign(target)))
        near=hi-.001;v=velocity_step(hi,near,np.ones(6)*60,.02,cfg,lo,hi)
        stop=near+v*cfg['velocity_lease_s']+v*v/(2*np.array(cfg['max_accel_deg_s2']))
        self.assertTrue(np.all(stop<=hi+1e-9))
        np.testing.assert_array_equal(velocity_step(target,q,q,.5,cfg,lo,hi),q)

    def test_read_only_connection_does_not_stop_someone_elses_motion(self):
        class FakeArm:
            connected=True
            def set_state(self,value): self.calls.append(value);return 0
            calls=[]
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()))
        robot.arm=FakeArm();robot._halt('Read-only exit');self.assertEqual(robot.arm.calls,[])
        robot.motion_owned=True;robot._halt('Owned motion');self.assertEqual(robot.arm.calls,[4])

    def test_stop_invalidates_queued_starts(self):
        state=State();generation=state.generation;state.stop();self.assertNotEqual(generation,state.generation)
        robot=Robot(state,CFG['robot'],Path(tempfile.gettempdir()))
        robot.jobs.put(('home',state.generation));robot._halt('Controller fault')
        self.assertNotEqual(robot.jobs.get()[1],state.generation)

    def test_reset_during_solver_rejects_pre_reset_target(self):
        state=State();robot=Robot(state,CFG['robot'],Path(tempfile.gettempdir()))
        pose={'p':[0,0,-.5],'r':np.eye(3),'kind':'hand'}
        with patch('sword_app.state.time.monotonic',return_value=10.3), patch('sword_app.robot.time.monotonic',return_value=10.3):
            for at in [10.1,10.2,10.3]:state.update(pose,'first',at)
            state.set_guard()
            source=state.current();self.assertTrue(robot._source_current(source))
            state.invalidate('Tracking origin changed',reset=True)
            self.assertFalse(robot._source_current(source))
            for at in [10.1,10.2,10.3]:state.update(pose,'second',at)
            state.set_guard()
            self.assertFalse(robot._source_current(source))

    def test_braking_forecast_keeps_actual_momentum_independent(self):
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()))
        robot._admissible=lambda q:bool(np.max(q)<=1)
        q=np.array([.99,0,0,0,0,0]);v=np.array([1.,0,0,0,0,0])
        self.assertFalse(robot._stopping_path_admitted(q,v,.12))
        self.assertTrue(robot._stopping_path_admitted(q,v,0))
        q[0]=.9999;self.assertFalse(robot._stopping_path_admitted(q,v,0))

    def test_guard_tolerates_brief_detection_gaps_without_reusing_stale_input(self):
        state=State();pose={'p':[0,0,-.5],'r':np.eye(3),'kind':'markers'}
        with patch('sword_app.state.time.monotonic',return_value=10.8):
            for at in [10.2,10.5,10.8]:
                state.invalidate('Marker briefly missing');state.update(pose,'fixed-board',at)
            state.set_guard();self.assertIsNotNone(state.neutral)
            state.invalidate('Marker missing')
            self.assertIsNone(state.target_source()[0]);self.assertIsNotNone(state.neutral)
            state.update(pose,'fixed-board',10.8)
            self.assertIsNotNone(state.target_source()[0])
        with patch('sword_app.state.time.monotonic',return_value=10.96):
            self.assertIsNone(state.target_source()[0])
        state.invalidate('Reference changed',reset=True)
        self.assertIsNone(state.neutral);self.assertEqual(len(state.history),0)

    def test_detection_blink_preserves_only_original_measurement_lifetime(self):
        state=State();pose={'p':[0,0,-.5],'r':np.eye(3),'kind':'markers'}
        with patch('sword_app.state.time.monotonic',return_value=10.3):
            for at in [10.1,10.2,10.3]:state.update(pose,'fixed-board',at)
            state.set_guard()
        with patch('sword_app.state.time.monotonic',return_value=10.4):
            state.miss_frame('Fixed reference not fully visible','fixed-board')
            source,neutral=state.target_source()
            self.assertEqual(source['at'],10.3);self.assertIsNotNone(neutral)
            self.assertEqual(state.frames,3)
        with patch('sword_app.state.time.monotonic',return_value=10.46):
            state.miss_frame('Fixed reference not fully visible','fixed-board')
            self.assertIsNone(state.target_source()[0]);self.assertIsNotNone(state.neutral)
            state.update(pose,'fixed-board',10.46)
            self.assertIsNotNone(state.target_source()[0])
            state.miss_frame('Missing after reference reset','new-space')
            self.assertIsNone(state.target_source()[0]);self.assertIsNone(state.neutral)

    def test_neutral_deadband_ignores_small_jitter_and_preserves_larger_response(self):
        neutral={'p':np.zeros(3),'r':np.eye(3),'basis':np.eye(3)};home=[500,0,400,0,0,0]
        small={'p':np.array([.004,0,0]),'r':from_rpy([0,0,1])}
        np.testing.assert_allclose(mirrored_pose(small,neutral,home,.35,.65,.008,2),home,atol=1e-6)
        larger={'p':np.array([.108,0,0]),'r':from_rpy([0,0,12])}
        result=mirrored_pose(larger,neutral,home,.35,.65,.008,2)
        np.testing.assert_allclose(result,[535,0,400,0,0,6.5],atol=1e-6)

    def test_height_projection_keeps_other_axes_and_the_modeled_blade_above_floor(self):
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()))
        robot.xyz_low=np.array([-1000.,-1000.,0.]);robot.xyz_high=np.array([1000.,1000.,1000.])
        robot.local_base=np.array([0.,0.,-40.]);robot.local_tip=np.array([300.,0.,-40.])
        requested=np.array([500.,20.,200.,0.,30.,0.]);result=robot._project_pose(requested)
        np.testing.assert_allclose(result[[0,1,3,4,5]],requested[[0,1,3,4,5]])
        r=from_rpy(result[3:]);points=[result[:3],result[:3]+r@robot.local_base,result[:3]+r@robot.local_tip]
        self.assertGreaterEqual(min(p[2] for p in points),CFG['robot']['minimum_height_mm']+.999)
        robot.xyz_high[2]=result[2]-2
        self.assertIsNone(robot._project_pose(requested))


class HomeContracts(unittest.TestCase):
    def test_only_a_warning_attributed_to_our_ik_query_is_nonfatal(self):
        class Arm:
            connected=True;mode=4;motor_enable_states=[1]*6
            faults=[0,14]
            _arm=SimpleNamespace(reported=True,_last_report_time=10.)
            def get_err_warn_code(self):return (0,self.faults)
            def get_state(self):return (0,2)
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()));robot.arm=Arm()
        with patch('sword_app.robot.time.monotonic',return_value=10.1):
            with self.assertRaises(RuntimeError):robot._status()
            robot.ik_no_solution_warning=True;robot._status()
            for faults in [[1,14],[0,11]]:
                robot.arm.faults=faults
                with self.assertRaises(RuntimeError):robot._status()
            robot.arm.faults=[0,0];robot._status();self.assertFalse(robot.ik_no_solution_warning)

    def test_unsolvable_native_ik_candidate_is_discarded_before_fallback(self):
        actual=np.array(CFG['robot']['guard_deg'])
        class Arm:
            def __init__(self):self.calls=0
            def get_inverse_kinematics(self,*a,**kw):
                self.calls+=1
                return (10,[]) if self.calls==1 else (0,[*(actual+1),0.])
            def get_err_warn_code(self):return (0,[0,14])
            def is_joint_limit(self,*a,**kw):return (0,False)
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()));robot.arm=Arm()
        robot._fk=lambda q:np.array([500.,0.,400.,0.,0.,0.]);robot._project_pose=lambda p:p
        robot._admissible=lambda q,**kwargs:True;robot.low=actual-10;robot.high=actual+10
        target,fraction=robot._solve(np.array([510.,0.,400.,0.,0.,5.]),actual)
        np.testing.assert_allclose(target,actual+1);self.assertEqual(fraction,.5)
        self.assertTrue(robot.ik_no_solution_warning)

    def test_small_height_undershoot_can_home_only_inward_and_does_not_admit_low_targets(self):
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()))
        floor=CFG['robot']['minimum_height_mm']
        robot.low=np.full(6,-1000.);robot.high=np.full(6,1000.)
        robot.xyz_low=np.array([-1000.,-1000.,0.]);robot.xyz_high=np.full(3,1000.)
        robot._points=lambda q:np.array([[500.,0.,floor+q[0]]]*3)
        origin=np.array([-.13,0.,0.,0.,0.,0.]);home=np.array([3.1,0.,0.,0.,0.,0.])
        self.assertTrue(robot._home_path_admitted(origin,home))
        self.assertFalse(robot._home_path_admitted(origin,np.array([-.3,0.,0.,0.,0.,0.])))
        self.assertFalse(robot._admissible(np.array([-.51,0.,0.,0.,0.,0.])))
        self.assertFalse(robot._admissible(origin,floor_margin_mm=CFG['robot']['target_height_clearance_mm']))
        self.assertTrue(robot._admissible(home,floor_margin_mm=CFG['robot']['target_height_clearance_mm']))

    def test_home_checks_use_commanded_origin_and_restore_planner_without_moving(self):
        actual=np.array(CFG['robot']['guard_deg']);commanded=actual+np.array([0,.01,0,0,0,0])
        class Arm:
            only_check_result=0
            def __init__(self):
                self.calls=[];self._arm=SimpleNamespace(arm_cmd=SimpleNamespace(get_joint_pos=lambda:[0,*np.radians(commanded),0.]))
            def set_servo_angle(self,**kw):self.calls.append(kw);return 0
            def get_cmdnum(self):return (0,0)
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()));robot.arm=Arm();robot._actual=lambda:actual.copy()
        target=actual+np.array([1,2,3,4,5,6]);delta=robot._checked_home_delta(target,actual)
        np.testing.assert_allclose(delta,target-commanded)
        self.assertEqual([x['only_check_type'] for x in robot.arm.calls],[1,2])
        np.testing.assert_allclose(robot.arm.calls[1]['angle'],-np.array(robot.arm.calls[0]['angle']))
        self.assertTrue(all(x['relative'] and x['mvtime']==0 for x in robot.arm.calls))

    def test_home_rejection_preserves_specific_result_and_never_executes(self):
        actual=np.array(CFG['robot']['guard_deg'])
        class Arm:
            only_check_result=24
            def __init__(self):
                self.calls=[];self._arm=SimpleNamespace(arm_cmd=SimpleNamespace(get_joint_pos=lambda:[0,*np.radians(actual),0.]))
            def set_servo_angle(self,**kw):self.calls.append(kw);return 1
        robot=Robot(State(),CFG['robot'],Path(tempfile.gettempdir()));robot.arm=Arm()
        with self.assertRaisesRegex(RuntimeError,'result 24'):
            robot._checked_home_delta(actual+1,actual)
        self.assertEqual([x['only_check_type'] for x in robot.arm.calls],[1])


class VisionContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.vision=Vision({**CFG['vision'],'reference':'marker'},self.tmp.name)
        self.k=np.array([[900.,0,640],[0,900,480],[0,0,1.]])
        self.d=np.zeros(5)

    def test_known_marker_pose_recovers_metric_position_and_orientation(self):
        self.vision._camera('known',(1280,960));self.vision.calibration={'matrix':self.k.tolist(),'distortion':self.d.tolist()}
        half=.025;obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
        r=from_rpy([165,10,4]);p=np.array([.1,.02,.6])
        corners=cv2.projectPoints(obj,cv2.Rodrigues(r)[0],p,self.k,self.d)[0].reshape(4,2)
        pose,reason=self.vision._pose(corners)
        self.assertIsNone(reason);np.testing.assert_allclose(pose['p'],p,atol=1e-5)
        np.testing.assert_allclose(pose['r'],r,atol=1e-4)

    def test_marker_detection_never_fabricates_depth_without_calibration(self):
        image=np.full((960,1280,3),255,np.uint8)
        for i,(ident,x) in enumerate([(10,100),(20,500),(30,900)]):
            marker=cv2.aruco.generateImageMarker(self.vision.dictionary,ident,180)
            image[350:530,x:x+180]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        jpeg=cv2.imencode('.jpg',image)[1].tobytes()
        result,pose=self.vision.process(jpeg,'camera')
        self.assertEqual(sorted(x['id'] for x in result['objects']),[10,20,30])
        self.assertIsNone(pose);self.assertFalse(result['camera']['calibrated'])
        self.assertTrue(all(x['pose'] is None for x in result['objects']))

    def test_recent_orientation_resolves_only_a_matching_ambiguous_image_pose(self):
        self.vision._camera('known',(1280,960))
        self.vision.calibration={'matrix':self.k.tolist(),'distortion':self.d.tolist()}
        h=.025;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
        r=from_rpy([170,5,4]);p=np.array([.02,.02,1.1])
        corners=cv2.projectPoints(obj,cv2.Rodrigues(r)[0],p,self.k,self.d)[0].reshape(4,2)
        pose,reason=self.vision._pose(corners)
        self.assertIsNone(pose);self.assertIn('ambiguous',reason)
        pose,reason=self.vision._pose(corners,{'p':p,'r':r})
        self.assertIsNone(reason);self.assertEqual(pose['orientation_source'],'recent_pose_and_image')
        np.testing.assert_allclose(pose['p'],p,atol=1e-5)
        np.testing.assert_allclose(pose['r'],r,atol=1e-4)
        pose,reason=self.vision._pose(corners,{'p':p,'r':from_rpy([90,0,0])})
        self.assertIsNone(pose);self.assertIn('ambiguous',reason)

    def test_orientation_history_follows_current_reference_and_expires(self):
        self.vision._camera('known',(1280,960))
        previous={'p':np.array([.3,.1,.2]),'r':from_rpy([10,20,30]),'at':10.}
        self.vision.pose_history['human']=previous
        anchor={'p':np.array([.2,-.3,1.]),'r':from_rpy([-15,40,20])}
        prior=self.vision._prediction('human',anchor,10.05)
        np.testing.assert_allclose(prior['r'],anchor['r']@previous['r'])
        np.testing.assert_allclose(prior['p'],anchor['r']@previous['p']+anchor['p'])
        self.assertIsNone(self.vision._prediction('human',None,10.05))
        self.assertIsNone(self.vision._prediction('human',anchor,10.16))
        self.assertIsNone(self.vision._prediction('human',anchor,9.9))
        self.vision._camera('another',(1280,960))
        self.assertIsNone(self.vision._prediction('human',anchor,10.05))

    def test_camera_or_resolution_change_invalidates_calibration(self):
        self.vision._camera('one',(1280,960));self.vision.calibration={'matrix':self.k.tolist(),'distortion':self.d.tolist()}
        self.vision._camera('two',(1280,960));self.assertIsNone(self.vision.calibration)
        self.vision.calibration={};self.vision._camera('two',(640,480));self.assertIsNone(self.vision.calibration)

    def test_duplicate_calibration_view_is_not_extra_evidence(self):
        self.vision.last_board=(np.zeros((54,1,2),np.float32),np.zeros(6))
        self.vision.add_calibration_view()
        with self.assertRaisesRegex(ValueError,'Move or tilt'):self.vision.add_calibration_view()
        with self.assertRaisesRegex(ValueError,'12 distinct'):self.vision.solve_calibration()

    def test_calibration_recovers_known_lens_and_holds_out_three_views(self):
        self.vision._camera('known',(1280,960))
        obj=np.zeros((54,3),np.float32);obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.025
        for i in range(12):
            rv=np.array([-.3+.07*i,.25*np.sin(i),.04*i])
            tv=np.array([-.14+.02*(i%3),-.1+.02*(i%4),.6+.035*i])
            self.vision.views.append(cv2.projectPoints(obj,rv,tv,self.k,self.d)[0])
        result=self.vision.solve_calibration()
        self.assertTrue(result['calibrated'])
        np.testing.assert_allclose(self.vision.calibration['matrix'],self.k,atol=.05)
        self.assertEqual(self.vision.calibration['training_views'],9)
        self.assertEqual(self.vision.calibration['validation_views'],3)
        self.assertLess(max(result['validation_px']),.01)

    def test_failed_camera_fit_preserves_observations_without_installing(self):
        self.vision._camera('known',(1280,960))
        obj=np.zeros((54,3),np.float32);obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.025
        for i in range(12):
            rv=np.array([-.3+.07*i,.25*np.sin(i),.04*i]);tv=np.array([-.1,-.05,.6+.035*i])
            self.vision.views.append(cv2.projectPoints(obj,rv,tv,self.k,self.d)[0])
        self.vision.views[-1]+=np.random.default_rng(42).normal(0,5,(54,1,2)).astype(np.float32)
        with self.assertRaisesRegex(ValueError,'Captured corners are saved'):
            self.vision.solve_calibration()
        self.assertIsNone(self.vision.calibration)
        self.assertFalse((Path(self.tmp.name)/'camera.json').exists())
        attempt=json.loads((Path(self.tmp.name)/'camera-attempt.json').read_text())
        self.assertEqual(attempt['status'],'rejected');self.assertEqual(len(attempt['corners']),12)
        self.assertGreater(max(attempt['validation_px']),1.5)

    def test_large_reference_preserves_pose_through_detector_order_reversals(self):
        self.vision._camera('known',(1280,960))
        self.vision.calibration={'matrix':self.k.tolist(),'distortion':self.d.tolist()}
        pattern=np.full((360,480),255,np.uint8)
        for row in range(7):
            for col in range(10):
                if (row+col)%2==0:pattern[40+row*40:80+row*40,40+col*40:80+col*40]=0
        outer=np.array([[-.125,.0875,0],[.125,.0875,0],[.125,-.0875,0],[-.125,-.0875,0]],np.float32)
        source=np.array([[40,40],[440,40],[440,320],[40,320]],np.float32)
        for roll in [0,90,180]:
            r=from_rpy([175,12,roll]);p=np.array([.02,-.03,.75])
            dest=cv2.projectPoints(outer,cv2.Rodrigues(r)[0],p,self.k,self.d)[0].reshape(4,2)
            gray=cv2.warpPerspective(pattern,cv2.getPerspectiveTransform(source,dest),(1280,960),borderValue=255)
            item,pose=self.vision._board_reference(gray)
            self.assertIsNotNone(pose,item)
            np.testing.assert_allclose(pose['p'],p,atol=.003)
            self.assertLess(np.linalg.norm(cv2.Rodrigues(pose['r']@r.T)[0]),np.radians(2))

    def test_large_reference_mode_never_silently_switches_to_marker_30(self):
        self.vision.cfg={**self.vision.cfg,'reference':'checkerboard'}
        self.vision._camera('known',(1280,960))
        self.vision.calibration={'matrix':self.k.tolist(),'distortion':self.d.tolist()}
        image=np.full((960,1280,3),255,np.uint8)
        for ident,x in [(10,100),(20,500),(30,900)]:
            marker=cv2.aruco.generateImageMarker(self.vision.dictionary,ident,180)
            image[350:530,x:x+180]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        result,pose=self.vision.process(cv2.imencode('.jpg',image)[1].tobytes(),'known')
        self.assertIsNone(pose)
        self.assertEqual(sorted(o['id'] for o in result['objects']),[10,20])


class ServerContracts(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.app=create_app();self.client=TestClient(TestServer(self.app));await self.client.start_server()
        self.addAsyncCleanup(self.client.close)
        self.initial=await (await self.client.get('/api/state')).json();self.key=self.initial['key']

    async def test_startup_never_connects_or_enables_a_robot(self):
        self.assertIsNone(self.app['robot'].arm)
        self.assertEqual(self.initial['robot']['state'],'disconnected')
        self.assertIsNone(self.initial['session_timer_s'])

    async def test_actions_require_same_origin_token(self):
        self.assertEqual((await self.client.post('/api/action',json={'action':'stop'})).status,403)
        response=await self.client.post('/api/action',json={'action':'stop'},headers={'X-Sword-Key':self.key,'Origin':'https://example.invalid'})
        self.assertEqual(response.status,403)
        response=await self.client.post('/api/action',json={'action':'stop'},headers={'X-Sword-Key':self.key})
        self.assertEqual(response.status,200)

    async def test_unready_start_never_reaches_hardware(self):
        response=await self.client.post('/api/action',json={'action':'start'},headers={'X-Sword-Key':self.key})
        self.assertEqual(response.status,400);self.assertIsNone(self.app['robot'].arm)

    async def test_stream_nonce_expiry_and_disconnect_invalidate_pose(self):
        ws=await self.client.ws_connect('/tracking?key='+self.key);first=await ws.receive_json()
        packet={'kind':'hand','valid':True,'epoch':'reference','position':[0,0,-.5],
                'orientation':[0,0,0,1],'head_orientation':[0,0,0,1],'capture_age_ms':0,'captured_after_challenge':True}
        await ws.send_json({**packet,'nonce':first['nonce']});reply=await ws.receive_json()
        self.assertIsNone(reply['error']);self.assertTrue(reply['tracking']['fresh'])
        await asyncio.sleep(.17);await ws.send_json({**packet,'nonce':reply['nonce']});reply=await ws.receive_json()
        self.assertIsNotNone(reply['error']);self.assertFalse(reply['tracking']['fresh'])
        await ws.send_json({**packet,'nonce':reply['nonce']});reply=await ws.receive_json()
        self.assertTrue(reply['tracking']['fresh']);await ws.close();await asyncio.sleep(.02)
        self.assertIsNone(self.app['state'].current())

    async def test_capture_delay_is_not_counted_twice_or_freshened_on_receipt(self):
        ws=await self.client.ws_connect('/tracking?key='+self.key);first=await ws.receive_json()
        received_at=time.monotonic()
        await asyncio.sleep(.06)
        await ws.send_json({'kind':'hand','valid':True,'epoch':'reference','position':[0,0,-.5],
                           'orientation':[0,0,0,1],'capture_age_ms':30,'captured_after_challenge':True,'nonce':first['nonce']})
        reply=await ws.receive_json()
        self.assertIsNone(reply['error']);self.assertGreaterEqual(reply['tracking']['age_ms'],55)
        self.assertLessEqual(reply['tracking']['age_ms'],(time.monotonic()-received_at)*1000+10)
        await ws.close()

    async def test_sample_cannot_predate_the_capture_challenge(self):
        ws=await self.client.ws_connect('/tracking?key='+self.key);first=await ws.receive_json()
        await ws.send_json({'kind':'hand','valid':True,'epoch':'reference','position':[0,0,-.5],
                           'orientation':[0,0,0,1],'capture_age_ms':90,'captured_after_challenge':True,'nonce':first['nonce']})
        reply=await ws.receive_json()
        self.assertIsNotNone(reply['error']);self.assertFalse(reply['tracking']['fresh'])
        await ws.close()


if __name__=='__main__':unittest.main()
