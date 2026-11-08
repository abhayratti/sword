"""One robot writer, native IK, finite velocity leases, and an explicit Stop.

This is non-contact mirroring. It is not a collision-aware sword-fighting policy.
"""
import json
from pathlib import Path
import queue
import threading
import time
import numpy as np
from .math3d import from_rpy, interpolate_rotation, mirrored_pose, to_rpy, vector


def velocity_step(target, actual, previous, dt, cfg, low, high):
    """Independent joint response with target braking and no integral windup."""
    q, goal, prev = vector(actual,6), vector(target,6), vector(previous,6)
    if not 0 < dt <= cfg['velocity_lease_s']: return np.zeros(6)
    accel = np.array(cfg['max_accel_deg_s2'])
    cap = np.array(cfg['max_speed_deg_s'])
    error = goal-q
    desired = np.sign(error)*np.minimum(np.abs(error)*cfg['kp'],np.sqrt(1.3*accel*np.abs(error)))
    v = np.clip(prev+np.clip(desired-prev,-accel*dt,accel*dt),-cap,cap)
    # Reserve a full hardware lease plus braking before each joint limit.
    room = np.where(v>=0,high-q,q-low)
    lease = cfg['velocity_lease_s']
    stopping_cap = np.maximum(0,-accel*lease+np.sqrt((accel*lease)**2+2*accel*np.maximum(0,room)))
    return np.sign(v)*np.minimum(np.abs(v),stopping_cap)


class Robot:
    def __init__(self, state, config, storage):
        self.state, self.cfg, self.storage = state, config, Path(storage)
        self.jobs = queue.Queue(maxsize=1)
        self.closed = threading.Event(); self.stop_requested = threading.Event()
        self.arm = None; self.active = False; self.log = None
        self.motion_owned = False; self.braking = False
        self.previous_velocity = np.zeros(6)
        self.last_tick = None; self.previous_actual = None
        self.previous_actual_at = None
        self.last_idle_feedback = 0.
        self.ik_no_solution_warning = False
        for name in ('guard_deg','joint_radius_deg','max_speed_deg_s','max_accel_deg_s2','mount_reference_deg'):
            values=vector(config[name],6)
            if name not in {'guard_deg','mount_reference_deg'} and np.any(values<=0):
                raise ValueError('Motion limits must be positive')
        if not 0 < config['velocity_lease_s'] <= .12 or not 0 < config['tracking_timeout_s'] <= .15:
            raise ValueError('Invalid motion expiry')
        if not 0<config['position_gain']<=1 or not 0<config['rotation_gain']<=1:
            raise ValueError('Response gains must be between zero and one')
        self.thread = threading.Thread(target=self._run,name='robot-writer',daemon=True)

    def start_worker(self): self.thread.start()

    def request(self, operation):
        if operation not in {'connect','home','start','disconnect'}: raise ValueError('Unknown robot operation')
        if operation == 'start' and self.state.target_source()[0] is None:
            raise ValueError('Track an object and set the guard first')
        if self.active or self.motion_owned: raise ValueError('Press Stop before changing the robot setup')
        try: self.jobs.put_nowait((operation,self.state.generation))
        except queue.Full: raise ValueError('A robot command is already pending')

    def stop(self):
        self.state.stop(); self.stop_requested.set()

    def close(self):
        self.stop(); self.closed.set(); self.thread.join(timeout=4)

    def _publish(self, status, reason, **values):
        with self.state.lock:
            self.state.robot.update(state=status,reason=reason,**values)

    def _record(self, event, **values):
        if self.log:
            self.log.write(json.dumps({'at':time.monotonic(),'event':event,**values},allow_nan=False)+'\n')
            self.log.flush()

    def _check(self, code, name):
        if code != 0: raise RuntimeError(f'{name} rejected by controller ({code})')

    def _cancelled(self):
        if self.stop_requested.is_set() or self.closed.is_set(): raise RuntimeError('Operator Stop')

    def _connect(self):
        if self.arm: self.arm.disconnect(); self.arm = None
        self.ik_no_solution_warning=False
        from xarm.wrapper import XArmAPI
        arm = XArmAPI(self.cfg['ip'],is_radian=False,do_not_open=True,enable_report=True,timed_comm=False)
        # The SDK may try to enable/clear during initial connection. This app
        # never substitutes for the physical operator's fault/reset decision.
        def forbidden(*args,**kwargs): raise RuntimeError('Enable or clear the controller in Studio')
        arm._arm.clean_warn = arm._arm.clean_error = arm._arm.motion_enable = forbidden
        self.arm = arm; arm.connect(); deadline = time.monotonic()+4
        while not arm._arm.reported or any(v not in (0,1) for v in arm.motor_enable_states[:6]):
            self._cancelled()
            if time.monotonic()>deadline: raise RuntimeError('Robot report connection timed out')
            time.sleep(.02)
        if arm.axis != 6 or list(arm.version_number) != self.cfg['firmware'] or arm.is_simulation_robot:
            raise RuntimeError('Expected real xArm 6 with the configured firmware')
        arm.set_timeout(.08)
        self._status()
        if max(self.cfg['max_speed_deg_s'])>arm.joint_speed_limit[1] or max(self.cfg['max_accel_deg_s2'])>arm.joint_acc_limit[1]:
            raise RuntimeError('Configured motion profile exceeds native limits')
        if np.max(np.abs(arm.tcp_offset))>.001 or np.max(np.abs(arm.world_offset))>.001:
            raise RuntimeError('Expected the verified zero TCP/world offsets')
        code, native = arm.get_reduced_states(is_radian=False); self._check(code,'Workspace query')
        if not isinstance(native,list) or len(native)!=7: raise RuntimeError('Invalid native workspace')
        xyz = vector(native[1],6); joint = vector(native[4],14).reshape(7,2)[:6]
        self.xyz_low,self.xyz_high = xyz[[1,3,5]],xyz[[0,2,4]]
        if np.any(self.xyz_low >= self.xyz_high) or np.any(joint[:,0]>=joint[:,1]): raise RuntimeError('Invalid native limits')
        guard,radius = vector(self.cfg['guard_deg'],6),vector(self.cfg['joint_radius_deg'],6)
        self.low,self.high = np.maximum(guard-radius,joint[:,0]),np.minimum(guard+radius,joint[:,1])
        reference = self._fk(self.cfg['mount_reference_deg'])
        self.local_base = from_rpy(reference[3:]).T @ np.array(self.cfg['mount_base_world_mm'])
        self.local_tip = self.local_base + from_rpy(reference[3:]).T @ np.array([self.cfg['blade_length_mm'],0.,0.])
        self.home_pose = self._fk(guard)
        q = self._actual()
        self._publish('connected','Connected; motion is off',joints=q.tolist(),guard_deg=guard.tolist(),
                      speed_caps=self.cfg['max_speed_deg_s'],firmware=arm.version,hardware_verified=False,
                      feedback_at=time.monotonic())

    def _actual(self):
        began = time.monotonic(); code,groups = self.arm.get_joint_states(is_radian=False,num=1)
        self._check(code,'Encoder query')
        if time.monotonic()-began>.1 or not isinstance(groups,list) or len(groups)!=1: raise RuntimeError('Encoder feedback stale or invalid')
        q = vector(groups[0],7)
        if q[6] != 0: raise RuntimeError('Unexpected seventh axis')
        return q[:6]

    def _status(self):
        arm = self.arm
        if not arm.connected or not arm._arm.reported or time.monotonic()-arm._arm._last_report_time>.45:
            raise RuntimeError('Robot feedback disconnected')
        code,faults = arm.get_err_warn_code(); self._check(code,'Fault query')
        code,state = arm.get_state(); self._check(code,'State query')
        if faults==[0,0]:self.ik_no_solution_warning=False
        expected_warning=faults==[0,14] and self.ik_no_solution_warning
        if (faults != [0,0] and not expected_warning) or state not in {0,1,2,4} or arm.mode not in {0,4}:
            raise RuntimeError(f'Controller state {state}, mode {arm.mode}, faults {faults}')
        with self.state.lock:self.state.robot['planner_warning']=14 if expected_warning else None
        if arm.motor_enable_states[:6] != [1]*6: raise RuntimeError('Enable the arm in Studio')
        if self.active and state==4: raise RuntimeError('Controller stopped')

    def _fk(self, joints):
        code,pose = self.arm.get_forward_kinematics([*joints,0.],input_is_radian=False,return_is_radian=False)
        self._check(code,'Forward kinematics')
        return vector(pose,6)

    def _points(self, q):
        pose = self._fk(q); r = from_rpy(pose[3:])
        return np.array([pose[:3],pose[:3]+r@self.local_base,pose[:3]+r@self.local_tip])

    def _admissible(self, q, *, floor_margin_mm=None):
        if np.any(q<self.low) or np.any(q>self.high): return False
        pts = self._points(q)
        margin = -self.cfg['height_tolerance_mm'] if floor_margin_mm is None else floor_margin_mm
        return (np.min(pts[:,2]) >= self.cfg['minimum_height_mm']+margin and
                np.all(pts[0]>=self.xyz_low) and np.all(pts[0]<=self.xyz_high))

    def _home_path_admitted(self, origin, target):
        if not self._admissible(origin): return False
        floor=self.cfg['minimum_height_mm']
        previous=float(np.min(self._points(origin)[:,2]))
        for f in np.linspace(0,1,21):
            q=origin+f*(target-origin)
            if not self._admissible(q): return False
            height=float(np.min(self._points(q)[:,2]))
            # A sub-millimetre feedback undershoot can return inward, never
            # use the tolerance to plan a deeper excursion before homing.
            if height < min(previous,floor)-1e-5: return False
            previous=height
        return True

    def _velocity(self, v):
        self._cancelled()
        began = time.monotonic()
        self._check(self.arm.vc_set_joint_velocity([*v,0.],is_radian=False,is_sync=False,
                                                 duration=self.cfg['velocity_lease_s']),'Velocity')
        if time.monotonic()-began>.1: raise RuntimeError('Velocity acknowledgement timed out')

    def _halt(self, reason):
        self.state.stop()  # No queued action may outlive a stop or fault.
        self.active = False; self.previous_velocity[:] = 0
        if self.motion_owned and self.arm and self.arm.connected:
            try: self._check(self.arm.set_state(4),'Stop')
            except Exception as exc: reason += '; stop acknowledgement unavailable: '+str(exc)
        self.motion_owned = False
        self._record('stop',reason=reason)
        if self.log: self.log.close(); self.log = None
        self._publish('stopped' if self.arm and self.arm.connected else 'disconnected',reason,command=[0.]*6)

    def _home(self):
        if not self.arm: raise ValueError('Connect the robot first')
        self._status(); q = self._actual(); target = np.array(self.cfg['guard_deg'])
        if not self._admissible(q): raise ValueError('Current pose needs repositioning in Studio')
        if self.arm.get_cmdnum() != (0,0): raise RuntimeError('Controller has queued motion')
        if not self._home_path_admitted(q,target): raise ValueError('Home path needs repositioning in Studio')
        self.motion_owned = True
        self._check(self.arm.set_mode(0),'Position mode'); self._check(self.arm.set_state(0),'Ready state')
        deadline = time.monotonic()+2
        while not self.arm._arm._is_sync or self.arm._arm._need_sync:
            self._cancelled()
            if time.monotonic()>deadline: raise RuntimeError('Planner did not synchronize')
            time.sleep(.01)
        # Measured-rest synchronization addresses the observed stale planner
        # reference. No automatic C24 bypass or fault clearing.
        self._check(self.arm.set_servo_angle(angle=[*q,0.],speed=1,mvacc=10,is_radian=False,wait=False),'Rest synchronization')
        q = self._settled_rest(q)
        delta = self._checked_home_delta(target,q)
        self._cancelled()
        self._check(self.arm.set_servo_angle(angle=[*delta,0.],relative=True,speed=4,mvacc=10,
                                            mvtime=0,radius=None,is_radian=False,wait=False,only_check_type=0),'Home move')
        deadline = time.monotonic()+np.max(np.abs(target-q))/4+4
        self._publish('homing','Returning to upright guard')
        settled = 0
        while time.monotonic()<deadline:
            self._cancelled(); self._status(); actual = self._actual()
            if not self._admissible(actual): raise RuntimeError('Home feedback left the workspace')
            self._publish('homing','Returning to upright guard',joints=actual.tolist())
            settled = settled+1 if np.max(np.abs(actual-target))<.08 else 0
            if settled>=4:
                self.motion_owned = False
                self._publish('connected','At upright guard; motion is off'); return
            time.sleep(.04)
        raise RuntimeError('Home move did not settle')

    def _settled_rest(self, origin):
        deadline=time.monotonic()+1.5;reference=None;stable_at=None
        while time.monotonic()<deadline:
            self._cancelled();self._status();actual=self._actual()
            if np.max(np.abs(actual-origin))>.03:raise RuntimeError('Unexpected movement during synchronization')
            code,status=self.arm.get_state();self._check(code,'Idle query')
            if status!=2 or self.arm.get_cmdnum()!=(0,0):reference=None
            elif reference is None or np.max(np.abs(actual-reference))>.005:
                reference=actual;stable_at=time.monotonic()
            elif time.monotonic()-stable_at>=.3:return actual
            time.sleep(.02)
        raise RuntimeError('Planner and encoders did not settle at rest')

    def _checked_home_delta(self, target, actual):
        # Relative moves start at the controller's commanded position, which
        # can differ slightly from the encoders after settling.
        raw=self.arm._arm.arm_cmd.get_joint_pos()
        if len(raw)!=8 or raw[0]!=0:raise RuntimeError('Invalid commanded home reference')
        commanded=np.degrees(vector(raw[1:],7))
        if commanded[6]!=0 or np.max(np.abs(commanded[:6]-actual))>.05:
            raise RuntimeError('Commanded and measured home references differ')
        delta=target-commanded[:6];checks=[]
        # Type 2 restores the checker's intermediate state. Neither check
        # executes motion; only the checked outward leg is executed below.
        for step,kind in ((delta,1),(-delta,2)):
            self._cancelled()
            code=self.arm.set_servo_angle(angle=[*step,0.],relative=True,speed=4,mvacc=10,
                                         mvtime=0,radius=None,is_radian=False,wait=False,only_check_type=kind)
            checks.append({'type':kind,'code':code,'result':self.arm.only_check_result})
            with self.state.lock:self.state.robot['home_checks']=checks.copy()
            if code or self.arm.only_check_result:
                raise RuntimeError(f'Home path check rejected: code {code}, result {self.arm.only_check_result}, type {kind}')
        if self.arm.get_cmdnum()!=(0,0) or np.max(np.abs(self._actual()-actual))>.02:
            raise RuntimeError('Home origin changed during the path check')
        return delta

    def _start(self):
        if not self.arm: raise ValueError('Connect the robot first')
        self._status(); q = self._actual()
        if np.max(np.abs(q-np.array(self.cfg['guard_deg'])))>.1: raise ValueError('Return to upright guard first')
        if not self._admissible(q): raise ValueError('Guard outside the admitted workspace')
        source,neutral = self.state.target_source()
        if source is None: raise ValueError('Fresh tracking and a set guard are required')
        self.motion_owned = True
        self._check(self.arm.set_mode(4),'Velocity mode'); self._check(self.arm.set_state(0),'Ready state')
        self._velocity(np.zeros(6))
        until = time.monotonic()+.25
        while time.monotonic()<until:
            self._cancelled()
            if np.max(np.abs(self._actual()-q))>.02: raise RuntimeError('Velocity entry moved unexpectedly')
            time.sleep(.02)
        self.home_pose = self._fk(q)
        self.previous_velocity = np.zeros(6); self.previous_actual = q; self.last_tick = time.monotonic()
        self.previous_actual_at = self.last_tick; self.braking = False
        self.storage.mkdir(parents=True,exist_ok=True)
        self.log = (self.storage / ('run-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'.jsonl')).open('x')
        self._record('start',mode='non_contact_mirror',session_timer_s=None,config=self.cfg)
        self.active = True; self._publish('running','Following tracked pose; no session timer')

    def _project_pose(self, pose):
        projected=pose.copy();r=from_rpy(pose[3:])
        # Keep the modeled tool above the existing floor without discarding
        # the requested horizontal movement and orientation.
        floor=(self.cfg['minimum_height_mm']+self.cfg['target_height_clearance_mm']
               -min(0.,(r@self.local_base)[2],(r@self.local_tip)[2]))
        projected[:3]=np.clip(projected[:3],self.xyz_low,self.xyz_high)
        projected[2]=max(projected[2],floor)
        return projected if projected[2]<=self.xyz_high[2] else None

    def _solve(self, desired, actual):
        current = self._fk(actual); a,b = from_rpy(current[3:]),from_rpy(desired[3:])
        # Native IK on firmware 2.2.2 does not support ref_angles. Unwrap
        # periodic aliases near the measured pose; do not assume seed support.
        for fraction in (1.,.5,.25,.125):
            pose = np.r_[current[:3]+fraction*(desired[:3]-current[:3]),to_rpy(interpolate_rotation(a,b,fraction))]
            projected_pose=self._project_pose(pose)
            if projected_pose is None:continue
            cartesian_projected=bool(np.max(np.abs(projected_pose-pose))>1e-6)
            pose=projected_pose
            code,q = self.arm.get_inverse_kinematics(pose.tolist(),input_is_radian=False,return_is_radian=False)
            if code==10:
                # Firmware 2.2.2 latches warning 14 for an unsuccessful IK
                # query. It is not a motor fault, and this candidate is unused.
                ec,faults=self.arm.get_err_warn_code();self._check(ec,'IK diagnostic query')
                if faults==[0,14]:self.ik_no_solution_warning=True
                elif faults!=[0,0]:raise RuntimeError(f'Unexpected controller faults after IK: {faults}')
                self._record('ik_no_solution',pose_mm_deg=pose.tolist(),warning=faults[1])
                continue
            self._check(code,'Inverse kinematics query')
            q = vector(q,7)[:6]
            for j in (0,3,5):
                q[j] += 360*round((actual[j]-q[j])/360)
            if self.arm.is_joint_limit([*q,0.],is_radian=False)!=(0,False): continue
            # Project into one declared joint envelope instead of latching a
            # return stroke or ending the session on an unreachable request.
            projected = np.clip(q,self.low+.3,self.high-.3)
            clearance=self.cfg['target_height_clearance_mm']-self.cfg['height_tolerance_mm']
            if self._admissible(projected,floor_margin_mm=clearance):
                self.joint_projected=bool(np.max(np.abs(q-projected))>1e-6)
                self.cartesian_projected=cartesian_projected
                return projected,fraction
        self.joint_projected=True;self.cartesian_projected=False
        return actual.copy(),0.

    def _stopping_path_admitted(self,q,velocity,reaction):
        accel=np.array(self.cfg['max_accel_deg_s2']); duration=np.abs(velocity)/accel
        for t in (reaction, reaction+float(max(duration))*.5, reaction+float(max(duration))):
            braking=np.minimum(np.maximum(0,t-reaction),duration)
            candidate=q+velocity*min(t,reaction)+velocity*braking-np.sign(velocity)*.5*accel*braking**2
            if not self._admissible(candidate): return False
        return True

    def _hold(self,q,reason,observed_at=None):
        observed_at = time.monotonic() if observed_at is None else observed_at
        self._velocity(np.zeros(6));self.previous_velocity[:]=0
        self.previous_actual=q;self.previous_actual_at=observed_at;self.last_tick=time.monotonic()
        self._publish('holding',reason,joints=q.tolist(),command=[0.]*6,feedback_at=observed_at)

    def _source_current(self, source):
        latest,neutral = self.state.target_source()
        return (latest is not None and neutral is not None and
                latest['epoch'] == source['epoch'] and latest['kind'] == source['kind'] and
                time.monotonic()-source['at'] <= self.cfg['tracking_timeout_s'])

    def _tick(self):
        now = time.monotonic(); dt = now-self.last_tick
        self._status(); q = self._actual()
        if not self._admissible(q): raise RuntimeError('Measured pose left the admitted workspace')
        source,neutral = self.state.target_source()
        if source is None or dt>self.cfg['velocity_lease_s']:
            self._hold(q,'Waiting for fresh tracking and a set guard')
            return
        desired = mirrored_pose(source,neutral,self.home_pose,self.cfg['position_gain'],
                                self.cfg['rotation_gain'],self.cfg['position_deadband_m'],self.cfg['angle_deadband_deg'])
        target,fraction = self._solve(desired,q)
        # Re-read after IK; a solve never renews the source timestamp.
        q = self._actual(); now = time.monotonic()
        if not self._admissible(q): raise RuntimeError('Measured pose left the admitted workspace')
        if not self._source_current(source):
            self._hold(q,'Waiting for a fresh pose and unchanged reference after IK',now); return
        measured=(q-self.previous_actual)/max(now-self.previous_actual_at,.001)
        if not self._stopping_path_admitted(q,measured,self.cfg['velocity_lease_s']):
            if not self._stopping_path_admitted(q,measured,0.):
                raise RuntimeError('Measured momentum requires controller stop')
            self.braking=True
        if self.braking:
            if np.max(np.abs(measured))>1.:
                self._hold(q,'Braking before workspace boundary',now);return
            self.braking=False
        v = velocity_step(target,q,self.previous_velocity,min(now-self.last_tick,.1),self.cfg,self.low,self.high)
        accel = np.array(self.cfg['max_accel_deg_s2']); lease=self.cfg['velocity_lease_s']
        start_pts = self._points(q)
        scale = 1.
        for scale in (1.,.5,.25,0.):
            command = v*scale
            if not self._stopping_path_admitted(q,command,lease): continue
            end_pts = self._points(q+command*lease)
            if np.max(np.linalg.norm(end_pts-start_pts,axis=1))/lease<=self.cfg['max_point_speed_mm_s']: break
        else: command=np.zeros(6);scale=0.
        # Expiry covers process loss; source expiry covers a connected but stale
        # producer. Holds remain in the same session and send no outbound stroke.
        self._status()
        if not self._source_current(source): command=np.zeros(6);scale=0.
        self._velocity(command); self.previous_velocity=command
        self._record('sample',actual_deg=q.tolist(),target_deg=target.tolist(),command_deg_s=command.tolist(),
                     encoder_speed_deg_s=measured.tolist(),projection_fraction=fraction,workspace_scale=scale,
                     requested_pose_mm_deg=desired.tolist(),joint_projected=self.joint_projected,cartesian_projected=self.cartesian_projected)
        self._publish('running' if scale and fraction else 'holding',
                      'Following tracked pose' if scale and fraction else 'Holding at reachable pose; move back into range',
                      joints=q.tolist(),target=target.tolist(),command=command.tolist(),
                      feedback_at=now,
                      source_age_ms=round((time.monotonic()-source['at'])*1000),projection_fraction=fraction,
                      cartesian_projected=self.cartesian_projected)
        self.previous_actual,self.previous_actual_at=q,now
        self.last_tick=time.monotonic()

    def _run(self):
        try:
            while not self.closed.is_set():
                if self.stop_requested.is_set():
                    self._halt('Operator Stop'); self.stop_requested.clear()
                try:
                    operation,generation = self.jobs.get_nowait()
                    if generation != self.state.generation: continue
                    self._publish('working',operation.capitalize())
                    if operation=='connect': self._connect()
                    elif operation=='home': self._home()
                    elif operation=='start': self._start()
                    elif operation=='disconnect':
                        self._halt('Disconnected')
                        if self.arm: self.arm.disconnect(); self.arm=None
                        self._publish('disconnected','Robot disconnected')
                except queue.Empty: pass
                except Exception as exc:
                    self._halt(str(exc)); self._publish('fault',str(exc))
                if self.active:
                    try: self._tick()
                    except Exception as exc:
                        self._halt(str(exc)); self._publish('fault',str(exc))
                elif self.arm and self.arm.connected and time.monotonic()-self.last_idle_feedback>.2:
                    try:
                        q=self._actual()
                        with self.state.lock:
                            self.state.robot.update(joints=q.tolist(),feedback_at=time.monotonic())
                    except Exception as exc:
                        self._publish('fault','Feedback unavailable: '+str(exc))
                    self.last_idle_feedback=time.monotonic()
                self.closed.wait(.01 if self.active else .05)
        finally:
            self._halt('Application closed')
            if self.arm: self.arm.disconnect()
