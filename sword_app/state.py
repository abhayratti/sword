"""Latest-only tracking state shared by the camera, UI, and robot worker."""
import copy
from collections import deque
import threading
import time
import numpy as np
from .math3d import rotation, source_basis, vector


class State:
    def __init__(self, timeout=.15):
        self.lock = threading.RLock()
        self.timeout = timeout
        self.pose = self.neutral = None
        self.epoch = None
        self.kind = 'markers'
        self.history = deque(maxlen=12)
        self.frames = 0
        self.reason = 'Start tracking'
        self.generation = 0
        self.robot = {'state':'disconnected','reason':'Connect the robot when ready','joints':None}
        self.vision = {'objects':[], 'camera':{'calibrated':False,'views':0,'needed':12}}
        self.frame_jpeg = None
        self.last_frame_at = None

    def invalidate(self, reason, *, reset=False):
        with self.lock:
            self.pose = None; self.reason = reason
            if reset:self.neutral=None;self.history.clear()

    def miss_frame(self, reason, epoch):
        """A detection blink never renews or extrapolates the last measurement."""
        with self.lock:
            if self.epoch != epoch or self.kind != 'markers':
                self.invalidate(reason,reset=True)
            else:
                self.reason = reason

    def update(self, pose, epoch, now=None):
        now = time.monotonic() if now is None else now
        if not isinstance(epoch,str) or not 1 <= len(epoch) <= 200: raise ValueError('Missing tracking reference')
        p, r = vector(pose['p'],3), rotation(pose['r'])
        if np.linalg.norm(p)>10: raise ValueError('Pose outside local tracking scale')
        kind = pose['kind']
        if kind not in {'markers','controller','hand'}: raise ValueError('Unknown tracking source')
        with self.lock:
            if self.epoch != epoch or self.kind != kind:
                self.neutral=None;self.history.clear()
            self.epoch, self.kind = epoch, kind
            self.pose = {'p':p,'r':r,'at':now,'kind':kind,'epoch':epoch,'head_q':pose.get('head_q')}
            self.history.append(self.pose)
            self.frames += 1; self.reason = 'Tracking'

    def current(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.pose is None or now - self.pose['at'] > self.timeout: return None
            return copy.deepcopy(self.pose)

    def set_guard(self):
        with self.lock:
            pose = self.current()
            if pose is None: raise ValueError('A fresh 3D pose is required to set the guard')
            recent=[s for s in self.history if 0<=time.monotonic()-s['at']<1.][-3:]
            if len(recent)<3:raise ValueError('Collecting three recent marker poses; keep your guard in view')
            self.neutral = copy.deepcopy(pose)
            self.neutral['p']=np.median([s['p'] for s in recent],axis=0)
            u,_,vt=np.linalg.svd(np.sum([s['r'] for s in recent],axis=0))
            self.neutral['r']=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
            self.neutral['basis'] = source_basis(pose['kind'],pose.get('head_q'))
            return 'Guard set'

    def target_source(self):
        with self.lock:
            pose = self.current()
            if pose is None or self.neutral is None: return None, None
            return pose, copy.deepcopy(self.neutral)

    def stop(self):
        with self.lock:
            self.generation += 1
            return self.generation

    def snapshot(self):
        with self.lock:
            pose = self.current()
            robot=copy.deepcopy(self.robot)
            robot['feedback_age_ms']=None if robot.get('feedback_at') is None else round((time.monotonic()-robot.pop('feedback_at'))*1000)
            reason = 'Tracking paused; the last pose is stale' if pose is None and self.pose is not None else self.reason
            return {'tracking':{'kind':self.kind,'fresh':pose is not None,'reason':reason,
                                'age_ms':None if self.pose is None else round((time.monotonic()-self.pose['at'])*1000),
                                'guard_set':self.neutral is not None,'frames':self.frames,
                                'position_m':None if pose is None else pose['p'].tolist()},
                    'robot':robot,'vision':copy.deepcopy(self.vision),
                    'frame_age_ms':None if self.last_frame_at is None else round((time.monotonic()-self.last_frame_at)*1000),
                    'stop_generation':self.generation}
