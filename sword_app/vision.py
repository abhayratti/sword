"""Camera calibration and measured fiducial poses; no hand-pose substitution."""
import json
import time
from pathlib import Path
import cv2
import numpy as np

cv2.ocl.setUseOpenCL(False)
cv2.setNumThreads(2)


class Vision:
    def __init__(self, config, storage):
        self.cfg = config
        self.storage = Path(storage)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.views, self.signatures = [], []
        self.camera_key, self.size, self.calibration = None, None, None
        self.last_board = None
        self.pose_history = {}

    def _camera(self, key, size):
        if not isinstance(key, str) or not 1 <= len(key) <= 200:
            raise ValueError('Missing camera identity')
        if self.camera_key == key and self.size == size:
            return
        self.camera_key, self.size = key, size
        self.views, self.signatures, self.calibration = [], [], None
        self.pose_history = {}
        p = self.storage / 'camera.json'
        if p.exists():
            value = json.loads(p.read_text())
            if value.get('camera_key') == key and value.get('size') == list(size):
                k = np.asarray(value.get('matrix'), dtype=float)
                d = np.asarray(value.get('distortion'), dtype=float)
                if k.shape == (3,3) and d.size >= 4 and np.isfinite(k).all() and np.isfinite(d).all() and k[0,0] > 0 and k[1,1] > 0:
                    self.calibration = value

    def status(self):
        return {'calibrated': self.calibration is not None, 'views': len(self.views),
                'needed': self.cfg['minimum_calibration_views'], 'size': self.size,
                'reference': self.cfg.get('reference','marker'),
                'method': self.calibration.get('method','checkerboard_fit') if self.calibration else None,
                'rms_px': self.calibration.get('rms_px') if self.calibration else None,
                'validation_px': self.calibration.get('validation_px') if self.calibration else None}

    def add_calibration_view(self):
        if self.last_board is None:
            raise ValueError('Show the complete checkerboard to the selected camera first')
        corners, signature = self.last_board
        if any(np.linalg.norm(signature - old) < .12 for old in self.signatures):
            raise ValueError('Move or tilt the board more before capturing another view')
        self.views.append(corners.copy()); self.signatures.append(signature.copy())
        return self.status()

    def solve_calibration(self):
        if len(self.views) < self.cfg['minimum_calibration_views']:
            raise ValueError('Capture at least 12 distinct board positions and angles')
        # Preserve the observations even when a fit fails; no camera pixels are saved.
        self.storage.mkdir(parents=True,exist_ok=True)
        attempt = {'camera_key':self.camera_key,'size':list(self.size),
                   'corners':[v.tolist() for v in self.views],'status':'fitting'}
        report = self.storage/'camera-attempt.json'
        report.write_text(json.dumps(attempt,indent=2))
        nx, ny = self.cfg['checkerboard_inner']
        obj = np.zeros((nx*ny,3), np.float32)
        obj[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2) * self.cfg['checkerboard_square_m']
        training, validation = self.views[:-3], self.views[-3:]
        rms, k, d, _, _ = cv2.calibrateCamera([obj] * len(training), training, self.size, None, None)
        errors = []
        for corners in validation:
            ok, r, t = cv2.solvePnP(obj, corners, k, d)
            if not ok: raise ValueError('Camera validation failed')
            projected = cv2.projectPoints(obj,r,t,k,d)[0]
            errors.append(float(np.sqrt(np.mean(np.sum((projected-corners).reshape(-1,2)**2,axis=1)))))
        width,height=self.size
        plausible=(.2*width<k[0,0]<5*width and .2*height<k[1,1]<5*height
                   and 0<k[0,2]<width and 0<k[1,2]<height)
        accepted = bool(plausible and np.isfinite(k).all() and np.isfinite(d).all()
                        and np.isfinite(rms) and np.isfinite(errors).all() and rms <= 1.0 and max(errors) <= 1.5)
        attempt.update(status='accepted' if accepted else 'rejected',rms_px=float(rms),
                       validation_px=errors,matrix=k.tolist(),distortion=d.tolist(),plausible=bool(plausible))
        report.write_text(json.dumps(attempt,indent=2))
        if not accepted:
            raise ValueError(f'Camera fit {rms:.2f} px; check {max(errors):.2f} px '
                             '(required ≤1.00 / ≤1.50). Keep the board rigid and vary its tilt. '
                             'Captured corners are saved for diagnosis.')
        # Keep the validation fit separate; do not refit using its three views.
        self.calibration = {'camera_key': self.camera_key, 'size': list(self.size),
                            'matrix': k.tolist(), 'distortion': d.tolist(), 'rms_px': float(rms),
                            'validation_px': errors, 'training_views': len(training), 'validation_views': 3}
        temp = self.storage / 'camera.json.tmp'; temp.write_text(json.dumps(self.calibration,indent=2))
        temp.replace(self.storage/'camera.json')
        return self.status()

    def _pose(self, corners, prior=None):
        if self.calibration is None: return None, 'Camera calibration needed'
        edges = np.linalg.norm(corners - np.roll(corners,-1,axis=0),axis=1)
        if min(edges) < self.cfg['minimum_marker_edge_px']: return None, 'Marker too small or edge-on'
        h = self.cfg['marker_size_m']/2
        obj = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
        return self._solve_pose(obj,corners,cv2.SOLVEPNP_IPPE_SQUARE,prior)

    def _solve_pose(self, obj, corners, method, prior=None):
        if self.calibration is None: return None, 'Camera calibration needed'
        k, d = np.array(self.calibration['matrix']), np.array(self.calibration['distortion'])
        result = cv2.solvePnPGeneric(obj, corners, k, d, flags=method)
        candidates = []
        if not result[0]: return None, 'No marker pose'
        for rv, tv in zip(result[1], result[2]):
            r, p = cv2.Rodrigues(rv)[0], tv.reshape(3)
            if np.min((obj @ r.T + p)[:,2]) <= 0: continue
            projected = cv2.projectPoints(obj,rv,tv,k,d)[0].reshape(-1,2)
            error = float(np.sqrt(np.mean(np.sum((projected-corners.reshape(-1,2))**2,axis=1))))
            if np.isfinite(error): candidates.append((error,r,p))
        candidates.sort(key=lambda c:c[0])
        if not candidates or candidates[0][0] > self.cfg['max_reprojection_px']:
            return None, 'Marker pose fit is poor'
        e,r,p = candidates[0]
        orientation_source='image'
        if len(candidates)>1 and candidates[1][0] - e < .2:
            angle = np.linalg.norm(cv2.Rodrigues(candidates[1][1] @ r.T)[0])
            if angle > np.radians(15):
                if prior is None:return None, 'Marker orientation ambiguous; tilt it slightly'
                distances=[np.linalg.norm(cv2.Rodrigues(c[1]@prior['r'].T)[0]) for c in candidates[:2]]
                order=np.argsort(distances)
                if distances[order[0]]>np.radians(20) or distances[order[1]]-distances[order[0]]<np.radians(10):
                    return None, 'Marker orientation ambiguous; tilt it slightly'
                e,r,p=candidates[order[0]]
                if e>self.cfg['max_reprojection_px']:return None, 'Marker pose fit is poor'
                orientation_source='recent_pose_and_image'
        return {'p':p,'r':r,'error_px':e,'orientation_source':orientation_source}, None

    def _prediction(self, name, anchor, now):
        previous=self.pose_history.get(name)
        if anchor is None or previous is None or not 0<=now-previous['at']<.15:return None
        # Transport the previous measured orientation through the newly observed
        # fixed board. Headset rotation is not mistaken for sword rotation.
        return {'p':anchor['r']@previous['p']+anchor['p'],'r':anchor['r']@previous['r']}

    def _board_reference(self, gray):
        nx,ny=self.cfg['checkerboard_inner'];height,width=gray.shape
        scale=min(1.,640/max(width,height))
        small=cv2.resize(gray,(round(width*scale),round(height*scale)))
        found,corners=cv2.findChessboardCornersSB(small,(nx,ny),flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:return None,None
        corners=(corners+.5)/scale-.5
        corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,30,.001))
        grid=corners.reshape(ny,nx,2)
        u,v=grid[0,1]-grid[0,0],grid[1,0]-grid[0,0]
        if u[0]*v[1]-u[1]*v[0]<0:grid=grid[::-1]
        # The supplied 10x7-square board has a black top-left outer cell and
        # a white bottom-right one. This fixes the detector's 180-degree ordering.
        first=grid[0,0]-.5*(grid[0,1]+grid[1,0]-2*grid[0,0])
        last=grid[-1,-1]+.5*(2*grid[-1,-1]-grid[-1,-2]-grid[-2,-1])
        levels=[]
        for point in (first,last):
            x,y=np.rint(point).astype(int)
            if not 2<=x<width-2 or not 2<=y<height-2:return None,None
            levels.append(float(np.median(gray[y-2:y+3,x-2:x+3])))
        if abs(levels[0]-levels[1])<30:return None,None
        if levels[0]>levels[1]:grid=grid[::-1,::-1]
        obj=np.zeros((nx*ny,3),np.float32)
        obj[:,:2]=np.mgrid[0:nx,0:ny].T.reshape(-1,2)
        obj[:,0]-=(nx-1)/2;obj[:,1]=(ny-1)/2-obj[:,1]
        obj*=self.cfg['checkerboard_square_m']
        pose,reason=self._solve_pose(obj,np.ascontiguousarray(grid.reshape(-1,2)),cv2.SOLVEPNP_IPPE)
        outline=grid[[0,0,-1,-1],[0,-1,-1,0]].tolist()
        item={'id':'board','name':'anchor','corners':outline,'blade_pixels':None,'reason':reason,
              'pose':None if pose is None else {'position_m':pose['p'].tolist(),'rotation':pose['r'].tolist(),'reprojection_px':pose['error_px']}}
        return item,pose

    def process(self, jpeg, key, frame_at=None):
        frame_at=time.monotonic() if frame_at is None else frame_at
        image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.size > 1280*1024*3: raise ValueError('Expected camera image up to 1280 × 1024')
        height,width = image.shape[:2]; self._camera(key,(width,height))
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # Lens calibration is on demand; the fixed board's pose is measured live.
        self.last_image = gray; self.last_image_at = time.monotonic()
        corners,ids,_ = self.detector.detectMarkers(gray)
        board_reference=self.cfg.get('reference','marker')=='checkerboard'
        board,anchor=self._board_reference(gray) if board_reference else (None,None)
        names = {self.cfg['human_id']:'human',self.cfg['robot_id']:'robot'}
        if not board_reference:names[self.cfg['anchor_id']]='anchor'
        objects, poses = [], {}
        seen = set()
        if ids is not None:
            for raw_id, raw_corners in zip(ids.flatten(),corners):
                ident = int(raw_id)
                if ident not in names: continue
                name = names[ident]; points = raw_corners.reshape(4,2)
                if name in seen: raise ValueError('Duplicate marker ID visible; use one copy of each ID')
                seen.add(name); pose,reason = self._pose(points,self._prediction(name,anchor,time.monotonic()))
                blade_pixels=None
                if pose is not None and name in {'human','robot'}:
                    blade=np.array([[0.,0.,0.],[0.,self.cfg['blade_length_m'],0.]])
                    if np.min((blade@pose['r'].T+pose['p'])[:,2])>.005:
                        blade_pixels=cv2.projectPoints(blade,cv2.Rodrigues(pose['r'])[0],pose['p'],
                            np.array(self.calibration['matrix']),np.array(self.calibration['distortion']))[0].reshape(2,2).tolist()
                objects.append({'id':ident,'name':name,'corners':points.tolist(),'blade_pixels':blade_pixels,
                                'pose':None if pose is None else {'position_m':pose['p'].tolist(),'rotation':pose['r'].tolist(),'reprojection_px':pose['error_px'],'orientation_source':pose['orientation_source']},'reason':reason})
                if pose is not None:
                    poses[name] = pose
                    if anchor is not None:
                        self.pose_history[name]={'p':anchor['r'].T@(pose['p']-anchor['p']),
                                                 'r':anchor['r'].T@pose['r'],'at':frame_at}
        if board_reference:
            if board is not None:objects.append(board)
            if anchor is not None:poses['anchor']=anchor
        source = None
        if 'anchor' in poses and 'human' in poses:
            anchor, human = poses['anchor'], poses['human']
            source = {'p':anchor['r'].T @ (human['p']-anchor['p']), 'r':anchor['r'].T @ human['r'], 'kind':'markers'}
        return {'objects':objects,'camera':self.status(),'width':width,'height':height}, source

    def capture_board(self):
        if not hasattr(self,'last_image') or time.monotonic()-self.last_image_at>.5:
            raise ValueError('Start the camera and keep its view visible first')
        found,corners = cv2.findChessboardCornersSB(self.last_image,tuple(self.cfg['checkerboard_inner']),
                                                flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found: raise ValueError('The complete checkerboard was not detected')
        pts = corners.reshape(-1,2); width,height = self.size
        centre = pts.mean(axis=0)/[width,height]
        span = np.ptp(pts,axis=0)/[width,height]
        delta = pts[self.cfg['checkerboard_inner'][0]-1] - pts[0]
        angle = np.arctan2(delta[1],delta[0])
        signature = np.r_[centre,span,np.sin(angle),np.cos(angle)]
        self.last_board = corners,signature
        return self.add_calibration_view()

