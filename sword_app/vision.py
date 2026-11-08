"""Camera calibration and marker poses relative to a fixed checkerboard."""
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
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary'])), params)
        self.views = []
        self.camera_key = self.size = self.calibration = None
        self.last_image = None
        self.pose_history = {}
        nx, ny = config['checkerboard_inner']
        self.board_flat = np.zeros((nx*ny, 3), np.float32)
        self.board_flat[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2) * config['checkerboard_square_m']
        self.board_centred = np.zeros((nx*ny, 3), np.float32)
        self.board_centred[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
        self.board_centred[:, 0] -= (nx-1)/2; self.board_centred[:, 1] = (ny-1)/2 - self.board_centred[:, 1]
        self.board_centred *= config['checkerboard_square_m']
        h = config['marker_size_m']/2
        self.marker = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

    def _camera(self, key, size):
        if self.camera_key == key and self.size == size: return
        self.camera_key, self.size = key, size
        self.views, self.calibration, self.pose_history = [], None, {}
        p = self.storage / 'camera.json'
        if p.exists():
            value = json.loads(p.read_text())
            if value.get('camera_key') == key and value.get('size') == list(size): self.calibration = value

    def reset(self):
        self.views, self.calibration = [], None
        (self.storage / 'camera.json').unlink(missing_ok=True)
        return self.status()

    def status(self):
        c = self.calibration or {}
        return {'calibrated': self.calibration is not None, 'views': len(self.views),
                'needed': self.cfg['minimum_calibration_views'], 'size': self.size,
                'method': c.get('method', 'checkerboard_fit') if c else None,
                'rms_px': c.get('rms_px'), 'validation_px': c.get('validation_px')}

    def capture_board(self):
        if self.last_image is None: raise ValueError('Start the camera first')
        found, corners = cv2.findChessboardCornersSB(self.last_image, tuple(self.cfg['checkerboard_inner']),
                                                    flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found: raise ValueError('The complete checkerboard was not detected')
        self.views.append(corners)
        return self.status()

    def solve_calibration(self):
        if len(self.views) < self.cfg['minimum_calibration_views']:
            raise ValueError(f"Capture at least {self.cfg['minimum_calibration_views']} board views")
        rms, k, d, _, _ = cv2.calibrateCamera([self.board_flat]*len(self.views), self.views, self.size, None, None)
        self.calibration = {'camera_key': self.camera_key, 'size': list(self.size), 'matrix': k.tolist(),
                            'distortion': d.tolist(), 'rms_px': float(rms), 'validation_px': [float(rms)]}
        self.storage.mkdir(parents=True, exist_ok=True)
        (self.storage / 'camera.json').write_text(json.dumps(self.calibration, indent=2))
        return self.status()

    def _solve_pose(self, obj, corners, method, prior=None):
        k, d = np.array(self.calibration['matrix']), np.array(self.calibration['distortion'])
        ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, corners, k, d, flags=method)
        candidates = []
        for rv, tv, e in zip(rvecs, tvecs, np.ravel(errors) if errors is not None else [0.]*len(rvecs)):
            r, p = cv2.Rodrigues(rv)[0], tv.reshape(3)
            if np.min((obj @ r.T + p)[:, 2]) > 0: candidates.append((float(e), r, p))  # in front of the camera
        if not ok or not candidates: return None
        candidates.sort(key=lambda c: c[0])
        if prior is not None and len(candidates) > 1 and candidates[1][0] - candidates[0][0] < .5:
            # Planar flip ambiguity: when both fits are similar, stay near the previous orientation.
            candidates[:2] = sorted(candidates[:2], key=lambda c: np.linalg.norm(cv2.Rodrigues(c[1] @ prior['r'].T)[0]))
        e, r, p = candidates[0]
        return {'p': p, 'r': r, 'error_px': e}

    def _board(self, gray):
        nx, ny = self.cfg['checkerboard_inner']; height, width = gray.shape
        scale = min(1., 640/max(width, height))
        small = cv2.resize(gray, (round(width*scale), round(height*scale)))
        found, corners = cv2.findChessboardCornersSB(small, (nx, ny), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found: return None, None
        corners = cv2.cornerSubPix(gray, (corners+.5)/scale-.5, (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
        grid = corners.reshape(ny, nx, 2)
        u, v = grid[0, 1]-grid[0, 0], grid[1, 0]-grid[0, 0]
        if u[0]*v[1]-u[1]*v[0] < 0: grid = grid[::-1]
        # The printed board has a black top-left outer cell and a white bottom-right one,
        # which fixes the detector's 180-degree ordering.
        first = grid[0, 0]-.5*(grid[0, 1]+grid[1, 0]-2*grid[0, 0])
        last = grid[-1, -1]+.5*(2*grid[-1, -1]-grid[-1, -2]-grid[-2, -1])
        levels = []
        for point in (first, last):
            x, y = np.clip(np.rint(point).astype(int), 2, [width-3, height-3])
            levels.append(float(np.median(gray[y-2:y+3, x-2:x+3])))
        if levels[0] > levels[1]: grid = grid[::-1, ::-1]
        pose = self._solve_pose(self.board_centred, np.ascontiguousarray(grid.reshape(-1, 2)), cv2.SOLVEPNP_IPPE)
        item = {'id': 'board', 'name': 'anchor', 'corners': grid[[0, 0, -1, -1], [0, -1, -1, 0]].tolist(),
                'blade_pixels': None, 'pose': None if pose is None else {'position_m': pose['p'].tolist()}}
        return item, pose

    def process(self, jpeg, key, frame_at=None):
        frame_at = time.monotonic() if frame_at is None else frame_at
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None: raise ValueError('Could not decode camera image')
        height, width = image.shape[:2]; self._camera(key, (width, height))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.last_image = gray
        objects, poses = [], {}
        if self.calibration is not None:
            board, anchor = self._board(gray)
            if board: objects.append(board)
            if anchor: poses['anchor'] = anchor
        else:
            anchor = None
        corners, ids, _ = self.detector.detectMarkers(gray)
        names = {self.cfg['human_id']: 'human', self.cfg['robot_id']: 'robot'}; seen = set()
        for raw_id, raw_corners in zip([] if ids is None else ids.flatten(), corners):
            name = names.get(int(raw_id))
            if name is None or name in seen: continue
            seen.add(name)
            points = raw_corners.reshape(4, 2)
            pose = blade_pixels = None
            if self.calibration is not None:
                previous = self.pose_history.get(name)
                prior = None
                if anchor is not None and previous is not None and frame_at - previous['at'] < .5:
                    prior = {'r': anchor['r'] @ previous['r']}  # carried through the fixed board
                pose = self._solve_pose(self.marker, points, cv2.SOLVEPNP_IPPE_SQUARE, prior)
            if pose is not None:
                poses[name] = pose
                blade = np.array([[0., 0., 0.], [0., self.cfg['blade_length_m'], 0.]])
                if np.min((blade @ pose['r'].T + pose['p'])[:, 2]) > .005:
                    blade_pixels = cv2.projectPoints(blade, cv2.Rodrigues(pose['r'])[0], pose['p'],
                                                     np.array(self.calibration['matrix']),
                                                     np.array(self.calibration['distortion']))[0].reshape(2, 2).tolist()
                if anchor is not None:
                    self.pose_history[name] = {'r': anchor['r'].T @ pose['r'], 'at': frame_at}
            objects.append({'id': int(raw_id), 'name': name, 'corners': points.tolist(), 'blade_pixels': blade_pixels,
                            'pose': None if pose is None else {'position_m': pose['p'].tolist()}})
        source = None
        if 'anchor' in poses and 'human' in poses:
            a, h = poses['anchor'], poses['human']
            source = {'p': a['r'].T @ (h['p'] - a['p']), 'r': a['r'].T @ h['r'], 'kind': 'markers'}
        return {'objects': objects, 'camera': self.status(), 'width': width, 'height': height}, source
