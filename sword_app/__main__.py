"""Local application server. No robot connection or movement on startup."""
import argparse
import asyncio
import base64
import json
from pathlib import Path
import secrets
import time
import tomllib
from urllib.parse import urlsplit
from aiohttp import WSMsgType, web
import numpy as np
from .math3d import from_quaternion
from .robot import Robot
from .state import State
from .vision import Vision

ROOT = Path(__file__).resolve().parent.parent


def create_app(root=ROOT):
    config = tomllib.loads((root/'config.toml').read_text())
    state = State(config['robot']['tracking_timeout_s'])
    robot = Robot(state, config['robot'], root/'var')
    vision = Vision(config['vision'], root/'var')
    key = secrets.token_urlsafe(32)
    owner = None
    vision_lock = asyncio.Lock()

    @web.middleware
    async def local_only(request, handler):
        # Blocks other websites from driving the robot through this browser.
        if urlsplit('//'+request.host).hostname not in {'localhost', '127.0.0.1', '::1'}:
            raise web.HTTPForbidden(text='Local access only')
        if request.method not in {'GET', 'HEAD'} and not secrets.compare_digest(request.headers.get('X-Sword-Key', ''), key):
            raise web.HTTPForbidden(text='Missing application token')
        try: response = await handler(request)
        except (ValueError, KeyError, TypeError) as error:
            response = web.json_response({'error': str(error)}, status=400)
        response.headers['Cache-Control'] = 'no-store'
        return response

    app = web.Application(client_max_size=4*1024*1024, middlewares=[local_only])

    async def snapshot(request):
        result = state.snapshot(); result.update(key=key, blade_length_m=config['vision']['blade_length_m'])
        return web.json_response(result)

    async def action(request):
        op = (await request.json()).get('action')
        if op == 'stop': robot.stop(); message = 'Stop requested'
        elif op == 'guard': message = state.set_guard()
        elif op in {'connect', 'home', 'start', 'disconnect'}:
            robot.request(op); message = op.capitalize()+' requested'
        elif op in {'capture_board', 'calibrate_camera', 'reset_camera'}:
            async with vision_lock:
                if op == 'capture_board': result = await asyncio.to_thread(vision.capture_board)
                elif op == 'calibrate_camera': result = await asyncio.to_thread(vision.solve_calibration)
                else: result = vision.reset()
            if op != 'capture_board': state.note('Camera setup changed; set guard again', reset=True)
            with state.lock: state.vision['camera'] = result
            message = 'Camera setup updated'
        else: raise ValueError('Unknown action')
        return web.json_response({'message': message})

    async def frame(request):
        with state.lock:
            if state.frame_jpeg is None: raise web.HTTPNotFound()
            jpeg = state.frame_jpeg
        return web.Response(body=jpeg, content_type='image/jpeg')

    async def tracking(request):
        nonlocal owner
        if not secrets.compare_digest(request.query.get('key', ''), key): raise web.HTTPForbidden()
        if owner is not None: raise web.HTTPConflict(text='A tracking device is already connected')
        ws = web.WebSocketResponse(max_msg_size=4*1024*1024, heartbeat=2)
        await ws.prepare(request); owner = ws
        sequence = 0
        await ws.send_json({'ready': True})
        try:
            # The device sends one sample, then waits for this reply before sending the next.
            async for msg in ws:
                if msg.type != WSMsgType.TEXT: break
                received = time.monotonic(); error = None
                try:
                    payload = json.loads(msg.data); epoch = payload.get('epoch'); kind = payload.get('kind')
                    if payload.get('valid') is not True:
                        state.note(str(payload.get('reason', 'Tracking unavailable')), reset=payload.get('reset') is True)
                    elif kind == 'markers':
                        jpeg = base64.b64decode(payload['jpeg'])
                        async with vision_lock:
                            result, pose = await asyncio.to_thread(vision.process, jpeg, payload['camera_key'], received)
                        sequence += 1; result['frame_id'] = sequence
                        with state.lock:
                            state.vision = result; state.frame_jpeg = jpeg; state.last_frame_at = received
                        if pose is not None: state.update(pose, epoch, received)
                        elif not result['camera']['calibrated']: state.note('Camera calibration needed')
                        else:
                            seen = {o['name'] for o in result['objects'] if o['pose']}
                            state.note('; '.join(label+' not visible' for name, label in
                                                 [('human', 'Your sword'), ('anchor', 'Reference board')] if name not in seen))
                    elif kind in {'hand', 'controller'}:
                        state.update({'p': np.asarray(payload['position'], float), 'r': from_quaternion(payload['orientation']),
                                      'kind': kind, 'head_q': payload.get('head_orientation', [0, 0, 0, 1])}, epoch, received)
                    else: raise ValueError('Unknown tracking source')
                except (ValueError, KeyError, TypeError) as exc:
                    error = str(exc)[:160]; state.note(error)
                await ws.send_json({'error': error, 'tracking': state.snapshot()['tracking']})
        finally:
            if owner is ws: owner = None; state.note('Tracking device disconnected')
        return ws

    async def page(request):
        file = (root/'web'/(request.match_info.get('path', '') or 'index.html')).resolve()
        if not file.is_relative_to((root/'web').resolve()) or not file.is_file(): raise web.HTTPNotFound()
        return web.FileResponse(file)

    async def startup(app): robot.start_worker()
    async def cleanup(app):
        if owner: await owner.close()
        await asyncio.to_thread(robot.close)
    app.on_startup.append(startup); app.on_cleanup.append(cleanup)
    app.router.add_get('/api/state', snapshot); app.router.add_post('/api/action', action)
    app.router.add_get('/api/frame', frame); app.router.add_get('/tracking', tracking)
    app.router.add_get('/{path:.*}', page)
    return app


def main():
    parser = argparse.ArgumentParser(description='Sword · local tracking and mirroring app')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    print(f'Sword: http://localhost:{args.port}', flush=True)
    web.run_app(create_app(), host='127.0.0.1', port=args.port, access_log=None, print=None)


if __name__ == '__main__': main()
