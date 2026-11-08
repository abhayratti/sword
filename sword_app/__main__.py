"""Local application server. No robot connection or movement on startup."""
import argparse
import asyncio
import json
from pathlib import Path
import secrets
import time
import tomllib
from urllib.parse import urlsplit
from aiohttp import WSMsgType, web
from .blocking import BLOCKS
from .math3d import from_quaternion
from .robot import Robot
from .state import State

ROOT = Path(__file__).resolve().parent.parent


def create_app(root=ROOT):
    config = tomllib.loads((root/'config.toml').read_text())
    state = State({**config['tracking'], **config['blocking']}, root/'var')
    robot = Robot(state, config['robot'], root/'var', config['blocks'], config.get('attacks'))
    key = secrets.token_urlsafe(32)
    owner = None

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

    app = web.Application(middlewares=[local_only])

    async def snapshot(request):
        result = state.snapshot()
        result.update(key=key, blocks=BLOCKS, taught=sorted(robot.blocks), zones=config['blocking'])
        return web.json_response(result)

    async def action(request):
        data = await request.json(); op = data.get('action')
        if op == 'stop': robot.stop(); message = 'Stop requested'
        elif op == 'guard': message = state.press_a()
        elif op == 'calibrate_zones': message = state.start_zone_calibration()
        elif op == 'blade_up': message = state.set_blade_up()
        elif op in {'connect', 'home', 'start', 'disconnect', 'freedrive', 'save_block', 'goto_block'}:
            robot.request(op, data.get('value')); message = op.replace('_', ' ').capitalize() + ' requested'
        else: raise ValueError('Unknown action')
        return web.json_response({'message': message})

    def hud():
        """What the in-headset status panel needs."""
        s = state.snapshot(); t, r = s['tracking'], s['robot']
        return {'fresh': t['fresh'], 'blade': t['blade_calibrated'], 'guard': t['guard_set'],
                'calibrating': t['calibrating'], 'step': t['calibration_step'], 'zones': bool(t['zones']),
                'connected': bool(r.get('firmware')) and r.get('state') != 'disconnected',
                'robot': r.get('state'), 'reason': r.get('reason'), 'zone': s['threat']['zone'], 'strike': s['threat']['strike']}

    async def tracking(request):
        nonlocal owner
        if not secrets.compare_digest(request.query.get('key', ''), key): raise web.HTTPForbidden()
        if owner is not None: raise web.HTTPConflict(text='A tracking device is already connected')
        ws = web.WebSocketResponse(heartbeat=2)
        await ws.prepare(request); owner = ws
        await ws.send_json({'ready': True})
        try:
            # The headset sends one sample, then waits for this reply before sending the next.
            async for msg in ws:
                if msg.type != WSMsgType.TEXT: break
                received = time.monotonic(); error = message = None
                try:
                    payload = json.loads(msg.data)
                    if payload.get('reset'): state.note(str(payload.get('reason', 'Tracking reset')), reset=True)
                    elif payload.get('valid'):
                        state.update(payload['position'], from_quaternion(payload['orientation']),
                                     payload['head_orientation'], payload['epoch'], received)
                    else: state.note(str(payload.get('reason', 'Controller not tracked')))
                    # Controller buttons: right A = set guard (or record the next zone while calibrating),
                    # right B = calibrate blade (pointing straight up), right thumbstick click = calibrate zones,
                    # left X = start blocking, left Y = stop.
                    command = payload.get('command')
                    if command == 'guard': message = state.press_a()
                    elif command == 'calibrate_zones': message = state.start_zone_calibration()
                    elif command == 'blade_up': message = state.set_blade_up()
                    elif command == 'stop': robot.stop(); message = 'Stopped'
                    elif command == 'start':
                        if robot.active: message = 'Already blocking'
                        elif robot.arm is None: raise ValueError('Connect the robot on the laptop first')
                        elif state.neutral is None: raise ValueError('Set your guard first (A)')
                        else: robot.request('start'); message = 'Starting'
                except (ValueError, KeyError, TypeError) as exc:
                    error = str(exc)[:160]; state.note(error)
                await ws.send_json({'error': error, 'message': message, 'hud': hud()})
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
    app.router.add_get('/tracking', tracking)
    app.router.add_get('/{path:.*}', page)
    return app


def main():
    parser = argparse.ArgumentParser(description='Sword · local blocking app')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    print(f'Sword: http://localhost:{args.port}', flush=True)
    web.run_app(create_app(), host='127.0.0.1', port=args.port, access_log=None, print=None)


if __name__ == '__main__': main()
