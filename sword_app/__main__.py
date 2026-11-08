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
from .math3d import from_quaternion, vector
from .robot import Robot
from .state import State
from .vision import Vision

ROOT = Path(__file__).resolve().parent.parent


def create_app(root=ROOT):
    config = tomllib.loads((root/'config.toml').read_text())
    state = State(config['robot']['tracking_timeout_s'])
    robot = Robot(state,config['robot'],root/'var')
    vision = Vision(config['vision'],root/'var')
    key = secrets.token_urlsafe(32)
    owner = None
    vision_lock = asyncio.Lock()

    @web.middleware
    async def local_only(request,handler):
        host = urlsplit('//'+request.host).hostname
        if host not in {'localhost','127.0.0.1','[::1]','::1'}: raise web.HTTPForbidden(text='Local access only')
        origin = request.headers.get('Origin')
        if origin and origin != f'{request.scheme}://{request.host}': raise web.HTTPForbidden(text='Origin mismatch')
        if request.method not in {'GET','HEAD'} and not secrets.compare_digest(request.headers.get('X-Sword-Key',''),key):
            raise web.HTTPForbidden(text='Missing application token')
        try: response = await handler(request)
        except (ValueError,KeyError,TypeError) as error:
            response = web.json_response({'error':str(error)},status=400)
        response.headers.update({'Cache-Control':'no-store','X-Content-Type-Options':'nosniff',
                                 'Permissions-Policy':'camera=(self), xr-spatial-tracking=(self), microphone=()',
                                 'Referrer-Policy':'same-origin'})
        return response

    app = web.Application(client_max_size=2*1024*1024,middlewares=[local_only])
    app['state'],app['robot'],app['vision'] = state,robot,vision

    async def snapshot(request):
        result = state.snapshot(); result.update(key=key,blade_length_m=config['vision']['blade_length_m'],
                                                app='Sword',session_timer_s=None)
        return web.json_response(result)

    async def action(request):
        data = await request.json(); op = data.get('action')
        if op=='stop': robot.stop(); message='Stop requested'
        elif op=='guard':
            if robot.active or robot.motion_owned: raise ValueError('Stop mirroring before setting a new guard')
            message=state.set_guard()
        elif op in {'connect','home','start','disconnect'}:
            robot.request(op); message=op.capitalize()+' requested'
        elif op in {'capture_board','calibrate_camera','reset_camera'}:
            if robot.active or robot.motion_owned: raise ValueError('Stop mirroring before camera setup')
            async with vision_lock:
                if op=='capture_board': result=await asyncio.to_thread(vision.capture_board)
                elif op=='calibrate_camera': result=await asyncio.to_thread(vision.solve_calibration)
                else:
                    vision.views=[];vision.signatures=[];vision.calibration=None
                    (root/'var'/'camera.json').unlink(missing_ok=True);result=vision.status()
                state.invalidate('Camera setup changed; set guard again',reset=True)
                with state.lock: state.vision['camera']=result
            message='Camera setup updated'
        else: raise ValueError('Unknown action')
        return web.json_response({'message':message})

    async def frame(request):
        with state.lock:
            if state.frame_jpeg is None: raise web.HTTPNotFound()
            if request.query.get('id') != str(state.vision.get('frame_id')): raise web.HTTPConflict()
            jpeg=state.frame_jpeg
        return web.Response(body=jpeg,content_type='image/jpeg')

    async def tracking(request):
        nonlocal owner
        if not secrets.compare_digest(request.query.get('key',''),key): raise web.HTTPForbidden()
        if owner is not None: raise web.HTTPConflict(text='A tracking device is already connected')
        ws=web.WebSocketResponse(max_msg_size=2*1024*1024,heartbeat=2)
        await ws.prepare(request); owner=ws
        nonce=secrets.token_urlsafe(16); issued=time.monotonic(); sequence=0
        await ws.send_json({'nonce':nonce,'max_age_ms':150})
        try:
            async for msg in ws:
                if msg.type!=WSMsgType.TEXT: break
                received=time.monotonic()
                try:
                    payload=json.loads(msg.data)
                    if not secrets.compare_digest(str(payload.get('nonce','')),nonce) or received-issued>.15:
                        raise ValueError('Capture or network late; waiting for a fresh frame')
                    age=payload.get('capture_age_ms')
                    if isinstance(age,bool) or not isinstance(age,(int,float)) or not 0<=age<100:
                        raise ValueError('Invalid capture age')
                    if payload.get('valid') is True and (payload.get('captured_after_challenge') is not True
                            or age>(received-issued)*1000+2):
                        raise ValueError('Expected a new camera/input sample after the capture challenge')
                    epoch=payload.get('epoch')
                    if not isinstance(epoch,str) or not 1<=len(epoch)<=150: raise ValueError('Invalid tracking reference')
                    kind=payload.get('kind')
                    if payload.get('valid') is not True:
                        state.invalidate(str(payload.get('reason','Tracking unavailable'))[:160],reset=payload.get('reset') is True)
                    elif kind=='markers':
                        jpeg=base64.b64decode(payload['jpeg'],validate=True)
                        if len(jpeg)>1400000: raise ValueError('Camera frame too large')
                        async with vision_lock:
                            result,pose=await asyncio.to_thread(vision.process,jpeg,payload['camera_key'],issued)
                        if time.monotonic()-issued>.15: raise ValueError('Camera processing late; waiting for next frame')
                        sequence+=1; result['frame_id']=sequence
                        with state.lock:
                            state.vision=result;state.frame_jpeg=jpeg;state.last_frame_at=received
                        if pose is None:
                            if not result['camera']['calibrated']:reason='Camera calibration needed'
                            else:
                                visible={o['name']:o for o in result['objects']}
                                issues=[]
                                for name,label in [('human','Your sword'),('anchor','Fixed reference')]:
                                    obj=visible.get(name)
                                    if obj is None:issues.append(label+' not fully visible')
                                    elif obj['pose'] is None:issues.append(label+': '+obj['reason'])
                                reason='; '.join(issues)
                            # A plain non-detection may use the remaining lifetime
                            # of the last measurement. Bad/ambiguous poses, setup
                            # changes and transport errors still invalidate it.
                            plain_miss = (result['camera']['calibrated'] and
                                          all(o['pose'] is not None for o in result['objects']
                                              if o['name'] in {'human','anchor'}))
                            if plain_miss: state.miss_frame(reason,epoch)
                            else: state.invalidate(reason)
                        # The sender waits for a frame after receiving this nonce.
                        # Encoding is already included in received-issued; subtracting
                        # capture_age again would count the same delay twice.
                        else: state.update(pose,epoch,issued)
                    elif kind in {'hand','controller'}:
                        pose={'p':vector(payload['position'],3),'r':from_quaternion(payload['orientation']),
                              'kind':kind,'head_q':vector(payload.get('head_orientation',[0,0,0,1]),4).tolist()}
                        state.update(pose,epoch,issued)
                    else: raise ValueError('Unknown tracking source')
                    error=None
                except (ValueError,KeyError,TypeError,OverflowError) as exc:
                    error=str(exc)[:160];state.invalidate(error)
                nonce=secrets.token_urlsafe(16);issued=time.monotonic()
                await ws.send_json({'nonce':nonce,'max_age_ms':150,'error':error,'tracking':state.snapshot()['tracking']})
        finally:
            if owner is ws: owner=None;state.invalidate('Tracking device disconnected')
        return ws

    async def page(request):
        path=request.match_info.get('path','') or 'index.html'
        # Existing headset bookmarks all converge on the same application.
        if path in {'live.html','sword-quick.html','sword.html','bridge.html','bench.html','mirror.html'}:
            raise web.HTTPFound('/')
        file=(root/'web'/path).resolve()
        if not file.is_relative_to((root/'web').resolve()) or not file.is_file(): raise web.HTTPNotFound()
        content_type={'.html':'text/html','.js':'text/javascript','.css':'text/css','.svg':'image/svg+xml','.json':'application/json','.pdf':'application/pdf'}.get(file.suffix,'application/octet-stream')
        return web.Response(body=file.read_bytes(),content_type=content_type)

    async def startup(app): robot.start_worker()
    async def cleanup(app):
        if owner: await owner.close()
        await asyncio.to_thread(robot.close)
    app.on_startup.append(startup);app.on_cleanup.append(cleanup)
    app.router.add_get('/api/state',snapshot);app.router.add_post('/api/action',action)
    app.router.add_get('/api/frame',frame);app.router.add_get('/tracking',tracking)
    app.router.add_get('/{path:.*}',page)
    return app


def main():
    parser=argparse.ArgumentParser(description='Sword · one local tracking and mirroring app')
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    print(f'Sword: http://localhost:{args.port} — robot motion starts only from the app',flush=True)
    web.run_app(create_app(),host='127.0.0.1',port=args.port,access_log=None,print=None)


if __name__=='__main__': main()
