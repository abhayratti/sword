import * as THREE from './vendor/three.module.js';

// One sample in flight: send, wait for the server's reply, send the newest next.
class Link {
  constructor(key,onStatus,onClosed){this.key=key;this.onStatus=onStatus;this.onClosed=onClosed;this.ws=null;this.ready=false;this.epoch=crypto.randomUUID();this.closed=false;}
  async open(){
    this.ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/tracking?key=${encodeURIComponent(this.key)}`);
    await new Promise((resolve,reject)=>{
      this.ws.onmessage=e=>{const r=JSON.parse(e.data);this.ready=true;if(r.ready){this.onStatus('Tracking link connected');resolve();}else if(r.error)this.onStatus(r.error);};
      this.ws.onerror=()=>reject(new Error('Could not open tracking. Another device may already be streaming.'));
      this.ws.onclose=()=>{const expected=this.closed;this.closed=true;this.ready=false;if(!expected)this.onClosed();};
    });
  }
  send(data){
    if(this.closed||this.ws?.readyState!==1||!this.ready)return false;
    this.ready=false;this.ws.send(JSON.stringify({...data,epoch:this.epoch}));return true;
  }
  reset(reason){this.epoch=crypto.randomUUID();this.send({valid:false,reset:true,reason});}
  close(){this.closed=true;this.ready=false;this.ws?.close();}
}

export class Tracker {
  constructor(onStatus,onStop){this.onStatus=onStatus;this.onStop=onStop;this.link=null;this.stream=null;this.session=null;this.running=false;this.generation=0;this.mode=null;}
  async start(kind,key){
    if(this.running)await this.stop();this.mode=kind;this.generation++;this.running=true;
    this.link=new Link(key,this.onStatus,()=>{this.stop().then(()=>this.onStatus('Tracking link lost. Start tracking to reconnect.'));});
    try{
      this.onStatus(kind==='markers'?'Waiting for camera access on this device…':'Opening tracked input…');
      if(kind==='markers')await this.camera();else await this.xr();
      if(!this.running)throw new Error('Tracking startup was cancelled.');
      await this.link.open();
    }catch(e){await this.stop();throw e;}
  }
  async camera(){
    if(!navigator.mediaDevices?.getUserMedia)throw new Error('Camera access unavailable in this browser');
    const quest=/OculusBrowser|Quest/i.test(navigator.userAgent);
    try{
      this.stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{
        ...(quest?{facingMode:{exact:'environment'}}:{}),width:{ideal:1280},height:{ideal:960},frameRate:{ideal:30,max:30}}});
    }catch(e){
      if(quest&&['OverconstrainedError','NotFoundError'].includes(e.name))
        throw new Error('Quest Browser is not exposing its environment camera. In Quest settings, open Browser → Permissions → Headset cameras and allow access, then retry.');
      if(e.name==='NotAllowedError')throw new Error('Camera permission was not granted. Allow this page to use the camera, then retry.');
      throw new Error(`Camera could not start: ${e.message||e.name}`);
    }
    if(!this.running){this.stream.getTracks().forEach(t=>t.stop());this.stream=null;throw new Error('Tracking startup was cancelled.');}
    const video=document.getElementById('camera');video.srcObject=this.stream;await video.play();
    const track=this.stream.getVideoTracks()[0],settings=track.getSettings();
    // Calibration is saved per camera identity and resolution.
    const fingerprint=JSON.stringify({device:settings.deviceId,width:settings.width,height:settings.height,facing:settings.facingMode,ua:navigator.userAgent});
    this.cameraKey=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(fingerprint)))).map(x=>x.toString(16).padStart(2,'0')).join('');
    const canvas=document.createElement('canvas'),ctx=canvas.getContext('2d',{alpha:false});
    const generation=this.generation,live=()=>this.running&&generation===this.generation;
    const send=source=>{
      if(!live()||!this.link?.ready)return;
      const width=source.displayWidth||source.videoWidth,height=source.displayHeight||source.videoHeight;
      const scale=Math.min(1,1280/width);
      canvas.width=Math.round(width*scale);canvas.height=Math.round(height*scale);ctx.drawImage(source,0,0,canvas.width,canvas.height);
      // Direct encoding: Quest can defer toBlob callbacks by hundreds of milliseconds.
      this.link.send({valid:true,kind:'markers',jpeg:canvas.toDataURL('image/jpeg',.85).split(',')[1],camera_key:this.cameraKey});
    };
    if(typeof MediaStreamTrackProcessor==='function'){
      // Reads frames even when the browser is not drawing <video>; a one-frame queue avoids backlog.
      const reader=new MediaStreamTrackProcessor({track,maxBufferSize:1}).readable.getReader();
      this.cameraReader=reader;
      (async()=>{
        try{
          while(live()){
            const {value:frame,done}=await reader.read();if(done)break;
            try{send(frame);}finally{frame.close();}
          }
        }catch(e){if(live())this.onStatus('Camera frames stopped: '+e.message);}
        if(live())await this.stop();
      })();
    }else{
      if(!video.requestVideoFrameCallback)throw new Error('This browser does not expose camera frames');
      const tick=()=>{if(!live())return;video.requestVideoFrameCallback(tick);send(video);};
      video.requestVideoFrameCallback(tick);
    }
    track.addEventListener('ended',()=>this.stop());
    this.onStatus('Camera active. Show marker 10 and the reference board.');
  }
  async xr(){
    if(this.session)return;
    if(!navigator.xr||!await navigator.xr.isSessionSupported('immersive-ar'))throw new Error('Open this app in Quest Browser for passthrough and tracked inputs');
    const hand=this.mode==='hand';
    const session=await navigator.xr.requestSession('immersive-ar',{requiredFeatures:['local',...(hand?['hand-tracking']:[])],optionalFeatures:['local-floor']});
    if(!this.running){await session.end();throw new Error('Tracking startup was cancelled.');}
    this.session=session;
    const renderer=new THREE.WebGLRenderer({alpha:true,antialias:true});renderer.setClearColor(0,0);renderer.xr.enabled=true;renderer.xr.setReferenceSpaceType('local');
    document.getElementById('xr').append(renderer.domElement);await renderer.xr.setSession(session);
    const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera();const axes=new THREE.AxesHelper(.08);axes.visible=false;scene.add(axes);
    const space=renderer.xr.getReferenceSpace();space.addEventListener('reset',()=>{this.link?.reset('Tracking origin changed; set guard again');});
    session.addEventListener('end',()=>{renderer.setAnimationLoop(null);renderer.dispose();renderer.domElement.remove();this.session=null;if(this.mode!=='markers')this.stop();else this.onStop();});
    renderer.setAnimationLoop((time,frame)=>{
      if(!frame)return;
      if(this.mode!=='markers'){
        const input=Array.from(session.inputSources).find(s=>s.handedness==='right'&&(hand?!!s.hand:!!s.gripSpace&&!s.hand));
        const inputSpace=hand?input?.hand?.get('wrist'):input?.gripSpace;
        let pose=null;try{pose=inputSpace&&(hand?frame.getJointPose(inputSpace,space):frame.getPose(inputSpace,space));}catch{}
        const head=frame.getViewerPose(space);
        axes.visible=!!(pose&&head);
        if(pose&&head){
          const p=pose.transform.position,q=pose.transform.orientation,h=head.transform.orientation;
          axes.position.set(p.x,p.y,p.z);axes.quaternion.set(q.x,q.y,q.z,q.w);
          this.link?.send({valid:true,kind:this.mode,position:[p.x,p.y,p.z],orientation:[q.x,q.y,q.z,q.w],head_orientation:[h.x,h.y,h.z,h.w]});
        }
      }
      renderer.render(scene,camera);
    });
  }
  async stop(){
    if(!this.running&&!this.session)return;this.running=false;this.generation++;
    const reader=this.cameraReader;this.cameraReader=null;if(reader)try{await reader.cancel();}catch{}
    this.link?.close();this.link=null;this.stream?.getTracks().forEach(t=>t.stop());this.stream=null;
    const session=this.session;this.session=null;if(session)try{await session.end();}catch{}
    document.getElementById('camera').srcObject=null;this.onStatus('Tracking stopped');this.onStop();
  }
}
