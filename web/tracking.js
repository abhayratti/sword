import * as THREE from './vendor/three.module.js';

class Link {
  constructor(key,onStatus,onClosed){this.key=key;this.onStatus=onStatus;this.onClosed=onClosed;this.ws=null;this.nonce=null;this.epoch=crypto.randomUUID();this.closed=false;}
  async open(){
    this.ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/tracking?key=${encodeURIComponent(this.key)}`);
    await new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>reject(new Error('Tracking connection timed out')),4000);
      this.ws.onmessage=e=>{const r=JSON.parse(e.data);this.nonce=r.nonce;this.nonceAt=performance.now();this.onStatus(r.error||'Tracking link connected');clearTimeout(timer);resolve();};
      this.ws.onerror=()=>{clearTimeout(timer);reject(new Error('Could not open tracking. Another device may already be streaming.'));};
      this.ws.onclose=()=>{const expected=this.closed;this.closed=true;this.nonce=null;if(!expected)this.onClosed();};
    });
  }
  send(data){
    if(this.closed||this.ws?.readyState!==1||!this.nonce)return false;
    const {sampled_at_ms=performance.now(),...payload}=data;
    if(data.valid===true&&sampled_at_ms<this.nonceAt)return false;
    const nonce=this.nonce;this.nonce=null;
    this.ws.send(JSON.stringify({...payload,captured_after_challenge:true,nonce,epoch:this.epoch}));return true;
  }
  reset(reason){this.epoch=crypto.randomUUID();this.send({valid:false,reset:true,reason,capture_age_ms:0});}
  close(){this.closed=true;this.nonce=null;this.ws?.close();}
}

export class Tracker {
  constructor(onStatus,onStop){this.onStatus=onStatus;this.onStop=onStop;this.link=null;this.stream=null;this.session=null;this.running=false;this.generation=0;this.mode=null;}
  async start(kind,key){
    if(this.running)await this.stop();this.mode=kind;this.generation++;this.running=true;
    this.link=new Link(key,this.onStatus,()=>{this.stop().then(()=>this.onStatus('Tracking link lost. Start tracking to reconnect.'));});
    try{
      this.onStatus(kind==='markers'?'Waiting for camera access on this device…':'Opening tracked input…');
      if(kind==='markers')await this.camera();else await this.xr();
      if(!this.running||!this.link)throw new Error('Tracking startup was cancelled.');
      await this.link.open();
    }catch(e){await this.stop();throw e;}
  }
  async camera(){
    if(!navigator.mediaDevices?.getUserMedia)throw new Error('Camera access unavailable in this browser');
    const quest=/OculusBrowser|Quest/i.test(navigator.userAgent);
    const startingGeneration=this.generation;let selectedStream;
    // Never silently use the Quest avatar/selfie camera as a world camera.
    try{
      selectedStream=await navigator.mediaDevices.getUserMedia({audio:false,video:{
        ...(quest?{facingMode:{exact:'environment'}}:{}),width:{ideal:1280},height:{ideal:960},frameRate:{ideal:30,max:30}}});
    }catch(e){
      if(quest&&['OverconstrainedError','NotFoundError'].includes(e.name))
        throw new Error('Quest Browser is not exposing its environment camera. In Quest settings, open Browser → Permissions → Headset cameras and allow access, then retry.');
      if(e.name==='NotAllowedError')throw new Error('Camera permission was not granted. Allow this page to use the camera, then retry.');
      throw new Error(`Camera could not start: ${e.message||e.name}`);
    }
    if(!this.running||startingGeneration!==this.generation){selectedStream.getTracks().forEach(t=>t.stop());throw new Error('Tracking startup was cancelled.');}
    this.stream=selectedStream;
    const video=document.getElementById('camera');video.srcObject=this.stream;await video.play();
    const settings=this.stream.getVideoTracks()[0].getSettings();
    if(quest&&settings.facingMode&&settings.facingMode!=='environment')throw new Error('The camera is not an environment camera');
    const fingerprint=JSON.stringify({device:settings.deviceId,width:settings.width,height:settings.height,facing:settings.facingMode,ua:navigator.userAgent});
    this.cameraKey=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(fingerprint)))).map(x=>x.toString(16).padStart(2,'0')).join('');
    const canvas=document.createElement('canvas'),ctx=canvas.getContext('2d',{alpha:false});
    const generation=this.generation;let busy=false,lastSent=0;
    const send=(source,frameAt)=>{
      const now=performance.now();
      if(!this.running||generation!==this.generation)return;
      if(busy||!this.link?.nonce||now-lastSent<55)return;
      if(frameAt<this.link.nonceAt)return;
      if(document.hidden&&this.session?.visibilityState!=='visible'){this.link.send({valid:false,reason:'Camera page hidden',capture_age_ms:0});return;}
      if(now-frameAt>100){this.link.send({valid:false,reason:'Camera frame is late',capture_age_ms:0});return;}
      busy=true;lastSent=now;
      try{
        const width=source.displayWidth||source.videoWidth,height=source.displayHeight||source.videoHeight;
        const scale=Math.min(1,1280/width,1024/height);
        canvas.width=Math.round(width*scale);canvas.height=Math.round(height*scale);ctx.drawImage(source,0,0,canvas.width,canvas.height);
        // Quest can defer toBlob for hundreds of milliseconds. Encode this
        // newest frame directly instead of waiting on an idle callback.
        const jpeg=canvas.toDataURL('image/jpeg',.85).split(',')[1];
        if(!this.running||generation!==this.generation)return;
        this.link.send({valid:true,kind:'markers',jpeg,camera_key:this.cameraKey,sampled_at_ms:frameAt,capture_age_ms:performance.now()-frameAt});
      }catch(e){this.onStatus(e.message);}finally{busy=false;}
    };
    if(typeof MediaStreamTrackProcessor==='function'){
      // Read camera frames even when the browser compositor is not drawing <video>.
      // A one-frame queue prevents a backlog; timestamps must keep advancing.
      const reader=new MediaStreamTrackProcessor({track:this.stream.getVideoTracks()[0],maxBufferSize:1}).readable.getReader();
      this.cameraReader=reader;
      (async()=>{
        let lastStamp=-Infinity,lastRead=null,clockOffset=null;
        try{
          while(this.running&&generation===this.generation){
            const {value:frame,done}=await reader.read();if(done)break;
            try{
              const now=performance.now(),stamp=frame.timestamp/1000;
              if(!Number.isFinite(stamp)||stamp<=lastStamp)continue;
              // Camera timestamps use a different clock origin on Quest. The
              // minimum observed delivery offset measures added queueing only,
              // not physical exposure latency. Requalify after a delivery gap.
              const requalify=lastRead===null||now-lastRead>100;
              lastStamp=stamp;lastRead=now;
              if(requalify){clockOffset=now-stamp;continue;}
              clockOffset=Math.min(clockOffset,now-stamp);
              void send(frame,stamp+clockOffset);
            }finally{frame.close();}
          }
        }catch(e){if(this.running&&generation===this.generation)this.onStatus('Camera frames stopped: '+e.message);}
        finally{reader.releaseLock();if(this.cameraReader===reader)this.cameraReader=null;}
        if(this.running&&generation===this.generation)await this.stop();
      })();
    }else{
      if(!video.requestVideoFrameCallback)throw new Error('This browser does not expose camera frames');
      const tick=(now,meta)=>{
        if(!this.running||generation!==this.generation)return;
        video.requestVideoFrameCallback(tick);
        void send(video,Math.min(performance.now(),meta.expectedDisplayTime));
      };
      video.requestVideoFrameCallback(tick);
    }
    this.stream.getVideoTracks()[0].addEventListener('ended',()=>this.stop());
    this.onStatus('Camera active. Show marker 10 and the fixed reference.');
  }
  async xr(){
    if(this.session)return;
    const startingGeneration=this.generation;
    if(!navigator.xr||!await navigator.xr.isSessionSupported('immersive-ar'))throw new Error('Open this app in Quest Browser for passthrough and tracked inputs');
    const hand=this.mode==='hand';
    const session=await navigator.xr.requestSession('immersive-ar',{requiredFeatures:['local',...(hand?['hand-tracking']:[])],optionalFeatures:['local-floor']});
    if(!this.running||startingGeneration!==this.generation){await session.end();throw new Error('Tracking startup was cancelled.');}
    this.session=session;
    const renderer=new THREE.WebGLRenderer({alpha:true,antialias:true});renderer.setClearColor(0,0);renderer.xr.enabled=true;renderer.xr.setReferenceSpaceType('local');
    document.getElementById('xr').append(renderer.domElement);await renderer.xr.setSession(session);
    const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera();const axes=new THREE.AxesHelper(.08);axes.visible=false;scene.add(axes);
    const space=renderer.xr.getReferenceSpace();space.addEventListener('reset',()=>{this.link?.reset('Tracking origin changed; set guard again');});
    session.addEventListener('visibilitychange',()=>{if(session.visibilityState!=='visible')this.link?.send({valid:false,reason:'Headset view hidden',capture_age_ms:0});});
    session.addEventListener('end',()=>{renderer.setAnimationLoop(null);renderer.dispose();renderer.domElement.remove();this.session=null;if(this.mode!=='markers')this.stop();else this.onStop();});
    renderer.setAnimationLoop((time,frame)=>{
      if(!frame)return;
      if(this.mode!=='markers'){
        const input=Array.from(session.inputSources).find(s=>s.handedness==='right'&&(hand?!!s.hand:!!s.gripSpace&&!s.hand));
        const inputSpace=hand?input?.hand?.get('wrist'):input?.gripSpace;
        let pose=null;try{pose=inputSpace&&(hand?frame.getJointPose(inputSpace,space):frame.getPose(inputSpace,space));}catch{}
        const head=frame.getViewerPose(space);
        const visible=session.visibilityState==='visible'&&pose&&head;
        axes.visible=!!visible;
        if(visible){
          const p=pose.transform.position,q=pose.transform.orientation,h=head.transform.orientation;
          axes.position.set(p.x,p.y,p.z);axes.quaternion.set(q.x,q.y,q.z,q.w);
          this.link?.send({valid:true,kind:this.mode,position:[p.x,p.y,p.z],orientation:[q.x,q.y,q.z,q.w],head_orientation:[h.x,h.y,h.z,h.w],capture_age_ms:0});
        }else this.link?.send({valid:false,reason:'Selected right-hand input not tracked',capture_age_ms:0});
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
