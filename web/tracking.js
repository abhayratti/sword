import * as THREE from './vendor/three.module.js';

// One sample in flight: send, wait for the server's reply, send the newest next.
class Link {
  constructor(key,onStatus,onClosed){this.key=key;this.onStatus=onStatus;this.onClosed=onClosed;this.ws=null;this.ready=false;this.epoch=crypto.randomUUID();this.closed=false;this.command=null;}
  async open(){
    this.ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/tracking?key=${encodeURIComponent(this.key)}`);
    await new Promise((resolve,reject)=>{
      this.ws.onmessage=e=>{const r=JSON.parse(e.data);this.ready=true;if(r.ready){this.onStatus('Tracking link connected');resolve();}else if(r.error||r.message)this.onStatus(r.error||r.message);};
      this.ws.onerror=()=>reject(new Error('Could not open tracking. Another device may already be streaming.'));
      this.ws.onclose=()=>{const expected=this.closed;this.closed=true;this.ready=false;if(!expected)this.onClosed();};
    });
  }
  send(data){
    if(this.closed||this.ws?.readyState!==1||!this.ready)return false;
    const command=this.command;this.command=null;this.ready=false;
    this.ws.send(JSON.stringify({...data,command,epoch:this.epoch}));return true;
  }
  reset(reason){this.epoch=crypto.randomUUID();this.send({reset:true,reason});}
  close(){this.closed=true;this.ready=false;this.ws?.close();}
}

// Right Touch controller gamepad buttons.
const BUTTON_A=4,BUTTON_B=5;

export class Tracker {
  constructor(onStatus,onStop){this.onStatus=onStatus;this.onStop=onStop;this.link=null;this.session=null;this.running=false;}
  async start(key){
    if(this.running)await this.stop();this.running=true;
    this.link=new Link(key,this.onStatus,()=>{this.stop().then(()=>this.onStatus('Tracking link lost. Start tracking to reconnect.'));});
    try{
      this.onStatus('Opening passthrough…');
      await this.xr();  // first, while the button press still counts as a user gesture
      await this.link.open();
    }catch(e){await this.stop();throw e;}
  }
  async xr(){
    if(!navigator.xr||!await navigator.xr.isSessionSupported('immersive-ar'))throw new Error('Open this app in Quest Browser on the headset');
    const session=await navigator.xr.requestSession('immersive-ar',{requiredFeatures:['local']});
    if(!this.running){await session.end();throw new Error('Tracking startup was cancelled.');}
    this.session=session;
    const renderer=new THREE.WebGLRenderer({alpha:true,antialias:true});renderer.setClearColor(0,0);renderer.xr.enabled=true;renderer.xr.setReferenceSpaceType('local');
    document.getElementById('xr').append(renderer.domElement);await renderer.xr.setSession(session);
    const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera();const axes=new THREE.AxesHelper(.08);axes.visible=false;scene.add(axes);
    const space=renderer.xr.getReferenceSpace();space.addEventListener('reset',()=>{this.link?.reset('Tracking origin changed; set guard again');});
    session.addEventListener('end',()=>{renderer.setAnimationLoop(null);renderer.dispose();renderer.domElement.remove();this.session=null;this.stop();});
    const pressed={};
    renderer.setAnimationLoop((time,frame)=>{
      if(!frame)return;
      const input=Array.from(session.inputSources).find(s=>s.handedness==='right'&&s.gripSpace&&!s.hand);
      let pose=null;try{pose=input&&frame.getPose(input.gripSpace,space);}catch{}
      const head=frame.getViewerPose(space);
      for(const [index,command] of [[BUTTON_A,'guard'],[BUTTON_B,'blade_up']]){
        const down=!!input?.gamepad?.buttons[index]?.pressed;
        if(down&&!pressed[index]&&this.link){this.link.command=command;input.gamepad.hapticActuators?.[0]?.pulse?.(.6,120);}
        pressed[index]=down;
      }
      axes.visible=!!(pose&&head);
      if(pose&&head){
        const p=pose.transform.position,q=pose.transform.orientation,h=head.transform.orientation;
        axes.position.set(p.x,p.y,p.z);axes.quaternion.set(q.x,q.y,q.z,q.w);
        this.link?.send({valid:true,position:[p.x,p.y,p.z],orientation:[q.x,q.y,q.z,q.w],head_orientation:[h.x,h.y,h.z,h.w]});
      }else this.link?.send({valid:false,reason:'Right controller not tracked'});
      renderer.render(scene,camera);
    });
  }
  async stop(){
    if(!this.running&&!this.session)return;this.running=false;
    this.link?.close();this.link=null;
    const session=this.session;this.session=null;if(session)try{await session.end();}catch{}
    this.onStatus('Tracking stopped');this.onStop();
  }
}
