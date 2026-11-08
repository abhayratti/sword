import * as THREE from './vendor/three.module.js';

// One sample in flight: send, wait for the server's reply, send the newest next.
class Link {
  constructor(key,onStatus,onClosed){this.key=key;this.onStatus=onStatus;this.onClosed=onClosed;this.ws=null;this.ready=false;this.epoch=crypto.randomUUID();this.closed=false;this.command=null;this.hud=null;this.flash=null;}
  async open(){
    this.ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/tracking?key=${encodeURIComponent(this.key)}`);
    await new Promise((resolve,reject)=>{
      this.ws.onmessage=e=>{
        const r=JSON.parse(e.data);this.ready=true;
        if(r.ready){this.onStatus('Tracking link connected');resolve();return;}
        if(r.hud)this.hud=r.hud;
        if(r.error||r.message){this.flash={text:r.error||r.message,error:!!r.error,until:performance.now()+2500};this.onStatus(r.error||r.message);}
      };
      this.ws.onerror=()=>reject(new Error('Could not open tracking. Another device may already be streaming.'));
      this.ws.onclose=()=>{const expected=this.closed;this.closed=true;this.ready=false;if(!expected)this.onClosed();};
    });
  }
  queue(command){if(this.command!=='stop')this.command=command;}  // Stop is never overwritten
  send(data){
    if(this.closed||this.ws?.readyState!==1||!this.ready)return false;
    const command=this.command;this.command=null;this.ready=false;
    this.ws.send(JSON.stringify({...data,command,epoch:this.epoch}));return true;
  }
  reset(reason){this.epoch=crypto.randomUUID();this.send({reset:true,reason});}
  close(){this.closed=true;this.ready=false;this.ws?.close();}
}

// xr-standard gamepad: button 3 is the thumbstick click, 4 is A (right) / X (left), 5 is B (right) / Y (left).
const BUTTONS={right:{3:'calibrate_zones',4:'guard',5:'blade_up'},left:{4:'start',5:'stop'}};
const ZONE_NAMES={guard:'GUARD',high:'HIGH',high_left:'HIGH LEFT',high_right:'HIGH RIGHT',mid_left:'MID LEFT',mid_right:'MID RIGHT'};

// Status panel shown inside passthrough, just below the centre of view.
class Hud {
  constructor(scene){
    this.canvas=document.createElement('canvas');this.canvas.width=1024;this.canvas.height=384;
    this.ctx=this.canvas.getContext('2d');this.texture=new THREE.CanvasTexture(this.canvas);
    this.mesh=new THREE.Mesh(new THREE.PlaneGeometry(.48,.18),new THREE.MeshBasicMaterial({map:this.texture,transparent:true,depthTest:false}));
    this.mesh.renderOrder=10;scene.add(this.mesh);this.last='';
  }
  lines(link){
    const h=link?.hud,f=link?.flash&&performance.now()<link.flash.until?link.flash:null;
    let main,color='#ffffff';
    if(!link||link.closed){main='Not connected to the laptop';color='#ff8a7a';}
    else if(!h)main='Connecting…';
    else if(!h.fresh){main='Controller not visible: keep its face uncovered';color='#ff8a7a';}
    else if(!h.blade)main='Point the sword straight up, press B';
    else if(!h.guard)main='Hold your guard facing the robot, press A';
    else if(h.calibrating){main=`Show your ${ZONE_NAMES[h.calibrating]} wind-up, press A`;color='#edb775';}
    else if(!h.connected)main='Connect the robot on the laptop';
    else if(h.robot==='running'&&(h.reason||'').startsWith('Attacking')){main=`INCOMING · ${h.reason.slice(11).replace('_',' ').toUpperCase()}`;color='#ff8a7a';}
    else if(h.robot==='running'){main=`BLOCKING · ${ZONE_NAMES[h.zone]||h.zone}`;color=h.strike?'#ff8a7a':'#79e2cf';}
    else main='Ready: press X (left controller) to start';
    let sub=f?f.text:(h?.reason||'');
    if(!f&&h?.calibrating)sub=`Zone calibration · step ${h.step+1} of 6`;
    else if(!f&&h?.guard&&!h.zones&&h.robot!=='running')sub='Tip: click the right stick to calibrate your zones';
    return [main,color,sub,f?.error?'#ff8a7a':'#c5ced4'];
  }
  draw(link){
    const [main,color,sub,subColor]=this.lines(link),key=main+color+sub+subColor;
    if(key===this.last)return;this.last=key;
    const c=this.ctx,w=this.canvas.width,h=this.canvas.height;
    c.clearRect(0,0,w,h);c.fillStyle='#10251fd9';c.beginPath();c.roundRect(0,0,w,h,36);c.fill();
    c.textAlign='center';c.fillStyle=color;c.font='bold 62px system-ui';c.fillText(main,w/2,130,w-60);
    c.fillStyle=subColor;c.font='40px system-ui';c.fillText(sub,w/2,215,w-60);
    c.fillStyle='#8fa19b';c.font='34px system-ui';c.fillText('A guard · B blade up · R-stick zones · X start · Y stop',w/2,320,w-60);
    this.texture.needsUpdate=true;
  }
  place(head){  // follow the head: 0.9 m ahead, a little below eye level
    const p=head.transform.position,q=head.transform.orientation;
    const quat=new THREE.Quaternion(q.x,q.y,q.z,q.w);
    this.mesh.position.set(p.x,p.y,p.z).add(new THREE.Vector3(0,-.28,-.9).applyQuaternion(quat));
    this.mesh.quaternion.copy(quat);
  }
}

export class Tracker {
  constructor(onStatus,onStop){this.onStatus=onStatus;this.onStop=onStop;this.link=null;this.session=null;this.running=false;}
  async start(key){
    if(this.running)await this.stop();this.running=true;
    this.link=new Link(key,this.onStatus,()=>{this.stop().then(()=>this.onStatus('Tracking link lost. Tap Enter to reconnect.'));});
    try{
      this.onStatus('Opening passthrough…');
      await this.xr();  // first, while the tap still counts as a user gesture
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
    const hud=new Hud(scene);
    const space=renderer.xr.getReferenceSpace();space.addEventListener('reset',()=>{this.link?.reset('Tracking origin changed; set guard again');});
    session.addEventListener('end',()=>{renderer.setAnimationLoop(null);renderer.dispose();renderer.domElement.remove();this.session=null;this.stop();});
    const pressed={};
    renderer.setAnimationLoop((time,frame)=>{
      if(!frame)return;
      const sources=Array.from(session.inputSources).filter(s=>s.gripSpace&&!s.hand);
      for(const source of sources){
        for(const [index,command] of Object.entries(BUTTONS[source.handedness]||{})){
          const id=source.handedness+index,down=!!source.gamepad?.buttons[index]?.pressed;
          if(down&&!pressed[id]&&this.link){this.link.queue(command);source.gamepad.hapticActuators?.[0]?.pulse?.(.6,120);}
          pressed[id]=down;
        }
      }
      const input=sources.find(s=>s.handedness==='right');
      let pose=null;try{pose=input&&frame.getPose(input.gripSpace,space);}catch{}
      const head=frame.getViewerPose(space);
      if(head)hud.place(head);
      hud.draw(this.link);
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
    this.onStatus('Left passthrough, so the robot was stopped. Tap Enter to go back in.');this.onStop();
  }
}
