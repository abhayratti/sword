import {Tracker} from './tracking.js';
const $=id=>document.getElementById(id);let state=null,key=null,renderedFrame=null,frameBusy=false,pollTimer;
const canvas=$('scene'),ctx=canvas.getContext('2d');const colors={human:'#79e2cf',robot:'#edb775',anchor:'#c5ced4'};
const tracker=new Tracker(text=>{$('device-message').textContent=text;},()=>{action('stop').catch(()=>{});$('track').firstChild.textContent='Start tracking on this device ';});
for(let i=1;i<=6;i++){const node=document.createElement('div');node.className='joint';node.innerHTML=`<label>J${i}</label><strong>—</strong><em>No feedback</em>`;$('joints').append(node);}
function notice(text){$('notice').textContent=text;$('notice').hidden=!text;}
async function action(name){
  if(!key)throw new Error('The local app is disconnected');
  const res=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Sword-Key':key},body:JSON.stringify({action:name})});
  const data=await res.json();if(!res.ok)throw new Error(data.error||data.message||`Action failed (${res.status})`);
  return data;
}
async function perform(name){try{const result=await action(name);notice(result.message);}catch(e){notice(e.message);}}
for(const [id,name] of [['stop','stop'],['connect','connect'],['home','home'],['mirror','start'],['guard','guard'],['capture','capture_board'],['calibrate','calibrate_camera'],['reset-camera','reset_camera']])$(id).addEventListener('click',()=>perform(name));
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!e.repeat&&!['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)){e.preventDefault();perform('stop');}});
$('track').addEventListener('click',async()=>{
  try{
    if(tracker.running){await tracker.stop();notice('Tracking stopped.');return;}
    if(!key)throw new Error('Wait for the Mac connection');
    notice('');
    $('track').disabled=true;await tracker.start($('source').value,key);
    $('track').firstChild.textContent='Stop tracking on this device ';
    notice('Tracking started. Set the guard when your normal pose is visible.');
  }catch(e){notice(e.message||e.name||'Tracking could not start');}finally{$('track').disabled=false;}
});
$('passthrough').addEventListener('click',async()=>{try{if(!tracker.running)throw new Error('Start tracking first');await tracker.xr();}catch(e){notice(e.message);}});
$('source').addEventListener('change',async()=>{
  if(tracker.running)await tracker.stop();
  const help={markers:'Directly detects the markers on both swords. A fixed reference removes headset motion.',controller:'Uses the right controller grip pose. Mount it rigidly; this does not observe foam-blade bending.',hand:'Tracks the right wrist, not the sword itself. A changed grip changes the inferred blade direction.'};
  $('source-help').textContent=help[$('source').value];
});
async function drawFrame(vision){
  if(frameBusy||!vision.frame_id||renderedFrame===vision.frame_id)return;
  frameBusy=true;
  try{
    const res=await fetch('/api/frame?id='+vision.frame_id);if(!res.ok)return;
    const bitmap=await createImageBitmap(await res.blob());canvas.width=vision.width;canvas.height=vision.height;ctx.drawImage(bitmap,0,0);bitmap.close();
    for(const object of vision.objects){
      const p=object.corners;ctx.strokeStyle=colors[object.name];ctx.lineWidth=3;ctx.beginPath();p.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.closePath();ctx.stroke();
      if(object.blade_pixels){const [base,tip]=object.blade_pixels;ctx.setLineDash([8,6]);ctx.beginPath();ctx.moveTo(...base);ctx.lineTo(...tip);ctx.stroke();ctx.setLineDash([]);}
      const label=(object.id==='board'?'REFERENCE BOARD':object.name.toUpperCase()+' '+object.id)+(object.pose?' · 3D':' · 2D');ctx.font='bold 17px system-ui';
      const x=Math.min(canvas.width-ctx.measureText(label).width-14,Math.max(5,p[0][0])),y=Math.max(30,p[0][1]-8);
      ctx.fillStyle='#10251fe8';ctx.fillRect(x-5,y-23,ctx.measureText(label).width+12,30);ctx.fillStyle=colors[object.name];ctx.fillText(label,x,y);
    }
    renderedFrame=vision.frame_id;$('empty').hidden=true;
  }catch(e){notice('Camera preview: '+e.message);}finally{frameBusy=false;}
}
function update(s){
  state=s;key=s.key;$('link-light').className='good';$('link-state').textContent='Mac connected';
  const t=s.tracking,r=s.robot,cam=s.vision.camera||{};
  $('pose-quality').textContent=t.fresh?(t.kind==='hand'?'HAND ESTIMATE':'LIVE 3D POSE'):'NO LIVE 3D POSE';
  $('guard').disabled=!t.fresh||['running','holding','homing','working'].includes(r.state);
  $('guard-status').textContent=t.guard_set?'Guard set. Move naturally from this position.':'Hold your normal guard, then set it here. No touching the blade base or tip.';
  const connected=!!r.firmware&&['connected','stopped'].includes(r.state);
  $('home').disabled=!connected;$('mirror').disabled=!connected||!t.fresh||!t.guard_set;
  $('connect').disabled=['working','running','holding','homing'].includes(r.state);
  $('connect').textContent=['disconnected','fault'].includes(r.state)?'Connect':'Reconnect';
  $('robot-status').textContent=r.reason;$('mirror').firstChild.textContent=['running','holding'].includes(r.state)?'Mirroring session open ':'Start mirroring ';
  $('age').textContent=t.fresh?`${t.age_ms} ms pose age estimate · ${t.kind}`:t.reason;
  $('view-title').textContent=t.kind==='markers'?'CAMERA VIEW':t.kind==='controller'?'CONTROLLER INPUT':'HAND INPUT / ESTIMATE';
  const nodes=Array.from($('joints').children);
  nodes.forEach((node,i)=>{node.querySelector('strong').textContent=Number.isFinite(r.joints?.[i])?`${r.joints[i].toFixed(1)}°`:'—';node.querySelector('em').textContent=Number.isFinite(r.command?.[i])?`${r.command[i].toFixed(1)}°/s cmd`:'No live command';});
  $('joint-description').textContent=r.joints?`Encoder feedback · ${r.feedback_age_ms??'—'} ms old`:'Waiting for robot encoders';
  $('camera-status').textContent=cam.calibrated?'Calibration saved':`${cam.views||0} / ${cam.needed||12} views`;
  $('board-count').textContent=`${cam.views||0}/12`;
  $('calibration-detail').textContent=cam.calibrated?`${cam.method==='factory_intrinsics_checked'?'Factory lens':`Fit ${cam.rms_px.toFixed(2)} px`} · validation ${Math.max(...cam.validation_px).toFixed(2)} px`:'Use the second page of the print kit; squares must measure 25 mm.';
  $('calibrate').disabled=(cam.views||0)<12;
  for(const [name,id] of [['human','human-state'],['robot','robot-marker-state'],['anchor','anchor-state']]){
    const object=s.vision.objects?.find(x=>x.name===name);$(id).textContent=s.frame_age_ms>500?'stale':object?.pose?'3D':object?'2D':'—';
  }
  if(s.frame_age_ms!==null&&s.frame_age_ms<500)drawFrame(s.vision);
  else if(renderedFrame!==null){ctx.clearRect(0,0,canvas.width,canvas.height);renderedFrame=null;$('empty').hidden=false;}
  if(r.speed_caps)$('motion-detail').textContent=`Configured joint caps: ${r.speed_caps.join(' / ')}°/s. One workspace, no SF1 prior, no session timer. Tracking loss holds; Stop and controller faults end the session. Hardware alignment and latency remain to be verified.`;
}
async function poll(){
  try{const res=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1500)});if(!res.ok)throw new Error();update(await res.json());}
  catch{$('link-light').className='';$('link-state').textContent='Mac disconnected';key=null;$('mirror').disabled=true;$('guard').disabled=true;}
  pollTimer=setTimeout(poll,150);
}
poll();
