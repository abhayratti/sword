import {Tracker} from './tracking.js';
const $=id=>document.getElementById(id);let key=null;
const canvas=$('scene'),ctx=canvas.getContext('2d');
const LABELS={guard:'Guard',high:'High',high_left:'High left',high_right:'High right',mid_left:'Mid left',mid_right:'Mid right'};
// Grid cell for each block, as [row, column] seen from behind the robot (robot's left on the left).
const CELLS={high_left:[0,0],high:[0,1],high_right:[0,2],mid_left:[1,0],guard:[1,1],mid_right:[1,2]};
const tracker=new Tracker(text=>{$('device-message').textContent=text;},()=>{action('stop').catch(()=>{});$('track').firstChild.textContent='Start tracking on this headset ';});
for(let i=1;i<=6;i++){const node=document.createElement('div');node.className='joint';node.innerHTML=`<label>J${i}</label><strong>—</strong><em>No feedback</em>`;$('joints').append(node);}
function notice(text){$('notice').textContent=text;$('notice').hidden=!text;}
async function action(name,value){
  if(!key)throw new Error('The local app is disconnected');
  const res=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Sword-Key':key},body:JSON.stringify({action:name,value})});
  const data=await res.json();if(!res.ok)throw new Error(data.error||`Action failed (${res.status})`);
  return data;
}
async function perform(name,value){try{notice((await action(name,value)).message);}catch(e){notice(e.message);}}
for(const [id,name] of [['stop','stop'],['connect','connect'],['home','home'],['start','start'],['guard','guard'],['blade','blade_up']])$(id).addEventListener('click',()=>perform(name));
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!e.repeat){e.preventDefault();perform('stop');}});
$('track').addEventListener('click',async()=>{
  try{
    if(tracker.running){await tracker.stop();notice('Tracking stopped.');return;}
    if(!key)throw new Error('Wait for the Mac connection');
    $('track').disabled=true;await tracker.start(key);
    $('track').firstChild.textContent='Stop tracking on this headset ';
  }catch(e){notice(e.message||'Tracking could not start');}finally{$('track').disabled=false;}
});
let freedrive=false;
$('freedrive').addEventListener('click',()=>perform('freedrive',!freedrive));
let blockRows=null;
function buildBlocks(names){
  blockRows={};
  for(const name of names){
    const row=document.createElement('div');row.className='block';
    row.innerHTML=`<span>${LABELS[name]}</span><em>—</em><button>Save</button><button>Move here</button>`;
    const [save,move]=row.querySelectorAll('button');
    save.addEventListener('click',()=>perform('save_block',name));move.addEventListener('click',()=>perform('goto_block',name));
    $('blocks').append(row);blockRows[name]=row;
  }
}
function draw(s){
  const w=canvas.width,h=canvas.height,z=s.zones,t=s.threat,range=.45,scale=Math.min(w,h)/(2*range);
  const X=x=>w/2-x*scale,Y=y=>h/2-y*scale;  // offset x is toward the robot's left: draw it on the left
  ctx.fillStyle='#10251f';ctx.fillRect(0,0,w,h);
  const xs=[0,X(z.side_m),X(-z.side_m),w],ys=[0,Y(z.height_m),Y(-z.height_m),h];
  const cell=CELLS[t.zone];
  for(let r=0;r<3;r++)for(let c=0;c<3;c++){
    const active=cell&&cell[0]===r&&cell[1]===c;
    ctx.fillStyle=active?(t.strike?'#c0503a':'#79e2cf55'):'transparent';ctx.fillRect(xs[c],ys[r],xs[c+1]-xs[c],ys[r+1]-ys[r]);
    ctx.strokeStyle='#c5ced455';ctx.lineWidth=2;ctx.strokeRect(xs[c],ys[r],xs[c+1]-xs[c],ys[r+1]-ys[r]);
    const name=Object.keys(CELLS).find(n=>CELLS[n][0]===r&&CELLS[n][1]===c)||'guard';
    ctx.fillStyle='#c5ced4';ctx.font='bold 26px system-ui';ctx.textAlign='center';
    ctx.fillText(LABELS[name],(xs[c]+xs[c+1])/2,ys[r]+40);
  }
  if(t.offset&&s.tracking.guard_set){
    ctx.strokeStyle='#edb775';ctx.lineWidth=3;ctx.beginPath();ctx.arc(X(t.predicted[0]),Y(t.predicted[1]),18,0,7);ctx.stroke();
    ctx.fillStyle='#ffffff';ctx.beginPath();ctx.arc(X(t.offset[0]),Y(t.offset[1]),12,0,7);ctx.fill();
  }
}
function update(s){
  key=s.key;$('link-light').className='good';$('link-state').textContent='Mac connected';
  const t=s.tracking,r=s.robot;
  if(!blockRows)buildBlocks(s.blocks);
  for(const [name,row] of Object.entries(blockRows))row.querySelector('em').textContent=s.taught.includes(name)?'taught':'—';
  $('teach-status').textContent=`${s.taught.length} / ${s.blocks.length} taught`;
  $('tracking-state').textContent=t.fresh?'live':'—';$('blade-state').textContent=t.blade_calibrated?'set':'—';$('guard-state').textContent=t.guard_set?'set':'—';
  $('zone-label').textContent=(s.threat.strike?'STRIKE · ':'')+(LABELS[s.threat.zone]||s.threat.zone).toUpperCase();
  $('guard-status').textContent=!t.blade_calibrated?'Point the sword straight up and press B on the controller.':!t.guard_set?'Hold your normal guard and press A on the controller.':'Ready. Press A again any time to re-centre your guard.';
  const connected=!!r.firmware&&!['disconnected','working'].includes(r.state);
  freedrive=r.state==='freedrive';$('freedrive').textContent=freedrive?'Freedrive off':'Freedrive on';$('freedrive').disabled=!connected;
  for(const row of Object.values(blockRows))row.querySelectorAll('button').forEach(b=>b.disabled=!connected);
  $('home').disabled=!connected;$('start').disabled=!connected||!t.guard_set;
  $('connect').textContent=['disconnected','fault'].includes(r.state)?'Connect':'Reconnect';
  $('robot-status').textContent=r.reason;$('start').firstChild.textContent=r.state==='running'?'Blocking… ':'Start blocking ';
  $('age').textContent=t.fresh?`${t.age_ms} ms since last controller pose`:t.reason;
  Array.from($('joints').children).forEach((node,i)=>{node.querySelector('strong').textContent=Number.isFinite(r.joints?.[i])?`${r.joints[i].toFixed(1)}°`:'—';node.querySelector('em').textContent=Number.isFinite(r.command?.[i])?`${r.command[i].toFixed(1)}°/s cmd`:'No live command';});
  $('joint-description').textContent=r.joints?`Encoder feedback · ${r.feedback_age_ms??'—'} ms old`:'Waiting for robot encoders';
  draw(s);
}
async function poll(){
  try{const res=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1500)});if(!res.ok)throw new Error();update(await res.json());}
  catch{$('link-light').className='';$('link-state').textContent='Mac disconnected';key=null;$('start').disabled=true;}
  setTimeout(poll,100);
}
poll();
