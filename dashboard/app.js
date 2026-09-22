const $ = id => document.getElementById(id);
let view = 'live', current = null, busy = false;
const words = {idle:'Waiting for detection',target:'Trigger preset selected',returning:'Returning to home preset'};
function select(next){view=next;for(const key of ['live','test']){const active=key===view;$(key+'-tab').classList.toggle('selected',active);$(key+'-tab').setAttribute('aria-pressed',String(active));}renderFrame();}
$('live-tab').onclick=()=>select('live');$('test-tab').onclick=()=>select('test');
function renderFrame(){
 if(!current)return;
 if(view==='test'&&$('frame').src.endsWith('/api/stream'))$('frame').removeAttribute('src');
 const demo=view==='test', t=current.test.telemetry;
 const available=demo ? Boolean(t && t.frames>0) : current.frame_age!==null;
 $('frame').hidden=!available;$('empty').hidden=available;
 if(available){const want=demo?'/api/test/frame?t='+Date.now():'/api/stream';
  // Re-assigning a live MJPEG src would restart the stream every second.
  if(demo||!$('frame').src.endsWith('/api/stream'))$('frame').src=want;
  $('frame').alt=demo?'Generated test image, not a real camera feed':'Live ROS camera stream';}
 if(!available){$('empty').querySelector('h3').textContent=demo?'Try the software test':'Waiting for the camera';$('empty').querySelector('p').textContent=demo?'Run the system test to watch generated images pass through ROS 2.':'The camera’s Wi-Fi connection still needs setup. USB is currently providing a serial connection.';}
 $('image-label').textContent=demo?'GENERATED TEST IMAGE · NO PHYSICAL CAMERA':current.camera_live?'LIVE CAMERA':available?'LAST FRAME · FEED STALE':'LIVE SOURCE · NO FRAMES';
 $('frame-detail').textContent=demo?(t?`${t.frames} test frames · ${words[t.phase]||t.phase}`:'Isolated test · ROS domain 87'):`${current.image_count} frames received · /camera/image_raw`;
 const detected=demo?t?.red:current.red;
 $('detection').textContent=detected===null||detected===undefined?'Detection —':detected?'Colour detected':'No colour detected';
}
function render(s){current=s;$('connection').textContent='Pi connected';$('connection').className='pill ok';$('offline').hidden=true;
 $('ros').textContent=s.ros?'Observer online':'Unavailable';$('ros').style.color=s.ros?'var(--mint)':'var(--amber)';$('ros-detail').textContent=s.ros?(s.arm_node?`${s.ros_distro} · domain ${s.ros_domain} · arm_poc connected`:`${s.ros_distro} · domain ${s.ros_domain} · arm_poc NOT running`):s.ros_error;
 $('camera').textContent=s.camera_live?'Receiving frames':s.frame_age!==null?'Feed stale':'Not connected';$('camera').style.color=s.camera_live?'var(--mint)':'var(--amber)';$('camera-detail').textContent=s.camera_live?'Live images arriving through ROS 2':s.frame_age!==null?`No frames for ${s.frame_age}s — the node latches a fault on a stale feed and must be restarted`:'Camera not connected yet';
 $('usb').textContent=`${s.usb.length} connected`;$('host').textContent=s.host;$('model').textContent=s.model;$('temp').textContent=s.temperature===null?'—':s.temperature+' °C';$('disk').textContent=s.disk_free_gb+' GB';
 const h=Math.floor(s.uptime_seconds/3600);$('uptime').textContent=h>=24?Math.floor(h/24)+'d '+h%24+'h':h+'h '+Math.floor(s.uptime_seconds%3600/60)+'m';
 $('devices').replaceChildren(...(s.usb.length?s.usb:['No serial adapters detected']).map(name=>{const li=document.createElement('li');li.textContent=name;return li;}));
 const t=s.test, running=t.status==='running';$('run-test').disabled=running||busy;$('run-test').firstChild.textContent=running?'Test running… ':'Run system test ';
 $('test-state').textContent={not_run:'Not run this session',running:'Running · no servo commands',passed:'Passed',failed:'Needs attention'}[t.status];$('test-state').className='pill '+(t.status==='passed'?'ok':t.status==='failed'?'warn':'');
 $('test-detail').textContent=running?(t.telemetry?`${t.telemetry.frames} images processed. ${words[t.telemetry.phase]||t.telemetry.phase}.`:'Starting logic and ROS checks…'):t.status==='passed'?'Logic and ROS checks passed. Physical camera and arm remain unverified.':t.status==='failed'?'Check the output below for the failure.':'Run a test to see current results.';
 $('output').textContent=t.output||(running?'Test in progress…':'No test has been run since the dashboard started.');$('updated').textContent='Updated '+new Date().toLocaleTimeString();renderFrame();renderArm(s);renderJog(s);renderNode(s);renderCalib(s);
}
async function refresh(){try{const r=await fetch('/api/status',{signal:AbortSignal.timeout(5000)});if(!r.ok)throw Error(r.status);render(await r.json());}catch(e){$('connection').textContent='Pi unreachable';$('connection').className='pill warn';$('offline').hidden=false;$('run-test').disabled=true;}finally{setTimeout(refresh,1000);}}
$('run-test').onclick=async()=>{busy=true;$('run-test').disabled=true;select('test');try{const r=await fetch('/api/test',{method:'POST',headers:{'X-Arm-Dashboard':'1'},signal:AbortSignal.timeout(5000)});if(!r.ok)throw Error((await r.json()).error||r.status);$('test-state').textContent='Starting…';}catch(e){$('test-detail').textContent='Could not start test: '+e.message;}finally{busy=false;}};
refresh();

/* --- Arm motion: self-contained 3D view of the commanded pose ---------------
   Projects a four-link chain onto a 2D canvas. Angles are commanded servo
   values, not measured feedback, and the joint-to-link mapping is an
   assumption until HARDWARE.md calibration records the real one. */
const AC=$('arm-canvas'), ACTX=AC.getContext('2d');
const LINK=[1.15,0.95,0.5], BASE_H=0.5, CAM=6.0, HOME=[90,90,90,90];
const JOINT_NAMES=['BASE YAW','SHOULDER','ELBOW','WRIST'];
let armView='live', armTarget=null, armShown=HOME.slice(),
    az=-0.62, el=0.34, spin=true, drag=null, W=0, H=0, FOC=0;

function armSelect(next){armView=next;for(const k of ['live','test']){const on=k===armView;
 $('arm-'+k+'-tab').classList.toggle('selected',on);$('arm-'+k+'-tab').setAttribute('aria-pressed',String(on));}}
$('arm-live-tab').onclick=()=>armSelect('live');$('arm-test-tab').onclick=()=>armSelect('test');

AC.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};spin=false;AC.setPointerCapture(e.pointerId);});
AC.addEventListener('pointermove',e=>{if(!drag)return;az+=(e.clientX-drag.x)*0.008;
 el=Math.max(-0.12,Math.min(1.15,el+(e.clientY-drag.y)*0.006));drag={x:e.clientX,y:e.clientY};});
for(const ev of ['pointerup','pointercancel'])AC.addEventListener(ev,()=>{drag=null;});

function proj(x,y,z){
 const ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
 const x1=x*ca-y*sa, y1=x*sa+y*ca;
 const d=y1*ce+z*se+CAM, s=FOC/Math.max(d,0.4);
 return [W/2+x1*s, H/2+H*0.16-(z*ce-y1*se)*s, d, s];
}
const shade=(d,base)=>{const t=Math.max(0.42,Math.min(1,(CAM+1.9-d)/3.4));
 return `rgba(${base[0]*t|0},${base[1]*t|0},${base[2]*t|0},1)`;};

function chain(a){
 const yaw=(a[0]-90)*Math.PI/180, pts=[[0,BASE_H]];
 let ang=0;
 for(let i=0;i<3;i++){ang+=(a[i+1]-90)*Math.PI/180;
  const p=pts[i];pts.push([p[0]+LINK[i]*Math.sin(ang),p[1]+LINK[i]*Math.cos(ang)]);}
 return pts.map(([r,z])=>[r*Math.cos(yaw),r*Math.sin(yaw),z]);
}

function drawArm(){
 const dpr=Math.min(window.devicePixelRatio||1,2), r=AC.getBoundingClientRect();
 if(!r.width){requestAnimationFrame(drawArm);return;}
 W=r.width;H=r.height;FOC=H*0.92;
 if(AC.width!==(W*dpr|0)||AC.height!==(H*dpr|0)){AC.width=W*dpr|0;AC.height=H*dpr|0;}
 ACTX.setTransform(dpr,0,0,dpr,0,0);
 ACTX.clearRect(0,0,W,H);
 if(spin)az+=0.0032;

 // Ease toward the commanded pose so preset jumps read as motion, roughly
 // matching the firmware's 1 degree per 20 ms ramp.
 const want=armTarget||HOME;
 for(let i=0;i<4;i++)armShown[i]+=(want[i]-armShown[i])*0.12;

 ACTX.lineWidth=1;
 for(let i=-4;i<=4;i++){const t=i*0.55;
  for(const seg of [[[t,-2.2,0],[t,2.2,0]],[[-2.2,t,0],[2.2,t,0]]]){
   const a=proj(...seg[0]),b=proj(...seg[1]);
   ACTX.strokeStyle=i===0?'#22303d':'#1a242e';
   ACTX.beginPath();ACTX.moveTo(a[0],a[1]);ACTX.lineTo(b[0],b[1]);ACTX.stroke();}}

 const live=Boolean(armTarget);
 const limb=live?[136,235,192]:[110,124,138], hub=live?[237,243,243]:[130,142,154];
 const pts=chain(armShown).map(p=>proj(...p));

 const base=proj(0,0,0);                                   // pedestal
 ACTX.fillStyle='rgba(20,29,39,.9)';ACTX.strokeStyle=shade(base[2],limb);ACTX.lineWidth=2;
 ACTX.beginPath();ACTX.ellipse(base[0],base[1],0.46*base[3],0.17*base[3],0,0,7);
 ACTX.fill();ACTX.stroke();
 ACTX.beginPath();ACTX.moveTo(base[0],base[1]);ACTX.lineTo(pts[0][0],pts[0][1]);
 ACTX.lineWidth=Math.max(6,0.13*base[3]);ACTX.lineCap='round';ACTX.stroke();

 for(let i=0;i<3;i++){const a=pts[i],b=pts[i+1],d=(a[2]+b[2])/2;
  ACTX.strokeStyle=shade(d,limb);ACTX.lineWidth=Math.max(4,(0.115-i*0.022)*a[3]);
  ACTX.lineCap='round';ACTX.beginPath();ACTX.moveTo(a[0],a[1]);ACTX.lineTo(b[0],b[1]);ACTX.stroke();}

 const tip=pts[3],wrist=pts[2];                            // gripper prongs
 const ux=tip[0]-wrist[0],uy=tip[1]-wrist[1],m=Math.hypot(ux,uy)||1;
 const nx=-uy/m*0.09*tip[3], ny=ux/m*0.09*tip[3];
 ACTX.strokeStyle=shade(tip[2],limb);ACTX.lineWidth=Math.max(3,0.035*tip[3]);
 for(const sgn of [1,-1]){ACTX.beginPath();ACTX.moveTo(tip[0]+nx*sgn,tip[1]+ny*sgn);
  ACTX.lineTo(tip[0]+nx*sgn+ux/m*0.22*tip[3],tip[1]+ny*sgn+uy/m*0.22*tip[3]);ACTX.stroke();}

 for(const p of pts){ACTX.fillStyle=shade(p[2],hub);
  ACTX.beginPath();ACTX.arc(p[0],p[1],Math.max(3,0.058*p[3]),0,7);ACTX.fill();}

 requestAnimationFrame(drawArm);
}

function renderArm(s){
 const demo=armView==='test', t=s.test.telemetry;
 armTarget=demo?(t&&t.joints?t.joints:null):(s.joints||null);
 $('arm-label').textContent=demo?(armTarget?'TEST POSE · GENERATED RUN':'NO TEST POSE · RUN THE SYSTEM TEST')
                                :(armTarget?'LIVE COMMANDED POSE':'NO POSE DATA · NODE NOT RUNNING');
 $('arm-detail').textContent=armTarget?(demo?`Test run · ${words[t.phase]||t.phase}`
   :`/arm/joint_states · ${words[s.phase]||'commanded pose'}`):'Showing home reference pose · arm_poc is not publishing';
 const show=armTarget||HOME;
 $('joint-readout').replaceChildren(...show.map((v,i)=>{
  const d=document.createElement('div'),n=document.createElement('span'),b=document.createElement('strong');
  n.textContent=JOINT_NAMES[i];b.textContent=Math.round(v)+'°';
  b.style.color=armTarget?'var(--text)':'var(--muted)';d.append(n,b);return d;}));
}
requestAnimationFrame(drawArm);

/* Fullscreen for either viewer. The canvas re-reads its own bounding box each
   frame, so the 3D view rescales without extra work. */
function toggleFull(el){
 if(document.fullscreenElement){document.exitFullscreen();return;}
 if(el.requestFullscreen)el.requestFullscreen().catch(()=>{});
}
$('frame-full').onclick=()=>toggleFull(document.querySelector('.viewer'));
$('arm-full').onclick=()=>toggleFull(document.querySelector('.armview'));
addEventListener('keydown',e=>{
 if(e.key==='f'&&!/^(INPUT|TEXTAREA)$/.test(document.activeElement.tagName))
  toggleFull(document.querySelector('.viewer'));
});

/* --- Manual jog ------------------------------------------------------------
   Sends absolute poses, never deltas, so a dropped request cannot accumulate
   into movement nobody asked for. The node expires a manual pose after its
   manual_timeout, so releasing simply means we stop refreshing. */
const JOINTS=[{n:'BASE',p:'D27'},{n:'SHOULDER',p:'D26'},{n:'ELBOW',p:'D25'},{n:'GRIPPER',p:'D33'}];
let JOG_MIN=[60,60,60,60], JOG_MAX=[120,120,120,120];
let jogPose=[90,90,90,90], jogOn=false, jogHold=null, jogLapse=null, jogBuilt=false;

function buildJog(){
 if(jogBuilt)return; jogBuilt=true;
 $('jog-grid').replaceChildren(...JOINTS.map((j,i)=>{
  const box=document.createElement('div');box.className='jog';
  const h=document.createElement('h3');h.textContent=`${j.n} · ${j.p}`;
  const v=document.createElement('div');v.className='val';v.id='jogv'+i;v.textContent='90°';
  const row=document.createElement('div');row.className='row';
  for(const d of [-5,-1,1,5]){
   const b=document.createElement('button');b.textContent=(d>0?'+':'')+d;
   b.onclick=()=>nudge(i,d);row.appendChild(b);
  }
  box.append(h,v,row);return box;}));
}
function nudge(i,d){
 jogPose[i]=Math.max(JOG_MIN[i],Math.min(JOG_MAX[i],jogPose[i]+d));
 $('jogv'+i).textContent=jogPose[i]+'°';
 sendJog();startHold();
}
async function sendJog(){
 try{
  const r=await fetch('/api/jog',{method:'POST',headers:{'X-Arm-Dashboard':'1',
   'Content-Type':'application/json'},body:JSON.stringify({pose:jogPose}),
   signal:AbortSignal.timeout(4000)});
  const j=await r.json();
  $('jog-reply').textContent=r.ok?`sent ${j.pose.join(' ')}`:('refused: '+(j.error||r.status));
 }catch(e){$('jog-reply').textContent='send failed: '+e.message;}
}
/* The node drops a manual pose that stops being refreshed, so hold it while
   the operator is still jogging, and let it lapse when they stop. */
function startHold(){
 if(jogHold)clearInterval(jogHold);
 if(jogLapse)clearTimeout(jogLapse);
 jogHold=setInterval(sendJog,700);
 // Restarted on every nudge, so continuous jogging never trips it; it only
 // fires after two idle minutes, releasing an arm nobody is driving.
 jogLapse=setTimeout(()=>{if(jogHold){clearInterval(jogHold);jogHold=null;
  $('jog-reply').textContent='idle — node returns home';}},120000);
}
$('jog-home').onclick=()=>{jogPose=[90,90,90,90];
 JOINTS.forEach((_,i)=>$('jogv'+i).textContent='90°');sendJog();startHold();};
$('jog-release').onclick=()=>{if(jogHold){clearInterval(jogHold);jogHold=null;}
 if(jogLapse){clearTimeout(jogLapse);jogLapse=null;}
 $('jog-reply').textContent='released — node returns home when the pose expires';};

function renderJog(s){
 buildJog();
 jogOn=Boolean(s.control);
 if(s.limits){JOG_MIN=s.limits.min;JOG_MAX=s.limits.max;}
 $('jog-state').textContent=jogOn?'Control enabled':'Read-only';
 $('jog-state').className='pill '+(jogOn?'warn':'');
 document.querySelector('.jog-panel').classList.toggle('locked',!jogOn);
 for(const b of document.querySelectorAll('.jogs button'))b.disabled=!jogOn;
 $('jog-home').disabled=!jogOn;$('jog-release').disabled=!jogOn;
 $('jog-keys').textContent=jogOn?keyLabel()+`   ·   range ${JOG_MIN[0]}–${JOG_MAX[0]}°`:'';
 if(!jogOn)$('jog-note').textContent='Control is disabled. Start the dashboard with '
  +'ARM_DASHBOARD_CONTROL=1 and the node with allow_manual:=true. Off by default because '
  +'this server has no authentication.';
}

/* --- Keyboard jogging -------------------------------------------------------
   Hold a key to jog continuously, release to stop. Keys are deliberately away
   from 'f' (fullscreen). Repeats send absolute poses at a fixed rate rather
   than one request per keypress, so holding a key cannot flood the node. */
const KEYMAP={
 arrowleft:[0,-1], arrowright:[0,1],   // base
 arrowup:[1,1],    arrowdown:[1,-1],   // shoulder
 w:[2,1],          s:[2,-1],           // elbow
 a:[3,-1],         d:[3,1],            // gripper
};
const held=new Map();
let repeatTimer=null, lastKeyJoint=null;

function keyLabel(){
 const rows=[['← →','BASE'],['↑ ↓','SHOULDER'],['W S','ELBOW'],
             ['A D','GRIPPER'],['H','home'],['Esc','release']];
 return rows.map(([k,v])=>`${k} ${v}`).join('   ·   ');
}
function pumpKeys(){
 if(!held.size){clearInterval(repeatTimer);repeatTimer=null;return;}
 for(const [joint,dir] of held.values()){
  jogPose[joint]=Math.max(JOG_MIN[joint],Math.min(JOG_MAX[joint],jogPose[joint]+dir));
  $('jogv'+joint).textContent=jogPose[joint]+'°';
  lastKeyJoint=joint;
 }
 sendJog();
}
function typing(){return /^(INPUT|TEXTAREA|SELECT)$/.test(document.activeElement.tagName);}
addEventListener('keydown',e=>{
 if(typing())return;
 const k=e.key.toLowerCase();
 if(k==='h'&&jogOn){e.preventDefault();$('jog-home').click();return;}
 if(k==='escape'&&jogOn){e.preventDefault();$('jog-release').click();return;}
 const m=KEYMAP[k];
 if(!m)return;
 e.preventDefault();
 if(!jogOn){$('jog-reply').textContent='control is disabled';return;}
 if(held.has(k))return;                  // ignore the OS auto-repeat
 held.set(k,m);
 if(!repeatTimer){pumpKeys();repeatTimer=setInterval(pumpKeys,120);}
 startHold();
});
addEventListener('keyup',e=>{held.delete(e.key.toLowerCase());});
addEventListener('blur',()=>held.clear());   // leaving the tab must stop motion

/* --- Node control ----------------------------------------------------------
   Restarting reopens the serial port and commands home, so it moves the arm.
   Buttons disable while a request is in flight: a second restart arriving
   mid-kill would race the first for the port. */
let nodeBusy=false;
async function nodeAction(action){
 if(nodeBusy)return;
 nodeBusy=true;
 $('node-restart').disabled=true;$('node-stop').disabled=true;
 $('node-reply').textContent=action==='restart'?'restarting…':'stopping…';
 try{
  const r=await fetch('/api/node/'+action,{method:'POST',
   headers:{'X-Arm-Dashboard':'1'},signal:AbortSignal.timeout(35000)});
  const j=await r.json();
  $('node-reply').textContent=r.ok?(j.detail||j.status):('failed: '+(j.error||r.status));
 }catch(e){$('node-reply').textContent='failed: '+e.message;}
 finally{nodeBusy=false;}
}
$('node-restart').onclick=()=>nodeAction('restart');
$('node-stop').onclick=()=>nodeAction('stop');

function renderNode(s){
 const up=Boolean(s.arm_node);
 $('node-state').textContent=up?'arm_poc running':'arm_poc not running';
 $('node-state').className='pill '+(up?'ok':'warn');
 if(!nodeBusy){
  $('node-restart').disabled=!s.control;
  $('node-stop').disabled=!s.control||!up;
  if(!s.control)$('node-reply').textContent='control disabled';
 }
}

/* --- Teaching pick positions -----------------------------------------------
   Pairs where the object appears with the joint angles that reach it. Only
   meaningful while the object is visible, so capture is disabled otherwise. */
async function calibAction(action){
 try{
  const r=await fetch('/api/calibration/'+action,{method:'POST',
   headers:{'X-Arm-Dashboard':'1'},signal:AbortSignal.timeout(6000)});
  const j=await r.json();
  $('calib-reply').textContent=r.ok?`${j.points.length} point(s) captured`
                                   :('failed: '+(j.error||r.status));
 }catch(e){$('calib-reply').textContent='failed: '+e.message;}
}
$('calib-capture').onclick=()=>calibAction('capture');
$('calib-clear').onclick=()=>calibAction('clear');

function renderCalib(s){
 const pts=s.calibration||[], seen=Boolean(s.target);
 $('calib-state').textContent=`${pts.length} point${pts.length===1?'':'s'}`;
 $('calib-state').className='pill '+(pts.length>=3?'ok':'');
 $('calib-capture').disabled=!s.control||!seen;
 $('calib-clear').disabled=!s.control||!pts.length;
 if(!s.control){$('calib-reply').textContent='control disabled';}
 else if(!seen){$('calib-reply').textContent='no object detected — show it to the camera';}
 if(pts.length>=3){
  const flat=pts.flatMap(p=>[p.u,p.v,...p.angles]);
  $('calib-yaml').textContent='calibration: ['+flat.join(', ')+']';
 }else{
  $('calib-yaml').textContent=pts.length?'need at least 3 points':'';
 }
}
