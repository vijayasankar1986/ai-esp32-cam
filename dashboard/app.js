const $ = id => document.getElementById(id);
let view = 'live', current = null, busy = false;
const words = {idle:'Waiting for detection',target:'Trigger preset selected',returning:'Returning to home preset'};
function select(next){view=next;for(const key of ['live','test']){const active=key===view;$(key+'-tab').classList.toggle('selected',active);$(key+'-tab').setAttribute('aria-pressed',String(active));}renderFrame();}
$('live-tab').onclick=()=>select('live');$('test-tab').onclick=()=>select('test');
function renderFrame(){
 if(!current)return;
 const demo=view==='test', t=current.test.telemetry;
 const available=demo ? Boolean(t && t.frames>0) : current.frame_age!==null;
 $('frame').hidden=!available;$('empty').hidden=available;
 if(available){$('frame').src=(demo?'/api/test/frame':'/api/frame')+'?t='+Date.now();$('frame').alt=demo?'Generated test image, not a real camera feed':'Latest ROS camera image';}
 if(!available){$('empty').querySelector('h3').textContent=demo?'Try the software test':'Waiting for the camera';$('empty').querySelector('p').textContent=demo?'Run the system test to watch generated images pass through ROS 2.':'The camera’s Wi-Fi connection still needs setup. USB is currently providing a serial connection.';}
 $('image-label').textContent=demo?'GENERATED TEST IMAGE · NO PHYSICAL CAMERA':current.camera_live?'LIVE CAMERA':available?'LAST FRAME · FEED STALE':'LIVE SOURCE · NO FRAMES';
 $('frame-detail').textContent=demo?(t?`${t.frames} test frames · ${words[t.phase]||t.phase}`:'Isolated test · ROS domain 87'):`${current.image_count} frames received · /camera/image_raw`;
 const detected=demo?t?.red:current.red;
 $('detection').textContent=detected===null||detected===undefined?'Detection —':detected?'Colour detected':'No colour detected';
}
function render(s){current=s;$('connection').textContent='Pi connected';$('connection').className='pill ok';$('offline').hidden=true;
 $('ros').textContent=s.ros?'Observer online':'Unavailable';$('ros').style.color=s.ros?'var(--mint)':'var(--amber)';$('ros-detail').textContent=s.ros?`${s.ros_distro} · domain ${s.ros_domain} · ${s.publishers} image publishers`:s.ros_error;
 $('camera').textContent=s.camera_live?'Receiving frames':s.frame_age!==null?'Feed stale':'Not connected';$('camera').style.color=s.camera_live?'var(--mint)':'var(--amber)';$('camera-detail').textContent=s.camera_live?'Live images arriving through ROS 2':'Camera network setup pending';
 $('usb').textContent=`${s.usb.length} connected`;$('host').textContent=s.host;$('model').textContent=s.model;$('temp').textContent=s.temperature===null?'—':s.temperature+' °C';$('disk').textContent=s.disk_free_gb+' GB';
 const h=Math.floor(s.uptime_seconds/3600);$('uptime').textContent=h>=24?Math.floor(h/24)+'d '+h%24+'h':h+'h '+Math.floor(s.uptime_seconds%3600/60)+'m';
 $('devices').replaceChildren(...(s.usb.length?s.usb:['No serial adapters detected']).map(name=>{const li=document.createElement('li');li.textContent=name;return li;}));
 const t=s.test, running=t.status==='running';$('run-test').disabled=running||busy;$('run-test').firstChild.textContent=running?'Test running… ':'Run system test ';
 $('test-state').textContent={not_run:'Not run this session',running:'Running · no servo commands',passed:'Passed',failed:'Needs attention'}[t.status];$('test-state').className='pill '+(t.status==='passed'?'ok':t.status==='failed'?'warn':'');
 $('test-detail').textContent=running?(t.telemetry?`${t.telemetry.frames} images processed. ${words[t.telemetry.phase]||t.telemetry.phase}.`:'Starting logic and ROS checks…'):t.status==='passed'?'Logic and ROS checks passed. Physical camera and arm remain unverified.':t.status==='failed'?'Check the output below for the failure.':'Run a test to see current results.';
 $('output').textContent=t.output||(running?'Test in progress…':'No test has been run since the dashboard started.');$('updated').textContent='Updated '+new Date().toLocaleTimeString();renderFrame();renderArm(s);
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
