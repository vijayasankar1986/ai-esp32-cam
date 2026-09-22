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
 $('detection').textContent=detected===null||detected===undefined?'Detection —':detected?'Red detected':'No red detected';
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
 $('output').textContent=t.output||(running?'Test in progress…':'No test has been run since the dashboard started.');$('updated').textContent='Updated '+new Date().toLocaleTimeString();renderFrame();
}
async function refresh(){try{const r=await fetch('/api/status',{signal:AbortSignal.timeout(5000)});if(!r.ok)throw Error(r.status);render(await r.json());}catch(e){$('connection').textContent='Pi unreachable';$('connection').className='pill warn';$('offline').hidden=false;$('run-test').disabled=true;}finally{setTimeout(refresh,1000);}}
$('run-test').onclick=async()=>{busy=true;$('run-test').disabled=true;select('test');try{const r=await fetch('/api/test',{method:'POST',headers:{'X-Arm-Dashboard':'1'},signal:AbortSignal.timeout(5000)});if(!r.ok)throw Error((await r.json()).error||r.status);$('test-state').textContent='Starting…';}catch(e){$('test-detail').textContent='Could not start test: '+e.message;}finally{busy=false;}};
refresh();
