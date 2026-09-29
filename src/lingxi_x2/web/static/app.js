const $ = (s) => document.querySelector(s);
const escapeText = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const log = (message, data) => { $('#log').textContent = (`[${new Date().toLocaleTimeString()}] ${message}${data ? '\n'+JSON.stringify(data,null,2) : ''}\n` + $('#log').textContent).slice(0,30000); };
async function api(path, options){ const response=await fetch(path,{signal:AbortSignal.timeout(10000),...options}); if(!response.ok) throw new Error(await response.text()); return response.json(); }
const post = (path, data={}) => api(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
document.querySelectorAll('.tab').forEach(button => button.onclick = () => { document.querySelectorAll('.tab,.view').forEach(x=>x.classList.remove('active')); button.classList.add('active'); $('#'+button.dataset.view).classList.add('active'); });
const armNames=['left_shoulder_pitch','left_shoulder_roll','left_shoulder_yaw','left_elbow','left_wrist_yaw','left_wrist_pitch','left_wrist_roll','right_shoulder_pitch','right_shoulder_roll','right_shoulder_yaw','right_elbow','right_wrist_yaw','right_wrist_pitch','right_wrist_roll'];
const handNames=['thumb_roll','thumb_abad','thumb_mcp','index_abad','index_pip','middle_pip','ring_abad','ring_pip','pinky_abad','pinky_pip'];
let liveState=null, motionAllowed=false, initialized=false, busy=false;
function jointMarkup(joints){ return joints.map(j=>`<div class="joint"><label>${escapeText(j.name)}</label><output>${j.position_rad.toFixed(4)}</output></div>`).join(''); }
function sliders(root,names,kind){ root.innerHTML=names.map((name,i)=>`<div class="slider"><label>${name}</label><input type="number" step="0.005" data-${kind}="${i}" disabled><output>rad</output></div>`).join(''); }
sliders($('#arm-controls'),armNames,'arm'); sliders($('#hand-controls'),handNames,'hand');
function initializeTargets(kind,joints){ document.querySelectorAll(`[data-${kind}]`).forEach((x,i)=>{x.value=joints[i].position_rad.toFixed(5);x.disabled=!motionAllowed;}); }
function controlButtons(){const enabled=motionAllowed&&initialized&&!busy&&$('#armed').checked; $('#send-arm').disabled=!enabled;$('#send-hand').disabled=!enabled;}
$('#armed').onchange=controlButtons;
$('#hand-side').onchange=()=>{if(liveState)initializeTargets('hand',liveState.hands[$('#hand-side').value].joints);};
function drawTactile(root,frames,referenceNs,isReplay=false){
  root.replaceChildren();
  for(const [side,frame] of Object.entries(frames)){
    const section=document.createElement('section'); const label=document.createElement('p');
    const age=(referenceNs-frame.timestamp.monotonic_ns)/1e9;
    label.textContent=`${side} · 源时间 ${frame.timestamp.sec}.${String(frame.timestamp.nanosec).padStart(9,'0')} · ${isReplay?'采集时':''}接收龄 ${age.toFixed(3)} s${age>0.5||age<0?' · 过期/时间异常':''}`;
    section.append(label);
    for(const surface of [frame.palm,frame.back_of_hand,...Object.values(frame.fingertips)]){
      const wrap=document.createElement('div');wrap.className='tactile-surface';
      const title=document.createElement('div');title.textContent=`${surface.name} · peak ${Math.max(...surface.values)}`;wrap.append(title);
      const canvas=document.createElement('canvas');const [rows,cols]=surface.shape;canvas.width=cols*24;canvas.height=rows*24;
      const ctx=canvas.getContext('2d');
      surface.values.forEach((v,i)=>{const x=i%cols*24,y=Math.floor(i/cols)*24;ctx.fillStyle=`rgb(${Math.round(v*0.9)},${30+Math.round(v*0.6)},${40+Math.round(v*0.2)})`;ctx.fillRect(x,y,23,23);ctx.fillStyle=v>150?'#101417':'#fff';ctx.font='10px monospace';ctx.fillText(String(v),x+2,y+15);});
      canvas.setAttribute('aria-label',`${side} ${surface.name}: ${surface.values.join(', ')}`);wrap.append(canvas);section.append(wrap);
    }
    root.append(section);
  }
}
async function refreshStatus(){
  try{
    const [value,ready]=await Promise.all([api('/api/status'),api('/api/readiness')]);motionAllowed=ready.web_motion_allowed;
    $('#readiness').textContent=`${ready.reason}。压力接触尚未验收。当前数据源：${value.backend}`;
    $('#connection').textContent=`${value.backend} / ${value.connected?'connected':'offline'}`;
    $('#capabilities').innerHTML=Object.entries(value.capabilities).map(([name,c])=>`<div class="cap ${c.available?'ok':'no'}"><strong>${escapeText(name)}</strong><span>${c.verified?'接口/样本已检查':c.available?'接口可用':'不可用'} · 非运动验收</span></div>`).join('');
    $('#summary').innerHTML=`<dt>Model</dt><dd>${escapeText(value.model||'--')}</dd><dt>Firmware</dt><dd>${escapeText(value.firmware||'--')}</dd><dt>Backend</dt><dd>${escapeText(value.backend)}</dd><dt>Warnings</dt><dd>${value.warnings.length}</dd>`;
  }catch(e){motionAllowed=false;$('#connection').textContent='offline';log('status error',String(e));}
  controlButtons();
}
async function refreshState(){
  try{
    const value=await api('/api/state');liveState=value;
    $('#arm-joints').innerHTML=jointMarkup(value.arm.joints);
    for(const side of ['left','right'])$('#'+side+'-hand').innerHTML=jointMarkup(value.hands[side].joints);
    drawTactile($('#live-tactile'),value.tactile,value.received_monotonic_ns);
    $('#tactile-status').textContent=value.tactile_error||(!Object.keys(value.tactile).length?'压力数据不可用':'已收到压力数据；单位为原始字节值');
    if(!initialized&&motionAllowed){initializeTargets('arm',value.arm.joints);initializeTargets('hand',value.hands[$('#hand-side').value].joints);initialized=true;}
  }catch(e){initialized=false;$('#live-tactile').replaceChildren();$('#tactile-status').textContent='实时读取失败，压力不可用';log('state error',String(e));}
  controlButtons();
}
function refreshCamera(){const img=$('#camera');img.onload=()=>{$('#frame-meta').textContent=`${img.naturalWidth} x ${img.naturalHeight}`;setTimeout(refreshCamera,500);};img.onerror=()=>{$('#frame-meta').textContent='图像不可用';setTimeout(refreshCamera,2000);};img.src=`/api/cameras/rgbd_front_rgb/frame?t=${Date.now()}`;}
async function send(path,payload){if(!motionAllowed||!initialized||!$('#armed').checked||busy)return;busy=true;controlButtons();try{log('命令发布结果（运动未验收）',await post(path,{...payload,confirmation:'MOVE X2'}));}catch(e){log('command error',String(e));}finally{busy=false;$('#armed').checked=false;controlButtons();}}
$('#send-arm').onclick=()=>send('/api/control/arm',{positions_rad:[...document.querySelectorAll('[data-arm]')].map(x=>Number(x.value)),duration_s:Number($('#arm-duration').value)});
$('#send-hand').onclick=()=>send('/api/control/hand',{side:$('#hand-side').value,positions_rad:[...document.querySelectorAll('[data-hand]')].map(x=>Number(x.value)),duration_s:Number($('#hand-duration').value)});
let activeRecording=null,replayId=null,replayIndex=0,replayCount=0;
async function refreshRecordings(){
  try{
    const jobs=await api('/api/recordings');activeRecording=jobs.find(j=>['recording','stopping'].includes(j.status))?.id||null;
    $('#record-start').disabled=!!activeRecording;$('#record-stop').disabled=!activeRecording;
    $('#record-list').replaceChildren();
    for(const job of jobs){
      const row=document.createElement('div');row.className='record-row';const label=document.createElement('span');
      label.textContent=`${new Date(job.created_unix_ns/1e6).toLocaleString()} · ${job.status} · ${job.samples??'?'} 帧${job.error?' · '+job.error:''}`;row.append(label);
      if(!['recording','stopping'].includes(job.status)){
        const link=document.createElement('a');link.href=`/api/recordings/${job.id}/download`;link.textContent='下载 JSONL';row.append(link);
        const button=document.createElement('button');button.textContent='查看回放';button.disabled=!job.inspection?.samples;
        button.onclick=()=>{replayId=job.id;replayIndex=0;replayCount=job.inspection.samples;showReplay();};row.append(button);
      }
      $('#record-list').append(row);
    }
  }catch(e){$('#record-status').textContent=String(e);}
}
$('#record-start').onclick=async()=>{try{$('#record-start').disabled=true;const camera=$('#record-camera').checked;const job=await post('/api/recordings',{duration_s:Number($('#record-duration').value),rate_hz:Number($('#record-rate').value),cameras:camera?['rgbd_front_rgb']:[],include_tactile:true,include_image_data:camera});$('#record-status').textContent=`开始只读采集 ${job.id}`;}catch(e){$('#record-status').textContent=String(e);}await refreshRecordings();};
$('#record-stop').onclick=async()=>{if(activeRecording){try{await post(`/api/recordings/${activeRecording}/stop`);$('#record-status').textContent='已请求停止，等待当前读取结束';}catch(e){$('#record-status').textContent=String(e);}}};
async function showReplay(){
  $('#replay-prev').disabled=true;$('#replay-next').disabled=true;
  try{const value=await api(`/api/recordings/${replayId}/samples?offset=${replayIndex}&limit=1`);const sample=value.samples[0];if(!sample)throw new Error('缺少回放帧');const o=sample.observation;
    $('#replay-source').textContent=`历史记录 ${replayId} · 原数据源 ${value.metadata.status.backend} · 帧 ${replayIndex+1}/${replayCount} · 原采集时间 ${o.captured_monotonic_ns} ns · 图像元数据 ${Object.keys(sample.camera_metadata).join(', ')||'无'}（此视图不显示图像）`;
    $('#replay-joints').innerHTML=jointMarkup(o.arm?.joints||[]);drawTactile($('#replay-tactile'),o.tactile,o.captured_monotonic_ns,true);
    $('#replay-prev').disabled=replayIndex===0;$('#replay-next').disabled=replayIndex+1>=replayCount;
  }catch(e){$('#replay-source').textContent=String(e);$('#replay-joints').replaceChildren();$('#replay-tactile').replaceChildren();}
}
$('#replay-prev').onclick=()=>{replayIndex--;showReplay();};$('#replay-next').onclick=()=>{replayIndex++;showReplay();};
async function poll(fn,delay){await fn();setTimeout(()=>poll(fn,delay),delay);}
poll(refreshStatus,5000);poll(refreshState,1000);poll(refreshRecordings,2000);refreshCamera();
