let selectedSid = null;
let autoTimer = null;
let statusBusy = false;
let lastSetpointAngles = {}; // sid -> setpoint angle (deg)
let unitMode = 'ticks'; // 'ticks' | 'deg'

async function api(method, url, body) {
  const res = await fetch(url, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  });
  return res.json();
}

function qs(id){ return document.getElementById(id); }

async function loadPorts(){
  const data = await api('GET','/api/ports');
  const sel = qs('portSelect');
  sel.innerHTML='';
  (data.ports||[]).forEach(p=>{
    const o = document.createElement('option');
    o.value = p; o.textContent = p; sel.appendChild(o);
  })
}

async function connect(){
  const port = qs('portSelect').value;
  const baudrate = parseInt(qs('baudInput').value||'1000000',10);
  const data = await api('POST','/api/connect',{port, baudrate});
  qs('connStatus').textContent = data.ok ? 'Connected' : ('Error: '+data.error);
}

async function disconnect(){
  // Stop auto update timer if running
  const au = qs('autoUpdate');
  if(au){ au.checked = false; }
  if(autoTimer){ clearInterval(autoTimer); autoTimer = null; }
  const data = await api('POST','/api/disconnect');
  qs('connStatus').textContent = data.ok ? 'Disconnected' : 'Error';
}

async function scan(){
  const start = parseInt(qs('scanStart').value||'1',10);
  const end = parseInt(qs('scanEnd').value||'30',10);
  const data = await api('GET',`/api/scan?start=${start}&end=${end}`);
  buildServoTabs(data.ids||[]);
  if ((data.ids||[]).length){ setActiveServo((data.ids||[])[0]); }
}

async function loadParams(){
  if(selectedSid==null) return;
  const data = await api('GET',`/api/params/${selectedSid}`);
  if(!data.ok){ alert(data.error||'Error'); return; }
  const e = data.params.eprom; const s = data.params.sram;
  qs('eprom_id').value = e.id;
  qs('eprom_baud').value = e.baud;
  qs('eprom_min').value = e.min_angle_limit;
  qs('eprom_max').value = e.max_angle_limit;
  qs('eprom_cw_dead').value = e.cw_dead;
  qs('eprom_ccw_dead').value = e.ccw_dead;
  qs('eprom_offset').value = e.offset;
  qs('eprom_mode').value = e.mode;

  qs('sram_torque').checked = !!s.torque_enable;
  qs('sram_acc').value = s.acc;
  qs('sram_goal_pos').value = s.goal_position;
  qs('sram_goal_time').value = s.goal_time;
  qs('sram_goal_speed').value = s.goal_speed;
  qs('sram_lock').checked = !!s.lock;
}

async function saveParams(){
  const payload = {
    eprom: {
      id: parseInt(qs('eprom_id').value,10),
      baud: parseInt(qs('eprom_baud').value,10),
      min_angle_limit: parseInt(qs('eprom_min').value,10),
      max_angle_limit: parseInt(qs('eprom_max').value,10),
      cw_dead: parseInt(qs('eprom_cw_dead').value,10),
      ccw_dead: parseInt(qs('eprom_ccw_dead').value,10),
      offset: parseInt(qs('eprom_offset').value,10),
      mode: parseInt(qs('eprom_mode').value,10),
    },
    sram: {
      torque_enable: qs('sram_torque').checked ? 1 : 0,
      acc: parseInt(qs('sram_acc').value,10),
      goal_position: parseInt(qs('sram_goal_pos').value,10),
      goal_time: parseInt(qs('sram_goal_time').value,10),
      goal_speed: parseInt(qs('sram_goal_speed').value,10),
      lock: qs('sram_lock').checked ? 1 : 0,
    }
  };
  const data = await api('POST',`/api/params/${selectedSid}`, payload);
  if(!data.ok){ alert(data.error||'Error'); }
}

async function refreshStatus(){
  if(selectedSid==null) return;
  if(statusBusy) return;
  statusBusy = true;
  const data = await api('GET',`/api/status/${selectedSid}`).catch(()=>({ok:false}));
  statusBusy = false;
  if(!data.ok){ alert(data.error||'Error'); return; }
  qs('statusPre').textContent = JSON.stringify(data.status, null, 2);
  const pos = data.status.present_position || 0;
  const angle = (pos/4095)*360.0;
  // Goal position as setpoint if provided and no user-set setpoint stored yet for this servo
  if(typeof data.status.goal_position === 'number' && lastSetpointAngles[selectedSid] === undefined){
    lastSetpointAngles[selectedSid] = (data.status.goal_position/4095)*360.0;
  }
  drawServo(angle, lastSetpointAngles[selectedSid]);
}

async function sendPos(){
  const {position, speed, acc} = readCmdInputsPosition();
  lastSetpointAngles[selectedSid] = (position/4095)*360.0;
  const data = await api('POST',`/api/command/${selectedSid}/position`,{position,speed,acc});
  if(!data.ok){ alert(data.error||'Error'); }
}

async function sendWheel(){
  const {speed, acc} = readCmdInputsWheel();
  const data = await api('POST',`/api/command/${selectedSid}/wheel`,{speed,acc});
  if(!data.ok){ alert(data.error||'Error'); }
}

async function torqueOn(){
  const data = await api('POST',`/api/command/${selectedSid}/torque`,{enable:true});
  if(!data.ok){ alert(data.error||'Error'); }
}
async function torqueOff(){
  const data = await api('POST',`/api/command/${selectedSid}/torque`,{enable:false});
  if(!data.ok){ alert(data.error||'Error'); }
}

async function lockOn(){
  const data = await api('POST',`/api/command/${selectedSid}/lock`,{lock:true});
  if(!data.ok){ alert(data.error||'Error'); }
}
async function lockOff(){
  const data = await api('POST',`/api/command/${selectedSid}/lock`,{lock:false});
  if(!data.ok){ alert(data.error||'Error'); }
}

async function changeId(){
  const new_id = parseInt(qs('newId').value,10);
  const data = await api('POST',`/api/command/${selectedSid}/change_id`,{new_id});
  if(!data.ok){ alert(data.error||'Error'); }
}

async function changeBaud(){
  const baud_key = qs('baudKey').value;
  const data = await api('POST',`/api/command/${selectedSid}/baud`,{baud_key});
  if(!data.ok){ alert(data.error||'Error'); }
}

function buildServoTabs(ids){
  const el = qs('servoTabs');
  el.innerHTML='';
  ids.forEach(id=>{
    const b = document.createElement('button');
    b.className = 'tab';
    b.textContent = `ID ${id}`;
    b.dataset.sid = id;
    b.addEventListener('click', ()=> setActiveServo(id));
    el.appendChild(b);
  });
}

function highlightActiveTab(){
  const el = qs('servoTabs');
  [...el.querySelectorAll('.tab')].forEach(b=>{
    b.classList.toggle('active', String(b.dataset.sid)===String(selectedSid));
  });
}

function setActiveServo(id){
  selectedSid = id;
  highlightActiveTab();
  loadParams();
  refreshStatus();
}

function drawServo(angleDeg, setpointDeg){
  const c = qs('servoCanvas'); if(!c) return; const ctx = c.getContext('2d');
  const w = c.width, h = c.height; ctx.clearRect(0,0,w,h);
  // background gradient
  const g = ctx.createLinearGradient(0,0,w,h); g.addColorStop(0,'#0b1222'); g.addColorStop(1,'#121a2c');
  ctx.fillStyle = g; ctx.fillRect(0,0,w,h);
  // servo body
  ctx.fillStyle = '#1f2937'; ctx.strokeStyle = '#334155'; ctx.lineWidth=2;
  const bw = 160, bh = 90; const bx = w*0.5 - bw/2, by = h*0.55 - bh/2;
  ctx.fillRect(bx,by,bw,bh); ctx.strokeRect(bx,by,bw,bh);
  // horn pivot
  const cx = w*0.5, cy = by; const r = 18; ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2); ctx.fillStyle='#94a3b8'; ctx.fill(); ctx.stroke();
  // horn arm
  const armLen = 90; 
  // actual (green)
  const radA = (angleDeg-90) * Math.PI/180;
  const exA = cx + Math.cos(radA)*armLen; const eyA = cy + Math.sin(radA)*armLen;
  ctx.strokeStyle = '#22c55e'; ctx.lineWidth=4; ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(exA,eyA); ctx.stroke();
  // setpoint (red)
  if(typeof setpointDeg === 'number'){
    const radS = (setpointDeg-90) * Math.PI/180;
    const exS = cx + Math.cos(radS)*armLen; const eyS = cy + Math.sin(radS)*armLen;
    ctx.strokeStyle = '#ef4444'; ctx.lineWidth=3; ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(exS,eyS); ctx.stroke();
  }
  // angle text
  ctx.fillStyle = '#e5e7eb'; ctx.font = '16px Inter, system-ui';
  ctx.fillText(`Angle: ${angleDeg.toFixed(1)}°`, 16, 24);
  if(typeof setpointDeg === 'number'){
    ctx.fillStyle = '#ef4444';
    ctx.fillText(`Setpoint: ${setpointDeg.toFixed(1)}°`, 16, 44);
  }
}

// Unit conversion helpers
function ticksToDegPos(t){ return (t/4095)*360.0; }
function degToTicksPos(d){ return Math.round((d/360.0)*4095); }
function ticksToDegSpeed(t){ return (t/4095)*360.0; }
function degToTicksSpeed(d){ return Math.round((d/360.0)*4095); }
// Map 0..255 acc units to 0..360 deg/s^2 linearly (similar to speed mapping)
function accToDegS2(a){ return (a/255)*360.0; }
function degS2ToAcc(a){ return Math.round((a/360.0)*255); }

function updateUnitHints(){
  const hp = qs('hintPos'); const hw = qs('hintWheel');
  if(unitMode==='deg'){
    if(hp) hp.textContent = 'Pos 0–360°, Speed 0–360°/s, Acc 0–360 deg/s²';
    if(hw) hw.textContent = 'Speed −360°/s … 360°/s, Acc 0–360 deg/s²';
    qs('cmd_pos').placeholder = '0–360°';
    qs('cmd_speed').placeholder = '0–360°/s';
    qs('cmd_wheel_speed').placeholder = '−360°/s … 360°/s';
    // Update constraints
    qs('cmd_pos').min = '0'; qs('cmd_pos').max = '360';
    qs('cmd_pos').step = '0.1';
    qs('cmd_speed').min = '0'; qs('cmd_speed').max = '360'; qs('cmd_speed').step = '0.1';
    qs('cmd_wheel_speed').min = '-360'; qs('cmd_wheel_speed').max = '360'; qs('cmd_wheel_speed').step = '0.1';
    qs('cmd_acc').min = '0'; qs('cmd_acc').max = '360'; qs('cmd_acc').step = '0.1';
    qs('cmd_wheel_acc').min = '0'; qs('cmd_wheel_acc').max = '360'; qs('cmd_wheel_acc').step = '0.1';
  } else {
    if(hp) hp.textContent = 'Pos 0–4095, Speed 0–4095, Acc 0–255';
    if(hw) hw.textContent = 'Speed −4095…4095 (neg = reverse), Acc 0–255';
    qs('cmd_pos').placeholder = '0–4095';
    qs('cmd_speed').placeholder = 'speed 0–4095';
    qs('cmd_wheel_speed').placeholder = '-4095…4095';
    // Update constraints
    qs('cmd_pos').min = '0'; qs('cmd_pos').max = '4095'; qs('cmd_pos').step = '1';
    qs('cmd_speed').min = '0'; qs('cmd_speed').max = '4095'; qs('cmd_speed').step = '1';
    qs('cmd_wheel_speed').min = '-4095'; qs('cmd_wheel_speed').max = '4095'; qs('cmd_wheel_speed').step = '1';
    qs('cmd_acc').min = '0'; qs('cmd_acc').max = '255'; qs('cmd_acc').step = '1';
    qs('cmd_wheel_acc').min = '0'; qs('cmd_wheel_acc').max = '255'; qs('cmd_wheel_acc').step = '1';
  }
}

function round1(x){ return Math.round(x*10)/10; }

function convertInputsToDeg(){
  // position
  let pos = parseFloat(qs('cmd_pos').value); if(isNaN(pos)) pos = 0;
  qs('cmd_pos').value = String(round1(ticksToDegPos(pos)));
  // speed
  let sp = parseFloat(qs('cmd_speed').value); if(isNaN(sp)) sp = 0;
  qs('cmd_speed').value = String(round1(ticksToDegSpeed(sp)));
  // wheel speed (can be negative)
  let wsp = parseFloat(qs('cmd_wheel_speed').value); if(isNaN(wsp)) wsp = 0;
  qs('cmd_wheel_speed').value = String(round1(ticksToDegSpeed(wsp)));
  // acc -> map 0..255 to 0..360 deg/s²
  let ac = parseFloat(qs('cmd_acc').value); if(isNaN(ac)) ac = 0;
  qs('cmd_acc').value = String(round1(accToDegS2(ac)));
  let wac = parseFloat(qs('cmd_wheel_acc').value); if(isNaN(wac)) wac = 0;
  qs('cmd_wheel_acc').value = String(round1(accToDegS2(wac)));
}

function convertInputsToTicks(){
  // position
  let pos = parseFloat(qs('cmd_pos').value); if(isNaN(pos)) pos = 0;
  qs('cmd_pos').value = String(degToTicksPos(pos));
  // speed
  let sp = parseFloat(qs('cmd_speed').value); if(isNaN(sp)) sp = 0;
  qs('cmd_speed').value = String(degToTicksSpeed(sp));
  // wheel speed
  let wsp = parseFloat(qs('cmd_wheel_speed').value); if(isNaN(wsp)) wsp = 0;
  qs('cmd_wheel_speed').value = String(degToTicksSpeed(wsp));
  // acc -> map 0..360 to 0..255
  let ac = parseFloat(qs('cmd_acc').value); if(isNaN(ac)) ac = 0;
  qs('cmd_acc').value = String(degS2ToAcc(ac));
  let wac = parseFloat(qs('cmd_wheel_acc').value); if(isNaN(wac)) wac = 0;
  qs('cmd_wheel_acc').value = String(degS2ToAcc(wac));
}

function readCmdInputsPosition(){
  let pos = parseFloat(qs('cmd_pos').value);
  let spd = parseFloat(qs('cmd_speed').value);
  let acc = parseFloat(qs('cmd_acc').value);
  if(unitMode==='deg'){
    pos = Math.max(0, Math.min(360, isNaN(pos)?0:pos));
    spd = Math.max(0, Math.min(360, isNaN(spd)?0:spd));
    acc = Math.max(0, Math.min(255, isNaN(acc)?0:acc));
    return {
      position: Math.max(0, Math.min(4095, degToTicksPos(pos))),
      speed: Math.max(0, Math.min(4095, degToTicksSpeed(spd))),
      acc: degS2ToAcc(acc)
    };
  } else {
    const position = Math.max(0, Math.min(4095, isNaN(pos)?0:pos));
    const speed = Math.max(0, Math.min(4095, isNaN(spd)?0:spd));
    const accC = Math.max(0, Math.min(255, isNaN(acc)?0:acc));
    return { position, speed, acc: accC };
  }
}

function readCmdInputsWheel(){
  let spd = parseFloat(qs('cmd_wheel_speed').value);
  let acc = parseFloat(qs('cmd_wheel_acc').value);
  if(unitMode==='deg'){
    spd = Math.max(-360, Math.min(360, isNaN(spd)?0:spd));
    acc = Math.max(0, Math.min(255, isNaN(acc)?0:acc));
    const speed = Math.max(-4095, Math.min(4095, degToTicksSpeed(spd)));
    return { speed, acc: degS2ToAcc(acc) };
  } else {
    const speed = Math.max(-4095, Math.min(4095, isNaN(spd)?0:spd));
    const accC = Math.max(0, Math.min(255, isNaN(acc)?0:acc));
    return { speed, acc: accC };
  }
}

function initSubtabs(){
  const buttons = document.querySelectorAll('.subtab');
  buttons.forEach(btn=>{
    btn.addEventListener('click', ()=>{
      buttons.forEach(b=>b.classList.remove('active'));
      btn.classList.add('active');
      const id = btn.dataset.tab;
      document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
      const panel = document.getElementById(`tab-${id}`);
      if(panel) panel.classList.add('active');
    });
  });
}

window.addEventListener('DOMContentLoaded', ()=>{
  qs('refreshPorts').addEventListener('click', loadPorts);
  qs('connectBtn').addEventListener('click', connect);
  qs('disconnectBtn').addEventListener('click', disconnect);
  qs('scanBtn').addEventListener('click', scan);
  qs('saveParams').addEventListener('click', saveParams);
  const sp2 = qs('saveParams2'); if(sp2) sp2.addEventListener('click', saveParams);
  qs('refreshStatus').addEventListener('click', refreshStatus);
  qs('sendPos').addEventListener('click', sendPos);
  qs('sendWheel').addEventListener('click', sendWheel);
  qs('torqueOn').addEventListener('click', torqueOn);
  qs('torqueOff').addEventListener('click', torqueOff);
  qs('lockOn').addEventListener('click', lockOn);
  qs('lockOff').addEventListener('click', lockOff);
  qs('changeId').addEventListener('click', changeId);
  qs('changeBaud').addEventListener('click', changeBaud);
  loadPorts();
  initSubtabs();

  const au = qs('autoUpdate');
  if(au){
    au.addEventListener('change', ()=>{
      if(au.checked){
        autoTimer = setInterval(refreshStatus, 200);
      } else {
        if(autoTimer) clearInterval(autoTimer);
        autoTimer = null;
      }
    });
    // Start fast polling by default if checkbox is checked
    if(au.checked){ autoTimer = setInterval(refreshStatus, 200); }
  }
  const unitSel = qs('unitMode');
  if(unitSel){
    unitSel.addEventListener('change', ()=>{
      unitMode = unitSel.value;
      // Convert current numbers to new unit to avoid unsafe values
      if(unitMode==='deg') convertInputsToDeg(); else convertInputsToTicks();
      updateUnitHints();
    });
  }
  updateUnitHints();
});


