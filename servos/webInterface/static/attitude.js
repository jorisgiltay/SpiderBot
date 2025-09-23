// Global display/log scaling to align commanded trace with IMU (does not affect applied command)
const CMD_SCALE = 0.8;
// (Level mode constants removed)
async function api(method, url, body){
  const res = await fetch(url, { method, headers:{'Content-Type':'application/json'}, body: body? JSON.stringify(body): undefined });
  return res.json();
}

async function loadPorts(){
  const data = await api('GET','/api/ports');
  const sel = document.getElementById('portSelect'); if(!sel) return;
  sel.innerHTML='';
  (data.ports||[]).forEach(p=>{
    const o = document.createElement('option'); o.value=p; o.textContent=p; sel.appendChild(o);
  });
}

async function connect(){
  const port = document.getElementById('portSelect').value;
  const baudrate = parseInt(document.getElementById('baudInput').value||'1000000',10);
  const data = await api('POST','/api/connect',{port, baudrate});
  const s = document.getElementById('connStatus'); if(s) s.textContent = data.ok ? 'Connected' : ('Error: '+(data.error||''));
}

async function disconnect(){
  const data = await api('POST','/api/disconnect');
  const s = document.getElementById('connStatus'); if(s) s.textContent = data.ok ? 'Disconnected' : 'Error';
}

/* Compute corner heights from roll/pitch and rectangle spans. */

function rad(deg) { return (deg * Math.PI) / 180; }

function updateHeights() {
  const spanRoll = parseFloat(document.getElementById('spanRoll').value) || 93; // along X (left-right)
  const spanPitch = parseFloat(document.getElementById('spanPitch').value) || 80; // along Y (front-back)
  const rollRaw = parseFloat(document.getElementById('rollDeg').value) || 0;
  const pitchRaw = parseFloat(document.getElementById('pitchDeg').value) || 0;
  // Invert slider semantics
  let rollDeg = -rollRaw;
  let pitchDeg = -pitchRaw;
  // (Level mode removed)
  const refHeight = parseFloat(document.getElementById('refHeight').value) || 34;
  const scale = parseFloat(document.getElementById('scaleFactor').value) || 2.0;

  // Display scaled commanded value (for visual alignment with plots)
  document.getElementById('rollLabel').textContent = (rollRaw*CMD_SCALE).toFixed(2) + '°';
  document.getElementById('pitchLabel').textContent = (pitchRaw*CMD_SCALE).toFixed(2) + '°';

  // Half spans
  const hx = spanRoll / 2.0;
  const hy = spanPitch / 2.0;

  // Height contribution from small-angle rotations. For exact, use tan(theta)*distance.
  // Using exact trig for generality: z = tan(roll)*x + tan(pitch)*y
  const tanR = Math.tan(rad(rollDeg));
  const tanP = Math.tan(rad(pitchDeg));

  // Front-Left (x=-hx, y=+hy), Front-Right (x=+hx, y=+hy)
  const zFL = refHeight + scale * ( -tanR * hx + tanP * hy );
  const zFR = refHeight + scale * ( +tanR * hx + tanP * hy );
  const zRL = refHeight + scale * ( -tanR * hx - tanP * hy );
  const zRR = refHeight + scale * ( +tanR * hx - tanP * hy );

  document.getElementById('hFL').textContent = zFL.toFixed(2);
  document.getElementById('hFR').textContent = zFR.toFixed(2);
  document.getElementById('hRL').textContent = zRL.toFixed(2);
  document.getElementById('hRR').textContent = zRR.toFixed(2);

  // Also compute servo angles via API (pure calculation)
  fetch('/api/attitude/angles', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      roll: rollDeg, pitch: pitchDeg,
      span_roll: spanRoll, span_pitch: spanPitch,
      ref_height: refHeight, scale: scale,
      r1: 77.816, off1: 31.0, r2: 85.788, off2: 22.5, switch: -30.0
    })
  }).then(r => r.json()).then(d => {
    if (!d.ok) return;
    const t = d.thetas || {};
    ['FL','FR','RL','RR'].forEach(k => {
      const el = document.getElementById('theta_'+k);
      if (el) {
        const val = (t[k] ?? 0);
        el.textContent = (val).toFixed(2) + '°';
      }
    });
  }).catch(() => {});

  // Preview ticks via apply endpoint (works offline / when disconnected)
  try {
    const mapping = {
      FL: parseInt(document.getElementById('sidFL').value || '0', 10),
      FR: parseInt(document.getElementById('sidFR').value || '0', 10),
      RL: parseInt(document.getElementById('sidRL').value || '0', 10),
      RR: parseInt(document.getElementById('sidRR').value || '0', 10),
    };
    const speed = parseInt(document.getElementById('applySpeed').value || '2400', 10);
    const acc = parseInt(document.getElementById('applyAcc').value || '50', 10);
    const bias_ticks = parseFloat(document.getElementById('biasTicks').value || '0');

    const req = () => fetch('/api/attitude/apply', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        roll: rollDeg, pitch: pitchDeg,
        span_roll: spanRoll, span_pitch: spanPitch,
        ref_height: refHeight, scale: scale,
        r1: 77.816, off1: 31.0, r2: 85.788, off2: 22.5, switch: -30.0,
        mapping, speed, acc, bias_ticks
      })
    }).then(r => r.json());
    req().then(d => {
      const res = d.results || {};
      const map = { FL:'ticks_FL', FR:'ticks_FR', RL:'ticks_RL', RR:'ticks_RR' };
      Object.keys(map).forEach(k=>{
        const el = document.getElementById(map[k]);
        if(!el) return;
        const item = res[k];
        el.textContent = item && typeof item.ticks === 'number' ? String(item.ticks) : '-';
      });
    }).catch(()=>{});
  } catch(e) { /* ignore */ }
}

function attach() {
  // Connection events
  const rp = document.getElementById('refreshPorts'); if(rp) rp.addEventListener('click', loadPorts);
  const cb = document.getElementById('connectBtn'); if(cb) cb.addEventListener('click', connect);
  const db = document.getElementById('disconnectBtn'); if(db) db.addEventListener('click', disconnect);
  loadPorts();

  ['spanRoll','spanPitch','rollDeg','pitchDeg','refHeight','scaleFactor','sidFL','sidFR','sidRL','sidRR','applySpeed','applyAcc','biasTicks'].forEach(id => {
    document.getElementById(id).addEventListener('input', updateHeights);
  });
  updateHeights();
}

window.addEventListener('DOMContentLoaded', attach);

// Apply button handler
function applyAngles() {
  const spanRoll = parseFloat(document.getElementById('spanRoll').value) || 93;
  const spanPitch = parseFloat(document.getElementById('spanPitch').value) || 80;
  const rollRaw = parseFloat(document.getElementById('rollDeg').value) || 0;
  const pitchRaw = parseFloat(document.getElementById('pitchDeg').value) || 0;
  let rollDeg = -rollRaw;
  let pitchDeg = -pitchRaw;
  const refHeight = parseFloat(document.getElementById('refHeight').value) || 34;
  const scale = parseFloat(document.getElementById('scaleFactor').value) || 2.0;

  const mapping = {
    FL: parseInt(document.getElementById('sidFL').value || '0', 10),
    FR: parseInt(document.getElementById('sidFR').value || '0', 10),
    RL: parseInt(document.getElementById('sidRL').value || '0', 10),
    RR: parseInt(document.getElementById('sidRR').value || '0', 10),
  };

  const speed = parseInt(document.getElementById('applySpeed').value || '2400', 10);
  const acc = parseInt(document.getElementById('applyAcc').value || '50', 10);
  const bias_ticks = parseFloat(document.getElementById('biasTicks').value || '0');
  // (duplicate declarations removed)

  fetch('/api/attitude/apply', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      roll: rollDeg, pitch: pitchDeg,
      span_roll: spanRoll, span_pitch: spanPitch,
      ref_height: refHeight, scale: scale,
      r1: 77.816, off1: 31.0, r2: 85.788, off2: 22.5, switch: -30.0,
      mapping, speed, acc, bias_ticks
    })
  }).then(r => r.json()).then(d => {
    const out = document.getElementById('applyResult');
    if (out) out.textContent = JSON.stringify(d, null, 2);
    try{
      const res = d.results || {};
      const map = { FL:'ticks_FL', FR:'ticks_FR', RL:'ticks_RL', RR:'ticks_RR' };
      Object.keys(map).forEach(k=>{
        const el = document.getElementById(map[k]);
        if(!el) return;
        const item = res[k];
        el.textContent = item && typeof item.ticks === 'number' ? String(item.ticks) : '-';
      });
    }catch(e){ /* ignore */ }
  }).catch(err => {
    const out = document.getElementById('applyResult');
    if (out) out.textContent = String(err);
  });
}

window.applyAngles = applyAngles;


// --------- MSP + Live IMU + Logging ---------
async function mspConnect(){
  const port = document.getElementById('mspPort').value;
  const baud = parseInt(document.getElementById('mspBaud').value||'115200',10);
  const data = await api('POST','/api/msp/connect',{port, baud});
  document.getElementById('mspStatus').textContent = data.ok? `Connected (${port})` : (data.error||'Error');
}

async function mspDisconnect(){
  const data = await api('POST','/api/msp/disconnect');
  document.getElementById('mspStatus').textContent = data.ok? 'Disconnected' : 'Error';
}

window.mspConnect = mspConnect;
window.mspDisconnect = mspDisconnect;

// Live IMU polling & logging
let imuTimer = null;
let logRows = [];

function getImuSettings(){ return {}; }

async function pollIMU(){
  const out = document.getElementById('imuReadout');
  const resp = await api('GET','/api/msp/attitude').catch(()=>({ok:false}));
  if(!resp.ok){ if(out) out.textContent = 'IMU: not connected'; return; }
  let r = resp.roll, p = resp.pitch, y = resp.yaw;
  window._imuLastR = r; window._imuLastP = p; window._imuLastY = y;
  if(out) out.textContent = `IMU roll: ${r.toFixed(1)}°, pitch: ${p.toFixed(1)}°, yaw: ${y.toFixed(1)}°`;
  // If recording, append row with latest commanded (from sliders) and IMU
  if(recording){
    const cmdRollRaw = parseFloat(document.getElementById('rollDeg').value)||0;
    const cmdPitchRaw = parseFloat(document.getElementById('pitchDeg').value)||0;
    const cmdRoll = cmdRollRaw * CMD_SCALE;   // slider-based (display aligned)
    const cmdPitch = cmdPitchRaw * CMD_SCALE;
    logRows.push({t: Date.now(), cmd_roll: cmdRoll, cmd_pitch: cmdPitch, imu_roll: r, imu_pitch: p, imu_yaw: y});
    // live update plot
    try { plotRecorded(); } catch(e) {}
  }
  // (Auto-apply removed)
}

function startImu(){ if(imuTimer) return; imuTimer = setInterval(pollIMU, 100); }
function stopImu(){ if(imuTimer){ clearInterval(imuTimer); imuTimer=null; } }

let recording = false;
function startRecording(){ recording = true; logRows = []; }
function stopRecording(){ recording = false; }
function downloadCSV(){
  const header = ['t','cmd_roll','cmd_pitch','imu_roll','imu_pitch','imu_yaw'];
  const lines = [header.join(',')].concat(logRows.map(r=>header.map(k=>r[k]??'').join(',')));
  const blob = new Blob([lines.join('\n')], {type:'text/csv'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a'); a.href=url; a.download='attitude_log.csv'; a.click(); URL.revokeObjectURL(url);
}

// ----- Plot helpers stay the same -----
function plotRecorded(){
  const rows = logRows;
  const c = document.getElementById('recordCanvas'); if(!c) return;
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  const pad = 50; const gap = 40; const W = c.width - pad*2; const H = c.height - pad*2 - gap; const ph = Math.floor(H/2);
  const topX = pad, topY = pad, botX = pad, botY = pad + ph + gap;
  ctx.fillStyle = '#0b1222'; ctx.fillRect(0,0,c.width,c.height);
  ctx.font = '13px Inter, system-ui'; ctx.fillStyle = '#e5e7eb';

  const n = rows.length;
  const xs = rows.map((_,i)=>i+1);
  function minmax(vals){ let lo=Infinity, hi=-Infinity; vals.forEach(v=>{ if(typeof v==='number' && isFinite(v)){ lo=Math.min(lo,v); hi=Math.max(hi,v);} }); if(!isFinite(lo)||!isFinite(hi)){lo=-1;hi=1;} if(lo===hi){lo-=1;hi+=1;} const span=hi-lo; return [lo-span*0.1, hi+span*0.1]; }
  function xticks(count){ const step=Math.max(1, Math.ceil(n/Math.max(2,count))); const ticks=[]; for(let i=1;i<=n;i+=step){ ticks.push(i);} if(ticks[ticks.length-1]!==n) ticks.push(n); return ticks; }
  function yticks(lo,hi,maxTicks){ const span=hi-lo; if(span<=0) return [lo,hi]; const raw=span/Math.max(2,maxTicks); const mag=Math.pow(10, Math.floor(Math.log10(raw))); const norm=raw/mag; let step=mag; if(norm>5) step=10*mag; else if(norm>2) step=5*mag; else if(norm>1) step=2*mag; else step=mag; const t0=Math.floor(lo/step)*step; const t1=Math.ceil(hi/step)*step; const ticks=[]; for(let t=t0;t<=t1+1e-9;t+=step){ ticks.push(+t.toFixed(6)); } return ticks; }

  function plotPanel(x,y,w,h, series, title){
    // series: [{key:'imu_roll', color:'#ef4444', name:'Roll'}, {key:'imu_pitch', color:'#22c55e', name:'Pitch'}]
    const allY = [].concat(...series.map(s=>rows.map(r=>r[s.key])));
    const [lo,hi] = minmax(allY);
    // frame
    ctx.strokeStyle = '#334155'; ctx.lineWidth=1; ctx.strokeRect(x,y,w,h);
    // grid + ticks
    const yt = yticks(lo,hi,6); ctx.textAlign='right'; ctx.textBaseline='middle';
    yt.forEach(t=>{ const yy = y + h - (t-lo)/(hi-lo)*h; ctx.strokeStyle='#1f2937'; ctx.beginPath(); ctx.moveTo(x,yy); ctx.lineTo(x+w,yy); ctx.stroke(); ctx.fillStyle='#94a3b8'; ctx.fillText(String(t.toFixed(1)), x-6, yy); });
    const xt = xticks(8); ctx.textAlign='center'; ctx.textBaseline='top';
    xt.forEach(t=>{ const xx = x + (t-1)/(n-1) * w; ctx.strokeStyle='#1f2937'; ctx.beginPath(); ctx.moveTo(xx,y); ctx.lineTo(xx,y+h); ctx.stroke(); ctx.fillStyle='#94a3b8'; ctx.fillText(String(t), xx, y+h+6); });
    // lines
    series.forEach(s=>{
      ctx.strokeStyle = s.color; ctx.lineWidth=2; ctx.beginPath(); let started=false;
      rows.forEach((r,i)=>{
        const val = r[s.key]; if(typeof val!=='number') return; const xx = x + (i)/(n-1) * w; const yy = y + h - (val-lo)/(hi-lo) * h; if(!started){ ctx.moveTo(xx,yy); started=true; } else { ctx.lineTo(xx,yy); }
      });
      ctx.stroke();
    });
    // labels
    ctx.fillStyle='#e5e7eb'; ctx.textAlign='center'; ctx.textBaseline='alphabetic'; ctx.fillText('Sample', x+w/2, y+h+26);
    ctx.save(); ctx.translate(x-40, y+h/2); ctx.rotate(-Math.PI/2); ctx.fillText('Angle (deg)', 0, 0); ctx.restore();
    ctx.textAlign='left'; ctx.textBaseline='top'; ctx.fillText(title, x, y-24);
    // legend
    let lx = x + w - 150, ly = y - 24; series.forEach(s=>{ ctx.fillStyle=s.color; ctx.fillRect(lx, ly+6, 10, 10); ctx.fillStyle='#e5e7eb'; ctx.fillText(s.name, lx+16, ly+16); lx += 80; });
  }

  // Top: Roll (cmd, measured)
  plotPanel(topX, topY, W, ph, [
    {key:'cmd_roll', color:'#60a5fa', name:'Cmd Roll'},
    {key:'imu_roll', color:'#ef4444', name:'IMU Roll'}
  ], 'Roll vs Sample');

  // Bottom: Pitch (cmd, measured)
  plotPanel(botX, botY, W, ph, [
    {key:'cmd_pitch', color:'#f59e0b', name:'Cmd Pitch'},
    {key:'imu_pitch', color:'#22c55e', name:'IMU Pitch'}
  ], 'Pitch vs Sample');
}

// auto-plot on stop
const _origStopRecording = stopRecording;
stopRecording = function(){ _origStopRecording(); plotRecorded(); };

window.startImu = startImu;
window.stopImu = stopImu;
window.startRecording = startRecording;
window.stopRecording = stopRecording;
window.downloadCSV = downloadCSV;
window.plotRecorded = plotRecorded;

// --------- Auto Sweep helpers ---------
function _numAttr(el, name, fallback){
  const v = parseFloat(el.getAttribute(name)||'');
  return isNaN(v)? fallback : v;
}

function getSweepStep(){
  const v = parseFloat(document.getElementById('sweepStep')?.value||'1');
  return isNaN(v)? 1 : v;
}

function getSweepSettle(){
  const v = parseFloat(document.getElementById('sweepSettle')?.value||'0.15');
  return isNaN(v)? 0.15 : v;
}

function delay(ms){ return new Promise(res=>setTimeout(res, ms)); }

async function readImuOnce(){
  const resp = await api('GET','/api/msp/attitude').catch(()=>({ok:false}));
  if(!resp.ok) return null;
  return { roll: resp.roll, pitch: resp.pitch, yaw: resp.yaw };
}

function readCmdRaw(){
  return {
    rollRaw: parseFloat(document.getElementById('rollDeg').value)||0,
    pitchRaw: parseFloat(document.getElementById('pitchDeg').value)||0,
  };
}

async function applyCurrent(){
  const spanRoll = parseFloat(document.getElementById('spanRoll').value) || 93;
  const spanPitch = parseFloat(document.getElementById('spanPitch').value) || 80;
  const {rollRaw, pitchRaw} = readCmdRaw();
  const rollDeg = -rollRaw, pitchDeg = -pitchRaw;
  const refHeight = parseFloat(document.getElementById('refHeight').value) || 34;
  const scale = parseFloat(document.getElementById('scaleFactor').value) || 2.0;
  const mapping = {
    FL: parseInt(document.getElementById('sidFL').value || '0', 10),
    FR: parseInt(document.getElementById('sidFR').value || '0', 10),
    RL: parseInt(document.getElementById('sidRL').value || '0', 10),
    RR: parseInt(document.getElementById('sidRR').value || '0', 10),
  };
  const speed = parseInt(document.getElementById('applySpeed').value || '2400', 10);
  const acc = parseInt(document.getElementById('applyAcc').value || '50', 10);
  const bias_ticks = parseFloat(document.getElementById('biasTicks').value || '0');
  const r = await fetch('/api/attitude/apply', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      roll: rollDeg, pitch: pitchDeg,
      span_roll: spanRoll, span_pitch: spanPitch,
      ref_height: refHeight, scale: scale,
      r1: 77.816, off1: 31.0, r2: 85.788, off2: 22.5, switch: -30.0,
      mapping, speed, acc, bias_ticks
    })
  });
  return r.json();
}

function setSliderValue(id, v){ const el = document.getElementById(id); el.value = String(v); }

async function sweepRoll(){
  const rollEl = document.getElementById('rollDeg');
  const pitchEl = document.getElementById('pitchDeg');
  const rmin = _numAttr(rollEl, 'min', -20), rmax = _numAttr(rollEl, 'max', 20);
  const step = getSweepStep(); const settle = getSweepSettle();
  logRows = [];
  const rows = [];
  for(let r = rmin; r <= rmax + 1e-9; r += step){
    setSliderValue('rollDeg', r);
    setSliderValue('pitchDeg', 0);
    updateHeights();
    await applyCurrent();
    await delay(settle*1000);
    const imu = await readImuOnce();
    const rr = r * CMD_SCALE; const pp = 0 * CMD_SCALE;
    if(imu){ rows.push({ cmd_roll: rr, cmd_pitch: pp, imu_roll: imu.roll, imu_pitch: imu.pitch, imu_yaw: imu.yaw }); }
  }
  logRows = rows; plotRecorded();
}

async function sweepPitch(){
  const rollEl = document.getElementById('rollDeg');
  const pitchEl = document.getElementById('pitchDeg');
  const pmin = _numAttr(pitchEl, 'min', -20), pmax = _numAttr(pitchEl, 'max', 20);
  const step = getSweepStep(); const settle = getSweepSettle();
  logRows = [];
  const rows = [];
  for(let p = pmin; p <= pmax + 1e-9; p += step){
    setSliderValue('rollDeg', 0);
    setSliderValue('pitchDeg', p);
    updateHeights();
    await applyCurrent();
    await delay(settle*1000);
    const imu = await readImuOnce();
    const rr = 0 * CMD_SCALE; const pp = p * CMD_SCALE;
    if(imu){ rows.push({ cmd_roll: rr, cmd_pitch: pp, imu_roll: imu.roll, imu_pitch: imu.pitch, imu_yaw: imu.yaw }); }
  }
  logRows = rows; plotRecorded();
}

async function sweepDiagPP(){
  const rollEl = document.getElementById('rollDeg');
  const rmin = _numAttr(rollEl, 'min', -20), rmax = _numAttr(rollEl, 'max', 20);
  const step = getSweepStep(); const settle = getSweepSettle();
  logRows = [];
  const rows = [];
  for(let v = rmin; v <= rmax + 1e-9; v += step){
    setSliderValue('rollDeg', v);
    setSliderValue('pitchDeg', v);
    updateHeights();
    await applyCurrent();
    await delay(settle*1000);
    const imu = await readImuOnce();
    const rr = v * CMD_SCALE; const pp = v * CMD_SCALE;
    if(imu){ rows.push({ cmd_roll: rr, cmd_pitch: pp, imu_roll: imu.roll, imu_pitch: imu.pitch, imu_yaw: imu.yaw }); }
  }
  logRows = rows; plotRecorded();
}

async function sweepDiagPN(){
  const rollEl = document.getElementById('rollDeg');
  const rmin = _numAttr(rollEl, 'min', -20), rmax = _numAttr(rollEl, 'max', 20);
  const step = getSweepStep(); const settle = getSweepSettle();
  logRows = [];
  const rows = [];
  for(let v = rmin; v <= rmax + 1e-9; v += step){
    setSliderValue('rollDeg', v);
    setSliderValue('pitchDeg', -v);
    updateHeights();
    await applyCurrent();
    await delay(settle*1000);
    const imu = await readImuOnce();
    const rr = v * CMD_SCALE; const pp = -v * CMD_SCALE;
    if(imu){ rows.push({ cmd_roll: rr, cmd_pitch: pp, imu_roll: imu.roll, imu_pitch: imu.pitch, imu_yaw: imu.yaw }); }
  }
  logRows = rows; plotRecorded();
}

window.sweepRoll = sweepRoll;
window.sweepPitch = sweepPitch;
window.sweepDiagPP = sweepDiagPP;
window.sweepDiagPN = sweepDiagPN;

// (Roll PID card removed; JS removed)
// (old PID helpers removed)

// (PID UI wiring removed by request)

