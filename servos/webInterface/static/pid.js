async function api(method, url, body){
  const res = await fetch(url, { method, headers:{'Content-Type':'application/json'}, body: body? JSON.stringify(body): undefined });
  return res.json();
}

// Connection controls moved to top bar; keep IMU start/stop only

let imuTimer = null;
async function pollIMU(){
  const out = document.getElementById('imuReadout');
  const resp = await api('GET','/api/msp/attitude').catch(()=>({ok:false}));
  if(!resp.ok){ if(out) out.textContent = 'IMU: not connected'; return; }
  const { roll, pitch, yaw } = resp;
  out.textContent = `IMU roll: ${roll.toFixed(1)}°, pitch: ${pitch.toFixed(1)}°, yaw: ${yaw.toFixed(1)}°`;
}
function startImu(){ if(imuTimer) return; imuTimer = setInterval(pollIMU, 100); }
function stopImu(){ if(imuTimer){ clearInterval(imuTimer); imuTimer=null; } }
window.startImu = startImu;
window.stopImu = stopImu;

async function pidStart(){
  const body = {
    kp: parseFloat(document.getElementById('pidKp').value)||0.6,
    ki: parseFloat(document.getElementById('pidKi').value)||0.03,
    kd: parseFloat(document.getElementById('pidKd').value)||0.05,
    rate_hz: parseFloat(document.getElementById('pidRate').value)||20,
    limit_deg: parseFloat(document.getElementById('pidLimit').value)||30,
    span_roll: parseFloat(document.getElementById('pidSpanRoll').value)||93,
    span_pitch: parseFloat(document.getElementById('pidSpanPitch').value)||80,
    ref_height: parseFloat(document.getElementById('pidRefH').value)||34,
    scale: parseFloat(document.getElementById('pidScale').value)||2.0,
    auto_level_gain: parseFloat(document.getElementById('pidAutoGain').value)||0.9,
    auto_level_tau: parseFloat(document.getElementById('pidAutoTau').value)||0.4,
    cmd_slew_deg_per_s: parseFloat(document.getElementById('pidSlew').value)||8,
  };
  const r = await api('POST','/pid/start', body).catch(()=>({ok:false}));
  document.getElementById('pidStatus').textContent = r.ok? 'PID started' : ('PID start failed: '+(r.error||''));
  pidRefresh();
}
async function pidStop(){
  const r = await api('POST','/pid/stop',{}).catch(()=>({ok:false}));
  document.getElementById('pidStatus').textContent = r.ok? 'PID stopped' : 'PID stop failed';
}
async function pidApplyConfig(){
  const body = {
    kp: parseFloat(document.getElementById('pidKp').value)||0.6,
    ki: parseFloat(document.getElementById('pidKi').value)||0.03,
    kd: parseFloat(document.getElementById('pidKd').value)||0.05,
    rate_hz: parseFloat(document.getElementById('pidRate').value)||20,
    limit_deg: parseFloat(document.getElementById('pidLimit').value)||30,
    span_roll: parseFloat(document.getElementById('pidSpanRoll').value)||93,
    span_pitch: parseFloat(document.getElementById('pidSpanPitch').value)||80,
    ref_height: parseFloat(document.getElementById('pidRefH').value)||34,
    scale: parseFloat(document.getElementById('pidScale').value)||2.0,
    auto_level_gain: parseFloat(document.getElementById('pidAutoGain').value)||0.9,
    auto_level_tau: parseFloat(document.getElementById('pidAutoTau').value)||0.4,
    cmd_slew_deg_per_s: parseFloat(document.getElementById('pidSlew').value)||8,
  };
  await api('POST','/pid/config', body).catch(()=>({ok:false}));
  pidRefresh();
}
async function pidSetSetpoint(){
  // Align UI expectation with plant sign: invert user setpoint once here
  const set_roll = (parseFloat(document.getElementById('pidSetRoll').value)||0);
  const base_roll = parseFloat(document.getElementById('pidBaseRoll').value)||0;
  const base_pitch = parseFloat(document.getElementById('pidBasePitch').value)||0;
  await api('POST','/pid/setpoint',{set_roll, base_roll, base_pitch}).catch(()=>({ok:false}));
  pidRefresh();
}
async function pidRefresh(){
  const s = await api('GET','/pid/status').catch(()=>({ok:false}));
  if(!s.ok){ document.getElementById('pidStatus').textContent = 'PID: status unavailable'; return; }
  document.getElementById('pidStatus').textContent = JSON.stringify(s.status, null, 2);
}

window.pidStart = pidStart;
window.pidStop = pidStop;
window.pidApplyConfig = pidApplyConfig;
window.pidSetSetpoint = pidSetSetpoint;

// ----- Simple plot of setpoint vs measured roll -----
let plotTimer = null;
let plotData = [];
function pidPlotStart(){ if(plotTimer) return; plotTimer = setInterval(pidPlotTick, 100); }
function pidPlotStop(){ if(plotTimer){ clearInterval(plotTimer); plotTimer=null; } }
function pidPlotClear(){ plotData = []; pidDraw(); }

async function pidPlotTick(){
  const st = await api('GET','/pid/status').catch(()=>({ok:false}));
  if(!st.ok || !st.status){ return; }
  const now = Date.now();
  // Use exact backend values for consistency with status panel
  const setVal = Number(st.status.set_roll)||0;
  const last = st.status.last || {};
  const measVal = Number(last.meas_roll)||0;
  const errVal = Number(last.err)|| (setVal - measVal);
  plotData.push({ t: now, set: setVal, meas: measVal, err: errVal });
  // Update status panel with the same snapshot
  const statusEl = document.getElementById('pidStatus');
  if(statusEl){ statusEl.textContent = JSON.stringify(st.status, null, 2); }
  if(plotData.length > 1000) plotData.shift();
  pidDraw();
}

function pidDraw(){
  const c = document.getElementById('pidCanvas'); if(!c) return;
  const ctx = c.getContext('2d'); const w=c.width, h=c.height; ctx.clearRect(0,0,w,h);
  ctx.fillStyle = '#0b1222'; ctx.fillRect(0,0,w,h);
  const pad = 40; const W = w - pad*2; const H = h - pad*2;
  // axes
  ctx.strokeStyle='#334155'; ctx.strokeRect(pad,pad,W,H);
  // x by samples
  const n = plotData.length || 1;
  // y range from data (include error) with padding
  let ymin=Infinity,ymax=-Infinity; plotData.forEach(p=>{ ymin=Math.min(ymin, p.set, p.meas, p.err); ymax=Math.max(ymax, p.set, p.meas, p.err); });
  if(!isFinite(ymin)||!isFinite(ymax)){ ymin=-10; ymax=10; }
  if(ymin===ymax){ ymin-=1; ymax+=1; }
  const padY = (ymax - ymin) * 0.1 + 0.5; ymin -= padY; ymax += padY;
  const xr = n-1 || 1, yr = ymax - ymin;
  function X(i){ return pad + (i/xr)*W; }
  function Y(v){ return pad + H - ((v - ymin)/yr)*H; }
  // grid
  ctx.strokeStyle='#1f2937'; ctx.beginPath(); for(let i=0;i<=4;i++){ const y=pad+(i/4)*H; ctx.moveTo(pad,y); ctx.lineTo(pad+W,y);} ctx.stroke();
  // setpoint (cyan, dashed)
  ctx.strokeStyle='#22d3ee'; ctx.lineWidth=2; ctx.setLineDash([6,4]); ctx.beginPath(); plotData.forEach((p,i)=>{ const x=X(i), y=Y(p.set); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }); ctx.stroke(); ctx.setLineDash([]);
  // measured (magenta)
  ctx.strokeStyle='#f472b6'; ctx.lineWidth=2; ctx.beginPath(); plotData.forEach((p,i)=>{ const x=X(i), y=Y(p.meas); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }); ctx.stroke();
  // error (orange, thin)
  ctx.strokeStyle='#f59e0b'; ctx.lineWidth=1.5; ctx.beginPath(); plotData.forEach((p,i)=>{ const x=X(i), y=Y(p.err); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }); ctx.stroke();
  // axes labels & y ticks
  ctx.fillStyle='#9ca3af'; ctx.font='12px Inter, system-ui';
  ctx.fillText('samples', pad+W-60, pad+H+28);
  // y ticks
  ctx.fillStyle='#94a3b8'; ctx.textAlign='right'; ctx.textBaseline='middle';
  for(let i=0;i<=4;i++){ const vy = ymin + (i/4)*(ymax-ymin); const y = Y(vy); ctx.fillText(vy.toFixed(1)+'°', pad-6, y); }
  ctx.textAlign='start'; ctx.textBaseline='alphabetic';
  // legend with color swatches
  const legendX = pad+8, legendY = pad+14;
  function swatch(x,y,color,label){ ctx.fillStyle=color; ctx.fillRect(x,y-8,14,3); ctx.fillStyle='#e5e7eb'; ctx.fillText(label, x+20, y);
  }
  swatch(legendX, legendY, '#22d3ee', 'Setpoint');
  swatch(legendX+110, legendY, '#f472b6', 'Measured');
  swatch(legendX+220, legendY, '#f59e0b', 'Error');
}

window.pidPlotStart = pidPlotStart;
window.pidPlotStop = pidPlotStop;
window.pidPlotClear = pidPlotClear;

// Align with other pages: bind connection buttons on load
window.addEventListener('DOMContentLoaded', ()=>{});

