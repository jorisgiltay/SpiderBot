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
  const rollDeg = parseFloat(document.getElementById('rollDeg').value) || 0;
  const pitchDeg = parseFloat(document.getElementById('pitchDeg').value) || 0;
  const refHeight = parseFloat(document.getElementById('refHeight').value) || 20;
  const scale = parseFloat(document.getElementById('scaleFactor').value) || 1.0;

  document.getElementById('rollLabel').textContent = rollDeg.toFixed(2) + '°';
  document.getElementById('pitchLabel').textContent = pitchDeg.toFixed(2) + '°';

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
  const invert = !!document.getElementById('invertSign').checked;
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
        el.textContent = (invert ? -val : val).toFixed(2) + '°';
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

    fetch('/api/attitude/apply', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        roll: rollDeg, pitch: pitchDeg,
        span_roll: spanRoll, span_pitch: spanPitch,
        ref_height: refHeight, scale: scale,
        r1: 77.816, off1: 31.0, r2: 85.788, off2: 22.5, switch: -30.0,
        mapping, speed, acc, bias_ticks, invert
      })
    }).then(r => r.json()).then(d => {
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

  ['spanRoll','spanPitch','rollDeg','pitchDeg','refHeight','scaleFactor','invertSign','sidFL','sidFR','sidRL','sidRR','applySpeed','applyAcc','biasTicks'].forEach(id => {
    document.getElementById(id).addEventListener('input', updateHeights);
  });
  updateHeights();
}

window.addEventListener('DOMContentLoaded', attach);

// Apply button handler
function applyAngles() {
  const spanRoll = parseFloat(document.getElementById('spanRoll').value) || 93;
  const spanPitch = parseFloat(document.getElementById('spanPitch').value) || 80;
  const rollDeg = parseFloat(document.getElementById('rollDeg').value) || 0;
  const pitchDeg = parseFloat(document.getElementById('pitchDeg').value) || 0;
  const refHeight = parseFloat(document.getElementById('refHeight').value) || 35;
  const scale = parseFloat(document.getElementById('scaleFactor').value) || 1.0;

  const mapping = {
    FL: parseInt(document.getElementById('sidFL').value || '0', 10),
    FR: parseInt(document.getElementById('sidFR').value || '0', 10),
    RL: parseInt(document.getElementById('sidRL').value || '0', 10),
    RR: parseInt(document.getElementById('sidRR').value || '0', 10),
  };

  const speed = parseInt(document.getElementById('applySpeed').value || '2400', 10);
  const acc = parseInt(document.getElementById('applyAcc').value || '50', 10);
  const bias_ticks = parseFloat(document.getElementById('biasTicks').value || '0');

  const invertApply = !!document.getElementById('invertSign').checked;
  fetch('/api/attitude/apply', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      roll: rollDeg, pitch: pitchDeg,
      span_roll: spanRoll, span_pitch: spanPitch,
      ref_height: refHeight, scale: scale,
      r1: 77.816, off1: 31.0, r2: 85.788, off2: 22.5, switch: -30.0,
      mapping, speed, acc, bias_ticks, invert: invertApply
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


