(() => {
  const STORAGE_KEY_THRESHOLD = 'battery_threshold_v';
  const STORAGE_KEY_MUTED = 'battery_alarm_muted';
  const STORAGE_KEY_PORT = 'battery_msp_port';
  const STORAGE_KEY_BAUD = 'battery_msp_baud';
  const STORAGE_KEY_SERVO_PORT = 'servo_port';
  const STORAGE_KEY_SERVO_BAUD = 'servo_baud';
  const DEFAULT_THRESHOLD_V = 21.0;
  const POLL_MS = 2000;
  const STATUS_MS = 5000;
  const SERVO_STATUS_MS = 5000;
  const BEEP_INTERVAL_MS = 1000; // repeat roughly every second while low (respects mute)
  let lastBeep = 0;

  function getStoredThreshold() {
    const v = parseFloat(localStorage.getItem(STORAGE_KEY_THRESHOLD) || '');
    return isFinite(v) && v > 0 ? v : DEFAULT_THRESHOLD_V;
  }

  function setStoredThreshold(v) {
    try { localStorage.setItem(STORAGE_KEY_THRESHOLD, String(v)); } catch (_) {}
  }

  function isMuted() {
    return localStorage.getItem(STORAGE_KEY_MUTED) === '1';
  }

  function setMuted(m) {
    try { localStorage.setItem(STORAGE_KEY_MUTED, m ? '1' : '0'); } catch (_) {}
  }

  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

  function pretty(volts) {
    return isFinite(volts) && volts > 0 ? volts.toFixed(1) + ' V' : '--.- V';
  }

  function beep() {
    if (isMuted()) return;
    const now = Date.now();
    if (now - lastBeep < BEEP_INTERVAL_MS) return;
    lastBeep = now;
    try {
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) return;
      const ctx = new AC();
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = 'square';
      osc.frequency.value = 880;
      // Start quiet to avoid click, then ramp to full volume quickly
      gain.gain.setValueAtTime(0.001, ctx.currentTime);
      osc.connect(gain);
      gain.connect(ctx.destination);
      const t0 = ctx.currentTime;
      gain.gain.linearRampToValueAtTime(1.0, t0 + 0.03);
      gain.gain.linearRampToValueAtTime(0.001, t0 + 0.30);
      osc.start();
      setTimeout(() => { try { osc.stop(); ctx.close(); } catch (_) {} }, 320);
      if (navigator.vibrate) { navigator.vibrate(200); }
    } catch (_) {}
  }

  function loadPort() {
    return localStorage.getItem(STORAGE_KEY_PORT) || 'COM11';
  }

  function savePort(p) {
    try { localStorage.setItem(STORAGE_KEY_PORT, p || ''); } catch (_) {}
  }

  function loadBaud() {
    const b = parseInt(localStorage.getItem(STORAGE_KEY_BAUD) || '115200', 10);
    return isFinite(b) && b > 0 ? b : 115200;
  }

  function saveBaud(b) {
    try { localStorage.setItem(STORAGE_KEY_BAUD, String(b)); } catch (_) {}
  }

  function loadServoPort() {
    return localStorage.getItem(STORAGE_KEY_SERVO_PORT) || 'COM5';
  }

  function saveServoPort(p) {
    try { localStorage.setItem(STORAGE_KEY_SERVO_PORT, p || ''); } catch (_) {}
  }

  function loadServoBaud() {
    const b = parseInt(localStorage.getItem(STORAGE_KEY_SERVO_BAUD) || '1000000', 10);
    return isFinite(b) && b > 0 ? b : 1000000;
  }

  function saveServoBaud(b) {
    try { localStorage.setItem(STORAGE_KEY_SERVO_BAUD, String(b)); } catch (_) {}
  }

  async function callJsonWithTimeout(url, options, timeoutMs){
    const controller = new AbortController();
    const id = setTimeout(()=>controller.abort(), timeoutMs||5000);
    try{
      const res = await fetch(url, { ...options, signal: controller.signal });
      const text = await res.text();
      let json;
      try { json = text ? JSON.parse(text) : {}; } catch(e){ json = { ok:false, error: 'Invalid JSON' }; }
      if(!res.ok){ return { ok:false, error: (json && json.error) || res.statusText || 'HTTP error' }; }
      return json;
    } catch(e){
      return { ok:false, error: (e && e.name === 'AbortError') ? 'Timeout' : (e && e.message) || 'Network error' };
    } finally {
      clearTimeout(id);
    }
  }

  function createTopbar() {
    document.body.classList.add('has-topbar');
    const bar = document.createElement('div');
    bar.id = 'batteryTopbar';
    bar.className = 'topbar';
    bar.innerHTML = `
      <div class="topbar-inner">
        <div class="tb-controls">
          <label class="small">MSP</label>
          <select id="tbMspPortSelect" class="small-input"></select>
          <input id="tbMspBaud" class="small-input" type="number" min="9600" step="1" value="115200" />
          <button id="tbMspRefresh">Refresh</button>
          <button id="tbMspConnect">Connect</button>
          <button id="tbMspDisconnect">Disconnect</button>
          <span id="tbMspStatus" class="status small">Battery: Disconnected</span>
        </div>
        <div class="tb-controls">
          <label class="small">Servos</label>
          <select id="tbServoPortSelect" class="small-input"></select>
          <input id="tbServoBaud" class="small-input" type="number" min="9600" step="1" value="1000000" />
          <button id="tbServoRefresh">Refresh</button>
          <button id="tbServoConnect">Connect</button>
          <button id="tbServoDisconnect">Disconnect</button>
          <span id="tbServoStatus" class="status small">Servos: Disconnected</span>
        </div>
        <div class="battery">
          <div class="bicon" aria-hidden="true">
            <div class="bcase"><div class="bfill" style="width:0%"></div></div>
            <div class="bnub"></div>
          </div>
          <span id="batteryVoltage" class="mono">--.- V</span>
          <label class="small thres">Alarm <input id="batteryThreshold" type="number" step="0.1" min="15" max="30" /> V</label>
          <button id="batteryMute" class="mute small" title="Toggle alarm sound"></button>
        </div>
      </div>`;
    document.body.prepend(bar);

    const thresholdEl = bar.querySelector('#batteryThreshold');
    const muteBtn = bar.querySelector('#batteryMute');
    const portSel = bar.querySelector('#tbMspPortSelect');
    const baudEl = bar.querySelector('#tbMspBaud');
    const refreshBtn = bar.querySelector('#tbMspRefresh');
    const connectBtn = bar.querySelector('#tbMspConnect');
    const disconnectBtn = bar.querySelector('#tbMspDisconnect');
    const statusEl = bar.querySelector('#tbMspStatus');
    const servoPortSel = bar.querySelector('#tbServoPortSelect');
    const servoBaudEl = bar.querySelector('#tbServoBaud');
    const servoRefreshBtn = bar.querySelector('#tbServoRefresh');
    const servoConnectBtn = bar.querySelector('#tbServoConnect');
    const servoDisconnectBtn = bar.querySelector('#tbServoDisconnect');
    const servoStatusEl = bar.querySelector('#tbServoStatus');
    thresholdEl.value = String(getStoredThreshold());
    muteBtn.textContent = isMuted() ? 'Unmute' : 'Mute';
    baudEl.value = String(loadBaud());
    muteBtn.addEventListener('click', () => {
      const m = !isMuted();
      setMuted(m);
      muteBtn.textContent = m ? 'Unmute' : 'Mute';
    });
    thresholdEl.addEventListener('change', () => {
      const v = parseFloat(thresholdEl.value);
      if (isFinite(v) && v > 0) setStoredThreshold(v);
    });
    portSel.addEventListener('change', () => savePort(portSel.value));
    baudEl.addEventListener('change', () => {
      const b = parseInt(baudEl.value||'115200',10); if(isFinite(b)&&b>0) saveBaud(b);
    });
    async function loadPorts() {
      const desired = loadPort();
      const desiredServo = loadServoPort();
      try {
        const j = await callJsonWithTimeout('/api/ports', { method: 'GET' }, 4000);
        const ports = Array.isArray(j.ports) ? j.ports : [];
        portSel.innerHTML = '';
        const set = new Set(ports);
        if (!set.has(desired)) {
          // ensure default exists in list so it's selectable
          ports.unshift(desired);
        }
        ports.forEach(p => {
          const opt = document.createElement('option');
          opt.value = p; opt.textContent = p;
          portSel.appendChild(opt);
        });
        portSel.value = desired;
        savePort(portSel.value);
        // mirror to servo list
        if (servoPortSel) {
          servoPortSel.innerHTML = '';
          ports.forEach(p => { const o=document.createElement('option'); o.value=p; o.textContent=p; servoPortSel.appendChild(o); });
          servoPortSel.value = (ports.includes(desiredServo) ? desiredServo : (ports[0] || desiredServo));
          saveServoPort(servoPortSel.value);
          if (servoBaudEl) servoBaudEl.value = String(loadServoBaud());
        }
      } catch (_) {
        // fallback: at least show the desired/default option
        portSel.innerHTML = '';
        const opt = document.createElement('option'); opt.value = desired; opt.textContent = desired; portSel.appendChild(opt);
        portSel.value = desired;
        if (servoPortSel) {
          servoPortSel.innerHTML = '';
          const sopt = document.createElement('option'); sopt.value = desired; sopt.textContent = desired; servoPortSel.appendChild(sopt);
          servoPortSel.value = desired;
          if (servoBaudEl) servoBaudEl.value = String(loadServoBaud());
        }
      }
    }
    loadPorts();
    if (refreshBtn) refreshBtn.addEventListener('click', loadPorts);
    if (servoPortSel) servoPortSel.addEventListener('change', ()=> saveServoPort(servoPortSel.value));
    if (servoBaudEl) servoBaudEl.addEventListener('change', ()=>{ const b=parseInt(servoBaudEl.value||'1000000',10); if(isFinite(b)&&b>0) saveServoBaud(b); });
    connectBtn.addEventListener('click', async () => {
      const port = portSel.value; const baud = parseInt(baudEl.value||'115200',10);
      if(!port){ statusEl.textContent = 'Battery: Missing port'; return; }
      try{
        const r = await fetch('/api/msp/connect', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({port, baud}) });
        const j = await r.json();
        statusEl.textContent = j.ok ? `Battery: Connected (${port})` : `Battery: ${j.error||'Error'}`;
      }catch(e){ statusEl.textContent = 'Battery: Error'; }
    });
    disconnectBtn.addEventListener('click', async () => {
      try{
        const r = await fetch('/api/msp/disconnect', { method:'POST' });
        const j = await r.json();
        statusEl.textContent = j.ok ? 'Battery: Disconnected' : 'Battery: Error';
      }catch(e){ statusEl.textContent = 'Battery: Error'; }
    });

    // Servo connect controls
    if (servoRefreshBtn) servoRefreshBtn.addEventListener('click', loadPorts);
    if (servoConnectBtn) servoConnectBtn.addEventListener('click', async () => {
      const port = servoPortSel && servoPortSel.value; const baudrate = parseInt((servoBaudEl && servoBaudEl.value) || '1000000',10);
      if(!port){ if(servoStatusEl) servoStatusEl.textContent = 'Servos: Missing port'; return; }
      if(servoConnectBtn) { servoConnectBtn.disabled = true; servoConnectBtn.textContent = 'Connecting…'; }
      const j = await callJsonWithTimeout('/api/connect', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({port, baudrate}) }, 6000);
      if(servoStatusEl) servoStatusEl.textContent = j.ok ? `Servos: Connected (${port})` : `Servos: ${j.error||'Error'}`;
      if(servoConnectBtn) { servoConnectBtn.disabled = false; servoConnectBtn.textContent = 'Connect'; }
    });
    if (servoDisconnectBtn) servoDisconnectBtn.addEventListener('click', async () => {
      if(servoDisconnectBtn) { servoDisconnectBtn.disabled = true; servoDisconnectBtn.textContent = 'Disconnecting…'; }
      const j = await callJsonWithTimeout('/api/disconnect', { method:'POST' }, 5000);
      if(servoStatusEl) servoStatusEl.textContent = j.ok ? 'Servos: Disconnected' : `Servos: ${j.error||'Error'}`;
      if(servoDisconnectBtn) { servoDisconnectBtn.disabled = false; servoDisconnectBtn.textContent = 'Disconnect'; }
    });
  }

  async function fetchBattery() {
    try {
      const r = await fetch('/api/msp/battery', { cache: 'no-store' });
      if (!r.ok) throw new Error('not ok');
      const j = await r.json();
      if (!j.ok) throw new Error(j.error || 'err');
      return j;
    } catch (_) { return null; }
  }

  function updateUI(data) {
    const bar = document.getElementById('batteryTopbar');
    if (!bar) return;
    const vEl = bar.querySelector('#batteryVoltage');
    const fillEl = bar.querySelector('.bfill');
    const batteryEl = bar.querySelector('.battery');
    const threshold = getStoredThreshold();

    if (!data || typeof data.voltage !== 'number') {
      vEl.textContent = '--.- V';
      batteryEl.classList.remove('low');
      fillEl.style.width = '0%';
      return;
    }

    const volts = data.voltage;
    vEl.textContent = pretty(volts);

    // Estimate percent for a 6S pack (18.0–25.2V) for visualization only
    const pct = clamp((volts - 18.0) / (25.2 - 18.0), 0, 1);
    fillEl.style.width = Math.round(pct * 100) + '%';

    const isLow = volts > 0 && volts < threshold;
    batteryEl.classList.toggle('low', isLow);
    if (isLow) beep();
  }

  function startPolling() {
    const tick = async () => {
      const data = await fetchBattery();
      updateUI(data);
      setTimeout(tick, POLL_MS);
    };
    tick();
  }

  function startStatusPolling(){
    const step = async () => {
      try{
        const r = await fetch('/api/msp/status', { cache: 'no-store' });
        const j = await r.json();
        const el = document.getElementById('tbMspStatus');
        if(el) el.textContent = j.ok ? 'Battery: Connected' : 'Battery: Disconnected';
        // Grey out IMU controls if not connected (PID page)
        const startBtn = document.getElementById('startImuBtn');
        const stopBtn = document.getElementById('stopImuBtn');
        if(startBtn) startBtn.disabled = !j.ok;
        if(stopBtn) stopBtn.disabled = !j.ok;
      }catch(_){}
      try{
        const rr = await fetch('/api/servo/status', { cache: 'no-store' });
        const sj = await rr.json();
        const sel = document.getElementById('tbServoStatus');
        if(sel) sel.textContent = sj.ok ? 'Servos: Connected' : 'Servos: Disconnected';
      }catch(_){ }
      setTimeout(step, Math.min(STATUS_MS, SERVO_STATUS_MS));
    };
    step();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => { createTopbar(); startPolling(); startStatusPolling(); });
  } else {
    createTopbar();
    startPolling();
    startStatusPolling();
  }
})();


