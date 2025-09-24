let pollTimer = null;

async function api(method, url, body){
  const res = await fetch(url, { method, headers:{'Content-Type':'application/json'}, body: body? JSON.stringify(body): undefined });
  return res.json();
}

function qs(id){ return document.getElementById(id); }

async function loadPorts(){
  const data = await api('GET','/api/ports');
  const sel = qs('portSelect'); sel.innerHTML='';
  (data.ports||[]).forEach(p=>{ const o=document.createElement('option'); o.value=p; o.textContent=p; sel.appendChild(o); });
}

async function connect(){
  const port = qs('portSelect').value;
  const baudrate = parseInt(qs('baudInput').value||'1000000',10);
  const data = await api('POST','/api/connect',{port, baudrate});
  qs('connStatus').textContent = data.ok ? 'Connected' : ('Error: '+(data.error||''));
}

async function disconnect(){
  await api('POST','/api/disconnect');
  qs('connStatus').textContent = 'Disconnected';
}

function readDriveParams(){
  return {
    speed: parseInt(qs('driveSpeed').value||'2400',10),
    acc: parseInt(qs('driveAcc').value||'50',10),
    stride_time: parseFloat(qs('driveStride').value||'0.8'),
    step: parseFloat(qs('driveStep').value||'18'),
    lift: parseFloat(qs('driveLift').value||'8'),
    height: parseFloat(qs('driveHeight').value||'20'),
  };
}

async function startMode(mode){
  const payload = { mode, params: readDriveParams() };
  const data = await api('POST','/api/motion/start', payload);
  if(!data.ok){ alert(data.error||'Failed'); }
}

async function stopMode(){
  await api('POST','/api/motion/stop');
}

async function applyHeight(){
  const height = parseFloat(qs('heightRange').value||'20');
  const speed = parseInt(qs('driveSpeed').value||'2400',10);
  const acc = parseInt(qs('driveAcc').value||'50',10);
  const data = await api('POST','/api/motion/height', { height, speed, acc });
  if(!data.ok){ alert(data.error||'Failed'); }
}

window.addEventListener('DOMContentLoaded', ()=>{
  const rp = qs('refreshPorts'); if(rp) rp.addEventListener('click', loadPorts);
  const cb = qs('connectBtn'); if(cb) cb.addEventListener('click', connect);
  const db = qs('disconnectBtn'); if(db) db.addEventListener('click', disconnect);

  qs('btnForward').addEventListener('click', ()=> startMode('forward'));
  qs('btnBackward').addEventListener('click', ()=> startMode('backward'));
  qs('btnLeft').addEventListener('click', ()=> startMode('left'));
  qs('btnRight').addEventListener('click', ()=> startMode('right'));
  const pl = document.getElementById('btnPivotLeft'); if(pl) pl.addEventListener('click', ()=> startMode('pivot_left'));
  const pr = document.getElementById('btnPivotRight'); if(pr) pr.addEventListener('click', ()=> startMode('pivot_right'));
  qs('btnStop').addEventListener('click', stopMode);

  qs('applyHeight').addEventListener('click', applyHeight);
  const hr = qs('heightRange');
  if(hr){
    const lbl = qs('heightLabel');
    const sync = ()=>{ lbl.textContent = `${hr.value} mm`; qs('driveHeight').value = hr.value; };
    hr.addEventListener('input', sync);
    sync();
  }
  const up = document.getElementById('btnUp'); if(up){ up.addEventListener('click', ()=>{ const hr = qs('heightRange'); hr.value = String(Math.min(60, parseFloat(hr.value||'0')+2)); qs('applyHeight').click(); }); }
  const down = document.getElementById('btnDown'); if(down){ down.addEventListener('click', ()=>{ const hr = qs('heightRange'); hr.value = String(Math.max(0, parseFloat(hr.value||'0')-2)); qs('applyHeight').click(); }); }

  if(rp){ loadPorts(); }
});


