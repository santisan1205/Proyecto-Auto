// =====================================================================
//  Dashboard web (HTML + CSS + JS) servido por el ESP32.
//  Consulta /api/estado cada segundo. No necesita Internet.
//
//  Se guarda en la flash (PROGMEM) como una cadena cruda R"rawliteral(...)"
//  y el ESP32 la envía tal cual en la ruta "/" (ver iniciarWeb()).
//  Partes de la página:
//    - Tarjetas de arriba: humanos, robots, total / 12 y estado.
//    - Puertas: estado y botón Abrir (POST /api/abrir).
//    - Control: Emergencia, Reset, Probar cámara y Calibrar sensores.
//    - Trabajadores dentro: foto, ID, tipo, tiempo y "Marcar que sale".
//    - Historial de accesos y lista de eventos.
//  El JavaScript del final pide el estado, lo dibuja y manda las acciones.
// =====================================================================
#pragma once
#include <Arduino.h>

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Control de Acceso</title>
<style>
:root{--bg:#f3f5f8;--card:#fff;--tx:#1b2130;--mu:#6b7385;--bd:#e2e6ed;--ok:#1f9d55;--bad:#d93b3b;--warn:#d98200;--hum:#2f6fdf;--rob:#8a4fd6}
@media(prefers-color-scheme:dark){:root{--bg:#0f1218;--card:#181d26;--tx:#e8ecf3;--mu:#8e97a8;--bd:#2a313d}}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--tx)}
header{display:flex;justify-content:space-between;align-items:center;gap:12px;padding:12px 16px;background:var(--card);border-bottom:1px solid var(--bd);position:sticky;top:0;z-index:5}
h1{font-size:18px;margin:0}
h2{font-size:15px;margin:0 0 10px;color:var(--mu);font-weight:600;text-transform:uppercase;letter-spacing:.04em}
.st{font-size:13px;color:var(--mu);white-space:nowrap}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--bad);margin-right:6px}
.dot.on{background:var(--ok)}
main{max-width:1150px;margin:0 auto;padding:16px;display:grid;gap:16px}
.banner{display:none;background:var(--bad);color:#fff;padding:14px;border-radius:12px;font-weight:700;text-align:center;font-size:18px;animation:bl 1s infinite}
@keyframes bl{50%{opacity:.55}}
.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:14px}
.kpi .v{font-size:34px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.1}
.kpi .l{color:var(--mu);font-size:13px;margin-top:4px}
.two{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
.door{display:flex;justify-content:space-between;align-items:center;gap:10px}
.door b{font-size:16px}.door small{display:block;color:var(--mu);margin-top:3px}
.pill{display:inline-block;padding:2px 9px;border-radius:999px;font-size:12px;font-weight:600;color:#fff}
.H{background:var(--hum)}.R{background:var(--rob)}.op{background:var(--ok)}.cl{background:var(--mu)}
button{border:0;border-radius:8px;padding:9px 14px;font-weight:600;font-size:14px;cursor:pointer;background:var(--hum);color:#fff}
button:disabled{opacity:.45;cursor:not-allowed}
button.red{background:var(--bad)}button.gray{background:var(--mu)}
.acts{display:flex;flex-wrap:wrap;gap:10px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:12px}
.w{border:1px solid var(--bd);border-radius:10px;overflow:hidden;background:var(--card)}
.w.al{border-color:var(--bad);box-shadow:0 0 0 2px var(--bad)}
.ph,.w img{width:100%;aspect-ratio:4/3;object-fit:cover;display:flex;align-items:center;justify-content:center;background:#8881;color:var(--mu);font-size:13px}
.w .i{padding:8px 10px;display:flex;justify-content:space-between;align-items:center;gap:6px}
.w .tm{font-variant-numeric:tabular-nums;font-weight:700}
.w.al .tm{color:var(--bad)}
.bar{height:4px;background:var(--bd)}.bar div{height:100%;background:var(--ok)}
.w.al .bar div{background:var(--bad)}
.w.sel{border-color:var(--warn);box-shadow:0 0 0 2px var(--warn)}
.w .mk{display:block;width:calc(100% - 20px);margin:0 10px 10px;padding:6px 8px;font-size:12px;background:var(--mu)}
.w.sel .mk{background:var(--warn)}
.empty{color:var(--mu);font-size:14px;padding:8px 0}
.tw{overflow-x:auto}
table{width:100%;border-collapse:collapse;font-size:14px}
th,td{text-align:left;padding:7px 8px;border-bottom:1px solid var(--bd);white-space:nowrap}
th{color:var(--mu);font-weight:600;font-size:12px;text-transform:uppercase}
td img,td .ph{width:52px;height:39px;border-radius:4px;font-size:10px}
.ev{list-style:none;margin:0;padding:0;font-size:14px;max-height:300px;overflow:auto}
.ev li{padding:6px 0;border-bottom:1px solid var(--bd)}.ev time{color:var(--mu);margin-right:8px;font-variant-numeric:tabular-nums}
dialog{border:0;border-radius:12px;padding:0;background:var(--card);color:var(--tx)}
dialog img{display:block;width:min(90vw,480px);image-rendering:pixelated}
</style>
</head>
<body>
<header>
  <h1>Control de Acceso &middot; Planta</h1>
  <span class="st"><span id="dot" class="dot"></span><span id="con">Conectando…</span></span>
</header>
<main>
  <div id="banner" class="banner">⚠ EMERGENCIA ACTIVA — Puertas abiertas (se sigue contando quién sale)</div>

  <section class="kpis">
    <div class="card kpi"><div class="v" id="kH">–</div><div class="l">Humanos en planta</div></div>
    <div class="card kpi"><div class="v" id="kR">–</div><div class="l">Robots en planta</div></div>
    <div class="card kpi"><div class="v" id="kT">–</div><div class="l">Total / capacidad</div></div>
    <div class="card kpi"><div class="v" id="kE" style="font-size:22px">–</div><div class="l">Estado del sistema</div></div>
  </section>

  <section class="two" id="doors"></section>

  <section class="card">
    <h2>Control</h2>
    <div class="acts">
      <button class="red" onclick="post('/api/emergencia','¿Activar EMERGENCIA? Se abrirán ambas puertas.')">Emergencia</button>
      <button class="gray" onclick="post('/api/reset','¿Reiniciar el sistema? Cierra puertas y pone contadores en cero.')">Reset</button>
      <button class="gray" onclick="probarFoto()">Probar cámara</button>
      <button class="gray" onclick="post('/api/calibrar','Recalibrar los sensores. Las dos zonas de acceso deben estar VACÍAS. ¿Continuar?')">Calibrar sensores</button>
    </div>
  </section>

  <section class="card">
    <h2>Trabajadores dentro de la planta</h2>
    <p class="empty" style="margin:-4px 0 10px">El sensor no sabe quién sale. Pulsa <b>Marcar que sale</b> en la tarjeta de quien va a salir; si no marcas a nadie, se asume que sale el de ese tipo que lleva más tiempo dentro.</p>
    <div class="grid" id="dentro"></div>
  </section>

  <section class="two">
    <div class="card">
      <h2>Historial de accesos</h2>
      <div class="tw"><table>
        <thead><tr><th>Foto</th><th>ID</th><th>Tipo</th><th>Entrada</th><th>Salida</th><th>Permanencia</th></tr></thead>
        <tbody id="hist"></tbody>
      </table></div>
    </div>
    <div class="card">
      <h2>Eventos</h2>
      <ul class="ev" id="ev"></ul>
    </div>
  </section>
</main>
<dialog id="dlg" onclick="this.close()"><img id="dimg" alt=""><p id="dmsg" style="margin:0;padding:14px 16px;max-width:min(90vw,480px)"></p></dialog>

<script>
// ---- Utilidades ----
// $: atajo de querySelector. off: diferencia entre el reloj del navegador y
// millis() del ESP32, para mostrar horas reales. D: último estado recibido.
// kDentro / kHist: "huella" de las listas, para no redibujarlas si no cambian
// (así no parpadean las fotos).
const $=s=>document.querySelector(s);
let off=0, D=null, kDentro='', kHist='';
// hora(ms): hora local de un instante millis() del ESP32. dur(s): "1m 05s".
// tipo(t): etiqueta Humano/Robot. foto(x): miniatura de la visita o "Sin foto".
const hora=ms=>new Date(off+ms).toLocaleTimeString();
const dur=s=>{s=Math.max(0,Math.floor(s));return s>=60?Math.floor(s/60)+'m '+String(s%60).padStart(2,'0')+'s':s+'s'};
const tipo=t=>`<span class="pill ${t}">${t=='H'?'Humano':'Robot'}</span>`;
const foto=(x)=>x.foto?`<img src="/foto?v=${x.visita}&b=${D.boot}" alt="${x.id}" onclick="ver(this.src)" onerror="this.outerHTML='<div class=ph>Sin foto</div>'">`:`<div class="ph">Sin foto</div>`;

// Envía una acción al ESP32 (POST). Si msg viene, pide confirmación antes.
// Al terminar refresca el estado.
async function post(url,msg){
  if(msg&&!confirm(msg))return;
  try{const r=await fetch(url,{method:'POST'});if(!r.ok){const j=await r.json().catch(()=>({}));alert(j.error||'Error '+r.status)}}catch(e){alert('Sin conexión con el ESP32')}
  tick();
}
// Abre una foto en grande en el cuadro de diálogo.
function ver(src){$('#dmsg').textContent='';$('#dimg').style.display='';$('#dimg').src=src;$('#dlg').showModal()}
// Botón "Probar cámara": pide una foto nueva. Si el ESP32 responde con
// error, muestra la razón en vez de una imagen rota.
async function probarFoto(){
  const img=$('#dimg'),msg=$('#dmsg');
  img.style.display='none';msg.textContent='Tomando foto...';$('#dlg').showModal();
  try{
    const r=await fetch('/api/foto-prueba?t='+Date.now());
    if(!r.ok){msg.textContent='No se pudo tomar la foto: '+await r.text();return}
    img.src=URL.createObjectURL(await r.blob());img.style.display='';msg.textContent='';
  }catch(e){msg.textContent='Sin conexión con el ESP32'}
}

// Dibuja todo el estado recibido de /api/estado: tarjetas, puertas,
// trabajadores dentro (tiempo, barra de progreso hasta 90 s, marca de
// salida), historial y eventos.
function render(d){
  D=d;
  $('#kH').textContent=d.humanos;
  $('#kR').textContent=d.robots;
  $('#kT').textContent=(d.humanos+d.robots)+' / '+d.max;
  $('#kE').innerHTML=d.emergencia?'<span style="color:var(--bad)">EMERGENCIA</span>':'<span style="color:var(--ok)">Normal</span>'+(d.camara?'':' <small style="color:var(--warn);font-size:13px">(sin cámara)</small>');
  $('#banner').style.display=d.emergencia?'block':'none';

  $('#doors').innerHTML=d.puertas.map(p=>{const k=p.nombre.toLowerCase();return `<div class="card door"><div><b>Puerta de ${k}</b> <span class="pill ${p.abierta?'op':'cl'}">${p.abierta?'ABIERTA':'CERRADA'}</span><small>Sensor: ${p.altura.toFixed(1)} cm sobre el piso (piso a ${p.piso.toFixed(1)} cm)</small></div><button onclick="post('/api/abrir?puerta=${k}')" ${d.emergencia||p.abierta?'disabled':''}>Abrir</button></div>`}).join('');

  // Trabajadores dentro (solo se reconstruye si cambia la lista)
  const k1=d.boot+':'+d.dentro.map(w=>w.visita).join(',');
  if(k1!==kDentro){
    kDentro=k1;
    $('#dentro').innerHTML=d.dentro.length?d.dentro.map(w=>`<div class="w" id="w${w.visita}">${foto(w)}<div class="bar"><div></div></div><div class="i"><div><b>${w.id}</b><br>${tipo(w.tipo)}</div><div style="text-align:right"><span class="tm"></span><br><small class="hi"></small></div></div><button class="mk" onclick="post('/api/marcar-salida?id='+this.dataset.id)"></button></div>`).join(''):'<div class="empty">No hay nadie dentro de la planta.</div>';
  }
  d.dentro.forEach(w=>{
    const el=$('#w'+w.visita);if(!el)return;
    const s=(d.ahora-w.t)/1000;
    el.classList.toggle('al',w.alerta);
    const sel=w.visita===d.proxSalida, mk=el.querySelector('.mk');
    el.classList.toggle('sel',sel);
    mk.textContent=sel?'Sale este · cancelar':'Marcar que sale';
    mk.dataset.id=sel?'':w.id;
    el.querySelector('.tm').textContent=dur(s);
    el.querySelector('.hi').textContent='desde '+hora(w.t);
    el.querySelector('.bar div').style.width=Math.min(100,s/d.tmax*100)+'%';
  });

  // Historial
  const k2=d.boot+':'+d.historial.map(h=>h.visita+'-'+h.ts).join(',');
  if(k2!==kHist){
    kHist=k2;
    $('#hist').innerHTML=d.historial.length?d.historial.map(h=>`<tr><td>${foto(h)}</td><td><b>${h.id}</b></td><td>${tipo(h.tipo)}</td><td>${hora(h.te)}</td><td class="hs"></td><td class="hd" data-v="${h.visita}"></td></tr>`).join(''):'<tr><td colspan="6" class="empty">Sin registros todavía.</td></tr>';
  }
  d.historial.forEach((h,i)=>{
    const tr=$('#hist').children[i];if(!tr||!tr.querySelector('.hs'))return;
    tr.querySelector('.hs').textContent=h.dentro?'— dentro —':hora(h.ts);
    tr.querySelector('.hd').textContent=dur(((h.dentro?d.ahora:h.ts)-h.te)/1000);
  });

  $('#ev').innerHTML=d.eventos.map(e=>`<li><time>${hora(e.t)}</time>${e.txt.replace(/</g,'&lt;')}</li>`).join('');
}

// Pide /api/estado, lo dibuja y actualiza el indicador "Conectado".
// Se llama al cargar la página y luego cada segundo.
async function tick(){
  try{
    const r=await fetch('/api/estado',{cache:'no-store'});
    const d=await r.json();
    off=Date.now()-d.ahora;
    render(d);
    $('#dot').className='dot on';$('#con').textContent='Conectado';
  }catch(e){$('#dot').className='dot';$('#con').textContent='Sin conexión'}
}
tick();setInterval(tick,1000);
</script>
</body>
</html>
)rawliteral";
