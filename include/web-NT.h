/*web-NT.h — Async web portal: configuration, live status, OTA (item H).
==========================================================================
Endpoints (all except / and /api/status require HTTP Basic auth,
user "admin", password = configStore.cfg.webPass):

  GET  /                single-page portal (PROGMEM, no filesystem needed)
  GET  /api/status      live JSON snapshot (temp, PWM, alarms, uptime, rssi)
  POST /api/config      save configuration to NVS, then reboot
  POST /api/thresholds  save Low/High temp to NVS, applied live (no reboot)
  POST /update          OTA firmware upload (.bin from CI artifacts)
  POST /reboot          manual restart

Layout: a single page split into four tabs (Telemetry / Control / Network /
Device) switched client-side; the ESP32 still serves one PROGMEM document.
- Telemetry polls /api/status every 2 s; Temperature Range "Set" applies the
  thresholds live via /api/thresholds (the control loop reads cfg each cycle).
- Network/Device cards each POST their own subset to /api/config; the handler
  only updates the fields present, so a partial save keeps the rest and then
  reboots (needed for WiFi/MQTT/identity).
- Control (relay mapping) and the fan-alarm popup are shown but their backend
  lands in later phases.

Design notes:
- ESPAsyncWebServer callbacks run in the async_tcp task: shared state is
  read via the injected fillStatus callback, which takes stateMutex with
  a short timeout on the main-firmware side.
- The portal works both in station mode and in the AP-rescue mode that
  main enables when WiFi association fails (SSID FanController-Setup,
  http://192.168.4.1).
==========================================================================*/
#pragma once
#include <Arduino.h>
#include <functional>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "config-NT.h"

static const char PORTAL_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FAN Controller</title><style>
:root{
--bg:#0b111f;--bg2:#0e1626;--card:#141d30;--card2:#1a2540;--line:#243149;
--txt:#e6edf7;--mut:#8896ac;--accent:#38bdf8;--accent2:#2563eb;
--ok:#34d399;--warn:#fbbf24;--bad:#f87171}
*{box-sizing:border-box}
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
background:radial-gradient(1200px 600px at 50% -10%,#16223b 0%,var(--bg) 60%);
color:var(--txt);margin:0;padding:0;line-height:1.5;-webkit-font-smoothing:antialiased}
.wrap{max-width:760px;margin:0 auto;padding:18px 16px 40px}
header{display:flex;align-items:center;justify-content:space-between;gap:12px;
flex-wrap:wrap;margin-bottom:18px}
.brand{display:flex;align-items:center;gap:12px}
.logo{width:40px;height:40px;border-radius:11px;flex:0 0 auto;
background:linear-gradient(145deg,var(--accent),var(--accent2));
display:grid;place-items:center;box-shadow:0 6px 18px rgba(37,99,235,.35)}
.logo svg{width:22px;height:22px;fill:#fff}
.brand h1{font-size:1.05rem;font-weight:700;margin:0;letter-spacing:.2px}
.brand p{margin:2px 0 0;font-size:.74rem;color:var(--mut);letter-spacing:.6px;text-transform:uppercase}
.conn{display:flex;align-items:center;gap:8px;font-size:.78rem;color:var(--mut);
background:var(--card);border:1px solid var(--line);padding:7px 12px;border-radius:999px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--mut);flex:0 0 auto}
.dot.live{background:var(--ok);box-shadow:0 0 0 4px rgba(52,211,153,.15);animation:pulse 2s infinite}
.dot.dead{background:var(--bad);box-shadow:0 0 0 4px rgba(248,113,113,.15)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.45}}
.tags{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:16px}
.tag{font-size:.72rem;color:var(--txt);background:var(--card);border:1px solid var(--line);
padding:4px 10px;border-radius:6px}
.tag b{color:var(--accent);font-weight:600;text-transform:uppercase}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;margin-bottom:14px}
.metric{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px 16px}
.metric .lab{font-size:.78rem;color:var(--mut);text-transform:uppercase;letter-spacing:.6px;margin-bottom:6px}
.metric .val{font-size:1.7rem;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.1}
.metric .val small{font-size:.85rem;font-weight:600;color:var(--mut);margin-left:3px}
.metric .sub{font-size:.74rem;color:var(--mut);margin-top:2px}
.span2{grid-column:span 2}
.bar{height:8px;border-radius:999px;background:var(--bg2);overflow:hidden;margin-top:10px;border:1px solid var(--line)}
.bar>i{display:block;height:100%;width:0;border-radius:999px;transition:width .5s ease,background .5s ease}
.duo{display:flex;gap:16px}
.duo>div{flex:1}
.badge{display:inline-flex;align-items:center;gap:6px;font-size:.82rem;font-weight:700;
padding:5px 12px;border-radius:999px;white-space:nowrap}
.badge.ok{color:var(--ok);background:rgba(52,211,153,.12);border:1px solid rgba(52,211,153,.3)}
.badge.bad{color:var(--bad);background:rgba(248,113,113,.12);border:1px solid rgba(248,113,113,.3)}
.badge.info{color:var(--accent);background:rgba(56,189,248,.12);border:1px solid rgba(56,189,248,.3)}
.badge.n{color:var(--txt);background:rgba(136,150,172,.12);border:1px solid var(--line)}
.spark{width:100%;height:96px;display:block;margin-top:10px}
section{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:18px;margin-top:16px}
section>h2{font-size:.95rem;color:var(--txt);margin:0 0 4px;display:flex;align-items:center;gap:8px}
section>h2 svg{width:16px;height:16px;fill:var(--accent)}
section>.hint{font-size:.76rem;color:var(--mut);margin:0 0 12px}
.fgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px 14px}
.f-full{grid-column:1/-1}
label{display:block;margin:6px 0 3px;font-size:.76rem;color:var(--mut);font-weight:600}
input,select{width:100%;padding:9px 11px;border-radius:9px;border:1px solid var(--line);
background:var(--bg2);color:var(--txt);font-size:.9rem;transition:border-color .15s,box-shadow .15s}
input:focus,select:focus{outline:0;border-color:var(--accent);box-shadow:0 0 0 3px rgba(56,189,248,.15)}
select{appearance:none;-webkit-appearance:none;padding-right:34px;cursor:pointer;
background-image:url("data:image/svg+xml;charset=utf-8,%3Csvg xmlns='http://www.w3.org/2000/svg' width='14' height='14' viewBox='0 0 24 24' fill='none' stroke='%2338bdf8' stroke-width='2.4' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpath d='M6 9l6 6 6-6'/%3E%3C/svg%3E");
background-repeat:no-repeat;background-position:right 12px center}
input[type=file]{padding:8px;color:var(--mut)}
button{margin-top:16px;padding:11px 18px;border:0;border-radius:10px;font-size:.9rem;
background:linear-gradient(145deg,var(--accent),var(--accent2));color:#fff;font-weight:700;
cursor:pointer;width:100%;transition:filter .15s,transform .05s}
button:hover{filter:brightness(1.08)}button:active{transform:translateY(1px)}
.btnghost{margin-top:0;width:auto;padding:9px 20px}
small{color:var(--mut);font-size:.74rem}
.bulb{display:inline-block;width:24px;height:24px;border-radius:50%;background:var(--ok);vertical-align:middle;cursor:pointer}
.bulb.on{background:var(--bad);animation:pulse 1s infinite}
.tabs{display:flex;gap:6px;margin-bottom:16px}
.tabbtn{flex:1;text-align:center;font-size:.82rem;font-weight:700;padding:10px 8px;border-radius:11px;
background:var(--card);border:1px solid var(--line);color:var(--mut);cursor:pointer;transition:.15s}
.tabbtn.active{background:linear-gradient(145deg,var(--accent),var(--accent2));color:#fff;border-color:transparent}
.tabpane{display:none}
.tabpane.active{display:block}
.tabpane.stale{opacity:.5;filter:saturate(.4)}
.orow2{display:flex;align-items:center;gap:12px;padding:12px 14px;margin-bottom:8px;background:var(--bg2);border:1px solid var(--line);border-radius:11px;cursor:pointer;transition:border-color .15s}
.orow2:hover{border-color:var(--accent)}
.orow2 .oname{flex:0 0 58px;font-weight:700;font-size:.82rem;color:var(--accent);letter-spacing:.4px}
.orow2 .oval{flex:1;font-size:.92rem}
.orow2 .ochev{color:var(--mut);font-size:1.3rem;line-height:1}
.opt.dis{opacity:.38;cursor:not-allowed}
.modal{position:fixed;inset:0;background:rgba(4,8,16,.62);display:none;place-items:center;z-index:50;padding:20px}
.modal.open{display:grid}
.modal-card{background:var(--card2);border:1px solid var(--line);border-radius:16px;padding:20px;max-width:330px;width:100%;box-shadow:0 24px 60px rgba(0,0,0,.55)}
.modal-card h3{margin:0 0 4px;font-size:1rem}
.modal-card p{margin:0 0 14px;font-size:.76rem;color:var(--mut)}
.opt{display:flex;align-items:center;gap:10px;padding:11px 13px;border:1px solid var(--line);border-radius:11px;margin-bottom:8px;cursor:pointer;background:var(--bg2);font-size:.88rem;font-weight:600}
.opt.sel{border-color:var(--accent);background:rgba(56,189,248,.1);box-shadow:0 0 0 2px rgba(56,189,248,.2)}
.opt .rd{width:15px;height:15px;border-radius:50%;border:2px solid var(--mut);flex:0 0 auto}
.opt.sel .rd{border-color:var(--accent);background:radial-gradient(circle,var(--accent) 40%,transparent 45%)}
.mrow{display:flex;gap:10px;margin-top:6px}
.spinner{width:38px;height:38px;border-radius:50%;border:3px solid var(--line);border-top-color:var(--accent);margin:0 auto 14px;animation:spin 1s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
@media(max-width:520px){.grid{grid-template-columns:1fr}.span2{grid-column:auto}.fgrid{grid-template-columns:1fr}
.metric .val{font-size:1.45rem}.tabbtn{font-size:.74rem;padding:9px 4px}
header{justify-content:center;text-align:center}.brand{justify-content:center}.conn{margin:0 auto}.tags{justify-content:center}}
</style></head><body><div class="wrap">
<header>
<div class="brand">
<div class="logo"><svg viewBox="0 0 24 24"><path d="M20.79,13.95L18.46,14.57L16.46,13.44V10.56L18.46,9.43L20.79,10.05L21.31,8.12L19.54,7.65L20,5.88L18.07,5.36L17.45,7.69L15.45,8.82L13,7.4V5.14L14.71,3.43L13.29,2L12,3.29L10.71,2L9.29,3.43L11,5.14V7.4L8.55,8.82L6.55,7.69L5.93,5.36L4,5.88L4.46,7.65L2.69,8.12L3.21,10.05L5.54,9.43L7.54,10.56V13.44L5.54,14.57L3.21,13.95L2.69,15.88L4.46,16.35L4,18.12L5.93,18.64L6.55,16.31L8.55,15.18L11,16.6V18.86L9.29,20.57L10.71,22L12,20.71L13.29,22L14.71,20.57L13,18.86V16.6L15.45,15.18L17.45,16.31L18.07,18.64L20,18.12L19.54,16.35L21.31,15.88L20.79,13.95M12,10A2,2 0 0,1 14,12A2,2 0 0,1 12,14A2,2 0 0,1 10,12A2,2 0 0,1 12,10Z"/></svg></div>
<div><h1>FAN Controller</h1><p>Power &middot; Climatizaci&oacute;n</p></div>
</div>
<div class="conn"><span class="dot" id="dot"></span><span id="connTxt">Conectando&hellip;</span></div>
</header>
<div class="tags">
<span class="tag"><b>Op</b> %MQTTOPER%</span>
<span class="tag"><b>City</b> %MQTTCITY%</span>
<span class="tag"><b>Site</b> %MQTTSITE%</span>
<span class="tag"><b>Subsys</b> %MQTTSUBSYS%</span>
<span class="tag"><b>Client ID</b> %CLIENTID%</span>
</div>

<div class="tabs">
<div class="tabbtn active" data-tab="tele">Telemetry</div>
<div class="tabbtn" data-tab="ctrl">Control</div>
<div class="tabbtn" data-tab="net">Network</div>
<div class="tabbtn" data-tab="dev">Device</div>
</div>

<div class="tabpane active" id="pane-tele">
<div class="grid">
<div class="metric span2">
<div class="lab">Temperature Range</div>
<div class="duo" style="align-items:flex-end">
<div><label>Low Temp &deg;C</label><input type="number" step="0.5" id="inLow" value="%LOWTEMP%"></div>
<div><label>High Temp &deg;C</label><input type="number" step="0.5" id="inHigh" value="%HIGHTEMP%"></div>
<div style="flex:0 0 auto"><button class="btnghost" onclick="setThr()">Set</button></div>
</div>
<div class="sub" id="thrMsg">&nbsp;</div>
</div>
<div class="metric span2">
<div class="lab">Cabinet Temperature</div>
<div class="val" id="mTemp">--<small>&deg;C</small></div>
<div class="sub" id="mTempSub">&nbsp;</div>
<div class="bar"><i id="bTemp"></i></div>
<canvas class="spark" id="spark"></canvas>
</div>
<div class="metric span2">
<div class="duo">
<div><div class="lab">Temp Sensor</div><div id="mTS"><span class="badge n">&mdash;</span></div></div>
<div><div class="lab">Temp Alarm</div><div id="mTA"><span class="badge n">&mdash;</span></div></div>
</div>
</div>
<div class="metric span2">
<div class="lab">Cabinet Fan Speed</div>
<div class="duo">
<div style="text-align:center">
<div style="display:flex;align-items:center;justify-content:center;gap:8px"><span style="font-size:.82rem;color:var(--mut)">FAN 1</span><span class="val" style="font-size:1.35rem" id="mF1">--<small>%</small></span></div>
<div class="bar" style="max-width:68%;margin:10px auto 0"><i id="bF1"></i></div>
<div class="sub" id="mR1">-- rpm</div>
</div>
<div style="text-align:center">
<div style="display:flex;align-items:center;justify-content:center;gap:8px"><span style="font-size:.82rem;color:var(--mut)">FAN 2</span><span class="val" style="font-size:1.35rem" id="mF2">--<small>%</small></span></div>
<div class="bar" style="max-width:68%;margin:10px auto 0"><i id="bF2"></i></div>
<div class="sub" id="mR2">-- rpm</div>
</div>
</div>
</div>
<div class="metric span2"><div class="lab">Fan Alarm</div>
<div class="duo" style="align-items:center;justify-content:space-around;gap:8px">
<div style="flex:0 0 auto;display:flex;align-items:center;gap:8px;white-space:nowrap"><span style="font-size:.82rem;color:var(--mut)">FAN 1</span><span id="aF1"><span class="badge n">&mdash;</span></span></div>
<div style="flex:0 0 auto;display:flex;align-items:center;gap:8px;white-space:nowrap"><span style="font-size:.82rem;color:var(--mut)">FAN 2</span><span id="aF2"><span class="badge n">&mdash;</span></span></div>
<div style="flex:0 0 auto"><span class="bulb" id="aFG" onclick="openFan()" title="Configurar l&oacute;gica de alarma"></span></div>
</div></div>
<div class="metric span2" style="display:flex;align-items:center;justify-content:space-between;gap:12px">
<div class="lab" style="margin:0">Cabinet Door Alarm</div>
<span id="mDoor"><span class="badge n">&mdash;</span></span>
</div>
<div class="metric"><div class="lab">WiFi Signal</div><div class="val" id="mRssi">--<small>dBm</small></div><div class="sub" id="mRssiSub">&nbsp;</div></div>
<div class="metric"><div class="lab">Uptime</div><div class="val" id="mUp" style="font-size:1.25rem">--</div></div>
</div>
</div>

<div class="tabpane" id="pane-ctrl">
<section>
<h2><svg viewBox="0 0 24 24"><path d="M16,7V3H14V7H10V3H8V7H8C7,7 6,8 6,9V14.5L9.5,18V21H14.5V18L18,14.5V9C18,8 17,7 16,7Z"/></svg>Output Relays</h2>
<p class="hint">Toca cada salida para asignarle un actuador. Un actuador ya usado se deshabilita en las dem&aacute;s (exclusi&oacute;n mutua). El guardado real llega en la siguiente fase.</p>
<div class="orow2" data-out="0"><span class="oname">OUT 1</span><span class="oval" id="ov0">&mdash;</span><span class="ochev">&rsaquo;</span></div>
<div class="orow2" data-out="1"><span class="oname">OUT 2</span><span class="oval" id="ov1">&mdash;</span><span class="ochev">&rsaquo;</span></div>
<div class="orow2" data-out="2"><span class="oname">OUT 3</span><span class="oval" id="ov2">&mdash;</span><span class="ochev">&rsaquo;</span></div>
<div class="orow2" data-out="3"><span class="oname">OUT 4</span><span class="oval" id="ov3">&mdash;</span><span class="ochev">&rsaquo;</span></div>
</section>
</div>

<div class="tabpane" id="pane-net">
<section>
<h2><svg viewBox="0 0 24 24"><path d="M12 18a2 2 0 100 4 2 2 0 000-4zM4.9 11 3.5 9.6a12 12 0 0117 0L19 11a10 10 0 00-14.1 0zm2.8 2.8L6.3 12.4a8 8 0 0111.4 0l-1.4 1.4a6 6 0 00-8.6 0z"/></svg>WiFi Setup</h2>
<p class="hint">Al guardar, el dispositivo se reinicia para aplicar los cambios.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>WiFi SSID</label><input name="wifiSsid" value="%WIFISSID%"></div>
<div><label>WiFi Password</label><input name="wifiPass" type="password" placeholder="(sin cambio)"></div>
</div>
<button type="submit">Save and Restart</button></form>
</section>
<section>
<h2><svg viewBox="0 0 24 24"><path d="M5 3a1 1 0 000 2 6 6 0 016 6 1 1 0 002 0A8 8 0 005 3zm0 4a1 1 0 100 2 2 2 0 012 2 1 1 0 102 0 4 4 0 00-4-4zm.5 5.5a1.5 1.5 0 100 3 1.5 1.5 0 000-3z"/></svg>MQTT Setup</h2>
<p class="hint">Al guardar, el dispositivo se reinicia para aplicar los cambios.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>MQTT Server</label><input name="mqttServer" value="%MQTTSERVER%"></div>
<div><label>MQTT Port</label><input name="mqttPort" type="number" value="%MQTTPORT%"></div>
<div><label>MQTT User</label><input name="mqttUser" value="%MQTTUSER%"></div>
<div><label>MQTT Password</label><input name="mqttPass" type="password" placeholder="(sin cambio)"></div>
<div class="f-full"><label>Client ID</label><input value="%CLIENTID%" readonly style="color:var(--mut)"></div>
<div class="f-full"><label>Publish topic (telemetr&iacute;a)</label><input value="%TOPICPUB%" readonly style="color:var(--mut)"></div>
<div class="f-full"><label>Subscribe topic (control)</label><input value="%TOPICSUB%" readonly style="color:var(--mut)"></div>
</div>
<button type="submit">Save and Restart</button></form>
</section>
</div>

<div class="tabpane" id="pane-dev">
<section>
<h2><svg viewBox="0 0 24 24"><path fill-rule="evenodd" d="M8 6h8a2 2 0 012 2v8a2 2 0 01-2 2H8a2 2 0 01-2-2V8a2 2 0 012-2zm0 2v8h8V8H8z"/><path d="M9 3h1v3H9zm4 0h1v3h-1zM9 18h1v3H9zm4 0h1v3h-1zM3 9h3v1H3zm0 4h3v1H3zm15-4h3v1h-3zm0 4h3v1h-3z"/></svg>Device Info</h2>
<p class="hint">Al guardar, el dispositivo se reinicia para aplicar los cambios.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>Operator</label><select name="mqttOper">%OPEROPTS%</select></div>
<div><label>City</label><input name="mqttCity" value="%MQTTCITY%"></div>
<div class="f-full"><label>Site / RBS</label><input name="mqttSite" value="%MQTTSITE%"></div>
<div><label>Subsystem</label><select name="mqttSubsys">%SUBSYSOPTS%</select></div>
<div><label>Zone</label><select name="tzOffset">%TZOPTS%</select></div>
<div><label>MAC address</label><input value="%MAC%" readonly style="color:var(--mut)"></div>
<div class="f-full"><label>Portal password</label><input name="webPass" type="password" placeholder="(sin cambio)"></div>
</div>
<button type="submit">Save and Restart</button>
<br><small>Usuario del portal: <b>admin</b></small></form>
</section>
<section>
<h2><svg viewBox="0 0 24 24"><path d="M12 2 4 6v6c0 5 3.4 8.7 8 10 4.6-1.3 8-5 8-10V6l-8-4zm-1 13-3-3 1.4-1.4L11 12.2l4.6-4.6L17 9l-6 6z"/></svg>Firmware OTA</h2>
<p class="hint">Sube el <b>firmware.bin</b> generado por el CI. El equipo se reinicia al terminar.</p>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Save and Restart</button></form>
<p class="hint" style="text-align:center;margin-top:14px">Firmware actual: <b id="mVer" style="color:var(--txt)">v--</b> &middot; se actualiza solo al flashear una versi&oacute;n nueva</p>
</section>
</div>

</div>

<div class="modal" id="fanModal">
<div class="modal-card">
<h3>Fan Alarm Setup</h3>
<p>Choose which condition triggers the general alarm (and the bulb color).</p>
<div class="opt" data-v="or"><span class="rd"></span>FAN1 <b style="color:var(--mut);font-weight:600">or</b> FAN2</div>
<div class="opt sel" data-v="and"><span class="rd"></span>FAN1 <b style="color:var(--mut);font-weight:600">and</b> FAN2</div>
<div class="opt" data-v="f1"><span class="rd"></span>Only FAN1</div>
<div class="opt" data-v="f2"><span class="rd"></span>Only FAN2</div>
<div class="mrow">
<button onclick="closeFan()">Set</button>
<button onclick="closeFan()" style="background:none;border:1px solid var(--line);color:var(--mut)">Cancel</button>
</div>
</div>
</div>

<div class="modal" id="saveModal">
<div class="modal-card" style="text-align:center">
<div class="spinner"></div>
<h3>Guardado &#10003;</h3>
<p id="saveMsg">Reiniciando el dispositivo&hellip;</p>
</div>
</div>

<div class="modal" id="outModal">
<div class="modal-card">
<h3 id="outTitle">OUT &rarr; actuador</h3>
<p>Elige el actuador que gobierna este rel&eacute;.</p>
<div id="outOpts"></div>
<div class="mrow">
<button onclick="outSet()">Set</button>
<button onclick="outClose()" style="background:none;border:1px solid var(--line);color:var(--mut)">Cancel</button>
</div>
</div>
</div>

<script>
var LOW=%LOWTEMP%,HIGH=%HIGHTEMP%,hist=[],fail=0;
function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v)}
function col(t){if(t>=HIGH)return css('--bad');if(t>=HIGH-3)return css('--warn');
if(t<=LOW)return css('--accent');return css('--ok')}
function clamp(v){return v<0?0:v>100?100:v}
function badge(a){return a?'<span class="badge bad">&#9888; ALARM</span>':'<span class="badge ok">&#10003; OK</span>'}
function drawSpark(){var c=document.getElementById('spark');if(!c)return;
var w=c.clientWidth||300,h=96,dpr=window.devicePixelRatio||1;
c.width=w*dpr;c.height=h*dpr;var x=c.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);x.clearRect(0,0,w,h);
if(hist.length<2)return;var lo=Math.min.apply(0,hist),hi=Math.max.apply(0,hist);
if(hi-lo<1){var m=(hi+lo)/2;hi=m+.5;lo=m-.5}var pad=12,gw=w,gh=h-pad*2;
var p=[];for(var i=0;i<hist.length;i++)p.push([i/(hist.length-1)*gw,pad+gh-(hist[i]-lo)/(hi-lo)*gh]);
function path(){x.beginPath();x.moveTo(p[0][0],p[0][1]);
 for(var i=0;i<p.length-1;i++){var a=p[i>0?i-1:0],b=p[i],d=p[i+1],e=p[i+2]||d;
  x.bezierCurveTo(b[0]+(d[0]-a[0])/6,b[1]+(d[1]-a[1])/6,d[0]-(e[0]-b[0])/6,d[1]-(e[1]-b[1])/6,d[0],d[1]);}}
var cc=col(hist[hist.length-1]).trim();
path();x.lineTo(gw,h);x.lineTo(0,h);x.closePath();
var g=x.createLinearGradient(0,0,0,h);g.addColorStop(0,cc);g.addColorStop(1,'transparent');
x.globalAlpha=.16;x.fillStyle=g;x.fill();x.globalAlpha=1;
path();x.strokeStyle=cc;x.lineWidth=2;x.lineJoin='round';x.lineCap='round';x.stroke();
var lp=p[p.length-1];x.beginPath();x.arc(lp[0],lp[1],3,0,7);x.fillStyle=cc;x.fill()}
function setLive(ok){var d=document.getElementById('dot'),t=document.getElementById('connTxt'),
s=document.getElementById('pane-tele');d.className='dot '+(ok?'live':'dead');
t.textContent=ok?'Online':'Offline';
s.className='tabpane'+(s.classList.contains('active')?' active':'')+(ok?'':' stale')}
document.querySelectorAll('.tabbtn').forEach(function(b){b.onclick=function(){
 document.querySelectorAll('.tabbtn').forEach(function(x){x.classList.remove('active')});
 document.querySelectorAll('.tabpane').forEach(function(x){x.classList.remove('active')});
 b.classList.add('active');document.getElementById('pane-'+b.dataset.tab).classList.add('active');
 if(b.dataset.tab=='tele')drawSpark();
};});
function openFan(){document.getElementById('fanModal').classList.add('open')}
function closeFan(){document.getElementById('fanModal').classList.remove('open')}
document.querySelectorAll('.opt').forEach(function(o){o.onclick=function(){
 document.querySelectorAll('.opt').forEach(function(x){x.classList.remove('sel')});o.classList.add('sel');
};});
var ACTS=[['','— free'],['speedFan1','speedFan1'],['speedFan2','speedFan2'],['doorOpenAlarm','doorOpenAlarm'],['fanAlarm','fanAlarm']];
var outMap=['speedFan1','speedFan2','doorOpenAlarm','fanAlarm'],outEdit=-1,outPick='';
function outText(v){return v===''?'— free':v}
function outRender(){for(var i=0;i<4;i++)document.getElementById('ov'+i).textContent=outText(outMap[i]);}
function openOut(i){outEdit=i;outPick=outMap[i];
 document.getElementById('outTitle').textContent='OUT '+(i+1)+' → actuador';
 var used=outMap.filter(function(v,j){return j!==i&&v!==''});
 var h='';ACTS.forEach(function(a){var v=a[0],dis=(v!==''&&used.indexOf(v)>=0);
  h+='<div class="opt'+(v===outPick?' sel':'')+(dis?' dis':'')+'" data-v="'+v+'"><span class="rd"></span>'+a[1]+'</div>';});
 document.getElementById('outOpts').innerHTML=h;
 document.querySelectorAll('#outOpts .opt').forEach(function(o){o.onclick=function(){
  if(o.classList.contains('dis'))return;
  document.querySelectorAll('#outOpts .opt').forEach(function(x){x.classList.remove('sel')});
  o.classList.add('sel');outPick=o.getAttribute('data-v');};});
 document.getElementById('outModal').classList.add('open');}
function outClose(){document.getElementById('outModal').classList.remove('open')}
function outSet(){outMap[outEdit]=outPick;outRender();outClose();}
document.getElementById('outModal').onclick=function(e){if(e.target===this)outClose();};
document.querySelectorAll('.orow2').forEach(function(r){r.onclick=function(){openOut(+r.getAttribute('data-out'));};});
outRender();
document.querySelectorAll('form[action="/api/config"]').forEach(function(f){f.onsubmit=function(){
 var body=new URLSearchParams();new FormData(f).forEach(function(v,k){body.append(k,v)});
 fetch('/api/config',{method:'POST',body:body}).then(function(r){
  if(!r.ok)return r.text().then(function(t){throw t});
  document.getElementById('saveModal').classList.add('open');
  var msg=document.getElementById('saveMsg'),t0=Date.now();
  msg.textContent='Reiniciando el dispositivo…';
  setTimeout(function ping(){
   fetch('/api/status',{cache:'no-store'}).then(function(rr){
    if(!rr.ok)throw 0;msg.textContent='¡Listo! Recargando…';location.href='/';
   }).catch(function(){
    if(Date.now()-t0>30000){msg.textContent='El equipo tarda o cambió de red. Reconéctate y abre http://fancontroller.local/';}
    else{msg.textContent='Reiniciando… ('+Math.round((Date.now()-t0)/1000)+' s)';setTimeout(ping,1500);}
   });
  },4000);
 }).catch(function(e){alert((''+e)||'Error al guardar');});
 return false;};});
function setThr(){var lo=document.getElementById('inLow').value,hi=document.getElementById('inHigh').value;
 var m=document.getElementById('thrMsg');var b=new URLSearchParams();b.append('lowTemp',lo);b.append('highTemp',hi);
 fetch('/api/thresholds',{method:'POST',body:b}).then(function(r){
  if(!r.ok)return r.text().then(function(t){throw t});return r.json();}).then(function(j){
  LOW=parseFloat(j.lowT);HIGH=parseFloat(j.highT);
  m.textContent='Guardado ✓ ('+LOW+'–'+HIGH+' °C)';m.style.color=css('--ok').trim();
 }).catch(function(e){m.textContent=(''+e)||'Error';m.style.color=css('--bad').trim();});}
async function poll(){try{
 var r=await fetch('/api/status');var d=await r.json();fail=0;setLive(true);
 var t=d.temp,tok=(t!=null&&!isNaN(t));
 if(tok){
  document.getElementById('mTemp').innerHTML=t.toFixed(2)+'<small>&deg;C</small>';
  var c=col(t).trim();document.getElementById('mTemp').style.color=c;
  var bt=document.getElementById('bTemp');bt.style.width=clamp((t-LOW)/(HIGH-LOW)*100)+'%';bt.style.background=c;
  document.getElementById('mTempSub').textContent=t>=HIGH?'Sobre límite alto':(t<=LOW?'Bajo límite':'');
  hist.push(t);if(hist.length>60)hist.shift();drawSpark();
 }else{
  document.getElementById('mTemp').innerHTML='--<small>&deg;C</small>';
  document.getElementById('mTemp').style.color=css('--bad').trim();
  document.getElementById('bTemp').style.width='0%';
  document.getElementById('mTempSub').textContent='Sensor sin lectura';
 }
 var sensorFail=(!tok||d.tempCode=='Sensor Failure'||d.tempCode=='Disconnected');
 document.getElementById('mTS').innerHTML=sensorFail?'<span class="badge bad">&#9888; FAILURE</span>':'<span class="badge ok">&#10003; OK</span>';
 var ta;if(sensorFail)ta='<span class="badge n">&mdash;</span>';
 else if(d.tempAlarm)ta='<span class="badge bad">&#9888; HIGH</span>';
 else if(tok&&d.tempCode&&d.tempCode.indexOf('Low')>=0)ta='<span class="badge info">LOW</span>';
 else ta='<span class="badge ok">&#10003; OK</span>';
 document.getElementById('mTA').innerHTML=ta;
 var p1=d.pwm1,p2=d.pwm2;
 document.getElementById('mF1').innerHTML=p1.toFixed(0)+'<small>%</small>';
 document.getElementById('mF2').innerHTML=p2.toFixed(0)+'<small>%</small>';
 var b1=document.getElementById('bF1'),b2=document.getElementById('bF2');
 b1.style.width=clamp(p1)+'%';b2.style.width=clamp(p2)+'%';
 b1.style.background=b2.style.background=css('--accent').trim();
 document.getElementById('mR1').textContent=(d.fan1Rpm||0).toFixed(0)+' rpm';
 document.getElementById('mR2').textContent=(d.fan2Rpm||0).toFixed(0)+' rpm';
 document.getElementById('aF1').innerHTML=badge(d.fan1Alarm);
 document.getElementById('aF2').innerHTML=badge(d.fan2Alarm);
 document.getElementById('aFG').className='bulb'+((d.fan1Alarm||d.fan2Alarm)?' on':'');
 document.getElementById('mDoor').innerHTML=d.door?'<span class="badge bad">&#9888; OPEN</span>':'<span class="badge ok">&#10003; CLOSED</span>';
 var rs=d.rssi;document.getElementById('mRssi').innerHTML=rs+'<small>dBm</small>';
 document.getElementById('mRssiSub').textContent=rs>=-60?'Excellent':(rs>=-70?'Good':(rs>=-80?'Fair':'Weak'));
 document.getElementById('mUp').textContent=d.uptime;
 document.getElementById('mVer').textContent='v'+d.version;
}catch(e){if(++fail>=2)setLive(false)}}
poll();setInterval(poll,2000);window.addEventListener('resize',drawSpark);
</script></body></html>
)HTML";

class WebPortal {
private:
    AsyncWebServer _server;
    ConfigStore* _store;
    std::function<void(JsonDocument&)> _fillStatus;
    bool _rebootPending = false;
    uint32_t _rebootAt = 0;

    bool _auth(AsyncWebServerRequest* req){
        if (req->authenticate("admin", _store->cfg.webPass)) return true;
        req->requestAuthentication();
        return false;
    }

    static void _copyParam(AsyncWebServerRequest* req, const char* name,
                           char* dst, size_t dstLen, bool skipIfEmpty){
        if (!req->hasParam(name, true)) return;
        String v = req->getParam(name, true)->value();
        if (skipIfEmpty && v.length() == 0) return;   /*passwords: empty = keep*/
        strlcpy(dst, v.c_str(), dstLen);
    }

    /*Build <option> items, marking the one equal to `current` as selected.*/
    /*Options stored/shown capitalized (as typed); the topic lowercases them
    separately. Match ignores case so a device saved before this change
    (lowercase value) still pre-selects the right option until re-saved.*/
    static String _selectOpts(const char* const* labels, int n, const char* current){
        String s;
        for (int i = 0; i < n; i++){
            s += "<option value=\""; s += labels[i]; s += "\"";
            if (String(labels[i]).equalsIgnoreCase(current)) s += " selected";
            s += ">"; s += labels[i]; s += "</option>";
        }
        return s;
    }

    String _renderPortal(){
        String page = FPSTR(PORTAL_HTML);
        page.replace("%WIFISSID%",   _store->cfg.wifiSsid);
        page.replace("%MQTTSERVER%", _store->cfg.mqttServer);
        page.replace("%MQTTPORT%",   String(_store->cfg.mqttPort));
        page.replace("%MQTTUSER%",   _store->cfg.mqttUser);
        page.replace("%HIGHTEMP%",   String(_store->cfg.highTemp, 1));
        page.replace("%LOWTEMP%",    String(_store->cfg.lowTemp, 1));
        page.replace("%MQTTCITY%",   _store->cfg.mqttCity);
        page.replace("%MQTTSITE%",   _store->cfg.mqttSite);
        /*Device-identity tags shown in the dashboard header so the operator
        knows which board of the fleet they are looking at (static config,
        no /api/status change needed).*/
        page.replace("%MQTTOPER%",   _store->cfg.mqttOperator);
        page.replace("%MQTTSUBSYS%", _store->cfg.mqttSubsystem);
        /*Client ID = site-MAC-subsystem, the same value the main builds for
        the MQTT client; the MAC makes it the unique device identity.*/
        String _mac = WiFi.macAddress(); _mac.replace(":", "");   /*uppercase hex*/
        String _siteId = String(_store->cfg.mqttSite) + "-" + _mac;
        /*Client ID = site-MAC (site keeps its typed case, MAC uppercase); no
        subsystem here because it already lives in the topic path.*/
        page.replace("%CLIENTID%", _siteId);
        /*Raw MAC (colon form) shown read-only in the Device tab.*/
        page.replace("%MAC%", WiFi.macAddress());
        /*Pub/sub topics shown read-only in MQTT Setup, lowercased to match the
        main's build (operator/city/site-MAC/subsystem/{telemetria,control}).*/
        String _base = String(_store->cfg.mqttOperator) + "/" + _store->cfg.mqttCity +
                       "/" + _siteId + "/" + _store->cfg.mqttSubsystem;
        _base.toLowerCase();
        page.replace("%TOPICPUB%", _base + "/telemetria");
        page.replace("%TOPICSUB%", _base + "/control");
        /*Stored/shown capitalized; the topic lowercases them at build time.*/
        static const char* const opers[]  = {"Claro","CNT","Tigo"};
        static const char* const subsys[] = {"Power","Generador","Baterias","Seguridad"};
        page.replace("%OPEROPTS%",   _selectOpts(opers, 3, _store->cfg.mqttOperator));
        page.replace("%SUBSYSOPTS%", _selectOpts(subsys, 4, _store->cfg.mqttSubsystem));
        /*Timezone: pick by representative city; option value = UTC offset (h).
        No DST handling (fine for Ecuador); offsets are standard time.*/
        struct Tz { float off; const char* label; };
        static const Tz tzs[] = {
            {-6, "Mexico (UTC-6)"},
            {-5, "Guayaquil / Bogota (UTC-5)"},
            {-4, "Santiago / Caracas / La Paz (UTC-4)"},
            {-3, "Buenos Aires / Sao Paulo (UTC-3)"},
            { 0, "UTC / Londres (UTC+0)"},
            { 1, "Madrid (UTC+1)"},
        };
        String tzOpts;
        for (auto& t : tzs){
            tzOpts += "<option value=\""; tzOpts += String(t.off, 2); tzOpts += "\"";
            float d = t.off - _store->cfg.tzOffset; if (d < 0) d = -d;
            if (d < 0.01f) tzOpts += " selected";
            tzOpts += ">"; tzOpts += t.label; tzOpts += "</option>";
        }
        page.replace("%TZOPTS%", tzOpts);
        return page;
    }

public:
    WebPortal(): _server(80) {}

    void begin(ConfigStore& store, std::function<void(JsonDocument&)> fillStatus,
               const char* mdnsName){
        _store = &store;
        _fillStatus = fillStatus;

        /*Auth required: the portal page embeds the current SSID, MQTT
        server and MQTT user, so an open "/" leaked config to anyone on
        the LAN (or on the rescue AP).*/
        _server.on("/", HTTP_GET, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            req->send(200, "text/html", _renderPortal());
        });

        _server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* req){
            JsonDocument doc;
            if (_fillStatus) _fillStatus(doc);
            String out;
            serializeJson(doc, out);
            req->send(200, "application/json", out);
        });

        _server.on("/api/config", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            /*softAP() needs an 8..63 char key or the rescue AP silently
            comes up open (or fails). Reject a weak new webPass before it
            is ever persisted; empty means "keep current".*/
            if (req->hasParam("webPass", true)){
                String wp = req->getParam("webPass", true)->value();
                if (wp.length() != 0 && (wp.length() < 8 || wp.length() > 63)){
                    req->send(400, "text/plain",
                              "Password del portal invalido: use 8 a 63 caracteres");
                    return;
                }
            }
            AppConfig& c = _store->cfg;
            _copyParam(req, "wifiSsid",   c.wifiSsid,   sizeof(c.wifiSsid),   false);
            _copyParam(req, "wifiPass",   c.wifiPass,   sizeof(c.wifiPass),   true);
            _copyParam(req, "mqttServer", c.mqttServer, sizeof(c.mqttServer), false);
            if (req->hasParam("mqttPort", true))
                c.mqttPort = (uint16_t) req->getParam("mqttPort", true)->value().toInt();
            _copyParam(req, "mqttUser",   c.mqttUser,   sizeof(c.mqttUser),   false);
            _copyParam(req, "mqttPass",   c.mqttPass,   sizeof(c.mqttPass),   true);
            _copyParam(req, "mqttOper",   c.mqttOperator,  sizeof(c.mqttOperator),  false);
            _copyParam(req, "mqttCity",   c.mqttCity,      sizeof(c.mqttCity),      false);
            _copyParam(req, "mqttSite",   c.mqttSite,      sizeof(c.mqttSite),      false);
            _copyParam(req, "mqttSubsys", c.mqttSubsystem, sizeof(c.mqttSubsystem), false);
            _copyParam(req, "webPass",    c.webPass,    sizeof(c.webPass),    true);
            if (req->hasParam("highTemp", true))
                c.highTemp = req->getParam("highTemp", true)->value().toFloat();
            if (req->hasParam("lowTemp", true))
                c.lowTemp = req->getParam("lowTemp", true)->value().toFloat();
            if (req->hasParam("tzOffset", true)){
                float tz = req->getParam("tzOffset", true)->value().toFloat();
                if (tz >= -12.0f && tz <= 14.0f) c.tzOffset = tz;   /*ignore junk*/
            }
            if (!(c.highTemp > c.lowTemp)) { c.highTemp = 43.0f; c.lowTemp = 24.0f; }
            if (!_store->save()){
                req->send(500, "text/plain", "Error al guardar");
                return;
            }
            /*Reply with a page that counts down and returns to the portal once
            the device is back (a plain-text reply left the browser stranded on
            /api/config). Works for changes that keep the same IP; a WiFi change
            moves the device to another network, so the redirect may not reach.*/
            req->send(200, "text/html",
                "<!DOCTYPE html><html lang=\"es\"><head><meta charset=\"utf-8\">"
                "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                "<style>body{font-family:system-ui,sans-serif;background:#111827;"
                "color:#e5e7eb;text-align:center;padding:48px}h2{color:#34d399}</style>"
                "</head><body><h2>Guardado &#10003;</h2>"
                "<p id=\"m\">Reiniciando el dispositivo...</p><script>"
                "var n=10,e=document.getElementById('m');var t=setInterval(function(){"
                "n--;e.textContent='Reiniciando... volviendo en '+n+'s';"
                "if(n<=0){clearInterval(t);location.href='/';}},1000);</script></body></html>");
            _rebootPending = true; _rebootAt = millis() + 2000;
        });

        /*Live temperature thresholds: the control loop reads cfg.highTemp/lowTemp
        each cycle, so these apply immediately — no reboot (unlike /api/config).*/
        _server.on("/api/thresholds", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            AppConfig& c = _store->cfg;
            float hi = c.highTemp, lo = c.lowTemp;
            if (req->hasParam("highTemp", true))
                hi = req->getParam("highTemp", true)->value().toFloat();
            if (req->hasParam("lowTemp", true))
                lo = req->getParam("lowTemp", true)->value().toFloat();
            if (!(hi > lo)){
                req->send(400, "text/plain", "High debe ser mayor que Low");
                return;
            }
            c.highTemp = hi; c.lowTemp = lo;
            if (!_store->save()){
                req->send(500, "text/plain", "Error al guardar");
                return;
            }
            req->send(200, "application/json",
                      "{\"ok\":true,\"lowT\":" + String(lo, 1) +
                      ",\"highT\":" + String(hi, 1) + "}");
        });

        /*OTA: upload handler streams the .bin into the OTA partition.*/
        _server.on("/update", HTTP_POST,
            [this](AsyncWebServerRequest* req){
                if (!_auth(req)) return;
                bool ok = !Update.hasError();
                req->send(ok ? 200 : 500, "text/plain",
                          ok ? "OTA OK. Reiniciando..." : "OTA FAILED");
                if (ok) { _rebootPending = true; _rebootAt = millis() + 1500; }
            },
            [this](AsyncWebServerRequest* req, String filename, size_t index,
                   uint8_t* data, size_t len, bool final){
                if (!req->authenticate("admin", _store->cfg.webPass)) return;
                if (index == 0){
                    Serial.printf("\nOTA start: %s", filename.c_str());
                    Update.begin(UPDATE_SIZE_UNKNOWN);
                }
                if (len) Update.write(data, len);
                if (final){
                    if (Update.end(true)) Serial.printf("\nOTA done: %u bytes", index + len);
                    else Update.printError(Serial);
                }
            });

        _server.on("/reboot", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            req->send(200, "text/plain", "Reiniciando...");
            _rebootPending = true; _rebootAt = millis() + 1000;
        });

        _server.begin();
        if (MDNS.begin(mdnsName)) MDNS.addService("http", "tcp", 80);
        Serial.printf("\nWeb portal up: http://%s.local/\n", mdnsName);
    }

    /*Call from loop(): deferred reboot so HTTP responses flush first.*/
    void handle(){
        if (_rebootPending && (int32_t)(millis() - _rebootAt) >= 0){
            Serial.println("\nRebooting (web portal request)...");
            ESP.restart();
        }
    }
};
