/*web-NT.h — Async web portal: configuration, live status, OTA (item H).
==========================================================================
Endpoints (all except / and /api/status require HTTP Basic auth,
user "admin", password = configStore.cfg.webPass):

  GET  /                single-page portal (PROGMEM, no filesystem needed)
  GET  /api/status      live JSON snapshot (temp, PWM, alarms, uptime, rssi)
  POST /api/config      save configuration to NVS, then reboot
  POST /api/thresholds  save Low/High temp to NVS, applied live (no reboot)
  POST /api/hysteresis  save high-temp alarm reset band, applied live
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
#include <memory>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <esp_mac.h>
#include "config-NT.h"

static const char PORTAL_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FAN Controller</title><style>
:root{
--bg:#0b111f;--bg2:#0e1626;--card:#141d30;--card2:#1a2540;--line:#243149;
--txt:#e6edf7;--mut:#8896ac;--accent:#38bdf8;--accent2:#2563eb;
--ok:#34d399;--warn:#fbbf24;--bad:#f87171}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
background:radial-gradient(1200px 600px at 50% -10%,#16223b 0%,var(--bg) 60%);
color:var(--txt);margin:0;padding:0;line-height:1.5;-webkit-font-smoothing:antialiased;visibility:hidden;animation:pgshow 0s linear 4s forwards}
button,[role=button],.tabbtn,.orow2,.opt,.bulb,.pwmg{
touch-action:manipulation;-webkit-touch-callout:none;-webkit-user-select:none;user-select:none}
button:focus:not(:focus-visible),[role=button]:focus:not(:focus-visible){outline:none}
button:focus-visible,[role=button]:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
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
button{-webkit-appearance:none;appearance:none;margin-top:16px;padding:11px 18px;border:0;border-radius:10px;font-size:.9rem;
background:linear-gradient(145deg,var(--accent),var(--accent2));color:#fff;font-weight:700;
cursor:pointer;width:100%;transition:filter .15s,transform .05s}
button:active{transform:translateY(1px)}
.btnghost{margin-top:0;width:auto;padding:9px 20px}
.thr-row{display:grid;grid-template-columns:minmax(96px,150px) minmax(150px,210px) auto;align-items:end;justify-content:start;gap:10px}
.thr-row>div{min-width:0}
.thr-row input{height:42px}
.thr-high{display:flex;align-items:stretch;gap:7px}
.thr-high input{min-width:0}
.thr-set .btnghost{height:42px;padding:0 16px}
.hyst-open{flex:0 0 42px;width:42px;height:42px;margin:0;padding:0;display:grid;place-items:center;
background:var(--bg2);border:1px solid var(--line);border-radius:9px;color:var(--accent)}
.hyst-open svg{width:16px;height:16px;fill:none;stroke:currentColor;stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round}
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
@keyframes pgshow{to{visibility:visible}}
.pc-shell{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 12px 8px;margin-bottom:12px}
.pc-top{display:flex;align-items:center;justify-content:space-between;gap:8px;min-height:26px;margin-bottom:7px}
.pc-tlbl{color:var(--mut);font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:.4px}
.pc-badges{display:inline-flex;gap:6px;flex:0 0 auto}
.pc-badge{padding:4px 10px;border-radius:999px;font-size:.72rem;font-weight:700;white-space:nowrap}
.pc-badge.n{color:var(--ok);background:rgba(52,211,153,.11);border:1px solid rgba(52,211,153,.28)}
.pc-badge.p{color:var(--accent);background:rgba(56,189,248,.12);border:1px solid rgba(56,189,248,.3)}
.pc-box{position:relative;width:100%;aspect-ratio:1.5;min-height:230px;border:1px solid var(--line);border-radius:10px;background:var(--bg2);overflow:hidden}
.pc-box canvas{display:block;width:100%;height:100%;touch-action:none;cursor:grab;border-radius:9px}
.pc-box.drag canvas{cursor:grabbing}
.pc-hint{margin:8px 2px 0;color:var(--mut);font-size:.72rem;text-align:center}
.pc-eq{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:16px}
.pc-eqhead{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:9px;color:var(--mut);font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:.4px}
.pc-eqrange{padding:3px 8px;color:var(--accent);background:rgba(56,189,248,.1);border:1px solid rgba(56,189,248,.25);border-radius:6px;font-size:.68rem;font-weight:700;text-transform:none;letter-spacing:0}
.pc-formula{display:flex;flex-wrap:wrap;align-items:center;justify-content:center;gap:5px;padding:12px 10px;background:var(--bg2);border:1px solid var(--line);border-radius:11px;color:var(--txt);font-size:.82rem;font-weight:700;margin-bottom:8px}
.pc-frac{display:inline-grid;grid-template-rows:auto auto;text-align:center;font-size:.72rem;line-height:1.2}
.pc-frac span:first-child{padding:0 6px 3px;border-bottom:1px solid var(--mut)}
.pc-frac span:last-child{padding:3px 6px 0}
.pc-pow{align-self:flex-start;padding:1px 5px;color:var(--accent);background:rgba(56,189,248,.12);border-radius:5px;font-size:.68rem}
.pc-lims{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.pc-lim{display:flex;flex-direction:column;gap:2px;padding:8px 10px;background:var(--bg2);border:1px solid var(--line);border-radius:11px;color:var(--mut);font-size:.7rem;font-weight:700}
.pc-lim b{color:var(--txt);font-size:.74rem;font-weight:700}
.hy-card{max-width:560px;max-height:calc(100dvh - 40px);overflow-y:auto;overscroll-behavior:contain;
-webkit-overflow-scrolling:touch;scrollbar-width:thin;scrollbar-color:#34445f transparent;touch-action:pan-y}
.hy-shell{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 12px 8px;margin:12px 0}
.hy-top{display:flex;align-items:center;justify-content:space-between;gap:8px;min-height:27px;margin-bottom:7px}
.hy-tlbl{min-width:0;color:var(--mut);font-size:.72rem;font-weight:700;text-transform:uppercase}
.hy-badges{display:inline-flex;align-items:center;justify-content:flex-end;gap:6px;flex:0 0 auto}
.hy-badge{min-height:27px;display:inline-flex;align-items:center;padding:4px 10px;border-radius:999px;font-size:.72rem;font-weight:700;white-space:nowrap}
.hy-badge.n{color:var(--ok);background:rgba(52,211,153,.11);border:1px solid rgba(52,211,153,.28)}
.hy-badge.reset{color:var(--accent);background:rgba(56,189,248,.11);border:1px solid rgba(56,189,248,.3)}
.hy-box{position:relative;width:100%;aspect-ratio:1.58;min-height:260px;overflow:hidden;background:var(--bg2);border:1px solid var(--line);border-radius:10px}
.hy-box canvas{display:block;width:100%;height:100%;border-radius:9px;cursor:grab;touch-action:pan-y}
.hy-box canvas:focus-visible{outline:2px solid var(--accent);outline-offset:-3px}
.hy-box.drag canvas{cursor:grabbing}
.hy-hint{min-height:17px;margin:8px 2px 0!important;color:var(--mut);font-size:.72rem!important;line-height:1.35;text-align:center}
.hy-hint span{display:block}
#hystModal .mrow{flex:0 0 auto}
.pwmg-wrap{display:flex;gap:16px;justify-content:center;padding:4px 0}
.pwmg{flex:1;max-width:160px;text-align:center;cursor:pointer;padding:8px 6px;border-radius:14px;border:1px solid transparent}
.pwmg-ring{position:relative;width:100%;max-width:130px;margin:0 auto;transition:filter .15s}
.pwmg-ring svg{width:100%;height:auto;display:block;transform:rotate(-90deg)}
.pwmg-trk{fill:none;stroke:var(--bg2);stroke-width:10}
.pwmg-val{fill:none;stroke:var(--accent);stroke-width:10;stroke-linecap:round;stroke-dasharray:0 314.16;transition:stroke-dasharray .5s ease,stroke .3s ease}
.pwmg-ctr{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;gap:1px;pointer-events:none}
.pwmg-pct{font-size:1.55rem;font-weight:700;font-variant-numeric:tabular-nums;color:var(--txt);line-height:1}
.pwmg-u{font-size:.8rem;font-weight:600;color:var(--mut)}
.pwmg-lbl{margin-top:6px;color:var(--mut);font-size:.78rem;font-weight:700;letter-spacing:.4px}
@media(hover:hover) and (pointer:fine){
button:hover{filter:brightness(1.08)}
.orow2:hover{border-color:var(--accent)}
.pwmg:hover .pwmg-ring{filter:brightness(1.08)}
.hyst-open:hover{border-color:var(--accent);background:rgba(56,189,248,.08)}
}
@media(max-width:520px){
#pwmModal .modal-card{padding:16px}
.pc-box{aspect-ratio:1.4;min-height:168px}
.pc-shell{padding:10px 10px 6px}
.pc-eq{padding:11px 10px}
.pc-formula{padding:11px 8px;font-size:.75rem}
.pc-frac{min-width:0}
.pwmg-wrap{gap:10px}
.pwmg-pct{font-size:1.35rem}
.thr-row{grid-template-columns:minmax(0,.9fr) minmax(0,1.35fr) auto;gap:6px}
.thr-row label{font-size:.68rem;white-space:nowrap}
.thr-high{gap:5px}
.hyst-open{flex-basis:38px;width:38px}
.thr-set{grid-column:auto}
.thr-set .btnghost{width:auto;padding:0 10px}
#hystModal .modal-card{padding:16px}
.hy-shell{padding:10px 10px 7px}
.hy-top{align-items:flex-start;flex-direction:column;gap:6px}
.hy-badges{width:100%;flex-wrap:wrap;justify-content:flex-start;gap:5px}
.hy-badge{padding-inline:8px;font-size:.68rem}
.hy-box{aspect-ratio:1.2;min-height:250px}
}
@media(max-width:520px){.grid{grid-template-columns:1fr}.span2{grid-column:auto}.fgrid{grid-template-columns:1fr}
.metric .val{font-size:1.45rem}.tabbtn{font-size:.74rem;padding:9px 4px}
header{justify-content:center;text-align:center}.brand{justify-content:center}.conn{margin:0 auto}.tags{justify-content:center}}
</style></head><body><div class="wrap">
<header>
<div class="brand">
<div class="logo"><svg viewBox="0 0 24 24"><path d="M20.79,13.95L18.46,14.57L16.46,13.44V10.56L18.46,9.43L20.79,10.05L21.31,8.12L19.54,7.65L20,5.88L18.07,5.36L17.45,7.69L15.45,8.82L13,7.4V5.14L14.71,3.43L13.29,2L12,3.29L10.71,2L9.29,3.43L11,5.14V7.4L8.55,8.82L6.55,7.69L5.93,5.36L4,5.88L4.46,7.65L2.69,8.12L3.21,10.05L5.54,9.43L7.54,10.56V13.44L5.54,14.57L3.21,13.95L2.69,15.88L4.46,16.35L4,18.12L5.93,18.64L6.55,16.31L8.55,15.18L11,16.6V18.86L9.29,20.57L10.71,22L12,20.71L13.29,22L14.71,20.57L13,18.86V16.6L15.45,15.18L17.45,16.31L18.07,18.64L20,18.12L19.54,16.35L21.31,15.88L20.79,13.95M12,10A2,2 0 0,1 14,12A2,2 0 0,1 12,14A2,2 0 0,1 10,12A2,2 0 0,1 12,10Z"/></svg></div>
<div><h1>FAN Controller</h1><p>Power &middot; Cooling</p></div>
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
<div class="lab">Cabinet Temperature</div>
<div class="val" id="mTemp">--<small>&deg;C</small></div>
<div class="sub" id="mTempSub">&nbsp;</div>
<div class="bar"><i id="bTemp"></i></div>
<canvas class="spark" id="spark"></canvas>
</div>
<div class="metric span2">
<div class="lab">Temperature Range</div>
<div class="thr-row">
<div><label>Low Temp &deg;C</label><input type="number" step="0.5" id="inLow" value="%LOWTEMP%"></div>
<div><label>High Temp &deg;C</label><div class="thr-high"><input type="number" step="0.5" id="inHigh" value="%HIGHTEMP%"><button type="button" class="hyst-open" onclick="openHyst()" aria-label="Configure high-temperature hysteresis" aria-haspopup="dialog" aria-controls="hystModal"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M3 17H16V7H21"/><path d="M21 7H8V17H3"/></svg></button></div></div>
<div class="thr-set"><button class="btnghost" onclick="setThr()">Set</button></div>
</div>
<div class="sub" id="thrMsg">&nbsp;</div>
</div>
<div class="metric span2">
<div class="lab">Temperature Sensor</div>
<div class="duo" style="align-items:center;justify-content:space-around;gap:8px">
<div style="flex:0 0 auto;display:flex;align-items:center;gap:8px;white-space:nowrap"><span style="font-size:.82rem;color:var(--mut)">SENSOR</span><span id="mTS"><span class="badge n">&mdash;</span></span></div>
<div style="flex:0 0 auto;display:flex;align-items:center;gap:8px;white-space:nowrap"><span style="font-size:.82rem;color:var(--mut)">TEMP</span><span id="mTA"><span class="badge n">&mdash;</span></span></div>
</div></div>
<div class="metric span2">
<div class="lab">Fan Speed Monitoring</div>
<div class="duo">
<div style="text-align:center">
<div style="font-size:.82rem;color:var(--mut);margin-bottom:6px">FAN 1</div>
<div class="val" id="mR1" style="font-size:1.5rem">-- rpm</div>
</div>
<div style="text-align:center">
<div style="font-size:.82rem;color:var(--mut);margin-bottom:6px">FAN 2</div>
<div class="val" id="mR2" style="font-size:1.5rem">-- rpm</div>
</div>
</div>
</div>
<div class="metric span2"><div class="lab">Fan Alarm</div>
<div class="duo" style="align-items:center;justify-content:space-around;gap:8px">
<div style="flex:0 0 auto;display:flex;align-items:center;gap:8px;white-space:nowrap"><span style="font-size:.82rem;color:var(--mut)">FAN 1</span><span id="aF1"><span class="badge n">&mdash;</span></span></div>
<div style="flex:0 0 auto;display:flex;align-items:center;gap:8px;white-space:nowrap"><span style="font-size:.82rem;color:var(--mut)">FAN 2</span><span id="aF2"><span class="badge n">&mdash;</span></span></div>
<div style="flex:0 0 auto"><span class="bulb" id="aFG" onclick="openFan()" role="button" tabindex="0" aria-label="Configure fan alarm logic"></span></div>
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
<p class="hint">Tap each output to assign it a signal. A signal already in use is disabled on the others (mutual exclusion). Saved to NVS, applied live.</p>
<div class="orow2" data-out="0"><span class="oname">OUT 1</span><span class="oval" id="ov0">&mdash;</span><span class="ochev">&rsaquo;</span></div>
<div class="orow2" data-out="1"><span class="oname">OUT 2</span><span class="oval" id="ov1">&mdash;</span><span class="ochev">&rsaquo;</span></div>
<div class="orow2" data-out="2"><span class="oname">OUT 3</span><span class="oval" id="ov2">&mdash;</span><span class="ochev">&rsaquo;</span></div>
<div class="orow2" data-out="3"><span class="oname">OUT 4</span><span class="oval" id="ov3">&mdash;</span><span class="ochev">&rsaquo;</span></div>
</section>
<section>
<h2><svg viewBox="0 0 24 24" style="fill:none;stroke:var(--accent);stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round"><path d="M2 17H6V7H12V17H18V7H22"/></svg>PWM Control - Fan</h2>
<p class="hint">The % commanding each fan. <b>Tap a gauge</b> to adjust the PWM curve (shared by both, saved to NVS, applied live).</p>
<div class="pwmg-wrap">
<div class="pwmg" onclick="openPwm()" role="button" tabindex="0" aria-label="Adjust PWM curve for Fan 1">
<div class="pwmg-ring"><svg viewBox="0 0 120 120" aria-hidden="true"><circle class="pwmg-trk" cx="60" cy="60" r="50"></circle><circle class="pwmg-val" id="gF1" cx="60" cy="60" r="50"></circle></svg><div class="pwmg-ctr"><span class="pwmg-pct" id="gF1t">--</span><span class="pwmg-u">%</span></div></div>
<div class="pwmg-lbl">FAN 1</div>
</div>
<div class="pwmg" onclick="openPwm()" role="button" tabindex="0" aria-label="Adjust PWM curve for Fan 2">
<div class="pwmg-ring"><svg viewBox="0 0 120 120" aria-hidden="true"><circle class="pwmg-trk" cx="60" cy="60" r="50"></circle><circle class="pwmg-val" id="gF2" cx="60" cy="60" r="50"></circle></svg><div class="pwmg-ctr"><span class="pwmg-pct" id="gF2t">--</span><span class="pwmg-u">%</span></div></div>
<div class="pwmg-lbl">FAN 2</div>
</div>
</div>
</section>
</div>

<div class="tabpane" id="pane-net">
<section>
<h2><svg viewBox="0 0 24 24"><path d="M12 18a2 2 0 100 4 2 2 0 000-4zM4.9 11 3.5 9.6a12 12 0 0117 0L19 11a10 10 0 00-14.1 0zm2.8 2.8L6.3 12.4a8 8 0 0111.4 0l-1.4 1.4a6 6 0 00-8.6 0z"/></svg>WiFi Setup</h2>
<p class="hint">On save, the device reboots to apply the changes.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>WiFi SSID</label><input name="wifiSsid" value="%WIFISSID%"></div>
<div><label>WiFi Password</label><input name="wifiPass" type="password" placeholder="(unchanged)"></div>
</div>
<button type="submit">Save and Restart</button></form>
</section>
<section>
<h2><svg viewBox="0 0 24 24"><path d="M5 3a1 1 0 000 2 6 6 0 016 6 1 1 0 002 0A8 8 0 005 3zm0 4a1 1 0 100 2 2 2 0 012 2 1 1 0 102 0 4 4 0 00-4-4zm.5 5.5a1.5 1.5 0 100 3 1.5 1.5 0 000-3z"/></svg>MQTT Setup</h2>
<p class="hint">On save, the device reboots to apply the changes.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>MQTT Server</label><input name="mqttServer" value="%MQTTSERVER%"></div>
<div><label>MQTT Port</label><input name="mqttPort" type="number" value="%MQTTPORT%"></div>
<div><label>MQTT User</label><input name="mqttUser" value="%MQTTUSER%"></div>
<div><label>MQTT Password</label><input name="mqttPass" type="password" placeholder="(unchanged)"></div>
<div class="f-full"><label>Client ID</label><input value="%CLIENTID%" readonly style="color:var(--mut)"></div>
<div class="f-full"><label>Publish topic (telemetr&iacute;a)</label><input value="%TOPICPUB%" readonly style="color:var(--mut)"></div>
<div class="f-full"><label>Subscribe topic (control)</label><input value="%TOPICSUB%" readonly style="color:var(--mut)"></div>
</div>
<button type="submit">Save and Restart</button></form>
<div class="f-full" style="margin-top:14px"><label>CA Certificate &middot; <span id="caState" style="color:var(--accent)">%CACERTSTATE%</span></label>
<input type="file" id="caFile" accept=".pem,.crt,.cer,.txt">
<div class="mrow" style="margin-top:8px">
<button type="button" style="margin-top:0;flex:1" onclick="caSave()">Save Certificate</button>
<button type="button" style="margin-top:0;flex:1;background:none;border:1px solid var(--line);color:var(--mut)" onclick="caReset()">Factory CA</button>
</div>
<div class="sub" id="caMsg" style="font-size:.74rem;color:var(--mut);min-height:1.1em;margin-top:6px">&nbsp;</div>
</div>
</section>
</div>

<div class="tabpane" id="pane-dev">
<section>
<h2><svg viewBox="0 0 24 24"><path fill-rule="evenodd" d="M8 6h8a2 2 0 012 2v8a2 2 0 01-2 2H8a2 2 0 01-2-2V8a2 2 0 012-2zm0 2v8h8V8H8z"/><path d="M9 3h1v3H9zm4 0h1v3h-1zM9 18h1v3H9zm4 0h1v3h-1zM3 9h3v1H3zm0 4h3v1H3zm15-4h3v1h-3zm0 4h3v1h-3z"/></svg>Device Info</h2>
<p class="hint">On save, the device reboots to apply the changes.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>Operator</label><select name="mqttOper">%OPEROPTS%</select></div>
<div><label>City</label><input name="mqttCity" value="%MQTTCITY%"></div>
<div class="f-full"><label>Site / RBS</label><input name="mqttSite" value="%MQTTSITE%"></div>
<div><label>Subsystem</label><select name="mqttSubsys">%SUBSYSOPTS%</select></div>
<div><label>Zone</label><select name="tzOffset">%TZOPTS%</select></div>
<div><label>MAC address</label><input value="%MAC%" readonly style="color:var(--mut)"></div>
<div class="f-full"><label>Portal password</label><input name="webPass" type="password" placeholder="(unchanged)"></div>
</div>
<button type="submit">Save and Restart</button>
<br><small>Portal user: <b>admin</b></small></form>
</section>
<section>
<h2><svg viewBox="0 0 24 24"><path d="M12 2 4 6v6c0 5 3.4 8.7 8 10 4.6-1.3 8-5 8-10V6l-8-4zm-1 13-3-3 1.4-1.4L11 12.2l4.6-4.6L17 9l-6 6z"/></svg>Firmware OTA</h2>
<p class="hint">Upload the <b>firmware.bin</b> from CI. The device reboots when done.</p>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Save and Restart</button></form>
<div class="sub" id="otaLine" style="text-align:center;font-size:.74rem;min-height:1.1em;margin-top:8px">&nbsp;</div>
<p class="hint" style="text-align:center;margin-top:8px">Current firmware: <b id="mVer" style="color:var(--txt)">v--</b> &middot; updates only when you flash a new version</p>
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
<button onclick="fanSet()">Set</button>
<button onclick="closeFan()" style="background:none;border:1px solid var(--line);color:var(--mut)">Cancel</button>
</div>
</div>
</div>

<div class="modal" id="pwmModal">
<div class="modal-card" style="max-width:440px;max-height:calc(100dvh - 40px);overflow-y:auto">
<h3>PWM Curve</h3>
<div class="pc-shell">
<div class="pc-top"><span class="pc-tlbl">PWM(T) / PWM_MAX</span><span class="pc-badges"><span id="pcN" class="pc-badge n">n = --</span><span id="pcP" class="pc-badge p">p = --</span></span></div>
<div id="pcBox" class="pc-box"><canvas id="pcCanvas" aria-label="PWM curve vs temperature"></canvas></div>
<p class="pc-hint">Drag the <b style="color:var(--ok)">n</b> and <b style="color:var(--accent)">p</b> points to shape the curve.</p>
</div>
<div class="pc-eq">
<div class="pc-eqhead"><span>Normalized PWM function</span><span class="pc-eqrange">1 &le; p &le; 10</span></div>
<div id="pcEq"></div>
</div>
<div class="mrow">
<button onclick="pwmSet()">Set</button>
<button onclick="pwmClose()" style="background:none;border:1px solid var(--line);color:var(--mut)">Close</button>
</div>
</div>
</div>

<div class="modal" id="hystModal" role="dialog" aria-modal="true" aria-labelledby="hystTitle">
<div class="modal-card hy-card">
<h3 id="hystTitle">Temperature Hysteresis</h3>
<div class="hy-shell">
<div class="hy-top"><span class="hy-tlbl">HIGH_T alarm hysteresis</span><span class="hy-badges"><span id="hyN" class="hy-badge n">n = -- &deg;C</span><span id="hyReset" class="hy-badge reset">Alarm reset = -- &deg;C</span></span></div>
<div id="hyBox" class="hy-box"><canvas id="hyCanvas" tabindex="0" role="slider" aria-label="High-temperature alarm hysteresis n" aria-valuemin="0" aria-valuenow="%TEMPHYST%"></canvas></div>
<p class="hy-hint"><span>Drag <b style="color:var(--ok)">n</b> to set the alarm reset.</span><span>Max n = 20% (HIGH_T &minus; LOW_T).</span></p>
</div>
<div class="mrow">
<button id="hySetBtn" onclick="hystSet()">Set</button>
<button onclick="hystClose()" style="background:none;border:1px solid var(--line);color:var(--mut)">Close</button>
</div>
</div>
</div>

<div class="modal" id="saveModal">
<div class="modal-card" style="text-align:center">
<div class="spinner"></div>
<h3>Saved &#10003;</h3>
<p id="saveMsg">Rebooting the device&hellip;</p>
</div>
</div>

<div class="modal" id="otaModal">
<div class="modal-card" style="text-align:center">
<div class="spinner"></div>
<h3>Actualizando firmware</h3>
<p id="otaMsg">Uploading&hellip;</p>
</div>
</div>

<div class="modal" id="outModal">
<div class="modal-card">
<h3 id="outTitle">OUT &rarr; signal</h3>
<p>Choose the signal that drives this relay.</p>
<div id="outOpts"></div>
<div class="mrow">
<button onclick="outSet()">Set</button>
<button onclick="outClose()" style="background:none;border:1px solid var(--line);color:var(--mut)">Cancel</button>
</div>
</div>
</div>

<script>
var LOW=%LOWTEMP%,HIGH=%HIGHTEMP%,HYST=%TEMPHYST%,hist=[],fail=0;
var _cssMemo={};function css(v){if(_cssMemo[v]==null)_cssMemo[v]=getComputedStyle(document.documentElement).getPropertyValue(v);return _cssMemo[v]}
function col(t){if(t>=HIGH)return css('--bad');if(t>=HIGH-3)return css('--warn');
if(t<=LOW)return css('--accent');return css('--ok')}
function clamp(v){return v<0?0:v>100?100:v}
function badge(a){return a?'<span class="badge bad">&#9888; ALARM</span>':'<span class="badge ok">&#10003; OK</span>'}
function drawSpark(){var c=document.getElementById('spark');if(!c)return;
var w=c.clientWidth||300,h=96,dpr=window.devicePixelRatio||1;
c.width=w*dpr;c.height=h*dpr;var x=c.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);x.clearRect(0,0,w,h);
if(!hist.length)return;var samples=hist.length===1?[hist[0],hist[0]]:hist;
var lo=Math.min.apply(0,samples),hi=Math.max.apply(0,samples);
if(hi-lo<1){var m=(hi+lo)/2;hi=m+.5;lo=m-.5}var pad=12,gw=w,gh=h-pad*2;
var p=[];for(var i=0;i<samples.length;i++)p.push([i/(samples.length-1)*gw,pad+gh-(samples[i]-lo)/(hi-lo)*gh]);
function path(){x.beginPath();x.moveTo(p[0][0],p[0][1]);
 for(var i=0;i<p.length-1;i++){var a=p[i>0?i-1:0],b=p[i],d=p[i+1],e=p[i+2]||d;
  x.bezierCurveTo(b[0]+(d[0]-a[0])/6,b[1]+(d[1]-a[1])/6,d[0]-(e[0]-b[0])/6,d[1]-(e[1]-b[1])/6,d[0],d[1]);}}
var cc=col(samples[samples.length-1]).trim();
path();x.lineTo(gw,h);x.lineTo(0,h);x.closePath();
var g=x.createLinearGradient(0,0,0,h);g.addColorStop(0,cc);g.addColorStop(1,'transparent');
x.globalAlpha=.16;x.fillStyle=g;x.fill();x.globalAlpha=1;
path();x.strokeStyle=cc;x.lineWidth=2;x.lineJoin='round';x.lineCap='round';x.stroke();
var lp=p[p.length-1];x.beginPath();x.arc(lp[0],lp[1],3,0,7);x.fillStyle=cc;x.fill()}
var _live=null;
function setLive(ok){if(ok===_live)return;_live=ok;   /*touch the DOM only when Online<->Offline actually flips*/
 document.getElementById('dot').className='dot '+(ok?'live':'dead');
 document.getElementById('connTxt').textContent=ok?'Online':'Offline';
 document.getElementById('pane-tele').classList.toggle('stale',!ok);}
function showTab(name){
 document.querySelectorAll('.tabbtn').forEach(function(x){x.classList.toggle('active',x.dataset.tab===name)});
 document.querySelectorAll('.tabpane').forEach(function(x){x.classList.remove('active')});
 var p=document.getElementById('pane-'+name);if(p)p.classList.add('active');
 if(name=='tele')drawSpark();}
document.querySelectorAll('.tabbtn').forEach(function(b){b.onclick=function(){
 try{localStorage.setItem('tab',b.dataset.tab)}catch(e){}
 showTab(b.dataset.tab);};});
(function(){var t;try{t=localStorage.getItem('tab')}catch(e){}   /*restore last tab after a reboot/reload*/
 if(t&&document.getElementById('pane-'+t))showTab(t);})();
var fanLogic=0,FANV=['or','and','f1','f2'];   /*index = stored logic value*/
function openFan(){document.querySelectorAll('#fanModal .opt').forEach(function(o){
 o.classList.toggle('sel',o.getAttribute('data-v')===FANV[fanLogic]);});
 document.getElementById('fanModal').classList.add('open')}
function closeFan(){document.getElementById('fanModal').classList.remove('open')}
function fanSet(){var s=document.querySelector('#fanModal .opt.sel');
 var v=s?FANV.indexOf(s.getAttribute('data-v')):0;if(v<0)v=0;
 var b=new URLSearchParams();b.append('logic',v);
 fetch('/api/fanlogic',{method:'POST',body:b}).then(function(r){if(r.ok)fanLogic=v;});
 closeFan();}
document.querySelectorAll('.opt').forEach(function(o){o.onclick=function(){
 document.querySelectorAll('.opt').forEach(function(x){x.classList.remove('sel')});o.classList.add('sel');
};});
var ACTS=[['','— free'],['tempAlarm','tempAlarm'],['doorOpenAlarm','doorOpenAlarm'],['fanAlarm','fanAlarm']];
var NUMSIG=['','tempAlarm','doorOpenAlarm','fanAlarm'];   /*numeric relayMap -> name*/
var outMap=['doorOpenAlarm','fanAlarm','tempAlarm',''],outEdit=-1,outPick='';
function outText(v){return v===''?'— free':v}
function outRender(){for(var i=0;i<4;i++)document.getElementById('ov'+i).textContent=outText(outMap[i]);}
function openOut(i){outEdit=i;outPick=outMap[i];
 document.getElementById('outTitle').textContent='OUT '+(i+1)+' → signal';
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
function outSet(){outMap[outEdit]=outPick;outRender();
 var b=new URLSearchParams();for(var i=0;i<4;i++)b.append('r'+i,NUMSIG.indexOf(outMap[i]));
 fetch('/api/relays',{method:'POST',body:b});
 outClose();}
document.getElementById('outModal').onclick=function(e){if(e.target===this)outClose();};
document.querySelectorAll('.orow2').forEach(function(r){r.onclick=function(){openOut(+r.getAttribute('data-out'));};});
outRender();
document.querySelectorAll('form[action="/api/config"]').forEach(function(f){f.onsubmit=function(){
 var body=new URLSearchParams();new FormData(f).forEach(function(v,k){body.append(k,v)});
 fetch('/api/config',{method:'POST',body:body}).then(function(r){
  if(!r.ok)return r.text().then(function(t){throw t});
  document.getElementById('saveModal').classList.add('open');
  var msg=document.getElementById('saveMsg'),t0=Date.now();
  msg.textContent='Rebooting the device…';
  setTimeout(function ping(){
   fetch('/api/status',{cache:'no-store'}).then(function(rr){
    if(!rr.ok)throw 0;msg.textContent='Done! Reloading…';location.href='/';
   }).catch(function(){
    if(Date.now()-t0>30000){msg.textContent='The device is taking long or changed network. Reconnect and open http://fancontroller.local/';}
    else{msg.textContent='Rebooting… ('+Math.round((Date.now()-t0)/1000)+' s)';setTimeout(ping,1500);}
   });
  },4000);
 }).catch(function(e){alert((''+e)||'Save failed');});
 return false;};});
function otaMsg2(txt,cvar){var m=document.getElementById('otaLine');
 m.textContent=txt;m.style.color=css(cvar).trim();m.style.opacity='1';}
function caMsgSet(t,cvar){var m=document.getElementById('caMsg');m.textContent=t;m.style.color=css(cvar).trim();}
function caSave(){var f=document.getElementById('caFile').files[0];
 if(!f){caMsgSet('Choose a certificate file first (.pem / .crt)','--warn');return;}
 var rd=new FileReader();
 rd.onload=function(){var p=(''+rd.result).trim();
  if(p.indexOf('-----BEGIN CERTIFICATE-----')!==0){caMsgSet('Not a PEM certificate (must start with -----BEGIN CERTIFICATE-----)','--warn');return;}
  var b=new URLSearchParams();b.append('pem',p);
  fetch('/api/cacert',{method:'POST',body:b}).then(function(r){
   if(!r.ok)return r.text().then(function(t){throw t});return r.json();}).then(function(){
   document.getElementById('caState').textContent='Custom';
   document.getElementById('caFile').value='';
   caMsgSet('Saved ✓ — applies on the next MQTT reconnect/restart','--ok');
  }).catch(function(e){caMsgSet((''+e)||'Save failed','--bad');});};
 rd.onerror=function(){caMsgSet('Could not read the file','--bad');};
 rd.readAsText(f);}
function caReset(){var b=new URLSearchParams();b.append('action','reset');
 fetch('/api/cacert',{method:'POST',body:b}).then(function(r){if(!r.ok)throw 0;return r.json();}).then(function(){
  document.getElementById('caState').textContent='Factory (Let’s Encrypt)';
  caMsgSet('Factory CA restored ✓','--ok');
 }).catch(function(){caMsgSet('Reset failed','--bad');});}
var _ota=document.querySelector('form[action="/update"]');
if(_ota)_ota.onsubmit=function(){
 document.getElementById('otaLine').innerHTML='&nbsp;';   /*limpia el resultado anterior al reintentar*/
 var fi=_ota.querySelector('input[type=file]');
 if(!fi.files.length){otaMsg2('Choose a .bin file first','--warn');return false;}
 var mod=document.getElementById('otaModal'),mm=document.getElementById('otaMsg');
 mm.textContent='Uploading… 0%';mod.classList.add('open');
 var x=new XMLHttpRequest();x.open('POST','/update');
 x.upload.onprogress=function(e){if(e.lengthComputable)mm.textContent='Uploading… '+Math.round(e.loaded/e.total*100)+'%';};
 x.onload=function(){mod.classList.remove('open');
  if(x.status===200){otaMsg2('Firmware uploaded ✓ · rebooting…','--ok');
   var t0=Date.now();(function ping(){fetch('/api/status',{cache:'no-store'}).then(function(rr){
    if(!rr.ok)throw 0;location.href='/';}).catch(function(){
    if(Date.now()-t0>30000)otaMsg2('The device is taking long; reload the page manually.','--warn');
    else setTimeout(ping,1500);});})();
  }else{otaMsg2('Update failed (code '+x.status+'). Check the .bin.','--bad');}};
 x.onerror=function(){mod.classList.remove('open');otaMsg2('Network error during upload.','--bad');};
 x.send(new FormData(_ota));
 return false;};

/* --- Fan PWM curve popup (Control tab): two draggable points n & p --- */
(function(){
 var LOW_T=+LOW,HIGH_T=+HIGH,MARG=6;
 var st={n:%PWMN%,p:%PWMP%,cx:0.62,drag:null};
 var saved={n:st.n,p:st.p,cx:st.cx};
 var cv=document.getElementById('pcCanvas'),g=cv.getContext('2d'),box=document.getElementById('pcBox');
 var eqEl=document.getElementById('pcEq'),nB=document.getElementById('pcN'),pB=document.getElementById('pcP');
 var plot=null;
 function cl(v,a,b){return Math.min(b,Math.max(a,v));}
 function np(pr){if(pr<=0)return st.n;if(pr>=1)return 1;return st.n+(1-st.n)*Math.pow(pr,st.p);}
 function pT(t){if(t<=LOW_T)return st.n;if(t>=HIGH_T)return 1;return np((t-LOW_T)/(HIGH_T-LOW_T));}
 function size(){var r=cv.getBoundingClientRect(),d=window.devicePixelRatio||1;cv.width=Math.round(r.width*d);cv.height=Math.round(r.height*d);g.setTransform(d,0,0,d,0,0);}
 function calc(){var r=cv.getBoundingClientRect(),c=r.width<360,l=c?38:46,ri=c?12:18,tp=14,bt=c?40:38;plot={w:r.width,h:r.height,l:l,t:tp,r:r.width-ri,b:r.height-bt};plot.iw=plot.r-plot.l;plot.ih=plot.b-plot.t;plot.mn=LOW_T-MARG;plot.mx=HIGH_T+MARG;}
 function tX(t){return plot.l+((t-plot.mn)/(plot.mx-plot.mn))*plot.iw;}
 function vY(v){return plot.b-cl(v,0,1)*plot.ih;}
 function prX(pr){return tX(LOW_T+pr*(HIGH_T-LOW_T));}
 function xPr(px){var t=plot.mn+((px-plot.l)/plot.iw)*(plot.mx-plot.mn);return cl((t-LOW_T)/(HIGH_T-LOW_T),0.04,0.96);}
 function yV(py){return cl((plot.b-py)/plot.ih,0,1);}
 function rre(x,y,w,h,rd){g.beginPath();g.moveTo(x+rd,y);g.arcTo(x+w,y,x+w,y+h,rd);g.arcTo(x+w,y+h,x,y+h,rd);g.arcTo(x,y+h,x,y,rd);g.arcTo(x,y,x+w,y,rd);g.closePath();}
 function grid(){
  g.clearRect(0,0,plot.w,plot.h);g.fillStyle='#0e1626';g.fillRect(0,0,plot.w,plot.h);
  g.strokeStyle='#243149';g.lineWidth=1;g.setLineDash([4,6]);
  for(var i=0;i<=4;i++){var y=plot.t+plot.ih*i/4;g.beginPath();g.moveTo(plot.l,y);g.lineTo(plot.r,y);g.stroke();}
  [LOW_T,HIGH_T].forEach(function(t){var x=tX(t);g.beginPath();g.moveTo(x,plot.t);g.lineTo(x,plot.b);g.stroke();});
  g.setLineDash([]);g.strokeStyle='#40506a';g.lineWidth=1.2;g.beginPath();g.moveTo(plot.l,plot.t);g.lineTo(plot.l,plot.b);g.lineTo(plot.r,plot.b);g.stroke();
  g.fillStyle='#8896ac';g.font='700 10px system-ui,sans-serif';g.textAlign='right';g.textBaseline='middle';
  g.fillText('1',plot.l-7,vY(1));g.fillText('0',plot.l-7,vY(0));
  g.textAlign='center';g.textBaseline='top';
  g.fillText('LOW '+LOW_T.toFixed(0)+'°',tX(LOW_T),plot.b+8);
  g.fillText('HIGH '+HIGH_T.toFixed(0)+'°',tX(HIGH_T),plot.b+8);
 }
 function curve(){
  var lx=tX(LOW_T),hx=tX(HIGH_T);
  g.fillStyle='rgba(56,189,248,.08)';g.fillRect(lx,plot.t,hx-lx,plot.ih);
  g.strokeStyle='#38bdf8';g.lineWidth=2;g.lineCap='round';g.lineJoin='round';g.beginPath();
  var s=Math.max(100,Math.round(plot.iw));
  for(var i=0;i<=s;i++){var t=plot.mn+(plot.mx-plot.mn)*i/s,x=tX(t),y=vY(pT(t));if(i===0)g.moveTo(x,y);else g.lineTo(x,y);}
  g.stroke();
 }
 function pt(x,y,lbl,stk,glow,c1,c2){
  g.save();
  g.fillStyle=glow;g.beginPath();g.arc(x,y,13,0,6.2832);g.fill();
  g.fillStyle='#0e1626';g.strokeStyle=stk;g.lineWidth=3;g.beginPath();g.arc(x,y,6.5,0,6.2832);g.fill();g.stroke();
  g.font='700 10px system-ui,sans-serif';var w=g.measureText(lbl).width+16,h=22,lx=cl(x-w/2,plot.l,plot.r-w),ly=y-34;if(ly<plot.t+2)ly=y+16;
  rre(lx,ly,w,h,8);var gr=g.createLinearGradient(lx,ly,lx+w,ly+h);gr.addColorStop(0,c1);gr.addColorStop(1,c2);g.fillStyle=gr;g.fill();
  g.fillStyle='#fff';g.textAlign='center';g.textBaseline='middle';g.fillText(lbl,lx+w/2,ly+h/2);g.restore();
 }
 function pts(){
  pt(plot.l,vY(st.n),'n = '+st.n.toFixed(2),'#34d399','rgba(52,211,153,.18)','#34d399','#0f9f75');
  pt(prX(st.cx),vY(np(st.cx)),'p = '+st.p.toFixed(2),'#38bdf8','rgba(56,189,248,.18)','#38bdf8','#2563eb');
 }
 function draw(){size();calc();grid();curve();pts();}
 function eq(){
  var n=st.n.toFixed(2),ga=(1-st.n).toFixed(2),p=st.p.toFixed(2);
  nB.textContent='n = '+n;pB.textContent='p = '+p;
  eqEl.innerHTML='<div class="pc-formula"><span>'+n+' + '+ga+' &middot;</span><span>(</span><span class="pc-frac"><span>T &minus; LOW_T</span><span>HIGH_T &minus; LOW_T</span></span><span>)</span><span class="pc-pow">'+p+'</span></div><div class="pc-lims"><div class="pc-lim"><span>T &le; LOW_T ('+LOW_T.toFixed(0)+'°)</span><b>PWM = '+n+'</b></div><div class="pc-lim"><span>T &ge; HIGH_T ('+HIGH_T.toFixed(0)+'°)</span><b>PWM = 1.00</b></div></div>';
 }
 function toC(e){var r=cv.getBoundingClientRect();return{x:e.clientX-r.left,y:e.clientY-r.top};}
 function setN(e){st.n=yV(toC(e).y);eq();draw();}
 function setP(e){if(!plot)return;var q=toC(e),pr=xPr(q.x),mv=st.n+Math.max(0.012,(1-st.n)*0.02),val=cl(yV(q.y),mv,0.985);if(Math.abs(1-st.n)<0.001)return;var nr=cl((val-st.n)/(1-st.n),0.015,0.985),ex=Math.log(nr)/Math.log(pr);st.cx=pr;st.p=cl(isFinite(ex)?ex:st.p,1,10);eq();draw();}
 function tgt(e){if(!plot)return null;var q=toC(e),ny=vY(st.n);if(Math.hypot(q.x-plot.l,q.y-ny)<=24||(q.x<tX(LOW_T)&&Math.abs(q.y-ny)<=16))return 'n';var px=prX(st.cx),py=vY(np(st.cx));if(Math.hypot(q.x-px,q.y-py)<=24)return 'p';var pr=xPr(q.x);if(q.x>=tX(LOW_T)&&q.x<=tX(HIGH_T)&&Math.abs(q.y-vY(np(pr)))<=18)return 'p';return null;}
 cv.addEventListener('pointerdown',function(e){var t=tgt(e);if(!t)return;st.drag=t;box.classList.add('drag');cv.setPointerCapture(e.pointerId);(t==='n'?setN:setP)(e);});
 cv.addEventListener('pointermove',function(e){if(!st.drag)return;(st.drag==='n'?setN:setP)(e);});
 function up(e){if(!st.drag)return;st.drag=null;box.classList.remove('drag');if(cv.hasPointerCapture(e.pointerId))cv.releasePointerCapture(e.pointerId);}
 cv.addEventListener('pointerup',up);cv.addEventListener('pointercancel',up);
 window.addEventListener('resize',function(){if(document.getElementById('pwmModal').classList.contains('open'))draw();});
 window.openPwm=function(){var li=document.getElementById('inLow'),hi=document.getElementById('inHigh');LOW_T=li?+li.value:+LOW;HIGH_T=hi?+hi.value:+HIGH;document.getElementById('pwmModal').classList.add('open');requestAnimationFrame(function(){eq();draw();});};
 window.pwmClose=function(){st.n=saved.n;st.p=saved.p;st.cx=saved.cx;st.drag=null;box.classList.remove('drag');document.getElementById('pwmModal').classList.remove('open');};
 window.pwmSet=function(){var b=new URLSearchParams();b.append('pwmN',st.n.toFixed(3));b.append('pwmP',st.p.toFixed(3));fetch('/api/pwmcurve',{method:'POST',body:b}).then(function(r){if(!r.ok)throw 0;return r.json();}).then(function(){saved.n=st.n;saved.p=st.p;saved.cx=st.cx;pwmClose();}).catch(function(){alert('Could not save the PWM curve.');});};
})();

/* --- High-temperature alarm hysteresis popup: draggable reset band n --- */
(function(){
 var LT=+LOW,HT=+HIGH,plot=null;
 var st={n:+HYST,saved:+HYST,drag:false};
 var mod=document.getElementById('hystModal'),cv=document.getElementById('hyCanvas'),g=cv.getContext('2d'),box=document.getElementById('hyBox');
 var nEl=document.getElementById('hyN'),resetEl=document.getElementById('hyReset'),setBtn=document.getElementById('hySetBtn');
 function hc(v,a,b){return Math.min(b,Math.max(a,v));}
 function maxN(){return Math.floor(Math.max(0,HT-LT)*2+.000001)/10;}
 function resetT(){return HT-st.n;}
 function calc(){var w=cv.clientWidth,h=cv.clientHeight,c=w<390;plot={w:w,h:h,l:c?48:58,r:w-(c?54:58),t:20,b:h-(c?52:50)};plot.on=plot.t+31;plot.off=plot.b-29;plot.mn=LT-5;plot.mx=HT+5;}
 function tx(t){return plot.l+(t-plot.mn)/(plot.mx-plot.mn)*(plot.r-plot.l);}
 function xt(x){return plot.mn+(x-plot.l)/(plot.r-plot.l)*(plot.mx-plot.mn);}
 function rr(x,y,w,h,r){r=Math.min(r,w/2,h/2);g.beginPath();g.moveTo(x+r,y);g.arcTo(x+w,y,x+w,y+h,r);g.arcTo(x+w,y+h,x,y+h,r);g.arcTo(x,y+h,x,y,r);g.arcTo(x,y,x+w,y,r);g.closePath();}
 function thresholdLabel(x,title,value,color,left){g.save();g.textAlign=left?'right':'left';g.textBaseline='top';g.font='700 10px system-ui,sans-serif';g.fillStyle=color;g.fillText(title,x+(left?-5:5),plot.b+8);g.font='600 9px system-ui,sans-serif';g.fillStyle='#8896ac';g.fillText(value,x+(left?-5:5),plot.b+23);g.restore();}
 function draw(){if(!plot)return;
  var lx=tx(LT),rx=tx(resetT()),hx=tx(HT),handleY=(plot.on+plot.off)/2,ph=plot.b-plot.t;
  g.clearRect(0,0,plot.w,plot.h);g.fillStyle='#0e1626';g.fillRect(0,0,plot.w,plot.h);
  g.save();g.strokeStyle='rgba(136,150,172,.12)';g.lineWidth=1;g.setLineDash([3,5]);
  for(var i=0;i<=4;i++){var y=plot.t+ph*i/4;g.beginPath();g.moveTo(plot.l,y);g.lineTo(plot.r,y);g.stroke();}g.restore();
  g.save();g.strokeStyle='#53627b';g.lineWidth=1.2;g.beginPath();g.moveTo(plot.l,plot.t);g.lineTo(plot.l,plot.b);g.lineTo(plot.r,plot.b);g.stroke();g.restore();
  [[lx,'rgba(136,150,172,.48)'],[rx,'rgba(52,211,153,.65)'],[hx,'rgba(248,113,113,.68)']].forEach(function(a){g.save();g.strokeStyle=a[1];g.lineWidth=1.2;g.setLineDash([5,5]);g.beginPath();g.moveTo(a[0],plot.b);g.lineTo(a[0],plot.off);g.stroke();g.restore();});
  g.save();g.lineWidth=2;g.lineJoin='round';g.lineCap='round';
  g.strokeStyle='#34d399';g.beginPath();g.moveTo(plot.l,plot.off);g.lineTo(hx,plot.off);g.stroke();
  g.strokeStyle='#f87171';g.beginPath();g.moveTo(rx,plot.on);g.lineTo(plot.r,plot.on);g.stroke();
  g.strokeStyle='#34d399';g.beginPath();g.moveTo(rx,plot.on);g.lineTo(rx,plot.off);g.stroke();
  g.strokeStyle='#f87171';g.beginPath();g.moveTo(hx,plot.off);g.lineTo(hx,plot.on);g.stroke();g.restore();
  g.save();g.font='700 10px system-ui,sans-serif';g.textAlign='left';g.textBaseline='bottom';g.fillStyle='#f87171';g.fillText('TEMP ALARM',plot.l+8,plot.on-8);g.fillStyle='#34d399';g.fillText('OK',plot.l+8,plot.off-8);
  g.translate(13,(plot.t+plot.b)/2);g.rotate(-Math.PI/2);g.textAlign='center';g.fillStyle='#8896ac';g.font='700 9px system-ui,sans-serif';g.fillText('ALARM STATE',0,0);g.restore();
  thresholdLabel(lx,'LOW_T',LT.toFixed(1)+' \u00B0C','#8896ac',true);
  thresholdLabel(rx,'RESET',resetT().toFixed(1)+' \u00B0C','#34d399',true);
  thresholdLabel(hx,'HIGH_T',HT.toFixed(1)+' \u00B0C','#f87171',false);
  g.save();g.fillStyle='#8896ac';g.font='700 10px system-ui,sans-serif';g.textAlign='center';g.textBaseline='bottom';g.fillText('Temperature',(plot.l+plot.r)/2,plot.h-4);g.restore();
  g.save();g.fillStyle='rgba(52,211,153,.18)';g.beginPath();g.arc(rx,handleY,13,0,Math.PI*2);g.fill();g.fillStyle='#0e1626';g.strokeStyle='#34d399';g.lineWidth=3;g.beginPath();g.arc(rx,handleY,6.5,0,Math.PI*2);g.fill();g.stroke();
  var label='n = '+st.n.toFixed(1)+' \u00B0C';g.font='700 10px system-ui,sans-serif';var lw=Math.ceil(g.measureText(label).width)+16,lh=24,onRight=rx<(plot.l+plot.r)/2,labelX=onRight?rx+14:rx-lw-14,labelY=handleY-lh/2;
  rr(labelX,labelY,lw,lh,7);g.fillStyle='rgba(20,29,48,.96)';g.fill();g.strokeStyle='rgba(52,211,153,.42)';g.lineWidth=1;g.stroke();g.fillStyle='#34d399';g.textAlign='center';g.textBaseline='middle';g.fillText(label,labelX+lw/2,handleY+.5);g.restore();
 }
 function fit(){var r=box.getBoundingClientRect();if(r.width<=0||r.height<=0)return;var d=Math.min(window.devicePixelRatio||1,2);cv.width=Math.round(r.width*d);cv.height=Math.round(r.height*d);g.setTransform(d,0,0,d,0,0);calc();draw();}
 function sync(announce){nEl.textContent='n = '+st.n.toFixed(1)+' \u00B0C';resetEl.textContent='Alarm reset = '+resetT().toFixed(1)+' \u00B0C';cv.setAttribute('aria-valuenow',st.n.toFixed(1));cv.setAttribute('aria-valuemax',maxN().toFixed(1));cv.setAttribute('aria-valuetext','Hysteresis '+st.n.toFixed(1)+' degrees Celsius; alarm reset '+resetT().toFixed(1)+' degrees Celsius');if(announce)cv.setAttribute('aria-label','High-temperature alarm hysteresis n, '+st.n.toFixed(1)+' degrees Celsius');draw();}
 function setN(v,announce){var nv=Math.round(hc(v,0,maxN())*10)/10;if(nv===st.n)return;st.n=nv;sync(announce);}
 function pos(e){var r=cv.getBoundingClientRect();return{x:e.clientX-r.left,y:e.clientY-r.top};}
 function near(e){if(!plot)return false;var q=pos(e),x=tx(resetT()),y=(plot.on+plot.off)/2;return Math.hypot(q.x-x,q.y-y)<=30||(Math.abs(q.x-x)<=17&&q.y>=plot.on-12&&q.y<=plot.off+12);}
 function fromPointer(e){setN(HT-xt(pos(e).x),false);}
 cv.addEventListener('pointerdown',function(e){if(!near(e))return;st.drag=true;box.classList.add('drag');cv.setPointerCapture(e.pointerId);fromPointer(e);draw();});
 cv.addEventListener('pointermove',function(e){if(st.drag)fromPointer(e);});
 function finish(e){if(!st.drag)return;st.drag=false;box.classList.remove('drag');if(cv.hasPointerCapture(e.pointerId))cv.releasePointerCapture(e.pointerId);sync(true);}
 cv.addEventListener('pointerup',finish);cv.addEventListener('pointercancel',finish);
 cv.addEventListener('keydown',function(e){var step=e.shiftKey?1:.1,next=st.n;if(e.key==='ArrowLeft'||e.key==='ArrowUp')next+=step;else if(e.key==='ArrowRight'||e.key==='ArrowDown')next-=step;else if(e.key==='Home')next=0;else if(e.key==='End')next=maxN();else return;e.preventDefault();setN(next,true);});
 window.addEventListener('resize',function(){if(mod.classList.contains('open'))fit();});
 window.openHyst=function(){LT=+LOW;HT=+HIGH;st.saved=hc(+HYST,0,maxN());st.n=st.saved;mod.classList.add('open');requestAnimationFrame(function(){sync(false);fit();cv.focus();});};
 window.hystClose=function(){st.n=st.saved;mod.classList.remove('open');document.querySelector('.hyst-open').focus();};
 window.hystSet=function(){setBtn.disabled=true;var b=new URLSearchParams();b.append('n',st.n.toFixed(1));fetch('/api/hysteresis',{method:'POST',body:b}).then(function(r){if(!r.ok)return r.text().then(function(t){throw t});return r.json();}).then(function(j){HYST=parseFloat(j.n);st.saved=HYST;st.n=HYST;mod.classList.remove('open');document.querySelector('.hyst-open').focus();}).catch(function(e){alert((''+e)||'Could not save hysteresis.');}).finally(function(){setBtn.disabled=false;});};
})();
document.querySelectorAll('[role="button"]').forEach(function(el){
 el.addEventListener('keydown',function(e){
  if(e.key!=='Enter'&&e.key!==' ')return;
  e.preventDefault();el.click();
 });
});
function thrMsg(txt,cvar){var m=document.getElementById('thrMsg');
 m.textContent=txt;m.style.color=css(cvar).trim();
 m.style.transition='none';m.style.opacity='1';void m.offsetWidth;   /*reflow: reset instantly*/
 m.style.transition='opacity 1s ease';
 clearTimeout(m._h);m._h=setTimeout(function(){m.style.opacity='0';},4000);}   /*fade out, gone ~5s*/
function setThr(){var lo=document.getElementById('inLow').value,hi=document.getElementById('inHigh').value;
 var b=new URLSearchParams();b.append('lowTemp',lo);b.append('highTemp',hi);
 fetch('/api/thresholds',{method:'POST',body:b}).then(function(r){
 if(!r.ok)return r.text().then(function(t){throw t});return r.json();}).then(function(j){
  LOW=parseFloat(j.lowT);HIGH=parseFloat(j.highT);if(j.n!=null)HYST=parseFloat(j.n);
  thrMsg('Saved ✓ ('+LOW+'–'+HIGH+' °C)','--ok');
 }).catch(function(e){thrMsg((''+e)||'Error','--bad');});}
var DASH_CACHE_KEY='fanDashboardStateV1',DASH_CACHE_MAX_AGE=60000;
function renderStatus(d,appendHistory){
 if(!d||!('temp' in d))return false;   /*partial snapshot: preserve the last complete view*/
 var t=d.temp,tok=(t!=null&&!isNaN(t));
 if(tok){
  document.getElementById('mTemp').innerHTML=t.toFixed(2)+'<small>&deg;C</small>';
  var c=col(t).trim();document.getElementById('mTemp').style.color=c;
  var bt=document.getElementById('bTemp');bt.style.width=clamp((t-LOW)/(HIGH-LOW)*100)+'%';bt.style.background=c;
  document.getElementById('mTempSub').textContent=t>=HIGH?'Above high limit':(t<=LOW?'Below low limit':'');
  if(appendHistory){hist.push(t);if(hist.length>60)hist.shift();}
  if(document.getElementById('pane-tele').classList.contains('active'))drawSpark();
 }else{
  document.getElementById('mTemp').innerHTML='--<small>&deg;C</small>';
  document.getElementById('mTemp').style.color=css('--bad').trim();
  document.getElementById('bTemp').style.width='0%';
  document.getElementById('mTempSub').textContent='No sensor reading';
 }
 var sensorFail=(!tok||d.tempCode=='Sensor Failure'||d.tempCode=='Disconnected');
 document.getElementById('mTS').innerHTML=sensorFail?'<span class="badge bad">&#9888; FAILURE</span>':'<span class="badge ok">&#10003; OK</span>';
 var ta;if(sensorFail)ta='<span class="badge n">&mdash;</span>';
 else if(d.tempAlarm)ta='<span class="badge bad">&#9888; HIGH</span>';
 else if(tok&&d.tempCode&&d.tempCode.indexOf('Low')>=0)ta='<span class="badge info">LOW</span>';
 else ta='<span class="badge ok">&#10003; OK</span>';
 document.getElementById('mTA').innerHTML=ta;
 var p1=d.pwm1,p2=d.pwm2;
 [['gF1','gF1t',p1],['gF2','gF2t',p2]].forEach(function(a){var r=document.getElementById(a[0]),t=document.getElementById(a[1]);if(!r||!t)return;var v=Math.max(0,Math.min(100,a[2]||0));r.style.strokeDasharray=(v/100*314.16).toFixed(1)+' 314.16';r.style.stroke=(v>=80?css('--bad'):v>=50?css('--warn'):css('--ok')).trim();t.textContent=v.toFixed(2);});
 document.getElementById('mR1').textContent=(d.fan1Rpm||0).toFixed(0)+' rpm';
 document.getElementById('mR2').textContent=(d.fan2Rpm||0).toFixed(0)+' rpm';
 document.getElementById('aF1').innerHTML=badge(d.fan1Alarm);
 document.getElementById('aF2').innerHTML=badge(d.fan2Alarm);
 if(d.fanLogic!=null)fanLogic=d.fanLogic;
 document.getElementById('aFG').className='bulb'+(d.fanGeneral?' on':'');
 if(d.relayMap&&!document.getElementById('outModal').classList.contains('open')){
  var ch=false;for(var i=0;i<4;i++){var nm=NUMSIG[d.relayMap[i]]||'';if(outMap[i]!==nm){outMap[i]=nm;ch=true;}}
  if(ch)outRender();}
 document.getElementById('mDoor').innerHTML=d.door?'<span class="badge bad">&#9888; OPEN</span>':'<span class="badge ok">&#10003; CLOSED</span>';
 var rs=d.rssi;document.getElementById('mRssi').innerHTML=rs+'<small>dBm</small>';
 document.getElementById('mRssiSub').textContent=rs>=-60?'Excellent':(rs>=-70?'Good':(rs>=-80?'Fair':'Weak'));
 document.getElementById('mUp').textContent=d.uptime;
 document.getElementById('mVer').textContent='v'+d.version;
 return true;
}
function saveDashboardSnapshot(d){try{
 sessionStorage.setItem(DASH_CACHE_KEY,JSON.stringify({ts:Date.now(),d:d,h:hist.slice(-60)}));
}catch(e){}}
function restoreDashboardSnapshot(){try{
 var raw=sessionStorage.getItem(DASH_CACHE_KEY);if(!raw)return;
 var snap=JSON.parse(raw),age=Date.now()-snap.ts;
 if(!snap.d||typeof snap.ts!=='number'||age<0||age>DASH_CACHE_MAX_AGE){sessionStorage.removeItem(DASH_CACHE_KEY);return;}
 hist=Array.isArray(snap.h)?snap.h.filter(function(v){return typeof v==='number'&&isFinite(v)}).slice(-60):[];
 renderStatus(snap.d,false);
}catch(e){}}
var _pollBusy=false;
async function poll(){
 if(_pollBusy)return;   /*single flight: skip the tick while one request is in progress (no pile-up, no out-of-order flips)*/
 _pollBusy=true;
 var ctl=new AbortController(),tmr=setTimeout(function(){ctl.abort();},2500);   /*bound each attempt: a dead board fails in 2.5 s, not at the TCP timeout*/
 try{
  var r=await fetch('/api/status',{cache:'no-store',signal:ctl.signal});if(!r.ok)throw 0;
  var d=await r.json();fail=0;setLive(true);
  if(renderStatus(d,true))saveDashboardSnapshot(d);
 }catch(e){if(++fail>=2)setLive(false)}   /*2-strike filter kept: with bounded attempts, Offline shows in ~5-7 s deterministically*/
 finally{clearTimeout(tmr);_pollBusy=false;}
}
restoreDashboardSnapshot();document.body.style.visibility='visible';poll();setInterval(poll,2000);window.addEventListener('resize',drawSpark);
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
        /*Reserve up front: the replacements below must not realloc the ~55 KB
        string mid-build (that momentarily doubles the heap and fails once
        MQTT/TLS have fragmented it). Reserve first, then fill the buffer.*/
        String page;
        page.reserve(sizeof(PORTAL_HTML) + 4096);   /*page size + margin for the %-token expansions; sizeof auto-tracks growth so it never goes stale*/
        page = FPSTR(PORTAL_HTML);
        page.replace("%WIFISSID%",   _store->cfg.wifiSsid);
        page.replace("%MQTTSERVER%", _store->cfg.mqttServer);
        page.replace("%MQTTPORT%",   String(_store->cfg.mqttPort));
        page.replace("%MQTTUSER%",   _store->cfg.mqttUser);
        page.replace("%HIGHTEMP%",   String(_store->cfg.highTemp, 1));
        page.replace("%LOWTEMP%",    String(_store->cfg.lowTemp, 1));
        page.replace("%TEMPHYST%",   String(_store->cfg.tempHysteresis, 1));
        page.replace("%PWMN%",       String(_store->cfg.pwmN, 2));
        page.replace("%PWMP%",       String(_store->cfg.pwmP, 2));
        page.replace("%CACERTSTATE%", _store->cfg.caCert[0] ? "Custom" : "Factory (Let&#39;s Encrypt)");
        page.replace("%MQTTCITY%",   _store->cfg.mqttCity);
        page.replace("%MQTTSITE%",   _store->cfg.mqttSite);
        /*Device-identity tags shown in the dashboard header so the operator
        knows which board of the fleet they are looking at (static config,
        no /api/status change needed).*/
        page.replace("%MQTTOPER%",   _store->cfg.mqttOperator);
        page.replace("%MQTTSUBSYS%", _store->cfg.mqttSubsystem);
        /*Client ID = site-MAC, the same value the main builds for the MQTT
        client; the MAC makes it the unique device identity.*/
        /*Factory MAC from eFuse (same source as the main), so the portal's
        Client ID / topics match exactly what the device publishes.*/
        uint8_t _mr[6]; esp_read_mac(_mr, ESP_MAC_WIFI_STA);
        char _mn[13], _mc[18];
        snprintf(_mn, sizeof(_mn), "%02X%02X%02X%02X%02X%02X",
                 _mr[0],_mr[1],_mr[2],_mr[3],_mr[4],_mr[5]);
        snprintf(_mc, sizeof(_mc), "%02X:%02X:%02X:%02X:%02X:%02X",
                 _mr[0],_mr[1],_mr[2],_mr[3],_mr[4],_mr[5]);
        String _siteId = String(_store->cfg.mqttSite) + "-" + String(_mn);
        /*Client ID = site-MAC (site keeps its typed case, MAC uppercase); no
        subsystem here because it already lives in the topic path.*/
        page.replace("%CLIENTID%", _siteId);
        /*Raw MAC (colon form) shown read-only in the Device tab.*/
        page.replace("%MAC%", String(_mc));
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
            /*Stream the page chunk-by-chunk from a shared String instead of
            req->send(String), which copies the whole ~42 KB a second time.
            This keeps the peak heap at ~1x the page (not ~2x), so a reload on
            a fragmented heap stops intermittently returning a blank page.*/
            auto page = std::make_shared<String>(_renderPortal());
            AsyncWebServerResponse* res = req->beginChunkedResponse("text/html",
                [page](uint8_t* buffer, size_t maxLen, size_t index) -> size_t {
                    size_t len = page->length();
                    if (index >= len) return 0;
                    size_t n = (len - index < maxLen) ? (len - index) : maxLen;
                    memcpy(buffer, page->c_str() + index, n);
                    return n;
                });
            req->send(res);
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
            float maxHysteresis =
                (float)((int)(((c.highTemp - c.lowTemp) * 2.0f) + 0.0001f)) / 10.0f;
            if (!(c.tempHysteresis >= 0.0f)) c.tempHysteresis = 0.0f;
            if (c.tempHysteresis > maxHysteresis) c.tempHysteresis = maxHysteresis;
            if (!_store->save()){
                req->send(500, "text/plain", "Save failed");
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
                "</head><body><h2>Saved &#10003;</h2>"
                "<p id=\"m\">Rebooting the device...</p><script>"
                "var n=10,e=document.getElementById('m');var t=setInterval(function(){"
                "n--;e.textContent='Rebooting... back in '+n+'s';"
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
            float maxHysteresis =
                (float)((int)(((hi - lo) * 2.0f) + 0.0001f)) / 10.0f;
            if (!(c.tempHysteresis >= 0.0f)) c.tempHysteresis = 0.0f;
            if (c.tempHysteresis > maxHysteresis) c.tempHysteresis = maxHysteresis;
            if (!_store->save()){
                req->send(500, "text/plain", "Save failed");
                return;
            }
            req->send(200, "application/json",
                      "{\"ok\":true,\"lowT\":" + String(lo, 1) +
                      ",\"highT\":" + String(hi, 1) +
                      ",\"n\":" + String(c.tempHysteresis, 1) + "}");
        });

        /*High-temperature alarm hysteresis: n is the reset band in deg C.
        It is constrained to 20% of HIGH_T - LOW_T and applied live.*/
        _server.on("/api/hysteresis", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            if (!req->hasParam("n", true)){
                req->send(400, "text/plain", "Missing n");
                return;
            }
            AppConfig& c = _store->cfg;
            float n = req->getParam("n", true)->value().toFloat();
            float maxN =
                (float)((int)(((c.highTemp - c.lowTemp) * 2.0f) + 0.0001f)) / 10.0f;
            if (!(n >= 0.0f && n <= maxN)){
                req->send(400, "text/plain", "n must be between 0 and 20% of HIGH_T - LOW_T");
                return;
            }
            c.tempHysteresis = n;
            if (!_store->save()){
                req->send(500, "text/plain", "Save failed");
                return;
            }
            req->send(200, "application/json",
                      "{\"ok\":true,\"n\":" + String(n, 1) +
                      ",\"resetT\":" + String(c.highTemp - n, 1) + "}");
        });

        /*PWM curve (Fan PWM Curve popup): n=floor(0..1), p=exponent(1..10),
        shared by both fans. Applied live (control loop reads cfg) -- no reboot.*/
        /*Broker CA certificate: paste-a-PEM stored in NVS; action=reset
        returns to the compiled-in factory CA. Applied on the next MQTT
        (re)connect -- typically the Save-and-Restart of a broker change.*/
        _server.on("/api/cacert", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            AppConfig& c = _store->cfg;
            if (req->hasParam("action", true) &&
                req->getParam("action", true)->value() == "reset"){
                c.caCert[0] = '\0';
                if (!_store->save()){ req->send(500, "text/plain", "Save failed"); return; }
                req->send(200, "application/json", "{\"ok\":true,\"custom\":false}");
                return;
            }
            if (!req->hasParam("pem", true)){
                req->send(400, "text/plain", "Missing pem");
                return;
            }
            String pem = req->getParam("pem", true)->value();
            pem.trim();
            if (pem.indexOf("-----BEGIN CERTIFICATE-----") != 0 ||
                pem.indexOf("-----END CERTIFICATE-----") < 0 ||
                pem.length() + 2 > sizeof(c.caCert)){
                req->send(400, "text/plain", "Not a PEM certificate (or too large)");
                return;
            }
            pem += "\n";   /*mbedTLS wants the PEM newline-terminated*/
            strlcpy(c.caCert, pem.c_str(), sizeof(c.caCert));
            if (!_store->save()){ req->send(500, "text/plain", "Save failed"); return; }
            req->send(200, "application/json", "{\"ok\":true,\"custom\":true}");
        });

        _server.on("/api/pwmcurve", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            AppConfig& c = _store->cfg;
            float n = c.pwmN, p = c.pwmP;
            if (req->hasParam("pwmN", true)) n = req->getParam("pwmN", true)->value().toFloat();
            if (req->hasParam("pwmP", true)) p = req->getParam("pwmP", true)->value().toFloat();
            if (!(n >= 0.0f && n <= 1.0f) || !(p >= 1.0f && p <= 10.0f)){
                req->send(400, "text/plain", "n (0-1) o p (1-10) fuera de rango");
                return;
            }
            c.pwmN = n; c.pwmP = p;
            if (!_store->save()){
                req->send(500, "text/plain", "Save failed");
                return;
            }
            req->send(200, "application/json",
                      "{\"ok\":true,\"pwmN\":" + String(n, 3) +
                      ",\"pwmP\":" + String(p, 3) + "}");
        });

        /*Fan-alarm logic (Fan Alarm Setup popup): 0=OR 1=AND 2=FAN1 3=FAN2.
        Applied live (the control side reads cfg each cycle) -- no reboot.*/
        _server.on("/api/fanlogic", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            if (!req->hasParam("logic", true)){
                req->send(400, "text/plain", "falta 'logic'");
                return;
            }
            int v = req->getParam("logic", true)->value().toInt();
            if (v < 0 || v > 3){
                req->send(400, "text/plain", "logic invalido (0-3)");
                return;
            }
            _store->cfg.fanAlarmLogic = (uint8_t)v;
            if (!_store->save()){
                req->send(500, "text/plain", "Save failed");
                return;
            }
            req->send(200, "application/json", "{\"ok\":true,\"logic\":" + String(v) + "}");
        });

        /*Output-relay mapping (Control tab): OUT1-4 -> signal (0=free 1=temp
        2=door 3=fan). Applied live by the control router -- no reboot.*/
        _server.on("/api/relays", HTTP_POST, [this](AsyncWebServerRequest* req){
            if (!_auth(req)) return;
            uint8_t nm[4];
            for (int i = 0; i < 4; i++){
                String k = "r" + String(i);
                int v = req->hasParam(k, true) ? req->getParam(k, true)->value().toInt() : 0;
                if (v < 0 || v > 3){ req->send(400, "text/plain", "valor invalido"); return; }
                nm[i] = (uint8_t)v;
            }
            /*Mutual exclusion: a non-free signal can only drive one relay.*/
            for (int i = 0; i < 4; i++)
                for (int j = i + 1; j < 4; j++)
                    if (nm[i] != 0 && nm[i] == nm[j]){
                        req->send(400, "text/plain", "mutual exclusion: one alarm on two relays");
                        return;
                    }
            for (int i = 0; i < 4; i++) _store->cfg.relayMap[i] = nm[i];
            if (!_store->save()){ req->send(500, "text/plain", "Save failed"); return; }
            req->send(200, "application/json", "{\"ok\":true}");
        });

        /*OTA: upload handler streams the .bin into the OTA partition.*/
        _server.on("/update", HTTP_POST,
            [this](AsyncWebServerRequest* req){
                if (!_auth(req)) return;
                bool ok = !Update.hasError();
                req->send(ok ? 200 : 500, "text/plain",
                          ok ? "OTA OK. Rebooting..." : "OTA FAILED");
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
            req->send(200, "text/plain", "Rebooting...");
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
