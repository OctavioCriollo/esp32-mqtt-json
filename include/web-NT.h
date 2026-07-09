/*web-NT.h — Async web portal: configuration, live status, OTA (item H).
==========================================================================
Endpoints (all except / and /api/status require HTTP Basic auth,
user "admin", password = configStore.cfg.webPass):

  GET  /            single-page portal (PROGMEM, no filesystem needed)
  GET  /api/status  live JSON snapshot (temp, PWM, alarms, uptime, rssi)
  POST /api/config  save configuration to NVS, then reboot
  POST /update      OTA firmware upload (.bin from CI artifacts)
  POST /reboot      manual restart

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
.grid.full{grid-template-columns:1fr}
.metric{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px 16px}
.metric .lab{font-size:.72rem;color:var(--mut);text-transform:uppercase;letter-spacing:.6px;margin-bottom:6px}
.metric .val{font-size:1.7rem;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.1}
.metric .val small{font-size:.85rem;font-weight:600;color:var(--mut);margin-left:3px}
.metric .sub{font-size:.74rem;color:var(--mut);margin-top:2px}
.span2{grid-column:span 2}
.bar{height:8px;border-radius:999px;background:var(--bg2);overflow:hidden;margin-top:10px;border:1px solid var(--line)}
.bar>i{display:block;height:100%;width:0;border-radius:999px;transition:width .5s ease,background .5s ease}
.duo{display:flex;gap:16px}
.duo>div{flex:1}
.badge{display:inline-flex;align-items:center;gap:6px;font-size:.82rem;font-weight:700;
padding:5px 12px;border-radius:999px}
.badge.ok{color:var(--ok);background:rgba(52,211,153,.12);border:1px solid rgba(52,211,153,.3)}
.badge.bad{color:var(--bad);background:rgba(248,113,113,.12);border:1px solid rgba(248,113,113,.3)}
.badge.n{color:var(--txt);background:rgba(136,150,172,.12);border:1px solid var(--line)}
.spark{width:100%;height:56px;display:block;margin-top:8px}
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
input[type=file]{padding:8px;color:var(--mut)}
button{margin-top:16px;padding:11px 18px;border:0;border-radius:10px;font-size:.9rem;
background:linear-gradient(145deg,var(--accent),var(--accent2));color:#fff;font-weight:700;
cursor:pointer;width:100%;transition:filter .15s,transform .05s}
button:hover{filter:brightness(1.08)}button:active{transform:translateY(1px)}
small{color:var(--mut);font-size:.74rem}
.stale{opacity:.5;filter:saturate(.4)}
@media(max-width:520px){.grid{grid-template-columns:1fr}.span2{grid-column:auto}.fgrid{grid-template-columns:1fr}
.metric .val{font-size:1.45rem}
header{justify-content:center;text-align:center}.brand{justify-content:center}.conn{margin:0 auto}.tags{justify-content:center}}
</style></head><body><div class="wrap">
<header>
<div class="brand">
<div class="logo"><svg viewBox="0 0 24 24"><path d="M12 12a2 2 0 100-4 2 2 0 000 4zm0-1.2a.8.8 0 110-1.6.8.8 0 010 1.6zM12 2c1.9 0 3 1.6 2.7 3.9 1.9-.6 3.6.2 4 2 .4 1.7-.7 3.2-2.9 3.7 1.6 1.2 1.8 3 .8 4.4-1 1.4-2.9 1.4-4.6 0 .3 2.3-.8 3.9-2 3.9s-2.3-1.6-2-3.9c-1.7 1.4-3.6 1.4-4.6 0-1-1.4-.8-3.2.8-4.4C2 11.1.9 9.6 1.3 7.9c.4-1.8 2.1-2.6 4-2C4.9 3.6 6 2 8 2c1.3 0 2.2 1.2 2.4 2.9C10.7 3.2 11.3 2 12 2z"/></svg></div>
<div><h1>FAN Controller</h1><p>Power &middot; Climatizaci&oacute;n</p></div>
</div>
<div class="conn"><span class="dot" id="dot"></span><span id="connTxt">Conectando&hellip;</span></div>
</header>
<div class="tags">
<span class="tag"><b>Op</b> %MQTTOPER%</span>
<span class="tag"><b>Ciudad</b> %MQTTCITY%</span>
<span class="tag"><b>Sitio</b> %MQTTSITE%</span>
<span class="tag"><b>Subsist</b> %MQTTSUBSYS%</span>
<span class="tag"><b>Client ID</b> %CLIENTID%</span>
</div>

<div id="st">
<div class="grid">
<div class="metric span2">
<div class="lab">Temperatura del gabinete</div>
<div class="val" id="mTemp">--<small>&deg;C</small></div>
<div class="sub" id="mTempSub">Rango %LOWTEMP%&ndash;%HIGHTEMP% &deg;C</div>
<div class="bar"><i id="bTemp"></i></div>
<canvas class="spark" id="spark"></canvas>
</div>
<div class="metric span2">
<div class="lab">Velocidad de ventiladores (PWM)</div>
<div class="duo">
<div><div class="sub">FAN 1</div><div class="val" id="mF1">--<small>%</small></div><div class="bar"><i id="bF1"></i></div></div>
<div><div class="sub">FAN 2</div><div class="val" id="mF2">--<small>%</small></div><div class="bar"><i id="bF2"></i></div></div>
</div>
</div>
<div class="metric"><div class="lab">Puerta</div><div id="mDoor"><span class="badge n">&mdash;</span></div></div>
<div class="metric"><div class="lab">Sensor de temperatura</div><div id="mTA"><span class="badge n">&mdash;</span></div></div>
<div class="metric"><div class="lab">Ventiladores / Alarma</div><div id="mFA"><span class="badge n">&mdash;</span></div></div>
<div class="metric"><div class="lab">Se&ntilde;al WiFi</div><div class="val" id="mRssi">--<small>dBm</small></div><div class="sub" id="mRssiSub">&nbsp;</div></div>
<div class="metric"><div class="lab">Tiempo activo</div><div class="val" id="mUp" style="font-size:1.25rem">--</div></div>
<div class="metric"><div class="lab">Firmware</div><div class="val" id="mVer" style="font-size:1.25rem">--</div></div>
</div>
</div>

<section><h2><svg viewBox="0 0 24 24"><path d="M12 8a4 4 0 100 8 4 4 0 000-8zm8.9 5-.7 1.3 1.3 1.6-1.9 1.9-1.6-1.3-1.3.7-.3 2h-2.7l-.3-2-1.3-.7-1.6 1.3-1.9-1.9 1.3-1.6-.7-1.3-2-.3v-2.7l2-.3.7-1.3-1.3-1.6 1.9-1.9 1.6 1.3 1.3-.7.3-2h2.7l.3 2 1.3.7 1.6-1.3 1.9 1.9-1.3 1.6.7 1.3 2 .3v2.7l-2 .3z"/></svg>Configuraci&oacute;n</h2>
<p class="hint">Al guardar, el dispositivo se reinicia para aplicar los cambios.</p>
<form method="POST" action="/api/config"><div class="fgrid">
<div><label>WiFi SSID</label><input name="wifiSsid" value="%WIFISSID%"></div>
<div><label>WiFi Password</label><input name="wifiPass" type="password" placeholder="(sin cambio)"></div>
<div><label>MQTT Server</label><input name="mqttServer" value="%MQTTSERVER%"></div>
<div><label>MQTT Port</label><input name="mqttPort" type="number" value="%MQTTPORT%"></div>
<div><label>MQTT User</label><input name="mqttUser" value="%MQTTUSER%"></div>
<div><label>MQTT Password</label><input name="mqttPass" type="password" placeholder="(sin cambio)"></div>
<div><label>Operador</label><select name="mqttOper">%OPEROPTS%</select></div>
<div><label>Ciudad</label><input name="mqttCity" value="%MQTTCITY%"></div>
<div class="f-full"><label>Sitio / RBS (la MAC se a&ntilde;ade sola)</label><input name="mqttSite" value="%MQTTSITE%"></div>
<div><label>Subsistema</label><select name="mqttSubsys">%SUBSYSOPTS%</select></div>
<div><label>Zona horaria</label><select name="tzOffset">%TZOPTS%</select></div>
<div><label>Temp alta &deg;C (PWM 100%)</label><input name="highTemp" type="number" step="0.5" value="%HIGHTEMP%"></div>
<div><label>Temp baja &deg;C (PWM 0%)</label><input name="lowTemp" type="number" step="0.5" value="%LOWTEMP%"></div>
<div class="f-full"><label>Password del portal</label><input name="webPass" type="password" placeholder="(sin cambio)"></div>
</div>
<button type="submit">Guardar y reiniciar</button>
<br><small>Usuario del portal: <b>admin</b></small></form></section>

<section><h2><svg viewBox="0 0 24 24"><path d="M12 2 4 6v6c0 5 3.4 8.7 8 10 4.6-1.3 8-5 8-10V6l-8-4zm-1 13-3-3 1.4-1.4L11 12.2l4.6-4.6L17 9l-6 6z"/></svg>Firmware OTA</h2>
<p class="hint">Sube el <b>firmware.bin</b> generado por el CI. El equipo se reinicia al terminar.</p>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Actualizar firmware</button></form></section>
</div>
<script>
var LOW=%LOWTEMP%,HIGH=%HIGHTEMP%,hist=[],fail=0;
function col(t){if(t>=HIGH)return getComputedStyle(document.documentElement).getPropertyValue('--bad');
if(t>=HIGH-3)return getComputedStyle(document.documentElement).getPropertyValue('--warn');
if(t<=LOW)return getComputedStyle(document.documentElement).getPropertyValue('--accent');
return getComputedStyle(document.documentElement).getPropertyValue('--ok')}
function clamp(v){return v<0?0:v>100?100:v}
function badge(alm){return alm?'<span class="badge bad">&#9888; ALARMA</span>':'<span class="badge ok">&#10003; OK</span>'}
function drawSpark(){var c=document.getElementById('spark');if(!c)return;
var w=c.clientWidth,h=56;c.width=w;c.height=h;var x=c.getContext('2d');x.clearRect(0,0,w,h);
if(hist.length<2)return;var lo=Math.min.apply(0,hist),hi=Math.max.apply(0,hist);
if(hi-lo<1){hi=lo+1}var pad=6,gw=w,gh=h-pad*2;
x.beginPath();for(var i=0;i<hist.length;i++){var px=i/(hist.length-1)*gw,
py=pad+gh-(hist[i]-lo)/(hi-lo)*gh;i?x.lineTo(px,py):x.moveTo(px,py)}
x.strokeStyle=col(hist[hist.length-1]).trim();x.lineWidth=2;x.lineJoin='round';x.stroke();
x.lineTo(gw,h);x.lineTo(0,h);x.closePath();
var g=x.createLinearGradient(0,0,0,h);g.addColorStop(0,'rgba(56,189,248,.22)');
g.addColorStop(1,'rgba(56,189,248,0)');x.fillStyle=g;x.fill()}
function setLive(ok){var d=document.getElementById('dot'),t=document.getElementById('connTxt'),
s=document.getElementById('st');d.className='dot '+(ok?'live':'dead');
t.textContent=ok?'En vivo &middot; act. 2 s':'Sin conexi\u00f3n';
t.innerHTML=ok?'En vivo &middot; actualizado ahora':'Sin conexi\u00f3n \u2014 dato antiguo';
s.className=ok?'':'stale'}
async function poll(){try{
 var r=await fetch('/api/status');var d=await r.json();fail=0;setLive(true);
 var t=d.temp,tok=(t!=null&&!isNaN(t));
 if(tok){
  document.getElementById('mTemp').innerHTML=t.toFixed(1)+'<small>&deg;C</small>';
  var c=col(t).trim();document.getElementById('mTemp').style.color=c;
  var bt=document.getElementById('bTemp');bt.style.width=clamp((t-LOW)/(HIGH-LOW)*100)+'%';bt.style.background=c;
  document.getElementById('mTempSub').textContent=t>=HIGH?'Sobre l\u00edmite alto':(t<=LOW?'Bajo l\u00edmite':'En rango '+LOW+'\u2013'+HIGH+' \u00b0C');
  hist.push(t);if(hist.length>60)hist.shift();drawSpark();
 }else{
  document.getElementById('mTemp').innerHTML='--<small>&deg;C</small>';
  document.getElementById('mTemp').style.color=getComputedStyle(document.documentElement).getPropertyValue('--bad').trim();
  document.getElementById('bTemp').style.width='0%';
  document.getElementById('mTempSub').textContent='Sensor sin lectura';
 }
 var p1=d.pwm1,p2=d.pwm2;
 document.getElementById('mF1').innerHTML=p1.toFixed(0)+'<small>%</small>';
 document.getElementById('mF2').innerHTML=p2.toFixed(0)+'<small>%</small>';
 var b1=document.getElementById('bF1'),b2=document.getElementById('bF2');
 b1.style.width=clamp(p1)+'%';b2.style.width=clamp(p2)+'%';
 b1.style.background=b2.style.background=getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
 document.getElementById('mDoor').innerHTML=d.door?'<span class="badge bad">Abierta</span>':'<span class="badge ok">Cerrada</span>';
 var sensorFail=(!tok||d.tempCode=='Sensor Failure'||d.tempCode=='Disconnected');
 document.getElementById('mTA').innerHTML=sensorFail?'<span class="badge bad">&#9888; FALLA</span>':'<span class="badge ok">&#10003; OK</span>';
 document.getElementById('mFA').innerHTML=badge(d.fanAlarm);
 var rs=d.rssi;document.getElementById('mRssi').innerHTML=rs+'<small>dBm</small>';
 document.getElementById('mRssiSub').textContent=rs>=-60?'Excelente':(rs>=-70?'Buena':(rs>=-80?'Regular':'D\u00e9bil'));
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
    static String _selectOpts(const char* const* values, int n, const char* current){
        String s;
        for (int i = 0; i < n; i++){
            s += "<option";
            if (strcmp(values[i], current) == 0) s += " selected";
            s += ">"; s += values[i]; s += "</option>";
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
        String _mac = WiFi.macAddress(); _mac.replace(":", "");
        page.replace("%CLIENTID%", String(_store->cfg.mqttSite) + "-" + _mac +
                                   "-" + _store->cfg.mqttSubsystem);
        static const char* const opers[]  = {"claro","cnt","tigo"};
        static const char* const subsys[] = {"power","generador","baterias","seguridad"};
        page.replace("%OPEROPTS%",   _selectOpts(opers, 3, _store->cfg.mqttOperator));
        page.replace("%SUBSYSOPTS%", _selectOpts(subsys, 4, _store->cfg.mqttSubsystem));
        /*Timezone: pick by representative city; option value = UTC offset (h).
        No DST handling (fine for Ecuador); offsets are standard time.*/
        struct Tz { float off; const char* label; };
        static const Tz tzs[] = {
            {-6, "Mexico (UTC-6)"},
            {-5, "Guayaquil / Quito / Bogota / Lima (UTC-5)"},
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
                "var n=15,e=document.getElementById('m');var t=setInterval(function(){"
                "n--;e.textContent='Reiniciando... volviendo en '+n+'s';"
                "if(n<=0){clearInterval(t);location.href='/';}},1000);</script></body></html>");
            _rebootPending = true; _rebootAt = millis() + 2000;
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
