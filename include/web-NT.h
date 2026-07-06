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
body{font-family:system-ui,sans-serif;background:#111827;color:#e5e7eb;margin:0;padding:16px}
h1{font-size:1.2rem;color:#93c5fd}h2{font-size:1rem;color:#93c5fd;margin-top:24px}
.card{background:#1f2937;border-radius:10px;padding:14px;margin:10px 0;max-width:560px}
.row{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #374151}
.row span:last-child{color:#fbbf24}
label{display:block;margin:8px 0 2px;font-size:.85rem;color:#9ca3af}
input,select{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:1px solid #374151;background:#111827;color:#e5e7eb}
button{margin-top:12px;padding:10px 16px;border:0;border-radius:6px;background:#2563eb;color:#fff;font-weight:600;cursor:pointer}
.alm{color:#f87171!important;font-weight:700}.ok{color:#34d399!important}
small{color:#6b7280}
.sub{color:#9ca3af;font-size:.8rem;margin:2px 0 10px;word-break:break-all}
.big{font-size:2.6rem;font-weight:700;line-height:1}
.unit{font-size:1rem;color:#9ca3af}
.bar{height:10px;border-radius:5px;background:#374151;overflow:hidden;margin:6px 0}
.bar>i{display:block;height:100%;border-radius:5px;transition:width .4s,background .4s}
.pill{display:inline-block;padding:3px 10px;border-radius:999px;font-size:.75rem;font-weight:600;margin:4px 4px 0 0}
.pill.on{background:#064e3b;color:#34d399}.pill.off{background:#7f1d1d;color:#fca5a5}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.stale{opacity:.4}
.warn{background:#7f1d1d;color:#fecaca;padding:8px 12px;border-radius:8px;margin:8px 0;max-width:560px;display:none}
</style></head><body>
<h1>FAN Controller &mdash; Power/Climatizaci&oacute;n</h1>
<div class="sub" id="sub">&nbsp;</div>
<div class="warn" id="warn">Sin conexi&oacute;n con el dispositivo &mdash; mostrando el &uacute;ltimo dato</div>
<div class="card" id="st">Cargando estado...</div>
<div class="card"><h2>Configuraci&oacute;n</h2>
<form method="POST" action="/api/config">
<label>WiFi SSID</label><input name="wifiSsid" value="%WIFISSID%">
<label>WiFi Password</label><input name="wifiPass" type="password" placeholder="(sin cambio)">
<label>MQTT Server</label><input name="mqttServer" value="%MQTTSERVER%">
<label>MQTT Port</label><input name="mqttPort" type="number" value="%MQTTPORT%">
<label>MQTT User</label><input name="mqttUser" value="%MQTTUSER%">
<label>MQTT Password</label><input name="mqttPass" type="password" placeholder="(sin cambio)">
<label>Operador</label><select name="mqttOper">%OPEROPTS%</select>
<label>Ciudad</label><input name="mqttCity" value="%MQTTCITY%">
<label>Sitio / RBS (la MAC se a&ntilde;ade sola)</label><input name="mqttSite" value="%MQTTSITE%">
<label>Subsistema</label><select name="mqttSubsys">%SUBSYSOPTS%</select>
<label>Temp alta &deg;C (PWM 100%)</label><input name="highTemp" type="number" step="0.5" value="%HIGHTEMP%">
<label>Temp baja &deg;C (PWM 0%)</label><input name="lowTemp" type="number" step="0.5" value="%LOWTEMP%">
<label>Zona horaria</label><select name="tzOffset">%TZOPTS%</select>
<label>Password del portal</label><input name="webPass" type="password" placeholder="(sin cambio)">
<button type="submit">Guardar y reiniciar</button>
<br><small>Usuario del portal: admin</small></form></div>
<div class="card"><h2>Firmware OTA</h2>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Actualizar firmware</button>
<br><small>Sube el firmware.bin generado por el CI</small></form></div>
<script>
var hist=[];
function tcol(t,lo,hi){return t>=hi?'#f87171':(t>=hi-3?'#fbbf24':'#34d399');}
function spark(){
 if(hist.length<2)return'';
 var w=280,h=38,mn=Math.min.apply(null,hist),mx=Math.max.apply(null,hist);
 if(mx-mn<1)mx=mn+1;
 var p=hist.map(function(v,i){return(i*w/(hist.length-1)).toFixed(1)+','+(h-(v-mn)/(mx-mn)*h).toFixed(1);}).join(' ');
 return'<svg width="100%" height="38" viewBox="0 0 '+w+' '+h+'" preserveAspectRatio="none"><polyline fill="none" stroke="#60a5fa" stroke-width="2" points="'+p+'"/></svg>';
}
function pill(on,t){return'<span class="pill '+(on?'on':'off')+'">'+t+'</span>';}
async function poll(){
 var st=document.getElementById('st');
 try{
  const r=await fetch('/api/status');const d=await r.json();
  document.getElementById('warn').style.display='none';st.classList.remove('stale');
  if(d.node)document.getElementById('sub').textContent=d.node;
  var t=d.temp,tok=(t!=null&&!isNaN(t));
  if(tok){hist.push(t);if(hist.length>60)hist.shift();}
  var lo=(d.lowT!=null?d.lowT:24),hi=(d.highT!=null?d.highT:43);
  var pct=tok?Math.max(0,Math.min(100,(t-lo)/(hi-lo)*100)):0;
  var col=tok?tcol(t,lo,hi):'#9ca3af';
  st.innerHTML=
   '<div class="big" style="color:'+col+'">'+(tok?t.toFixed(1):'--')+'<span class="unit"> &deg;C</span></div>'+
   '<div class="bar"><i style="width:'+pct+'%;background:'+col+'"></i></div>'+
   '<small>rango '+lo.toFixed(0)+'&ndash;'+hi.toFixed(0)+' &deg;C</small>'+spark()+
   '<div class="grid" style="margin-top:12px">'+
    '<div><small>FAN1</small><div class="bar"><i style="width:'+d.pwm1+'%;background:#60a5fa"></i></div><b>'+d.pwm1.toFixed(0)+'%</b></div>'+
    '<div><small>FAN2</small><div class="bar"><i style="width:'+d.pwm2+'%;background:#60a5fa"></i></div><b>'+d.pwm2.toFixed(0)+'%</b></div>'+
   '</div>'+
   '<div style="margin-top:10px">'+
    pill(!d.tempAlarm,d.tempAlarm?'Sensor ALARMA':'Sensor OK')+
    pill(!d.fanAlarm,d.fanAlarm?'FANs ALARMA':'FANs OK')+
    pill(!d.door,d.door?'Puerta ABIERTA':'Puerta cerrada')+
    pill(d.mqtt,d.mqtt?'MQTT conectado':'MQTT sin conexion')+
   '</div>'+
   '<div class="row" style="margin-top:12px"><span>WiFi</span><span>'+d.rssi+' dBm</span></div>'+
   '<div class="row"><span>Uptime</span><span>'+d.uptime+'</span></div>'+
   '<div class="row"><span>Firmware</span><span>v'+d.version+'</span></div>';
 }catch(e){
  st.classList.add('stale');document.getElementById('warn').style.display='block';
 }
}
poll();setInterval(poll,2000);
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
