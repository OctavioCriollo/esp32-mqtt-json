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
input{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:1px solid #374151;background:#111827;color:#e5e7eb}
button{margin-top:12px;padding:10px 16px;border:0;border-radius:6px;background:#2563eb;color:#fff;font-weight:600;cursor:pointer}
.alm{color:#f87171!important;font-weight:700}.ok{color:#34d399!important}
small{color:#6b7280}</style></head><body>
<h1>FAN Controller &mdash; Power/Climatizaci&oacute;n</h1>
<div class="card" id="st">Cargando estado...</div>
<div class="card"><h2>Configuraci&oacute;n</h2>
<form method="POST" action="/api/config">
<label>WiFi SSID</label><input name="wifiSsid" value="%WIFISSID%">
<label>WiFi Password</label><input name="wifiPass" type="password" placeholder="(sin cambio)">
<label>MQTT Server</label><input name="mqttServer" value="%MQTTSERVER%">
<label>MQTT Port</label><input name="mqttPort" type="number" value="%MQTTPORT%">
<label>MQTT User</label><input name="mqttUser" value="%MQTTUSER%">
<label>MQTT Password</label><input name="mqttPass" type="password" placeholder="(sin cambio)">
<label>Temp alta &deg;C (PWM 100%)</label><input name="highTemp" type="number" step="0.5" value="%HIGHTEMP%">
<label>Temp baja &deg;C (PWM 0%)</label><input name="lowTemp" type="number" step="0.5" value="%LOWTEMP%">
<label>Password del portal</label><input name="webPass" type="password" placeholder="(sin cambio)">
<button type="submit">Guardar y reiniciar</button>
<br><small>Usuario del portal: admin</small></form></div>
<div class="card"><h2>Firmware OTA</h2>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Actualizar firmware</button>
<br><small>Sube el firmware.bin generado por el CI</small></form></div>
<script>
async function poll(){try{
 const r=await fetch('/api/status');const d=await r.json();
 const a=x=>x?'<span class="alm">ALARMA</span>':'<span class="ok">OK</span>';
 document.getElementById('st').innerHTML=
  '<h2>Estado</h2>'+
  '<div class="row"><span>Temperatura</span><span>'+d.temp.toFixed(1)+' &deg;C</span></div>'+
  '<div class="row"><span>FAN1 / FAN2 PWM</span><span>'+d.pwm1.toFixed(0)+'% / '+d.pwm2.toFixed(0)+'%</span></div>'+
  '<div class="row"><span>Puerta</span><span>'+(d.door?'ABIERTA':'cerrada')+'</span></div>'+
  '<div class="row"><span>Sensor</span><span>'+a(d.tempAlarm)+'</span></div>'+
  '<div class="row"><span>Ventiladores</span><span>'+a(d.fanAlarm)+'</span></div>'+
  '<div class="row"><span>WiFi RSSI</span><span>'+d.rssi+' dBm</span></div>'+
  '<div class="row"><span>Uptime</span><span>'+d.uptime+'</span></div>'+
  '<div class="row"><span>Firmware</span><span>'+d.version+'</span></div>';
}catch(e){}}
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

    String _renderPortal(){
        String page = FPSTR(PORTAL_HTML);
        page.replace("%WIFISSID%",   _store->cfg.wifiSsid);
        page.replace("%MQTTSERVER%", _store->cfg.mqttServer);
        page.replace("%MQTTPORT%",   String(_store->cfg.mqttPort));
        page.replace("%MQTTUSER%",   _store->cfg.mqttUser);
        page.replace("%HIGHTEMP%",   String(_store->cfg.highTemp, 1));
        page.replace("%LOWTEMP%",    String(_store->cfg.lowTemp, 1));
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
            AppConfig& c = _store->cfg;
            _copyParam(req, "wifiSsid",   c.wifiSsid,   sizeof(c.wifiSsid),   false);
            _copyParam(req, "wifiPass",   c.wifiPass,   sizeof(c.wifiPass),   true);
            _copyParam(req, "mqttServer", c.mqttServer, sizeof(c.mqttServer), false);
            if (req->hasParam("mqttPort", true))
                c.mqttPort = (uint16_t) req->getParam("mqttPort", true)->value().toInt();
            _copyParam(req, "mqttUser",   c.mqttUser,   sizeof(c.mqttUser),   false);
            _copyParam(req, "mqttPass",   c.mqttPass,   sizeof(c.mqttPass),   true);
            _copyParam(req, "webPass",    c.webPass,    sizeof(c.webPass),    true);
            if (req->hasParam("highTemp", true))
                c.highTemp = req->getParam("highTemp", true)->value().toFloat();
            if (req->hasParam("lowTemp", true))
                c.lowTemp = req->getParam("lowTemp", true)->value().toFloat();
            if (!(c.highTemp > c.lowTemp)) { c.highTemp = 43.0f; c.lowTemp = 24.0f; }
            bool ok = _store->save();
            req->send(ok ? 200 : 500, "text/plain",
                      ok ? "Guardado. Reiniciando en 2s..." : "Error al guardar");
            if (ok) { _rebootPending = true; _rebootAt = millis() + 2000; }
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
