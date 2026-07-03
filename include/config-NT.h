/*config-NT.h — Persistent runtime configuration (NVS-backed).
==========================================================================
Item G of the modernization plan: parameters that previously required a
recompile (WiFi/MQTT credentials, temperature thresholds) now live in the
ESP32 NVS partition via Preferences.

- First boot: values fall back to the compile-time defaults in secrets.h,
  so flashing behaves exactly as before.
- After a save (e.g. from the web portal, item H), NVS wins and survives
  both reboots and reflashes.
- factoryReset() wipes the namespace back to compile-time defaults.
==========================================================================*/
#pragma once
#include <Preferences.h>
#include "secrets.h"

#ifndef WEB_ADMIN_PASS
#define WEB_ADMIN_PASS "changeme"
#endif

#define CFG_NAMESPACE "fanctl"

struct AppConfig {
    char wifiSsid[33];
    char wifiPass[65];
    char mqttServer[65];
    uint16_t mqttPort;
    char mqttUser[33];
    char mqttPass[65];
    char webPass[33];
    float highTemp;   /*PWM = 100% at/above*/
    float lowTemp;    /*PWM = 0% at/below*/
};

class ConfigStore {
private:
    Preferences _prefs;

    void _getStr(const char* key, char* dst, size_t dstLen, const char* dflt){
        String v = _prefs.getString(key, dflt);
        strlcpy(dst, v.c_str(), dstLen);
    }

public:
    AppConfig cfg;

    void load(){
        _prefs.begin(CFG_NAMESPACE, true);
        _getStr("wifiSsid",  cfg.wifiSsid,  sizeof(cfg.wifiSsid),  WIFI_SSID);
        _getStr("wifiPass",  cfg.wifiPass,  sizeof(cfg.wifiPass),  WIFI_PASSWORD);
        _getStr("mqttServer",cfg.mqttServer,sizeof(cfg.mqttServer),MQTT_SERVER);
        cfg.mqttPort = _prefs.getUShort("mqttPort", MQTT_PORT);
        _getStr("mqttUser",  cfg.mqttUser,  sizeof(cfg.mqttUser),  MQTT_CLIENT_USER);
        _getStr("mqttPass",  cfg.mqttPass,  sizeof(cfg.mqttPass),  MQTT_CLIENT_PASS);
        _getStr("webPass",   cfg.webPass,   sizeof(cfg.webPass),   WEB_ADMIN_PASS);
        cfg.highTemp = _prefs.getFloat("highTemp", 43.0f);
        cfg.lowTemp  = _prefs.getFloat("lowTemp",  24.0f);
        _prefs.end();
        /*Sanity: a broken saved range must never disable cooling.*/
        if (!(cfg.highTemp > cfg.lowTemp)) { cfg.highTemp = 43.0f; cfg.lowTemp = 24.0f; }
    }

    bool save(){
        if (!_prefs.begin(CFG_NAMESPACE, false)) return false;
        _prefs.putString("wifiSsid",  cfg.wifiSsid);
        _prefs.putString("wifiPass",  cfg.wifiPass);
        _prefs.putString("mqttServer",cfg.mqttServer);
        _prefs.putUShort("mqttPort",  cfg.mqttPort);
        _prefs.putString("mqttUser",  cfg.mqttUser);
        _prefs.putString("mqttPass",  cfg.mqttPass);
        _prefs.putString("webPass",   cfg.webPass);
        _prefs.putFloat("highTemp",   cfg.highTemp);
        _prefs.putFloat("lowTemp",    cfg.lowTemp);
        _prefs.end();
        return true;
    }

    void factoryReset(){
        _prefs.begin(CFG_NAMESPACE, false);
        _prefs.clear();
        _prefs.end();
        load();
    }
};
