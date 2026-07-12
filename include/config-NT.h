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
    float tzOffset;   /*UTC offset in hours for SNTP (e.g. -5.0 = Ecuador)*/
    /*MQTT topic: operator/city/site-MAC/subsystem/{telemetria,control}. The
    device MAC is appended to the site at boot, so it is globally unique and
    one firmware serves the whole fleet.*/
    char mqttOperator[24];   /*claro, cnt, tigo*/
    char mqttCity[24];       /*e.g. "quito", "guayaquil"*/
    char mqttSite[24];       /*RBS name; the MAC is appended automatically*/
    char mqttSubsystem[24];  /*power, generador, baterias, seguridad*/
    uint8_t fanAlarmLogic;   /*general fan alarm: 0=OR 1=AND 2=only FAN1 3=only FAN2*/
    uint8_t relayMap[4];     /*OUT1-4 -> signal: 0=free 1=tempAlarm 2=doorOpenAlarm 3=fanAlarm*/
    float pwmN;              /*PWM curve floor (0..1): fan minimum, held at/below lowTemp*/
    float pwmP;              /*PWM curve exponent (1..10): 1=linear, 2=parabolic*/
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
        cfg.tzOffset = _prefs.getFloat("tzOffset", -5.0f);
        _getStr("mqttOper",  cfg.mqttOperator,  sizeof(cfg.mqttOperator),  "claro");
        _getStr("mqttCity",  cfg.mqttCity,      sizeof(cfg.mqttCity),      "guayaquil");
        _getStr("mqttSite",  cfg.mqttSite,      sizeof(cfg.mqttSite),      "RBS-000");
        _getStr("mqttSubsys",cfg.mqttSubsystem, sizeof(cfg.mqttSubsystem), "power");
        cfg.fanAlarmLogic = _prefs.getUChar("fanLogic", 0);
        if (_prefs.getBytes("relayMap", cfg.relayMap, sizeof(cfg.relayMap)) != sizeof(cfg.relayMap)){
            cfg.relayMap[0]=2; cfg.relayMap[1]=3; cfg.relayMap[2]=1; cfg.relayMap[3]=0;   /*door,fan,temp,free*/
        }
        cfg.pwmN = _prefs.getFloat("pwmN", 0.1f);
        cfg.pwmP = _prefs.getFloat("pwmP", 1.0f);
        _prefs.end();
        /*Sanity: a broken saved range must never disable cooling.*/
        if (!(cfg.highTemp > cfg.lowTemp)) { cfg.highTemp = 43.0f; cfg.lowTemp = 24.0f; }
        /*Sanity: keep the UTC offset within the real-world range.*/
        if (!(cfg.tzOffset >= -12.0f && cfg.tzOffset <= 14.0f)) cfg.tzOffset = -5.0f;
        if (cfg.fanAlarmLogic > 3) cfg.fanAlarmLogic = 0;   /*guard the enum range*/
        for (int i = 0; i < 4; i++) if (cfg.relayMap[i] > 3) cfg.relayMap[i] = 0;
        /*Sanity: keep the PWM curve params in their valid ranges.*/
        if (!(cfg.pwmN >= 0.0f && cfg.pwmN <= 1.0f)) cfg.pwmN = 0.1f;
        if (!(cfg.pwmP >= 1.0f && cfg.pwmP <= 10.0f)) cfg.pwmP = 1.0f;
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
        _prefs.putFloat("tzOffset",   cfg.tzOffset);
        _prefs.putString("mqttOper",  cfg.mqttOperator);
        _prefs.putString("mqttCity",  cfg.mqttCity);
        _prefs.putString("mqttSite",  cfg.mqttSite);
        _prefs.putString("mqttSubsys",cfg.mqttSubsystem);
        _prefs.putUChar("fanLogic",   cfg.fanAlarmLogic);
        _prefs.putBytes("relayMap",   cfg.relayMap, sizeof(cfg.relayMap));
        _prefs.putFloat("pwmN",       cfg.pwmN);
        _prefs.putFloat("pwmP",       cfg.pwmP);
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
