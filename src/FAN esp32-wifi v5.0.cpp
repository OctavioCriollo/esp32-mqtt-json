/*This code is for ESPWROOM-32, Connect to Wifi network and MQTT broker.
WiFi conection is by event at void loop.
Copyright (c) 2023 Octavio Criollo.             
========================================================================*/
#include <Arduino.h>
#include <PubSubClient.h>
#include "sensor-NT-02.h"
#include "wireless-NT.h"
#include "mqtt-NT.h"
#include "IoT-NT.h"
//#include <arduino-timer.h>   /*parked: Timer is only used by the (parked) tachometer code*/
#include <esp_task_wdt.h>
#include <esp_mac.h>   /*esp_read_mac(): factory MAC from eFuse, no WiFi needed*/

/*PWM variables
==============================*/
u_int8_t pwm_resolution = 10;
u_int8_t pwm_channel0 = 0, pwm_channel1 = 1;
int pwm_freq = 10000;
u_int8_t fan_pulses_per_rev = 2;   /*tach pulses per revolution (typical DC fan; check the datasheet)*/

/*WIFI AND MQTT INFORMATION
Credentials live in include/secrets.h (gitignored). Copy
include/secrets.h.example and fill in real values.*/
/*========================================================*/
#include "secrets.h"
#include "ca_cert.h"
#include "config-NT.h"
#include "web-NT.h"

#define FW_VERSION "6.0"
/*MQTT client id and topics are built at boot from NVS config
(operator/site/subsystem), so one firmware serves the whole fleet.*/
#define MQTT_RETRY_MS 15000   /*min gap between MQTT/TLS reconnect attempts*/
#define WIFI_BOOT_ATTEMPTS 10 /*~10 s STA association window at boot (was 5)*/
#define AP_RESCUE_RETRY_MS 180000 /*retry STA every 3 min while in AP rescue*/
/*SNTP time sync (real timestamps). The UTC offset is per-site and lives in
NVS config (configStore.cfg.tzOffset, hours) editable from the portal.
Applied whenever WiFi comes up; nowIso8601() in the sensor library reads
the synced clock.*/
#define DAYLIGHT_OFFSET_SEC 0
#define NTP_SERVER         "pool.ntp.org"

ConfigStore configStore;   /*NVS-backed runtime config (item G)*/
WebPortal webPortal;       /*config/status/OTA portal (item H)*/
bool apRescueMode = false; /*true: WiFi assoc failed, AP FanController-Setup up*/
bool g_mqttUp = false;     /*MQTT link state, set in loop(), shown on dashboard*/
const char* ssid;          /*bound to configStore.cfg after load()*/
const char* password;  
String ip;
/*Real timestamps are pending via ESP32 SNTP configTime(); the old
NTPClient/WiFiUDP approach was removed. See docs/lab-01-roadmap.md.*/
/*========================================================*/

const char* mqtt_server;
uint16_t mqtt_port;
const char* mqtt_user;
const char* mqtt_password;
String mqtt_topic_pub;   /*built from config in setup(): oper/site/subsys/telemetria*/
String mqtt_topic_sub;   /*built from config in setup(): oper/site/subsys/control*/
String mqtt_ID;          /*built from config in setup()*/

WiFiClientSecure WIFIClient;
MQTT mqtt(WIFIClient);

#define ON LOW
#define OFF HIGH

#define OPEN true
#define CLOSE false

/*Definicion Variable Globales:
====================================================*/
//Timer<1000> timer;   /*parked with the tachometer feature*/

volatile float pwm = 100;
float n = 0.1;  /*FAN se apaga al 10% del PWM MAX*/
boolean doorOpenCabinet = OPEN;
ulong current_time, last_time;
/*event-driven networking globals removed; see docs/lab-01-roadmap.md*/

DS18B20 temp1(ONE_WIRE_PIN,"temp1");
PWM speedFan1(PWM_FAN_1,pwm_channel0,pwm_freq,pwm_resolution,"speedFan1");
PWM speedFan2(PWM_FAN_2,pwm_channel1,pwm_freq,pwm_resolution,"speedFan2");
TACHOMETER fan1(TACH_FAN_1,INPUT_PULLUP,fan_pulses_per_rev,"fan1");
TACHOMETER fan2(TACH_FAN_2,INPUT_PULLUP,fan_pulses_per_rev,"fan2");
PINSTATE doorOpenMon(IN_1,INPUT_PULLUP,"doorOpenMon");
PINCONTROL doorOpenAlarm(RELAY_OUT_1,OUTPUT,"doorOpenAlarm");
PINCONTROL fanAlarm(RELAY_OUT_2,OUTPUT,"fanAlarm");
PINCONTROL tempAlarm(RELAY_OUT_3,OUTPUT,"tempAlarm");
Device controller("Controller",ESP_32);
Sensors sensors;
Actuators actuators;

void serial_setup(int bitRate){
    delay(200); 
    Serial.begin(bitRate);
    while(not Serial)
       delay(200); 
    Serial.println(F("\nSerial Port OK!!!"));
    delay(200);
}
/*
bool timerCallback(void *){
  fan1.setRPM(timing);
  fan2.setRPM(timing);
  return true;
}
*/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\n TOPIC: %s",topic);
  Serial.print("\nRECEIVED MESSAGE:\n");
  JsonDocument doc;
  deserializeJson(doc,(const byte*)payload,length);
  serializeJson(doc,Serial);
  Serial.println();
}
void IRAM_ATTR isr1(){
  fan1.count();
}
void IRAM_ATTR isr2(){
  fan2.count();
}
/*Item F globals: defined here so setup() can reference them; the task
implementation lives after the control-cycle function below.*/
SemaphoreHandle_t stateMutex;
TaskHandle_t controlTaskHandle;
void controlTask(void*);
bool fanGeneralAlarm();   /*general fan alarm per cfg.fanAlarmLogic (0=OR/1=AND/2=FAN1/3=FAN2)*/
void applyRelayMapping();   /*point each alarm actuator at its NVS-mapped relay*/
/*Broker CA for TLS: the NVS-stored PEM when one was uploaded from the portal,
else the compiled-in factory CA. cfg lives for the whole runtime, so the
pointer stays valid for the TLS session.*/
const char* activeCaCert(){
  return configStore.cfg.caCert[0] ? configStore.cfg.caCert : CA_CERT;
}

void setup(){
  serial_setup(115200);

  /*Item G: runtime config from NVS (compile-time defaults on first boot)*/
  configStore.load();
  ssid          = configStore.cfg.wifiSsid;
  password      = configStore.cfg.wifiPass;
  mqtt_server   = configStore.cfg.mqttServer;
  mqtt_port     = configStore.cfg.mqttPort;
  mqtt_user     = configStore.cfg.mqttUser;
  mqtt_password = configStore.cfg.mqttPass;
  mqtt.setServer(mqtt_server);
  mqtt.setPort(mqtt_port);
  mqtt.setUser(mqtt_user);
  mqtt.setPassword(mqtt_password);
  /*Build the topic hierarchy and client id from config:
  operator/city/site-MAC/subsystem/{telemetria,control}. The device MAC is
  appended to the site, so it is globally unique; one firmware serves the
  whole fleet and wildcard ACLs work (e.g. claro/+/+/power/telemetria).*/
  /*Read the factory MAC straight from eFuse (esp_read_mac): always valid,
  independent of WiFi init/mode/timing. Same value WiFi.macAddress() gives
  once WiFi is up, so the topic/client-id match the STA MAC on the network.
  (Before this, the MAC was read pre-WiFi-init and came out all-zeros.)*/
  uint8_t macRaw[6]; esp_read_mac(macRaw, ESP_MAC_WIFI_STA);
  char macNo[13], macCo[18];
  snprintf(macNo, sizeof(macNo), "%02X%02X%02X%02X%02X%02X",
           macRaw[0],macRaw[1],macRaw[2],macRaw[3],macRaw[4],macRaw[5]);
  snprintf(macCo, sizeof(macCo), "%02X:%02X:%02X:%02X:%02X:%02X",
           macRaw[0],macRaw[1],macRaw[2],macRaw[3],macRaw[4],macRaw[5]);
  String mac = macNo;         /*uppercase hex, no colons — for topic/client-id*/
  controller.setMAC(macCo);   /*colon form for the device JSON identity*/
  String siteId = String(configStore.cfg.mqttSite) + "-" + mac;
  /*Topics are forced to lowercase (MQTT convention) regardless of how the
  identity is typed in Device Info, so subscribers/ACLs have one stable case.*/
  String topicBase = String(configStore.cfg.mqttOperator) + "/" +
                     configStore.cfg.mqttCity + "/" + siteId + "/" +
                     configStore.cfg.mqttSubsystem;
  topicBase.toLowerCase();
  mqtt_topic_pub = topicBase + "/telemetria";
  mqtt_topic_sub = topicBase + "/control";
  mqtt.setTopicPUB(mqtt_topic_pub.c_str());
  mqtt.setTopicSUB(mqtt_topic_sub.c_str());
  /*Client ID = site-MAC only (site keeps its typed case, MAC uppercase); the
  subsystem is dropped because it already lives in the topic path.*/
  mqtt_ID = siteId;
  mqtt.setId(mqtt_ID.c_str());

  if(wifi_connect(ssid,password,WIFI_BOOT_ATTEMPTS)){
    #ifdef MQTT_TLS_INSECURE
    WIFIClient.setInsecure();   /*Diagnostics only: no certificate validation*/
    #else
    WIFIClient.setCACert(activeCaCert());
    #endif
    configTime((long)(configStore.cfg.tzOffset*3600), DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    /*Real-cert TLS validates the broker certificate's dates against the
    system clock; an unsynced clock (1970) fails the handshake with a date
    error. SNTP syncs asynchronously, so wait (bounded) for a valid time
    before the first TLS/MQTT connect. Safe to block here: fans are at the
    failsafe PWM_MAX and the task watchdog is not armed yet.*/
    struct tm _tsync;
    if(!getLocalTime(&_tsync, 8000))
      Serial.println("\nWARNING: clock not synced in 8s; first MQTT connect may retry");
    if(mqtt.connect(1)){
      //mqtt.client.setCallback(callback);
      mqtt.subscribe();
      String welcome = "Hello I am " + mqtt_ID;
      mqtt.client.publish(mqtt_topic_pub.c_str(),welcome.c_str());
    }
  }
  else{
    /*Item H: AP rescue mode. WiFi association failed (wrong credentials,
    network down): raise our own AP so the portal stays reachable at
    http://192.168.4.1 and the device can be reconfigured without a
    reflash. Thermal control keeps running regardless.*/
    apRescueMode = true;
    /*AP_STA (not AP): the rescue portal stays reachable while the station
    interface keeps retrying the configured network in the background, so
    a router that comes up late (e.g. after a power cut) reconnects us
    automatically instead of stranding the device on its own AP forever.*/
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("FanController-Setup", configStore.cfg.webPass);
    Serial.printf("\nAP RESCUE MODE: SSID FanController-Setup, http://%s/\n",
                  WiFi.softAPIP().toString().c_str());
  } 

  /*Apply the saved relay mapping (NVS) before the first writes, so each alarm
  starts on ITS configured relay -- not on the constructor's default pin.*/
  applyRelayMapping();
  doorOpenAlarm.writePin(doorOpenMon.readPin());
  speedFan1.write(PWM_MAX);
  speedFan2.write(PWM_MAX);
  fanAlarm.on();

  /*Degraded boot (item E): a missing/broken DS18B20 must NOT hang the
  device forever. Bounded attempts; on failure the fans stay at PWM_MAX
  (failsafe set above), the alarm relay fires, and boot continues so the
  failure is REPORTED over MQTT instead of silencing the controller.*/
  bool sensorOk = false;
  for (int i = 0; i < 10 && !(sensorOk = temp1.tryConnection()); i++)
    delay(1000);
  if (!sensorOk) {
    temp1.status.setAlm(ALARM);
    temp1.status.setCode(SENSOR_FAILURE);
    Serial.println("\nDS18B20 NOT FOUND: degraded boot, fans at 100%");
  }
  fanAlarm.writePin(not temp1.status.alm());
  temp1.setUpper(configStore.cfg.highTemp);
  temp1.setLower(configStore.cfg.lowTemp);

  sensors.add(&temp1);
  sensors.add(&doorOpenMon);
  sensors.add(&fan1);
  sensors.add(&fan2);
  actuators.add(&speedFan1);
  actuators.add(&speedFan2);
  actuators.add(&doorOpenAlarm);
  actuators.add(&fanAlarm);
  actuators.add(&tempAlarm);
  controller.sensors = sensors;
  controller.actuators = actuators;
 
  /*Hardware watchdog (item E): if the main loop wedges (e.g. a stuck TLS
  handshake or a driver fault), reboot instead of leaving a frozen
  controller in the cabinet. Arduino core may pre-initialize the TWDT, so
  fall back to reconfigure.*/
  esp_task_wdt_config_t wdt_cfg = {
      .timeout_ms = 30000,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  if (esp_task_wdt_init(&wdt_cfg) == ESP_ERR_INVALID_STATE)
    esp_task_wdt_reconfigure(&wdt_cfg);
  esp_task_wdt_add(NULL);   /*register the Arduino loop task*/

  stateMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 2,
                          &controlTaskHandle, 1);

  /*Item H: web portal (station or AP-rescue mode). Status callback
  copies live values under the state mutex.*/
  webPortal.begin(configStore, [](JsonDocument& doc){
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(500)) == pdTRUE){
      doc["temp"]      = temp1.temperature();   /*cached: no OneWire here*/
      doc["pwm1"]      = speedFan1.value();
      doc["pwm2"]      = speedFan2.value();
      doc["fan1Rpm"]   = fan1.rpm();
      doc["fan2Rpm"]   = fan2.rpm();
      doc["fan1Alarm"] = (bool)fan1.status.alm();
      doc["fan2Alarm"] = (bool)fan2.status.alm();
      doc["door"]      = (bool)doorOpenMon.readPin();
      doc["tempAlarm"] = (bool)temp1.status.alm();
      doc["tempCode"]  = temp1.status.code();   /*OK/High Temperature/Low.../Sensor Failure*/
      doc["fanAlarm"]  = (bool)fanAlarm.status.alm();
      doc["fanGeneral"]= fanGeneralAlarm();   /*fan1/fan2 combined per configured logic*/
      doc["fanLogic"]  = configStore.cfg.fanAlarmLogic;
      JsonArray _rm = doc["relayMap"].to<JsonArray>();   /*OUT1-4 -> assigned signal*/
      for(int i = 0; i < 4; i++) _rm.add(configStore.cfg.relayMap[i]);
      xSemaphoreGive(stateMutex);
    }
    doc["rssi"]    = WiFi.RSSI();
    uint32_t up = millis()/1000;
    char buf[24];
    snprintf(buf, sizeof(buf), "%lud %02lu:%02lu", up/86400, (up/3600)%24, (up/60)%60);
    doc["uptime"]  = buf;
    doc["version"] = FW_VERSION;
    doc["node"]    = String(configStore.cfg.mqttOperator) + "/" +
                     configStore.cfg.mqttCity + "/" + configStore.cfg.mqttSite +
                     "/" + configStore.cfg.mqttSubsystem;
    doc["highT"]   = configStore.cfg.highTemp;
    doc["lowT"]    = configStore.cfg.lowTemp;
    doc["mqtt"]    = g_mqttUp;
  }, "fan-controller");

  last_time = millis();

  /*Tachometers (fan1/fan2): count pulses by interrupt on the falling edge.
  RPM is computed once per control cycle from the accumulated pulses.*/
  fan1.enableISR(FALLING,isr1);
  fan1.resetTime();
  fan1.resetCount();
  fan2.enableISR(FALLING,isr2);
  fan2.resetTime();
  fan2.resetCount();
}

/*==================================================================
ITEM F: control decoupled from networking.
The fan-control cycle runs in its own FreeRTOS task at higher priority,
so blocking WiFi/MQTT reconnects in loop() can NEVER freeze thermal
control. Shared device objects are guarded by stateMutex; the network
side only holds it long enough to deep-copy a JSON snapshot.
==================================================================*/
/*General fan alarm: combine fan1/fan2 status per the configured logic
(0=OR, 1=AND, 2=only FAN1, 3=only FAN2). Drives the dashboard bulb.*/
bool fanGeneralAlarm(){
  bool a = fan1.status.alm(), b = fan2.status.alm();
  switch(configStore.cfg.fanAlarmLogic){
    case 1:  return a && b;
    case 2:  return a;
    case 3:  return b;
    default: return a || b;
  }
}

/*Relay assignment: which OUT (0-3) each alarm drives, -1 = unassigned. Applied
by pointing each alarm actuator at its mapped relay with setPin(); re-run only
when the map changes (free/vacated relays are driven LOW here).*/
int g_tempOut = -1, g_doorOut = -1, g_fanOut = -1;
void applyRelayMapping(){
  const int relayGpio[4] = {RELAY_OUT_1, RELAY_OUT_2, RELAY_OUT_3, RELAY_OUT_4};
  for(int i = 0; i < 4; i++){ pinMode(relayGpio[i], OUTPUT); digitalWrite(relayGpio[i], LOW); }
  g_tempOut = g_doorOut = g_fanOut = -1;
  for(int i = 0; i < 4; i++){
    switch(configStore.cfg.relayMap[i]){
      case 1: g_tempOut = i; tempAlarm.setPin(relayGpio[i]);     break;
      case 2: g_doorOut = i; doorOpenAlarm.setPin(relayGpio[i]); break;
      case 3: g_fanOut  = i; fanAlarm.setPin(relayGpio[i]);      break;
    }
  }
}

void runControlCycle(){
  /*Dynamic thresholds (item G): editable at runtime via config*/
  const float HIGH_T = configStore.cfg.highTemp;
  const float LOW_T  = configStore.cfg.lowTemp;
  const float TEMP_HYSTERESIS = configStore.cfg.tempHysteresis;
  /*Keep the DS18B20's reported upper/lower in sync with the live thresholds
  (shown in the JSON; the /api/thresholds Set updates cfg but not these).
  Cheap float setters, safe to refresh each cycle.*/
  temp1.setUpper(HIGH_T);
  temp1.setLower(LOW_T);
bool doorCabinet = doorOpenMon.readPin();
static bool highTempAlarmActive = false;
float tempCabinet = temp1.readTemperature();

/*PROCESSING PARAMETERS
======================================================*/
/*Failsafe (item E/A4): readTemperature() returns NAN for a missing
sensor OR an invalid reading (e.g. the DS18B20 -127 disconnect sentinel).
Either way the fans go to full and the alarm fires; a garbage low value
must never be allowed to coast the fans down. This also avoids a second
OneWire round-trip that the old isConnected() guard incurred.*/
if(!isnan(tempCabinet)){
  /*Configurable PWM curve (shared by both fans), from NVS/portal:
       PWM = PWM_MAX * ( n + (1-n)*x^p ),  x = (T-LOW_T)/(HIGH_T-LOW_T)
    n = floor (0..1), p = exponent (1=linear, 2=parabolic). The floor n is
    held at/below LOW_T; set n=0 for fully off when cold. p=1 & n=0.1 reproduce
    the previous linear ramp (10% at LOW_T -> 100% at HIGH_T).*/
  const float pN = configStore.cfg.pwmN, pP = configStore.cfg.pwmP;
  if(tempCabinet >= HIGH_T){
    pwm = PWM_MAX;
  }
  else if(tempCabinet <= LOW_T){
    pwm = pN * PWM_MAX;
  }
  else{
    float x = (tempCabinet - LOW_T) / (HIGH_T - LOW_T);
    pwm = PWM_MAX * (pN + (1.0f - pN) * powf(x, pP));
  }
  pwm = round(pwm*100)/100;
  speedFan1.write(pwm);
  speedFan2.write(pwm);
  /*Temperature-LEVEL alarm (distinct from the sensor-failure alarm, which
  the library sets in readTemperature). HIGH is critical (drives the relay);
  LOW is informational (not critical). Hysteresis holds the high alarm until
  temp drops below HIGH_T - TEMP_HYSTERESIS. The reset band is configured
  from the dashboard and persisted in NVS.*/
  /*Canonical hysteresis state:
      next = above HIGH_T OR (previous AND above HIGH_T - n)
    Keep this latch separate because readTemperature() resets the sensor
    status on every valid read. Sensor-failure alarms therefore cannot be
    mistaken for a previously active high-temperature alarm.*/
  highTempAlarmActive = (tempCabinet > HIGH_T) ||
                        (highTempAlarmActive &&
                         tempCabinet > HIGH_T - TEMP_HYSTERESIS);
  if(highTempAlarmActive){
    temp1.status.setAlm(ALARM);
    temp1.status.setCode(tempCabinet > HIGH_T ? HIGH_TEMP : TEMP_DECREASING);
  }
  else if(tempCabinet < LOW_T){
    temp1.status.setAlm(NOT_ALARM);  temp1.status.setCode(LOW_TEMP);
  }
  else{
    temp1.status.setAlm(NOT_ALARM);  temp1.status.setCode(OK);
  }
}
else{
  temp1.status.setAlm(ALARM);
  temp1.status.setCode(SENSOR_FAILURE);
  speedFan1.write(PWM_MAX);
  speedFan2.write(PWM_MAX);
}
 
if(doorCabinet == OPEN){
  doorOpenAlarm.status.setCode("Door is OPEN");
  speedFan1.write(PWM_MIN);
  speedFan2.write(PWM_MIN);
  if(!speedFan1.status.alm())  speedFan1.status.setCode("FAN1 OFF");  
  if(!speedFan2.status.alm())  speedFan2.status.setCode("FAN2 OFF");
}
else{
  doorOpenAlarm.status.setCode("Door is CLOSE");
  if(!speedFan1.status.alm() and tempCabinet > LOW_T*(1 + 0.06)) speedFan1.status.setCode("FAN1 ON");  /*check here*/
  else if (tempCabinet < LOW_T)  speedFan1.status.setCode("FAN1 OFF");   
  if(!speedFan2.status.alm() and tempCabinet > LOW_T*(1 + 0.06)) speedFan2.status.setCode("FAN2 ON");
  else if (tempCabinet < LOW_T)  speedFan2.status.setCode("FAN2 OFF"); 
  
  /*Fan-failure logic lives below (after the door block) so it is gated by
  the commanded speedFan value, not nested in the door-closed branch.*/
}
doorOpenAlarm.status.setAlm(doorCabinet);   /*physical relay driven by the router below*/

/*Fan RPM + failure alarm from the tachometers (fan1/fan2). Measured once
per cycle; gated by the commanded speed so a fan intentionally off (door
open or cold) never false-alarms -- only "commanded to spin but RPM 0".*/
static ulong lastRpmTime = millis();
ulong nowRpm = millis();
ulong fanTiming = nowRpm - lastRpmTime; lastRpmTime = nowRpm;
if(fanTiming == 0) fanTiming = 1;
fan1.setRPM(fanTiming);
fan2.setRPM(fanTiming);
if(speedFan1.value() <= 0){ fan1.status.setAlm(NOT_ALARM); fan1.status.setCode("Fan Off"); }
else if(fan1.rpm() == 0){   fan1.status.setAlm(ALARM);     fan1.status.setCode(FAN_STOPPED); }
else{                       fan1.status.setAlm(NOT_ALARM); fan1.status.setCode(FAN_WORKING); }
if(speedFan2.value() <= 0){ fan2.status.setAlm(NOT_ALARM); fan2.status.setCode("Fan Off"); }
else if(fan2.rpm() == 0){   fan2.status.setAlm(ALARM);     fan2.status.setCode(FAN_STOPPED); }
else{                       fan2.status.setAlm(NOT_ALARM); fan2.status.setCode(FAN_WORKING); }

/*Alarm signals -- the logic stays in each actuator's status (for the JSON).*/
bool tempAlm = temp1.status.alm();
bool doorAlm = doorCabinet;
bool fanAlm  = fanGeneralAlarm();
fanAlarm.status.setAlm(fanAlm);
fanAlarm.status.setCode(fanAlm ? "Fan Alarm" : "Fan OK");
tempAlarm.status.setAlm(tempAlm);
tempAlarm.status.setCode(tempAlm ? "Temp Alarm" : "Temp OK");

/*Configurable output relays (item B, setPin approach): each alarm actuator is
pointed at its mapped relay via setPin() -- re-pointed only when the map
changes. Then every cycle each ASSIGNED alarm writes its own pin: temp/door
direct (relay ON = alarm), fan inverted (fail-safe). Unassigned alarms don't
write; free/vacated relays are held LOW by applyRelayMapping().*/
static uint8_t appliedMap[4] = {255, 255, 255, 255};
if(memcmp(appliedMap, configStore.cfg.relayMap, 4) != 0){
  applyRelayMapping();
  memcpy(appliedMap, configStore.cfg.relayMap, 4);
}
if(g_tempOut >= 0) tempAlarm.writePin(tempAlm);
if(g_doorOut >= 0) doorOpenAlarm.writePin(doorAlm);
if(g_fanOut  >= 0) fanAlarm.writePin(!fanAlm);

Serial.println();
Serial.print("POWER FAN MONITORING:");
Serial.print("\nCabinet: ");  Serial.print(doorOpenAlarm.status.code()); 
Serial.print("\nSensor Temp status: ");   Serial.print(temp1.status.code());
Serial.print("\nSensor Temp value: ");   Serial.print(tempCabinet); Serial.print("°C");
Serial.print("\nFAN1 status: ");  Serial.print(fan1.status.code());
Serial.print("\nFAN1 RPM: "); Serial.print(fan1.rpm()); Serial.print(" rpm");
Serial.print("\nFAN1 PWM (%): "); Serial.print(speedFan1.value()); Serial.print("%");
Serial.print("\nFAN2 status: ");  Serial.print(fan2.status.code());
Serial.print("\nFAN2 RPM: "); Serial.print(fan2.rpm()); Serial.print(" rpm");
Serial.print("\nFAN2 PWM (%): "); Serial.print(speedFan2.value()); Serial.print("%");
Serial.println();
}

void controlTask(void*){
  esp_task_wdt_add(NULL);
  for(;;){
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2000)) == pdTRUE){
      runControlCycle();
      xSemaphoreGive(stateMutex);
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void loop() {
  esp_task_wdt_reset();
  webPortal.handle();
  current_time = millis();
  if (apRescueMode){
    /*Non-blocking STA recovery (item A3): kick a fresh association every
    AP_RESCUE_RETRY_MS and poll status each loop. On success drop the AP,
    switch to STA-only and resume normal networking; the rescue portal is
    served the whole time via WIFI_AP_STA.*/
    if (WiFi.status() == WL_CONNECTED){
      Serial.printf("\nSTA link recovered, leaving AP rescue: http://%s/\n",
                    WiFi.localIP().toString().c_str());
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      configTime((long)(configStore.cfg.tzOffset*3600), DAYLIGHT_OFFSET_SEC, NTP_SERVER);  /*sync clock*/
      apRescueMode = false;
    }else{
      static ulong lastStaRetry = 0;
      if (millis() - lastStaRetry >= AP_RESCUE_RETRY_MS){
        lastStaRetry = millis();
        WiFi.begin(ssid, password);   /*async; result checked next cycles*/
      }
      return;   /*stay in rescue; portal keeps serving*/
    }
  }
  if (millis() - last_time > (ulong)configStore.cfg.pubSecs * 1000UL){   /*portal-configurable publish cadence*/
    last_time = current_time;
    /*Snapshot under mutex, network OUTSIDE it: a slow TLS handshake
    must not stall the control task.*/
    JsonDocument snapshot;
    bool haveSnapshot = false;
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2000)) == pdTRUE){
      /*Keep the controller IP current for the snapshot; only touches the heap
      when it actually changes (avoids strdup churn every publish).*/
      String curIp = (WiFi.status()==WL_CONNECTED) ? WiFi.localIP().toString() : String("0.0.0.0");
      if(controller.IP()==nullptr || curIp != controller.IP())
        controller.setIP(curIp.c_str());
      snapshot = controller.toJson();
      haveSnapshot = true;
      xSemaphoreGive(stateMutex);
    }
    /*Codigo para conexion MQTT y envio de Data
    /*================================================================*/
    if(wifi_check_connection(ssid,password,2)){
      #ifdef MQTT_TLS_INSECURE
      WIFIClient.setInsecure();
      #else
      WIFIClient.setCACert(activeCaCert());
      #endif
      /*Reconnect backoff: with the broker down, a TLS handshake every
      second churns ~40 KB of heap per attempt and blocks this task for
      seconds, starving AsyncTCP. Retry at most every MQTT_RETRY_MS; an
      established session is never throttled.*/
      static ulong lastMqttAttempt = 0;
      bool mqttReady = mqtt.client.connected();
      if(!mqttReady && millis() - lastMqttAttempt >= MQTT_RETRY_MS){
        lastMqttAttempt = millis();
        mqttReady = mqtt.connect(1);
      }
      if(mqttReady && haveSnapshot)
        mqtt.publish(true,snapshot);
      if(!WIFIClient.connected())
        Serial.printf("\nFAILURE Conection to %s",mqtt.server());
      g_mqttUp = mqttReady;   /*published for the dashboard (read in callback)*/
    }
    else g_mqttUp = false;
    /*================================================================*/
    mqtt.client.loop();  
    //timer.tick(); // tick the timer
  }
}
