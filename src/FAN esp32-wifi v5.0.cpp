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

/*PWM variables
==============================*/
u_int8_t pwm_resolution = 10;
u_int8_t pwm_channel0 = 0, pwm_channel1 = 1;
int pwm_freq = 10000;

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
#define TEMP_HISTERESIS 3

/*Definicion Variable Globales:
====================================================*/
unsigned long delayTime = 1000;
//Timer<1000> timer;   /*parked with the tachometer feature*/

volatile float pwm = 100;
float n = 0.1;  /*FAN se apaga al 10% del PWM MAX*/
boolean doorOpenCabinet = OPEN;
ulong current_time, last_time;
/*event-driven networking globals removed; see docs/lab-01-roadmap.md*/

DS18B20 temp1(ONE_WIRE_PIN,"temp1");
PWM fan1(PWM_FAN_1,pwm_channel0,pwm_freq,pwm_resolution,"fan1");
PWM fan2(PWM_FAN_2,pwm_channel1,pwm_freq,pwm_resolution,"fan2");
//TACHOMETER tachMon1(TACH_FAN_1,INPUT_PULLUP,3,"tachMon1");
//TACHOMETER tachMon2(TACH_FAN_2,INPUT_PULLUP,3,"tachMon2");  /*TACH_FAN_2: Cambiar GPIO0 a GPIO12*/
PINSTATE doorOpenMon(IN_1,INPUT_PULLUP,"doorOpenMon");
PINCONTROL doorOpenAlarm(RELAY_OUT_1,OUTPUT,"doorOpenAlarm");
PINCONTROL fanAlarm(RELAY_OUT_2,OUTPUT,"fanAlarm");
Device controller("Controller",ESP_32,"/power/climatizacion");
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
  tachMon1.setRPM(timing);
  tachMon2.setRPM(timing);
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
/*
void IRAM_ATTR isr1(){
  tachMon1.count();
}
void IRAM_ATTR isr2(){
  tachMon2.count();
}
*/
/*Item F globals: defined here so setup() can reference them; the task
implementation lives after the control-cycle function below.*/
SemaphoreHandle_t stateMutex;
TaskHandle_t controlTaskHandle;
void controlTask(void*);

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
  String mac = WiFi.macAddress(); mac.replace(":", "");
  String siteId = String(configStore.cfg.mqttSite) + "-" + mac;
  String topicBase = String(configStore.cfg.mqttOperator) + "/" +
                     configStore.cfg.mqttCity + "/" + siteId + "/" +
                     configStore.cfg.mqttSubsystem;
  mqtt_topic_pub = topicBase + "/telemetria";
  mqtt_topic_sub = topicBase + "/control";
  mqtt.setTopicPUB(mqtt_topic_pub.c_str());
  mqtt.setTopicSUB(mqtt_topic_sub.c_str());
  mqtt_ID = siteId + "-" + configStore.cfg.mqttSubsystem;
  mqtt.setId(mqtt_ID.c_str());

  if(wifi_connect(ssid,password,WIFI_BOOT_ATTEMPTS)){
    #ifdef MQTT_TLS_INSECURE
    WIFIClient.setInsecure();   /*Diagnostics only: no certificate validation*/
    #else
    WIFIClient.setCACert(CA_CERT);
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

  doorOpenAlarm.writePin(doorOpenMon.readPin());
  fan1.write(PWM_MAX);
  fan2.write(PWM_MAX);
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
    temp1.status.setCode("Sensor Failure!");
    Serial.println("\nDS18B20 NOT FOUND: degraded boot, fans at 100%");
  }
  fanAlarm.writePin(not temp1.status.alm());
  temp1.setUpper(configStore.cfg.highTemp);
  temp1.setLower(configStore.cfg.lowTemp);

  sensors.add(&temp1);
  sensors.add(&doorOpenMon);
  //sensors.add(&tachMon1);
  //sensors.add(&tachMon2);
  actuators.add(&fan1);
  actuators.add(&fan2);
  actuators.add(&doorOpenAlarm);
  actuators.add(&fanAlarm);
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
      doc["pwm1"]      = fan1.value();
      doc["pwm2"]      = fan2.value();
      doc["door"]      = (bool)doorOpenMon.readPin();
      doc["tempAlarm"] = (bool)temp1.status.alm();
      doc["fanAlarm"]  = (bool)fanAlarm.status.alm();
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
  //last_wifi_event_time = millis();  //Last wifi event time
  //last_mqtt_event_time = millis();  //Last MQTT event time
  
  /*
  tachMon1.enableISR(FALLING,isr1);
  tachMon1.resetTime();
  tachMon1.resetCount();
  tachMon2.enableISR(FALLING,isr2);
  tachMon2.resetTime();
  tachMon2.resetTime();
  //timer.every(1000,timerCallback);
  */
}

/*==================================================================
ITEM F: control decoupled from networking.
The fan-control cycle runs in its own FreeRTOS task at higher priority,
so blocking WiFi/MQTT reconnects in loop() can NEVER freeze thermal
control. Shared device objects are guarded by stateMutex; the network
side only holds it long enough to deep-copy a JSON snapshot.
==================================================================*/
void runControlCycle(){
  /*Dynamic thresholds (item G): editable at runtime via config*/
  const float HIGH_T = configStore.cfg.highTemp;
  const float LOW_T  = configStore.cfg.lowTemp;
bool doorCabinet = doorOpenMon.readPin();
float tempCabinet = temp1.readTemperature();

/*PROCESSING PARAMETERS
======================================================*/
/*Failsafe (item E/A4): readTemperature() returns NAN for a missing
sensor OR an invalid reading (e.g. the DS18B20 -127 disconnect sentinel).
Either way the fans go to full and the alarm fires; a garbage low value
must never be allowed to coast the fans down. This also avoids a second
OneWire round-trip that the old isConnected() guard incurred.*/
if(!isnan(tempCabinet)){
  if(tempCabinet > HIGH_T){
    fan1.write(PWM_MAX);
    fan2.write(PWM_MAX);
    temp1.status.setAlm(ALARM);
    temp1.status.setCode("High Temperature");
  }
  if(tempCabinet < LOW_T){
    temp1.status.setCode("Low Temperature");
  }
  if(tempCabinet < LOW_T - n/(1-n)*(HIGH_T-LOW_T)){
    fan1.write(PWM_MIN);
    fan2.write(PWM_MIN);
  }
  if(tempCabinet >= (LOW_T - n/(1-n)*(HIGH_T-LOW_T)) and tempCabinet <= HIGH_T){
    temp1.status.setAlm(tempCabinet > (HIGH_T - TEMP_HISTERESIS) and temp1.status.alm());
    pwm = PWM_MAX*(1-n)/(HIGH_T-LOW_T)*(tempCabinet-LOW_T) + n*PWM_MAX; //Ecuacion PWM=f(Temp): PWM = m(temp - LOW_T)
    pwm = round(pwm*100)/100;
    fan1.write(pwm);
    fan2.write(pwm);
    if(tempCabinet >= LOW_T and tempCabinet <= HIGH_T and !temp1.status.alm())
      temp1.status.setCode("Temperature OK"); 
    if (tempCabinet > (HIGH_T - TEMP_HISTERESIS) and tempCabinet <= HIGH_T and temp1.status.alm())
      temp1.status.setCode("Temperature decreasing");
  } 
}
else{
  temp1.status.setAlm(ALARM);
  temp1.status.setCode("Sensor Failure!");
  fan1.write(PWM_MAX);
  fan2.write(PWM_MAX);
}
 
if(doorCabinet == OPEN){
  doorOpenAlarm.status.setCode("Door is OPEN");
  fan1.write(PWM_MIN);
  fan2.write(PWM_MIN);
  if(!fan1.status.alm())  fan1.status.setCode("FAN1 OFF");  
  if(!fan2.status.alm())  fan2.status.setCode("FAN2 OFF");
}
else{
  doorOpenAlarm.status.setCode("Door is CLOSE");
  if(!fan1.status.alm() and tempCabinet > LOW_T*(1 + 0.06)) fan1.status.setCode("FAN1 ON");  /*check here*/
  else if (tempCabinet < LOW_T)  fan1.status.setCode("FAN1 OFF");   
  if(!fan2.status.alm() and tempCabinet > LOW_T*(1 + 0.06)) fan2.status.setCode("FAN2 ON");
  else if (tempCabinet < LOW_T)  fan2.status.setCode("FAN2 OFF"); 
  
  /*
  if(tachMon1.rpm() == 0){
    if(!fan1.status.alm()){
      fan1.status.setCode("FAN1 OFF");
      fan1.status.setAlm(NOT_ALARM);
    }
    else{
      fan1.status.setCode("FAN1 failure!");
      fan1.status.setAlm(ALARM);
    }   
  }
  else{
    fan1.status.setAlm(NOT_ALARM);
    fan1.status.setCode("FAN1 Working!");
  }   
  if(tachMon2.rpm() == 0){
    if(!fan2.status.alm()){
      fan2.status.setCode("FAN2 OFF");
      fan2.status.setAlm(NOT_ALARM);
    }
    else{
      fan2.status.setCode("FAN2 failure!");
      fan2.status.setAlm(ALARM);
    } 
  }
  else{
    fan2.status.setAlm(NOT_ALARM);
    fan2.status.setCode("FAN2 Working!");
  }
  */
}
doorOpenAlarm.status.setAlm(doorCabinet);
doorOpenAlarm.writePin(doorCabinet);
fanAlarm.status.setAlm(temp1.status.alm() or fan1.status.alm() or fan2.status.alm());
fanAlarm.writePin(not (temp1.status.alm() or fan1.status.alm() or fan2.status.alm()));
if(fanAlarm.status.alm())  fanAlarm.status.setCode("FAN's Alarms!");
else  fanAlarm.status.setCode("FAN's is OK!");

Serial.println();
Serial.print("POWER FAN MONITORING:");
Serial.print("\nCabinet: ");  Serial.print(doorOpenAlarm.status.code()); 
Serial.print("\nSensor Temp status: ");   Serial.print(temp1.status.code());
Serial.print("\nSensor Temp value: ");   Serial.print(tempCabinet); Serial.print("°C");
Serial.print("\nFAN1 status: ");  Serial.print(fan1.status.code()); 
//Serial.print("\nFAN1 RPM: "); Serial.print(tachMon1.rpm()); Serial.print(" rpm"); 
Serial.print("\nFAN1 PWM (%): "); Serial.print(fan1.value()); Serial.print("%"); 
Serial.print("\nFAN2 status: ");  Serial.print(fan2.status.code()); 
//Serial.print("\nFAN2 RPM: "); Serial.print(tachMon2.rpm()); Serial.print(" rpm");    
Serial.print("\nFAN2 PWM (%): "); Serial.print(fan2.value()); Serial.print("%");
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
  if (millis() - last_time > delayTime){
    last_time = current_time;
    /*Snapshot under mutex, network OUTSIDE it: a slow TLS handshake
    must not stall the control task.*/
    JsonDocument snapshot;
    bool haveSnapshot = false;
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2000)) == pdTRUE){
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
      WIFIClient.setCACert(CA_CERT);
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