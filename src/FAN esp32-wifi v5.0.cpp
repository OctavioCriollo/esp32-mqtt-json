/*This code is for ESPWROOM-32, Connect to Wifi network and MQTT broker.
WiFi conection is by event at void loop.
Copyright (c) 2023 Octavio Criollo.             
========================================================================*/
#include <Arduino.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include "sensor-NT-02.h"
#include "wireless-NT.h"
#include "mqtt-NT.h"
#include "IoT-NT.h"
#include <arduino-timer.h>

/*PWM variables
==============================*/
u_int8_t pwm_resolution = 10;
u_int8_t pwm_channel0 = 0, pwm_channel1 = 1;
int pwm_freq = 10000;

/*WIFI AND MQTT INFORMATION*/
/*========================================================*/
//#define MQTT_SERVER "mosquitto.network-telemetrix.com"
#define MQTT_SERVER "mosquitto.network-telemetrix.com"
#define MQTT_PORT 8883
#define MQTT_CLIENT_USER "Claro-IoT"
#define MQTT_CLIENT_PASS "Claro.2023"
#define MQTT_ID "Controller-iot"
#define MQTT_TOPIC_SUB "/FAN/control"
#define MQTT_TOPIC_PUB "/FAN/monitoring"

const char* ssid = "Claro-IoT";
const char* password = "Claro.2023";  
String ip;
const char* ntpServer = "pool.ntp.org";
WiFiUDP ntpUDP;
/*========================================================*/

const char* mqtt_server = MQTT_SERVER;
uint16_t mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_CLIENT_USER;
const char* mqtt_password = MQTT_CLIENT_PASS;
const char* mqtt_topic_pub = MQTT_TOPIC_PUB;
const char* mqtt_topic_sub = MQTT_TOPIC_SUB;
String mqtt_ID = MQTT_ID;

WiFiClientSecure WIFIClient;
MQTT mqtt(WIFIClient);

#define HIGH_TEMP 43    /*DC = 100%*/
#define LOW_TEMP 24     /*DC = 0%*/
#define ON LOW
#define OFF HIGH

#define OPEN true
#define CLOSE false
#define TEMP_HISTERESIS 3

/*Definicion Variable Globales:
====================================================*/
unsigned long delayTime = 1000;
//Timer<1000> timer;
//unsigned long timing = 1000;
unsigned long timing1 = 1000;
unsigned long timing2 = 1000;

volatile float pwm = 100;
float n = 0.1;  /*FAN se apaga al 10% del PWM MAX*/
boolean doorOpenCabinet = OPEN;
ulong current_time, last_time;
ulong current_wifi_event_time, last_wifi_event_time;
ulong current_mqtt_event_time, last_mqtt_event_time;

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
  DynamicJsonDocument doc(2048);
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
void setup(){
  serial_setup(115200);
  mqtt.setServer(mqtt_server);
  mqtt.setPort(mqtt_port);
  mqtt.setUser(mqtt_user);
  mqtt.setPassword(mqtt_password);
  mqtt.setTopicPUB(mqtt_topic_pub);
  mqtt.setTopicSUB(mqtt_topic_sub);
  mqtt_ID += " (" + String(WiFi.macAddress()) + ")";
  mqtt.setId(mqtt_ID.c_str());

  if(wifi_connect(ssid,password,5)){
    WIFIClient.setInsecure();
    if(mqtt.connect(1)){
      //mqtt.client.setCallback(callback);
      mqtt.subscribe();
      const char* welcome = ("Hello I am " + mqtt_ID).c_str();
      mqtt.client.publish(mqtt_topic_pub,welcome);
    }
  } 

  doorOpenAlarm.writePin(doorOpenMon.readPin());
  fan1.write(PWM_MAX);
  fan2.write(PWM_MAX);
  fanAlarm.on();

  while(!temp1.tryConnection())
    delay(1000);
  fanAlarm.writePin(not temp1.status.alm());
  temp1.setUpper(HIGH_TEMP);
  temp1.setLower(LOW_TEMP);

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

void loop() {
  current_time = millis();
  bool doorCabinet;
  float tempCabinet;
  /*
  if(millis() - tachMon1.lastTime() >= timing1){
    tachMon1.disableISR();
    tachMon1.setRPM(timing1);
    tachMon1.enableISR(FALLING,isr1);
  }
  if(millis() - tachMon2.lastTime() >= timing2){
    tachMon2.disableISR();
    tachMon2.setRPM(timing2);
    tachMon2.enableISR(FALLING,isr2);
  }
  */
  if (millis() - last_time > delayTime){
    last_time = current_time;
    /*MONITORING PARAMETERS
    ================================================*/
    doorCabinet = doorOpenMon.readPin();
    tempCabinet = temp1.readTemperature();
    
    /*PROCESSING PARAMETERS
    ======================================================*/
    if(temp1.isConnected()){
      if(tempCabinet > HIGH_TEMP){
        fan1.write(PWM_MAX);
        fan2.write(PWM_MAX);
        temp1.setAlm(ALARM);
        temp1.setCode("High Temperature");
      }
      if(tempCabinet < LOW_TEMP){
        temp1.setCode("Low Temperature");
      }
      if(tempCabinet < LOW_TEMP - n/(1-n)*(HIGH_TEMP-LOW_TEMP)){
        fan1.write(PWM_MIN);
        fan2.write(PWM_MIN);
      }
      if(tempCabinet >= (LOW_TEMP - n/(1-n)*(HIGH_TEMP-LOW_TEMP)) and tempCabinet <= HIGH_TEMP){
        temp1.setAlm(tempCabinet > (HIGH_TEMP - TEMP_HISTERESIS) and temp1.alm());
        //pwm = 100.00/(float)(HIGH_TEMP-LOW_TEMP)*(tempCabinet-(float)LOW_TEMP); //Ecuacion PWM=f(Temp): PWM = m(temp - LOW_TEMP)
        pwm = PWM_MAX*(1-n)/(HIGH_TEMP-LOW_TEMP)*(tempCabinet-LOW_TEMP) + n*PWM_MAX; //Ecuacion PWM=f(Temp): PWM = m(temp - LOW_TEMP)
        pwm = round(pwm*100)/100;
        fan1.write(pwm);
        fan2.write(pwm);
        if(tempCabinet >= LOW_TEMP and tempCabinet <= HIGH_TEMP and !temp1.alm())
          temp1.setCode("Temperature OK"); 
        if (tempCabinet > (HIGH_TEMP - TEMP_HISTERESIS) and tempCabinet <= HIGH_TEMP and temp1.alm())
          temp1.setCode("Temperature decreasing");
      } 
    }
    else{
      temp1.setAlm(ALARM);
      temp1.setCode("Sensor Failure!");
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
      if(!fan1.status.alm() and tempCabinet > LOW_TEMP*(1 + 0.06)) fan1.status.setCode("FAN1 ON");  /*check here*/
      else if (tempCabinet < LOW_TEMP)  fan1.status.setCode("FAN1 OFF");   
      if(!fan2.status.alm() and tempCabinet > LOW_TEMP*(1 + 0.06)) fan2.status.setCode("FAN2 ON");
      else if (tempCabinet < LOW_TEMP)  fan2.status.setCode("FAN2 OFF"); 
      
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
    fanAlarm.status.setAlm(temp1.alm() or fan1.status.alm() or fan2.status.alm());
    fanAlarm.writePin(not (temp1.alm() or fan1.status.alm() or fan2.status.alm()));
    if(fanAlarm.status.alm())  fanAlarm.status.setCode("FAN's Alarms!");
    else  fanAlarm.status.setCode("FAN's is OK!");

    Serial.println();
    Serial.print("POWER FAN MONITORING:");
    Serial.print("\nCabinet: ");  Serial.print(doorOpenAlarm.status.code()); 
    Serial.print("\nSensor Temp status: ");   Serial.print(temp1.code());
    Serial.print("\nSensor Temp value: ");   Serial.print(tempCabinet); Serial.print("°C");
    Serial.print("\nFAN1 status: ");  Serial.print(fan1.status.code()); 
    //Serial.print("\nFAN1 RPM: "); Serial.print(tachMon1.rpm()); Serial.print(" rpm"); 
    Serial.print("\nFAN1 PWM (%): "); Serial.print(fan1.value()); Serial.print("%"); 
    Serial.print("\nFAN2 status: ");  Serial.print(fan2.status.code()); 
    //Serial.print("\nFAN2 RPM: "); Serial.print(tachMon2.rpm()); Serial.print(" rpm");    
    Serial.print("\nFAN2 PWM (%): "); Serial.print(fan2.value()); Serial.print("%");
    Serial.println();
    /*
    Serial.println("\n");
    serializeJson(controller.toJson(),Serial);
    Serial.println("\n");
    */

    /*Codigo para conexion MQTT y envio de Data
    /*================================================================*/
    if(wifi_check_connection(ssid,password,2)){
      WIFIClient.setInsecure();
      if(mqtt.checkConnection(1)){
        mqtt.publish(true,controller.toJson());
      }
      if(!WIFIClient.connected())
        Serial.printf("\nFAILURE Conection to %s",mqtt.server());    
    }
    /*================================================================*/
    mqtt.client.loop();  
    //timer.tick(); // tick the timer
  }
}