/*This code is for ESPWROOM-32, Connect to Wifi network and MQTT broker.
WiFi conection is by event at void loop.
Copyright (c) 2023 Octavio Criollo.             
========================================================================*/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include "sensor-NT-02.h"
#include "wireless.h"
#include "mqtt-NT.h"
#include "iot-NT.h"

/*WIFI AND MQTT INFORMATION*/
/*========================================================*/
#define MQTT_SERVER "mosquitto.network-telemetrix.com"
#define MQTT_PORT 8883
#define MQTT_CLIENT_USER "Claro-IoT"
#define MQTT_CLIENT_PASS "Claro.2023"
#define MQTT_ID "ESP32-Claro"
#define MQTT_TOPIC_SUB "/claro/control"
#define MQTT_TOPIC_PUB "/claro/monitoring"
const char* ssid = "Claro-Ecuador";
const char* password = "Claro.2023";
//const char* ssid = "Wifi Octavio Indoor 2G";    
//const char* password = "199905498";  
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

int bitRate = 115200;
void serial_setup(int bitRate){
    Serial.begin(bitRate);
    while(not Serial)
       delay(500); 
    Serial.println(F("\nSerial Port OK!!!"));
    delay(500);
}
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\nBROKER: %s",mqtt.server());
  Serial.printf("\nTOPIC: %s",topic);
  Serial.printf("\nRECEIVED MESSAGE:\n");
  DynamicJsonDocument doc(2048);
  deserializeJson(doc,(const byte*)payload,length);
  serializeJson(doc,Serial);
  Serial.println();
}

//Sensor sensor1(DS18B20_MODEL,ONE_WIRE_PIN,"Sensor-01","/biodigestor/temp");
//Sensor input1(IN_1,INPUT_PULLUP,"Door-Open","/Power/doorOpen");
DS18B20 sensor1(ONE_WIRE_PIN,"sensor-01");
PINSTATE input1(IN_1,INPUT_PULLUP,"Door-Open");
PINCONTROL output1(RELAY_OUT_1,PULLUP,"Relay-01");
PWM fan1(PWM_FAN_1,0,2000,10,"FAN 01");
PWM fan2(PWM_FAN_2,0,2000,10,"FAN 02");

Sensors sensors;
Actuators actuators;
Device device("device-01",ESP_32,"/device-01/valencia");

void setup() {
  serial_setup(bitRate);
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
  while(!sensor1.tryConnection()){
    delay(1000);
  }
  sensors.add(&sensor1);
  sensors.add(&input1);
  actuators.add(&fan1);
  actuators.add(&fan2);
  actuators.add(&output1);
  device.sensors = sensors;
  device.actuators = actuators;

  sensor1.setlabel("Sensor temp DS18B20");
}

void loop() {
  float Temperature = sensor1.readTemperature(); 
  float DutyCycle = 100.0f/(float)(48-25)*(Temperature-(float)25);
  fan1.write(DutyCycle);
  fan2.write(DutyCycle);
  output1.writePin(input1.readPin());

  
  Serial.println("\n");
  serializeJson(sensors.toJson(),Serial);
  Serial.println("\n");
  serializeJson(actuators.toJson(),Serial);
  Serial.println("\n");
  serializeJson(device.toJson(),Serial);    
  
  if(wifi_check_connection(ssid,password,4)){
    if(mqtt.checkConnection(1)){
      mqtt.publish(true,device.toJson());
      //mqtt_publish(true,rbs.toJson(),mqtt.client,2048,mqtt.topicPUB());
    }
    if(!WIFIClient.connected())
      Serial.printf("\nFAILURE Conection to %s",mqtt.server()); 
  }
  delay(1000);
  mqtt.client.loop(); 
}