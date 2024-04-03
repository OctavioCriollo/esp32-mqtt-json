/*This code is for ESPWROOM-32, Connect to Wifi network and MQTT broker.
WiFi conection is by event at void loop.
Copyright (c) 2023 Octavio Criollo.             
========================================================================*/
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

/*Definiciones de PWM:        
============================*/
#define FREQ_PWM 2000
#define CH_PWM 0
#define RESOLUTION_PWM 10
#define PWMRANGE 1023

/*Definiciones de PIN´s GPIO:                                          
=============================================================================*/
#define PWM_FAN_1 GPIO_NUM_13
#define PWM_FAN_2 GPIO_NUM_14
#define TEMPERATURE_PIN_ALM GPIO_NUM_17 //Relay OUT1: Temp ALM
#define DOOR_OPEN_PIN_ALM GPIO_NUM_16   //Relay OUT2: Door Open ALM
#define DOOR_OPEN_PIN_MONITORING GPIO_NUM_32
#define DS_PIN_MONITORING GPIO_NUM_27  //For DS18B20 Temperature Sensor 1-WIRE

/*Definiciones de etiquetas: Redefinicion
===================================================*/
#define HIGH_TEMP 43    /*DC = 100%*/
#define LOW_TEMP 25     /*DC = 0%*/
#define ON LOW
#define OFF HIGH
#define OPEN true
#define CLOSE false
#define TEMP_HISTERESIS 3
#define TEMP_PRECISION 10     //Precision 10 bits

/*Definicion Variable Globales:
====================================================*/
unsigned long delayTime =1000;
volatile float DutyCycle = 0;
volatile float Temperature = 0;
boolean Gabinet_Open = OPEN;
ulong current_time_ds18b20, last_time_ds18b20;
ulong current_wifi_event_time, last_wifi_event_time;
ulong current_mqtt_event_time, last_mqtt_event_time;
boolean error_DS_sensor = true;

/*Definicion class Sensor:
=================================================*/
OneWire OneWire_Temperature(DS_PIN_MONITORING);
DallasTemperature DS_sensor(&OneWire_Temperature);
DeviceAddress Addr_DS_sensor;

/*Datos de la red Wifi:
======================================================================================*/
const char* ssid = "Infinix Octavio";            //Ingresa el nombre de tu red WiFi*/
//const char* ssid = "Wifi Octavio Indoor 2G";   //Ingresa el nombre de tu red WiFi
const char* password = "199905498";              //Ingresa la contraseña de tu red WiFi

/*Datos del Broker MQTT:
=================================================================*/
const char* mqtt_server = "192.168.0.105";
//const char* mqtt_server = "mosquitto.network-telemetrix.com";
const int mqtt_port = 1883;
const char* mqtt_Client_ID = "ESP_0919452490_Biodigestor";
const char* mqtt_user = "NT";
const char* mqtt_password = "199905498";
const char* mqtt_topic_sub = "Data_Sensor/Valencia/0919452490";
const char* mqtt_topic_pub = "Data_Sensor/Valencia/0919452490";

WiFiClient Wifi_ESP_client;   //Objeto class WiFiClient
PubSubClient mqtt_ESP_client(Wifi_ESP_client);    //Objeto Class PubSubClient

/*Funcion para configurar sensor DS18B20
========================================================*/
void ds18b20Setup();

/*Funcion para conectar a WiFi: Intentara conexion 5 veces (5 seg)
==================================================================*/
void connectToWiFi();

/*Funcion para re-conectar a WiFi. Un solo intento con mensaje de 
NO CONEXION cada n*DelayTime siempre y cuando aun no este conectado.
=====================================================================*/
void reConnectToWiFi();

/*Funcion que indica el estatus de la conexion Wifi
====================================================*/
bool isWiFiConnected();

/*Funcion de eventos por conexion/desconexion del Wifi
=======================================================*/
void wifiEvent(WiFiEvent_t event);

/*Función para conectar al broker MQTT, devuelve true si la conexion es exitosa caso contrario devuelve false
=====================================================================================================================*/
bool connectToMQTT(PubSubClient &mqtt_client, const char *mqtt_ID, const char *mqtt_user, const char *mqtt_password);

/*Función para re-conectar al broker MQTT, devuelve true si la conexion es exitosa caso contrario devuelve false
=======================================================================================================================*/
bool reConnectToMQTT(PubSubClient &mqtt_client, const char *mqtt_ID, const char *mqtt_user, const char *mqtt_password);

void setup(){
  /*Serial Port Setup
  ======================================================*/
  Serial.begin(115200);  
  while (!Serial);{
    Serial.println(F("\nConnecting to Serial Port!!!"));
    delay(0.5*delayTime);
  }
  Serial.println(F("Serial Port OK...\n"));
  delay(0.25*delayTime);

  //connectToWiFi();  //Connect to WiFi
  //mqtt_ESP_client.setServer(mqtt_server, mqtt_port);      //Estableciendo la conexión con el Broker

  //Configurar la seguridad del Cliente MQTT
  //=========================================
  /*client.setBufferSize(1024);
  client.setSecure(true);
  wifiClient.setCACert(root_ca);
  wifiClient.setCertificate(certificate);
  wifiClient.setPrivateKey(private_key);*/

  //if(isWiFiConnected())  
  //  connectToMQTT(mqtt_ESP_client, mqtt_Client_ID, mqtt_user, mqtt_password);   //Conectar al broker MQTT
  //WiFi.onEvent(wifiEvent);  //Función de callback para eventos de WiFi

  /*GPIO Setup
  ===============================================*/
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,OFF);
  pinMode(DOOR_OPEN_PIN_MONITORING,INPUT_PULLUP);
  pinMode(DS_PIN_MONITORING,INPUT_PULLUP);
  pinMode(DOOR_OPEN_PIN_ALM,OUTPUT);

  /*PWM SETUP
  ==============================================*/
  ledcSetup(CH_PWM,FREQ_PWM,RESOLUTION_PWM);
  ledcAttachPin(PWM_FAN_1,CH_PWM);
  ledcAttachPin(PWM_FAN_2,CH_PWM);

  ds18b20Setup(); //DS18B20 Sensor Setup

  last_time_ds18b20 = millis();
  last_wifi_event_time = millis();  //Last wifi event time
  last_mqtt_event_time = millis();  //Last MQTT event time

}

void loop() {
  current_time_ds18b20 = millis();
  if (current_time_ds18b20 - last_time_ds18b20 > delayTime){
    last_time_ds18b20 = current_time_ds18b20;
    Gabinet_Open = digitalRead(DOOR_OPEN_PIN_MONITORING);
    digitalWrite(DOOR_OPEN_PIN_ALM,!Gabinet_Open);
    if(DS_sensor.isConnected(Addr_DS_sensor)){
      error_DS_sensor = false;
      DS_sensor.requestTemperatures();
      Temperature = DS_sensor.getTempCByIndex(0);    
      if(Temperature>=LOW_TEMP && Temperature<=HIGH_TEMP){
        if(Gabinet_Open)
          DutyCycle = 0;
        else 
          DutyCycle = 100.0/(float)(HIGH_TEMP-LOW_TEMP)*(Temperature-(float)LOW_TEMP); //Ecuacion DC=f(Temp): DC = m(temp - LOW_TEMP)
      }
      if(Temperature>HIGH_TEMP){
        DutyCycle = 100;
        digitalWrite(TEMPERATURE_PIN_ALM,ON);
      }
      if(Temperature<HIGH_TEMP-TEMP_HISTERESIS) digitalWrite(TEMPERATURE_PIN_ALM,OFF);
      if(Temperature<LOW_TEMP)  DutyCycle=0;

      //PWM UPDATE
      //DutyCycle = 100-DutyCycle;
      ledcWrite(CH_PWM, map(DutyCycle,0,100,0,PWMRANGE));
      
      Serial.println("Reading was OK");
      Serial.print("Gabinete: ");  if(Gabinet_Open) Serial.println("OPEN"); else Serial.println("CLOSE"); 
      Serial.print("Temp: ");  Serial.print(Temperature); Serial.println("°C");
      Serial.print("DC (%): ");  Serial.print(DutyCycle); Serial.println("%"); 
      
      Serial.println("");    
    }
    else{
      error_DS_sensor = true;
      Serial.println(F("Sensor Failure!!!"));
      digitalWrite(TEMPERATURE_PIN_ALM,ON);
      DutyCycle = 100;
      ledcWrite(CH_PWM, map(DutyCycle,0,100,0,PWMRANGE));
    }
    
    //Codigo para conexion MQTT y envio de data
    //================================================================================
    //if(mqtt_ESP_client.connected() && isWiFiConnected() && !error_DS_sensor){
    //  mqtt_ESP_client.publish(mqtt_topic_pub,String(Temperature).c_str());
    //  mqtt_ESP_client.subscribe(mqtt_topic_sub);
    //}
    //if(!mqtt_ESP_client.connected() && isWiFiConnected())
    //  reConnectToMQTT(mqtt_ESP_client, mqtt_Client_ID, mqtt_user, mqtt_password);
      
    //mqtt_ESP_client.loop(); 
    //================================================================================
  }
}

void ds18b20Setup(){
  Serial.println(F("\nConnecting DS18B20 Sensor!!!"));
  DS_sensor.begin();
  digitalWrite(TEMPERATURE_PIN_ALM,ON);
  while(!DS_sensor.getAddress(Addr_DS_sensor,0)){
    Serial.println(F("Could not find a valid DS18B20 sensor, check wiring or "
                     "try a different address!"));
    DutyCycle = 100;
    ledcWrite(CH_PWM, map(DutyCycle,0,100,0,PWMRANGE));
    delay(delayTime);

    Gabinet_Open=digitalRead(DOOR_OPEN_PIN_MONITORING);
    digitalWrite(DOOR_OPEN_PIN_ALM,!Gabinet_Open);
  }
  digitalWrite(TEMPERATURE_PIN_ALM,OFF);
  Serial.println(F("DS18B20 Sensor Ready..."));
  Serial.println(F(""));
  DS_sensor.setResolution(Addr_DS_sensor,TEMP_PRECISION);   //10 bits resolution, 0.25°C, 187.5ms
}

void connectToWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Conecting to WiFi network: %s...\n",ssid);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 5){
    Serial.printf(".");
    delay(delayTime);
    retry++;
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.printf("\nConection is SUCCESSFUL!!!\n");
    Serial.printf("SSID: %s\n",ssid);
    Serial.printf("IP: %s\n\n",WiFi.localIP().toString().c_str());
  }
  else
    Serial.printf("\nNO CONNECTED!!!\n\n");  
}

void reConnectToWiFi() {
  current_wifi_event_time = millis();
  if(current_wifi_event_time - last_wifi_event_time > 10*delayTime && WiFi.status() != WL_CONNECTED){
    last_wifi_event_time = millis();
    Serial.println("\nWifi DISCONNECTED!!!");
    Serial.printf("Tray to Connect to Wifi network: %s!!!\n\n",ssid);
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(delayTime);
}

bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void wifiEvent(WiFiEvent_t event){
  switch(event){
    case SYSTEM_EVENT_STA_DISCONNECTED:
      WiFi.removeEvent(wifiEvent);
      reConnectToWiFi();
      WiFi.onEvent(wifiEvent);
      break;
    
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.printf("\nSUCCESSFUL connection to Wifi!!!\n");
      Serial.printf("SSID: %s\n",ssid);
      Serial.printf("IP: %s\n\n",WiFi.localIP().toString().c_str());
      break;

    default:
      break;
  }
}

bool connectToMQTT(PubSubClient &mqtt_client, const char *mqtt_ID, const char *mqtt_user, const char *mqtt_password){
  if(!mqtt_client.connected()){
    Serial.printf("Tray connection to broker: %s!!!\n",mqtt_server);
    if(mqtt_client.connect(mqtt_ID, mqtt_user, mqtt_password)){
      Serial.printf("Connection to MQTT Broker %s is OK!!!\n",mqtt_server);
      return true;
    }
    else{
      Serial.printf("Error connection to broker MQTT: ");
      Serial.printf("%d\n\n",mqtt_client.state());
      return false;
    }
  }
  return true;
}

bool reConnectToMQTT(PubSubClient &mqtt_client, const char *mqtt_ID, const char *mqtt_user, const char *mqtt_password){
  current_mqtt_event_time = millis();
  if(!mqtt_client.connected() && current_mqtt_event_time - last_mqtt_event_time > 10*delayTime){
    last_mqtt_event_time = millis();
    Serial.println("\nMQTT Connection is lost!!!");
    Serial.printf("Tray connection to broker: %s!!!\n",mqtt_server);
    if(mqtt_client.connect(mqtt_ID, mqtt_user, mqtt_password)){
      Serial.printf("Connection to MQTT Broker %s is OK!!!\n\n",mqtt_server);
      return true;
    }
    else{
      Serial.printf("Error connection to broker MQTT: ");
      Serial.printf("%d\n\n",mqtt_client.state());
      return false;
    }
  }
  return true;
}