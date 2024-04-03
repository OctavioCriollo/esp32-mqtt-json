/*This code is for ESPWROOM-32*/
/*===========================================*/
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define FREQ_PWM 2000
#define CH_PWM 0
#define RESOLUTION_PWM 10

#define HIGH_TEMP 45    /*DC = 100%*/
#define LOW_TEMP 22     /*DC = 0%*/
#define PWMRANGE 1023
#define ON LOW
#define OFF HIGH
#define OPEN true
#define CLOSE false

#define TEMP_HISTERESIS 3
#define TEMP_PRECISION 10     //Precision 10 bits


#define PWM_FAN_1 GPIO_NUM_13
#define PWM_FAN_2 GPIO_NUM_14
#define TEMPERATURE_PIN_ALM GPIO_NUM_17 //Relay OUT1: Temp ALM
#define DOOR_OPEN_PIN_ALM GPIO_NUM_16   //Relay OUT2: Door Open ALM

#define DOOR_OPEN_PIN_MONITORING GPIO_NUM_32
#define DS_PIN_MONITORING GPIO_NUM_27  //For DS18B20 Temperature Sensor 1-WIRE

const char* ssid = "Wifi Octavio Indoor 2G";   //Ingresa el nombre de tu red WiFi
const char* password = "199905498";  //Ingresa la contraseña de tu red WiFi
const char* mqtt_server = "mosquitto.network-telemetrix.com/mqtt";
const char* mqtt_ID = "Home_Octavio";
const char* mqtt_user = "NT";
const char* mqtt_password = "199905498";
const char* mqtt_topic_temp = "valencia/octavio/home/temp";

WiFiClient WIFI_CLIENT;
PubSubClient MQTT_CLIENT(WIFI_CLIENT);

unsigned long delayTime =1000;

volatile float DutyCycle = 0;
volatile float Temperature = 0;
boolean Gabinet_Open = OPEN;
ulong current_time, prev_time;
uint16_t dt_time = 1000;

OneWire OneWire_Temperature(DS_PIN_MONITORING);
DallasTemperature DS_sensor(&OneWire_Temperature);
DeviceAddress Addr_DS_sensor;

void setup() {
  //SERIAL PORT SETUP
  //==============================================================
  Serial.begin(115200);  
  while (!Serial);
  Serial.println(F("\nConnecting Serial Port!!!"));
  delay(0.5*delayTime);
  
  Serial.println(F("Serial Port OK..."));
  Serial.println(F(""));
  delay(0.25*delayTime);

  WiFi.begin(ssid, password);  // Conecta el ESP32 a la red WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi: ");
  }
  Serial.println("Wifi is OK...");
  
  MQTT_CLIENT.setServer(mqtt_server, 9001);

  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,OFF);
  pinMode(DOOR_OPEN_PIN_MONITORING,INPUT_PULLUP);
  pinMode(DOOR_OPEN_PIN_ALM,OUTPUT);
  
    //PWM SETUP
  //==============================================================
  ledcSetup(CH_PWM,FREQ_PWM,RESOLUTION_PWM);
  ledcAttachPin(PWM_FAN_1,CH_PWM);
  ledcAttachPin(PWM_FAN_2,CH_PWM);

    //DS18B20 SENSOR SETUP
  //===============================================================
  Serial.println(F("Connecting DS18B20 Sensor!!!"));
  DS_sensor.begin();
  while(!DS_sensor.getAddress(Addr_DS_sensor,0)){
    Serial.println(F("Could not find a valid DS18B20 sensor, check wiring or "
                      "try a different address!"));
    DutyCycle = 100;
    digitalWrite(TEMPERATURE_PIN_ALM,ON);
    ledcWrite(CH_PWM, map(DutyCycle,0,100,0,PWMRANGE));
    delay(delayTime);

    Gabinet_Open=digitalRead(DOOR_OPEN_PIN_MONITORING);
    digitalWrite(DOOR_OPEN_PIN_ALM,!Gabinet_Open);

  }
  Serial.println(F("DS18B20 Sensor Ready..."));
  Serial.println(F(""));
  DS_sensor.setResolution(Addr_DS_sensor,TEMP_PRECISION);   //10 bits resolution, 0.25°C, 187.5ms
  prev_time = millis();

}

void reconnect() {
  while (!MQTT_CLIENT.connected()) {
    Serial.println("Conectando al servidor MQTT...");

    if (MQTT_CLIENT.connect(mqtt_ID,mqtt_user,mqtt_password)) {
      Serial.println("Conectado al servidor MQTT");

      // Suscribirse a un tema
      MQTT_CLIENT.subscribe(mqtt_topic_temp);
    } else {
      Serial.print("Fallo al conectar al servidor MQTT con error: ");
      Serial.println(MQTT_CLIENT.state());
      delay(2000);
    }
  }
}

void loop() {
  current_time = millis();
  if (current_time - prev_time > dt_time){
    prev_time = current_time;
    Gabinet_Open = digitalRead(DOOR_OPEN_PIN_MONITORING);
    digitalWrite(DOOR_OPEN_PIN_ALM,!Gabinet_Open);
    if(DS_sensor.isConnected(Addr_DS_sensor)){
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
      Serial.println(F("Sensor Failure!!!"));
      digitalWrite(TEMPERATURE_PIN_ALM,ON);
      DutyCycle = 100;
      ledcWrite(CH_PWM, map(DutyCycle,0,100,0,PWMRANGE));
    }
    }

    if (!MQTT_CLIENT.connected()) {
    reconnect();
  }

  MQTT_CLIENT.loop();

  // Publicar un mensaje en un tópico
  MQTT_CLIENT.publish(mqtt_topic_temp, String(Temperature).c_str());
}