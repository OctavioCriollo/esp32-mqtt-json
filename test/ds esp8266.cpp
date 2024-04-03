#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

#define FREQ 5000
#define HIGH_TEMP 42    /*DC = 100%*/
#define LOW_TEMP 24    /*DC = 0%*/
#define PWMRANGE 1023

#define PWM_PIN_OUT D2
#define TEMPERATURE_PIN_ALM D6    
#define DS_PIN_MONITORING D4   //For DS18B20 Temperature Sensor
#define TEMP_HISTERESIS 5
#define TEMP_PRECISION 10   //Precision 10 bits

unsigned long delayTime =1000;

volatile float DutyCycle = 100;
volatile float Temperature = 0;
volatile boolean Reading = false;
volatile boolean Available_Temperature_DS = false;

OneWire OneWire_Temperature(DS_PIN_MONITORING);
DallasTemperature DS_sensor(&OneWire_Temperature);
DeviceAddress Addr_DS_sensor; //= {0x28, 0xA8, 0xF8, 0xE7, 0x08, 0x00, 0x00, 0x91};

void setup() {
  analogWriteRange(PWMRANGE);
  analogWriteFreq(FREQ);
  pinMode(PWM_PIN_OUT,OUTPUT);
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  pinMode(DS_PIN_MONITORING,INPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,LOW);

  //SERIAL PORT SETUP
  //==============================================================
  Serial.begin(9600);  
  while (!Serial);
  Serial.println(F("\nConnecting Serial Port!!!"));
  delay(0.5*delayTime);
  
  Serial.println(F("Serial Port OK..."));
  Serial.println(F(""));
  delay(0.5*delayTime);

  //DS18B20 SENSOR SETUP
  //===============================================================
  Serial.println(F("Connecting DS18B20 Sensor!!!"));
  DS_sensor.begin();
  while(!DS_sensor.getAddress(Addr_DS_sensor,0)){
    Serial.println(F("Could not find a valid DS18B20 sensor, check wiring or "
                      "try a different address!"));
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));
    delay(delayTime);
  }
  Serial.println(F("DS18B20 Sensor Ready..."));
  Serial.println(F(""));
  DS_sensor.setResolution(Addr_DS_sensor,TEMP_PRECISION);   //10 bits resolution, 0.25°C, 187.5ms
}

void loop() {
  if(DS_sensor.isConnected(Addr_DS_sensor)){
    DS_sensor.requestTemperatures();
    Temperature = DS_sensor.getTempCByIndex(0);    
    if(Temperature>=LOW_TEMP && Temperature<=HIGH_TEMP)
      DutyCycle = 100.0/(float)(HIGH_TEMP-LOW_TEMP)*(Temperature-(float)LOW_TEMP); //Ecuacion DC=f(Temp): DC = m(temp - LOW_TEMP)
    if(Temperature>HIGH_TEMP){
      DutyCycle = 100;
      digitalWrite(TEMPERATURE_PIN_ALM,HIGH);
    }
    if(Temperature<HIGH_TEMP-TEMP_HISTERESIS) digitalWrite(TEMPERATURE_PIN_ALM,LOW);
    if(Temperature<LOW_TEMP)  DutyCycle=0;

    //PWM UPDATE
    DutyCycle = 100-DutyCycle;
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));
    
    Serial.println("Reading was OK");
    Serial.print("Temp: ");  Serial.print(Temperature); Serial.println("°C");
    Serial.print("DC (%): ");  Serial.print(100-DutyCycle); Serial.println("%"); 
    Serial.println("");    
  }
  else{
    Serial.println(F("Sensor Failure!!!"));
    digitalWrite(TEMPERATURE_PIN_ALM,HIGH);
    DutyCycle = 100;
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));

  }
  delay(delayTime);
}