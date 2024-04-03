#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define FREQ 5000
#define HIGH_TEMP 42   /*DC = 100%*/
#define LOW_TEMP 22    /*DC = 0%*/
#define TEMP_HISTERESIS 5
#define PWMRANGE 255

//#define PWM_PIN_OUT PD5 //D5 Pin
#define PWM_PIN_OUT_1 9   //D9 Pin
#define PWM_PIN_OUT_2 10   //D10 Pin
#define TEMPERATURE_PIN_ALM PD6  
#define DS_PIN_MONITORING 2   //For DS18B20 Temperature Sensor  

unsigned long delayTime =1000;

volatile float DutyCycle = 100;
volatile float Temperature = 0;
volatile boolean Reading = false;
//boolean ISR_timer = false;

OneWire OneWire_Temperature(DS_PIN_MONITORING);
DallasTemperature DS_sensor(&OneWire_Temperature);
DeviceAddress Addr_DS_sensor = {0x28, 0xA8, 0xF8, 0xE7, 0x08, 0x00, 0x00, 0x91};

void setup() {
  //PWM SETUP
  TCCR1B = (TCCR1B) & (B11111000 | B00000010);  //Set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
  pinMode(PWM_PIN_OUT_1,OUTPUT);
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  pinMode(DS_PIN_MONITORING,INPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,LOW);

  //SERIAL SETUP
  Serial.begin(9600);
  while(!Serial);    // time to get serial running
  Serial.println(F("\nConnecting Serial Port!!!"));
  delay(0.5*delayTime);
  
  Serial.println(F("Serial Port OK..."));
  Serial.println(F(""));
  delay(0.5*delayTime);

  //DS18B20 SENSOR SETUP
  Serial.println(F("Connecting DS18B20 Sensor!!!"));
  DS_sensor.begin();
  while(!DS_sensor.getAddress(Addr_DS_sensor,0)){
    Serial.println(F("Could not find a valid DS18B20 sensor, check wiring or "
                      "try a different address!"));
    analogWrite(PWM_PIN_OUT_1, map(DutyCycle,0,100,0,PWMRANGE));
    delay(0.5*delayTime);
  }
  
  Serial.println(F("DS18B20 Sensor Ready..."));
  Serial.println(F(""));
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
    if(Temperature<HIGH_TEMP-TEMP_HISTERESIS)  digitalWrite(TEMPERATURE_PIN_ALM,LOW);
    if(Temperature<LOW_TEMP)  DutyCycle=0;

    //PWM UPDATE
    DutyCycle = 100-DutyCycle;
    analogWrite(PWM_PIN_OUT_1, map(DutyCycle,0,100,0,PWMRANGE));

    Serial.println(F("Reading Sensor was OK"));
    Serial.print(F("Temp: ")); Serial.print(Temperature);  Serial.println(F("°C"));
    Serial.print(F("DC (%): "));  Serial.print(100-DutyCycle); Serial.println(F("%")); 
    Serial.println(F("")); 
  }
  else{
    Serial.println(F("Sensor Failure!!!"));
    digitalWrite(TEMPERATURE_PIN_ALM,HIGH);
    DutyCycle = 100;
    analogWrite(PWM_PIN_OUT_1, map(DutyCycle,0,100,0,PWMRANGE));
  }
  delay(delayTime);
}