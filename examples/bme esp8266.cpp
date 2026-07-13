#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define FREQ 2000
#define HIGH_TEMP 44   /*DC = 100%*/
#define LOW_TEMP 22    /*DC = 0%*/
#define TEMP_HISTERESIS 5
#define PWMRANGE 1023

#define PWM_PIN_OUT D5
#define TEMPERATURE_PIN_ALM D6    

/*
#define BME_SCK   13
#define BME_MISO  12
#define BME_MOSI  11
#define BME_CS    10
#define SEALEVELPRESSURE_HPA (1013.25)
*/
unsigned long delayTime =1000;

volatile float DutyCycle = 0;
volatile float Temperature = 0;
volatile boolean Reading = false;
//boolean ISR_timer = false;

//SCK (SCL Pin)	--> GPIO5 - D1
//SDI (SDA pin)	 --> GPIO4 - D2
Adafruit_BME280 BME_sensor;   //i2c

void setup() {
  //PWM SETUP
  analogWriteRange(PWMRANGE);
  analogWriteFreq(FREQ);
  pinMode(PWM_PIN_OUT,OUTPUT);
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,LOW);

  //SERIAL SETUP
  Serial.begin(9600);
  while(!Serial);    // time to get serial running
  Serial.print(F("\nConnecting Serial Port!!!"));
  delay(delayTime);
  
  Serial.println(F("\nSerial Port OK..."));
  Serial.println(F(""));
  delay(delayTime);
  
  //BMEP280 SENSOR SETUP
  Serial.println(F("Connecting BME280 Sensor!!!"));

  while(!BME_sensor.begin(0x76,&Wire)){
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or "
                      "try a different address!"));
    DutyCycle = 100;
    digitalWrite(TEMPERATURE_PIN_ALM,HIGH);
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));
    delay(delayTime);
  }
  Serial.println(F("BME280 Sensor Ready..."));
  Serial.println(F(""));
}

void loop() {
  if(BME_sensor.begin(0x76,&Wire)){
    Temperature = BME_sensor.readTemperature();
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
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));

    Serial.println(F("Reading Sensor was OK"));
    Serial.print(F("Temp: ")); Serial.print(Temperature);  Serial.println(F("°C"));
    Serial.print(F("Humidity: "));  Serial.print(BME_sensor.readHumidity());  Serial.println(F("%"));
    Serial.print(F("DC (%): "));  Serial.print(100-DutyCycle); Serial.println(F("%")); 
    Serial.println(F("")); 
  }
  else{
    Serial.println(F("Sensor Failure!!!"));
    digitalWrite(TEMPERATURE_PIN_ALM,HIGH);
    DutyCycle = 100;
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));
  }
  delay(delayTime);
}