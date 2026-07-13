#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define FREQ 5000
#define HIGH_TEMP 42   /*DC = 100%*/
#define LOW_TEMP 24    /*DC = 0%*/
#define TEMP_HISTERESIS 5
#define PWMRANGE 255

//#define PWM_PIN_OUT PD5 //D5 Pin
#define PWM_PIN_OUT_1 9   //D9 Pin
#define PWM_PIN_OUT_2 10   //D10 Pin
#define TEMPERATURE_PIN_ALM PD6    

/*
#define BME_SCK   13
#define BME_MISO  12
#define BME_MOSI  11
#define BME_CS    10
#define SEALEVELPRESSURE_HPA (1013.25)
*/
unsigned long delayTime =1000;

volatile float DutyCycle = 100;
volatile float Temperature = 0;
volatile boolean Reading = false;
//boolean ISR_timer = false;

//SCK (SCL Pin)	--> A5
//SDI (SDA pin)	 --> A4
Adafruit_BME280 BME_sensor;   //i2c

void setup() {
  //PWM SETUP
  TCCR1B = (TCCR1B) & (B11111000 | B00000010);  //Set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
  pinMode(PWM_PIN_OUT_1,OUTPUT);
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,LOW);

  //SERIAL SETUP
  Serial.begin(115200);
  while(!Serial);    // time to get serial running
  Serial.println(F("\nConnecting Serial Port!!!"));
  delay(delayTime);
  
  Serial.println(F("Serial Port OK..."));
  Serial.println(F(""));
  delay(delayTime);

  //BMEP280 SENSOR SETUP
  Serial.println(F("Connecting BME280 Sensor!!!"));

  while(!BME_sensor.begin(0x76,&Wire)){
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or "
                      "try a different address!"));
    analogWrite(PWM_PIN_OUT_1, map(DutyCycle,0,100,0,PWMRANGE));                
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
    analogWrite(PWM_PIN_OUT_1, map(DutyCycle,0,100,0,PWMRANGE));

    Serial.println(F("Reading Sensor was OK"));
    Serial.print(F("Temp: ")); Serial.print(Temperature);  Serial.println(F("°C"));
    Serial.print(F("Humidity: "));  Serial.print(BME_sensor.readHumidity());  Serial.println(F("%"));
    Serial.print(F("DC (%): "));  Serial.print(DutyCycle); Serial.println(F("%")); 
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