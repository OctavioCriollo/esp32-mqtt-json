/*This code is for ESPWROOM-32*/
/*===========================================*/
//#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>

#define FREQ_PWM 2000
#define CH_PWM 0
#define RESOLUTION_PWM 10

#define HIGH_TEMP 45    /*DC = 100%*/
#define LOW_TEMP 25     /*DC = 0%*/
#define PWMRANGE 1023
#define OFF LOW
#define ON HIGH

#define PWM_FAN_1 GPIO_NUM_13  
#define PWM_FAN_2 GPIO_NUM_14

//#define TEMPERATURE_PIN_ALM GPIO_NUM_4 
//#define TEMPERATURE_PIN_ALM GPIO_NUM_15   
//#define TEMPERATURE_PIN_ALM GPIO_NUM_16
#define TEMPERATURE_PIN_ALM GPIO_NUM_17
#define BME_PIN_MONITORING GPIO_NUM_27  //For DS18B20 Temperature Sensor 1-WIRE

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
//#define I2C_FREQ 100000

#define TEMP_HISTERESIS 3
#define TEMP_PRECISION 10     //Precision 10 bits

unsigned long delayTime =1000;

volatile float DutyCycle = 0;
volatile float Temperature = 0;
volatile float Humidity = 0;
ulong current_time, prev_time;
uint16_t dt_time = 1000;

//SCK (SCL Pin)	--> GPIO_NUM_22
//SDI (SDA pin)	 --> GPIO_NUM_21

Adafruit_BME280 BME_sensor;   //I2C

void setup() {
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,OFF);
  
  //PWM SETUP
  //==============================================================
  ledcSetup(CH_PWM,FREQ_PWM,RESOLUTION_PWM);
  ledcAttachPin(PWM_FAN_1,CH_PWM);
  ledcAttachPin(PWM_FAN_2,CH_PWM);
  
  //SERIAL PORT SETUP
  //==============================================================
  Serial.begin(115200);  
  while (!Serial);
  Serial.println(F("\nConnecting Serial Port!!!"));
  delay(0.25*delayTime);
  
  Serial.println(F("Serial Port OK..."));
  Serial.println(F(""));
  delay(0.25*delayTime);

  //BME280 SENSOR SETUP
  //===============================================================
  Serial.println(F("Connecting BME280 Sensor!!!"));
  while(!BME_sensor.begin(0x76,&Wire)){
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or "
                      "try a different address!"));
    DutyCycle = 100;
    digitalWrite(TEMPERATURE_PIN_ALM,ON);
    ledcWrite(CH_PWM, map(DutyCycle,0,100,0,PWMRANGE));
    delay(delayTime);
  }
  Serial.println(F("BME280 Sensor Ready..."));
  Serial.println(F(""));
  prev_time = millis();
}

void loop() {
  current_time = millis();
  if (current_time - prev_time > dt_time){
    prev_time = current_time;
    if(BME_sensor.begin(0x76,&Wire)){
      Temperature = BME_sensor.readTemperature();  
      Humidity = BME_sensor.readHumidity();
      if(Temperature>=LOW_TEMP && Temperature<=HIGH_TEMP)
        DutyCycle = 100.0/(float)(HIGH_TEMP-LOW_TEMP)*(Temperature-(float)LOW_TEMP); //Ecuacion DC=f(Temp): DC = m(temp - LOW_TEMP)
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
      Serial.print("Temp: ");  Serial.print(Temperature); Serial.println("°C");
      Serial.print(F("Humidity: "));  Serial.print(Humidity); Serial.println("%"); 
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
}