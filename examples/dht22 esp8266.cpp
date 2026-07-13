#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> 
#include <DHT_U.h>
#include <Adafruit_BMP280.h>

#define FREQ 5000
#define HIGH_TEMP 45   /*DC = 100%*/
#define LOW_TEMP 23    /*DC = 0%*/
#define PWMRANGE 1023

#define PWM_PIN_OUT D2
#define TEMPERATURE_PIN_ALM D6    
#define DHT_PIN_MONITORING D4           //For DHT22 Temperature Sensor
#define DHTTYPE DHT11
//const int ADC_pin_monitoring = A0;

volatile float DutyCycle = 0;
volatile float Temperature = 0;
volatile boolean Reading = false;
//boolean ISR_timer = false;

DHT_Unified DHT_sensor(DHT_PIN_MONITORING,DHTTYPE);
uint32_t delayMS;

void setup() {
  analogWriteRange(PWMRANGE);
  analogWriteFreq(FREQ);
  pinMode(PWM_PIN_OUT,OUTPUT);
  pinMode(TEMPERATURE_PIN_ALM,OUTPUT);
  digitalWrite(TEMPERATURE_PIN_ALM,LOW);

  Serial.begin(9600);  
  while (!Serial);
  Serial.println(F("\nSerial Port OK:"));

  DHT_sensor.begin();
  sensor_t sensor;
  DHT_sensor.temperature().getSensor(&sensor);
   Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  DHT_sensor.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;    
}

void loop() {
  sensors_event_t event;
  DHT_sensor.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else{
    Temperature = event.temperature;
    
    if(Temperature>=LOW_TEMP && Temperature<=HIGH_TEMP)
      DutyCycle = 100.0/(float)(HIGH_TEMP-LOW_TEMP)*(Temperature-(float)LOW_TEMP); //Ecuacion DC=f(Temp): DC = m(temp - LOW_TEMP)
    if(Temperature>HIGH_TEMP){
      DutyCycle = 100;
      digitalWrite(TEMPERATURE_PIN_ALM,HIGH);
    }
    if(Temperature<HIGH_TEMP-6) digitalWrite(TEMPERATURE_PIN_ALM,LOW);
    if(Temperature<LOW_TEMP)  DutyCycle=0;

    //PWM UPDATE
    DutyCycle = 100-DutyCycle;
    analogWrite(PWM_PIN_OUT, map(DutyCycle,0,100,0,PWMRANGE));

    Serial.println("Reading was OK");
    Serial.print(F("Temperature: ")); Serial.print(event.temperature);  Serial.println(F("°C"));
    Serial.print("DutyCycle (%): ");  Serial.print(100-DutyCycle); Serial.println("%"); 
  }
  // Get humidity event and print its value.
  DHT_sensor.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    Serial.println(""); 
  }    
  delay(delayMS);
}