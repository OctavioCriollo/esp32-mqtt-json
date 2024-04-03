//last change: 18 sept 2023
#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <map>
#include <vector>

/*Default Timestamp
==============================================*/
#define DEFAULT_TIMESTAMP "2023-08-03T12:34:56z"

/*Default Resolution
==============================================*/
#define DEFAULT_RESOLUTION 10

/*Sensor Models:
==================================*/ 
#define DS18B20_MODEL "DS18B20"
#define BME280_MODEL "BME280"
#define BME180_MODEL "BME180"
#define DTH22_MODEL "DTH22"
#define DIGITAL_STATE_MODEL "BIT_STATE"
#define TACHOMETER_MODEL "TACHOMETER_MODEL"

/*Actuator Models:
=======================================*/ 
#define PWM_MODEL "PWM"
#define DIGITAL_CONTROL_MODEL "ON_OFF"

/*Sensor: Environment Parameter
================================*/ 
#define pH "pH"
#define TEMP "Temperature"
#define HUMIDITY "Humidity"
#define O2 "Oxigeno disuelto"
#define NH3 "NH3"
#define ORP "Redox"
#define CONDUCTIVITY "Conductivity"
#define PRESSURE "Presure"
#define ADC "ADC"

/*Communication Sensor with Node
================================*/ 
#define I2C "I2C"
#define ONE_WIRE "1-Wire"
#define SPI "SPI"
#define RS_485 "RS-485"
#define MODBUS "Modbus"
#define ETHERNET "Ethernet"
#define WIFI "Wifi"

/*Working Mode from Sensor
==========================*/
#define MASTER "Master"
#define SLAVE "Slave"
#define NONE "None"

/*Alert Code from Status of Sensor
=====================================*/
#define OK "OK"
#define SLEEP "Sleep"
#define STAND_BY "Stand-by"
#define WORKING "Working"
#define DISCONNECTED "Disconnected"
#define FAILURE "Failure"

/*Platform Device
===================================*/
#define ESP_32 "ESP-32"
#define ESP_8266 "ESP-8266"
#define ARDUINO_NANO "Arduino Nano"

/*Threshold from sensor and other devices
==========================================*/
#define PWM_MAX 100.0
#define PWM_MIN 0.0
#define HUMIDITY_LOW 0.0
#define HUMIDITY_HIGH 100.0
//#define TEMP_LOW 0
//#define TEMP_HIGH 100

#define ALARM true
#define NOT_ALARM false

class Status {
private:
    const char* _code;
    boolean _alm;
public:
    /*Constructor Class Status
    ===========================================*/
    Status(): _code(OK), _alm(NOT_ALARM) {}

    /*GETTER atributos Class Status
    ===========================================*/
    const char* code() const{ 
        return _code;
    }
    const boolean alm() const{
        return _alm;
    }
    
    /*SETTER atributos Class Status
    ===========================================*/
    void setCode(const char* code){
        _code = code;
    }
    void setAlm(boolean alm){
        _alm = alm;
    }
    
    /*ARDUINO-JSON: Convert Class Status to JSON
    =================================================*/ 
    DynamicJsonDocument toJson() {
        DynamicJsonDocument doc(512);
        doc["code"] = _code;
        doc["alm"] = _alm;
        //return doc.as<JsonObject>();
        return doc;
    }
};

/*SENSOR SECTION*/
class Sensor {
private:
    const char* _label;
    const char* _topic; 
    const char* _id;
    const char* _timestamp;
    const char* _model;
public:
    Status status;
    
    /*CONSTRUCTOR Class Sensor
    ========================================*/ 
    Sensor(const char* id):
    _id(id), 
    _label(nullptr),
    _topic(nullptr), 
    _timestamp(DEFAULT_TIMESTAMP), 
    _model(nullptr),
    status() {}
    
    virtual ~Sensor() {
        if (_label != nullptr)  delete[] _label;
        if (_topic != nullptr)  delete[] _topic;
        if (_model != nullptr)  delete[] _model;
    };   /*Destructor*/

    /*GETTER atributos Class Sensor
    ========================================*/ 
    const char* label() const{
        return _label;
    }
    const char* topic() const{
        return _topic;
    }
    const char* id() const{
        return _id;
    }
    const char* timestamp() const{
        return _timestamp;
    }
    const char* model() const{
        return _model;
    }
    
    /*SETTER atributos Class Sensor
    ========================================*/ 
    void setlabel(const char* label){
        if(_label != nullptr)   delete[] _label;
        _label = strdup(label);
    }
    void setTopic(const char* topic){
        if (_topic != nullptr)  delete[] _topic;    /*Liberar la memoria antigua si es necesario*/
        _topic = strdup(topic);                     /*Crear una nueva copia del topic*/
    }

    void setID(const char* id){
        _id = id;
    }
    void setTimestamp(const char* timestamp){
        _timestamp = timestamp;
    }
    void setModel(const char* model){
        if(_label != nullptr)   delete[] _label;
        _model = strdup(model);
    }
    
    /*ARDUINO-JSON: Convert Class Sensor to JSON
    =============================================*/ 
    virtual DynamicJsonDocument toJson(){
        DynamicJsonDocument doc(1024);
        return doc;
    }
};

class DS18B20: public Sensor {
private:
    //const char* _model;
    u_int8_t _resolution;
    const char* _communication;
    const char* _workingMode;
    u_int8_t _pin;
    
    OneWire _oneWire;
    DeviceAddress _addr;    /*ROM Code 64 bits or 8 Bytes*/
    DallasTemperature sensor;
    JsonArray _address;
    float _temperature;
    float _upper;
    float _lower;
    bool _alm;
    const char* _code;

public:
    /*CONSTRUCTOR Class DS18B20
    ==================================================*/ 
    DS18B20(u_int8_t pin, const char* id): 
    Sensor(id), _pin(pin),
    _resolution(DEFAULT_RESOLUTION), _communication(ONE_WIRE), _workingMode(SLAVE),   
    _oneWire(pin), sensor(&_oneWire), _alm(NOT_ALARM),
    _code(nullptr)
    {   
        String str = "/ds18b20/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(DS18B20_MODEL);
        pinMode(pin,INPUT_PULLUP);      /*Obligatoria para IoT-NT board*/
        sensor.begin();
    }
    /*GETTER atributos Class DS18B20
    ==================================================*/    
    const u_int8_t resolution() const{
        return _resolution;
    }
    const char* communication() const{
        return _communication;
    }   
    const char* workingMode() const{
        return _workingMode;
    }
    u_int8_t pin() const{
        return _pin;
    }
    const char* code() const{ 
        return _code;
    }
    const boolean alm() const{
        return _alm;
    }
    const float upper() const{
        return _upper;
    }
    const float lower() const{
        return _lower;
    }
   
    /*SETTER atributos Class DS18B20
    ===========================================*/
    void setCode(const char* code){
        if(_code != nullptr)    delete[] _code;
        _code = strdup(code);
    }
    void setAlm(boolean alm){
        _alm = alm;
    }
    void setUpper(float temperature){
        _upper = temperature;
    }
     void setLower(float temperature){
        _lower = temperature;
    }

    /*METHOD Class DS18B20
    ==================================================*/
    float readTemperature() {
        sensor.requestTemperaturesByAddress(_addr);
        if(!sensor.isConnected(_addr)){
            status.setAlm(ALARM);
            status.setCode(DISCONNECTED);
        }else{
            _temperature = sensor.getTempC(_addr);
            status.setAlm(NOT_ALARM);
            status.setCode(OK);
        }
        return _temperature;
    }
    bool isConnected() {
        if(!sensor.getAddress(_addr,0)){
            status.setAlm(ALARM);
            status.setCode(DISCONNECTED); 
            return false;
        }
        status.setAlm(NOT_ALARM);
        status.setCode(OK);
        if(sensor.getResolution()!=_resolution)
            sensor.setResolution(_addr,_resolution);
        return true;
    } 
    const String addrStr() {
        String addrStr;
        for (uint8_t i=0;i<sizeof(_addr);i++) {
            if(_addr[i]<16)    
                addrStr+="0";
            addrStr += String(_addr[i], HEX);
        }
        addrStr.toUpperCase();
        return addrStr;
    }
    const JsonArray& address() {
        const size_t capacity = JSON_ARRAY_SIZE(sizeof(_addr));
        DynamicJsonDocument doc(capacity);
        JsonArray array = doc.to<JsonArray>();
        for (size_t i=0;i<sizeof(_addr);i++){
            array.add(_addr[i]);
        }
        _address = array;
        return _address;
    }
    bool tryConnection() { 
        Serial.printf("\nConnecting to %s Sensor!!!",model());
        if(!isConnected()){
            Serial.printf("\n%s sensor no Found.",model());
            Serial.printf("\nCheck wiring or try a different address!\n");
            return false;
        }
        Serial.printf("\n%s sensor Connected.",model());
        Serial.printf("\nAddress: %s",addrStr().c_str());
        Serial.println();
        return true;
    }
   
    /*ARDUINO-JSON: Convert Class DS18B20 to JSON
    ==================================================*/ 
    DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(1024);
        doc["id"] = id();
        doc["model"] = model();
        doc["temperature"] = _temperature; 
        doc["upper"] = _upper; 
        doc["lower"] = _lower; 
        doc["status"] = status.toJson();  
        doc["alm"] = alm();
        doc["code"] = code();
        doc["workingMode"] = _workingMode;
        doc["communication"] = _communication;
        doc["address"] = address();
        doc["addrStr"] = addrStr();               
        //doc["label"] = label();                    
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    }       
   
};

class BME280: public Sensor {
private:
    u_int8_t _resolution;
    const char* _communication;
    const char* _workingMode;
    float _temperature;
    float _humidity;
    float _pressure;

public:
    /*CONSTRUCTOR Class BME280
    ====================================================*/ 
    BME280(const char* id):
    Sensor(id),
    _resolution(DEFAULT_RESOLUTION), _communication(I2C), _workingMode(SLAVE)
    {
        String str = "/bme280/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(BME280_MODEL);
    }
    
    /*GETTER atributos Class BME280
    =======================================*/
    const u_int8_t resolution() const{
        return _resolution;
    }
    const char* communication() const{
        return _communication;
    }   
    const char* workingMode() const{
        return _workingMode;
    }
    
    /*METHOD Class BME280
    =======================================*/
    float readTemperature() {
        //Code here
        return _temperature;
    }
    float readHumidity() {
        //Code here
        return _humidity;
    }
    float readPressure() {
        //Code here
        return _pressure;
    }

    /*ARDUINO-JSON: Convert Class BME280 to JSON
    ==============================================*/ 
       DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(1024);
        doc["id"] = id();
        doc["model"] = model();           
        //Code 
        //doc["label"] = label(); 
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    }    
};

class DTH22: public Sensor {
private:
    u_int8_t _resolution;
    const char* _communication;
    const char* _workingMode;
    u_int8_t _pin;
    
    float _temperature;
    float _humidity;
    float _pressure;

public:
    /*CONSTRUCTOR Class DTH22
    ====================================================*/ 
    DTH22(u_int8_t pin, const char* id):
    Sensor(id), _pin(pin),
    _resolution(DEFAULT_RESOLUTION), _communication(ONE_WIRE), _workingMode(SLAVE) 
    {
       String str = "/dth22/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(BME280_MODEL);
    }
    /*GETTER atributos Class DTH22
    ====================================================*/
    const u_int8_t resolution() const{
        return _resolution;
    }
    const char* communication() const{
        return _communication;
    }   
    const char* workingMode() const{
        return _workingMode;
    }
    u_int8_t pin() const{
        return _pin;
    }

    /*METHOD Class DTH22
    ==================================================*/
    float readTemperature() {
        //Code here
        return _temperature;
    }
    float readHumidity() {
        //Code here
        return _humidity;
    }

    /*ARDUINO-JSON: Convert Class DTH22 to JSON
    ==================================================*/
    DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(1024);
        doc["id"] = id();
        doc["model"] = model();            
        //doc["label"] = label();
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    }    
};

class PINSTATE: public Sensor {
private:
    u_int8_t _pin;
    bool _state;

public:
    /*CONSTRUCTOR Class PINSTATE
    ===================================================================*/ 
    PINSTATE(u_int8_t pin, u_int8_t pin_mode, const char* id):
    Sensor(id), _pin(pin)
    {
       String str = "/pinState/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(DIGITAL_STATE_MODEL);
       pinMode(pin,pin_mode); 
    }
    /*GETTER atributos Class PINSTATE
    ====================================*/
    u_int8_t pin() const{
        return _pin;
    }

    /*METHOD Class PINSTATE
    ====================================*/
    bool readPin(){
        _state = digitalRead(_pin);
        return _state;
    }

    /*ARDUINO-JSON: Convert Class PINSTATE to JSON
    ==================================================*/
    DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(1024);
        doc["id"] = id();
        doc["model"] = model();     
        doc["state"] = _state;
        //doc["label"] = label();  
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    }    
};

class TACHOMETER: public Sensor {
private:
    u_int8_t _pin;
    volatile long _pulses;
    unsigned long _rpm;
    ulong _lastTime;
    u_int8_t _pulseByCycle;

public:
    /*CONSTRUCTOR Class TACHOMETER
    ==============================================================*/ 
    TACHOMETER(u_int8_t pin, u_int8_t pin_mode, u_int8_t pulseByCycle, const char* id):
    Sensor(id), _pin(pin)
    {
       String str = "/tachometer/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(TACHOMETER_MODEL);
       pinMode(pin,pin_mode); 
       _pulses = 0;
       _rpm = 0;
       _lastTime = millis();
       _pulseByCycle = pulseByCycle;
    }
    /*GETTER atributos Class TACHOMETER
    ====================================*/
    u_int8_t pin() const{
        return _pin;
    }
    unsigned long pulses() const{
        return _pulses;
    }
    unsigned long rpm() const{
        return _rpm;
    }
    ulong lastTime() const{
        return _lastTime;
    }

    /*METHOD Class TACHOMETER
    ==================================================================*/
    void resetCount(){
        _pulses = 0;
    }
    void resetTime(){
        _lastTime = millis();
    }
    void count(){
        _pulses++;           
    }
    void setRPM(unsigned long timing){
        _rpm = (_pulses*60000.0)/(_pulseByCycle*timing);
        _pulses = 0;
        _lastTime = millis();
    }
    void enableISR(int mode, void(*intRoutine)()){
        attachInterrupt(digitalPinToInterrupt(_pin), intRoutine, mode);
    } 
    void disableISR(){
        detachInterrupt(digitalPinToInterrupt(_pin));
    } 
       
    /*ARDUINO-JSON: Convert Class TACHOMETER to JSON
    ==================================================*/
    DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(1024);
        doc["id"] = id();
        doc["model"] = model();     
        doc["pulses"] = pulses();
        doc["rpm"] = _rpm;
        //doc["label"] = label();  
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    }    
};

class Sensors {
private:
    std::map<const char*, std::vector<Sensor*>> _map;

public:
    /*CONSTRUCTOR Class Sensors
    ========================================*/ 
    Sensors() {} 

    /*GETTER atributos Class Sensors
    ========================================*/ 
    /*Obtener un sensor por ID*/
    Sensor get(const char* id){
        for(const auto& entry: _map){
            for (Sensor* sensor: entry.second) {
                if (sensor->id() == id) {
                    return *sensor;
                }
            }
        }
        throw std::runtime_error("Sensor not found");
    }

    /*SETTER atributos Class Sensors
    ========================================*/
    /*Agregar un sensor ya creado al objeto Sensors*/
    void add(Sensor* sensor) {
        _map[sensor->id()].push_back(sensor);    
    }
        
    /*ARDUINO-JSON: Convert Class Sensors to JSON
    =================================================================*/
    DynamicJsonDocument toJson() {
        DynamicJsonDocument doc(2048);
        JsonObject obj = doc.to<JsonObject>();
        for (const auto& entry : _map) {
            for (Sensor* sensor : entry.second) {
                obj[sensor->id()] = sensor->toJson();
            }
        }
        return doc;
    }
};

/*ACTUATOR SECTION*/
class Actuator {
private:
    const char* _id;
    const char* _topic; 
    const char* _label;
    const char* _timestamp;   /*ISO 8601 ("AAAA-MM-DDTHH:MM:SS")*/
    const char* _model;

public:
    Status status;

    /*CONSTRUCTOR Class Actuator
    ========================================*/ 
    Actuator(const char* id):
    _id(id), 
    _topic(nullptr), 
    _label(nullptr), 
    _timestamp(DEFAULT_TIMESTAMP),
    _model(nullptr),
    status() {}
    
    virtual ~Actuator() {
        if(_label != nullptr)   delete[] _label;
        if(_model != nullptr)   delete[] _model;
        if(_topic != nullptr)   delete[] _topic;
    };   /*Destructor*/

    /*GETTER atributos Class Actuator
    ========================================*/ 
    const char* label() const{
        return _label;
    }
    const char* topic() const{
        return _topic;
    }
    const char* id() const{
        return _id;
    }
    const char* timestamp() const{
        return _timestamp;
    }
    const char* model() const{
        return _model;
    }
    
    /*SETTER atributos Class Actuator
    ========================================*/ 
    void setlabel(const char* label){
        if(_label != nullptr)   delete[] _label;
        _label = strdup(label);
    }
    void setTopic(const char* topic){
        if(_topic != nullptr)   delete[] _topic;
        _topic = strdup(topic);
    }
    void setID(const char* id){
        _id = id;
    }
    void setTimestamp(const char* timestamp){
        _timestamp = timestamp;
    }
    void setModel(const char* model){
        if(_model != nullptr)   delete[] _model;
        _model = strdup(model);
    }
    
    /*METHOD virtuales: ARDUINO-JSON
    ========================================*/ 
    virtual DynamicJsonDocument toJson(){
        DynamicJsonDocument doc(1024);
        return doc;
    }
};

class PWM: public Actuator {
private:
    u_int8_t _pin;
    u_int8_t _channel;
    u_int8_t _resolution;
    int _freq;
    float _value;  
    float _low;
    float _high;      
    int _range;

public:  
    /*CONSTRUCTOR Class PWM
    ===================================================================================*/ 
    PWM(u_int8_t pin, u_int8_t channel, int freq, u_int8_t resolution, const char* id): 
    Actuator(id), _pin(pin), _channel(channel), _freq(freq), _resolution(resolution),
    _low(PWM_MIN), _high(PWM_MAX)
    {
        String str = "/pwm/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(PWM_MODEL);
        ledcSetup(channel,freq,resolution);
        ledcAttachPin(pin,channel);
        _range = pow(2,_resolution)-1.0f;
    }

    /*GETTER atributos Class PWM
    ========================================*/
    float value() const{
        return _value;
    }
    const u_int8_t resolution() const {
        return _resolution;
    }
    const u_int8_t pin() const {
        return _pin;
    }
    const u_int8_t channel() const {
        return _channel;
    }
    const int freq() const {
        return _freq;
    }

    /*SETTER atributos Class PWM
    =========================================*/
    
    /*METHOD Class PWM
    =========================================*/
    void write(float value){
        //int range = pow(2,_resolution)-1;
        ledcWrite(_channel, map(value,_low,_high,0.0f,_range));
        _value = value;
    }

    /*ARDUINO-JSON: Convert Class PWM to JSON
    ===========================================*/
    DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(512);
        doc["id"] = id();
        doc["value"] = _value;
        doc["status"] = status.toJson();
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    } 
};

class PINCONTROL: public Actuator {
private:
    u_int8_t _pin;
    bool _state;
  
public:
    /*CONSTRUCTOR Class PINCONTROL
    =============================================================*/ 
    PINCONTROL(u_int8_t pin, u_int8_t pin_mode, const char* id):
    Actuator(id), _pin(pin)
    {
        String str = "/pinControl/" + String(id);
        const char* topic = str.c_str();
        setTopic(topic);
        setModel(DIGITAL_CONTROL_MODEL);
        pinMode(pin,pin_mode); 
    }
    /*GETTER atributos Class PINCONTROL
    ====================================*/
    u_int8_t pin() const{
        return _pin;
    }

    /*METHOD Class PINCONTROL
    ====================================*/
    void on(){
        _state = true;
        digitalWrite(_pin,_state);
    }
    void off(){
        _state = false;
        digitalWrite(_pin,_state);
    }
     void writePin(bool value){
        _state = value;
        digitalWrite(_pin,_state);
    }
   

    /*ARDUINO-JSON: Convert Class PINCONTROL to JSON
    ==================================================*/
    DynamicJsonDocument toJson() override{
        DynamicJsonDocument doc(1024);
        doc["id"] = id();
        doc["model"] = model();     
        doc["state"] = _state;
        //doc["label"] = label();  
        doc["status"] = status.toJson(); 
        doc["topic"] = topic();
        doc["timestamp"] = timestamp();
        return doc;
    }    
};

class Actuators {
private:
    std::map<const char*, std::vector<Actuator*>> _map;

public:
    /*CONSTRUCTOR Class Actuators
    ========================================*/ 
    Actuators() {} 

    /*GETTER atributos Class Actuators
    ========================================*/ 
    /*Obtener un actuator por ID*/
    Actuator get(const char* id) {
        for(const auto& entry: _map){
            for (Actuator* actuator: entry.second) {
                if (actuator->id() == id) {
                        return *actuator;
                }
            }
        }
        throw std::runtime_error("Actuator not found");
    }

    /*SETTER atributos Class Actuators
    ========================================*/
    /*Agregar un actuator ya creado al objeto Actuators*/
    void add(Actuator* actuator) {
        _map[actuator->id()].push_back(actuator);    
    }
        
    /*ARDUINO-JSON: Convert Class Actuators to JSON
    ===================================================================*/
    DynamicJsonDocument toJson() {
        DynamicJsonDocument doc(2048);
        JsonObject obj = doc.to<JsonObject>();
        for (const auto& entry : _map) {
            for (Actuator* actuator : entry.second) {
                obj[actuator->id()] = actuator->toJson();
            }
        }
        return doc;
    }
};

/*DEVICES SECTION*/
class Device {
private:
    const char* _id;
    const char* _label;
    const char* _platform;
    const char* _MAC;
    const char* _IP;
    const char* _communication;
    const char* _workingMode;
    const char* _topic;
    const char* _timestamp;   /*ISO 8601 ("AAAA-MM-DDTHH:MM:SS")*/

public:
    Status status;
    Sensors sensors;
    Actuators actuators;

    /*CONSTRUCTOR Class Device
    ==============================================================================================*/
    Device(const char* id, const char* platform, const char* topic): 
    _id(id), _platform(platform), _topic(topic),
    _label(nullptr),
    _MAC(nullptr),
    _IP(nullptr),
    _communication(nullptr),
    _workingMode(nullptr),
    _timestamp(DEFAULT_TIMESTAMP), 
    status(), sensors(sensors), actuators(actuators) {} 

    /*GETTER atributos privados Class Device
    ========================================*/
    const char* id() const{
        return _id;
    }
    const char* label() const{
        return _label;
    }
    const char* platform() const{
        return _platform;
    }
    const char* MAC() const {
        return _MAC;
    }
    const char* IP() const {
        return _IP;
    }
    const char* communication() {
        return _communication;
    }
    const char* workingMode() {
        return _workingMode;
    }
    const char* topic() {
        return _topic;
    }    
    const char* timestamp() {
        return _timestamp;
    }

    /*SETTER atributos privados Class Device
    ========================================*/
    void setID(const char* id){
        _id = id;
    }
    void setLabel(const char* label){
        if(_label!= nullptr)    delete[] _label;
        _label = strdup(label);
    }
    void setPlatform(const char* platform){
        _platform = platform;
    }
    void setMAC(const char* MAC){
        if(_MAC != nullptr) delete[] _MAC;
        _MAC = strdup(MAC);
    }
    void setIP(const char* IP){
        if(_IP != nullptr) delete[] _IP;
        _IP = strdup(IP);
    }
    void setCommunication(const char* communication){
        if(_communication != nullptr) delete[] _communication;
        _communication = strdup(communication);
    }
    void setWorkingMode(const char* workingMode){
        if(_workingMode != nullptr) delete[] _workingMode;
        _workingMode = strdup(workingMode);
    }
    void setTopic(const char* topic){
        _topic = topic;
    }
    void setTimestamp(const char* timestamp){
        _timestamp = timestamp;
    }

    /*ARDUINO-JSON: Convert Class Device to JSON
    ===============================================*/
    DynamicJsonDocument toJson(){
        DynamicJsonDocument doc(2048);
        JsonObject obj = doc.createNestedObject("device");
        obj["sensors"] = sensors.toJson();
        obj["actuators"] = actuators.toJson();
        obj["id"] = _id;
        obj["platform"] = _platform;
        obj["MAC"] = _MAC;
        obj["IP"] = _IP;
        obj["status"] = status.toJson();
        obj["topic"] = _topic;
        obj["timestamp"] = _timestamp;
        return doc;
    } 
};