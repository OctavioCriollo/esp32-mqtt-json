#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <map>
#include <vector>

/*Default Timestamp*/
/*==============================================*/
#define DEFAULT_TIMESTAMP "2023-08-03T12:34:56"
#define DEFAULT_RESOLUTION 10

/*Sensor Models:*/
/*================================*/ 
#define DS18B20_MODEL "DS18B20"
#define BME280_MODEL "BME280"
#define BME180_MODEL "BME180"
#define DTH22_MODEL "DTH22"
#define DIGITAL_STATE_MODEL "BIT_STATE"

/*Sensor: Environment Parameter*/
/*================================*/ 
#define pH "pH"
#define TEMP "Temperature"
#define HUMIDITY "Humidity"
#define O2 "Oxigeno disuelto"
#define NH3 "NH3"
#define ORP "Redox"
#define CONDUCTIVITY "Conductivity"
#define PRESSURE "Presure"
#define ADC "ADC"

/*Communication Sensor with Node*/
/*==============================*/ 
#define I2C "I2C"
#define ONE_WIRE "1-Wire"
#define SPI "SPI"
#define RS_485 "RS-485"
#define MODBUS "Modbus"
#define ETHERNET "Ethernet"
#define WIFI "Wifi"

/*Working Mode from Sensor*/
/*========================*/
#define MASTER "Master"
#define SLAVE "Slave"
#define NONE "None"

/*Alert Code from Status of Sensor*/
/*=====================================*/
#define OK "OK"
#define SLEEP "Sleep"
#define STAND_BY "Stand-by"
#define WORKING "Working"
#define DISCONNECTED "Disconnected"
#define FAILURE "Failure"

/*Platform Node*/
/*===================================*/
#define ESP_32 "ESP-32"
#define ESP_8266 "ESP-8266"
#define ARDUINO_NANO "Arduino Nano"

/*Threshold from sensor and other devices*/
/*=======================================*/
#define PWM_MAX 100
#define PWM_MIN 0
#define HUMIDITY_LOW 0
#define HUMIDITY_HIGH 100
#define TEMP_LOW 0
#define TEMP_HIGH 100

#define ALARM true
#define NOT_ALARM false

class Status {
    private:
        String _code;
        boolean _alm;
    public:
        /*Constructor Class Status*/
        /*===========================================*/
        Status(): _code(OK), _alm(NOT_ALARM) {}

        /*GETTER atributos Class Status*/
        /*===========================================*/
        const String& code() const{ 
            return _code;
        }
        const boolean alm() const{
            return _alm;
        }
        /*SETTER atributos Class Status*/
        /*===========================================*/
        void setCode(const String& code){
            _code = code;
        }
        void setAlm(boolean alm){
            _alm = alm;
        }
        /*Convert all Class Status to JSON*/
        /*===========================================*/ 
        DynamicJsonDocument toJson() {
            DynamicJsonDocument doc(256);
            doc["code"] = _code;
            doc["alm"] = _alm;
            return doc;
        }
};

class Sensor{
    private:
        String _label;  
        String _model; 
        u_int8_t _pin;
        String _topic;
          
        u_int8_t _resolution;
        String _communication;
        String _workingMode;
        String _timestamp;
        String _id;

        float _temperature;
        float _humidity;
        bool _state;

        OneWire _oneWire;
        DeviceAddress _addr;    /*ROM Code 64 bits or 8 Bytes*/
        JsonArray _address;

    public:
        Status status;
        DallasTemperature ds18b20;  

        /*Constructor para Sensor Temperatura Model=DS18B20 
        ========================================================================*/
        Sensor(String model, u_int8_t pin, String id, String topic): 
        _id(id), _model(model), _pin(pin),  _topic(topic), 
        _resolution(DEFAULT_RESOLUTION), _communication(ONE_WIRE), _label(),
        _workingMode(SLAVE), _timestamp(DEFAULT_TIMESTAMP),
        _oneWire(pin), ds18b20(&_oneWire), status() 
        {
            pinMode(pin,INPUT_PULLUP);      /*Obligatoria para board IoT-NT*/
            ds18b20.begin();
        }

        /*Constructor para Digital Signal from Pin 
        ========================================================================*/
        Sensor(u_int8_t pin, u_int8_t pin_mode, String id, String topic): 
        _id(id), _pin(pin), _topic(topic),
        _model(DIGITAL_STATE_MODEL), _timestamp(DEFAULT_TIMESTAMP), status(), _label()
        {
            pinMode(pin,pin_mode);      
        }

        /*GETTER atributos Class Sensor
        ========================================*/ 
        const String& label() const{
            return _label;
        }
        const String& id() const{
            return _id;
        } 
        const String& model() const{
            return _model;
        }
        const String& communication() const{
            return _communication;
        }
        const String& topic() const{
            return _topic;
        }    
        const String& timestamp() const{
            return _timestamp;
        }
        const String& workingMode() const{
            return _workingMode;
        }
        u_int8_t resolution() const{
            return _resolution;
        }
        u_int8_t pin() const{
            return _pin;
        }
        
        /*SETTER atributos Class Sensor
        ========================================*/ 
        void setlabel(const String& label){
            _label = label;
        }
        void setID(const String& id){
            _id = id;
        }
        void setTopic(const String& topic){
            _topic = topic;
        }
        void setTimestamp(const String& timestamp){
            _timestamp = timestamp;
        }
        void setWorkingMode(const String& workingMode){
            _workingMode = workingMode;
        }        

        /*METHOD for DS18B20 Sensor
        ========================================*/
        bool isConnected_ds18b20(){
            if(!ds18b20.getAddress(_addr,0)){
                status.setAlm(ALARM);
                status.setCode(DISCONNECTED); 
                return false;
            }
            status.setAlm(NOT_ALARM);
            status.setCode(OK);
            if(ds18b20.getResolution()!=_resolution)
                ds18b20.setResolution(_addr,_resolution);
            return true;
        } 
        const String addr_ds18b20() {
            String addrStr;
            for (uint8_t i=0;i<sizeof(_addr);i++) {
                if(_addr[i]<16)    
                    addrStr+="0";
                addrStr += String(_addr[i], HEX);
            }
            addrStr.toUpperCase();
            return addrStr;
        }
        float read_ds18b20(){
            ds18b20.requestTemperaturesByAddress(_addr);
            if(!ds18b20.isConnected(_addr)){
                status.setAlm(ALARM);
                status.setCode(DISCONNECTED);
            }else{
                _temperature = ds18b20.getTempC(_addr);
                status.setAlm(NOT_ALARM);
                status.setCode(OK);
            }
            return _temperature;
        }

        /*METHOD for BME280 Sensor
        ========================================*/
        std::vector<float> read_bme280(){
            std::vector<float> data(3);
            //Code for read BME280 Sensor
            return data;
        }
        
        /*METHOD for DTH22 Sensor
        ========================================*/
        std::vector<float> read_dth22(){
            std::vector<float> data(2);
            //Code for read DTH22 Sensor
            return data;
        }
        
        /*GLOBAL METHOD for all Sensor
        ========================================*/
        float readTemperature(){
            if(_model==DS18B20_MODEL)
                return read_ds18b20();
            
            if(_model==BME280_MODEL)
                return read_bme280()[0];
            
            if(_model==DTH22_MODEL)
                return read_dth22()[0];
            
            return _temperature;    /*Return the last value*/
        }
        float readHumidity(){
            if(_model==BME280_MODEL)
                return read_bme280()[1];
            
            if(_model==DTH22_MODEL)
                return read_dth22()[1];
            
            return _humidity;   /*Return the last value*/
        }
        bool readPin(){
            if(_model==DIGITAL_STATE_MODEL)
                _state = digitalRead(_pin);
            return _state;
        }

        const String addrStr(){
            if(_model==DS18B20_MODEL)
                return addr_ds18b20();
            
            if(_model==BME280_MODEL)
                return "";
            
            if(_model==DTH22_MODEL)
                return "";
            
            return "";
        }
        const JsonArray& address(){
            const size_t capacity = JSON_ARRAY_SIZE(sizeof(_addr));
            DynamicJsonDocument doc(capacity);
            JsonArray array = doc.to<JsonArray>();
            for (size_t i=0;i<sizeof(_addr);i++){
                array.add(_addr[i]);
            }
            _address = array;
            return _address;
        }
        bool isConnected(){
            if(_model==DS18B20_MODEL)
                return isConnected_ds18b20();
            
            if(_model==BME280_MODEL)
                return false;
            
            if(_model==DTH22_MODEL)
                return false;
            
            return false;
        }
        bool tryConnection(){ 
            Serial.printf("\nConnecting to %s Sensor!!!",_model);
            if(!isConnected()){
                Serial.printf("\n%s sensor no Found.",_model);
                Serial.printf("\nCheck wiring or try a different address!\n");
                return false;
            }
            Serial.printf("\n%s sensor Connected.",_model);
            Serial.printf("\nAddress: %s",addrStr().c_str());
            Serial.println();
            return true;
        }

        /*ARDUINOJSON: Convert Class Sensor to JSON
        =============================================*/ 
        DynamicJsonDocument toJson(){
            DynamicJsonDocument doc(1024);
            doc["id"] = _id;
            doc["model"] = _model;         
            if(_model==DS18B20_MODEL){
                doc["temperature"] = _temperature;
                doc["status"] = status.toJson();
                doc["workingMode"] = _workingMode;
                doc["communication"] = _communication;
                doc["address"] = address();
                doc["addrStr"] = addrStr();               
            }
            else if(_model==BME280_MODEL){
                /* code */
            }
            else if (_model==BME180_MODEL){
                /* code */
            }
            else if (_model==DIGITAL_STATE_MODEL){
                doc["state"] = _state;
            }              
            doc["label"] = _label;
            doc["topic"] = _topic;
            doc["timestamp"] = _timestamp;
            return doc;
        }       
};

class Sensors{
    private:
      u_int8_t _cant;    
      std::map<String, std::vector<Sensor*>> _map;

    public:
        /*Constructor Class Sensors
        ========================================*/ 
        Sensors(): _cant() {} 

        /*GETTER atributos Class Sensors
        ========================================*/
        u_int8_t cant() const{
            return _cant;
        }

        /*Obtener un sensor por tipo y etiqueta */
        Sensor get(const String& label){
            for(const auto& entry: _map){
                for (Sensor* sensor: entry.second) {
                    if (sensor->label() == label) {
                        return *sensor;
                    }
                }
            }
            throw std::runtime_error("Sensor not found");
        }

        /*SETTER atributos Class Sensor
        ========================================*/
        void setCant(u_int8_t cant){
            _cant = cant;
        }

        /*Agregar un sensor ya creado al objeto Sensors*/
        void add(Sensor* sensor) {
            _map[sensor->model()].push_back(sensor);    
        }
        
        /*Convert Class Sensors to JSON
        =================================================================*/
        DynamicJsonDocument toJson() {
            DynamicJsonDocument doc(2048);
            JsonObject obj = doc.to<JsonObject>();
            for (const auto& entry : _map) {
                //JsonArray array = doc.createNestedArray(entry.first);
                JsonObject nestedObj = doc.createNestedObject(entry.first);
                for (Sensor* sensor : entry.second) {
                    //array.add(sensor.toJson());
                    nestedObj[sensor->id()] = sensor->toJson();
                }
            }
            return doc;
        }
};

class PWM{
    private:
        u_int8_t _pin;
        u_int8_t _channel;
        u_int8_t _resolution;
        String _id;
        String _label;  
        String _model; 
        String _topic;
        String _timestamp;   /*ISO 8601 ("AAAA-MM-DDTHH:MM:SS")*/
        int _freq;
        float _value;  
        float _low;
        float _high; 
        
        int _range;     

    public:
        Status status;
        
        /*Constructor Class PWM
        ==========================================================================================*/ 
        PWM(u_int8_t pin, u_int8_t channel, int freq, u_int8_t resolution,String id, String topic): 
        _pin(pin), _channel(channel), _freq(freq), _resolution(resolution), _id(id), _topic(topic),
        _timestamp(DEFAULT_TIMESTAMP), status(), _high(PWM_MAX), _low(PWM_MIN)         
        {
            ledcSetup(channel,freq,resolution);
            ledcAttachPin(pin,channel);
            _range = pow(2,_resolution)-1;
        }

        /*GETTER atributos Class PWM
        =========================================*/
        const String& id() const{
            return _id;
        }
        float value() const{
            return _value;
        }
         const String& topic() const{
            return _topic;
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
        const String& timestamp() const{
            return _timestamp;
        }

        /*SETTER atributos Class PWM
        =========================================*/
        void setTimestamp(const String& timestamp){
            _timestamp = timestamp;
        }

        /*METHOD Class PWM
        ===========================================================*/
        void write(float value){
            //int range = pow(2,_resolution)-1;
            ledcWrite(_channel, map(value,_low,_high,0,_range));
            _value = value;
        }

        /*ARDUINO-JSON: Convert Class PWM to JSON
        ===========================================*/
        DynamicJsonDocument toJson(){
            DynamicJsonDocument doc(512);
            doc["id"] = id();
            doc["value"] = _value;
            doc["status"] = status.toJson();
            doc["topic"] = topic();
            doc["timestamp"] = _timestamp;
            return doc;
        }
};

class Device{
    private:
        String _id;
        String _name;
        
    public:
        Sensors sensors;
       
        /*Constructor Class Device*/
        /*======================================*/
        Device(String id): _id(id), _name(),
        sensors(sensors) {
          
        } 

        /*GETTER atributos privados Class Device*/
        /*====================================*/
        const String id() const{
            return _id;
        }
        const String name() const{
            return _name;
        }
        /*SETTER atributos privados Class Device*/
        /*====================================*/
        void setID(const String& id){
            _id = id;
        }
        void setName(const String& name){
            _name = name;
        }
        /*Convert all Class Sensors to Device*
        /*========================================*/
        DynamicJsonDocument toJson(){
            DynamicJsonDocument doc(2048);
            doc["id"] = _id;
            doc["name"] = _name;
            doc["sensors"] = sensors.toJson();

            Serial.println();
            //serializeJson(doc,Serial);
            serializeJsonPretty(doc,Serial);
            Serial.println();
            return doc;
            //return doc;
        } 
};
