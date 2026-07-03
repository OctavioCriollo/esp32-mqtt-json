#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector>

class MQTT{
private:
    const char* _server;
    uint16_t _port;
    const char* _user;
    const char* _password;
    const char* _id;
    const char* _topicPUB;
    const char* _topicSUB;
    size_t _bufferSize;

public:    
    PubSubClient client;

    /*CONSTRUCTOR Class MQTT
    ===============================================*/
    MQTT(WiFiClientSecure& WIFIClient): client(WIFIClient), _bufferSize(2048), _id("default") {
        client.setCallback([this](char* topic, byte* payload, unsigned int length) {
            this->callback(topic, payload, length);
        });
        setBufferSize(_bufferSize);
    }

    /*GETTER atributos Class MQTT
    ===============================================*/
    const char* server() const{
        return _server;
    }
    uint16_t port() const{
        return _port;
    }
    const char* user() const{
        return _user;
    }
    const char* password() const{
        return _password;
    }
    const char* id() const{
        return _id;
    }
    const char* topicPUB() const{
        return _topicPUB;
    }
    const char* topicSUB() const{
        return _topicSUB;
    }
    size_t bufferSize() const{
        return _bufferSize;
    }

    /*SETTER atributos Class MQTT
    ===============================================*/
    void setServer(const char* server){
        _server = server;
    }
    void setPort(uint16_t port){
        _port = port;
    }
    void setUser(const char* user){
        _user = user;
    }
    void setPassword(const char* password){
        _password = password;
    }
    void setId(const char* id){
        _id = id;
    }
    void setTopicPUB(const char* topicPUB){
        _topicPUB = topicPUB;
    }
    void setTopicSUB(const char* topicSUB){
        _topicSUB = topicSUB;
    }
    void setBufferSize(size_t bufferSize){
        _bufferSize = bufferSize;           /*Update BEFORE applying: the old
        order applied the previous value to the client (off-by-one-call bug).*/
        client.setBufferSize(_bufferSize);
    }
    
    bool publish(bool event, JsonDocument doc){
        if(!event)
            return false;
        std::vector<char> buffer(_bufferSize);   /*heap, not a 2KB stack VLA*/
        size_t n = serializeJson(doc,buffer.data(),buffer.size());
        client.publish(_topicPUB,(const uint8_t*)buffer.data(),n,false);

        Serial.printf("\nSENT MESSAGE:");
        Serial.printf("\nBROKER: %s",_server);
        Serial.printf("\nTOPIC: %s\n",_topicPUB);
        serializeJson(doc,Serial);
        Serial.println();

        return true;
    }
    void subscribe(){
        client.subscribe(_topicSUB);
    }
    bool connect(u_int8_t attempt){
        delay(200);
        Serial.print("\nConnecting to MQTT broker");
        client.setServer(_server,_port);
        for (int i=1;i<=attempt;i++){
            Serial.print("...");
            if(client.connect(_id,_user,_password)){
                break;
            }
        }
        if(!client.connected()){
            Serial.printf("\nNO CONNECTED!, Error: %d\n",client.state()); 
            return false;
        }
        Serial.println(" SUSSCEFUL!!!");
        Serial.printf("* Broker: %s\n",_server);
        Serial.printf("* Port: %d\n",_port);
        Serial.printf("* MQTT User: %s\n",_user);
        Serial.printf("* MQTT ID: %s\n",_id);
        Serial.println();
        subscribe();
        return true;
    }
    bool checkConnection(u_int8_t attempt){  
        if(client.connected())
            return true;
        return connect(attempt);
    }
    void callback(char* topic, byte* payload, unsigned int length) {
        Serial.printf("\nRECEIVED MESSAGE:");
        Serial.printf("\nBROKER: %s",_server);
        Serial.printf("\nTOPIC: %s\n",topic);
        JsonDocument doc;
        deserializeJson(doc,(const byte*)payload,length);
        serializeJson(doc,Serial);
        Serial.println();
    }
};

/*publicar un mensaje JSON en un servidor MQTT utilizando la biblioteca PubSubClient de Arduino. La publicacion es por evento. 
  Dado que esta publicando un JSON la funcion interna MQTTClient.publish tiene limitacion en capacidad por lo que se debe dar mayor 
  tamaño al buffer de MQTT. Le damos tambien el mismo tamaño a buffer que es donde se almacenara el JSON y luego se publicara este buffer.
===========================================================================================================================================*/
bool mqtt_publish(bool event, JsonDocument doc, PubSubClient& MQTTClient, size_t mqttBufferSize, const char* mqtt_topic_pub){
    if(!event)
        return false;
    char buffer[mqttBufferSize];
    MQTTClient.setBufferSize(mqttBufferSize);
    size_t n = serializeJson(doc,buffer,sizeof(buffer));
    MQTTClient.publish(mqtt_topic_pub,buffer,n);
    return true;
}

bool mqtt_connect(PubSubClient& MQTTClient, String mqttClientID, const char* mqttServer, uint16_t mqttPort, const char* mqttUser, const char* mqttPass, u_int8_t attempt){
    delay(200);
    Serial.print("\nConnecting to MQTT broker");
    MQTTClient.setServer(mqttServer,mqttPort);

    mqttClientID += "(" + String(WiFi.macAddress()) + ")";
    for (int i=1; i<=attempt; i++){
        Serial.print(".");
        if(MQTTClient.connect(mqttClientID.c_str(),mqttUser,mqttPass)){
            break;
        }
    }
    if(!MQTTClient.connected()){
        Serial.printf("\nNO CONNECTED!, Error: %d\n",MQTTClient.state()); 
        return false;
    }
    Serial.println(" SUSSCEFUL!!!");
    Serial.printf("* Broker: %s\n",mqttServer);
    Serial.printf("* Port: %d\n",mqttPort);
    Serial.printf("* MQTT User: %s\n",mqttUser);
    Serial.printf("* MQTT Pass: %s\n",mqttPass);
    Serial.printf("* MQTT ID: %s\n",mqttClientID.c_str());
    Serial.println();
    return true;
}

bool mqtt_check_connection(PubSubClient& MQTTClient, String mqttClientID, const char* mqttServer, uint16_t mqttPort, const char* mqttUser, const char* mqttPass, int attempt){  
    if(MQTTClient.connected())
        return true;
    return mqtt_connect(MQTTClient,mqttClientID,mqttServer,mqttPort,mqttUser,mqttPass,1);
}
