#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

bool wifi_connect(const char* ssid, const char* password, int waitConnect){
    delay(200);
    Serial.print("\nConnecting to WiFi");
    WiFi.begin(ssid,password);
    for (int i=1; i<=waitConnect; i++){
        if (WiFi.status() == WL_CONNECTED)
            break;
        Serial.print(".");
        delay(1000);
    }
    if (WiFi.status() != WL_CONNECTED){
        Serial.println(" NO CONNECTED!!!"); 
        return false;
    }
    IPAddress ip = WiFi.localIP();
    Serial.println(" SUSSCEFUL!!!");
    Serial.printf("* SSID: %s\n",ssid);
    Serial.printf("* Pass: %s\n",password);
    Serial.printf("* IP:   %s\n",ip.toString().c_str());
    return true;
}

bool wifi_check_connection(const char* ssid, const char* password, int waitConnect){
    if(WiFi.isConnected()){
        Serial.println();
        Serial.print("Wifi OK");
        return true;
    }
    Serial.print("\nWifi DISCONNECTED");   
    return wifi_connect(ssid,password,waitConnect);
}
