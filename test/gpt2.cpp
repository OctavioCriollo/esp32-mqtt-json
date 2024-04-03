#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Datos de la red WiFi
const char* ssid = "nombre_de_la_red_wifi";
const char* password = "contraseña_de_la_red_wifi";

// Datos del broker MQTT
const char* mqtt_server = "mosquitto.network-telemetrix.com";
const int mqtt_port = 8883;
const char* mqtt_user = "usuario_del_broker_mqtt";
const char* mqtt_password = "contraseña_del_broker_mqtt";
const char* mqtt_topic = "topic_para_los_datos";

// Objeto de la clase WiFiClient
WiFiClient wifiClient;

// Objeto de la clase PubSubClient
PubSubClient client(wifiClient);

void setup() {
  // Inicializar el puerto serial
  Serial.begin(9600);

  // Conectar a la red WiFi
  connectToWiFi();

  // Configurar el cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Configurar la seguridad del cliente MQTT
  client.setBufferSize(1024);
  client.setSecure(true);
  wifiClient.setCACert(root_ca);
  wifiClient.setCertificate(certificate);
  wifiClient.setPrivateKey(private_key);

  // Conectar al broker MQTT
  connectToMQTT();
}

void loop() {
  // Verificar la conexión WiFi y MQTT
  if (!isWiFiConnected()) {
    Serial.println("La conexión WiFi se perdió, intentando reconectar...");
    connectToWiFi();
    return;
  }

  if (!isMQTTConnected()) {
    Serial.println("La conexión MQTT se perdió, intentando reconectar...");
    connectToMQTT();
    return;
  }

  // Crear el objeto JSON con los datos de las señales
  StaticJsonDocument<200> doc;
  doc["temp"] = 25.0;
  doc["O2"] = 5.0;
  doc["pH"] = 7.0;
  doc["conductividad"] = 1000.0;
  doc["caudal"] = 10.0;

  // Convertir el objeto JSON a una cadena de caracteres
  char json[200];
  serializeJson(doc, json);

  // Publicar la cadena JSON en el topic MQTT
  client.publish(mqtt_topic, json);

  // Esperar un tiempo antes de volver a enviar los datos
  delay(5000);
}

// Función para conectar al broker MQTT
void connectToMQTT() {
  while (!client.connected()) {
    Serial.println("Conectando al broker MQTT...");
    if (client.connect("ESP32", mqtt_user, mqtt_password)) {
      Serial.println("Conexión al broker MQTT establecida.");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("Error al conectar al broker MQTT: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// Función para conectar a la red WiFi
void connectToWiFi(){
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a la red WiFi...");
  while (WiFi.status() != WL_CONNECTED)
    delay(1000);
}

