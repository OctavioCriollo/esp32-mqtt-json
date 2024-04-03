# ESP32-MQTT-JSON: JSON and MQTT, FAN Controller with PWM
ESP32 device send data monitoring in JSON format to MQTT server

## Description
This project uses an ESP32 to control fans based on the temperature measured by a DS18B20 sensor. It connects to a WiFi network and uses MQTT for communication with a MQTT server or broker. The ESP32 send to the broker the data in JSON format.

## Features
- WiFi connection.
- MQTT connection and communication.
- Temperature monitoring with DS18B20 sensor.
- Fan control using PWM signals.
- Cabinet state monitoring (door open/closed).
- Alarm generation for temperature and fan failures.
- Sending monitoring data and alarms via MQTT.

## Requirements
- ESP32
- DS18B20 temperature sensor
- Fans with PWM control
- Door state sensor (optional)

## Required Libraries
To use this code, you need to install the following libraries:
- `ArduinoJson` for JSON serialization and deserialization.
- `PubSubClient` for MQTT communication.
- `SPIFFS` for SPI Flash File System.
- Library files for specific sensors and actuators used in your project, such as:
  - `sensor-NT-02.h` for the temperature sensor.
  - `wireless-NT.h` for WiFi connection.
  - `mqtt-NT.h` for MQTT configuration.
  - `IoT-NT.h` for IoT device management.
  - Other library files as needed for your specific hardware setup.

## Configuration
1. Modify the `ssid` and `password` constants to set up the WiFi connection.
2. Adjust the MQTT connection parameters (`MQTT_SERVER`, `MQTT_PORT`, `MQTT_CLIENT_USER`, `MQTT_CLIENT_PASS`).
3. Set the temperature limits (`HIGH_TEMP` and `LOW_TEMP`).
4. (Optional) Configure the door state sensor and adjust the control logic as needed.

## Usage
Upload the code to an ESP32 and ensure it is connected to the WiFi network and MQTT broker. The device will monitor the temperature and control the fans automatically. Monitoring data and alarms will be sent to the MQTT broker.

## JSON Data Format
Monitoring data and alarms are sent to the MQTT broker in a JSON format structured as follows:

```json
{
  "device": "Controller",
  "type": "ESP_32",
  "location": "/power/climatizacion",
  "sensors": [
    {
      "name": "temp1",
      "type": "DS18B20",
      "temperature": 25,
      "status": "Temperature OK",
      "alarm": false
    }
  ],
  "actuators": [
    {
      "name": "fan1",
      "type": "PWM",
      "value": 100,
      "status": "FAN1 Working!",
      "alarm": false
    },
    {
      "name": "fan2",
      "type": "PWM",
      "value": 100,
      "status": "FAN2 Working!",
      "alarm": false
    },
    {
      "name": "doorOpenAlarm",
      "type": "PINCONTROL",
      "status": "Door is CLOSE",
      "alarm": false
    },
    {
      "name": "fanAlarm",
      "type": "PINCONTROL",
      "status": "FAN's is OK!",
      "alarm": false
    }
  ]
}

```

## IoT development board
Board on which the code was developed, you can watch the following video in Youtube:

[![Project Demo](https://img.youtube.com/vi/xHSDZ5ZZDWI/sddefault.jpg)](https://www.youtube.com/watch?v=xHSDZ5ZZDWI)

Click on the image above to watch the video.

## Contributions
Contributions to this project are welcome. Please open an issue or a pull request to suggest improvements or add features.
