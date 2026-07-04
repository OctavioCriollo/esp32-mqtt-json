# ESP32-MQTT-JSON: JSON and MQTT, FAN Controller with PWM
ESP32 device send data monitoring in JSON format to MQTT server

## Description
This project uses an ESP32 to control fans based on the temperature measured by a DS18B20 sensor. It connects to a WiFi network and uses MQTT for communication with a MQTT server or broker. The ESP32 send to the broker the data in JSON format.

![System Diagram](images/fan-controller-power-plant.png)

## v6.0 Modernization (July 2026)

The firmware was modernized end to end. Highlights:

- **Platform:** [pioarduino](https://github.com/pioarduino/platform-espressif32) (Arduino core 3.x / ESP-IDF 5.x) — the espressif32 PlatformIO platform is frozen upstream.
- **Security:** credentials moved out of source into `include/secrets.h` (gitignored; copy `include/secrets.h.example`). Real TLS validation of the MQTT broker against ISRG Root X1 (`include/ca_cert.h`).
- **Robustness:** fan control runs in a dedicated FreeRTOS task decoupled from networking — WiFi/MQTT outages can no longer stall thermal regulation. Hardware watchdog (30 s). Degraded boot: a failed DS18B20 no longer hangs the device (fans failsafe to 100%, alarm reported over MQTT).
- **Web portal:** `http://fan-controller.local` — live status, runtime configuration (WiFi/MQTT credentials, temperature thresholds; stored in NVS, no recompile needed) and **browser OTA** firmware updates. HTTP Basic auth (`admin` / configurable password).
- **AP rescue mode:** if WiFi association fails at boot, the device raises the `FanController-Setup` access point with the portal at `192.168.4.1`.
- **CI/CD:** every push compiles the firmware on GitHub Actions; pushing a `v*` tag publishes a GitHub Release with the versioned `.bin` (flash it via the OTA page).

### Quickstart

```bash
cp include/secrets.h.example include/secrets.h   # fill in your values
pio run -t upload                                # first flash over USB
# subsequent updates: portal -> Firmware OTA -> upload the CI-built .bin
```

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
- `ArduinoJson` (v7) for JSON serialization and deserialization.
- `PubSubClient` for MQTT communication (TLS via `WiFiClientSecure` + CA validation).
- `ESPAsyncWebServer` + `AsyncTCP` (ESP32Async forks) for the web portal.
- `Preferences` (NVS) for persistent runtime configuration.
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
