# ESP32-IoT-Weather-Station-with-MQTT-Cloud-Integration

IoT Weather Station (ESP32 + BME280)
This project demonstrates a complete IoT weather station using an ESP32 microcontroller and a BME280 environmental sensor. Sensor data (temperature, humidity, and pressure) is read at regular intervals and transmitted to a cloud server over Wi-Fi using MQTT or HTTP protocols.

✨ Features
Sensor data acquisition via I2C using Bosch’s bme280_driver

MQTT publishing to broker (e.g., Mosquitto) or HTTP POST to REST API

JSON-formatted data packets with timestamp

TLS-secured communication (MQTTS/HTTPS)

OTA firmware updates support (optional)

FreeRTOS-based task scheduling

🧰 Tech Stack
ESP-IDF v5.x or Arduino-ESP32

BME280 sensor

MQTT (via esp-mqtt) or HTTP (via esp_http_client)

FreeRTOS

(Optional) Node-RED / ThingsBoard / Grafana for visualization
