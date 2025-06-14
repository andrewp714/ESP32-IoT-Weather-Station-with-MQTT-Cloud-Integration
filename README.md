# ESP32-IoT-Weather-Station-with-MQTT-Cloud-Integration

## Overview
This project implements a low-power IoT weather station using the ESP32 microcontroller and Bosch BME280 sensor, developed with the ESP-IDF framework. The system collects temperature, humidity, and pressure data, publishes it to an MQTT broker (`broker.hivemq.com`) every 60 seconds, and enters deep sleep to minimize power consumption. Data is visualized using MQTT Explorer, making it ideal for real-time environmental monitoring. The project demonstrates advanced embedded systems skills, including FreeRTOS task management, I2C communication, MQTT protocol integration, and low-power design.

## Features
- **Sensor Integration**: Reads temperature, humidity, and pressure from the BME280 sensor via I2C.
- **MQTT Communication**: Publishes JSON-formatted data (`{\"Temperature\":\"%.2f C\",\"Humidity\":\"%.2f %%\",\"Pressure\":\"%.2f hPa\"}`) to the `weather/data` topic.
- **Low-Power Operation**: Utilizes ESP32 deep sleep to reduce power consumption to ~20µA between 60-second data cycles.
- **Wi-Fi Connectivity**: Connects to a Wi-Fi network for MQTT communication.
- **Error Handling**: Robust initialization and logging for Wi-Fi, MQTT, and BME280 operations.
- **Modular Design**: Organized into `main.c`, `wifi.c`, `mqtt.c`, and `bme280_task.c` for maintainability.

## Hardware Requirements
- ESP32 development board (e.g., ESP32-WROOM-32 with 4MB flash).
- Bosch BME280 sensor module.
- Breadboard and jumper wires for connections:
  - BME280 VCC to ESP32 3.3V
  - BME280 GND to ESP32 GND
  - BME280 SDA to ESP32 GPIO 21
  - BME280 SCL to ESP32 GPIO 22

## Software Requirements
- **ESP-IDF v5.4.1**: Framework for ESP32 development.
- **Visual Studio Code**: With ESP-IDF extension for building and flashing.
- **MQTT Explorer**: For visualizing MQTT data.
- **Bosch BME280 Driver**: Included in `components/bme280_driver`.

## Project Structure
```
mqtt_iot_weather_station/
├── main/
│   ├── main.c              # Entry point, initializes tasks and deep sleep
│   ├── wifi.c              # Wi-Fi connection setup
│   ├── wifi.h
│   ├── mqtt.c              # MQTT client configuration and publishing
│   ├── mqtt.h
│   ├── bme280_task.c       # BME280 sensor reading and MQTT publishing
│   ├── bme280_task.h
│   ├── CMakeLists.txt
├── components/
│   ├── bme280_driver/
│   │   ├── bme280.c        # Bosch BME280 driver
│   │   ├── bme280.h
│   │   ├── CMakeLists.txt
├── CMakeLists.txt          # Root CMake configuration
├── sdkconfig               # ESP-IDF configuration
├── README.md               # This file
```

## Setup Instructions
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/mqtt_iot_weather_station.git
   cd mqtt_iot_weather_station
   ```

2. **Install ESP-IDF**:
   - Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32/get-started/index.html) to install ESP-IDF v5.4.1.
   - Set up the environment:
     ```bash
     . $HOME/esp/v5.4.1/esp-idf/export.sh  # Linux/macOS
     C:\Users\andre\esp\v5.4.1\export.bat   # Windows
     ```

3. **Configure Wi-Fi and MQTT**:
   - Edit `wifi.c` to set your Wi-Fi credentials:
     ```c
     .ssid = "YOUR_SSID",
     .password = "YOUR_PASSWORD",
     ```
   - The default MQTT broker is `mqtt://broker.hivemq.com:1883`. Update `mqtt.c` if using a local broker (e.g., Mosquitto).

4. **Build and Flash**:
   - Connect the ESP32 to your computer via USB.
   - Build and flash the project:
     ```bash
     idf.py build flash monitor
     ```

5. **Verify with MQTT Explorer**:
   - Download and install [MQTT Explorer](https://mqtt-explorer.com/).
   - Connect to `mqtt://broker.hivemq.com:1883`.
   - Subscribe to the `weather/data` topic.
   - Expect one JSON message every 60 seconds, e.g.:
     ```json
     {"temperature":25.50,"humidity":60.20,"pressure":1013.25}
     ```

## Technical Challenges Overcome
- **BME280 Initialization Failure**: Resolved a "BME280 initialization failed: -1" error, when intf_ptr, read, write, and delay_ms were checked to not be NULL, but the problem persisted. A broken version of the Bosch BME280 driver was suspected to be used and fixed by correcting the delay_ms function in bme280.c file.
- **FreeRTOS Configuration**: Fixed the `CONFIG_FREERTOS_HZ` undefined error in `bme280_task.c` by including `<freertos/FreeRTOS.h>` and using `pdMS_TO_TICKS` for timing.
- **Guru Meditation Error**: A Guru Meditation Error: Core 1 panic'ed (InstrFetchProhibited) was triggered, which usually points to trying to execute a NULL or invalid function pointer.
- **I2C Driver Warning**: Upgraded from the deprecated `driver/i2c.h` to `driver/i2c_master.h`, improving I2C reliability.
- **Low-Power Design**: Implemented deep sleep in `main.c` and `bme280_task.c`, reducing power consumption from ~100mA to ~20µA during sleep.
- **Undefined Symbols and Missing Macros**: I2C_MASTER_WRITE was reported as undefined — indicating a missing or improperly configured I2C master setup in ESP-IDF.
