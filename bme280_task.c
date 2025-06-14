#include <stdio.h>              // standard I/O for printf
#include <driver/i2c.h>         // I2C driver for ESP-IDF I2C communication
#include <freertos/FreeRTOS.h>  // FreeRTOS headers for task management
#include <freertos/task.h>
#include "bme280.h"             // BME280 library for sensor operations
#include "mqtt.h"               // MQTT header for publishing data
#include "bme280_task.h"        // BME280 task header for function declarations
#include "esp_log.h"            // ESP-IDF logging library
#include "esp_sleep.h"          // ESP-IDF sleep library for deep sleep functionality

// Define I2C pin and configuration constants
#define I2C_MASTER_SCL_IO 22          // GPIO pin for I2C SCL
#define I2C_MASTER_SDA_IO 21          // GPIO pin for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number
#define I2C_MASTER_FREQ_HZ 100000     // I2C clock frequency (100 kHz)

// Declare static BME280 device structure for sensor operations
static struct bme280_dev bme280;

// Initialize I2C interface for BME280 sensor communication
static void i2c_init(void) {
    // Configure I2C master parameters
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,              // Set I2C to master mode
        .sda_io_num = I2C_MASTER_SDA_IO,      // Assign SDA pin
        .scl_io_num = I2C_MASTER_SCL_IO,      // Assign SCL pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up for SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Set clock speed
    };
    // Apply I2C configuration
    i2c_param_config(I2C_MASTER_NUM, &conf);
    // Install I2C driver (no RX/TX buffers, no interrupt flags)
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// I2C read function for BME280 sensor
// Parameters: reg_addr (register to read), reg_data (buffer for data), len (data length), intf_ptr (device address)
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Check for NULL pointers
    if (intf_ptr == NULL || reg_data == NULL) {
        return BME280_E_NULL_PTR; // Return error for invalid pointers
    }

    // Get device address from interface pointer
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    // Create I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) return BME280_E_COMM_FAIL; // Return error if link creation fails

    // Build I2C read sequence
    i2c_master_start(cmd); // Send start condition
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true); // Write device address (write mode)
    i2c_master_write_byte(cmd, reg_addr, true); // Write register address
    i2c_master_start(cmd); // Send repeated start
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true); // Write device address (read mode)
    i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK); // Read data with NACK for last byte
    i2c_master_stop(cmd); // Send stop condition

    // Execute I2C command with 1000ms timeout
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    // Free command link
    i2c_cmd_link_delete(cmd);
    // Return success or communication failure
    return (ret == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

// I2C write function for BME280 sensor
// Parameters: reg_addr (register to write), reg_data (data to write), len (data length), intf_ptr (device address)
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Check for NULL pointers
    if (intf_ptr == NULL || reg_data == NULL) {
        return BME280_E_NULL_PTR; // Return error for invalid pointers
    }

    // Get device address from interface pointer
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    // Create I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) return BME280_E_COMM_FAIL; // Return error if link creation fails

    // Build I2C write sequence
    i2c_master_start(cmd); // Send start condition
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true); // Write device address (write mode)
    i2c_master_write_byte(cmd, reg_addr, true); // Write register address
    i2c_master_write(cmd, reg_data, len, true); // Write data
    i2c_master_stop(cmd); // Send stop condition

    // Execute I2C command with 1000ms timeout
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    // Free command link
    i2c_cmd_link_delete(cmd);
    // Return success or communication failure
    return (ret == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}

// Delay function for BME280 sensor
// Parameters: period (delay in milliseconds), intf_ptr (unused)
int8_t delay_msec(uint32_t period, void *intf_ptr) {
    // Convert milliseconds to FreeRTOS ticks and delay
    vTaskDelay(period / portTICK_PERIOD_MS);
    return 0; // Return success
}

// BME280 sensor task to read data, publish to MQTT, and enter deep sleep
// Parameters: pvParameters (task parameters, unused)
void bme280_task(void *pvParameters) {
    // Initialize I2C interface for sensor communication
    i2c_init();
    
    // Define BME280 I2C address (0x76)
    static uint8_t bme280_address = BME280_I2C_ADDR_PRIM; // Primary I2C address
    
    // Configure BME280 interface settings
    bme280.intf = BME280_I2C_INTF;       // Set I2C interface
    bme280.read = i2c_read;               // Assign read function
    bme280.write = i2c_write;             // Assign write function
    bme280.delay_ms = delay_msec;         // Assign delay function
    bme280.intf_ptr = &bme280_address;    // Set device address pointer

    // Log BME280 interface details for debugging
    ESP_LOGI("BME280", "intf_ptr: %p, addr: 0x%02X", bme280.intf_ptr, *(uint8_t *)bme280.intf_ptr);
    ESP_LOGI("BME280", "Calling bme280_init...");
    ESP_LOGI("BME280", "read=%p, write=%p, delay=%p",
             bme280.read, bme280.write, bme280.delay_ms);
    
    // Initialize BME280 sensor
    int8_t rslt = bme280_init(&bme280);
    
    // Check initialization result
    if (rslt != BME280_OK) {
        // Log and print error if initialization fails
        printf("BME280 initialization failed: %d\n", rslt);
        vTaskDelete(NULL); // Terminate task on failure
    } else {
        // Print success message
        printf("BME280 initialization successful\n");
    }

    // Configure BME280 sensor settings
    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;      // Humidity oversampling (1x)
    settings.osr_p = BME280_OVERSAMPLING_1X;      // Pressure oversampling (1x)
    settings.osr_t = BME280_OVERSAMPLING_1X;      // Temperature oversampling (1x)
    settings.filter = BME280_FILTER_COEFF_2;       // Filter coefficient
    settings.standby_time = BME280_STANDBY_TIME_1000_MS; // Standby time (1000ms)
    
    // Select settings to apply
    uint8_t desired_settings = BME280_SEL_OSR_PRESS |
                              BME280_SEL_OSR_TEMP |
                              BME280_SEL_OSR_HUM  |
                              BME280_SEL_FILTER |
                              BME280_SEL_STANDBY;

    // Apply sensor settings
    rslt = bme280_set_sensor_settings(desired_settings, &settings, &bme280);
    
    // Set sensor to normal mode for continuous measurements
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280);
    // Wait 1 second for measurement to complete
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Structure to store sensor data
    struct bme280_data comp_data;
    // Read temperature, humidity, and pressure
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280);
    if (rslt == BME280_OK) {
        // Log sensor data to console with units
        ESP_LOGI("BME280", "Temperature: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa",
                 comp_data.temperature, comp_data.humidity, comp_data.pressure / 100.0);
        
        // Format sensor data as JSON with units for MQTT
        char payload[128];
        snprintf(payload, sizeof(payload), "{\"Temperature\":\"%.2f C\",\"Humidity\":\"%.2f %%\",\"Pressure\":\"%.2f hPa\"}",
                 comp_data.temperature, comp_data.humidity, comp_data.pressure / 100.0);
        // Publish data to MQTT topic "weather/data"
        mqtt_publish("weather/data", payload);
    } else {
        // Log error if sensor data read fails
        ESP_LOGE("BME280", "Failed to read sensor data: %d", rslt);
    }
    
    // Configure deep sleep to wake up after 60 seconds
    esp_sleep_enable_timer_wakeup(60 * 1000000); // Time in microseconds
    // Log deep sleep entry
    ESP_LOGI("BME280", "Entering deep sleep...");
    // Enter deep sleep, resetting the ESP32 on wake-up
    esp_deep_sleep_start();
}