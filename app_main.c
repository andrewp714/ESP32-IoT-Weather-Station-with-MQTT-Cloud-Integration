#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_sleep.h"
#include "wifi.h"
#include "mqtt.h"
#include "bme280_task.h"

void app_main(void) {
    // Configure wake-up sources
    esp_sleep_enable_timer_wakeup(60 * 1000000);

    // Initialize Wi-Fi
    wifi_init();
    
    // Initialize MQTT
    mqtt_init();
    
    // Start BME280 task
    xTaskCreate(bme280_task, "bme280_task", 8192, NULL, 5, NULL);
}