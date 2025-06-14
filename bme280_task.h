#ifndef BME280_TASK_H
#define BME280_TASK_H

/**
 * FreeRTOS task to read data from the BME280 sensor and publish via MQTT.
 * @param pvParameters Task parameters (not used).
 */
void bme280_task(void *pvParameters);

int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t delay_msec(uint32_t period, void *intf_ptr);

#endif // BME280_TASK_H