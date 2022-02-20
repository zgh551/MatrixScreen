#include "hdc1080.h"

#include <stdio.h>

static const char *TAG = "HDC1080";

/**
 * @brief Write half word a HDC1080 sensor register
 */
static esp_err_t hdc1080_register_write(uint8_t reg_addr, uint16_t data)
{
    int ret;
    uint8_t write_buf[3] = {reg_addr, (uint8_t)((data >> 8) & 0xff),
                            (uint8_t)(data & 0xff)};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, HDC1080_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief Read a sequence of bytes from a HDC1080 sensor registers
 */
static esp_err_t hdc1080_register_read(uint8_t reg_addr, uint16_t *data)
{
    int ret;
    uint8_t read_buf[2];
    ret = i2c_master_write_read_device(
        I2C_MASTER_NUM, HDC1080_SENSOR_ADDR, &reg_addr, 1, read_buf,
        sizeof(read_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data = (uint16_t)((read_buf[0] << 8) | read_buf[1]);
    }
    return ret;
}

/**
 * @brief Read a sequence of bytes from a HDC1080 sensor registers
 */
static esp_err_t hdc1080_data_read_one(uint16_t *data)
{
    int ret;
    uint8_t read_buf[2];
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, HDC1080_SENSOR_ADDR,
                                      read_buf, sizeof(read_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data = (uint16_t)((read_buf[0] << 8) | read_buf[1]);
    }
    return ret;
}

/**
 * @brief Read a sequence of bytes from a HDC1080 sensor registers
 */
static esp_err_t hdc1080_data_read_two(uint16_t *data1, uint16_t *data2)
{
    int ret;
    uint8_t read_buf[4];
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, HDC1080_SENSOR_ADDR,
                                      read_buf, sizeof(read_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data1 = (uint16_t)((read_buf[0] << 8) | read_buf[1]);
        *data2 = (uint16_t)((read_buf[2] << 8) | read_buf[3]);
    }
    return ret;
}

static void hdc1080_init(void)
{
    uint16_t device_id;
    ESP_ERROR_CHECK(hdc1080_register_read(DEVICE_ID, &device_id));
    ESP_LOGI(TAG, "HDC1080 Device ID:%X", device_id);
}

void hdc1080_task(void)
{
    hdc1080_init();
    while (1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        ESP_LOGI(TAG, "HDC_1080 Task");
    }
}
