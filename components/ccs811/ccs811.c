#include "ccs811.h"

#include <stdio.h>

static const char *TAG = "CCS811";

/**
 * @brief Write empty data to CCS811 sensor register
 */
static esp_err_t ccs811_register_write_none(uint8_t reg_addr)
{
    int ret;
    uint8_t write_buf[1] = {reg_addr};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CCS811_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    return ret;
}

/**
 * @brief Write one byte to CCS811 sensor register
 */
static esp_err_t ccs811_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CCS811_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief Write half word a CCS811 sensor register
 */
static esp_err_t ccs811_register_write_hword(uint8_t reg_addr, uint16_t data)
{
    int ret;
    uint8_t write_buf[3] = {reg_addr, (uint8_t)((data >> 8) & 0xff),
                            (uint8_t)(data & 0xff)};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CCS811_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief Write one word to CCS811 sensor register
 */
static esp_err_t ccs811_register_write_word(uint8_t reg_addr, uint16_t data1,
                                            uint16_t data2)
{
    int ret;
    uint8_t write_buf[5] = {
        reg_addr, (uint8_t)((data1 >> 8) & 0xff), (uint8_t)(data1 & 0xff),
        (uint8_t)((data2 >> 8) & 0xff), (uint8_t)(data2 & 0xff)};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CCS811_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief Read one byte from a CCS811 sensor registers
 */
static esp_err_t ccs811_register_read_byte(uint8_t reg_addr, uint8_t *data)
{
    int ret;
    uint8_t read_buf[1];
    ret = i2c_master_write_read_device(
        I2C_MASTER_NUM, CCS811_SENSOR_ADDR, &reg_addr, 1, read_buf,
        sizeof(read_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data = read_buf[0];
    }
    return ret;
}

/**
 * @brief Read two bytes from a CCS811 sensor registers
 */
static esp_err_t ccs811_data_read_one(uint16_t *data)
{
    int ret;
    uint8_t read_buf[2];
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, CCS811_SENSOR_ADDR,
                                      read_buf, sizeof(read_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data = (uint16_t)((read_buf[0] << 8) | read_buf[1]);
    }
    return ret;
}

/**
 * @brief Read two data from CCS811 sensor registers
 */
static esp_err_t ccs811_data_read_two(uint16_t *data1, uint16_t *data2)
{
    int ret;
    uint8_t read_buf[4];
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, CCS811_SENSOR_ADDR,
                                      read_buf, sizeof(read_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data1 = (uint16_t)((read_buf[0] << 8) | read_buf[1]);
        *data2 = (uint16_t)((read_buf[2] << 8) | read_buf[3]);
    }
    return ret;
}

/********************** CCS811 Function ***********************/

/*
 * get status register
 */
uint8_t getFirmwareMode(void)
{
    uint8_t reg_status;
    ccs811_register_read_byte(STATUS, &reg_status);
    return (reg_status & 0x80);
}

uint8_t getErrorStatus(void)
{
    uint8_t reg_status;
    ccs811_register_read_byte(STATUS, &reg_status);
    return (reg_status & 0x01);
}

uint8_t getAppValid(void)
{
    uint8_t reg_status;
    ccs811_register_read_byte(STATUS, &reg_status);
    return (reg_status & 0x10);
}

uint8_t getDataReady(void)
{
    uint8_t reg_status;
    ccs811_register_read_byte(STATUS, &reg_status);
    return (reg_status & 0x08);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static esp_err_t ccs811_init(void)
{
    CCS811_Reset(0);
    CCS811_Wake(1);
    vTaskDelay(100 / portTICK_RATE_MS);
    CCS811_Reset(1);
    CCS811_Wake(0);

    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    return ESP_OK;
}
