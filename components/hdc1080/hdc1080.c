#include "hdc1080.h"

#include <stdio.h>

static const char *TAG = "HDC1080";
QueueHandle_t HDC1080Queue;

/**
 * @brief Write none a HDC1080 sensor register
 */
static esp_err_t hdc1080_register_write_none(uint8_t reg_addr)
{
    int ret;
    uint8_t write_buf[1] = {reg_addr};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, HDC1080_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    return ret;
}


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

static esp_err_t hdc1080_write_configuration_register(uint8_t rst, uint8_t heat, uint8_t mode, uint8_t t_res, uint8_t h_res)
{

    uint16_t configuration_reg= ((rst & 0x01) << 15) | ( (heat & 0x01) << 13) | ((mode & 0x01) << 12) | ((t_res & 0x01) << 10)|
        ((h_res & 0x03)<< 9);


        int ret = hdc1080_register_write(CONFIGURATION, configuration_reg);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "configurate finish!");
        }
        return ret;
}


static esp_err_t hdc1080_init(void)
{
    // step1: get chip id
    ESP_ERROR_CHECK(hdc1080_register_read(DEVICE_ID, &DeviceID));
    ESP_ERROR_CHECK(hdc1080_register_read(MANUFACTURER_ID, &ManufacturerID));
    ESP_LOGI(TAG, "HDC1080 Device ID: 0x%X", DeviceID);
    ESP_LOGI(TAG, "HDC1080 Manufacturer ID: 0x%X", ManufacturerID);
    if ( (DeviceID == 0x1050) && (ManufacturerID == 0x5449  ))
    {
        // step2: get serial id
        uint16_t id_f, id_m, id_l;
        ESP_ERROR_CHECK(hdc1080_register_read(SERIAL_ID_F, &id_f));
        ESP_ERROR_CHECK(hdc1080_register_read(SERIAL_ID_M, &id_m));
        ESP_ERROR_CHECK(hdc1080_register_read(SERIAL_ID_L, &id_l));
        SerialID = (uint64_t)((id_f << 25) | (id_m << 9) | (id_l >>7));
        ESP_LOGI(TAG, "HDC1080 Serial ID:%016llX", SerialID);

        // step3: configure the register
        ESP_ERROR_CHECK(hdc1080_write_configuration_register(NORMAL_OPERATION, HEATER_DISABLED, ACQUIRED_SEQUENCE, TEMP_RES_14BIT,  HUMD_RES_14BIT));


    HDC1080Queue = xQueueCreate(10,sizeof(HDC1080_SensorPacket));
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

void hdc1080_task(void)
{
    ESP_ERROR_CHECK(hdc1080_init());
    while (1) {


        ESP_ERROR_CHECK(hdc1080_register_write_none(TEMPERATURE));
        vTaskDelay(1000 / portTICK_RATE_MS);
sHDC1080SensorPacket
        ESP_LOGI(TAG, "HDC_1080 Task");
    }
}
