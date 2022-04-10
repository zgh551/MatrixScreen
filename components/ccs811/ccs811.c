#include "ccs811.h"

#include <stdio.h>

#include "../../gpio_module/include/gpio_module.h"
#include "../../hdc1080/include/hdc1080.h"
#include "../../i2c_module/include/i2c_module.h"
static const char *TAG = "CCS811";

static CCS811WorkState ccs811_work_state_ = IdleMode;
static HDC1080_SensorPacket HDC1080SensorPacket_;

SemaphoreHandle_t xBinarySemaphoreSample;
QueueHandle_t xHDC1080Queue;
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
 * @brief Read two byte from a CCS811 sensor registers
 */
static esp_err_t ccs811_register_read_hword(uint8_t reg_addr, uint16_t *data)
{
    int ret;
    uint8_t read_buf[2];
    ret = i2c_master_write_read_device(
        I2C_MASTER_NUM, CCS811_SENSOR_ADDR, &reg_addr, 1, read_buf,
        sizeof(read_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret == ESP_OK) {
        *data = (uint16_t)((read_buf[0] << 8) | read_buf[1]);
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
    return (reg_status & 0x80) >> 7;
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
    return (reg_status & 0x10) >> 4;
}

uint8_t getDataReady(void)
{
    uint8_t reg_status;
    ccs811_register_read_byte(STATUS, &reg_status);
    return (reg_status & 0x08) >> 3;
}

static void isrDataSampleTask(void *arg)
{
    ESP_LOGI(TAG, "ISR Interrupt For Sample Data");
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "CCS811 Interrupt GPIO[%d], val:%d", io_num,
                     gpio_get_level(io_num));
            xSemaphoreGive(xBinarySemaphoreSample);
        }
    }
}

static void ccs811_task(void *arg)
{
    uint8_t mode;
    ccs811_work_state_ = ConstantPowerMode;
    for (;;) {
        switch (ccs811_work_state_) {
            case IdleMode:
                ESP_ERROR_CHECK(
                    ccs811_register_write_byte(MEAS_MODE, DRIVE_MODE_IDLE));
                break;

            case ConstantPowerMode:  // IAQ measurement every second
                mode = DRIVE_MODE_1SEC | INTERRUPT_DRIVEN;
                ESP_ERROR_CHECK(ccs811_register_write_byte(MEAS_MODE, mode));
                ccs811_work_state_ = EnvironmentalCompensation;
                break;

            case PulseHeatingMode:  // IAQ measurement every 10 seconds
                mode = DRIVE_MODE_10SEC | INTERRUPT_DRIVEN;
                ESP_ERROR_CHECK(ccs811_register_write_byte(MEAS_MODE, mode));
                ccs811_work_state_ = EnvironmentalCompensation;
                break;

            case LowPowerPulseHeatingMode:  // IAQ measurement every 60 seconds
                mode = DRIVE_MODE_60SEC | INTERRUPT_DRIVEN;
                ESP_ERROR_CHECK(ccs811_register_write_byte(MEAS_MODE, mode));
                ccs811_work_state_ = EnvironmentalCompensation;
                break;

            case ConstantPowerRawDataMode:  // sensor measurement every 250ms
                mode = DRIVE_MODE_250MS | INTERRUPT_DRIVEN;
                ESP_ERROR_CHECK(ccs811_register_write_byte(MEAS_MODE, mode));
                ccs811_work_state_ = EnvironmentalCompensation;
                break;

            case EnvironmentalCompensation:
                if (xQueueReceive(xHDC1080Queue, &HDC1080SensorPacket_,
                                  portMAX_DELAY) == pdPASS) {
                    ESP_LOGI(TAG,
                             "HDC_1080 Trmperature:%02f, Humidity:%02f[RH]",
                             HDC1080SensorPacket_.Temperature,
                             HDC1080SensorPacket_.Humidity);
                    // TODO write into env_data register
                }
                break;
            default:

                break;
        }
    }
}

esp_err_t ccs811_init(void)
{
    // Power On delay 20ms for i2c transform
    CCS811_Reset(0);                   // enable reset single
    CCS811_Wake(1);                    // set hight level for idle mode
    vTaskDelay(1 / portTICK_RATE_MS);  // reset single keep low at leat 15us
    CCS811_Reset(1);                   // disable reset single
    CCS811_Wake(0);                    // set to low level for wake up chip
    vTaskDelay(2 / portTICK_RATE_MS);  // wait at least 2ms for i2c operation

    uint8_t hardware_version;
    uint8_t hardware_id;
    uint16_t firmware_bootloader_version;
    uint16_t firmware_application_version;

    ESP_ERROR_CHECK(ccs811_register_read_byte(HW_ID, &hardware_id));
    ESP_LOGI(TAG, "CCS811 Hardware ID:%d", hardware_id);
    ESP_ERROR_CHECK(ccs811_register_read_byte(HW_VERSION, &hardware_version));
    ESP_LOGI(TAG, "CCS811 Hardware Version:%d", hardware_version);
    ESP_ERROR_CHECK(ccs811_register_read_hword(FW_BOOT_VERSION,
                                               &firmware_bootloader_version));
    ESP_LOGI(TAG, "CCS811 Firmware Botloader Version:v%d.%d.%d",
             (firmware_bootloader_version >> 12) & 0x0f,
             (firmware_bootloader_version >> 8) & 0x0f,
             firmware_bootloader_version & 0xff);
    ESP_ERROR_CHECK(ccs811_register_read_hword(FW_APP_VERSION,
                                               &firmware_application_version));
    ESP_LOGI(TAG, "CCS811 Firmware Application Version:v%d.%d.%d",
             (firmware_application_version >> 12) & 0x0f,
             (firmware_application_version >> 8) & 0x0f,
             firmware_application_version & 0xff);
    if (0x81 == hardware_id) {
        if (1 == getAppValid()) {
            // wait at least 20ms for i2c operation
            vTaskDelay(20 / portTICK_RATE_MS);
            // Start Application from Boot mode
            ESP_ERROR_CHECK(ccs811_register_write_none(APP_START));
            // wait at least 1ms for i2c operation
            vTaskDelay(1 / portTICK_RATE_MS);
            // judge whether in application mode
            if (1 == getFirmwareMode()) {
                // init gpio for output pin: nReset and nWake
                gpio_init();
                // init gpio for data ready interrupt
                interrupt_init();
                // Start GPIO Interrupt Task
                xTaskCreate(isrDataSampleTask, "ccs811 interrupt task", 2048,
                            NULL, 10, NULL);
                // Create task for ccs811 state machine process
                xTaskCreate(ccs811_task, "ccs811 task", 2048, NULL, 10, NULL);
                return ESP_OK;
            }
            else {
                ESP_LOGE(TAG, "Firmware is in boot mode");
                return ESP_FAIL;
            }
        }
        else {
            ESP_LOGE(TAG, "No application firmware loaded");
            return ESP_FAIL;
        }
    }
    else {
        ESP_LOGE(TAG, "CCS811 Invalid Hardware ID:%d", hardware_id);
        return ESP_FAIL;
    }
}
