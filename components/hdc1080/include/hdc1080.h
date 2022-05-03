#ifndef __HDC_1080_H__
#define __HDC_1080_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#define HDC1080_SENSOR_ADDR (0x40) /*!< slave address for HDC1080 sensor */
/**
 * Define the hdc1080 register address:
 */
#define TEMPERATURE 0x00
#define HUMIDITY 0x01
#define CONFIGURATION 0x02

#define SERIAL_ID_F 0xFB
#define SERIAL_ID_M 0xFC
#define SERIAL_ID_L 0xFD
#define MANUFACTURER_ID 0xFE
#define DEVICE_ID 0xFF

// configure register
// RST
#define NORMAL_OPERATION (0)
#define SOFTWARE_RST (1)

// HEAT
#define HEATER_DISABLED (0)
#define HEATER_ENABLED (1)

// Mode
#define ACQUIRED_SINGLE (0)
#define ACQUIRED_SEQUENCE (1)

// Temperature Measurement Resolution
#define TEMP_RES_14BIT (0)
#define TEMP_RES_11BIT (1)

// Humidity Measurement Resolution
#define HUMD_RES_14BIT (0)
#define HUMD_RES_11BIT (1)
#define HUMD_RES_8BIT (2)

typedef struct _HDC1080_SensorPacket {
    float Temperature;
    float Humidity;
} HDC1080_SensorPacket;

extern QueueHandle_t xHDC1080Queue;
extern SemaphoreHandle_t xBinarySemaphoreSample;

static uint16_t DeviceID       = 0;
static uint16_t ManufacturerID = 0;
static uint64_t SerialID       = 0;

esp_err_t hdc1080_init(void);

#ifdef __cplusplus
}
#endif

#endif

