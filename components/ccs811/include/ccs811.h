#ifndef __CCS811_H__
#define __CCS811_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../i2c_module/include/i2c_module.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// CCS811 ADDR pin is logic zero otherwise address would be 0x5B
#define CCS811_SENSOR_ADDR 0x5A /*!< slave address for CCS811 sensor */

/**
 * Define the ccs811 register address:
 */
#define STATUS 0x00
#define MEAS_MODE 0x01
#define ALG_RESULT_DATA 0x02
#define RAW_DATA 0x03
#define ENV_DATA 0x05
#define THRESHOLDS 0x10
#define BASELINE 0x11
#define HW_ID 0x20
#define HW_VERSION 0x21
#define FW_BOOT_VERSION 0x23
#define FW_APP_VERSION 0x24
#define INTERNAL_STATE 0xA0
#define ERROR_ID 0xE0
#define APP_ERASE 0xF1
#define APP_DATA 0xF2
#define APP_VERIFY 0xF3
#define APP_START 0xF4
#define SW_RESET 0xFF

/**
 * Coonfigure measurement mode register
 */
#define DRIVE_MODE_IDLE 0x0
#define DRIVE_MODE_1SEC 0x10
#define DRIVE_MODE_10SEC 0x20
#define DRIVE_MODE_60SEC 0x30
#define INTERRUPT_DRIVEN 0x8
#define THRESHOLDS_ENABLED 0x4

typedef struct _CCS811_SensorPacket {
    uint16_t TVOC;
    uint16_t CO2;
} CCS811_SensorPacket;

extern QueueHandle_t CCS811_SensorQueue;

static uint8_t HardwareID                  = 0;
static uint8_t HardwareVersion             = 0;
static uint16_t FirmwareBootVersion        = 0;
static uint16_t FirmwareApplicationVersion = 0;

void ccs811_task_init();

void func(void);

#endif
