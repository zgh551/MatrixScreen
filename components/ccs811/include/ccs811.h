#ifndef __CCS811_H__
#define __CCS811_H__

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
#define DRIVE_MODE_250MS 0x40
#define INTERRUPT_DRIVEN 0x08
#define THRESHOLDS_ENABLED 0x04
typedef enum _CCS811WorkState {
    IdleMode = 0,
    ConstantPowerMode,         // IAQ measurement every second
    PulseHeatingMode,          // IAQ measurement every 10 seconds
    LowPowerPulseHeatingMode,  // IAQ measurement every 60 seconds
    ConstantPowerRawDataMode,  // sensor measurement every 250ms
    EnvironmentalCompensation  // environmental compensation from hdc1080
} CCS811WorkState;

typedef struct _CCS811_SensorPacket {
    uint16_t TVOC;
    uint16_t CO2;
} CCS811_SensorPacket;

extern QueueHandle_t xCCS811_SensorQueue;

// static uint8_t HardwareID                  = 0;
// static uint8_t HardwareVersion             = 0;
// static uint16_t FirmwareBootVersion        = 0;
// static uint16_t FirmwareApplicationVersion = 0;

// static HDC1080_SensorPacket sHDC1080SensorPacket;
//  CCS811_State ccs811_state = Reset_CCS811;
esp_err_t ccs811_init(void);

#ifdef __cplusplus
}
#endif
#endif
