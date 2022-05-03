#ifndef __GPIO_MODULE_H__
#define __GPIO_MODULE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define CCS811_nRESET_IO CONFIG_nRESET_CCS811
#define CCS811_nWAKE_IO CONFIG_nWAKE_CCS811
#define CCS811_nINT_IO CONFIG_nINT_CCS811

#define GPIO_OUTPUT_PIN_SEL \
    ((1ULL << CCS811_nRESET_IO) | (1ULL << CCS811_nWAKE_IO))

#define GPIO_INPUT_PIN_SEL (1ULL << CCS811_nINT_IO)

// QueueHandle_t
extern QueueHandle_t gpio_evt_queue;

void gpio_init(void);
void interrupt_init(void);

void setWake(uint32_t v);
void setReset(uint32_t v);

int getWake(void);
int getReset(void);

#ifdef __cplusplus
}
#endif

#endif
