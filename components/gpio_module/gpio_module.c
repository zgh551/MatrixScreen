#include "gpio_module.h"

#include <stdio.h>

static const char* TAG = "GPIO_Module";

QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void CCS811_Wake(uint32_t v) { gpio_set_level(CCS811_nWAKE_IO, v); }
void CCS811_Reset(uint32_t v) { gpio_set_level(CCS811_nRESET_IO, v); }

void gpio_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
}

void interrupt_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // bit mask of the pins, use GPIO INT here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // change gpio intrrupt type for one pin
    // set falling edge single to trigger ccs811 interrupt for sample finish
    gpio_set_intr_type(CCS811_nINT_IO, GPIO_INTR_NEGEDGE);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(CCS811_nINT_IO, gpio_isr_handler,
                         (void*)CCS811_nINT_IO);

    ESP_LOGI(TAG, "Minimum free heap size: %d bytes\n",
             esp_get_minimum_free_heap_size());

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
}

