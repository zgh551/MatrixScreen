#include <stdio.h>

#include "FastLED.h"
#include "ccs811.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_module.h"
#include "hdc1080.h"
#include "i2c_module.h"
#include "smartconfig.h"

#define NUM_LEDS 60
CRGB leds[NUM_LEDS];

static const char *TAG = "MatricScreen";

extern "C" void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    /*
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET,
           chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                         : "external");

    printf("Minimum free heap size: %d bytes\n",
           esp_get_minimum_free_heap_size());
    */

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(hdc1080_init());
    ESP_LOGI(TAG, "hdc1080 initialized successfully");

    // init gpio for output pin: nReset and nWake
    gpio_init();
    // init gpio for data ready interrupt
    interrupt_init();
    // ESP_ERROR_CHECK(ccs811_init());
    // ESP_LOGI(TAG, "ccs811 initialized successfully");
    /*
    FastLED.addLeds<WS2812B, 19>(leds, NUM_LEDS);
    leds[0] = CRGB::White;
    FastLED.show();
    delay(30);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(30);*/

    init_smart_config();
    // Task Begin Scheduler
    // vTaskStartScheduler();
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
