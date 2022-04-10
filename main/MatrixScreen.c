#include <stdio.h>

#include "ccs811.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hdc1080.h"
#include "i2c_module.h"

static const char *TAG = "MatricScreen";
void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET,
           chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                         : "external");

    printf("Minimum free heap size: %d bytes\n",
           esp_get_minimum_free_heap_size());

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(hdc1080_init());
    ESP_LOGI(TAG, "hdc1080 initialized successfully");
    ESP_ERROR_CHECK(ccs811_init());
    ESP_LOGI(TAG, "ccs811 initialized successfully");

    // Task Begin Scheduler
    vTaskStartScheduler();
    while (1) {
    }
}
