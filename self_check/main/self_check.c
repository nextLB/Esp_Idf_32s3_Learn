#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "SYS_MONITOR";

/* 监测周期，单位毫秒；可根据需要改成 1000、5000 等 */
#define MONITOR_INTERVAL_MS 3000

static void print_chip_info_once(void)
{
    esp_chip_info_t chip_info;
    uint32_t flash_size;

    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "========== CHIP INFO (read once) ==========");
    ESP_LOGI(TAG, "Chip target : %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Cores       : %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon rev : v%d.%d",
             chip_info.revision / 100, chip_info.revision % 100);

    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash size  : %" PRIu32 " MB", flash_size / (1024 * 1024));
    } else {
        ESP_LOGW(TAG, "Flash size  : read failed");
    }

    ESP_LOGI(TAG, "Features    : %s%s%s%s",
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
             (chip_info.features & CHIP_FEATURE_BT)       ? "BT "   : "",
             (chip_info.features & CHIP_FEATURE_BLE)      ? "BLE "  : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? "802.15.4" : "");
    ESP_LOGI(TAG, "==========================================");
}

static void print_runtime_status(void)
{
    /* 系统运行时间 */
    int64_t uptime_us = esp_timer_get_time();
    uint32_t uptime_s = (uint32_t)(uptime_us / 1000000);

    /* 内部 RAM 状态 */
    size_t internal_free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    /* PSRAM 状态（如果板子带 PSRAM 且已启用） */
    size_t spiram_free  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t spiram_block = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

    /* 当前任务栈剩余（高水位线） */
    UBaseType_t stack_left = uxTaskGetStackHighWaterMark(NULL);

    ESP_LOGI(TAG, "---------- RUNTIME STATUS ----------");
    ESP_LOGI(TAG, "Uptime              : %" PRIu32 " s", uptime_s);
    ESP_LOGI(TAG, "Internal RAM free   : %u B (%.1f KB)",
             (unsigned)internal_free, (float)internal_free / 1024.0f);
    ESP_LOGI(TAG, "Internal largest blk: %u B (%.1f KB)",
             (unsigned)internal_block, (float)internal_block / 1024.0f);
    ESP_LOGI(TAG, "PSRAM free          : %u B (%.1f KB)",
             (unsigned)spiram_free, (float)spiram_free / 1024.0f);
    ESP_LOGI(TAG, "PSRAM largest blk   : %u B (%.1f KB)",
             (unsigned)spiram_block, (float)spiram_block / 1024.0f);
    ESP_LOGI(TAG, "Current task stack  : %u B left (high-water mark)",
             (unsigned)stack_left);
    ESP_LOGI(TAG, "------------------------------------");
}

void app_main(void)
{
    print_chip_info_once();

    while (1) {
        print_runtime_status();
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}