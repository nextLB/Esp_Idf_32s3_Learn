#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "wifi_manager.h"
#include "udp_listener.h"

static const char *TAG = "APP";

/* 改成你自己的 WiFi 名称和密码 */
#define WIFI_SSID      "616"
#define WIFI_PASSWORD  "aa472888"

void app_main(void)
{
    ESP_LOGI(TAG, "开始连接 WiFi ...");

    wifi_conn_result_t wifi =
        wifi_connect_and_get_ips(0, WIFI_SSID, WIFI_PASSWORD);

    if (wifi.connected) {
        ESP_LOGI(TAG, "========== WiFi 连接成功 ==========");
        ESP_LOGI(TAG, "  局域网 IP : %s", wifi.local_ip);
        ESP_LOGI(TAG, "  公网 IP   : %s", wifi.public_ip);
        ESP_LOGI(TAG, "===================================");

        /* 连接成功后启动 UDP 监听，绑核 0 */
        udp_listener_start(0, 8080);
    } else {
        ESP_LOGE(TAG, "WiFi 连接失败");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}