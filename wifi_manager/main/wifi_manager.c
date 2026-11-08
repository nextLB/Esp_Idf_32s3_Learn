#include "wifi_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY          5
#define HTTP_RESP_BUF_SIZE 256

static const char *TAG = "wifi_mgr";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

/* ---------- WiFi 事件处理 ---------- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "重连中 %d/%d", s_retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ---------- 获取公网 IP（通过 httpbin.org/ip） ---------- */
typedef struct {
    char buf[HTTP_RESP_BUF_SIZE];
    int  len;
} http_resp_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_resp_ctx_t *ctx = (http_resp_ctx_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && ctx) {
        int copy = (evt->data_len < (HTTP_RESP_BUF_SIZE - ctx->len - 1))
                   ? evt->data_len : (HTTP_RESP_BUF_SIZE - ctx->len - 1);
        if (copy > 0) {
            memcpy(ctx->buf + ctx->len, evt->data, copy);
            ctx->len += copy;
            ctx->buf[ctx->len] = '\0';
        }
    }
    return ESP_OK;
}

static bool fetch_public_ip(char *out, size_t out_size)
{
    http_resp_ctx_t ctx = { .len = 0 };
    ctx.buf[0] = '\0';

    esp_http_client_config_t cfg = {
        .url = "http://httpbin.org/ip",
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .timeout_ms = 5000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return false;

    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || ctx.len == 0) {
        ESP_LOGW(TAG, "公网 IP 获取失败: %s", esp_err_to_name(err));
        return false;
    }

    // httpbin.org/ip 返回 JSON: {"origin": "1.2.3.4"}
    char *p = strstr(ctx.buf, "\"origin\"");
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '"') p++;

    char *end = p;
    while (*end && *end != '"' && *end != '\n' && *end != '}') end++;

    size_t ip_len = end - p;
    if (ip_len == 0 || ip_len >= out_size) return false;

    memcpy(out, p, ip_len);
    out[ip_len] = '\0';
    return true;
}

/* ================================================================
 *  连接 WiFi 并获取 IP
 * ================================================================ */
wifi_conn_result_t wifi_connect_and_get_ips(int core_id,
                                             const char *ssid,
                                             const char *password)
{
    (void)core_id;   // 当前只做连接，core_id 暂未使用

    wifi_conn_result_t result = {0};
    strncpy(result.local_ip, "N/A", sizeof(result.local_ip));
    strncpy(result.public_ip, "N/A", sizeof(result.public_ip));

    /* 1. NVS 初始化 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败");
        return result;
    }

    /* 2. 网络栈与事件循环 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    s_wifi_event_group = xEventGroupCreate();

    /* 3. WiFi 初始化 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         wifi_event_handler, NULL, NULL);

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, password, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 4. 等待连接结果，最多 15 秒 */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000)
    );

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "WiFi 连接失败（SSID: %s）", ssid);

        esp_wifi_stop();
        esp_wifi_deinit();
        vEventGroupDelete(s_wifi_event_group);
        return result;
    }

    result.connected = true;
    ESP_LOGI(TAG, "WiFi 已连接，SSID: %s", ssid);

    /* 5. 获取局域网 IP */
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
        snprintf(result.local_ip, sizeof(result.local_ip),
                 IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "局域网 IP: %s", result.local_ip);
    } else {
        ESP_LOGW(TAG, "获取局域网 IP 失败");
    }

    /* 6. 尝试获取公网 IP，失败不影响连接状态 */
    if (fetch_public_ip(result.public_ip, sizeof(result.public_ip))) {
        ESP_LOGI(TAG, "公网 IP: %s", result.public_ip);
    } else {
        ESP_LOGW(TAG, "公网 IP 不可用（可能无外网访问或 DNS 失败）");
        strncpy(result.public_ip, "N/A", sizeof(result.public_ip));
    }

    return result;
}