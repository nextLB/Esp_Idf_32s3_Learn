#include "udp_listener.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <errno.h>

static const char *TAG = "UDP";
static int s_port = 8080;

static void udp_listener_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket 创建失败: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {0};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(s_port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);   // 监听所有网卡

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind 失败: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP 监听已启动，端口 %d", s_port);

    char rx_buf[128];
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);

    while (1) {
        src_len = sizeof(src_addr);
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0,
                           (struct sockaddr *)&src_addr, &src_len);
        if (len < 0) {
            ESP_LOGW(TAG, "recvfrom 错误: errno %d", errno);
            continue;
        }

        rx_buf[len] = '\0';

        char ip_str[16];
        inet_ntoa_r(src_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "收到来自 %s:%d 的消息: [%s]",
                 ip_str, ntohs(src_addr.sin_port), rx_buf);

        /* 可选：回一个 ACK，方便电脑端确认 ESP32 收到了 */
        const char *ack = "OK";
        sendto(sock, ack, strlen(ack), 0,
               (struct sockaddr *)&src_addr, src_len);
    }
}

void udp_listener_start(int core_id, int port)
{
    s_port = port;
    xTaskCreatePinnedToCore(udp_listener_task, "udp_listener",
                            4096, NULL, 5, NULL, core_id);
}