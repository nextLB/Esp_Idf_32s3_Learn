#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    bool connected;        // 是否连接成功
    char local_ip[16];     // 局域网 IP，失败为 "N/A"
    char public_ip[16];    // 公网 IP，失败为 "N/A"
} wifi_conn_result_t;

/*
 * 连接 WiFi 并获取 IP 信息。
 *
 * core_id   : 预留参数，当前只做连接，内部未使用
 * ssid      : WiFi 名称
 * password  : WiFi 密码
 *
 * 返回值    : 连接状态 + 局域网 IP + 公网 IP
 */
wifi_conn_result_t wifi_connect_and_get_ips(int core_id,
                                             const char *ssid,
                                             const char *password);

#endif