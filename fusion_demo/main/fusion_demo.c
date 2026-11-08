/*
 * ============================================================================
 *  ESP32-S3 四足机器人 + WiFi/UDP 遥控  融合程序（单文件版）
 * ----------------------------------------------------------------------------
 *  核 0 : WiFi 连接 + UDP 监听（收指令，写全局命令）
 *  核 1 : 运动任务 dog_task（执行 停止/前进/后退，保持状态直到新指令）
 *
 *  UDP 指令：
 *      "0" -> 停止（回中位站住）
 *      "1" -> 前进（BOUND 步态）
 *      "2" -> 后退（DIAGONAL 步态）
 *  非法指令回 "ERR"，合法指令回 "OK"。
 *
 *  所有可调参数集中在下面的【全局可调配置区】。
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"


/* ============================================================================
 *                          全局可调配置区
 * ============================================================================ */

/* ---------------- WiFi ---------------- */
#define WIFI_SSID                   "616"
#define WIFI_PASSWORD               "aa472888"
#define WIFI_MAX_RETRY              5
#define WIFI_CONNECT_TIMEOUT_MS     15000

/* ---------------- UDP ---------------- */
#define UDP_LISTEN_PORT             8080
#define UDP_RX_BUF_SIZE             128
#define UDP_ACK_OK                  "OK"
#define UDP_ACK_ERR                 "ERR"

/* ---------------- 任务核心绑定（ESP32-S3 双核） ---------------- */
#define CORE_WIFI_UDP               0    /* 核0：WiFi + UDP 监听 */
#define CORE_DOG                    1    /* 核1：运动控制       */

/* ---------------- 任务栈 / 优先级 ---------------- */
#define UDP_TASK_STACK              4096
#define UDP_TASK_PRIO               5
#define DOG_TASK_STACK              4096
#define DOG_TASK_PRIO               5

/* ---------------- I2C / PCA9685 硬件 ---------------- */
#define I2C_MASTER_SDA_IO           GPIO_NUM_8
#define I2C_MASTER_SCL_IO           GPIO_NUM_9
#define PCA9685_I2C_ADDR            0x40
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define PCA9685_PRESCALE_VAL        0x79     /* 50Hz */

/* PCA9685 寄存器 */
#define PCA9685_MODE1               0x00
#define PCA9685_PRESCALE            0xFE
#define PCA9685_LED0_ON_L           0x06
#define MODE1_SLEEP                 (1 << 4)
#define MODE1_AI                    (1 << 5)
#define MODE1_RESTART               (1 << 7)

/* 舵机脉宽范围 */
#define SERVO_180_PULSE_MIN_US      500
#define SERVO_180_PULSE_MAX_US      2500

/* ---------------- 腿配置 ---------------- */
#define LEG_LF_CH                   0
#define LEG_RF_CH                   4
#define LEG_LR_CH                   8
#define LEG_RR_CH                   12

#define LEG_LF_TYPE                 SERVO_TYPE_SG90_180
#define LEG_RF_TYPE                 SERVO_TYPE_SG90_180
#define LEG_LR_TYPE                 SERVO_TYPE_SF006_180
#define LEG_RR_TYPE                 SERVO_TYPE_SF006_180

#define DIR_LF                      (+1)
#define DIR_RF                      (-1)
#define DIR_LR                      (+1)
#define DIR_RR                      (-1)

/* ---------------- 步态参数 ---------------- */
#define DOG_CENTER_DEG              90.0f
#define DOG_STEP_DEG                40.0f   /* 腿相对 90° 偏 ±40° */
#define DOG_PUSH_MS                 420     /* 推进相：慢蹬   */
#define DOG_RESET_MS                140     /* 复位相：快滑   */
#define DOG_FRAME_MS                8       /* 帧间隔 ≈125Hz  */
#define DOG_SETTLE_MS               200     /* 切指令回中停顿 */

/* ---------------- 命令号 ---------------- */
#define DOG_CMD_STOP                0
#define DOG_CMD_FORWARD             1
#define DOG_CMD_BACKWARD            2

/* ---------------- 日志 TAG ---------------- */
static const char *TAG_APP  = "APP";
static const char *TAG_WIFI = "wifi_mgr";
static const char *TAG_UDP  = "UDP";
static const char *TAG_DOG  = "4LEG";


/* ============================================================================
 *                          舵机类型 / 模式枚举
 * ============================================================================ */
typedef enum { SERVO_TYPE_SF006_180 = 0, SERVO_TYPE_SG90_180 } servo_type_t;
typedef enum { SERVO_MODE_POSITION = 0 } servo_mode_t;


/* ============================================================================
 *                          PCA9685 底层驱动
 * ============================================================================ */
static esp_err_t pca9685_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t pca9685_init(void)
{
    if (pca9685_write_reg(PCA9685_MODE1, MODE1_SLEEP) != ESP_OK) return ESP_FAIL;
    if (pca9685_write_reg(PCA9685_PRESCALE, PCA9685_PRESCALE_VAL) != ESP_OK) return ESP_FAIL;
    if (pca9685_write_reg(PCA9685_MODE1, MODE1_AI | MODE1_RESTART) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static void pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (channel > 15) return;
    uint16_t ticks = (uint16_t)((pulse_us / 4.875f) + 0.5f);
    if (ticks > 4095) ticks = 4095;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    pca9685_write_reg(reg + 0, 0);
    pca9685_write_reg(reg + 1, 0);
    pca9685_write_reg(reg + 2, ticks & 0xFF);
    pca9685_write_reg(reg + 3, (ticks >> 8) & 0x0F);
}

static esp_err_t servo_control(uint8_t channel, servo_type_t type, servo_mode_t mode,
                               float param1, float param2)
{
    (void)type; (void)param2;
    if (channel > 15) return ESP_ERR_INVALID_ARG;
    if (mode != SERVO_MODE_POSITION) return ESP_ERR_INVALID_ARG;

    float angle = param1;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint16_t pulse_us = SERVO_180_PULSE_MIN_US
                      + (uint16_t)((SERVO_180_PULSE_MAX_US - SERVO_180_PULSE_MIN_US)
                                   * angle / 180.0f + 0.5f);
    pca9685_set_pulse_us(channel, pulse_us);
    return ESP_OK;
}


/* ============================================================================
 *                          单腿控制
 * ============================================================================ */
static void dog_set_leg(uint8_t ch, servo_type_t type, int dir, float offset_deg)
{
    float angle = DOG_CENTER_DEG + dir * offset_deg;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    servo_control(ch, type, SERVO_MODE_POSITION, angle, 0.0f);
}

static void set_LF(float off) { dog_set_leg(LEG_LF_CH, LEG_LF_TYPE, DIR_LF, off); }
static void set_RF(float off) { dog_set_leg(LEG_RF_CH, LEG_RF_TYPE, DIR_RF, off); }
static void set_LR(float off) { dog_set_leg(LEG_LR_CH, LEG_LR_TYPE, DIR_LR, off); }
static void set_RR(float off) { dog_set_leg(LEG_RR_CH, LEG_RR_TYPE, DIR_RR, off); }

/* ---------------- 两套分组 ---------------- */
/* BOUND（前进）：A=前腿(LF+RF)，B=后腿(LR+RR) */
static void set_A_bound(float off) { set_LF(off); set_RF(off); }
static void set_B_bound(float off) { set_LR(off); set_RR(off); }

/* DIAGONAL（后退）：A=LF+RR，B=RF+LR */
static void set_A_diag(float off) { set_LF(off); set_RR(off); }
static void set_B_diag(float off) { set_RF(off); set_LR(off); }


/* ============================================================================
 *                          步态插值 / 周期
 * ============================================================================ */
typedef void (*leg_set_fn)(float);

static void dog_interp(float a_from, float a_to, uint32_t a_ms,
                       float b_from, float b_to, uint32_t b_ms,
                       leg_set_fn set_a, leg_set_fn set_b)
{
    uint32_t total   = (a_ms > b_ms) ? a_ms : b_ms;
    uint32_t elapsed = 0;
    while (elapsed < total) {
        elapsed += DOG_FRAME_MS;
        if (elapsed > total) elapsed = total;

        float ta = (elapsed >= a_ms) ? 1.0f : (float)elapsed / (float)a_ms;
        float tb = (elapsed >= b_ms) ? 1.0f : (float)elapsed / (float)b_ms;

        set_a(a_from + (a_to - a_from) * ta);
        set_b(b_from + (b_to - b_from) * tb);
        vTaskDelay(pdMS_TO_TICKS(DOG_FRAME_MS));
    }
}

/* 前进周期（BOUND）：A(前腿) 慢蹬 +s→-s，B(后腿) 快滑 -s→+s，再交换 */
static void dog_cycle_forward(void)
{
    const float s = DOG_STEP_DEG;
    dog_interp(+s, -s, DOG_PUSH_MS,
               -s, +s, DOG_RESET_MS,
               set_A_bound, set_B_bound);
    dog_interp(-s, +s, DOG_RESET_MS,
               +s, -s, DOG_PUSH_MS,
               set_A_bound, set_B_bound);
}

/* 后退周期（DIAGONAL）：A(LF+RR) 慢蹬 -s→+s，B(RF+LR) 快滑 +s→-s，再交换 */
static void dog_cycle_backward(void)
{
    const float s = DOG_STEP_DEG;
    dog_interp(-s, +s, DOG_PUSH_MS,
               +s, -s, DOG_RESET_MS,
               set_A_diag, set_B_diag);
    dog_interp(+s, -s, DOG_RESET_MS,
               -s, +s, DOG_PUSH_MS,
               set_A_diag, set_B_diag);
}

/* 回中位 + 稳定 */
static void dog_center(void)
{
    set_LF(0.0f);
    set_RF(0.0f);
    set_LR(0.0f);
    set_RR(0.0f);
    vTaskDelay(pdMS_TO_TICKS(DOG_SETTLE_MS));
}


/* ============================================================================
 *                          命令接口（跨核）
 * ============================================================================ */
static volatile int g_cmd      = DOG_CMD_STOP;
static volatile int g_last_cmd = -1;

/* 任意任务可调用；不阻塞，dog_task 会立刻切换 */
void dog_execute(int cmd)
{
    if (cmd < DOG_CMD_STOP || cmd > DOG_CMD_BACKWARD) {
        ESP_LOGW(TAG_DOG, "invalid cmd %d (only 0/1/2)", cmd);
        return;
    }
    g_cmd = cmd;
}

static const char *cmd_name(int cmd)
{
    switch (cmd) {
    case DOG_CMD_STOP:     return "STOP     (停止)";
    case DOG_CMD_FORWARD:  return "FORWARD  (前进/BOUND)";
    case DOG_CMD_BACKWARD: return "BACKWARD (后退/DIAGONAL)";
    default:               return "UNKNOWN";
    }
}

/* ---------------- 后台运动任务（绑核 CORE_DOG） ---------------- */
static void dog_task(void *arg)
{
    (void)arg;
    while (1) {
        int cmd = g_cmd;

        /* 命令切换：先回中位（两套步态分组不同，必须归零再切） */
        if (cmd != g_last_cmd) {
            ESP_LOGI(TAG_DOG, "========================================");
            ESP_LOGI(TAG_DOG, ">>>  %s  <<<", cmd_name(cmd));
            ESP_LOGI(TAG_DOG, "========================================");
            dog_center();
            g_last_cmd = cmd;
        }

        switch (cmd) {
        case DOG_CMD_STOP:     vTaskDelay(pdMS_TO_TICKS(50)); break;
        case DOG_CMD_FORWARD:  dog_cycle_forward();           break;
        case DOG_CMD_BACKWARD: dog_cycle_backward();          break;
        default:               vTaskDelay(pdMS_TO_TICKS(50)); break;
        }
    }
}


/* ============================================================================
 *                          WiFi 管理
 * ============================================================================ */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define HTTP_RESP_BUF_SIZE 256

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

typedef struct {
    bool connected;
    char local_ip[16];
    char public_ip[16];
} wifi_conn_result_t;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG_WIFI, "重连中 %d/%d", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ---------------- 获取公网 IP ---------------- */
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
        ESP_LOGW(TAG_WIFI, "公网 IP 获取失败: %s", esp_err_to_name(err));
        return false;
    }

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

/* ---------------- 连接 WiFi 并获取 IP ---------------- */
static wifi_conn_result_t wifi_connect_and_get_ips(const char *ssid,
                                                   const char *password)
{
    wifi_conn_result_t result = {0};
    strncpy(result.local_ip,  "N/A", sizeof(result.local_ip));
    strncpy(result.public_ip, "N/A", sizeof(result.public_ip));

    /* 1. NVS 初始化 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "NVS 初始化失败");
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
    strncpy((char *)wc.sta.ssid,     ssid,     sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, password, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 4. 等待连接结果 */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS)
    );

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG_WIFI, "WiFi 连接失败（SSID: %s）", ssid);
        esp_wifi_stop();
        esp_wifi_deinit();
        vEventGroupDelete(s_wifi_event_group);
        return result;
    }

    result.connected = true;
    ESP_LOGI(TAG_WIFI, "WiFi 已连接，SSID: %s", ssid);

    /* 5. 获取局域网 IP */
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
        snprintf(result.local_ip, sizeof(result.local_ip),
                 IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG_WIFI, "局域网 IP: %s", result.local_ip);
    } else {
        ESP_LOGW(TAG_WIFI, "获取局域网 IP 失败");
    }

    /* 6. 公网 IP（失败不影响连接） */
    if (fetch_public_ip(result.public_ip, sizeof(result.public_ip))) {
        ESP_LOGI(TAG_WIFI, "公网 IP: %s", result.public_ip);
    } else {
        ESP_LOGW(TAG_WIFI, "公网 IP 不可用（可能无外网或 DNS 失败）");
        strncpy(result.public_ip, "N/A", sizeof(result.public_ip));
    }

    return result;
}


/* ============================================================================
 *                          UDP 监听（绑核 CORE_WIFI_UDP）
 * ============================================================================ */
static int s_udp_port = UDP_LISTEN_PORT;

/* 解析 "0"/"1"/"2"（容忍前后空白/换行）；非法返回 -1 */
static int udp_parse_cmd(const char *buf, int len)
{
    int i = 0;
    while (i < len && (buf[i] == ' ' || buf[i] == '\t' ||
                       buf[i] == '\r' || buf[i] == '\n')) i++;
    if (i >= len) return -1;

    char c = buf[i];
    if (c < '0' || c > '2') return -1;
    int cmd = c - '0';

    i++;
    while (i < len && (buf[i] == ' ' || buf[i] == '\t' ||
                       buf[i] == '\r' || buf[i] == '\n')) i++;
    if (i != len) return -1;

    return cmd;
}

static void udp_listener_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG_UDP, "socket 创建失败: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {0};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(s_udp_port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG_UDP, "bind 失败: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_UDP, "UDP 监听已启动，端口 %d（核%d）", s_udp_port, CORE_WIFI_UDP);

    char rx_buf[UDP_RX_BUF_SIZE];
    struct sockaddr_in src_addr;
    socklen_t src_len;

    while (1) {
        src_len = sizeof(src_addr);
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0,
                           (struct sockaddr *)&src_addr, &src_len);
        if (len < 0) {
            ESP_LOGW(TAG_UDP, "recvfrom 错误: errno %d", errno);
            continue;
        }
        rx_buf[len] = '\0';

        char ip_str[16];
        inet_ntoa_r(src_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG_UDP, "收到来自 %s:%d 的消息: [%s]",
                 ip_str, ntohs(src_addr.sin_port), rx_buf);

        /* 解析命令并交给 dog_task 执行 */
        int cmd = udp_parse_cmd(rx_buf, len);
        const char *ack;
        if (cmd >= 0) {
            dog_execute(cmd);
            ESP_LOGI(TAG_UDP, "命令执行: %d -> %s", cmd, cmd_name(cmd));
            ack = UDP_ACK_OK;
        } else {
            ESP_LOGW(TAG_UDP, "非法命令 [%s]，仅支持 0/1/2", rx_buf);
            ack = UDP_ACK_ERR;
        }

        sendto(sock, ack, strlen(ack), 0,
               (struct sockaddr *)&src_addr, src_len);
    }
}

static void udp_listener_start(int core_id, int port)
{
    s_udp_port = port;
    xTaskCreatePinnedToCore(udp_listener_task, "udp_listener",
                            UDP_TASK_STACK, NULL, UDP_TASK_PRIO, NULL, core_id);
}


/* ============================================================================
 *                          app_main
 * ============================================================================ */
void app_main(void)
{
    /* ---------------- 1. 运动硬件初始化 ---------------- */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    if (pca9685_init() != ESP_OK) {
        ESP_LOGE(TAG_APP, "PCA9685 初始化失败，退出");
        return;
    }
    ESP_LOGI(TAG_APP, "PCA9685 ready.");

    /* 上电立刻回中位站正（防止 PCA9685 默认 0 占空比让舵机乱转） */
    dog_center();

    /* 启动运动任务（绑核 CORE_DOG） */
    xTaskCreatePinnedToCore(dog_task, "dog_task",
                            DOG_TASK_STACK, NULL, DOG_TASK_PRIO,
                            NULL, CORE_DOG);
    ESP_LOGI(TAG_APP, "dog_task 已启动（核%d），初始状态 STOP", CORE_DOG);

    /* ---------------- 2. WiFi 连接（app_main 运行在核0） ---------------- */
    ESP_LOGI(TAG_APP, "开始连接 WiFi ...");
    wifi_conn_result_t wifi = wifi_connect_and_get_ips(WIFI_SSID, WIFI_PASSWORD);

    if (wifi.connected) {
        ESP_LOGI(TAG_APP, "========== WiFi 连接成功 ==========");
        ESP_LOGI(TAG_APP, "  局域网 IP : %s", wifi.local_ip);
        ESP_LOGI(TAG_APP, "  公网 IP   : %s", wifi.public_ip);
        ESP_LOGI(TAG_APP, "===================================");

        /* 启动 UDP 监听（绑核 CORE_WIFI_UDP） */
        udp_listener_start(CORE_WIFI_UDP, UDP_LISTEN_PORT);
    } else {
        ESP_LOGE(TAG_APP, "WiFi 连接失败，UDP 未启动");
    }

    /* 主任务常驻 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}