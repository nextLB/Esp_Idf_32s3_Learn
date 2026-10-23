#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

/* ==================== 接线配置 ==================== */
/* 舵机黄线 -> ESP32-S3 的 GPIO4 */
#define SERVO_GPIO          GPIO_NUM_4
/* ================================================= */

#define SERVO_FREQ_HZ       50              /* 标准舵机 50Hz */
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0
#define SERVO_MODE          LEDC_LOW_SPEED_MODE

/* 14 位分辨率，占空比 0 ~ 16383 */
#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY      ((1 << 14) - 1)

/*
 * 50Hz 周期 = 20ms
 * 0°   -> 0.5ms 脉宽 -> duty ≈ 410
 * 180° -> 2.5ms 脉宽 -> duty ≈ 2048
 */
#define SERVO_MIN_PULSE_MS  0.5f
#define SERVO_MAX_PULSE_MS  2.5f
#define SERVO_PERIOD_MS     20.0f

static const char *TAG = "servo_sweep";

static uint32_t angle_to_duty(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    float pulse_ms = SERVO_MIN_PULSE_MS +
                     (SERVO_MAX_PULSE_MS - SERVO_MIN_PULSE_MS) * angle / 180.0f;

    return (uint32_t)(pulse_ms / SERVO_PERIOD_MS * SERVO_MAX_DUTY);
}

static void servo_set_angle(int angle)
{
    uint32_t duty = angle_to_duty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
    ESP_LOGI(TAG, "Angle: %d deg (duty: %lu)", angle, (unsigned long)duty);
}

void app_main(void)
{
    /* 1. 配置 LEDC 定时器 */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* 2. 配置 LEDC 通道，绑定信号 GPIO */
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ESP_LOGI(TAG, "Servo sweep on GPIO%d, 50Hz", SERVO_GPIO);

    /* 3. 循环: 0 -> 180 -> 0 */
    while (1) {
        for (int angle = 0; angle <= 180; angle += 5) {
            servo_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        for (int angle = 180; angle >= 0; angle -= 5) {
            servo_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}