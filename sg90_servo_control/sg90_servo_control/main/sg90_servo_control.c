#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

/* ==================== 接线配置 ==================== */
#define SERVO_GPIO          GPIO_NUM_4
/* ================================================= */

/* ==================== 舵机参数 ==================== */
#define SERVO_FREQ_HZ       50
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0
#define SERVO_MODE          LEDC_LOW_SPEED_MODE

#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY      ((1 << 14) - 1)
#define SERVO_PERIOD_MS     20.0f

/*
 * SG90 360° 连续旋转舵机官方速度参考（空载）：
 *   约 110 RPM @ 4.8V
 *   约 120 RPM @ 5V
 *   约 130 RPM @ 6V
 *
 * 注意：RPM 是“空载最大转速”的物理单位。
 * 实际转速会随供电电压、负载增加而下降，
 * 低速指令下可能因扭矩不足而堵转（不转）。
 */
#define SERVO_MIN_PULSE_MS  0.5f    /* 对应 -100（全速反转） */
#define SERVO_MAX_PULSE_MS  2.5f    /* 对应 +100（全速正转） */
#define SERVO_STOP_PULSE_MS 1.5f    /* 对应 0（停止） */

/*
 * 速度范围：-100 ~ +100
 * 这里的数值不是官方物理单位，而是“占空比/速度百分比映射”：
 *   +100 ≈ 正转方向全速（对应脉宽2.5ms，空载约110~130 RPM）
 *   +50  ≈ 正转方向半速（对应脉宽2.0ms，实际RPM小于全速值）
 *    0   = 停止（对应脉宽1.5ms，但实际舵机停止点可能有微小偏差）
 *   -50  ≈ 反转方向半速（对应脉宽1.0ms）
 *   -100 ≈ 反转方向全速（对应脉宽0.5ms，空载约110~130 RPM）
 *
 * 若 speed=0 时舵机缓慢漂移，微调 SERVO_STOP_PULSE_MS（如1.48f~1.52f）。
 */
#define SERVO_SPEED_MAX     100
#define SERVO_DEAD_ZONE     4   /* |speed|<=4 视为停止，避免低速堵转抖动 */

#define SERVO_BOOT_STOP_MS  2000

static const char *TAG = "servo_360";

static uint32_t speed_to_duty(int speed)
{
    if (speed >  SERVO_SPEED_MAX) speed =  SERVO_SPEED_MAX;
    if (speed < -SERVO_SPEED_MAX) speed = -SERVO_SPEED_MAX;

    if (abs(speed) <= SERVO_DEAD_ZONE) {
        speed = 0;
    }

    /*
     * 映射关系（占空比百分比，非官方RPM）：
     *   -100 -> 0.5ms
     *      0 -> 1.5ms
     *   +100 -> 2.5ms
     */
    float pulse_ms = SERVO_STOP_PULSE_MS +
                     (float)speed / (float)SERVO_SPEED_MAX *
                     (SERVO_MAX_PULSE_MS - SERVO_STOP_PULSE_MS);

    if (pulse_ms < SERVO_MIN_PULSE_MS) pulse_ms = SERVO_MIN_PULSE_MS;
    if (pulse_ms > SERVO_MAX_PULSE_MS) pulse_ms = SERVO_MAX_PULSE_MS;

    return (uint32_t)(pulse_ms / SERVO_PERIOD_MS * SERVO_MAX_DUTY);
}

static void servo_set_speed(int speed)
{
    uint32_t duty = speed_to_duty(speed);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));

    float pulse_ms = (float)duty * SERVO_PERIOD_MS / (float)SERVO_MAX_DUTY;

    /* 日志中“百分比”指占空比映射值，不代表官方RPM */
    if (speed == 0) {
        ESP_LOGI(TAG, "STOP            (speed=%d, duty=%lu, pulse=%.3fms)",
                 speed, (unsigned long)duty, (double)pulse_ms);
    } else if (speed > 0) {
        ESP_LOGI(TAG, "FORWARD  %3d%%   (speed=%d, duty=%lu, pulse=%.3fms)",
                 speed, speed, (unsigned long)duty, (double)pulse_ms);
    } else {
        ESP_LOGI(TAG, "REVERSE  %3d%%   (speed=%d, duty=%lu, pulse=%.3fms)",
                 -speed, speed, (unsigned long)duty, (double)pulse_ms);
    }
}

/*
 * ==================== 核心函数 ====================
 *
 * servo_rotate(speed, duration_ms)
 *
 * speed:
 *   +1 ~ +100 : 正转，数值越大越接近空载全速（约110~130 RPM）
 *     0       : 停止
 *   -1 ~ -100 : 反转，绝对值越大越接近空载全速
 *   |speed| <= 4 视为停止（死区，避免低速堵转）
 *
 * duration_ms:
 *   >0 : 保持该速度 duration_ms 后自动停止
 *   =0 : 一直保持，不自动停止
 *
 * 示例：
 *   servo_rotate( 50, 2000); // 正转半速映射，2秒后停
 *   servo_rotate(-100, 1000); // 反转全速映射，1秒后停
 *   servo_rotate(  0,    0); // 立即停止并保持
 */
static void servo_rotate(int speed, uint32_t duration_ms)
{
    servo_set_speed(speed);

    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        servo_set_speed(0);
    }
}

static void servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = speed_to_duty(0),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ESP_LOGI(TAG, "Init GPIO%d, 50Hz, STOP. Ref: ~110RPM@4.8V, ~130RPM@6V (no load)",
             SERVO_GPIO);
}

void app_main(void)
{
    servo_init();

    servo_set_speed(0);
    vTaskDelay(pdMS_TO_TICKS(SERVO_BOOT_STOP_MS));

    while (1) {
        servo_rotate( 50, 2000);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_rotate(-50, 2000);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_rotate(100, 1000);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_rotate(-100, 1000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}