#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

/* =========================================================================
 * 硬件配置说明
 * -------------------------------------------------------------------------
 * 舵机型号: SunFounder SF006FM (9g 全金属齿轮 180° 位置舵机)
 * 信号线 (橙/黄) -> ESP32-S3 的 GPIO4
 * 电源线 (红)    -> 外部 5V 电源正极 (切勿接 ESP32 的 3.3V)
 * 地线 (棕/黑)   -> 外部 5V 电源负极，且必须与 ESP32 的 GND 相连 (共地)
 * ========================================================================= */

#define SERVO_GPIO          GPIO_NUM_4

/* =========================================================================
 * 全局测试变量
 * -------------------------------------------------------------------------
 * 你可以修改这里的 g_test_angle 的值（范围 0 ~ 180），
 * 重新编译烧录后，舵机就会转到对应的角度并保持不动。
 * 建议测试顺序：90 (中位) -> 0 (最左/最右) -> 180 (最右/最左) -> 45 -> 135
 * ========================================================================= */
static int g_target_angle = 180;

/* =========================================================================
 * 舵机参数配置 (适配 SunFounder SF006FM)
 * -------------------------------------------------------------------------
 * 1. 标准模拟/数字位置舵机均采用 50Hz 频率 (周期 20ms)。
 * 2. SF006FM 官方数据手册标明：控制脉宽 500us ~ 2500us 对应 0° ~ 180°。
 *    - 0°   -> 0.5ms 脉宽
 *    - 90°  -> 1.5ms 脉宽 (中位)
 *    - 180° -> 2.5ms 脉宽
 * 3. ESP32 LEDC 采用 14 位分辨率，占空比数值范围为 0 ~ 16383。
 * ========================================================================= */
#define SERVO_FREQ_HZ       50              /* 50Hz 周期 20ms */
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL       LEDC_CHANNEL_0
#define SERVO_MODE          LEDC_LOW_SPEED_MODE

#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY      ((1 << 14) - 1) /* 16383 */

#define SERVO_MIN_PULSE_MS  0.5f            /* 对应 0° */
#define SERVO_MAX_PULSE_MS  2.5f            /* 对应 180° */
#define SERVO_PERIOD_MS     20.0f           /* 50Hz 的周期 */

#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_REFRESH_MS    1000            /* 循环刷新周期，适应先烧录后上电 */

static const char *TAG = "SunFounder_Servo";

/* =========================================================================
 * 内部辅助函数：角度转占空比 (Duty)
 * ========================================================================= */
static uint32_t angle_to_duty(int angle)
{
    /* 限制输入角度在安全范围内，防止堵转烧毁齿轮 */
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    /* 线性映射：计算对应的脉冲宽度 (ms) */
    float pulse_ms = SERVO_MIN_PULSE_MS +
                     (SERVO_MAX_PULSE_MS - SERVO_MIN_PULSE_MS) *
                     (float)(angle - SERVO_MIN_ANGLE) /
                     (float)(SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);

    /* 将脉冲宽度转换为 14位 LEDC 占空比数值 */
    return (uint32_t)(pulse_ms / SERVO_PERIOD_MS * SERVO_MAX_DUTY);
}

/* =========================================================================
 * 核心功能函数：设置舵机角度
 * -------------------------------------------------------------------------
 * 函数名：servo_set_angle
 * 功能：控制舵机转动到指定的绝对角度，并在到达后保持锁定。
 * 参数：angle - 目标角度，范围 0 ~ 180 (整数)
 * 返回值：无
 * ========================================================================= */
static void servo_set_angle(int angle)
{
    /* 1. 将角度转换为 LEDC 需要的占空比数值 */
    uint32_t duty = angle_to_duty(angle);
    
    /* 2. 写入硬件寄存器并生效 */
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));

    /* 3. 计算实际输出脉宽并打印日志，方便调试验证 */
    float actual_pulse_ms = (float)duty * SERVO_PERIOD_MS / (float)SERVO_MAX_DUTY;
    ESP_LOGI(TAG, "Target: %3d° | Duty: %lu | Pulse: %.3f ms", 
             angle, (unsigned long)duty, (double)actual_pulse_ms);
}

/* =========================================================================
 * 初始化函数：配置 LEDC 硬件
 * ========================================================================= */
static void servo_init(void)
{
    /* 配置 LEDC 定时器 */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* 配置 LEDC 通道，直接绑定 GPIO */
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = angle_to_duty(g_target_angle), /* 初始化时直接输出目标角度，防止上电乱转 */
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ESP_LOGI(TAG, "SF006FM Servo Init Success on GPIO%d", SERVO_GPIO);
}

/* =========================================================================
 * 主程序入口
 * ========================================================================= */
void app_main(void)
{
    /* 1. 硬件初始化 */
    servo_init();

    /* 2. 主循环 */
    /*
     * 之所以在主循环里持续调用 servo_set_angle：
     * 1) 满足你“先烧录，后上电”的习惯，上电后马上能收到目标角度；
     * 2) 位置舵机在收到固定占空比的 PWM 信号时会“锁死”在对应角度，
     *    持续输出能保证舵机维持这个扭力，不会因为信号丢失而松懈。
     */
    while (1) {
        servo_set_angle(g_target_angle);
        vTaskDelay(pdMS_TO_TICKS(SERVO_REFRESH_MS));
    }
}