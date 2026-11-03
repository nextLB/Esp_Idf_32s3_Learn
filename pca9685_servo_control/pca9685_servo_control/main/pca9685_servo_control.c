
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

/* ======================= 硬件配置 ======================= */
#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define PCA9685_I2C_ADDR    0x40
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

static const char *TAG = "PCA9685";

/* ======================= PCA9685 寄存器 ======================= */
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define MODE1_SLEEP         (1 << 4)
#define MODE1_AI            (1 << 5)
#define MODE1_RESTART       (1 << 7)

/* =========================================================================
 *  ★ 舵机型号（拆开语义，避免冲突）★
 * ========================================================================= */
typedef enum {
    SERVO_TYPE_SF006_180 = 0,   /* 180° 位置舵机（SF006）        */
    SERVO_TYPE_SG90_180,        /* 180° 位置舵机（SG90 位置版）  */
    SERVO_TYPE_SG90_360,        /* 360° 连续旋转舵机（SG90 连续版）*/
} servo_type_t;

/* =========================================================================
 *  ★ 控制模式 ★
 * ========================================================================= */
typedef enum {
    SERVO_MODE_POSITION = 0,    /* 位置模式：param1 = 目标角度 0~180           */
                                /* 仅 SERVO_TYPE_*_180 可用                    */
    SERVO_MODE_SPEED_TIME,      /* 速度+时间模式：param1 = 速度 -100~100       */
                                /*                param2 = 运行时长(ms)        */
                                /* 仅 SERVO_TYPE_SG90_360 可用                 */
    SERVO_MODE_SPEED_ANGLE,     /* 速度+角度模式：param1 = 速度 6~20（带符号） */
                                /*                param2 = 目标角度(度)        */
                                /* 仅 SERVO_TYPE_SG90_360 可用                 */
} servo_mode_t;

/* ======================= 校准参数 ======================= */
/* 180° 位置舵机：0° -> 500us，180° -> 2500us
 * 若实测打限位，可改窄到 700~2300 */
#define SERVO_180_PULSE_MIN_US   500
#define SERVO_180_PULSE_MAX_US   2500

/* 360° 连续旋转舵机：1630us 停点，实测停不住就微调 */
#define SG90_STOP_PULSE_US       1630

/* SPEED_TIME 模式：每 1% 速度偏移 10us */
#define SG90_360_PULSE_PER_PCT   10

/* SPEED_ANGLE 模式：查表速度范围 */
#define SG90_MIN_SPEED           6
#define SG90_MAX_SPEED           20

/* SPEED_ANGLE 模式：时间补偿 */
#define SG90_ACCEL_COMP_MS       0
#define SG90_COAST_DEG           0.0f
#define SG90_TIME_CORRECTION     1.22f

/* ======================= 角速度表 ======================= */
typedef struct {
    int   speed;
    float omega;
} omega_point_t;

static const omega_point_t g_fwd_omega_table[] = {
    {  6, 142.0f }, {  8, 154.0f }, { 10, 173.0f }, { 15, 214.0f }, { 20, 220.0f },
};
#define FWD_TABLE_LEN (sizeof(g_fwd_omega_table) / sizeof(g_fwd_omega_table[0]))

static const omega_point_t g_rev_omega_table[] = {
    {  6, 142.0f }, {  8, 154.0f }, { 10, 173.0f }, { 15, 214.0f }, { 20, 220.0f },
};
#define REV_TABLE_LEN (sizeof(g_rev_omega_table) / sizeof(g_rev_omega_table[0]))

static float omega_lookup(const omega_point_t *tab, int len, int speed)
{
    if (speed <= tab[0].speed)       return tab[0].omega;
    if (speed >= tab[len - 1].speed) return tab[len - 1].omega;
    for (int i = 0; i < len - 1; i++) {
        if (speed >= tab[i].speed && speed <= tab[i + 1].speed) {
            float t = (float)(speed - tab[i].speed)
                    / (float)(tab[i + 1].speed - tab[i].speed);
            return tab[i].omega + t * (tab[i + 1].omega - tab[i].omega);
        }
    }
    return tab[len - 1].omega;
}

/* ======================= 底层 I2C / PCA9685 ======================= */
static esp_err_t pca9685_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t pca9685_init(void)
{
    if (pca9685_write_reg(PCA9685_MODE1, MODE1_SLEEP) != ESP_OK) return ESP_FAIL;
    if (pca9685_write_reg(PCA9685_PRESCALE, 0x79) != ESP_OK) return ESP_FAIL;
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

/* =========================================================================
 *  内部实现 1：180° 位置舵机（SF006 / SG90_180 共用）
 * ========================================================================= */
static esp_err_t internal_servo_180_set_angle(uint8_t channel, float angle,
                                              const char *model)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint16_t pulse_us = SERVO_180_PULSE_MIN_US
                      + (uint16_t)((SERVO_180_PULSE_MAX_US - SERVO_180_PULSE_MIN_US)
                                   * angle / 180.0f + 0.5f);
    pca9685_set_pulse_us(channel, pulse_us);

    ESP_LOGI(TAG, "CH%02d [%s] POSITION -> %5.1f° (%uus)",
             channel, model, angle, pulse_us);
    return ESP_OK;
}

/* =========================================================================
 *  内部实现 2：360° 舵机 速度 + 时间
 * ========================================================================= */
static esp_err_t internal_servo_360_speed_time(uint8_t channel, int speed,
                                               uint32_t duration_ms)
{
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;
    if (speed >= -5 && speed <= 5) speed = 0;

    uint16_t pulse_us;
    if (speed == 0) {
        pulse_us = SG90_STOP_PULSE_US;
    } else {
        int32_t p = (int32_t)SG90_STOP_PULSE_US + speed * SG90_360_PULSE_PER_PCT;
        if (p < 500)  p = 500;
        if (p > 2500) p = 2500;
        pulse_us = (uint16_t)p;
    }
    pca9685_set_pulse_us(channel, pulse_us);

    if (speed == 0) {
        ESP_LOGI(TAG, "CH%02d [SG90_360] SPEED_TIME -> STOP (%uus)", channel, pulse_us);
    } else {
        ESP_LOGI(TAG, "CH%02d [SG90_360] SPEED_TIME -> %s %d%% (%uus, %lums)",
                 channel, speed > 0 ? "FWD" : "REV",
                 speed > 0 ? speed : -speed, pulse_us,
                 (unsigned long)duration_ms);
    }

    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        pca9685_set_pulse_us(channel, SG90_STOP_PULSE_US);
        ESP_LOGI(TAG, "CH%02d [SG90_360] SPEED_TIME -> STOP (time up)", channel);
    }
    return ESP_OK;
}

/* =========================================================================
 *  内部实现 3：360° 舵机 速度 + 角度（查表算时间）
 * ========================================================================= */
static esp_err_t internal_servo_360_speed_angle(uint8_t channel, int speed,
                                                float angle_deg)
{
    if (angle_deg <= 0.0f) {
        ESP_LOGE(TAG, "SPEED_ANGLE: invalid angle %.1f", angle_deg);
        return ESP_ERR_INVALID_ARG;
    }

    int dir       = (speed >= 0) ? 1 : -1;
    int abs_speed = (speed >= 0) ? speed : -speed;

    if (abs_speed < SG90_MIN_SPEED) abs_speed = SG90_MIN_SPEED;
    if (abs_speed > SG90_MAX_SPEED) abs_speed = SG90_MAX_SPEED;

    const omega_point_t *tab = (dir > 0) ? g_fwd_omega_table : g_rev_omega_table;
    int   len   = (dir > 0) ? FWD_TABLE_LEN : REV_TABLE_LEN;
    float omega = omega_lookup(tab, len, abs_speed);

    float t_rotate_ms = angle_deg / omega * 1000.0f * SG90_TIME_CORRECTION;
    float t_coast_ms  = SG90_COAST_DEG / omega * 1000.0f;
    float t_total_ms  = t_rotate_ms + SG90_ACCEL_COMP_MS - t_coast_ms;
    if (t_total_ms < 10.0f) t_total_ms = 10.0f;

    int32_t pulse_us = SG90_STOP_PULSE_US + dir * abs_speed * 10;
    if (pulse_us < 500 || pulse_us > 2500) {
        ESP_LOGE(TAG, "SPEED_ANGLE: unsafe pulse %ldus", (long)pulse_us);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "CH%02d [SG90_360] SPEED_ANGLE -> %s spd=%d ω=%.1f°/s ang=%.1f° t=%.0fms",
             channel, dir > 0 ? "FWD" : "REV", abs_speed, omega,
             angle_deg, t_total_ms);

    pca9685_set_pulse_us(channel, (uint16_t)pulse_us);
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(t_total_ms + 0.5f)));
    pca9685_set_pulse_us(channel, SG90_STOP_PULSE_US);
    return ESP_OK;
}

/* =========================================================================
 *  ★★★ 统一舵机控制入口 ★★★
 * -------------------------------------------------------------------------
 * @param channel  PCA9685 通道 0~15
 * @param type     舵机型号
 *                   SERVO_TYPE_SF006_180  : 180° 位置舵机
 *                   SERVO_TYPE_SG90_180   : 180° 位置舵机
 *                   SERVO_TYPE_SG90_360   : 360° 连续旋转舵机
 * @param mode     控制模式
 *                   SERVO_MODE_POSITION    : 位置（仅 180° 型）
 *                   SERVO_MODE_SPEED_TIME  : 速度+时间（仅 360° 型）
 *                   SERVO_MODE_SPEED_ANGLE : 速度+角度（仅 360° 型）
 * @param param1   含义见下
 * @param param2   含义见下
 *
 * param 语义速查：
 * ┌──────────────────┬──────────────┬────────────────────────────┐
 * │ 型号 / 模式       │ param1        │ param2                     │
 * ├──────────────────┼──────────────┼────────────────────────────┤
 * │ *_180 / POSITION │ 角度 0~180    │ 忽略                       │
 * │ SG90_360 / SPEED_TIME  │ 速度 -100~100 │ 运行时长(ms)，0=持续转    │
 * │ SG90_360 / SPEED_ANGLE │ 速度 -20~20   │ 目标角度(度)              │
 * └──────────────────┴──────────────┴────────────────────────────┘
 *
 * @return ESP_OK / ESP_ERR_INVALID_ARG
 * ========================================================================= */
esp_err_t servo_control(uint8_t channel, servo_type_t type, servo_mode_t mode,
                        float param1, float param2)
{
    if (channel > 15) {
        ESP_LOGE(TAG, "invalid channel %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    /* ---- 180° 位置舵机 ---- */
    if (type == SERVO_TYPE_SF006_180 || type == SERVO_TYPE_SG90_180) {
        if (mode != SERVO_MODE_POSITION) {
            ESP_LOGE(TAG, "type=%d only supports SERVO_MODE_POSITION", type);
            return ESP_ERR_INVALID_ARG;
        }
        const char *name = (type == SERVO_TYPE_SF006_180) ? "SF006" : "SG90_180";
        return internal_servo_180_set_angle(channel, param1, name);
    }

    /* ---- 360° 连续旋转舵机 ---- */
    if (type == SERVO_TYPE_SG90_360) {
        if (mode == SERVO_MODE_SPEED_TIME) {
            return internal_servo_360_speed_time(channel,
                                                 (int)param1,
                                                 (uint32_t)param2);
        } else if (mode == SERVO_MODE_SPEED_ANGLE) {
            return internal_servo_360_speed_angle(channel,
                                                  (int)param1,
                                                  param2);
        } else {
            ESP_LOGE(TAG, "SG90_360 doesn't support SERVO_MODE_POSITION");
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_ERR_INVALID_ARG;
}

/* =========================================================================
 *  便捷封装（推荐在主函数里用，免去记 param1/param2 含义）
 * ========================================================================= */
static inline esp_err_t servo_180_set_angle(uint8_t ch, servo_type_t type, float angle)
{
    return servo_control(ch, type, SERVO_MODE_POSITION, angle, 0.0f);
}
static inline esp_err_t servo_360_speed_time(uint8_t ch, int speed, uint32_t ms)
{
    return servo_control(ch, SERVO_TYPE_SG90_360, SERVO_MODE_SPEED_TIME,
                         (float)speed, (float)ms);
}
static inline esp_err_t servo_360_speed_angle(uint8_t ch, int speed, float angle)
{
    return servo_control(ch, SERVO_TYPE_SG90_360, SERVO_MODE_SPEED_ANGLE,
                         (float)speed, angle);
}
static inline esp_err_t servo_360_stop(uint8_t ch)
{
    return servo_control(ch, SERVO_TYPE_SG90_360, SERVO_MODE_SPEED_TIME,
                         0.0f, 0.0f);
}

/* =========================================================================
 *  主函数示例
 * ========================================================================= */
void app_main(void)
{
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
        ESP_LOGE(TAG, "PCA9685 init failed!");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 ready.");

    while (1) {

        /* ============================================================
         * 写法 A：直接调统一入口 servo_control
         * ============================================================ */

        // /* ① 180° SF006：转到 90° */
        // servo_control(0, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 90.0f, 0);
        // vTaskDelay(pdMS_TO_TICKS(1000));



        /* ② 180° SG90：转到 45° */
        servo_control(0, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 0.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(0, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 45.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(0, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(0, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 180.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(0, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));




        servo_control(4, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 0.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(4, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 45.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(4, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(4, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 180.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(4, SERVO_TYPE_SG90_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));




        servo_control(8, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 0.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(8, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 45.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(8, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(8, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 180.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(8, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        

        
        servo_control(12, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 0.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(12, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 45.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(12, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(12, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 180.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_control(12, SERVO_TYPE_SF006_180, SERVO_MODE_POSITION, 90.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));










        // /* ③ 360° SG90：50% 速度转 1 秒（速度+时间） */
        // servo_control(4, SERVO_TYPE_SG90_360, SERVO_MODE_SPEED_TIME, 50.0f, 1000.0f);
        // vTaskDelay(pdMS_TO_TICKS(1500));

        // /* ④ 360° SG90：速度 10 正转 90°（速度+角度） */
        // servo_control(4, SERVO_TYPE_SG90_360, SERVO_MODE_SPEED_ANGLE, 10.0f, 90.0f);
        // vTaskDelay(pdMS_TO_TICKS(1500));

        // /* ⑤ 360° SG90：反转 180°（速度+角度，负速度=反转） */
        // servo_control(4, SERVO_TYPE_SG90_360, SERVO_MODE_SPEED_ANGLE, -15.0f, 180.0f);
        // vTaskDelay(pdMS_TO_TICKS(1500));

        // /* ============================================================
        //  * 写法 B：用便捷封装，语义更清楚
        //  * ============================================================ */

        // servo_180_set_angle(0, SERVO_TYPE_SF006_180, 0.0f);
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // servo_180_set_angle(0, SERVO_TYPE_SF006_180, 180.0f);
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // /* 360° 正转 80%，持续转，5 秒后手动停 */
        // servo_360_speed_time(4, 80, 0);              /* 0 = 一直转 */
        // vTaskDelay(pdMS_TO_TICKS(5000));
        // servo_360_stop(4);
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // /* 360° 以速度 8 反转 45° */
        // servo_360_speed_angle(4, -8, 45.0f);
        // vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "--- Test Loop Done ---");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}