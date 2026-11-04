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
#define I2C_MASTER_FREQ_HZ  400000

static const char *TAG = "4LEG";

/* ======================= PCA9685 寄存器 ======================= */
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define MODE1_SLEEP         (1 << 4)
#define MODE1_AI            (1 << 5)
#define MODE1_RESTART       (1 << 7)

#define SERVO_180_PULSE_MIN_US   500
#define SERVO_180_PULSE_MAX_US   2500

typedef enum { SERVO_TYPE_SF006_180 = 0, SERVO_TYPE_SG90_180 } servo_type_t;
typedef enum { SERVO_MODE_POSITION = 0 } servo_mode_t;

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
    if (pca9685_write_reg(PCA9685_PRESCALE, 0x79) != ESP_FAIL && 0) return ESP_FAIL;
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

esp_err_t servo_control(uint8_t channel, servo_type_t type, servo_mode_t mode,
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

/* =========================================================================
 *  腿配置
 * ========================================================================= */
#define LEG_LF_CH    0
#define LEG_RF_CH    4
#define LEG_LR_CH    8
#define LEG_RR_CH    12

#define LEG_LF_TYPE  SERVO_TYPE_SG90_180
#define LEG_RF_TYPE  SERVO_TYPE_SG90_180
#define LEG_LR_TYPE  SERVO_TYPE_SF006_180
#define LEG_RR_TYPE  SERVO_TYPE_SF006_180

#define DIR_LF   (+1)
#define DIR_RF   (-1)
#define DIR_LR   (+1)
#define DIR_RR   (-1)

/* =========================================================================
 *  步态参数（保持你测试能走的那一档）
 * ========================================================================= */
#define DOG_CENTER_DEG   90.0f
#define DOG_STEP_DEG     40.0f   /* 步幅：腿相对 90° 偏 ±40°，角度 50°~130° */
#define DOG_PUSH_MS      420     /* 推进相：慢蹬 420ms */
#define DOG_RESET_MS     140     /* 复位相：快滑 140ms（效率 3:1） */
#define DOG_FRAME_MS     8       /* 帧间隔 8ms ≈125Hz */
#define DOG_SETTLE_MS    200     /* 切换指令时回中位的停顿 */

/* ===================== 单腿 ===================== */
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

/* =========================================================================
 *  两套分组
 * ========================================================================= */
/* BOUND（前进用）：A=前腿(LF+RF)，B=后腿(LR+RR) */
static void set_A_bound(float off) { set_LF(off); set_RF(off); }
static void set_B_bound(float off) { set_LR(off); set_RR(off); }

/* DIAGONAL（后退用）：A=LF+RR，B=RF+LR */
static void set_A_diag(float off) { set_LF(off); set_RR(off); }
static void set_B_diag(float off) { set_RF(off); set_LR(off); }

/* ===================== 通用插值（接收函数指针） ===================== */
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

/* ===================== 前进周期（BOUND） =====================
 * A(前腿) 慢蹬 +s→-s，B(后腿) 快滑 -s→+s，然后交换。
 * ============================================================ */
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

/* ===================== 后退周期（DIAGONAL） =====================
 * A(LF+RR) 慢蹬 -s→+s，B(RF+LR) 快滑 +s→-s，然后交换。
 * ============================================================ */
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

/* ===================== 回中位 + 稳定 ===================== */
static void dog_center(void)
{
    set_LF(0.0f);
    set_RF(0.0f);
    set_LR(0.0f);
    set_RR(0.0f);
    vTaskDelay(pdMS_TO_TICKS(DOG_SETTLE_MS));
}

/* =========================================================================
 *  ★★★ 统一命令入口 ★★★
 * -------------------------------------------------------------------------
 *  dog_execute(0) = 停止（回中位站住）
 *  dog_execute(1) = 前进（BOUND 步态）
 *  dog_execute(2) = 后退（DIAGONAL 步态）
 *
 *  可任意任务里调用，不阻塞，后台 dog_task 立刻切换。
 * ========================================================================= */
typedef enum {
    DOG_CMD_STOP     = 0,
    DOG_CMD_FORWARD  = 1,
    DOG_CMD_BACKWARD = 2,
} dog_cmd_t;

static volatile int g_cmd      = DOG_CMD_STOP;
static volatile int g_last_cmd = -1;

void dog_execute(int cmd)
{
    if (cmd < 0 || cmd > 2) {
        ESP_LOGW(TAG, "invalid cmd %d (only 0/1/2)", cmd);
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

/* ===================== 后台运动任务 ===================== */
static void dog_task(void *arg)
{
    while (1) {
        int cmd = g_cmd;

        /* 命令切换：先回中位（两套步态分组不同，必须归零再切） */
        if (cmd != g_last_cmd) {
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, ">>>  %s  <<<", cmd_name(cmd));
            ESP_LOGI(TAG, "========================================");
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

/* =========================================================================
 *  app_main —— 演示循环
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
    ESP_LOGI(TAG, "PCA9685 ready. (前进=BOUND, 后退=DIAGONAL)");

    /* 启动先站正 */
    dog_center();

    /* 启动后台运动任务 */
    xTaskCreate(dog_task, "dog_task", 4096, NULL, 5, NULL);

    /* 演示循环：前进 5 秒 → 停 2.5 秒 → 后退 5 秒 → 停 2.5 秒 */
    while (1) {
        ESP_LOGI(TAG, "##########  DEMO LOOP  ##########");

        ESP_LOGI(TAG, "---------- [1/2] 前进 5 秒 ----------");
        dog_execute(DOG_CMD_FORWARD);
        vTaskDelay(pdMS_TO_TICKS(5000));
        dog_execute(DOG_CMD_STOP);
        vTaskDelay(pdMS_TO_TICKS(2500));

        ESP_LOGI(TAG, "---------- [2/2] 后退 5 秒 ----------");
        dog_execute(DOG_CMD_BACKWARD);
        vTaskDelay(pdMS_TO_TICKS(5000));
        dog_execute(DOG_CMD_STOP);
        vTaskDelay(pdMS_TO_TICKS(2500));

        ESP_LOGI(TAG, "##########  LOOP DONE  ##########");
    }
}