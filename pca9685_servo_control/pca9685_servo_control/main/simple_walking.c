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

static const char *TAG = "DOG";

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
 *  步态选择（改这一行切换步态）
 * ========================================================================= */
#define GAIT_DIAGONAL   0   /* 对角：A=LF+RR, B=RF+LR（你原来的，后退好前进差） */
#define GAIT_BOUND      1   /* 前后交替：A=LF+RF（前）, B=LR+RR（后） ← 推荐先试 */
#define GAIT_ALL        2   /* 四腿同相：所有腿一起扫 */
#define GAIT_PACE       3   /* 左右同侧：A=LF+LR（左）, B=RF+RR（右） */

#define GAIT_MODE  GAIT_BOUND      /* ← 改这里切换 */

/* =========================================================================
 *  步态参数（观察版，方便肉眼比对）
 * ========================================================================= */
#define DOG_CENTER_DEG   90.0f
#define DOG_STEP_DEG     40.0f   /* 摆幅 80°，角度 50°~130° */
#define DOG_PUSH_MS      420     /* 慢推 420ms，舵机从容 */
#define DOG_RESET_MS     140     /* 快回 140ms，推力效率 3:1 */
#define DOG_FRAME_MS     8       /* 125Hz 刷新 */

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

/* ===================== 分组（根据 GAIT_MODE 编译不同分组） ===================== */
#if GAIT_MODE == GAIT_DIAGONAL
static void set_A(float off) { set_LF(off); set_RR(off); }
static void set_B(float off) { set_RF(off); set_LR(off); }
#elif GAIT_MODE == GAIT_BOUND
static void set_A(float off) { set_LF(off); set_RF(off); }   /* 前腿 */
static void set_B(float off) { set_LR(off); set_RR(off); }   /* 后腿 */
#elif GAIT_MODE == GAIT_ALL
static void set_A(float off) { set_LF(off); set_RF(off); set_LR(off); set_RR(off); }
static void set_B(float off) { /* no-op */ }
#elif GAIT_MODE == GAIT_PACE
static void set_A(float off) { set_LF(off); set_LR(off); }   /* 左侧 */
static void set_B(float off) { set_RF(off); set_RR(off); }   /* 右侧 */
#endif

/* ===================== 两组同步插值 ===================== */
static void dog_interp(float a_from, float a_to, uint32_t a_ms,
                       float b_from, float b_to, uint32_t b_ms)
{
    uint32_t total   = (a_ms > b_ms) ? a_ms : b_ms;
    uint32_t elapsed = 0;
    while (elapsed < total) {
        elapsed += DOG_FRAME_MS;
        if (elapsed > total) elapsed = total;

        float ta = (elapsed >= a_ms) ? 1.0f : (float)elapsed / (float)a_ms;
        float tb = (elapsed >= b_ms) ? 1.0f : (float)elapsed / (float)b_ms;

        set_A(a_from + (a_to - a_from) * ta);
        set_B(b_from + (b_to - b_from) * tb);
        vTaskDelay(pdMS_TO_TICKS(DOG_FRAME_MS));
    }
}

/* ===================== 一个步态周期 ===================== */
static void dog_walk_cycle(int dir)
{
    const float s = DOG_STEP_DEG;
    float p_from =  (float)dir * s;
    float p_to   = -(float)dir * s;

    dog_interp(p_from, p_to, DOG_PUSH_MS,  p_to,   p_from, DOG_RESET_MS);
    dog_interp(p_to,   p_from, DOG_RESET_MS, p_from, p_to, DOG_PUSH_MS);
}

static void dog_center(void)
{
    set_LF(0.0f); set_RF(0.0f); set_LR(0.0f); set_RR(0.0f);
}

/* =========================================================================
 *  app_main
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

    const char *gait_name =
#if GAIT_MODE == GAIT_DIAGONAL
        "DIAGONAL (对角)";
#elif GAIT_MODE == GAIT_BOUND
        "BOUND (前后交替/跑步式)";
#elif GAIT_MODE == GAIT_ALL
        "ALL (四腿同相)";
#elif GAIT_MODE == GAIT_PACE
        "PACE (左右同侧)";
#endif
    ESP_LOGI(TAG, "PCA9685 ready. GAIT = %s", gait_name);

    dog_center();
    vTaskDelay(pdMS_TO_TICKS(800));

    while (1) {
        ESP_LOGI(TAG, ">>> 前进 5 秒");
        TickType_t t0 = xTaskGetTickCount();
        while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(5000)) {
            dog_walk_cycle(+1);
        }
        dog_center();
        vTaskDelay(pdMS_TO_TICKS(2500));

        ESP_LOGI(TAG, ">>> 后退 5 秒");
        t0 = xTaskGetTickCount();
        while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(5000)) {
            dog_walk_cycle(-1);
        }
        dog_center();
        vTaskDelay(pdMS_TO_TICKS(2500));

        ESP_LOGI(TAG, ">>> 一轮结束");
    }
}