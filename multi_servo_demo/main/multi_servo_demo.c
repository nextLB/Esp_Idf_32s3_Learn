#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

/* ==================== 根据你的接线修改 ==================== */
#define I2C_MASTER_SDA_IO   GPIO_NUM_8      /* SDA接GPIO8 */
#define I2C_MASTER_SCL_IO   GPIO_NUM_9      /* SCL接GPIO9 */
#define PCA9685_I2C_ADDR    0x40            /* PCA9685默认地址 */
/* ========================================================= */

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

/* ==================== 通用测试参数 ==================== */
#define TEST_ALL_CHANNELS   16              /* 测试全部 16 个通道，插哪都覆盖 */
#define SWEEP_STEP_DEG      3               /* 每步角度，越小越平滑 */
#define SWEEP_STEP_MS       10              /* 每步间隔(ms)，越小越快 */
#define SWEEP_PAUSE_MS      300             /* 每个通道测试后的停顿 */
/* ===================================================== */

static const char *TAG = "PCA9685_Test";

/* PCA9685 寄存器地址 */
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

#define MODE1_SLEEP         (1 << 4)
#define MODE1_AI            (1 << 5)
#define MODE1_RESTART       (1 << 7)

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
    if (pca9685_write_reg(PCA9685_PRESCALE, 0x79) != ESP_OK) return ESP_FAIL;   /* 50Hz */
    uint8_t mode1 = MODE1_AI | MODE1_RESTART;
    if (pca9685_write_reg(PCA9685_MODE1, mode1) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/* 不打印日志的底层写角度函数 */
static void servo_set_angle_raw(uint8_t channel, int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    uint16_t pulse = 102 + (uint16_t)((512 - 102) * angle / 180.0f);

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    pca9685_write_reg(reg + 0, 0);
    pca9685_write_reg(reg + 1, 0);
    pca9685_write_reg(reg + 2, pulse & 0xFF);
    pca9685_write_reg(reg + 3, (pulse >> 8) & 0x0F);
}

/* 平滑扫动 from -> to */
static void servo_sweep_smooth(uint8_t channel, int from, int to)
{
    int dir = (to >= from) ? SWEEP_STEP_DEG : -SWEEP_STEP_DEG;
    int a = from;

    while (1) {
        servo_set_angle_raw(channel, a);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_STEP_MS));

        if ((dir > 0 && a + dir >= to) || (dir < 0 && a + dir <= to)) {
            break;
        }
        a += dir;
    }
    servo_set_angle_raw(channel, to);
}

void app_main(void)
{
    /* ---------- 1. 初始化I2C ---------- */
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

    /* ---------- 2. 初始化PCA9685 ---------- */
    ESP_LOGI(TAG, "Initializing PCA9685...");
    if (pca9685_init() != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed! Check SDA=GPIO8 / SCL=GPIO9");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 ready at 50Hz.");
    ESP_LOGI(TAG, "Universal test: CH0~CH%d, step=%d deg/%d ms",
             TEST_ALL_CHANNELS - 1, SWEEP_STEP_DEG, SWEEP_STEP_MS);

    /* ---------- 3. 循环测试全部 16 个通道 ---------- */
    while (1) {
        for (int ch = 0; ch < TEST_ALL_CHANNELS; ch++) {
            ESP_LOGI(TAG, "===== CH%d: 0 -> 180 -> 0 =====", ch);

            /* 先归零 */
            servo_sweep_smooth(ch, 0, 0);

            /* 0° -> 180° */
            servo_sweep_smooth(ch, 0, 180);
            /* 180° -> 0° */
            servo_sweep_smooth(ch, 180, 0);

            vTaskDelay(pdMS_TO_TICKS(SWEEP_PAUSE_MS));
        }
        ESP_LOGI(TAG, "Full 0~15 round done. Repeating...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}