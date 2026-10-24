#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define PCA9685_I2C_ADDR    0x40
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

static const char *TAG = "PCA9685";

#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define MODE1_SLEEP         (1 << 4)
#define MODE1_AI            (1 << 5)
#define MODE1_RESTART       (1 << 7)

typedef enum {
    SERVO_TYPE_SF006 = 0,
    SERVO_TYPE_SG90,
} servo_type_t;

/* =========================================================================
 * ★ 终极校准区 ★
 * -------------------------------------------------------------------------
 * PCA9685 在 50Hz 下的精度是 4.875us/格。
 * 如果你发现 STOP 时舵机依然慢速转动，请以 5 为单位修改下面的值。
 * ========================================================================= */
#define SG90_STOP_PULSE_US  1630

/* =========================================================================
 * 底层写入函数
 * ========================================================================= */
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
    if (pca9685_write_reg(PCA9685_PRESCALE, 0x79) != ESP_OK) return ESP_FAIL; // 50Hz
    uint8_t mode1 = MODE1_AI | MODE1_RESTART;
    if (pca9685_write_reg(PCA9685_MODE1, mode1) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/* 
 * 精确计算：50.08Hz 频率下，1 tick = 4.875us
 */
static void pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (channel > 15) return;

    // 使用真实的 4.875us 进行浮点运算，四舍五入到最近的整数 tick
    uint16_t ticks = (uint16_t)((pulse_us / 4.875f) + 0.5f);

    if (ticks > 4095) ticks = 4095;

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    pca9685_write_reg(reg + 0, 0);
    pca9685_write_reg(reg + 1, 0);
    pca9685_write_reg(reg + 2, ticks & 0xFF);
    pca9685_write_reg(reg + 3, (ticks >> 8) & 0x0F);
}

static void servo_ctrl(uint8_t channel, servo_type_t type, int param1, int param2)
{
    if (channel > 15) return;

    if (type == SERVO_TYPE_SF006) {
        int angle = param1;
        if (angle < 0)   angle = 0;
        if (angle > 180) angle = 180;

        uint16_t pulse_us = 500 + (uint16_t)((2500 - 500) * angle / 180.0f);
        pca9685_set_pulse_us(channel, pulse_us);
        ESP_LOGI(TAG, "CH%02d [SF006] -> %3d deg (Pulse: %d us)", channel, angle, pulse_us);
    } 
    else if (type == SERVO_TYPE_SG90) {
        int speed = param1;
        int duration_ms = param2;

        if (speed > 100)  speed = 100;
        if (speed < -100) speed = -100;

        // 加大死区处理：如果速度极低，直接当0处理，防止堵转发热
        if (speed >= -5 && speed <= 5) speed = 0;

        uint16_t pulse_us = 0;
        
        if (speed == 0) {
            pulse_us = SG90_STOP_PULSE_US; 
        } else {
            // 速度映射：每 1% 速度，偏移 10us
            pulse_us = SG90_STOP_PULSE_US + (uint16_t)(speed * 10);
        }
        
        pca9685_set_pulse_us(channel, pulse_us);
        
        // 打印出 PCA9685 真正输出的 tick 数，让你看清真相
        uint16_t real_ticks = (uint16_t)((pulse_us / 4.875f) + 0.5f);
        float real_us = real_ticks * 4.875f;
        
        if (speed == 0) {
            ESP_LOGI(TAG, "CH%02d [SG90]  -> STOP (Target: %d us, Actual: %.1f us)", 
                     channel, pulse_us, real_us);
        } else {
            ESP_LOGI(TAG, "CH%02d [SG90]  -> %s %d%% (Actual: %.1f us)", 
                     channel, (speed > 0 ? "FWD" : "REV"), (speed > 0 ? speed : -speed), real_us);
        }

        if (duration_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
            pca9685_set_pulse_us(channel, SG90_STOP_PULSE_US); 
            ESP_LOGI(TAG, "CH%02d [SG90]  -> STOP (Time up)", channel);
        }
    } 
}

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
    ESP_LOGI(TAG, "PCA9685 ready. Start testing...");

    while (1) {
        servo_ctrl(0, SERVO_TYPE_SG90, 50, 1000);  
        vTaskDelay(pdMS_TO_TICKS(1000));           

        servo_ctrl(0, SERVO_TYPE_SG90, -50, 1000); 
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 1. 转到 0 度（通常是最左边/最右边）
        servo_ctrl(4, SERVO_TYPE_SF006, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 等 1 秒让它转到位

        // 2. 转到中位 90 度
        servo_ctrl(4, SERVO_TYPE_SF006, 90, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 3. 转到 180 度（另一侧极限）
        servo_ctrl(4, SERVO_TYPE_SF006, 180, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 4. 回到中位，准备下一轮
        servo_ctrl(4, SERVO_TYPE_SF006, 90, 0);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 停 2 秒再循环
        
        ESP_LOGI(TAG, "--- Test Loop Done ---");
    }
}