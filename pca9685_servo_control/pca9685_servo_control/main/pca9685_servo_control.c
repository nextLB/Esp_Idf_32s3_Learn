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

/* ============================================================
 * SG90 固定角度旋转控制
 *   基于实测「速度 → 角速度」表 + 分段线性插值
 *   当前表数据：通道 0 实测（10s 转的圈数折算）
 *   后续扩充：只需往表里加行，按 speed 升序，无需改函数
 * ============================================================ */

#define SG90_MIN_SPEED      6       // 标定表最低速度
#define SG90_MAX_SPEED      20      // 当前实测最高速度，后续补表后改大

/* ---- 补偿参数（经验初值，实测后再微调） ---- */
#define SG90_ACCEL_COMP_MS  0     // 启动加速补偿  单位ms
#define SG90_COAST_DEG      0.0f   // 停止滑行补偿  单位度

/* 时间修正系数：单次短动作平均效率只有稳态的 ~82%
 * 实测：90° 实际/理论=0.778，360° 实际/理论=0.833
 * 取 1/0.82 ≈ 1.22，后续微调 */
#define SG90_TIME_CORRECTION   1.22f


typedef struct {
    int   speed;    // 速度绝对值
    float omega;    // 角速度 °/s
} omega_point_t;

/* ---- 正转实测表（speed 必须升序）---- */
static const omega_point_t g_fwd_omega_table[] = {
    {  6, 142.0f },
    {  8, 154.0f },
    { 10, 173.0f },
    { 15, 214.0f },
    { 20, 220.0f },
    /* 补齐 30/40/50/60/70 后在这里继续加 */
};
#define FWD_TABLE_LEN (sizeof(g_fwd_omega_table) / sizeof(g_fwd_omega_table[0]))

/* ---- 反转实测表（暂用正转值占位，实测后替换）---- */
static const omega_point_t g_rev_omega_table[] = {
    {  6, 142.0f },
    {  8, 154.0f },
    { 10, 173.0f },
    { 15, 214.0f },
    { 20, 220.0f },
};
#define REV_TABLE_LEN (sizeof(g_rev_omega_table) / sizeof(g_rev_omega_table[0]))

/* 分段线性插值：任意 speed 都能算，超范围自动 clamp */
static float omega_lookup(const omega_point_t *tab, int len, int speed)
{
    if (speed <= tab[0].speed)         return tab[0].omega;
    if (speed >= tab[len - 1].speed)   return tab[len - 1].omega;

    for (int i = 0; i < len - 1; i++) {
        if (speed >= tab[i].speed && speed <= tab[i + 1].speed) {
            float t = (float)(speed - tab[i].speed)
                    / (float)(tab[i + 1].speed - tab[i].speed);
            return tab[i].omega + t * (tab[i + 1].omega - tab[i].omega);
        }
    }
    return tab[len - 1].omega;
}




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


/**
 * @brief  让指定通道的 SG90 按速度旋转指定角度后自动停止
 * @param  channel     PCA9685 通道（0~15）
 * @param  speed       有符号速度：正=正转，负=反转；绝对值超出 [6,20] 会被 clamp
 * @param  angle_deg   目标角度（度），必须 > 0
 * @return ESP_OK / ESP_ERR_INVALID_ARG
 */
static esp_err_t sg90_rotate_angle(uint8_t channel, int speed, float angle_deg)
{
    if (channel > 15) {
        ESP_LOGE(TAG, "ROTATE: invalid channel %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    if (angle_deg <= 0.0f) {
        ESP_LOGE(TAG, "ROTATE: invalid angle %.1f", angle_deg);
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. 拆方向与速度绝对值 */
    int dir      = (speed >= 0) ? 1 : -1;
    int abs_speed = (speed >= 0) ? speed : -speed;

    /* 2. clamp 到标定范围 */
    if (abs_speed < SG90_MIN_SPEED) {
        ESP_LOGW(TAG, "ROTATE: speed %d < min %d, clamp",
                 abs_speed, SG90_MIN_SPEED);
        abs_speed = SG90_MIN_SPEED;
    }
    if (abs_speed > SG90_MAX_SPEED) {
        ESP_LOGW(TAG, "ROTATE: speed %d > max %d, clamp",
                 abs_speed, SG90_MAX_SPEED);
        abs_speed = SG90_MAX_SPEED;
    }

    /* 3. 查表得角速度 */
    const omega_point_t *tab = (dir > 0) ? g_fwd_omega_table : g_rev_omega_table;
    int   len   = (dir > 0) ? FWD_TABLE_LEN : REV_TABLE_LEN;
    float omega = omega_lookup(tab, len, abs_speed);

    /* 4. 计算时间：目标角度时间 + 加速补偿 - 滑行补偿 */
    float t_rotate_ms = angle_deg / omega * 1000.0f * SG90_TIME_CORRECTION;   // ← 乘系数
    float t_coast_ms  = SG90_COAST_DEG / omega * 1000.0f;
    float t_total_ms  = t_rotate_ms + SG90_ACCEL_COMP_MS - t_coast_ms;
    if (t_total_ms < 10.0f) t_total_ms = 10.0f;

    /* 5. 脉宽安全检查 */
    int32_t pulse_us = SG90_STOP_PULSE_US + dir * abs_speed * 10;
    if (pulse_us < 500 || pulse_us > 2500) {
        ESP_LOGE(TAG, "ROTATE: unsafe pulse %ldus", (long)pulse_us);
        return ESP_ERR_INVALID_ARG;
    }

    /* 6. 执行 */
    ESP_LOGI(TAG, "ROTATE CH%d | %s spd=%d ω=%.1f°/s ang=%.1f° t=%.0fms pulse=%ldus",
             channel, dir > 0 ? "FWD" : "REV", abs_speed, omega,
             angle_deg, t_total_ms, (long)pulse_us);

    pca9685_set_pulse_us(channel, (uint16_t)pulse_us);
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(t_total_ms + 0.5f)));
    pca9685_set_pulse_us(channel, SG90_STOP_PULSE_US);

    ESP_LOGI(TAG, "ROTATE CH%d done", channel);
    return ESP_OK;
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

        // 这里是速度加时间的调控
        servo_ctrl(0, SERVO_TYPE_SG90, 50, 1000);  
        vTaskDelay(pdMS_TO_TICKS(10000));           

        // servo_ctrl(0, SERVO_TYPE_SG90, -50, 1000); 
        // vTaskDelay(pdMS_TO_TICKS(1000));


        // 这里是速度加角度的调控

        /* 正转 90°，speed=10 */
        sg90_rotate_angle(0, +10, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(10000));

        /* 反转 90°，回到原位附近 */
        sg90_rotate_angle(0, -10, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(10000));

        /* 正转一整圈 360°，speed=15 */
        sg90_rotate_angle(0, +15, 360.0f);
        vTaskDelay(pdMS_TO_TICKS(10000));

        /* 反转 180° */
        sg90_rotate_angle(0, -15, 180.0f);
        vTaskDelay(pdMS_TO_TICKS(10000));

        /* 低速小角度测试：speed=6，转 45° */
        sg90_rotate_angle(0, +6, 45.0f);
        vTaskDelay(pdMS_TO_TICKS(10000));

        
        
        /* 正转 90°，speed=10 */
        sg90_rotate_angle(8, +10, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 反转 90°，回到原位附近 */
        sg90_rotate_angle(8, -10, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 正转一整圈 360°，speed=15 */
        sg90_rotate_angle(8, +15, 360.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 反转 180° */
        sg90_rotate_angle(8, -15, 180.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 低速小角度测试：speed=6，转 45° */
        sg90_rotate_angle(8, +6, 45.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));



        /* 正转 90°，speed=10 */
        sg90_rotate_angle(4, +10, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 反转 90°，回到原位附近 */
        sg90_rotate_angle(4, -10, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 正转一整圈 360°，speed=15 */
        sg90_rotate_angle(4, +15, 360.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 反转 180° */
        sg90_rotate_angle(4, -15, 180.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 低速小角度测试：speed=6，转 45° */
        sg90_rotate_angle(4, +6, 45.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));




        // servo_ctrl(4, SERVO_TYPE_SG90, 50, 1000);  
        // vTaskDelay(pdMS_TO_TICKS(1000));           

        // servo_ctrl(4, SERVO_TYPE_SG90, -50, 1000); 
        // vTaskDelay(pdMS_TO_TICKS(1000));


        // servo_ctrl(8, SERVO_TYPE_SG90, 50, 1000);  
        // vTaskDelay(pdMS_TO_TICKS(1000));           

        // servo_ctrl(8, SERVO_TYPE_SG90, -50, 1000); 
        // vTaskDelay(pdMS_TO_TICKS(1000));
        


        // // 1. 转到 0 度（通常是最左边/最右边）
        // servo_ctrl(4, SERVO_TYPE_SF006, 0, 0);
        // vTaskDelay(pdMS_TO_TICKS(1000)); // 等 1 秒让它转到位

        // // 2. 转到中位 90 度
        // servo_ctrl(4, SERVO_TYPE_SF006, 90, 0);
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // // 3. 转到 180 度（另一侧极限）
        // servo_ctrl(4, SERVO_TYPE_SF006, 180, 0);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        
        // // 4. 回到中位，准备下一轮
        // servo_ctrl(4, SERVO_TYPE_SF006, 90, 0);
        // vTaskDelay(pdMS_TO_TICKS(2000)); // 停 2 秒再循环
        
        ESP_LOGI(TAG, "--- Test Loop Done ---");
    }
}