
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

/* ============================================================
 * 1. PCA9685 硬件配置
 * ============================================================ */

#define I2C_MASTER_SDA_IO       GPIO_NUM_8
#define I2C_MASTER_SCL_IO       GPIO_NUM_9

#define PCA9685_I2C_ADDR        0x40
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

#define PCA9685_MODE1           0x00
#define PCA9685_PRESCALE        0xFE
#define PCA9685_LED0_ON_L       0x06

#define MODE1_SLEEP             (1 << 4)
#define MODE1_AI                (1 << 5)
#define MODE1_RESTART           (1 << 7)

static const char *TAG = "SG90_CALIBRATION";

/* ============================================================
 * 2. SG90 标定参数
 * ============================================================ */

// PCA9685 输出通道
#define SG90_CHANNEL            0
    
// SG90 停止脉宽：沿用之前校准的数值
#define SG90_STOP_PULSE_US      1630

// 每增加 1 个速度单位，PWM 脉宽变化 10us
#define SG90_SPEED_STEP_US      10

// 速度死区：-5 ~ +5 均视为停止
#define SG90_DEADBAND           5

// 单次正式测量时长
#define CALIBRATION_DURATION_MS 10000

// 开始测量前的速度稳定时间
#define SPEED_STABILIZE_MS      1000

// 两次测试之间的停止等待时间
#define TEST_INTERVAL_MS       5000

// 开始标定前的等待时间
#define STARTUP_DELAY_MS       5000

// 限制标定速度，先不测试极限速度
#define MAX_CALIBRATION_SPEED  70

// 标定速度列表
static const int g_speed_points[] = {
    6,
    8,
    10,
    15,
    20,
    30,
    40,
    50,
    60,
    70
};

#define SPEED_POINT_COUNT \
    (sizeof(g_speed_points) / sizeof(g_speed_points[0]))

/* ============================================================
 * 3. PCA9685 底层 I2C 寄存器写入
 * ============================================================ */

static esp_err_t pca9685_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (PCA9685_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        1000 / portTICK_PERIOD_MS
    );

    i2c_cmd_link_delete(cmd);

    return ret;
}

/* ============================================================
 * 4. PCA9685 初始化
 * ============================================================ */

static esp_err_t pca9685_init(void)
{
    // 进入睡眠模式，准备设置 PWM 频率
    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(PCA9685_MODE1, MODE1_SLEEP),
        TAG,
        "Failed to enter sleep mode"
    );

    // 设置预分频器，目标 PWM 频率约为 50Hz
    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(PCA9685_PRESCALE, 0x79),
        TAG,
        "Failed to set prescale"
    );

    // 开启自动递增和重新启动
    uint8_t mode1 = MODE1_AI | MODE1_RESTART;

    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(PCA9685_MODE1, mode1),
        TAG,
        "Failed to configure MODE1"
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "PCA9685 initialized successfully.");
    ESP_LOGI(TAG, "PWM frequency: approximately 50Hz");

    return ESP_OK;
}

/* ============================================================
 * 5. 设置 PCA9685 指定通道的 PWM 脉宽
 *
 * 50Hz 下：
 * 1 tick ≈ 4.875us
 * ============================================================ */

static esp_err_t pca9685_set_pulse_us(
    uint8_t channel,
    uint16_t pulse_us
)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    if (pulse_us == 0 || pulse_us > 20000) {
        return ESP_ERR_INVALID_ARG;
    }

    // 将脉宽转换成 PCA9685 的 12 位计数值
    uint16_t ticks = (uint16_t)(
        (pulse_us / 4.875f) + 0.5f
    );

    if (ticks > 4095) {
        ticks = 4095;
    }

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;

    // ON = 0
    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(reg + 0, 0),
        TAG,
        "Failed to write ON_L"
    );

    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(reg + 1, 0),
        TAG,
        "Failed to write ON_H"
    );

    // OFF = ticks
    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(reg + 2, ticks & 0xFF),
        TAG,
        "Failed to write OFF_L"
    );

    ESP_RETURN_ON_ERROR(
        pca9685_write_reg(
            reg + 3,
            (ticks >> 8) & 0x0F
        ),
        TAG,
        "Failed to write OFF_H"
    );

    return ESP_OK;
}

/* ============================================================
 * 6. SG90 速度控制函数
 *
 * speed > 0：正转
 * speed < 0：反转
 * speed = 0：停止
 *
 * 速度范围：-100 ~ +100
 * ============================================================ */

static esp_err_t sg90_set_speed(int speed)
{
    // 限制速度范围
    if (speed > 100) {
        speed = 100;
    }

    if (speed < -100) {
        speed = -100;
    }

    // 死区处理
    if (speed >= -SG90_DEADBAND &&
        speed <= SG90_DEADBAND) {

        speed = 0;
    }

    int32_t pulse_us;

    if (speed == 0) {

        pulse_us = SG90_STOP_PULSE_US;

    } else {

        // 使用有符号整数计算，避免 uint16_t 负数溢出问题
        pulse_us =
            (int32_t)SG90_STOP_PULSE_US +
            (int32_t)speed * SG90_SPEED_STEP_US;
    }

    // 安全检查，避免输出过宽的脉冲
    if (pulse_us < 500 || pulse_us > 2500) {

        ESP_LOGE(
            TAG,
            "Unsafe pulse width: %ld us",
            (long)pulse_us
        );

        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = pca9685_set_pulse_us(
        SG90_CHANNEL,
        (uint16_t)pulse_us
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo speed.");
        return ret;
    }

    if (speed == 0) {

        ESP_LOGI(
            TAG,
            "SG90 STOP | Pulse: %ld us",
            (long)pulse_us
        );

    } else {

        ESP_LOGI(
            TAG,
            "SG90 %s | Speed: %d | Pulse: %ld us",
            speed > 0 ? "FWD" : "REV",
            speed > 0 ? speed : -speed,
            (long)pulse_us
        );
    }

    return ESP_OK;
}

/* ============================================================
 * 7. 单次速度标定测试
 *
 * 测试流程：
 *
 * 1. 先停止舵机
 * 2. 等待舵机稳定
 * 3. 设置指定速度
 * 4. 等待速度稳定
 * 5. 开始正式测量
 * 6. 持续运行指定时间
 * 7. 自动停止
 * 8. 输出实际测量时间
 * ============================================================ */

static esp_err_t run_calibration_test(int speed)
{
    const char *direction;

    if (speed > 0) {
        direction = "FWD";
    } else if (speed < 0) {
        direction = "REV";
    } else {
        direction = "STOP";
    }

    ESP_LOGI(
        TAG,
        "\n================================================"
    );

    ESP_LOGI(
        TAG,
        "PREPARING TEST | Direction: %s | Speed: %d",
        direction,
        speed
    );

    // 先停止舵机
    ESP_RETURN_ON_ERROR(
        sg90_set_speed(0),
        TAG,
        "Failed to stop servo"
    );

    vTaskDelay(
        pdMS_TO_TICKS(1000)
    );

    // 设置测试速度
    ESP_RETURN_ON_ERROR(
        sg90_set_speed(speed),
        TAG,
        "Failed to set test speed"
    );

    // 等待速度稳定
    vTaskDelay(
        pdMS_TO_TICKS(SPEED_STABILIZE_MS)
    );

    // 正式测量开始
    ESP_LOGI(
        TAG,
        "MEASURE START | DIR=%s | SPEED=%d | TARGET=%d ms",
        direction,
        speed,
        CALIBRATION_DURATION_MS
    );

    int64_t start_time_us = esp_timer_get_time();

    // 保持指定速度运行
    vTaskDelay(
        pdMS_TO_TICKS(CALIBRATION_DURATION_MS)
    );

    int64_t end_time_us = esp_timer_get_time();

    // 计算真实经过时间
    int64_t actual_elapsed_us =
        end_time_us - start_time_us;

    float actual_elapsed_ms =
        actual_elapsed_us / 1000.0f;

    // 测量结束，立即停止舵机
    esp_err_t ret = sg90_set_speed(0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop servo after measurement!");
        return ret;
    }

    // 输出测量结果
    ESP_LOGI(
        TAG,
        "MEASURE END | DIR=%s | SPEED=%d | ACTUAL_TIME=%.2f ms",
        direction,
        speed,
        actual_elapsed_ms
    );

    ESP_LOGI(
        TAG,
        "Please record the number of revolutions from video."
    );

    ESP_LOGI(
        TAG,
        "================================================\n"
    );

    // 两次测试之间等待
    vTaskDelay(
        pdMS_TO_TICKS(TEST_INTERVAL_MS)
    );

    return ESP_OK;
}

/* ============================================================
 * 8. 完整速度标定流程
 *
 * 先测试正转，再测试反转
 * ============================================================ */

static void run_speed_calibration(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "****************************************");
    ESP_LOGI(TAG, "* SG90 SPEED CALIBRATION START");
    ESP_LOGI(TAG, "* PCA9685 CHANNEL: %d", SG90_CHANNEL);
    ESP_LOGI(TAG, "* TEST DURATION: %d ms",
             CALIBRATION_DURATION_MS);
    ESP_LOGI(TAG, "* SPEED POINTS: %d",
             (int)SPEED_POINT_COUNT);
    ESP_LOGI(TAG, "****************************************");

    int total_tests = SPEED_POINT_COUNT * 2;
    int current_test = 0;

    // --------------------------------------------------------
    // 第一阶段：正转测试
    // --------------------------------------------------------

    ESP_LOGI(TAG, "\n========== FORWARD CALIBRATION ==========");

    for (int i = 0; i < SPEED_POINT_COUNT; i++) {

        int speed = g_speed_points[i];

        current_test++;

        ESP_LOGI(
            TAG,
            "TEST %d / %d",
            current_test,
            total_tests
        );

        if (speed <= SG90_DEADBAND ||
            speed > MAX_CALIBRATION_SPEED) {
            ESP_LOGW(TAG, "Skipping invalid speed: %d", speed);
            continue;
        }

        if (run_calibration_test(speed) != ESP_OK) {
            ESP_LOGE(TAG, "Forward calibration aborted.");
            return;
        }
    }

    // --------------------------------------------------------
    // 第二阶段：反转测试
    // --------------------------------------------------------

    ESP_LOGI(TAG, "\n========== REVERSE CALIBRATION ==========");

    for (int i = 0; i < SPEED_POINT_COUNT; i++) {

        int speed = -g_speed_points[i];

        current_test++;

        ESP_LOGI(
            TAG,
            "TEST %d / %d",
            current_test,
            total_tests
        );

        if (-speed <= SG90_DEADBAND ||
            -speed > MAX_CALIBRATION_SPEED) {
            ESP_LOGW(TAG, "Skipping invalid speed: %d", speed);
            continue;
        }

        if (run_calibration_test(speed) != ESP_OK) {
            ESP_LOGE(TAG, "Reverse calibration aborted.");
            return;
        }
    }

    // --------------------------------------------------------
    // 标定结束
    // --------------------------------------------------------

    sg90_set_speed(0);

    ESP_LOGI(TAG, "\n****************************************");
    ESP_LOGI(TAG, "* SG90 CALIBRATION COMPLETED!");
    ESP_LOGI(TAG, "* ALL TESTS FINISHED.");
    ESP_LOGI(TAG, "****************************************");
}

/* ============================================================
 * 9. ESP-IDF 程序入口
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,

        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(
        i2c_param_config(I2C_MASTER_NUM, &conf)
    );

    ESP_ERROR_CHECK(
        i2c_driver_install(
            I2C_MASTER_NUM,
            conf.mode,
            0,
            0,
            0
        )
    );

    // 初始化 PCA9685
    if (pca9685_init() != ESP_OK) {

        ESP_LOGE(TAG, "PCA9685 initialization failed!");

        return;
    }

    // 启动后先停止舵机
    if (sg90_set_speed(0) != ESP_OK) {

        ESP_LOGE(TAG, "Initial servo stop failed!");

        return;
    }

    ESP_LOGI(TAG, "PCA9685 READY!");

    ESP_LOGI(
        TAG,
        "Calibration starts in %d ms...",
        STARTUP_DELAY_MS
    );

    // 给用户准备时间
    vTaskDelay(
        pdMS_TO_TICKS(STARTUP_DELAY_MS)
    );

    // 执行完整标定
    run_speed_calibration();

    // 确保最终停止
    sg90_set_speed(0);

    ESP_LOGI(TAG, "Application finished.");
}
