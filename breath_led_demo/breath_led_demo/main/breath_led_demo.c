#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

/* ==================== N16R8 板载 WS2812 配置 ==================== */
#define LED_STRIP_GPIO      GPIO_NUM_48   /* 不亮可改 GPIO_NUM_38 */
#define LED_STRIP_LED_COUNT 1             /* 板载只有 1 颗 */

#define FADE_STEP_MS        15            /* 每步渐变间隔(ms)，越小呼吸越快 */
#define FADE_STEP_SIZE      2             /* 每步亮度变化，越小越平滑 */
#define MAX_BRIGHTNESS      80            /* 最大亮度 0-255，避免刺眼 */
/* =============================================================== */

static const char *TAG = "ws2812_breath";
static led_strip_handle_t led_strip;

/* 颜色表：RGB 分量 + 名字（方便串口观察） */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    const char *name;
} led_color_t;

static const led_color_t colors[] = {
    {255,   0,   0, "Red"},
    {  0, 255,   0, "Green"},
    {  0,   0, 255, "Blue"},
    {255, 255,   0, "Yellow"},
    {  0, 255, 255, "Cyan"},
    {255,   0, 255, "Magenta"},
    {255, 255, 255, "White"},
};
#define COLOR_COUNT (sizeof(colors) / sizeof(colors[0]))

/* 按 brightness(0~MAX_BRIGHTNESS) 等比例缩放颜色并刷新 */
static void set_color(const led_color_t *c, uint8_t brightness)
{
    uint8_t r = (uint16_t)c->r * brightness / MAX_BRIGHTNESS;
    uint8_t g = (uint16_t)c->g * brightness / MAX_BRIGHTNESS;
    uint8_t b = (uint16_t)c->b * brightness / MAX_BRIGHTNESS;

    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void app_main(void)
{
    /* ---------- 初始化 WS2812（RMT 后端） ---------- */
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_STRIP_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));

    ESP_LOGI(TAG, "Color-cycling breath on GPIO%d, %d colors",
             LED_STRIP_GPIO, (int)COLOR_COUNT);

    size_t idx = 0;

    while (1) {
        const led_color_t *c = &colors[idx];
        ESP_LOGI(TAG, "Color: %s", c->name);

        /* 1. 渐亮：0 -> MAX_BRIGHTNESS */
        for (int b = 0; b <= MAX_BRIGHTNESS; b += FADE_STEP_SIZE) {
            set_color(c, (uint8_t)b);
            vTaskDelay(pdMS_TO_TICKS(FADE_STEP_MS));
        }
        set_color(c, MAX_BRIGHTNESS);   /* 确保到达最亮 */

        /* 2. 渐暗：MAX_BRIGHTNESS -> 0 */
        for (int b = MAX_BRIGHTNESS; b >= 0; b -= FADE_STEP_SIZE) {
            set_color(c, (uint8_t)b);
            vTaskDelay(pdMS_TO_TICKS(FADE_STEP_MS));
        }
        set_color(c, 0);                /* 确保完全熄灭 */

        /* 3. 熄灭后切换到下一种颜色 */
        idx = (idx + 1) % COLOR_COUNT;

        vTaskDelay(pdMS_TO_TICKS(100)); /* 全灭后稍停一下，切换更明显 */
    }
}