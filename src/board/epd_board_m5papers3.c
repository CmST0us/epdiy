/**
 * @file epd_board_m5papers3.c
 * @brief M5PaperS3 epdiy 板级定义。
 *
 * 参考自 vroland/epdiy PR #392 (M5PaperS3 支持)。
 * 定义 EPD 控制引脚、数据总线和电源管理序列。
 */

#include <stdint.h>
#include "epd_board.h"
#include "epdiy.h"

#include "../output_common/render_method.h"
#include "../output_lcd/lcd_driver.h"

#include "esp_log.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* EPD 控制引脚 */
#define EPD_SPV     GPIO_NUM_17
#define EPD_EN      GPIO_NUM_45
#define BST_EN      GPIO_NUM_46
#define EPD_XLE     GPIO_NUM_15

/* 时序信号 */
#define CKV         GPIO_NUM_18
#define STH         GPIO_NUM_13
#define CKH         GPIO_NUM_16

/* 8-bit 数据总线 */
#define D0          GPIO_NUM_6
#define D1          GPIO_NUM_14
#define D2          GPIO_NUM_7
#define D3          GPIO_NUM_12
#define D4          GPIO_NUM_9
#define D5          GPIO_NUM_11
#define D6          GPIO_NUM_8
#define D7          GPIO_NUM_10

static lcd_bus_config_t lcd_config = {
    .clock = CKH,
    .ckv = CKV,
    .leh = EPD_XLE,
    .start_pulse = STH,
    .stv = EPD_SPV,
    .data = {D0, D1, D2, D3, D4, D5, D6, D7},
};

/**
 * @brief 初始化 EPD 板级硬件和 LCD 外设。
 */
static void epd_board_init(uint32_t epd_row_width) {
    gpio_hold_dis(CKH);

    gpio_set_direction(EPD_SPV, GPIO_MODE_OUTPUT);
    gpio_set_direction(EPD_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BST_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(EPD_XLE, GPIO_MODE_OUTPUT);

    gpio_set_level(EPD_XLE, 0);
    gpio_set_level(EPD_SPV, 1);
    gpio_set_level(EPD_EN, 0);
    gpio_set_level(BST_EN, 0);

    const EpdDisplay_t *display = epd_get_display();

    LcdEpdConfig_t config = {
        .pixel_clock = display->bus_speed * 1000 * 1000,
        .ckv_high_time = 60,
        .line_front_porch = 4,
        .le_high_time = 4,
        .bus_width = display->bus_width,
        .bus = lcd_config,
    };
    epd_lcd_init(&config, display->width, display->height);
}

/**
 * @brief 反初始化 EPD 板级硬件。
 */
static void epd_board_deinit(void) {
    epd_lcd_deinit();

    gpio_set_level(EPD_XLE, 0);
    gpio_set_level(EPD_SPV, 0);
    gpio_set_level(EPD_EN, 0);
    gpio_set_level(BST_EN, 0);
}

/**
 * @brief 设置 EPD 控制信号状态。
 */
static void epd_board_set_ctrl(epd_ctrl_state_t *state,
                               const epd_ctrl_state_t *const mask) {
    if (state->ep_sth) {
        gpio_set_level(STH, 1);
    } else {
        gpio_set_level(STH, 0);
    }

    if (state->ep_stv) {
        gpio_set_level(EPD_SPV, 1);
    } else {
        gpio_set_level(EPD_SPV, 0);
    }

    if (state->ep_latch_enable) {
        gpio_set_level(EPD_XLE, 1);
    } else {
        gpio_set_level(EPD_XLE, 0);
    }
}

/**
 * @brief EPD 上电序列: EN → BST → SPV/STH。
 */
static void epd_board_poweron(epd_ctrl_state_t *state) {
    gpio_set_level(EPD_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(BST_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(EPD_SPV, 1);
    gpio_set_level(STH, 1);
}

/**
 * @brief EPD 下电序列: BST → EN → SPV。
 */
static void epd_board_poweroff(epd_ctrl_state_t *state) {
    gpio_set_level(BST_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(EPD_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(EPD_SPV, 0);
}

/**
 * @brief 返回环境温度（固定值，M5PaperS3 无温度传感器在 EPD 驱动板上）。
 */
static float epd_board_ambient_temperature(void) {
    return 20.0f;
}

const EpdBoardDefinition epd_board_m5papers3 = {
    .init = epd_board_init,
    .deinit = epd_board_deinit,
    .set_ctrl = epd_board_set_ctrl,
    .poweron = epd_board_poweron,
    .poweroff = epd_board_poweroff,
    .get_temperature = epd_board_ambient_temperature,
    .set_vcom = NULL,
    .gpio_set_direction = NULL,
    .gpio_read = NULL,
    .gpio_write = NULL,
};
