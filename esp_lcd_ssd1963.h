/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2023 Daniel Austin <me@dan.me.uk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#pragma once

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_vendor.h"

/* commands for SSD1963 (skipping read commands) */
#define SSD1963_CMD_NOOP			0x00
#define SSD1963_CMD_SWRESET			0x01
#define SSD1963_CMD_SLEEP			0x10
#define SSD1963_CMD_END_SLEEP		0x11
#define SSD9163_CMD_PARTIAL_MODE	0x12
#define SSD1963_CMD_NORMAL_MODE		0x13
#define SSD1963_CMD_INV_OFF			0x20
#define SSD1963_CMD_INV_ON			0x21
#define SSD1963_CMD_SET_GAMMA_CURVE	0x26
#define SSD1963_CMD_DISP_OFF		0x28
#define SSD1963_CMD_DISP_ON			0x29
#define SSD1963_CMD_SET_COL_ADDR	0x2A
#define SSD1963_CMD_SET_PAGE_ADDR	0x2B
#define SSD1963_CMD_WRITE_MEM		0x2C
#define SSD1963_CMD_SET_PARTIAL		0x30
#define SSD1963_CMD_SET_SCROLL		0x33
#define SSD1963_CMD_SET_TEAR_OFF	0x34
#define SSD1963_CMD_SET_TEAR_ON		0x35
#define SSD1963_CMD_SET_ADDR_MODE	0x36
#define SSD1963_CMD_SET_SCROLL_ST	0x37
#define SSD1963_CMD_EXIT_IDLE		0x38
#define SSD1963_CMD_ENTER_IDLE		0x39
#define SSD1963_CMD_SET_PIXEL_FMT   0x3A
#define SSD1963_CMD_WRITE_MEM_CONT	0x3C
#define SSD1963_CMD_SET_TEAR_SCAN	0x44
#define SSD1963_CMD_SET_LCD_MODE	0xB0
#define SSD1963_CMD_SET_HOR_PERIOD	0xB4
#define SSD1963_CMD_SET_VER_PERIOD	0xB6
#define SSD1963_CMD_SET_GPIO_CONF	0xB8
#define SSD1963_CMD_SET_GPIO_VAL	0xBA
#define SSD1963_CMD_SET_POST_PROC	0xBC
#define SSD1963_CMD_SET_PWM_CONF	0xBE
#define SSD1963_CMD_SET_LCD_GEN0	0xC0
#define SSD1963_CMD_SET_LCD_GEN1	0xC2
#define SSD1963_CMD_SET_LCD_GEN2	0xC4
#define SSD1963_CMD_SET_LCD_GEN3	0xC6
#define SSD1963_CMD_SET_GPIO0_ROP   0xC8
#define SSD1963_CMD_SET_GPIO1_ROP   0xCA
#define SSD1963_CMD_SET_GPIO2_ROP   0xCC
#define SSD1963_CMD_SET_GPIO3_ROP   0xCE
#define SSD1963_CMD_SET_DBC_CONF	0xD0
#define SSD1963_CMD_SET_DBC_TH      0xD4
#define SSD1963_CMD_SET_PLL			0xE0
#define SSD1963_CMD_SET_PLLMN		0xE2
#define SSD1963_CMD_DEEP_SLEEP		0xE5
#define SSD1963_CMD_SET_LSH_FREQ	0xE6
#define SSD1963_CMD_SET_PIXEL_INTF	0xF0

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t                        lcd_width;          /*!< Width size of the LCD panel in pixels */
    uint16_t                        lcd_height;         /*!< Height size of the LCD panel in pixels */
    int                             mcu_bit_interface;  /*!< Selection between 8-bit and 16-bit MCU interface */
} esp_lcd_panel_ssd1963_config_t;

/**
 * @brief Create LCD panel for model SSD1963
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t esp_lcd_new_panel_ssd1963(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Update LCD panel backlight via SSD1963 PWM
 * 
 * @param[in] LCD panel handle
 * @param[in] backlight level (%)
 * @return
 *			- ESP_OK				on success
 */
esp_err_t ssd1963_set_backlight(esp_lcd_panel_t *panel, uint8_t level);

#ifdef __cplusplus
}
#endif