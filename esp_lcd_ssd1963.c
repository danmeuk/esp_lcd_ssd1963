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

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_ssd1963.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

static const char *TAG = "ssd1963";

static esp_err_t panel_ssd1963_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1963_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1963_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1963_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ssd1963_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1963_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1963_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1963_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1963_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
	esp_lcd_panel_t					base;
	esp_lcd_panel_io_handle_t		io;
	int								reset_gpio_num;
	bool							reset_level;
	int								x_gap;
	int								y_gap;
	uint16_t						width;
	uint16_t						height;
	unsigned int					bits_per_pixel;
	uint8_t							addr_mode;
	uint8_t							pixel_intf;
	bool							swap_axes;
	uint8_t							backlight;
} ssd1963_panel_t;

esp_err_t esp_lcd_new_panel_ssd1963(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t						ret = ESP_OK;
    ssd1963_panel_t					*ssd1963 = NULL;
    esp_lcd_panel_ssd1963_config_t	*vendor_cfg = (esp_lcd_panel_ssd1963_config_t *) panel_dev_config->vendor_config;

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(vendor_cfg, ESP_ERR_INVALID_ARG, err, TAG, "vendor config cannot be null");
    ssd1963 = calloc(1, sizeof(ssd1963_panel_t));
    ESP_GOTO_ON_FALSE(ssd1963, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1963 panel");

    /* RST GPIO config */
    if (panel_dev_config->reset_gpio_num != GPIO_NUM_NC)
    {
        gpio_config_t io_conf = {
	        .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
	        .pull_down_en = GPIO_PULLDOWN_DISABLE,
	        .pull_up_en = GPIO_PULLUP_DISABLE
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

	ssd1963->addr_mode = 0x00;
	ssd1963->pixel_intf = 0x03;				/* for now, only 16-bit (565) data supported */
	ssd1963->width = vendor_cfg->lcd_width;
	ssd1963->height = vendor_cfg->lcd_height;
	ESP_GOTO_ON_FALSE(ssd1963->width <= 864, ESP_ERR_INVALID_ARG, err, TAG, "maximum width for ssd1963 is 864 pixels");
	ESP_GOTO_ON_FALSE(ssd1963->height <= 480, ESP_ERR_INVALID_ARG, err, TAG, "maximum height for ssd1963 is 480 pixels");
	
    switch (panel_dev_config->color_space)
    {
    case ESP_LCD_COLOR_SPACE_RGB:
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    switch (panel_dev_config->bits_per_pixel)
    {
    case 16:
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

	switch (vendor_cfg->mcu_bit_interface)
	{
	case 8:
		break;
	case 16:
		break;
	default:
		ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported mcu bit interface (8 or 16 bit only)");
	}
	
    ssd1963->io = io;
    ssd1963->bits_per_pixel = panel_dev_config->bits_per_pixel;
    ssd1963->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ssd1963->reset_level = panel_dev_config->flags.reset_active_high;
	ssd1963->swap_axes = false;
	ssd1963->backlight = 0x80;						/* 50% by default */
    ssd1963->base.del = panel_ssd1963_del;
    ssd1963->base.reset = panel_ssd1963_reset;
    ssd1963->base.init = panel_ssd1963_init;
    ssd1963->base.draw_bitmap = panel_ssd1963_draw_bitmap;
    ssd1963->base.invert_color = panel_ssd1963_invert_color;
    ssd1963->base.set_gap = panel_ssd1963_set_gap;
    ssd1963->base.mirror = panel_ssd1963_mirror;
    ssd1963->base.swap_xy = panel_ssd1963_swap_xy;
    ssd1963->base.disp_on_off = panel_ssd1963_disp_on_off;
    *ret_panel = &(ssd1963->base);
    ESP_LOGD(TAG, "new ssd1963 panel @%p", ssd1963);

    return ESP_OK;

err:
    if (ssd1963)
    {
        if (panel_dev_config->reset_gpio_num != GPIO_NUM_NC)
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        free(ssd1963);
    }
    return ret;
}

static esp_err_t panel_ssd1963_del(esp_lcd_panel_t *panel)
{
    ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);

	if (ssd1963->reset_gpio_num >= 0)
		gpio_reset_pin(ssd1963->reset_gpio_num);

	ESP_LOGD(TAG, "del ssd1963 panel @%p", ssd1963);
    free(ssd1963);
    return ESP_OK;
}

static esp_err_t panel_ssd1963_reset(esp_lcd_panel_t *panel)
{
    ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t	io = ssd1963->io;

    if (ssd1963->reset_gpio_num != GPIO_NUM_NC)
    {
	    /* perform hardware reset */
        gpio_set_level(ssd1963->reset_gpio_num, !ssd1963->reset_level);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(ssd1963->reset_gpio_num, ssd1963->reset_level);
        vTaskDelay(pdMS_TO_TICKS(200));
	    gpio_set_level(ssd1963->reset_gpio_num, !ssd1963->reset_level);
	    vTaskDelay(pdMS_TO_TICKS(200));
    } else {
	    /* perform software reset */
        esp_lcd_panel_io_tx_param(io, SSD1963_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

static esp_err_t panel_ssd1963_init(esp_lcd_panel_t *panel)
{
	ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);

	/* hardware reset (if RST pin configured) */
	panel_ssd1963_reset(panel);

	/* PLL multiplier, set PLL clock to 120M */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PLLMN, (uint8_t[]) { 0x23, 0x02, 0x54 }, 3);
	/* PLL enable */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PLL, (uint8_t[]) { 0x01 }, 1);
	vTaskDelay(pdMS_TO_TICKS(1));	/* at least 10uS */
	/* now use PLL output as system clock */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PLL, (uint8_t[]) { 0x03 }, 1);
	vTaskDelay(pdMS_TO_TICKS(1));	/* at least 10uS */
	/* software reset */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SWRESET, NULL, 0);
	vTaskDelay(pdMS_TO_TICKS(1));	/* at least 20uS */
	/* PLL setting for PCLK, depends on resolution */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_LSH_FREQ, (uint8_t[]) { 0x03, 0x33, 0x33 }, 3);
	/* LCD specification */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_LCD_MODE, (uint8_t[]) {
			0x20,							/* 24-bit TFT panel */
			0x00,							/* Hsync+Vsync +DE mode TFT mode */
			((ssd1963->width - 1) >> 8),	/* HDP */
			(ssd1963->width - 1) & 0xff,
			((ssd1963->height - 1) >> 8),	/* VDP */
			(ssd1963->height - 1) & 0xff,
			0x00
		}, 7);
	/* HSYNC */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_HOR_PERIOD, (uint8_t[]) {
			0x04, 0x1f,				/* HT */
			0x00, 0xd2,				/* HPS */
			0x00,					/* HPW */
			0x00, 0x00,				/* HPS */
			0x00
		}, 8);
	/* VSYNC */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_VER_PERIOD, (uint8_t[]) {
			0x02, 0x0c,				/* VT */
			0x00, 0x22,				/* VPS */
			0x00,					/* VPW */
			0x00, 0x00				/* FPS */
		}, 7);
	/* GPIO[0] out 1 --- LCD display on/off control pin */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_GPIO_VAL, (uint8_t[]) { 0x01 }, 1);
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_GPIO_CONF, (uint8_t[]) { 0x0f, 0x01 }, 2);
	/* rotation */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_ADDR_MODE, (uint8_t[]) { ssd1963->addr_mode }, 1);
	/* set current pixel format for RGB image data (16-bit/pixel) */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PIXEL_FMT, (uint8_t[]) { 0x50 }, 1);
	/* pixel data interface format (16-bit 565 data) */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PIXEL_INTF, (uint8_t[]) { ssd1963->pixel_intf }, 1);
	/* configure post processor */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_POST_PROC, (uint8_t[]) {
			0x40,					/* contrast */
			0x80,					/* brightness */
			0x40,					/* saturation */
			0x01					/* enable post-processor */
		}, 4);
	vTaskDelay(pdMS_TO_TICKS(1));	/* at least 5uS */
	/* set PWM for backlight */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PWM_CONF, (uint8_t[]) { 0x06, ssd1963->backlight, 0x01, 0xf0, 0x00, 0x00 }, 6);
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_DBC_CONF, (uint8_t[]) { 0x0d }, 1);

	/* set default window to full screen */
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_COL_ADDR, (uint8_t[]) {
			0 >> 8, 0 & 0xff,				/* X start */
			(ssd1963->width - 1) >> 8,		/* X end */
			(ssd1963->width - 1) & 0xff
		}, 4);
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PAGE_ADDR, (uint8_t[]) {
			0 >> 8, 0 & 0xff,				/* Y start */
			(ssd1963->height - 1) >> 8,		/* Y end */
			(ssd1963->height - 1) & 0xff
		}, 4);

	return ESP_OK;
}

static void panel_ssd1963_set_window(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
	ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
	esp_lcd_panel_io_handle_t	io = ssd1963->io;
	assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");

	/* set window for pixel writing */
	esp_lcd_panel_io_tx_param(io, SSD1963_CMD_SET_COL_ADDR, (uint8_t[]) {
			(x_start >> 8) & 0xff, x_start & 0xff,
			(x_end >> 8) & 0xff, x_end & 0xff
		}, 4);
	esp_lcd_panel_io_tx_param(io, SSD1963_CMD_SET_PAGE_ADDR, (uint8_t[]) {
			(y_start >> 8) & 0xff, y_start & 0xff,
			(y_end >> 8) & 0xff, y_end & 0xff
		}, 4);
}

static esp_err_t panel_ssd1963_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t	io = ssd1963->io;
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");

	x_start += ssd1963->x_gap;
	x_end += ssd1963->x_gap;
	y_start += ssd1963->y_gap;
	y_end += ssd1963->y_gap;

	panel_ssd1963_set_window(panel, x_start, y_start, x_end, y_end);

    /* Write to graphic RAM */
	size_t len = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;
	if (esp_lcd_panel_io_tx_color(io, SSD1963_CMD_WRITE_MEM, color_data, len) != ESP_OK)
		ESP_LOGE(TAG, "esp_lcd_panel_io_tx_color() failed");

    return ESP_OK;
}

static esp_err_t panel_ssd1963_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
	ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
	esp_lcd_panel_io_handle_t	io = ssd1963->io;
	int							command = 0;

	if (invert_color_data)
		command = SSD1963_CMD_INV_ON;
	else
		command = SSD1963_CMD_INV_OFF;
	esp_lcd_panel_io_tx_param(io, command, NULL, 0);

	return ESP_OK;
}

static esp_err_t panel_ssd1963_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
	ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
	esp_lcd_panel_io_handle_t	io = ssd1963->io;

	if (mirror_x)
		ssd1963->addr_mode |= LCD_CMD_MX_BIT;
	else
		ssd1963->addr_mode &= ~LCD_CMD_MX_BIT;
	if (mirror_y)
		ssd1963->addr_mode |= LCD_CMD_MY_BIT;
	else
		ssd1963->addr_mode &= ~LCD_CMD_MY_BIT;
	esp_lcd_panel_io_tx_param(io, SSD1963_CMD_SET_ADDR_MODE, (uint8_t[]) { ssd1963->addr_mode }, 1);

    return ESP_OK;
}

static esp_err_t panel_ssd1963_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
	esp_lcd_panel_io_handle_t	io = ssd1963->io;

	ssd1963->swap_axes = swap_axes;
	return ESP_OK;
	
	if (ssd1963->swap_axes)
		ssd1963->addr_mode |= LCD_CMD_MV_BIT;
	else
		ssd1963->addr_mode &= ~LCD_CMD_MV_BIT;
	esp_lcd_panel_io_tx_param(io, SSD1963_CMD_SET_ADDR_MODE, (uint8_t[]) { ssd1963->addr_mode }, 1);

    return ESP_OK;
}

static esp_err_t panel_ssd1963_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
	ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);

	ssd1963->x_gap = x_gap;
	ssd1963->y_gap = y_gap;

	return ESP_OK;
}

static esp_err_t panel_ssd1963_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
	ssd1963_panel_t				*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
	esp_lcd_panel_io_handle_t	io = ssd1963->io;

	if (!on_off)
	{
		esp_lcd_panel_io_tx_param(io, SSD1963_CMD_ENTER_IDLE, NULL, 0);
		esp_lcd_panel_io_tx_param(io, SSD1963_CMD_DISP_OFF, NULL, 0);
		esp_lcd_panel_io_tx_param(io, SSD1963_CMD_SLEEP, NULL, 0);
		vTaskDelay(pdMS_TO_TICKS(5));
	} else {
		esp_lcd_panel_io_tx_param(io, SSD1963_CMD_END_SLEEP, NULL, 0);
		vTaskDelay(pdMS_TO_TICKS(5));
		esp_lcd_panel_io_tx_param(io, SSD1963_CMD_EXIT_IDLE, NULL, 0);
	}

    return ESP_OK;
}

esp_err_t ssd1963_set_backlight(esp_lcd_panel_t *panel, uint8_t level)
{
	esp_err_t			ret = ESP_OK;
	ssd1963_panel_t		*ssd1963 = __containerof(panel, ssd1963_panel_t, base);
	uint8_t				raw_level;

	raw_level = (uint8_t)(level * 2.55f);
	
	if (raw_level == ssd1963->backlight)
		return ret;
	
	/* set new backlight level
	 * if new level is zero, turn backlight off completely
	 */
	ssd1963->backlight = raw_level;
	esp_lcd_panel_io_tx_param(ssd1963->io, SSD1963_CMD_SET_PWM_CONF, (uint8_t[]) { 0x06, ssd1963->backlight, (level == 0) ? 0x00 : 0x01, 0xf0, 0x00, 0x00 }, 6);

	return ret;
}
	
#ifdef __cplusplus
}
#endif