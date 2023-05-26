# esp_lcd driver for SSD1963 based displays

This component provides an implementation of the SSD1963 LCD controller using the esp_lcd component APIs.

| LCD controller | Communication interface | Component name  | Link to datasheet |
| :------------: | :---------------------: | :-------------: | :---------------: |
| SSD1963        | Intel 8080              | esp_lcd_ssd1963 | [Specification](https://www.buydisplay.com/download/ic/SSD1963.pdf) |

## Note on supported communication interfaces

When using the Intel 8080 (Parallel) interface the 16-bit color depth mode should be used.
This uses RGB565 format data.

## Using this component in your project

This package can be added to your project as follows:

```
dependencies:
  esp_lcd_ssd1963:
    git: https://github.com/danmeuk/esp_lcd_ssd1963.git
```

For more information on the usage of the `idf_component.yml` file please refer to [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Supported platforms

Due to the i80 nature of the interface, this requires ESP32-S3 or higher.
Older ESP32 chips do not have hardware support for i80 interfaces.

## Testing

This has been tested in 16-bit parallel mode up to 16 MHz clock with an ESP32-S3 (internal and SPIRAM buffers) and a BuyDisplay ER-TFTM043A1-7S board.

## Example code

This assumes you have pulled the RD pin high either in software or on your PCB.
If you don't do this, you will experience trouble.

This defaults to an 8MHz speed, always best to start slow and build up.

To same some precious ESP32 pins, you can tie the CS pin to GND permanently and set LCD_CS to GPIO_NUM_NC.

This code will not work on its own, you need to configure other things such as LVGL and buffers etc.
It is provided for guidance only.

```
/* create i80 bus */
esp_lcd_i80_bus_handle_t i80_bus = NULL;
esp_lcd_i80_bus_config_t bus_config = {
	.clk_src = LCD_CLK_SRC_DEFAULT,
	.dc_gpio_num = LCD_DC,
	.wr_gpio_num = LCD_WR,
	.data_gpio_nums = {
		LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7,
		LCD_D8, LCD_D9, LCD_D10, LCD_D11, LCD_D12, LCD_D13, LCD_D14, LCD_D15
	},
	.bus_width = 16,
	.psram_trans_align = 64,
	.sram_trans_align = 4,
	.max_transfer_bytes = 800 * 10 * sizeof(lv_color_t)
};
esp_lcd_new_i80_bus(&bus_config, &i80_bus);
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_io_i80_config_t io_config = {
	.cs_gpio_num = LCD_CS,
	.pclk_hz = (8 * 1000 * 1000),
	.trans_queue_depth = 32,
	.dc_levels = {
		.dc_idle_level = 0,
		.dc_cmd_level = 0,
		.dc_dummy_level = 0,
		.dc_data_level = 1,
	},
	.flags = {
		.swap_color_bytes = 0,
		.pclk_idle_low = 0,
	},
	.on_color_trans_done = _notify_lvgl_flush_ready,
	.user_ctx = &disp_drv,
	.lcd_cmd_bits = 16,
	.lcd_param_bits = 8,
};
esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle);

/* configure SSD1963 panel */
const esp_lcd_panel_ssd1963_config_t vendor_config = {
	.lcd_width = 800,
	.lcd_height = 480,
	.mcu_bit_interface = 16,
};
esp_lcd_panel_dev_config_t panel_config = {
	.reset_gpio_num = LCD_RST,
	.color_space = ESP_LCD_COLOR_SPACE_RGB,
	.bits_per_pixel = 16,
	.vendor_config = (void*)&vendor_config,
};
esp_lcd_new_panel_ssd1963(io_handle, &panel_config, &lcd_panel_handle);
esp_lcd_panel_init(lcd_panel_handle);
esp_lcd_panel_disp_on_off(lcd_panel_handle, true);
```
