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
