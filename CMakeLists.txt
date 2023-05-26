idf_component_register(
	SRCS
		"esp_lcd_ssd1963.c"
	INCLUDE_DIRS
		"."
	REQUIRES
		"esp_lcd"
	PRIV_REQUIRES
		"driver esp_timer"
)
