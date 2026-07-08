idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES gps lora mpumlx lm35 esp_driver_i2c
)