add_executable(sht30d_i2c
        sht30d_i2c.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(sht30d_i2c pico_stdlib hardware_i2c)

add_compile_definitions(

        # Set as needed to override pico standard defaults:
        PICO_DEFAULT_I2C=0     
        PICO_DEFAULT_I2C_SDA_PIN=16
        PICO_DEFAULT_I2C_SCL_PIN=17

        # Set to match hardware configuration:
        ADDR=0x44

        # Options
        RTD_AUTO_CONV_MODE=0 # auto runs conversion at 50 or 60 Hz (if not using auto set 1-shot conversion bit to trigger conv).
)

# create map/bin/hex file etc.
pico_add_extra_outputs(sht30d_i2c)

# add url via pico_set_program_url
example_auto_set_url(sht30d_i2c)
