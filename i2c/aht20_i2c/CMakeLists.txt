add_executable(aht20_i2c
        aht20_i2c.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(aht20_i2c pico_stdlib hardware_i2c)

add_compile_definitions(

#         # Set as needed to override pico standard defaults:
        PICO_DEFAULT_I2C=0     
        PICO_DEFAULT_I2C_SDA_PIN=16
        PICO_DEFAULT_I2C_SCL_PIN=17
 )

# create map/bin/hex file etc.
pico_add_extra_outputs(aht20_i2c)

# add url via pico_set_program_url
example_auto_set_url(aht20_i2c)
