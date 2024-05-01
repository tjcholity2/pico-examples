add_executable(max31865_spi
        max31865_spi.c
        )

# pull in common dependencies and additional spi hardware support
target_link_libraries(max31865_spi pico_stdlib hardware_spi)

add_compile_definitions(

        PICO_SPI_0_SCK_PIN=2
        PICO_SPI_0_TX_PIN=3
        PICO_SPI_0_RX_PIN=4
        PICO_SPI_0_CSN_PIN=5

        PICO_SPI_1_SCK_PIN=10
        PICO_SPI_1_TX_PIN=11
        PICO_SPI_1_RX_PIN=12
        PICO_SPI_1_CSN_PIN=13

        # Set to match hardware configuration:
        RTD_WIRE_COUNT=3
        RTD_RESISTANCE_AT_0C=100
        REF_RESISTOR=430
        FILTER_FREQUENCY=60

        # Options
        RTD_AUTO_CONV_MODE=0 # auto runs conversion at 50 or 60 Hz (if not using auto set 1-shot conversion bit to trigger conv).
        RTD_BIAS_ON=0 # to save power between conversions
        RTD_FAULT_THRESHOLD_LOWER=0x2110 # RTD raw value that triggers a low fault 
        RTD_FAULT_THRESHOLD_UPPER=0x2115 # RTD raw value that triggers a high fault
        RTD_FAULT_DETECTION_DELAY=10 # 10ms: worst case delay for RTD voltage settling assumes 10xRC time of input (10uf and 10K resistance).
)

# create map/bin/hex file etc.
pico_add_extra_outputs(max31865_spi)

# add url via pico_set_program_url
example_auto_set_url(max31865_spi)
