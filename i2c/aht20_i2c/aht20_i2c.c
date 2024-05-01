#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

 /* Example code to talk to a AHT20 temperature and humidity sensor

    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.

    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.

    Connections on Raspberry Pi Pico board, other boards may vary.

    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on BMP280
    board
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> SCL on
    BMP280 board
    3.3v (pin 36) -> VCC on BMP280 board
    GND (pin 38)  -> GND on BMP280 board
 */

 // device has default bus address of 0x38
#define ADDR _u(0x38)
#define ADDR_ALT _u(0x39)

#define AHTX0_CMD_SEND 0x70          ///< Send command to device (this is first byte; second byte is device command)
#define AHTX0_CMD_STATUS 0x71        ///< Get status command
#define AHTX0_CMD_CALIBRATE 0xE1     ///< Calibration command
#define AHTX0_CMD_NOR_MODE 0xA8      ///< Enter NOR working mode command
#define AHTX0_CMD_TRIGGER 0xAC       ///< Trigger measurement command
#define AHTX0_CMD_MODIFY_REG 0xB0    ///< Modify of register needs this command
#define AHTX0_CMD_SOFTRESET 0xBA     ///< Soft reset command
#define AHTX0_CMD_INIT_AHT20 0xBE    ///< Initialization command for the AHT20
#define AHTX0_CMD_INIT_AHT10 0xE1    ///< Initialization command for the AHT10
#define AHTX0_CMD_READ 0x33          ///< Trigger read command? (not documented)
#define AHTX0_STATUS_NORMAL 0x18     ///< Status bit for normal and calibrated?
#define AHTX0_STATUS_BUSY 0x80       ///< Status bit for busy
#define AHTX0_STATUS_CALIBRATED 0x08 ///< Status bit for calibrated

int _sensorID = 0x1020;

#ifdef i2c_default

//uint8_t out_buf[4];
//uint8_t in_buf[7];

void reinit_register(uint8_t reg_num){

    uint8_t out_buf[4];

    out_buf[0] = AHTX0_CMD_SEND;
    out_buf[1] = reg_num;
    out_buf[2] = 0x00;
    out_buf[3] = 0x00;

    i2c_write_blocking(i2c_default, ADDR, out_buf, 4, false);
    
    sleep_ms(5);

    out_buf[0] = AHTX0_CMD_STATUS;
    out_buf[1] = 0x00;
    out_buf[2] = 0x00;
    out_buf[3] = 0x00;

    uint8_t in_buf[7] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
    i2c_write_blocking(i2c_default, ADDR, out_buf, 4, true);
    i2c_read_blocking(i2c_default, ADDR, in_buf, 3, false);  // false - finished with bus
    uint8_t byte_second = in_buf[1];
    uint8_t byte_third  = in_buf[2];

    sleep_ms(10);

    out_buf[0] = AHTX0_CMD_SEND;
    out_buf[1] = (AHTX0_CMD_MODIFY_REG | reg_num);
    out_buf[2] = byte_second;
    out_buf[3] = byte_third;
    
    i2c_write_blocking(i2c_default, ADDR, out_buf, 4, false);

    sleep_ms(10);
}

void reinit_registers(){
    reinit_register(0x1B);
    reinit_register(0x1C);
    reinit_register(0x1E);
}

int aht20_init() {
 
    // Set device into NOR working mode (not documented?).
    uint8_t out_buf[4];
    out_buf[0] = AHTX0_CMD_SEND;
    out_buf[1] = AHTX0_CMD_NOR_MODE;
    out_buf[2] = 0x00;
    out_buf[3] = 0x00;

    i2c_write_blocking(i2c_default, ADDR, out_buf, 4, false);  // stop and release bus

    sleep_ms(10);

    // Send initialization command to device.
    out_buf[0] = AHTX0_CMD_SEND;
    out_buf[1] = AHTX0_CMD_INIT_AHT20;
    out_buf[2] = 0x08;
    out_buf[3] = 0x00;
    i2c_write_blocking(i2c_default, ADDR, out_buf, 4, false);  // stop and release bus

    sleep_ms(10);

    return 0;
}

uint8_t aht20_read_status() {

    uint8_t out_buf;
    uint8_t in_buf;

    // Attempt a read of values - looping until 
    out_buf = AHTX0_CMD_STATUS;
    i2c_write_blocking(i2c_default, ADDR, &out_buf, 1, true);
    i2c_read_blocking(i2c_default, ADDR, &in_buf, 1, false);
    return in_buf;
}

void aht20_read_raw(int32_t* temp, int32_t* humidity) {

    uint8_t out_buf[4];
    uint8_t in_buf[7];
    uint32_t RetData;

    out_buf[0] = AHTX0_CMD_SEND; //
    out_buf[1] = AHTX0_CMD_TRIGGER;
    out_buf[2] = AHTX0_CMD_READ;
    out_buf[3] = 0x00;
    i2c_write_blocking(i2c_default, ADDR, out_buf, 4, false);

    sleep_ms(80);

    uint8_t count =0;

    while((aht20_read_status() & AHTX0_STATUS_BUSY) == AHTX0_STATUS_BUSY){

        sleep_ms(80);
        if (count++ > 100){
            break;
        }
    }

    uint8_t out_cmd = AHTX0_CMD_STATUS; // Trigger sensor read
    in_buf[0] = 0x00;
    in_buf[1] = 0x00;
    in_buf[2] = 0x00;
    in_buf[3] = 0x00;
    in_buf[4] = 0x00;
    in_buf[5] = 0x00;
    in_buf[6] = 0x00;
    i2c_write_blocking(i2c_default, ADDR, &out_cmd, 1, true);
    i2c_read_blocking(i2c_default, ADDR, in_buf, 7, false);

    // *humidity = (in_buf[1] << 8) | (in_buf[2] << 4) | (in_buf[3] >> 4);
    // *temp = (in_buf[3] << 16) | (in_buf[4] << 8) | in_buf[5];
    RetData = 0;
    RetData = (RetData | in_buf[1]) << 8;
    RetData = (RetData | in_buf[2]) << 8;
    *humidity = (RetData | in_buf[3]) >> 4;

    RetData = 0;
    RetData = (RetData | in_buf[3]) << 8;
    RetData = (RetData | in_buf[4]) << 8;
    *temp   = (RetData | in_buf[5]) & 0x000fffff;
}

void aht20_reset() {
    // reset the device with the power-on-reset procedure
    uint8_t reg = AHTX0_CMD_SOFTRESET;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, false);
}

float aht20_convert_temp(int32_t temp) {
    float _temperature = ((float)temp * 200 / 0x100000) - 50;
    _temperature = ((float)temp * 200 / 1024 / 1024) - 50;
    return _temperature;
}

float aht20_convert_humidity(int32_t humidity) {
    float _humidity = ((float)humidity * 100) / 0x100000;
    _humidity = (float)humidity * 100 / 1024 / 1024;
    return _humidity;
}

#endif

int main() {
    
    stdio_init_all();

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c / aht20_i2c example requires a board with I2C pins
        puts("Default I2C pins were not defined");
    return 0;
#else
    // useful information for picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("AHT20 I2C example for the Raspberry Pi Pico"));

    printf("Hello, AHT20! Reading temperaure and humidity values from sensor...\n");

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    //sleep_ms(500); // device needs 500 ms to be ready.

    //aht20_init();

    if ((aht20_read_status() & 0x18) != 0x18){
        printf("Initialization failed!");
        reinit_registers();
        // return 0;
    }
    else{
        printf("Initialization complete!");
    }

    int32_t raw_temperature;
    int32_t raw_humidity;


    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (1) {
        aht20_read_raw(&raw_temperature, &raw_humidity);
        float temperature = aht20_convert_temp(raw_temperature);
        float humidity = aht20_convert_humidity(raw_humidity);
        printf("Humidity = %.3f %\n", humidity);
        printf("Temperature = %.3f C\n", temperature);
        // poll every 500ms
        sleep_ms(500);
    }
#endif
}
