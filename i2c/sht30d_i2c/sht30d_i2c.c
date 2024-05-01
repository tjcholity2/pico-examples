#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

 /* Example code to talk to a SHT30D temperature and humidity sensor

    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.

    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.

    Connections on Raspberry Pi Pico board, other boards may vary.

    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> green wire (SDA) to sensor
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> yellow wire (SCL) to sensor
    3.3v (pin 36) -> red wire (GND) to sensor
    GND (pin 38)  -> black wire (GND) to sensor
 */
#include "hardware/i2c.h"
#define SHT3X_CMD_SEND 0x70          ///< Send command to device (this is first byte; second byte is device command)
#define SHT3X_CMD_STATUS 0xF32D      ///< Get status command
#define SHT3X_CMD_CALIBRATE 0xE1     ///< Calibration command
#define SHT3X_CMD_NOR_MODE 0xA8      ///< Enter NOR working mode command
#define SHT3X_CMD_TRIGGER 0xAC       ///< Trigger measurement command
#define SHT3X_CMD_MODIFY_REG 0xB0    ///< Modify of register needs this command
#define SHT3X_CMD_SOFTRESET 0xBA     ///< Soft reset command
#define SHT3X_CMD_INIT_SHT30D 0xBE   ///< Initialization command for the SHT30D
#define SHT3X_CMD_READ 0x33          ///< Trigger read command? (not documented)
#define SHT3X_STATUS_NORMAL 0x18     ///< Status bit for normal and calibrated?
#define SHT3X_STATUS_BUSY 0x80       ///< Status bit for busy
#define SHT3X_STATUS_CALIBRATED 0x08 ///< Status bit for calibrated


bool debug = false;
float temperature_C = 0.0;
float temperature_F = 0.0;
float humidity = 0.0;


int sht30d_init(bool generateDebugComments, int sda_pico_gp_number, int scl_pico_gp_number) 
{

    debug = generateDebugComments;
 
    //#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    //#warning i2c / sht30d_i2c example requires a board with I2C pins
    //    puts("Default I2C pins were not defined");
    //return 0;
    //#else

    // useful information for picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    //bi_decl(bi_program_description("SHT30D I2C example for the Raspberry Pi Pico"));

    if(debug){printf("celsiusHello, SHT30D! Reading temperaure and humidity values from sensor...\n");}

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(sda_pico_gp_number, GPIO_FUNC_I2C);
    gpio_set_function(scl_pico_gp_number, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pico_gp_number);
    gpio_pull_up(scl_pico_gp_number);

    sleep_ms(500); // device needs 500 ms to be ready.

    return 0;
}

uint16_t sht30d_read_status() {

    uint8_t out_buf[4];
    uint8_t in_buf[4];
    out_buf[0] = (uint8_t)((SHT3X_CMD_STATUS & 0xFF00) >> 8);
    out_buf[1] = SHT3X_CMD_STATUS & 0x00FF;
    out_buf[2] = 0x00;
    out_buf[3] = 0x00;

    in_buf[0] = 0x00;
    in_buf[1] = 0x00;
    in_buf[2] = 0x00;
    in_buf[3] = 0x00;
    uint16_t retVal;
    i2c_write_blocking(i2c_default, ADDR, out_buf, 2, true);
    i2c_read_blocking(i2c_default, ADDR, in_buf, 2, false);  // false - finished with bus
    retVal = (in_buf[0] << 8) | in_buf[1];
    return retVal;
}

void sht30d_read_raw(int32_t* temp, int32_t* humidity) {

    uint8_t out_buf[4];
    uint8_t in_buf[7];
    uint32_t RetData;

    out_buf[0] = 0x24; // MSB of command
    out_buf[1] = 0x16; // LSB of command

    i2c_write_blocking(i2c_default, ADDR, out_buf, 2, false);

    sleep_ms(80);

    in_buf[0] = 0x00;
    in_buf[1] = 0x00;
    in_buf[2] = 0x00;
    in_buf[3] = 0x00;
    in_buf[4] = 0x00;
    in_buf[5] = 0x00;
    in_buf[6] = 0x00;
    // i2c_write_blocking(i2c_default, ADDR, &out_cmd, 2, true);
    i2c_read_blocking(i2c_default, ADDR, in_buf, 6, false);

    RetData = in_buf[0];
    RetData = RetData << 8;
    RetData = RetData | in_buf[1];
    *temp  = RetData;

    RetData = in_buf[3];
    RetData = RetData << 8;
    RetData = RetData | in_buf[4];
    *humidity = RetData;
}

void sht30d_reset() {

    temperature_C = 0.0;
    temperature_F = 0.0;
    humidity = 0.0;

    // reset the device with the power-on-reset procedure
    uint8_t reg = SHT3X_CMD_SOFTRESET;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, false);
}

void sht30d_convert_temp(int32_t temp_raw) {
    // formula comes from manufacturer (see https://sensirion.com/media/documents/213E6A3B/63A5A569/Datasheet_SHT3x_DIS.pdf)
    temperature_C = ((float)temp_raw * 175 / (0x10000 - 1)) - 45;
    temperature_F = ((float)temp_raw * 315 / (0x10000 - 1)) - 49;
    return;
}

void sht30d_convert_humidity(int32_t humidity_raw) {
    // formula comes from manufacturer (see https://sensirion.com/media/documents/213E6A3B/63A5A569/Datasheet_SHT3x_DIS.pdf)
    humidity = ((float)humidity_raw * 100) / (0x10000 - 1);
    return;
}

float sht30d_get_C_Temperature()
{
    return temperature_C;
}

float sht30d_get_F_Temperature()
{
    return temperature_F;
}

float sht30d_getHumidityPercent()
{
    return humidity;
}

int sht30d_readSensors(){

    int32_t raw_temperature = 0.0;
    int32_t raw_humidity = 0.0;

    sht30d_read_raw(&raw_temperature, &raw_humidity);
    sht30d_convert_temp(raw_temperature);
    sht30d_convert_humidity(raw_humidity);

    if(debug){
        printf("Humidity = %.3f %\n", humidity);
        printf("Temperature C = %.3f C\n", temperature_C);
        printf("Temperature F = %.3f C\n", temperature_F);
    }

    return 0;
}

// The following main is for independent testing:
/*
int main() {
    
    stdio_init_all();

    sht30d_init(true, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);

    uint16_t sensor_status = sht30d_read_status();

    if (debug && (sensor_status != 0)){

        printf("Initialization failed!\n");

        if(sensor_status & 0b1000000000000000){
            printf(", Alert pending \n");
        };
        if(sensor_status & 0b0000000000010000){
            printf(", System reset detected\n");
        };
    }
    else{
        printf("Initialization complete!\n");
    }

    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (1) {
        sht30d_readSensors();
        printf("Humidity = %.3f %\n", humidity);
        printf("Temperature = %.3f C\n", temperature_C);
        // poll every 500ms
        sleep_ms(500);
    }
}
*/
