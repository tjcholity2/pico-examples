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

#ifndef SHT30D_I2C_H_  /* Include guard */
#define SHT30D_I2C_H_

int sht30d_init(bool generateDebugComments=false, int sda_pico_gp_number=PICO_DEFAULT_I2C_SDA_PIN, int scl_pico_gp_number=PICO_DEFAULT_I2C_SCL_PIN);

// To be called if a soft reset is needed - not needed on powerup (hard reset).
void sht30d_reset();

// Returns last temperature read.
float sht30d_get_C_Temperature();
float sht30d_get_F_Temperature();

// Returns last humidity read.
float sht30d_getHumidityPercent(); // returns 0.0 ... 100.0

// Returns zero if successful.  
// Returns 1 if device not found.
// Returns 2 if device not communicating.
int sht30d_readSensors();

#endif // SHT30D_I2C_H_
