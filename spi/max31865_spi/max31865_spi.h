
/* Example code to talk to a max31865 platinum RTD temperature sensor.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore SPI) cannot be used at 5v.

   You will need to use a level shifter on the SPI lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic max31865 board, other
   boards may vary.

   GPIO 16 (pin 21) MISO/spi0_rx-> SDO/SDO on max31865 board
   GPIO 17 (pin 22) Chip select -> CSB/!CS on max31865 board
   GPIO 18 (pin 24) SCK/spi0_sclk -> SCL/SCK on max31865 board
   GPIO 19 (pin 25) MOSI/spi0_tx -> SDA/SDI on max31865 board
   3.3v (pin 36) -> VCC on max31865 board
   GND (pin 38)  -> GND on max31865 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

   This code uses a bunch of register definitions, and some compensation code derived
   from the Analog datasheet which can be found here.
   https://www.analog.com/media/en/technical-documentation/data-sheets/MAX31865.pdf
*/

#ifndef MAX31865_SPI_H_  /* Include guard */
#define MAX31865_SPI_H_

typedef struct max31865_spi
{
  spi_inst_t *spi;
  bool debug;
  float temperature_C;
  float temperature_F;
  int sck_pico_gp_number;
  int rx_pico_gp_number;
  int tx_pico_gp_number;
  int cs_pico_gp_number;
  uint baudrate;
  uint16_t lower_temp_warn_limit;
  uint16_t upper_temp_warn_limit;
} max31865_spi_type;

int max31865_init(max31865_spi_type *ptr);

// To be called if a soft reset is needed - not needed on powerup (hard reset).
void max31865_reset(max31865_spi_type *ptr);

// Returns last temperature read.
float max31865_get_C_Temperature(max31865_spi_type *ptr);
float max31865_get_F_Temperature(max31865_spi_type *ptr);

// Returns zero if successful.  
// Returns 1 if device not found.
// Returns 2 if device not communicating.
int max31865_readSensor(max31865_spi_type *ptr);

#endif // max31865_SPI_H_