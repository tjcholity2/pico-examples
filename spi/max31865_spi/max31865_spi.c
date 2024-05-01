/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "max31865_spi.h"

#define READ_BIT 0x80

// Bit assignments for config features (register 0):
#define MAX31865_CONFIG_BIAS 0x80 // set this bit true to power (bias) ext ref resister (power saving feature) - must be true for RTD convertsion.
#define MAX31865_CONFIG_MODEAUTO 0x40 // set this bit true for continuous (50 or 60 Hz rate) AUTO RTD conversion.
#define MAX31865_CONFIG_MODEOFF 0x00
#define MAX31865_CONFIG_1SHOT 0x20 // set this bit true to trigger RTD conversion (used when config for AUTO false) - cleared after conversion.
#define MAX31865_CONFIG_3WIRE 0x10 // set this bit true when using a 3-wire RTD - compensates for missing wire in RTD conversion.
#define MAX31865_CONFIG_24WIRE 0x00
#define MAX31865_CONFIG_FAULT_DET_MANU 0x08 // set this to start manual fault detection - manually add extra time for ext RTD interface delay - see manual.
#define MAX31865_CONFIG_FAULT_DET_AUTO 0x04 // set this to start auto fault detection.
#define MAX31865_CONFIG_FAULT_STATUS 0x02 // set this bit true to reset fault. Note: during reset also turn config bits 0x20, 0x08, 0x04 to zero (fault controls). 
#define MAX31865_CONFIG_FILT50HZ 0x01 // set true when power supply runs @ 50Hz (extends RTD conversion)
#define MAX31865_CONFIG_FILT60HZ 0x00

// Bit assignments for detected faults (register 7):
#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04 // Over Voltage or under Voltage fault detected
// Note: the two lowest bits of the fault register are not assigned a fault.

// 8-bit registers internal to device
#define MAX31865_CONFIG_REG 0x00 // See above config constants for bit function 
#define MAX31865_RTDMSB_REG 0x01 // MSB of RTD
#define MAX31865_RTDLSB_REG 0x02 // LSB of RTD
#define MAX31865_HFAULTMSB_REG 0x03 // MSB of highest resistance for RTD (Power-On-Reset (POR) value is 0xFF) - host sets @ intialization.
#define MAX31865_HFAULTLSB_REG 0x04 // LSB of highest resistance for RTD (Power-On-Reset (POR) value is 0xFF) - host sets @ intialization.
#define MAX31865_LFAULTMSB_REG 0x05 // MSB of lowest resistance for RTD (Power-On-Reset (POR) value is 0x00) - host sets @ intialization.
#define MAX31865_LFAULTLSB_REG 0x06 // LSB of lowest resistance for RTD (Power-On-Reset (POR) value is 0x00) - host sets @ intialization.
#define MAX31865_FAULTSTAT_REG 0x07 // fault indications - see above fault constants for bit function

#define RTD_A 3.90830e-3
#define RTD_B -5.77500e-7
#define RTD_C -4.18301e-12    

const int polarity = 0; // clock idles low
const int phase = 1;

static inline void cs_select(max31865_spi_type *ptr) {
    asm volatile("nop \n nop \n nop");
    gpio_put(ptr->cs_pico_gp_number, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(max31865_spi_type *ptr) {
    asm volatile("nop \n nop \n nop");
    gpio_put(ptr->cs_pico_gp_number, 1);
    asm volatile("nop \n nop \n nop");
}

static void writeRegister8(max31865_spi_type *ptr, uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg | 0x80; // Set MSB to 1 to indicate write
    buf[1] = data;
    cs_select(ptr);
    spi_write_blocking(ptr->spi, buf, 2);
    sleep_us(20);
    cs_deselect(ptr);
    sleep_ms(10);
}

static uint8_t readRegister8(max31865_spi_type *ptr, uint8_t reg) {
    reg &= 0x7F; // make sure top bit is not set
    uint8_t data1 = 0x00;
    cs_select(ptr);
    spi_write_blocking(ptr->spi, &reg, 1);
    //sleep_ms(10);
    spi_read_blocking(ptr->spi, 0, &data1, 1);
    sleep_us(20);
    cs_deselect(ptr);
    sleep_ms(10);
    return data1;
}

static void writeRegister16(max31865_spi_type *ptr, uint8_t reg, uint16_t *data) {
    uint8_t buf[3];
    uint16_t local_data = *data;
    uint8_t lsb = (uint8_t)(local_data & 0x00FF);
    buf[0] = reg | 0x80; // Set MSB to 1 to indicate write
    buf[1] = (uint8_t)((local_data >> 8) & 0x00FF);
    buf[2] = lsb;
    cs_select(ptr);
    spi_write_blocking(ptr->spi, buf, 3);
    sleep_us(20);
    cs_deselect(ptr);
    sleep_ms(10);
}

static uint16_t readRegister16(max31865_spi_type *ptr, uint8_t reg) {
    uint8_t buf[2];
    cs_select(ptr);
    spi_write_blocking(ptr->spi, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(ptr->spi, 0, buf, 2);
    sleep_us(20);
    cs_deselect(ptr);
    sleep_ms(10);
    uint16_t ret = ((uint16_t)buf[0]) << 8;
    ret = ret + (uint16_t)buf[1];
    return ret;
}

/**************************************************************************/
/*!
    @brief Flips the config register bit that indicates which power supply 
    frequency is in use (usually be 50 or 60 Hz).
    @param freq The frequency of the AC power supply.
*/
/**************************************************************************/
void setFilterfreq(max31865_spi_type *ptr, int8_t freq) {
  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);
  if (freq == 50) {
    t |= MAX31865_CONFIG_FILT50HZ;
  } else if (freq == 60){
    // 2 or 4 wire clears 3-wire bit.
    t &= ~MAX31865_CONFIG_FILT50HZ;
  } else{
    printf("FILTER_FREQUENCY must be a value of 50 or 60!");
    t |= MAX31865_CONFIG_FILT50HZ; // default to longer filter.
  }
  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE, MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void setWires(max31865_spi_type *ptr, int8_t wires) {
  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);
  if (wires == 3) {
    t |= MAX31865_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire clears 3-wire bit.
    t &= ~MAX31865_CONFIG_3WIRE;
  }
  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void enableBias(max31865_spi_type *ptr, bool b) {
  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
    t = t | MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void autoConvert(max31865_spi_type *ptr, bool b) {
  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Sets the config register bits appropriately to start the fault detection cycle. 
    @param autoCycle If true, auto conversion is enabled

// Bit assignments for config features (register 0):
#define MAX31865_CONFIG_BIAS 0x80 // set this bit true to power (bias) ext ref resister (power saving feature) - must be true for RTD convertsion.
#define MAX31865_CONFIG_MODEAUTO 0x40 // set this bit true for continuous (50 or 60 Hz rate) AUTO RTD conversion.
#define MAX31865_CONFIG_MODEOFF 0x00
#define MAX31865_CONFIG_1SHOT 0x20 // set this bit true to trigger RTD conversion (used when config for AUTO false) - cleared after conversion.
#define MAX31865_CONFIG_3WIRE 0x10 // set this bit true when using a 3-wire RTD - compensates for missing wire in RTD conversion.
#define MAX31865_CONFIG_24WIRE 0x00
#define MAX31865_CONFIG_FAULT_DET_MANU 0x08 // set this to start manual fault detection - manually add extra time for ext RTD interface delay - see manual.
#define MAX31865_CONFIG_FAULT_DET_AUTO 0x04 // set this to start auto fault detection.
#define MAX31865_CONFIG_FAULT_STATUS 0x02 // set this bit true to reset fault. Note: during reset also turn config bits 0x20, 0x08, 0x04 to zero (fault controls). 
#define MAX31865_CONFIG_FILT50HZ 0x01 // set true when power supply runs @ 50Hz (extends RTD conversion)
#define MAX31865_CONFIG_FILT60HZ 0x00


*/
/**************************************************************************/
void setConfigRegisterBits(max31865_spi_type *ptr, bool autoCycle) {

  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);

  // 100X010Xb
  if (autoCycle) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }
    
  if (autoCycle) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }

  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Configure the 16-bit fault detection thresholds for detecting lower and upper faults. 
    @param lower RTD raw value that triggers fault (ratio of RTD resistance to reference resistance)
    @param upper RTD raw value that triggers fault (ratio of RTD resistance to reference resistance)
*/
/**************************************************************************/
void setThresholds(max31865_spi_type *ptr, uint16_t lower, uint16_t upper) {
  writeRegister8(ptr, MAX31865_LFAULTLSB_REG, lower & 0xFF);
  writeRegister8(ptr, MAX31865_LFAULTMSB_REG, lower >> 8);
  writeRegister8(ptr, MAX31865_HFAULTLSB_REG, upper & 0xFF);
  writeRegister8(ptr, MAX31865_HFAULTMSB_REG, upper >> 8);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void clearFault(max31865_spi_type *ptr) {
  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);
  // The three bits to be cleared are set true and then inverted to be used for clearing those bits.
  uint8_t total = MAX31865_CONFIG_1SHOT + MAX31865_CONFIG_FAULT_DET_MANU + MAX31865_CONFIG_FAULT_DET_AUTO;
  t &= ~total; // Quick way of clearing the above three bits.
  t |= MAX31865_CONFIG_FAULT_STATUS; // Setting true to trigger fault reset.
  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Run the fault detection cycle.
    @return The fault status register (reg 7) 8-bit value - zero if no fault found.
*/
/**************************************************************************/
uint8_t startFaultDetectionCycle(max31865_spi_type *ptr) {

  uint8_t configReg = readRegister8(ptr, MAX31865_CONFIG_REG);
  uint8_t testConfig;
  uint8_t faultReg;
  uint8_t waitMilliSec = 0;
  bool autoModeSave = configReg & MAX31865_CONFIG_MODEAUTO;
  bool testComplete = false;

  testConfig = configReg & 0b00010001; // Save config bits (if set) for 3-wire and 50 Hertz values.

  if(autoModeSave){
    testConfig |= 0b10000100; // Set bits to turn on bias and auto fault det mode while preserving 3-wire and 50 Hertz values.
    writeRegister8(ptr, MAX31865_CONFIG_REG, testConfig); // initiates automatic fault detection cycle.
  }
  else{ // manual mode is where fault detection delays must be added.
    testConfig |= 0b10000000; // Set bit to turn on bias while preserving 3-wire and 50 Hertz values.
    writeRegister8(ptr, MAX31865_CONFIG_REG, testConfig); // starts bias going to reference resistor.
    sleep_ms(RTD_FAULT_DETECTION_DELAY); // Delay for settling.
    testConfig |= 0b00001000; // Set bit to enable manual fault detection while preserving bias, 3-wire, and 50 Hertz values.
    writeRegister8(ptr, MAX31865_CONFIG_REG, testConfig); // initiates first part of manual fault detection cycle.
    sleep_ms(RTD_FAULT_DETECTION_DELAY); // Delay for settling.
    testConfig |= 0b00001100; // Set bit to enable manual fault detection while preserving bias, 3-wire, and 50 Hertz values.  }
    writeRegister8(ptr, MAX31865_CONFIG_REG, testConfig); // initiates second part of manual fault detection cycle.
  }

  // Wait for test completion:
  while (!testComplete && (waitMilliSec < 10)) {
    configReg = readRegister8(ptr, MAX31865_CONFIG_REG);
    configReg &= 0b00001100; // select Fault Detect Cycle bits
    configReg = (testConfig == 0b000000000); // Fault Detect Cycle bits are cleared.
    sleep_ms(1);
    waitMilliSec += 1;
  }

  faultReg = readRegister8(ptr, MAX31865_FAULTSTAT_REG);  

  return faultReg;
}


/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @param RTD_raw The output value for the raw unsigned 16-bit value, NOT temperature!
    @return The fault status register (reg 7) 8-bit value (reg read only if fault) - zero if no fault found.
*/
/**************************************************************************/
uint8_t readRTD(max31865_spi_type *ptr, uint16_t *RTD_raw) {

  clearFault(ptr);
  enableBias(ptr, true);
  sleep_ms(10);
  uint8_t t = readRegister8(ptr, MAX31865_CONFIG_REG);
  t |= MAX31865_CONFIG_1SHOT;
  writeRegister8(ptr, MAX31865_CONFIG_REG, t);
  sleep_ms(65);

  uint8_t rtd1 = readRegister8(ptr, MAX31865_RTDMSB_REG);
  // printf("readRTD rtd1: 0x%x\n", rtd1);

  uint8_t rtd2 = readRegister8(ptr, MAX31865_RTDLSB_REG);
  // printf("readRTD rtd2: 0x%x\n", rtd2);

  uint16_t rtd = ((uint16_t)rtd1) << 8;
  rtd += (uint16_t)rtd2;

  enableBias(ptr, 
  false); // Disable bias current again to reduce selfheating.

  uint8_t fault = 0x00;

  // Check if bit zero is set - which would indicate a fault.
  if(rtd & 0x0001){
    fault = readRegister8(ptr, MAX31865_FAULTSTAT_REG);
  }

  // remove fault bit
  rtd >>= 1;

  *RTD_raw = rtd;

  return fault;
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface and configure the MAX31865 device.
*/
/**************************************************************************/
int max31865_init(max31865_spi_type *ptr){

    // printf("Config: 0x%x\n", currentConfig);
    // // Set wire config register based on the number of wires specified.
    // if wires not in (2, 3, 4):
    //     printf("Wires must be a value of 2, 3, or 4!");
    // if wires == 3:
    //     config |= _MAX31865_CONFIG_3WIRE;
    // else:
    //     // 2 or 4 wire
    //     config &= ~_MAX31865_CONFIG_3WIRE;

    // self._write_u8(_MAX31865_CONFIG_REG, config);
    // // Default to no bias and no auto conversion.
    // self.bias = False;
    // self.auto_convert = False;

    ptr->temperature_C = 0.0;
    ptr->temperature_F = 0.0;
  
    ptr->baudrate = spi_init_mode(ptr->spi, ptr->baudrate, SPI_CPOL_0, SPI_CPHA_1);

    if(ptr->debug){
      printf("Hello, max31865! Reading raw data from registers via SPI...\n");
      printf("sck_pico_gp_number: %u\n", ptr->sck_pico_gp_number);
      printf(" rx_pico_gp_number: %u\n", ptr->rx_pico_gp_number);
      printf(" tx_pico_gp_number: %u\n", ptr->tx_pico_gp_number);
      printf(" cs_pico_gp_number: %u\n", ptr->cs_pico_gp_number);
      printf("baudrate: %d\n", ptr->baudrate);
    }

    gpio_set_function(ptr->rx_pico_gp_number, GPIO_FUNC_SPI);
    gpio_set_function(ptr->sck_pico_gp_number, GPIO_FUNC_SPI);
    gpio_set_function(ptr->tx_pico_gp_number, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    //bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(ptr->cs_pico_gp_number);
    gpio_set_dir(ptr->cs_pico_gp_number, GPIO_OUT);
    gpio_put(ptr->cs_pico_gp_number, 1);
    // Make the CS pin available to picotool
    //bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    // uint8_t config = readRegister8(ptr, MAX31865_CONFIG_REG);
    // printf("Config: 0x%x\n", config);

    uint8_t currentConfig = 0x00;
    uint8_t currentFaults = 0x00;

    setFilterfreq(ptr, FILTER_FREQUENCY);

    setWires(ptr, RTD_WIRE_COUNT);
  
    enableBias(ptr, false);
  
    autoConvert(ptr, false);

    setThresholds(ptr, ptr->lower_temp_warn_limit, ptr->upper_temp_warn_limit);

    clearFault(ptr);

    if(ptr->debug){
      currentConfig = readRegister8(ptr, MAX31865_CONFIG_REG);
      currentFaults = readRegister8(ptr, MAX31865_FAULTSTAT_REG);
      printf("Config: 0x%x\n", currentConfig);
      printf("Faults: 0x%x\n", currentFaults);
    }
    return 0;
}

/**************************************************************************/
/*!
    @brief Calculate the raw 15-bit value from a resistance. Primarily used for code testing.
    @param RTD_resistance A ohm input value for the RTD device.
    @return The 15-bit value that represents a ratio of RTD resistance to reference resistance that should 
    be returned via SPI bus from the MAX31865 device. Note: The device sends a fault indication in the LSB
    of the value - return value is not shifted left as the device value woud be to make room in the LSB.
*/
/**************************************************************************/
uint16_t calculate_RTD_raw(float RTD_resistance){

  double ratio = (double)RTD_resistance / (double)REF_RESISTOR;
  ratio *= 32768.0f;
  uint16_t ret = (uint16_t)ratio;
  return ret;
}

/**************************************************************************/
/*!
    @brief Calculate the expected resistance of a platinum RTD. Primarily used for code testing.
    @param t Temperature of the RTD device.
    @return the expected resistance of the RTD at the given temperature.
*/
/**************************************************************************/
float calculateResistance(float t){

  // The following notes are from: https://www.analog.com/en/technical-articles/rtd-measurement-system-design-essentials.html
  //
  // For a platinum RTD, the Callendar-Van Dusen equation describes the relationship between resistance and temperature (C) as:

  // R(t) = R0 × (1 + A × t +B × t2 + (t - 100) × C × t3),

  // where

  // R(t) = RTD resistance
  // t = temperature
  // R0 = resistance of the RTD at 0°C
  // A = 3.9083 × 10-3
  // B = -5.775 × 10-7
  // C = -4.183 × 10-12 when t < 0°C
  // C = 0 when t > 0°C

  float R0 = RTD_RESISTANCE_AT_0C;
  float A = RTD_A;
  float B = RTD_B;
  float C = RTD_C;
  float t2 = t * t;
  float t3 = t2 * t;
  float ret = 0;

  if(t > 0){
    C = 0;
  }

  ret = R0 * (1 + (A * t) + (B * t2) + ((t - 100) * C * t3));

  return ret;
}

/**************************************************************************/
/*!
    @brief Calculate the temperature of the platinum RTD.
    @param RTD_raw 15-bit resistance ratio value from the RTD device (MAX31865).
    @return Temperature in Centigrade.
*/
/**************************************************************************/
float calculateTemperature(uint16_t RTD_raw) {

  double Z1, Z2, Z3, Z4, Rt, temp;

  Rt = (float)RTD_raw;
  //printf("raw = %.2fC\n", Rt);
  Rt /= 32768.0f;
  Rt *= (double)REF_RESISTOR;

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTD_RESISTANCE_AT_0C;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTD_RESISTANCE_AT_0C;
  Rt *= 100.0f; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Print (serial output) the faults indicated by the device fault register.
    @param faultReg 8-bit fault register from the RTD device (MAX31865).
*/
/**************************************************************************/
const void printFaults(max31865_spi_type *ptr, uint8_t faultReg){

  uint8_t config_msb = 0x00;
  uint8_t config_lsb = 0x00;

  config_msb = readRegister8(ptr, MAX31865_HFAULTMSB_REG);
  config_lsb = readRegister8(ptr, MAX31865_HFAULTLSB_REG);
  uint16_t highLimit = (config_msb << 8) | config_lsb; // Value given to device.
  uint16_t highLimitTemp = highLimit >> 1; // Shift value into LSB where fault bit is.

  config_msb = readRegister8(ptr, MAX31865_LFAULTMSB_REG);
  config_lsb = readRegister8(ptr, MAX31865_LFAULTLSB_REG);
  uint16_t lowLimit = (config_msb << 8) | config_lsb; // Value given to device.
  uint16_t lowLimitTemp = lowLimit >> 1; // Shift value into LSB where fault bit is.

  if(faultReg & MAX31865_FAULT_HIGHTHRESH){
    printf("Fault: RTD temperature >= %.2f (config: 0x%x)\n", calculateTemperature(highLimitTemp), highLimit);
  }

  if(faultReg & MAX31865_FAULT_LOWTHRESH){
    printf("Fault: RTD temperature <= %.2f (config: 0x%x)\n", calculateTemperature(lowLimitTemp), lowLimit);
  }

  if(faultReg & MAX31865_FAULT_REFINHIGH){
    printf("Fault: REFIN- > 0.85 BIAS\n");
  }

  if(faultReg & MAX31865_FAULT_REFINLOW){
    printf("Fault: REFIN- < 0.85 BIAS (while FORCE- open)\n");
  }

  if(faultReg & MAX31865_FAULT_RTDINLOW){
    printf("Fault: rtdLow\n");
  }

  if(faultReg & MAX31865_FAULT_OVUV){
    printf("Fault: Protected input has under or over voltage.\n");
  }
}

const void handleFaultCycle(){

}

void max31865_reset(max31865_spi_type *ptr)
{
    return; // TBD
}

float max31865_get_C_Temperature(max31865_spi_type *ptr)
{
    return ptr->temperature_C;
}

float max31865_get_F_Temperature(max31865_spi_type *ptr)
{
    return ptr->temperature_F;
}

int max31865_readSensor(max31865_spi_type *ptr){

    uint8_t faults;
    uint16_t RTD_raw;
    float resistance;
    float resistanceRatio;
    uint16_t check_RTD_raw;

    faults = readRTD(ptr, &RTD_raw);

    ptr->temperature_C = calculateTemperature(RTD_raw);
    ptr->temperature_F = (ptr->temperature_C * (9.0f / 5.0f)) + 32.0f;

    if(ptr->debug){

      resistance = calculateResistance(ptr->temperature_C);
      resistanceRatio = resistance / RTD_RESISTANCE_AT_0C;
      check_RTD_raw = calculate_RTD_raw(resistance);

      printf("Temperature C = %.3f C\n", ptr->temperature_C);
      printf("Temperature F = %.3f C\n", ptr->temperature_F);
      printf("resistance / RTD_RESISTANCE_AT_0C = %.3f C\n", resistanceRatio);

      printf("RTD raw (15-bit ratio) 0x%x Temp. = %.2fC (%.2fF) Calc raw 0x%x\n", RTD_raw, ptr->temperature_C, ptr->temperature_F, check_RTD_raw);
    }


    if(faults){
      printf("Faults:");
      printFaults(ptr, faults);
    }

    return 0;
}


// The following main is for independent testing:

int main() {

  stdio_init_all();

  max31865_spi_type sensor0;
  max31865_spi_type sensor1;

  sensor0.spi = spi0;
  sensor0.debug = true;
  sensor0.temperature_C = 0.0f;
  sensor0.temperature_F = 0.0f;
  sensor0.sck_pico_gp_number = PICO_SPI_0_SCK_PIN;
  sensor0.rx_pico_gp_number = PICO_SPI_0_RX_PIN;
  sensor0.tx_pico_gp_number = PICO_SPI_0_TX_PIN;
  sensor0.cs_pico_gp_number = PICO_SPI_0_CSN_PIN;
  sensor0.baudrate = 500000;
  sensor0.lower_temp_warn_limit = RTD_FAULT_THRESHOLD_LOWER;
  sensor0.upper_temp_warn_limit = RTD_FAULT_THRESHOLD_UPPER;

  sensor1.spi = spi0;
  sensor1.debug = true;
  sensor1.temperature_C = 0.0f;
  sensor1.temperature_F = 0.0f;
  sensor1.sck_pico_gp_number = PICO_SPI_1_SCK_PIN;
  sensor1.rx_pico_gp_number = PICO_SPI_1_RX_PIN;
  sensor1.tx_pico_gp_number = PICO_SPI_1_TX_PIN;
  sensor1.cs_pico_gp_number = PICO_SPI_1_CSN_PIN;
  sensor1.baudrate = 500000;
  sensor1.lower_temp_warn_limit = RTD_FAULT_THRESHOLD_LOWER;
  sensor1.upper_temp_warn_limit = RTD_FAULT_THRESHOLD_UPPER;

  int retVal0 = max31865_init(&sensor0);
  printf("retVal for sensor0 init: %d\n", retVal0);

  int retVal1 = max31865_init(&sensor1);
  printf("retVal for sensor1 init: %d\n", retVal1);

  uint8_t loopCount = 0;
  uint8_t faults0, faults1;

  while (1) {

    if(loopCount > 100){

      loopCount = 0;

      faults0 = startFaultDetectionCycle(&sensor0);
      faults1 = startFaultDetectionCycle(&sensor1);



      if (sensor0.debug && (faults0 > 0)){
        printf("sensor0 fault: 0x%x found!\n", faults0);
        printFaults(&sensor0, faults0);
      }

      if (sensor1.debug && (faults1 > 0)){
        printf("sensor1 faults 0x%x found!\n", faults1);
        printFaults(&sensor1, faults1);
      }
    }

    max31865_readSensor(&sensor0);
    max31865_readSensor(&sensor1);
    printf("sensor0 Temperature = %.3f C\n", sensor0.temperature_C);
    printf("sensor1 Temperature = %.3f C\n", sensor1.temperature_C);

    // poll every 500ms
    sleep_ms(500);

    loopCount++;
  }
}

