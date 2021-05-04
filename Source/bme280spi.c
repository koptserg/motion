#include "bme280spi.h"
#include "Debug.h"

#include <stdlib.h>

/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/
/*
  LCD pins

  //control
  P0.0 - LCD_MODE (DC)
  P1.1 - LCD_FLASH_RESET (RST)
  P1.2 - LCD_CS (CS)

  //spi
  P1.5 - CLK
  P1.6 - MOSI
  P1.7 - MISO
*/

/* LCD Control lines */
//#define HAL_LCD_MODE_PORT 0
//#define HAL_LCD_MODE_PIN  0

//#define HAL_LCD_RESET_PORT 1
//#define HAL_LCD_RESET_PIN  1

#define HAL_LCD_CS_PORT 1
#define HAL_LCD_CS_PIN  2

/* LCD SPI lines */
#define HAL_LCD_CLK_PORT 1
#define HAL_LCD_CLK_PIN  5

#define HAL_LCD_MOSI_PORT 1
#define HAL_LCD_MOSI_PIN  6

#define HAL_LCD_MISO_PORT 1
#define HAL_LCD_MISO_PIN  7

/* SPI settings */
#define HAL_SPI_CLOCK_POL_LO       0x00 // CPOL 0 0x00, CPOL 1 0x80
#define HAL_SPI_CLOCK_PHA_0        0x00 // CPHA 0 0x00, CPHA 1 0x40
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20 // ORDER 0 0x00 LSB first, ORDER 1 0x20 MSB first

#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin = val; )

#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin = val; \
                                                      P##port##DIR |= BV(pin); )

#define HAL_CONFIG_IO_INPUT(port, pin, val)      HAL_CONFIG_IO_INPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_INPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin = val; \
                                                      P##port##DIR &= ~BV(pin); )

#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )

#define HAL_CONFIG_IO_GP(port, pin)      HAL_CONFIG_IO_GP_PREP(port, pin)
#define HAL_CONFIG_IO_GP_PREP(port, pin) st( P##port##SEL &= ~BV(pin); )


/* SPI interface control */
#define LCD_SPI_BEGIN()     HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  0); /* chip select */
#define LCD_SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  1); /* chip select */         \
}
/* clear the received and transmit byte status, write tx data to buffer, wait till transmit done */
#define LCD_SPI_TX(x)                   { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define LCD_SPI_WAIT_RXRDY()            { while(!(U1CSR & BV(1))); }


/* Control macros */
//#define LCD_DO_WRITE()        HAL_IO_SET(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  1);
//#define LCD_DO_CONTROL()      HAL_IO_SET(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  0);

//#define LCD_ACTIVATE_RESET()  HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 0);
//#define LCD_RELEASE_RESET()   HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);

void bme_HW_WaitUs(uint16 i);
void bme280_write8(uint8 reg, uint8 data);
uint8 bme280_read8(uint8 reg);
bool bme280_isReadingCalibration(void);
void bme280_readCoefficients(void);
uint8 LCD_SPI_WAIT_RX(void);
uint16 bme280_read16(uint8 reg);
uint16 bme280_read16_LE(uint8 reg);
int16 bme280_readS16(uint8 reg);
uint32 bme280_read24(uint8 reg);
float bme280_readTemperature(void);
float bme280_readHumidity(void);
float bme280_readPressure(void);
void bme280_takeForcedMeasurement(void);

int32 t_fine;

/**************************************************************************/
/*!
    @brief  calibration data
*/
/**************************************************************************/
  uint16 bme280_dig_T1; ///< temperature compensation value
  int16 bme280_dig_T2;  ///< temperature compensation value
  int16 bme280_dig_T3;  ///< temperature compensation value

  uint16 bme280_dig_P1; ///< pressure compensation value
  int16 bme280_dig_P2;  ///< pressure compensation value
  int16 bme280_dig_P3;  ///< pressure compensation value
  int16 bme280_dig_P4;  ///< pressure compensation value
  int16 bme280_dig_P5;  ///< pressure compensation value
  int16 bme280_dig_P6;  ///< pressure compensation value
  int16 bme280_dig_P7;  ///< pressure compensation value
  int16 bme280_dig_P8;  ///< pressure compensation value
  int16 bme280_dig_P9;  ///< pressure compensation value

  uint8 bme280_dig_H1; ///< humidity compensation value
  int16 bme280_dig_H2; ///< humidity compensation value
  uint8 bme280_dig_H3; ///< humidity compensation value
  int16 bme280_dig_H4; ///< humidity compensation value
  int16 bme280_dig_H5; ///< humidity compensation value
  int8 bme280_dig_H6;  ///< humidity compensation value


uint8 LCD_SPI_WAIT_RX(void) { 
  while(!(U1CSR & BV(1))); 
  return U1DBUF; 
}

bool BME280Init(void) {
  
  /* Initialize LCD IO lines */
  bme_ConfigIO();

  /* Initialize SPI */
  bme_ConfigSPI();


  // check if sensor, i.e. the chip ID is correct
  uint8 chip = bme280_read8(BME280_REGISTER_CHIPID);
  LREP("BME280_REGISTER_CHIPID=%d\r\n", chip);;
  if (chip != 0x60) {
    LREPMaster("NOT BME280\r\n");
    /* Initialize GPIO */
    bme_ConfigIOInput();
    bme_ConfigGP();
    return 0;
  }
  
  // reset the device using soft-reset
  // this makes sure the IIR is off, etc.  
  bme280_write8(BME280_REGISTER_SOFTRESET, 0xB6);
  
  // wait for chip to wake up.  
  bme_HW_WaitUs(10);
  
  // if chip is still reading calibration, delay
  while (bme280_isReadingCalibration())
  bme_HW_WaitUs(10);

  bme280_readCoefficients(); // read trimming parameters, see DS 4.2.2

//  bme280_setSampling(MODE_NORMAL, SAMPLING_X16, SAMPLING_X16, SAMPLING_X16, FILTER_OFF, STANDBY_MS_0_5); // use defaults
  bme280_setSampling(MODE_FORCED, SAMPLING_X2, SAMPLING_X16, SAMPLING_X1, FILTER_X16, STANDBY_MS_62_5); // air
    
  bme_HW_WaitUs(100);
  
//  float temperature = bme280_readTemperature();
//  LREP("Temperature=%d\r\n", temperature);

  return 1;
}

void bme_ConfigIO(void)
{
  /* GPIO configuration */
//  HAL_CONFIG_IO_OUTPUT(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  1);
//  HAL_CONFIG_IO_OUTPUT(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT,    HAL_LCD_CS_PIN,    1);
}

void bme_ConfigIOInput(void)
{
  /* GPIO configuration */
  HAL_CONFIG_IO_INPUT(HAL_LCD_CS_PORT,    HAL_LCD_CS_PIN,    0);
}

void bme_ConfigSPI(void)
{
  /* UART/SPI Peripheral configuration */

   uint8 baud_exponent;
   uint8 baud_mantissa;

  /* Set SPI on UART 1 alternative 2 */
  PERCFG |= 0x02;

  /* Configure clk, master out and master in lines */
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);


  /* Set SPI speed to 1 MHz (the values assume system clk of 32MHz)
   * Confirm on board that this results in 1MHz spi clk.
   */
  baud_exponent = 15;
  baud_mantissa =  0;

  /* Configure SPI */
  U1UCR  = 0x00;      /* Flush and goto IDLE state. 8-N-1. */
  U1CSR  = 0x00;      /* SPI mode, master. */
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_0 | HAL_SPI_CLOCK_POL_LO | baud_exponent;
  U1BAUD = baud_mantissa;
}

void bme_ConfigGP(void)
{
  /* Set SPI on UART 1 alternative 2 */
  PERCFG &= ~0x02;

  /* Configure clk, master out and master in lines */
  HAL_CONFIG_IO_GP(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_GP(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_GP(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);
}

void bme_HW_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

void bme280_write8(uint8 reg, uint8 data)
{
  LCD_SPI_BEGIN();
  LCD_SPI_TX(reg & ~0x80);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_TX(data);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_END();
}

uint8 bme280_read8(uint8 reg)
{
  uint8 value = 0;
  LCD_SPI_BEGIN();
  LCD_SPI_TX(reg | 0x80);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_TX(0);
  LCD_SPI_WAIT_RXRDY();
  value = LCD_SPI_WAIT_RX();
  LCD_SPI_END();
  return value;
}

bool bme280_isReadingCalibration(void) {
  uint8 const rStatus = bme280_read8(BME280_REGISTER_STATUS);
  LREP("BME280_REGISTER_STATUS=%d\r\n", rStatus);;
  return (rStatus & (1 << 0)) != 0;
}

void bme280_readCoefficients(void) {
  bme280_dig_T1  = bme280_read16_LE(BME280_REGISTER_DIG_T1);
  bme280_dig_T2 = bme280_readS16_LE(BME280_REGISTER_DIG_T2);
  bme280_dig_T3 = bme280_readS16_LE(BME280_REGISTER_DIG_T3);

  bme280_dig_P1 = bme280_read16_LE(BME280_REGISTER_DIG_P1);
  bme280_dig_P2 = bme280_readS16_LE(BME280_REGISTER_DIG_P2);
  bme280_dig_P3 = bme280_readS16_LE(BME280_REGISTER_DIG_P3);
  bme280_dig_P4 = bme280_readS16_LE(BME280_REGISTER_DIG_P4);
  bme280_dig_P5 = bme280_readS16_LE(BME280_REGISTER_DIG_P5);
  bme280_dig_P6 = bme280_readS16_LE(BME280_REGISTER_DIG_P6);
  bme280_dig_P7 = bme280_readS16_LE(BME280_REGISTER_DIG_P7);
  bme280_dig_P8 = bme280_readS16_LE(BME280_REGISTER_DIG_P8);
  bme280_dig_P9 = bme280_readS16_LE(BME280_REGISTER_DIG_P9);

  bme280_dig_H1 = bme280_read8(BME280_REGISTER_DIG_H1);
  bme280_dig_H2 = bme280_readS16_LE(BME280_REGISTER_DIG_H2);
  bme280_dig_H3 = bme280_read8(BME280_REGISTER_DIG_H3);
  bme280_dig_H4 = ((int8)bme280_read8(BME280_REGISTER_DIG_H4) << 4) |
                         (bme280_read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
  bme280_dig_H5 = ((int8)bme280_read8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                         (bme280_read8(BME280_REGISTER_DIG_H5) >> 4);
  bme280_dig_H6 = (int8)bme280_read8(BME280_REGISTER_DIG_H6);
}


uint16 bme280_read16(uint8 reg) {
  uint16 value16;
  
  LCD_SPI_BEGIN();
  LCD_SPI_TX(reg | 0x80);
  LCD_SPI_WAIT_RXRDY();
  
  LCD_SPI_TX(0);
  LCD_SPI_WAIT_RXRDY();
  uint8 value_H = LCD_SPI_WAIT_RX();
  LCD_SPI_TX(0);
  LCD_SPI_WAIT_RXRDY();
  uint8 value_L = LCD_SPI_WAIT_RX();
  LCD_SPI_END();
  
  value16 = (value_H << 8) | value_L;

  return value16;
}

uint16 bme280_read16_LE(uint8 reg) {
  uint16 temp = bme280_read16(reg);
  return (temp >> 8) | (temp << 8);
}

int16 bme280_readS16(uint8 reg) {
  return (int16)bme280_read16(reg); 
}

int16 bme280_readS16_LE(uint8 reg) {
  return (int16)bme280_read16_LE(reg);
}

uint32 bme280_read24(uint8 reg) {
  uint32 value32;

  LCD_SPI_BEGIN();
  LCD_SPI_TX(reg | 0x80);
  LCD_SPI_WAIT_RXRDY();

    LCD_SPI_TX(0);
    LCD_SPI_WAIT_RXRDY();
    uint32 value32_1 = (uint32)LCD_SPI_WAIT_RX();
    value32_1 <<= 12;

    LCD_SPI_TX(0);
    LCD_SPI_WAIT_RXRDY();
    uint32 value32_2 = (uint32)LCD_SPI_WAIT_RX();
    value32_2 <<= 4;

    LCD_SPI_TX(0);
    LCD_SPI_WAIT_RXRDY();
    uint32 value32_3 = (uint32)LCD_SPI_WAIT_RX();
    value32_3 >>= 4;
    LCD_SPI_END();
    value32 = value32_1 | value32_2 | value32_3;
  return value32;
}

/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function, so SPI users
 *   don't get confused about the library requiring an address.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
void bme280_setSampling(sensor_mode mode,
                        sensor_sampling tempSampling,
                        sensor_sampling pressSampling,
                        sensor_sampling humSampling,
                        sensor_filter filter,
                        standby_duration duration) {

  unsigned int configRegspi3w_en = 0; ///< unused - don't set

  // making sure sensor is in sleep mode before setting configuration
  // as it otherwise may be ignored
  bme280_write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

  // you must make sure to also set REGISTER_CONTROL after setting the
  // CONTROLHUMID register, otherwise the values won't be applied (see
  // DS 5.4.3)

//  LREP("BME280_REGISTER_CONTROLHUMID=%d\r\n", humSampling);
  bme280_write8(BME280_REGISTER_CONTROLHUMID, humSampling);

//  LREP("BME280_REGISTER_CONFIG=%d\r\n", (duration << 5) | (filter << 2) | configRegspi3w_en);
  bme280_write8(BME280_REGISTER_CONFIG, (duration << 5) | (filter << 2) | configRegspi3w_en);

//  LREP("BME280_REGISTER_CONTROL=%d\r\n", (tempSampling << 5) | (pressSampling << 2) | mode);
  bme280_write8(BME280_REGISTER_CONTROL, (tempSampling << 5) | (pressSampling << 2) | mode);

}

void bme280_takeForcedMeasurement(void) {
  // If we are in forced mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.
  // In normal mode simply does new measurements periodically.

    bme280_write8(BME280_REGISTER_CONTROL, (SAMPLING_X16 << 5) | (SAMPLING_X16 << 2) | MODE_FORCED);
    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    while (bme280_read8(BME280_REGISTER_STATUS) & 0x08)
      bme_HW_WaitUs(1000);

}

float bme280_readTemperature(void)
{
    int32 adc_T = bme280_read24(BME280_REGISTER_TEMPDATA);
    
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = ((double)adc_T) / 16384.0 - ((double)bme280_dig_T1) / 1024.0;
    var1 = var1 * ((double)bme280_dig_T2);
    var2 = (((double)adc_T) / 131072.0 - ((double)bme280_dig_T1) / 8192.0);
    var2 = (var2 * var2) * ((double)bme280_dig_T3);
    t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;
    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

float bme280_readHumidity(void)
{
  bme280_readTemperature(); // must be done first to get t_fine

  int32 adc_H = bme280_read16(BME280_REGISTER_HUMIDDATA);
  
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)t_fine) - 76800.0;
    var2 = (((double)bme280_dig_H4) * 64.0 + (((double)bme280_dig_H5) / 16384.0) * var1);
    var3 = adc_H - var2;
    var4 = ((double)bme280_dig_H2) / 65536.0;
    var5 = (1.0 + (((double)bme280_dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)bme280_dig_H6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)bme280_dig_H1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity ;
}

float bme280_readPressure(void)
{
  
//  bme280_readTemperature(); // must be done first to get t_fine
  int32 adc_P = bme280_read24(BME280_REGISTER_PRESSUREDATA);
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)bme280_dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)bme280_dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)bme280_dig_P4) * 65536.0);
    var3 = ((double)bme280_dig_P3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)bme280_dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)bme280_dig_P1);

    /* avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) adc_P;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)bme280_dig_P9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)bme280_dig_P8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)bme280_dig_P7)) / 16.0;
        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure / 100;
}

