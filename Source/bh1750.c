/**************************************************************************************************
  This is a library for the BH1750FVI Digital Light Sensor breakout board.

  The BH1750 board uses I2C for communication. Two pins are required to
  interface to the device. Configuring the I2C bus is expected to be done
  in user code. The BH1750 library doesn't do this automatically.

  Written by Christopher Laws, March, 2013.
**************************************************************************************************/
#include "bh1750.h"
#include "Debug.h"
#include <stdlib.h>
#include "hal_i2c.h"

uint8 bh1750_MTreg = (uint8)BH1750_DEFAULT_MTREG;
uint8 bh1750_mode = CONTINUOUS_HIGH_RES_MODE;

/**
 * Configure BH1750 with specified mode
 * @param mode Measurement mode
 */
bool bh1750_init(uint8 mode) {
//  LREPMaster("[BH1750] INIT: start\r\n");
  bh1850_Write(BH1750_RESET);

  bh1850_Write(BH1750_POWER_ON);

  bh1850_Write(ONE_TIME_LOW_RES_MODE);
  bh1750_WaitMs(30);
  float initread = bh1850_Read();
//  LREP("[BH1750] initread 0x%X \r\n", (uint16)initread);
  if((uint16)initread != 0xD554){
    bh1750_mode = mode;
    bh1850_Write(mode);
    bh1750_setMTreg(bh1750_MTreg);
  
    bh1850_PowerDown();
    LREPMaster("[BH1750] INIT\r\n");
    return 1;
  } 
  LREPMaster("[BH1750] ERROR: NOT BH1750\r\n");
  return 0;
}

/**
 * Configure BH1750 MTreg value
 * MT reg = Measurement Time register
 * @param MTreg a value between 32 and 254. Default: 69
 * @return bool true if MTReg successful set
 * 		false if MTreg not changed or parameter out of range
 */
bool bh1750_setMTreg(uint8 MTreg) {
  bh1750_MTreg = MTreg;
  //Bug: lowest value seems to be 32!
  if (MTreg <= 31 || MTreg > 254) {
    LREPMaster("[BH1750] ERROR: MTreg out of range\r\n");
    return 0;
  }
  // Send MTreg and the current mode to the sensor
  //   High bit: 01000_MT[7,6,5]
  //    Low bit: 011_MT[4,3,2,1,0]
  bh1850_Write((0x08 << 3) | (MTreg >> 5));
  bh1850_Write((0x03 << 5) | (MTreg & 0x1F));
  
  // Wait a few moments to wake up
  bh1750_WaitMs(10);

  return 0;
}

void bh1850_PowerDown(void) {
  if (bh1750_mode == CONTINUOUS_HIGH_RES_MODE || bh1750_mode == CONTINUOUS_HIGH_RES_MODE_2 || bh1750_mode == CONTINUOUS_LOW_RES_MODE) {
    bh1850_Write(BH1750_POWER_DOWN);
  }
}

float bh1850_Read(void) {
    uint8 address = ((BH1750_I2CADDR << 1) | OCM_READ);
    uint8 buf[] = {0x00, 0x00};
    HalI2CReceive(address, buf, 2);
    uint16 value16 = ((buf[0] << 8) | buf[1]);
    float level = 0;
    level = value16;
    if (bh1750_MTreg != BH1750_DEFAULT_MTREG) {
      level *= (float)((uint8)BH1750_DEFAULT_MTREG/(float)bh1750_MTreg);
    }
    if (bh1750_mode == ONE_TIME_HIGH_RES_MODE_2 || bh1750_mode == CONTINUOUS_HIGH_RES_MODE_2) {
      level /= 2;
    }
    // Convert raw value to lux
    level /= BH1750_CONV_FACTOR;
    LREP("[BH1750] level %d \r\n", (uint16)level);
    return level; 
}

void bh1850_Write(uint8 mode) {
    // begin the write sequence with the address byte
    uint8 address = ((BH1750_I2CADDR << 1) | OCM_WRITE);
    uint8 buf[] = {mode};
    HalI2CSend(address, buf, 1);
  switch (mode) {
    case CONTINUOUS_HIGH_RES_MODE:
    case CONTINUOUS_HIGH_RES_MODE_2:
    case CONTINUOUS_LOW_RES_MODE:
    case ONE_TIME_HIGH_RES_MODE:
    case ONE_TIME_HIGH_RES_MODE_2:
    case ONE_TIME_LOW_RES_MODE:
//      LREP("[BH1750] mode %d \r\n", mode);
      bh1750_mode = mode;
      break;
    default:
//      LREP("[BH1750] command %d \r\n", mode);
      break;

  }
}

void bh1750_WaitUs(uint16 microSecs) {
  while(microSecs--) {
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

void bh1750_WaitMs(uint32_t period) { bh1750_WaitUs(period * 1000); }