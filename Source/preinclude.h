#define TC_LINKKEY_JOIN
#define NV_INIT
#define NV_RESTORE


#define TP2_LEGACY_ZC
//patch sdk
// #define ZDSECMGR_TC_ATTEMPT_DEFAULT_KEY TRUE

#define NWK_AUTO_POLL
#define MULTICAST_ENABLED FALSE

#define ZCL_READ
#define ZCL_WRITE
#define ZCL_BASIC
#define ZCL_IDENTIFY
#define ZCL_ON_OFF
#define ZCL_LEVEL_CTRL
#define ZCL_REPORTING_DEVICE

#define ZSTACK_DEVICE_BUILD (DEVICE_BUILD_ENDDEVICE)

#define DISABLE_GREENPOWER_BASIC_PROXY
#define BDB_FINDING_BINDING_CAPABILITY_ENABLED 1
#define BDB_REPORTING TRUE


#define ISR_KEYINTERRUPT
#define HAL_BUZZER FALSE

#define HAL_LED TRUE
#define HAL_I2C TRUE
#define BLINK_LEDS TRUE

#define LQI_REQ
#if defined(LQI_REQ)
#define MT_TASK
#define MT_SYS_FUNC
#define MT_ZDO_FUNC
#define MT_ZDO_MGMT
#define INT_HEAP_LEN (2688)
#endif

//one of this boards
// #define HAL_BOARD_MOTION
// #define HAL_BOARD_CHDTECH_DEV

#if !defined(HAL_BOARD_MOTION) && !defined(HAL_BOARD_CHDTECH_DEV)
#error "Board type must be defined"
#endif

#define BDB_MAX_CLUSTERENDPOINTS_REPORTING 10

#define LUMOISITY_PORT 0
#define LUMOISITY_PIN 7

//#define SMART

#if defined(HAL_BOARD_MOTION)
#define POWER_SAVING
//#define DO_DEBUG_UART

#elif defined(HAL_BOARD_CHDTECH_DEV)
// #define DO_DEBUG_UART
#define DO_DEBUG_MT

#endif


//i2c bh1750
#define OCM_CLK_PORT 0
#define OCM_DATA_PORT 0
#define OCM_CLK_PIN 5
#define OCM_DATA_PIN 6

#define HAL_I2C_RETRY_CNT 1


#ifdef DO_DEBUG_UART
#define HAL_UART TRUE
#define HAL_UART_DMA 1
#define INT_HEAP_LEN (2685 - 0x4B - 0xBB)
#endif

#ifdef DO_DEBUG_MT
#define HAL_UART TRUE
#define HAL_UART_DMA 1
#define HAL_UART_ISR 2
#define INT_HEAP_LEN (2688-0xC4-0x15-0x44-0x20-0x1E)

#define MT_TASK

#define MT_UTIL_FUNC
#define MT_UART_DEFAULT_BAUDRATE HAL_UART_BR_115200
#define MT_UART_DEFAULT_OVERFLOW FALSE

#define ZTOOL_P1

#define MT_APP_FUNC
#define MT_APP_CNF_FUNC
#define MT_SYS_FUNC
#define MT_ZDO_FUNC
#define MT_ZDO_MGMT
#define MT_DEBUG_FUNC

#endif



#if defined(HAL_BOARD_MOTION)
#define FACTORY_RESET_BY_LONG_PRESS_PORT 0x04 //port2

//#define HAL_KEY_P0_INPUT_PINS BV(4)
#define HAL_KEY_P0_INPUT_PINS BV(0)
#define HAL_KEY_P0_INPUT_PINS_EDGE HAL_KEY_RISING_EDGE


#define HAL_KEY_P1_INPUT_PINS BV(3)
#define HAL_KEY_P1_INPUT_PINS_EDGE HAL_KEY_RISING_EDGE

#define HAL_KEY_P2_INPUT_PINS BV(0)

#elif defined(HAL_BOARD_CHDTECH_DEV)
#define HAL_KEY_P0_INPUT_PINS BV(1)
#define HAL_KEY_P2_INPUT_PINS BV(0)
#endif

#include "hal_board_cfg.h"

#include "stdint.h"
#include "stddef.h"
