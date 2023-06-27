///
// basic system macros
//

#ifndef __main_h
#define __main_h

// System Include Files
#include <stddef.h>       // offsetof()
#include <stdlib.h>       // abs()
#include <stdbool.h>      // bool type defined properly whether using C99 or C++
#include <stdio.h>        // needed for NULL and debug printf.
#include <string.h>       // memset, memcpy
// needed for IAR v8.x+:
#include <intrinsics.h>   // including  __no_operation()

// CPU specific defines
#include "py32f0xx.h"

// Make the HAL happy
#include "py32f0xx_hal.h"

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_exti.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_utils.h"
#include "py32f0xx_ll_pwr.h"
//#include "py32f0xx_ll_dma.h"
#include "py32f0xx_ll_gpio.h"
//#include "py32f0xx_ll_usart.h"
#include "py32f0xx_ll_tim.h"

extern bool bRunLoop, pause, bDmaComplete;
extern uint32_t Seconds;

///
// @brief DEBUG macros
//
#if defined (DEBUG)
  #define DEBUG_BREAK() do { __asm("bkpt #0"); } while(0)
  #define DEBUG_NOP() __no_operation()
  // helps debugging local function vars in debug mode.
  #define STATIC    static
#else
  #define DEBUG_BREAK()
  #define DEBUG_NOP()
  #define STATIC
#endif

#endif