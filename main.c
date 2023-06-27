#include "main.h"
#include "pin-port.h"


// global functions
void InitGPIO(void);
void InitTimer1Pwm(void);
void InitADC(void);
bool ReadADC(void);
void PrintADC(void);
void StartAdc(void);

// Global Vars
bool bRunLoop, pause, bDmaComplete = false;
uint32_t Seconds;

uint32_t u32DEVID, u32REVID, u32UUID[3];
HAL_StatusTypeDef  status;

// local functions
static void SystemClockConfig(void);

//
//
int main (void)
{
  bool toggleDone = false;
  uint32_t LastSeconds = 0;

  SystemClockConfig();  // start HSI and PLL for 24MHz
  status = HAL_Init();  // init and start systick (uwTick)
  u32DEVID = HAL_GetDEVID();
  u32REVID = HAL_GetREVID();
  u32UUID[0] = HAL_GetUIDw0();
  u32UUID[1] = HAL_GetUIDw1();
  u32UUID[2] = HAL_GetUIDw2();

  InitGPIO();

  InitTimer1Pwm();

  InitADC();

  while(1) {

    if (LastSeconds != Seconds)
    {
      LastSeconds = Seconds;
      PrintADC();
    }

    if (pause) {
      pause = false;
      LL_GPIO_ResetOutputPin(USER_LED);
      HAL_Delay(1000);
      LL_GPIO_SetOutputPin(USER_LED);
      HAL_Delay(1000);
    }
    else
    {
      if (!toggleDone && ( 0 == HAL_GetTick() % 250))
      {
        toggleDone = true;
        LL_GPIO_TogglePin(USER_LED);

        if (bDmaComplete)
        {
          bDmaComplete = false;
        }
//        if (ReadADC())
//          PrintADC();
      }
      else
        toggleDone = false;
    }

    while (!bRunLoop)
      __WFI();

  }
}

/**
  * @brief  System clock configuration function
  * @param  none
  * @retval none
  * Configuration:
  * HSI enabled (24MHZ)
  * SYSCLK, AHB/HCLK, APB/PCLK = 24 MHz
  */
static void SystemClockConfig(void)
{
  /* HSI enable and initialize */
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB frequency division */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);  // no divide

  /* Configure HSISYS as system clock and initialization */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);  // no wait

  /* Set APB1 frequency division and initialization */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);  // no divide

  /* Update the system clock global variable SystemCoreClock
     (can also be updated by calling the SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}
