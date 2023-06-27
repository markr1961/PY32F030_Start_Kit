// system IRQs

#include "main.h"
#include "pin-port.h"

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler()
{
  HAL_IncTick();
  uint32_t currentTick = HAL_GetTick();
  if (0 == currentTick % 10)
    bRunLoop = true;
  if (0 == currentTick % 1000)
    Seconds++;
}

//
// EXTI 12 is connected to the user button.
//
void EXTI4_15_IRQHandler(void)
{
  if(LL_EXTI_IsActiveFlag(LL_EXTI_LINE_12))
  {
    LL_GPIO_TogglePin(USER_LED);
    LL_EXTI_ClearFlag(LL_EXTI_LINE_12);
    pause = true;
    while (0 == LL_GPIO_IsInputPinSet(USER_BUTTON))
      ;
  }
}
