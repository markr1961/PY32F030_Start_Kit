#include "main.h"

void SysTick_Handler()
{
    HAL_IncTick();
}

bool pause;

void EXTI4_15_IRQHandler(void)
{
  if(LL_EXTI_IsActiveFlag(LL_EXTI_LINE_12))
  {
    LL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
    LL_EXTI_ClearFlag(LL_EXTI_LINE_12);
    pause = true;
    while (0 == LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_12))
      ;
  }
}

void InputGPIO(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA11 as output (LED) */
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){.Mode = GPIO_MODE_OUTPUT_OD, .Pin = GPIO_PIN_11});

  /* PA12 as input */
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Triggerred by falling edge */
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

   /**
   * Enable interrupt
   * - EXTI0_1_IRQn for PA/PB/PC[0,1]
   * - EXTI2_3_IRQn for PA/PB/PC[2,3]
   * - EXTI4_15_IRQn for PA/PB/PC[4,15]
  */
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

int main (void)
{
    HAL_Init();
    SystemCoreClockUpdate();
    InputGPIO();
    while(1) {

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
      HAL_Delay(100);

      if (pause)  {
        pause = false;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_Delay(1000);
      }

      __WFI();

    }
}
