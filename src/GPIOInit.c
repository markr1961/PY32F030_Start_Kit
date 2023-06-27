// module initialization
#include "main.h"
#include "pin-port.h"

void InitGPIO(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA11 as output (LED) */
  HAL_GPIO_Init(LED_PORT, &(GPIO_InitTypeDef){.Mode = GPIO_MODE_OUTPUT_OD, .Pin = LED_PIN});

  /* PA12 as input */
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  /* Triggerred by falling edge */
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* Configure pin PA4/5/6/7 as analog input */
  LL_GPIO_SetPinMode(JOYSTCK_PORT,
                     JOYSTICK_X_PIN | JOYSTICK_Y_PIN | LL_GPIO_PIN_6 | LL_GPIO_PIN_7,
                     LL_GPIO_MODE_ANALOG);

   /**
   * Enable interrupt
   * - EXTI0_1_IRQn for PA/PB/PC[0,1]
   * - EXTI2_3_IRQn for PA/PB/PC[2,3]
   * - EXTI4_15_IRQn for PA/PB/PC[4,15]
  */
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  LL_GPIO_InitTypeDef TIM1CH1MapInit = {0};
  /* Configure PA8/PA9/PA10 as TIM1_CH1N/TIM1_CH1/TIM1_CH2/TIM1_CH3*/
  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &TIM1CH1MapInit);
}
