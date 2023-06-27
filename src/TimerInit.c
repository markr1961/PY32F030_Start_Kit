//
// Initialize Timer1 as 3 channel PWM
//
#include "main.h"

static void ConfigTIM1Base(void);
static void ConfigPWMChannel(void);


// RC servo timing
// from https://en.wikipedia.org/wiki/Servo_control
// 20mS period (50Hz)
// 1500uS pulse = center
// 2000uS pulse = full CW (+90 degree)
// 1000uS pulse = full CCW (-90 degree)
// 1000 usable counts = 1us/count -> 1Mhz clock source.
// 20 mS period = 20,000 1uS ticks

// Use a logic analyzer to observe the waveform of pin PA8/PA9/PA10
// PA8 -> CH1 duty cycle: Sevro centered
// PA9 -> CH2 duty cycle: Servo minimum
// PA10 -> CH3 duty cycle: Servo maximum

// Servo pinout:
// gnd: black/brown
// +5V: red/center
// PWM: orange/yellow
// NOTE: Futaba uses a different pinout!

#define SERVO_CENTER  1500
#define SERVO_MIN     1000
#define SERVO_MAX     2000

bool  SetServoOutput(int channel, int percent)
{
    if (percent < 0 || percent > 100)
  {
    DEBUG_BREAK();
    return false;
  }
  uint32_t CompareValue = (uint32_t)percent * 10;  // 0->1000
  CompareValue += SERVO_MIN;

  switch (channel)
  {
  case 0:
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH1(TIM1, CompareValue);
    break;
  case 1:
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH2(TIM1, CompareValue);
    break;
  case 2:
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH3(TIM1, CompareValue);
    break;
  case 3:
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH4(TIM1, CompareValue);
    break;
  default:
    DEBUG_BREAK();
    break;
  }
  return true;
}
void InitTimer1Pwm(void)
{
  /* Enable TIM1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  /* Configure TIM1 PWM channel and PWM mode */
  ConfigPWMChannel();
  /* Configure and enable TIM1 PWM2 mode */
  ConfigTIM1Base();

}
/**
   * @brief Configure TIM1 PWM related GPIO
   * @param None
   * @retval None
   */
static void ConfigPWMChannel(void)
{
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct = {0};

  TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM2;     /* Mode: PWM1 */
  TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;  /* channel open */
  TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH; /* Effective polarity: high level */
  TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW; /* idle state: low level */
  /* Channel 1 comparison value: */
  TIM_OC_Initstruct.CompareValue  = SERVO_CENTER;
  /* configure channel 1 */
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_Initstruct);
  /* Channel 2 comparison value: */
  TIM_OC_Initstruct.CompareValue  = SERVO_MIN;
  /* configure channel 2 */
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_Initstruct);
  /* Channel 3 comparison value: */
  TIM_OC_Initstruct.CompareValue  = SERVO_MAX;
  /* configure channel 3 */
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_Initstruct);
}

/**
   * @brief configure TIM base
   * @param None
   * @retval None
   */
static void ConfigTIM1Base(void)
{
  /* Configure TIM1 */
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;   /* clock without frequency division */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;       /* Up counting mode */
  TIM1CountInit.Prescaler           = 24 - 1;                      /* Prescaler value: 24 = 1MHz */
  TIM1CountInit.Autoreload          = 20000 - 1;                   /* Auto reload value: 20,000uS period */
  TIM1CountInit.RepetitionCounter   = 0;                           /* Repeat count value: 0 */

  /* Initialize TIM1 */
  LL_TIM_Init(TIM1, &TIM1CountInit);

  /* Main output enable */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 counter */
  LL_TIM_EnableCounter(TIM1);
}
