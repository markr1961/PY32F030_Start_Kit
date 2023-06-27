
#include "main.h"
#include "py32f0xx_ll_adc.h"
#include "py32f0xx_ll_dma.h"

/* Private define ------------------------------------------------------------*/
#define ADC_CALIBRATION_TIMEOUT_MS       ((uint32_t) 1)
#define VDDA_APPLI                       ((uint32_t)3300)
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

//#define VOLTAGE_READING   // print voltage instead of temperture

/* Private variables ---------------------------------------------------------*/
uint16_t uhADCxConvertedData_Voltage, uhADCxConvertedData_Temperature;
// For DMA storage:
uint32_t uwADCxRawData[4];
uint16_t uhADCxConvertedData[4] = {0};

/* Private function prototypes -----------------------------------------------*/
static void APP_AdcSingleConfig(void);
static void APP_AdcDmaConfig(void);
static void APP_AdcEnable(void);
static void APP_AdcCalibrate(void);
static void APP_DmaConfig(void);

void StartAdc(void)
{
  APP_AdcEnable();

  /* 开始ADC转换(如果是软件触发则直接开始转换) */
  /* Start ADC conversion (if it is triggered by software,
     it will start conversion directly) */
  LL_ADC_REG_StartConversion(ADC1);
}

void InitADC(void)
{
  LL_ADC_Reset(ADC1);

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  APP_AdcCalibrate();

  APP_DmaConfig();

  /* 配置ADC相关参数 */
  /* Configure ADC related parameters */
  //APP_AdcSingleConfig();
  APP_AdcDmaConfig();

  StartAdc();

}

bool ReadADC(void)
{
  bool result = false;
  uint16_t uhADCxConverterData;

  /* 判断ADC序列结束标志 */
  /* Determine the end flag of the ADC sequence */
  if(LL_ADC_IsActiveFlag_EOS(ADC1))
  {
    LL_ADC_ClearFlag_EOS(ADC1);

    /* 获取ADC转换数据 */
    /* Get ADC conversion data */
    uhADCxConverterData = LL_ADC_REG_ReadConversionData12(ADC1);

    /* Convert ADC raw data to physical value */

#ifdef VOLTAGE_READING
    /* 计算VCC电压 */
    /* Calculate VCC voltage */
    uhADCxConvertedData_Voltage = (uint16_t)((4095 * 1200)
                                            / (uint32_t)uhADCxConverterData);

#else // temperature
    /*
    * @brief  Helper macro to calculate the temperature (unit: degree Celsius)
    *         from ADC conversion data of internal temperature sensor.
    * @param  __VREFANALOG_VOLTAGE__  Analog reference voltage (unit: mV)
    * @param  __TEMPSENSOR_ADC_DATA__ ADC conversion data of internal
    *                                 temperature sensor (unit: digital value).
    * @param  __ADC_RESOLUTION__      ADC resolution at which internal temperature
    *                                 sensor voltage has been measured.
    */
    uhADCxConvertedData_Temperature = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI,
                                                            uhADCxConverterData,
                                                            LL_ADC_RESOLUTION_12B);
#endif

    result = true;

  }
  return (result);
}

void PrintADC(void)
{
    for (int i = 0; i < 4; i++)
    {
      printf("Channel %d: %4dmV, raw: 0x%04X.\r\n", i+4, uhADCxConvertedData[i], uwADCxRawData[i]);
    }
    printf("Vcc: %4dmV.\r\n", uhADCxConvertedData_Voltage);
    printf("Temperature: %3dC.\r\n",uhADCxConvertedData_Temperature);
}

/**
  * @brief  DMA传输完成回调函数 DMA transfer complete callback function
  * @param  无
  * @retval 无
  */
void APP_DMATransferCompleteCallback()
{
  for(int i = 0; i < 4; i++)
  {
    switch (i)
    {
    case 2: // temperature (ADC_IN10)
      uhADCxConvertedData_Temperature = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI,
                                                            uwADCxRawData[i],
                                                            LL_ADC_RESOLUTION_12B);
      break;
    case 3: // voltage (ADC_IN11)
      uhADCxConvertedData_Voltage = (uint16_t)((4095 * 1200) / (uint32_t)uwADCxRawData[i]);
      break;
    }

    uhADCxConvertedData[i] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI,
                                                           uwADCxRawData[i],
                                                           LL_ADC_RESOLUTION_12B);

  }

  StartAdc();
}

/**
  * @brief  ADC配置函数 ADC configuration function
  * @param  无         None
  * @retval 无         None
  */
static void APP_AdcSingleConfig(void)
{
  __IO uint32_t wait_loop_index = 0;

  LL_ADC_InitTypeDef ADC_Init;
  LL_ADC_REG_InitTypeDef LL_ADC_REG_InitType;
  ADC_Common_TypeDef ADC_Common_Type;

  /* ADC通道和时钟源需在ADEN=0时配置，其余的需在ADSTART=0时配置 */
  /* ADC channels and clock sources need to be configured when ADEN=0,
     and the rest need to be configured when ADSTART=0 */

  /* ADC部分功能初始化 */
  /* ADC partial function initialization */
  ADC_Init.Clock=LL_ADC_CLOCK_SYNC_PCLK_DIV64;
  ADC_Init.DataAlignment=LL_ADC_DATA_ALIGN_RIGHT;
  ADC_Init.LowPowerMode=LL_ADC_LP_MODE_NONE;
  ADC_Init.Resolution=LL_ADC_RESOLUTION_12B;
  LL_ADC_Init(ADC1,&ADC_Init);

  /* 设置通道转换时间 */
  /* Set the channel conversion time */
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  /* ADC结构功能初始化 */
  /* ADC structure function initialization */
  LL_ADC_REG_InitType.ContinuousMode=LL_ADC_REG_CONV_CONTINUOUS;
  LL_ADC_REG_InitType.DMATransfer=LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_InitType.Overrun=LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_InitType.SequencerDiscont=LL_ADC_REG_SEQ_DISCONT_DISABLE;
  LL_ADC_REG_InitType.TriggerSource=LL_ADC_REG_TRIG_SOFTWARE;
  LL_ADC_REG_Init(ADC1,&LL_ADC_REG_InitType);

  /* ADC共用参数设置 */
  /* ADC common parameter setting */

#ifdef VOLTAGE_READING
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_VREFINT);
  wait_loop_index = ((LL_ADC_DELAY_VREFINT_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
#else // temperature
  /* ADC TempSensor 等待稳定 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
#endif

  /* ADC TempSensor waits for stabilization */
  while(wait_loop_index-- != 0)
  {
  }

#ifdef VOLTAGE_READING
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_VREFINT);
#else // temperature
  /* 设置通道11为转换通道 */
  /* Set channel 11 as conversion channel */
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
#endif
}

/**
  * @brief  ADC配置函数   ADC configuration function using DAM
  * @param  无
  * @retval 无
  * Channels: JOYSTICK_X, JOYSTICK_Y, PA6, PA7
  */
static void APP_AdcDmaConfig(void)
{
  /* ADC通道和时钟源需在ADEN=0时配置，其余的需在ADSTART=0时配置 */
  /* 配置内部转换通道 */
  /* configure internal conversion channels */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  /* 设置ADC时钟 */
  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);

  /* 设置12位分辨率 */
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);

  /* 设置数据右对齐 */
  LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

  /* 设置低功耗模式无 */
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);

  /* 设置通道转换时间 */
  //LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  /* 设置触发源为Software */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

  /* 设置转换模式为单次转换 */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

  /* 设置DMA模式为循环 */
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

  /* 设置过载管理模式为覆盖上一个值 */
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

  /* 扫描方向为向上 */
  LL_ADC_REG_SetSequencerScanDirection(ADC1,LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);

  /* 设置非连续模式为不使能 */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

  /* 设置通道4/5/6/7为转换通道 */
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4
                                      | LL_ADC_CHANNEL_5
                                      | LL_ADC_CHANNEL_VREFINT
                                      | LL_ADC_CHANNEL_TEMPSENSOR);
}

/**
  * @brief  ADC校准函数 ADC calibration function
  * @param  无
  * @retval 无
  */
static void APP_AdcCalibrate(void)
{
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
#endif /* USE_TIMEOUT */

  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* 使能校准 */
    LL_ADC_StartCalibration(ADC1);

#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
#if (USE_TIMEOUT == 1)
      /* 检测校准是否超时 */
      /* Check if the calibration has timed out */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
          break;
        }
      }
#endif /* USE_TIMEOUT */
    }

    /* ADC校准结束和使能ADC之间的延时最低4个ADC Clock */
    /* The delay between the end of ADC calibration and enabling the ADC is at least 4 ADC Clocks */
    LL_mDelay(1);
  }
}

/**
  * @brief  ADC使能函数   ADC enable function
  * @param  无
  * @retval 无
  */
static void APP_AdcEnable(void)
{
  /* 使能ADC */
  LL_ADC_Enable(ADC1);

  /* 使能ADC 稳定时间，最低8个ADC Clock */
  /* Enable ADC stabilization time, minimum 8 ADC Clock */
  LL_mDelay(1);
}

/**
  * @brief  DMA配置函数
  * @param  无
  * @retval 无
  */
static void APP_DmaConfig()
{
  /* 使能DMA1 时钟 */
  /* Enable DMA1 clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* 使能syscfg 时钟 */
  /* Enable syscfg clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  /* ADC对应通道LL_DMA_CHANNEL_1 */
  /* ADC corresponds to channel LL_DMA_CHANNEL_1 */
  //SET_BIT(SYSCFG->CFGR3, 0x0);
  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_ADC);

  /* 配置DMA传输方向为外设到存储器 */
  /* Configure DMA transfer direction as peripheral to memory */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* 配置DMA优先级为高 */
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  /* 配置DMA循环模式 */
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  /* 配置DMA外设地址不变模式 */
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  /* 配置DMA存储地址自增模式 */
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  /* 配置DMA外设传输方式为字 */
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  /* 配置DMA存储器传输方式为字 */
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* 配置DMA传输长度为4 */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);

  /* 配置DMA外设和存储器的地址 */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR,\
                         (uint32_t)uwADCxRawData, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));

  /* 使能DMA传输完成中断 */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  /* DMA中断配置 */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* 使能DMA */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

/******************************************************************************/
/* PY32F0xx Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/
/**
  * @brief This function handles DMA1_Channel1 Interrupt .
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* 检查 DMA 的 TCIF1 标志位 */
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    /* 清 DMA 的 TCIF1 标志位 */
    LL_DMA_ClearFlag_TC1(DMA1);

    /* 调用DMA完成回调函数 */
    APP_DMATransferCompleteCallback();

    bDmaComplete = true;

  }
}
