#include "main.h"
#include "subghz_phy_app.h"
#include "RocketFactory.hpp"
#include "sys_app.h"
#include "time.h"
#include "tim.h"
#include "stm32wlxx_hal_tim.h"
#include "adc.h"
#include "RocketDefs.hpp"
#include "usart.h"

#define RTC_CLOCK_SOURCE_LSE

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xF9
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif

RocketFactory rocket_factory_;

//UTIL_TIMER_Object_t peripheral_service_timer_;
//volatile int peripheral_service_interrupts_;

volatile float mAGL = 0.0;
volatile float mVelocityShortSample = 0.0;
volatile float mVelocityLongSample = 0.0;
volatile int mFlightState = FlightStates::kWaitingLaunch;
volatile DeployMode mDeployMode = DeployMode::kDroguePrimaryDrogueBackup;
volatile float mX = 0.0, mY = 0.0, mZ = 0.0;
volatile float m_g_force = 0.0;
volatile int m_rocket_service_state = 0;
volatile uint8_t m_radio_send = 0;
volatile uint8_t m_uart1_rec = 0;
volatile uint8_t m_processing_new_gga_sentence_ = 0;
volatile uint8_t m_processing_new_rmc_sentence_ = 0;
volatile uint8_t m_bad_gps_message = 0;

enum DeviceState device_state_ = kStandby;

uint32_t tim2_time = 0, previous_tim2_time = 0;
uint32_t sample_time = 0;
uint32_t uart_time = 0, previous_uart_time = 0;
uint32_t pps_start_time = 0, previous_pps_start_time = 0;
uint8_t tim2_count = 0, sample_time_avg_calc_sample_count = 0;
uint32_t sample_time_avg_accumulator = 0;

int main(void) {
  HAL_Init();
  SystemClock_Config();
  sample_time = SystemCoreClock / SAMPLES_PER_SECOND; //clock rate divided by rocket data samples per second
  MX_GPIO_Init();
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  SystemApp_Init();
  SubghzApp_Init();
  MX_USART1_UART_Init();
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  MX_USART2_UART_Init();
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  rocket_factory_.Begin();
  MX_TIM2_Init();
#ifndef BOARD_TEST
  HAL_TIM_Base_Start(&htim2);
#endif
#ifndef BOARD_REVISION_A_04
  MX_ADC_Init();
  Activate_ADC();
#endif
  //htim2.Instance->CCR1 = 127; //duty cycle on scale of 0-255
  while (1){
#ifdef BOARD_TEST
    htim2.Instance->PSC = 79;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }
    HAL_Delay (50);
    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }
    HAL_Delay (50);
    htim2.Instance->PSC = 75;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }
    HAL_Delay (50);
    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }
    HAL_Delay (50);
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }
    HAL_Delay (50);
    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }
    HAL_Delay (1750);
#endif
#ifndef BOARD_TEST
    tim2_time = __HAL_TIM_GET_COUNTER(&htim2);
  	if (tim2_time - previous_tim2_time >= sample_time){
      m_rocket_service_state = 1;
      rocket_factory_.ProcessRocketEvents(tim2_count);
      m_rocket_service_state = 0;
      previous_tim2_time = tim2_time;
      if (tim2_count < SAMPLES_PER_SECOND - 1)
        tim2_count++;
      else
        tim2_count = 0;
      m_uart1_rec = 0;
    }
#endif
  }
}

//void OnPeripheralServiceTimer(void *context) {
//	peripheral_service_interrupts_++;
//}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability */
  //HAL_PWR_EnableBkUpAccess();
  //__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
    Error_Handler();
  }
}

void LoraTxCallback() {
  rocket_factory_.ReadyToSend();
}

void LoraRxCallback(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
  rocket_factory_.ProcessIncomingLoRaMessage(payload, size, rssi, LoraSnr_FskCfo);
}

void UART1_CharReception_Callback(void) {
  m_uart1_rec = 1;
#ifndef BOARD_TEST
  previous_uart_time = uart_time;
  uart_time = __HAL_TIM_GET_COUNTER(&htim2);
  if (uart_time - previous_uart_time > 24000000){
    previous_pps_start_time = pps_start_time;
    pps_start_time = uart_time;
    sample_time_avg_accumulator += ((pps_start_time - previous_pps_start_time) / SAMPLES_PER_SECOND);
    sample_time_avg_calc_sample_count++;
    if (sample_time_avg_calc_sample_count == SAMPLE_TIME_AVG_CALC_SAMPLES){
      sample_time_avg_calc_sample_count = 0;
      sample_time = sample_time_avg_accumulator / SAMPLE_TIME_AVG_CALC_SAMPLES;
      sample_time_avg_accumulator = 0;
    }
    tim2_count = 0;
    previous_tim2_time = __HAL_TIM_GET_COUNTER(&htim2);
  }
#endif
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  rocket_factory_.ProcessUART1Char(LL_USART_ReceiveData8(USART1));
  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); /* LED_BLUE */
  //HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
  //HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
}

void UART2_CharReception_Callback(void) {
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
	rocket_factory_.ProcessUART2Char(&huart2, LL_USART_ReceiveData8(USART2));
}

void UART_TXEmpty_Callback(void) {
}

void UART_CharTransmitComplete_Callback(void) {
}

void UART_Error_Callback(void) {
  //__IO uint32_t isr_reg;

  // Disable USARTx_IRQn
  /*NVIC_DisableIRQ(USART1_IRQn);

  //Error handling example :
  //  - Read USART ISR register to identify flag that leads to IT raising
  //  - Perform corresponding error handling treatment according to flag

  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    // Turn LED3 on: Transfer error in reception/transmission process
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // LED_RED
  }
  else
  {
    // Turn LED3 on: Transfer error in reception/transmission process
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // LED_RED
  }*/
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) ||
      ((huart->ErrorCode & (HAL_UART_ERROR_RTO | HAL_UART_ERROR_ORE)) != 0U))
  {
	  if (HAL_UART_Init(huart) != HAL_OK){
	    Error_Handler();
	  }
	  LL_USART_EnableIT_RXNE(huart->Instance);
		LL_USART_EnableIT_ERROR(huart->Instance);
  }
}

//static void MX_ADC_Init(void)
//{
//
//  /* USER CODE BEGIN ADC_Init 0 */
//
//  /* USER CODE END ADC_Init 0 */
//
//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
//
//  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
//  /**ADC GPIO Configuration
//  PB4   ------> ADC_IN3
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* ADC interrupt Init */
//  NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
//  NVIC_EnableIRQ(ADC_IRQn);
//
//  /* USER CODE BEGIN ADC_Init 1 */
//
//  /* USER CODE END ADC_Init 1 */
//
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//
//   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
//   #if (USE_TIMEOUT == 1)
//   uint32_t Timeout ; /* Variable used for Timeout management */
//   #endif /* USE_TIMEOUT */
//
//  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC, &ADC_InitStruct);
//  LL_ADC_REG_SetSequencerConfigurable(ADC, LL_ADC_REG_SEQ_CONFIGURABLE);
//
//   /* Poll for ADC channel configuration ready */
//   #if (USE_TIMEOUT == 1)
//   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
//   #endif /* USE_TIMEOUT */
//   while (LL_ADC_IsActiveFlag_CCRDY(ADC) == 0)
//     {
//   #if (USE_TIMEOUT == 1)
//   /* Check Systick counter flag to decrement the time-out value */
//   if (LL_SYSTICK_IsActiveCounterFlag())
//     {
//   if(Timeout-- == 0)
//         {
//   Error_Handler();
//         }
//     }
//   #endif /* USE_TIMEOUT */
//     }
//   /* Clear flag ADC channel configuration ready */
//   LL_ADC_ClearFlag_CCRDY(ADC);
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
//  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
//  LL_ADC_REG_Init(ADC, &ADC_REG_InitStruct);
//
//   /* Enable ADC internal voltage regulator */
//   LL_ADC_EnableInternalRegulator(ADC);
//   /* Delay for ADC internal voltage regulator stabilization. */
//   /* Compute number of CPU cycles to wait for, from delay in us. */
//   /* Note: Variable divided by 2 to compensate partially */
//   /* CPU processing cycles (depends on compilation optimization). */
//   /* Note: If system core clock frequency is below 200kHz, wait time */
//   /* is only a few CPU processing cycles. */
//   uint32_t wait_loop_index;
//   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
//   while(wait_loop_index != 0)
//     {
//   wait_loop_index--;
//     }
//  LL_ADC_SetOverSamplingScope(ADC, LL_ADC_OVS_DISABLE);
//  LL_ADC_SetSamplingTimeCommonChannels(ADC, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
//  LL_ADC_SetSamplingTimeCommonChannels(ADC, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_39CYCLES_5);
//  LL_ADC_DisableIT_EOC(ADC);
//  LL_ADC_DisableIT_EOS(ADC);
//  LL_ADC_SetTriggerFrequencyMode(ADC, LL_ADC_TRIGGER_FREQ_HIGH);
//
//  /** Configure Regular Channel
//  */
//  LL_ADC_REG_SetSequencerRanks(ADC, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
//
//   /* Poll for ADC channel configuration ready */
////   #if (USE_TIMEOUT == 1)
////   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
////   #endif /* USE_TIMEOUT */
////   while (LL_ADC_IsActiveFlag_CCRDY(ADC) == 0)
////     {
////   #if (USE_TIMEOUT == 1)
////   /* Check Systick counter flag to decrement the time-out value */
////   if (LL_SYSTICK_IsActiveCounterFlag())
////     {
////   if(Timeout-- == 0)
////         {
////   Error_Handler();
////         }
////     }
////   #endif /* USE_TIMEOUT */
////     }
//   /* Clear flag ADC channel configuration ready */
////   LL_ADC_ClearFlag_CCRDY(ADC);
////  LL_ADC_SetChannelSamplingTime(ADC, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_COMMON_1);
//  /* USER CODE BEGIN ADC_Init 2 */
//
//  /* Configuration of ADC interruptions */
//  /* Enable interruption ADC group regular overrun */
////  LL_ADC_EnableIT_OVR(ADC);
//  /* USER CODE END ADC_Init 2 */
//
//}

void Activate_ADC(void) {
  __IO uint32_t wait_loop_index = 0U;
  __IO uint32_t backup_setting_adc_dma_transfer = 0U;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC) == 0)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC);

    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Disable ADC DMA transfer request during calibration */
    /* Note: Specificity of this STM32 series: Calibration factor is           */
    /*       available in data register and also transferred by DMA.           */
    /*       To not insert ADC calibration factor among ADC conversion data   */
    /*       in DMA destination address, DMA transfer must be disabled during */
    /*       calibration.                                                     */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC);
    LL_ADC_REG_SetDMATransfer(ADC, LL_ADC_REG_DMA_TRANSFER_NONE);

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC);

    /* Poll for ADC effectively calibrated */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */

    while (LL_ADC_IsCalibrationOnGoing(ADC) != 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }

    /* Restore ADC DMA transfer request after calibration */
    LL_ADC_REG_SetDMATransfer(ADC, backup_setting_adc_dma_transfer);

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(ADC);

    /* Poll for ADC ready to convert */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */

    while (LL_ADC_IsActiveFlag_ADRDY(ADC) == 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }

    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }

  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */

  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: Feature not available on this STM32 series */

}

void ConversionStartPoll_ADC_GrpRegular(void) {
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the function          */
  /*       below):                                                            */
  /*       On this STM32 series, setting of this feature is conditioned to     */
  /*       ADC state:                                                         */
  /*       ADC must be enabled without conversion on going on group regular,  */
  /*       without ADC disable command on going.                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if ((LL_ADC_IsEnabled(ADC) == 1)               &&
      (LL_ADC_IsDisableOngoing(ADC) == 0)        &&
      (LL_ADC_REG_IsConversionOngoing(ADC) == 0)   )
  {
    LL_ADC_REG_StartConversion(ADC);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
  }

  #if (USE_TIMEOUT == 1)
  Timeout = ADC_UNITARY_CONVERSION_TIMEOUT_MS;
  #endif /* USE_TIMEOUT */

  while (LL_ADC_IsActiveFlag_EOC(ADC) == 0)
  {
  #if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
      /* Time-out occurred. Set LED to blinking mode */
      LED_Blinking(LED_BLINK_SLOW);
      }
    }
  #endif /* USE_TIMEOUT */
  }

  /* Clear flag ADC group regular end of unitary conversion */
  /* Note: This action is not needed here, because flag ADC group regular   */
  /*       end of unitary conversion is cleared automatically when          */
  /*       software reads conversion data from ADC data register.           */
  /*       Nevertheless, this action is done anyway to show how to clear    */
  /*       this flag, needed if conversion data is not always read          */
  /*       or if group injected end of unitary conversion is used (for      */
  /*       devices with group injected available).                          */
  LL_ADC_ClearFlag_EOC(ADC);

}

void Error_Handler(void) {
  __disable_irq();
  while (1){
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line){
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1){
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
