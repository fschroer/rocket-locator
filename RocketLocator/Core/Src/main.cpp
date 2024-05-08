#include "main.h"
#include "subghz_phy_app.h"
#include "RocketFactory.hpp"
#include "sys_app.h"
#include "time.h"

//#define MAX_APP_BUFFER_SIZE 255
//#define PAYLOAD_LEN 64

#define RTC_CLOCK_SOURCE_LSE
/*#define RTC_CLOCK_SOURCE_LSI*/

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
enum DeviceState device_state_ = kStandby;

int main(void){
  HAL_Init();
  SystemClock_Config();
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
  //UTIL_TIMER_Create(&peripheral_service_timer_, MILLIS_PER_SECOND / SAMPLES_PER_SECOND, UTIL_TIMER_PERIODIC, OnPeripheralServiceTimer, NULL);
  //UTIL_TIMER_Start(&peripheral_service_timer_);
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->PSC = HAL_RCC_GetPCLK1Freq()/1000000 - 1;
  TIM2->CR1 = TIM_CR1_EN;
  while (1){
  	if (TIM2->CNT > 0){
      peripheral_service_interrupts_--;
      m_rocket_service_state = 1;
      rocket_factory_.ProcessRocketEvents();
      m_rocket_service_state = 0;
  	}
  }
}

//void OnPeripheralServiceTimer(void *context) {
//	peripheral_service_interrupts_++;
//}

void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability */
  //HAL_PWR_EnableBkUpAccess();
  //__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks */
  //RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  //RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers */
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

void UART1_CharReception_Callback(void){
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  uint8_t usart1_char = LL_USART_ReceiveData8(USART1);
  if( usart1_char > 0x90)
    int j = 0;
  rocket_factory_.ProcessUART1Char(usart1_char);
  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); /* LED_BLUE */
  //HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
  //HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
}

void UART2_CharReception_Callback(void){
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
	rocket_factory_.ProcessUART2Char(&huart2, LL_USART_ReceiveData8(USART2));
}

void UART_TXEmpty_Callback(void){
}

void UART_CharTransmitComplete_Callback(void){
}

void UART_Error_Callback(void){
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
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

void Error_Handler(void){
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
