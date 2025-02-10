/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

#include "stm32wlxx_ll_adc.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_cortex.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_dma.h"

#include "stm32wlxx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_ll_usart.h"
#include "stm32_timer.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum DeviceState{
  kStandby = 0,
  kRunning,
  kConfig,
  kConfigSavePending,
  kTest
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void OnPeripheralServiceTimer(void *context);
void SystemClock_Config(void);
void LoraTxCallback();
void LoraRxCallback(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
void UART1_CharReception_Callback(void);
void UART2_CharReception_Callback(void);
void UART_TXEmpty_Callback(void);
void UART_CharTransmitComplete_Callback(void);
void UART_Error_Callback(void);
void HAL_UART1_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART2_ErrorCallback(UART_HandleTypeDef *huart);
//static void MX_ADC_Init(void);
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);
void Error_Handler(void);
//static void MX_TIM2_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)

#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_2
#define LED5_GPIO_Port GPIOB
#define POWER_SENSE_Pin GPIO_PIN_5
#define POWER_SENSE_GPIO_Port GPIOB
#define USARTx_RX_Pin GPIO_PIN_3
#define USARTx_RX_GPIO_Port GPIOA
#define USARTx_TX_Pin GPIO_PIN_2
#define USARTx_TX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define BOARD_REVISION_A_04
//#define BOARD_REVISION_A_05
//#define BOARD_REVISION_A_06
//#define BOARD_TEST

#ifdef BOARD_REVISION_A_06
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOA
#define SPKR_Pin GPIO_PIN_6
#define SPKR_GPIO_Port GPIOA
#else
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOA
#endif

#ifndef BOARD_REVISION_A_04
#define DEPLOY_1_Pin GPIO_PIN_12
#define DEPLOY_1_GPIO_Port GPIOB
#define DEPLOY_2_Pin GPIO_PIN_8
#define DEPLOY_2_GPIO_Port GPIOB
#define EMATCH_SENSE_1_Pin GPIO_PIN_10
#define EMATCH_SENSE_1_GPIO_Port GPIOA
#define EMATCH_SENSE_2_Pin GPIO_PIN_0
#define EMATCH_SENSE_2_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOA
#else
#define DEPLOY_1_Pin GPIO_PIN_8
#define DEPLOY_1_GPIO_Port GPIOB
#define DEPLOY_2_Pin GPIO_PIN_12
#define DEPLOY_2_GPIO_Port GPIOB
#define EMATCH_SENSE_1_Pin GPIO_PIN_4
#define EMATCH_SENSE_1_GPIO_Port GPIOB
#define EMATCH_SENSE_2_Pin GPIO_PIN_10
#define EMATCH_SENSE_2_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOA
#endif

#define SAMPLE_TIME_AVG_CALC_SAMPLES 10
#define ADC_FS              4095U       // ADC in 12bit resolution
#define VREF_MEAS_MV        3000U       // VDDA voltage during VREFINT factory calibration in mV.
#define VREFINT_CAL         (*((uint16_t *)0x1FFF75AA)) // DS13293 Table 14.
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
