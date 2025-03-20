/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Trig3_Pin GPIO_PIN_1
#define Trig3_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Dil_A2_Pin GPIO_PIN_5
#define Dil_A2_GPIO_Port GPIOA
#define Dil_A1_Pin GPIO_PIN_6
#define Dil_A1_GPIO_Port GPIOA
#define Trig1_Pin GPIO_PIN_1
#define Trig1_GPIO_Port GPIOB
#define Echo_up_Pin GPIO_PIN_15
#define Echo_up_GPIO_Port GPIOB
#define Echo_up_EXTI_IRQn EXTI15_10_IRQn
#define Trig_Pin GPIO_PIN_7
#define Trig_GPIO_Port GPIOC
#define Echof_Pin GPIO_PIN_8
#define Echof_GPIO_Port GPIOA
#define Echo_r_Pin GPIO_PIN_9
#define Echo_r_GPIO_Port GPIOA
#define Echof_l_Pin GPIO_PIN_10
#define Echof_l_GPIO_Port GPIOA
#define Echob_Pin GPIO_PIN_11
#define Echob_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Trig2_Pin GPIO_PIN_15
#define Trig2_GPIO_Port GPIOA
#define Dir_A1_Pin GPIO_PIN_3
#define Dir_A1_GPIO_Port GPIOB
#define Dir_A2_Pin GPIO_PIN_5
#define Dir_A2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
