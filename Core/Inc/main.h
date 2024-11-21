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
#include "stm32g4xx_hal.h"

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
#define USB_CAN_Pin GPIO_PIN_13
#define USB_CAN_GPIO_Port GPIOC
#define LED_SIG_R_Pin GPIO_PIN_14
#define LED_SIG_R_GPIO_Port GPIOC
#define LED_SIG_F_Pin GPIO_PIN_15
#define LED_SIG_F_GPIO_Port GPIOC
#define ENC_A_Pin GPIO_PIN_4
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_5
#define ENC_B_GPIO_Port GPIOA
#define CS_Sensing_Pin GPIO_PIN_6
#define CS_Sensing_GPIO_Port GPIOA
#define CS_Normal_Pin GPIO_PIN_1
#define CS_Normal_GPIO_Port GPIOB
#define BOOT0_IN_Pin GPIO_PIN_8
#define BOOT0_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
