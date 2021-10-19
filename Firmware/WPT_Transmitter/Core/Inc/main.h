/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define USR1_Pin GPIO_PIN_0
#define USR1_GPIO_Port GPIOA
#define USR1_EXTI_IRQn EXTI0_1_IRQn
#define USR2_Pin GPIO_PIN_1
#define USR2_GPIO_Port GPIOA
#define USR2_EXTI_IRQn EXTI0_1_IRQn
#define LD1_Pin GPIO_PIN_2
#define LD1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_3
#define LD2_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_4
#define LD3_GPIO_Port GPIOA
#define GD_EN_Pin GPIO_PIN_5
#define GD_EN_GPIO_Port GPIOA
#define EX1_Pin GPIO_PIN_6
#define EX1_GPIO_Port GPIOA
#define EX2_Pin GPIO_PIN_7
#define EX2_GPIO_Port GPIOA
#define LIN_Pin GPIO_PIN_1
#define LIN_GPIO_Port GPIOB
#define EX3_Pin GPIO_PIN_9
#define EX3_GPIO_Port GPIOA
#define HIN_Pin GPIO_PIN_10
#define HIN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
