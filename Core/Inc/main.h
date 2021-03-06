/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LV_READ_Pin GPIO_PIN_1
#define LV_READ_GPIO_Port GPIOC
#define THROTTLE_Pin GPIO_PIN_5
#define THROTTLE_GPIO_Port GPIOA
#define YAW_Pin GPIO_PIN_6
#define YAW_GPIO_Port GPIOA
#define PS_INIT_Pin GPIO_PIN_14
#define PS_INIT_GPIO_Port GPIOB
#define HEAT_OUT_Pin GPIO_PIN_15
#define HEAT_OUT_GPIO_Port GPIOB
#define WP_Pin GPIO_PIN_6
#define WP_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_7
#define TEMP_GPIO_Port GPIOC
#define SPEEDO_Pin GPIO_PIN_8
#define SPEEDO_GPIO_Port GPIOC
#define SOC_Pin GPIO_PIN_9
#define SOC_GPIO_Port GPIOC
#define TACH_Pin GPIO_PIN_8
#define TACH_GPIO_Port GPIOA
#define SW12_Pin GPIO_PIN_4
#define SW12_GPIO_Port GPIOB
#define HEAT_REQ_Pin GPIO_PIN_5
#define HEAT_REQ_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
