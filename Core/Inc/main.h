/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADDR_13_Pin GPIO_PIN_13
#define ADDR_13_GPIO_Port GPIOC
#define ADDR_14_Pin GPIO_PIN_14
#define ADDR_14_GPIO_Port GPIOC
#define ADDR_15_Pin GPIO_PIN_15
#define ADDR_15_GPIO_Port GPIOC
#define ADDR_0_Pin GPIO_PIN_0
#define ADDR_0_GPIO_Port GPIOC
#define ADDR_1_Pin GPIO_PIN_1
#define ADDR_1_GPIO_Port GPIOC
#define ADDR_2_Pin GPIO_PIN_2
#define ADDR_2_GPIO_Port GPIOC
#define ADDR_3_Pin GPIO_PIN_3
#define ADDR_3_GPIO_Port GPIOC
#define IO_0_Pin GPIO_PIN_0
#define IO_0_GPIO_Port GPIOA
#define IO_1_Pin GPIO_PIN_1
#define IO_1_GPIO_Port GPIOA
#define IO_2_Pin GPIO_PIN_2
#define IO_2_GPIO_Port GPIOA
#define IO_3_Pin GPIO_PIN_3
#define IO_3_GPIO_Port GPIOA
#define IO_4_Pin GPIO_PIN_4
#define IO_4_GPIO_Port GPIOA
#define IO_5_Pin GPIO_PIN_5
#define IO_5_GPIO_Port GPIOA
#define IO_6_Pin GPIO_PIN_6
#define IO_6_GPIO_Port GPIOA
#define IO_7_Pin GPIO_PIN_7
#define IO_7_GPIO_Port GPIOA
#define ADDR_4_Pin GPIO_PIN_4
#define ADDR_4_GPIO_Port GPIOC
#define ADDR_5_Pin GPIO_PIN_5
#define ADDR_5_GPIO_Port GPIOC
#define ADDR_16_Pin GPIO_PIN_0
#define ADDR_16_GPIO_Port GPIOB
#define ADDR_17_Pin GPIO_PIN_1
#define ADDR_17_GPIO_Port GPIOB
#define ADDR_18_Pin GPIO_PIN_2
#define ADDR_18_GPIO_Port GPIOB
#define M2_CE_Pin GPIO_PIN_10
#define M2_CE_GPIO_Port GPIOB
#define M3_CE_Pin GPIO_PIN_11
#define M3_CE_GPIO_Port GPIOB
#define FET_CTRL_Pin GPIO_PIN_12
#define FET_CTRL_GPIO_Port GPIOB
#define ADDR_6_Pin GPIO_PIN_6
#define ADDR_6_GPIO_Port GPIOC
#define ADDR_7_Pin GPIO_PIN_7
#define ADDR_7_GPIO_Port GPIOC
#define ADDR_8_Pin GPIO_PIN_8
#define ADDR_8_GPIO_Port GPIOC
#define ADDR_9_Pin GPIO_PIN_9
#define ADDR_9_GPIO_Port GPIOC
#define OUT_EN_Pin GPIO_PIN_11
#define OUT_EN_GPIO_Port GPIOA
#define WRITE_EN_Pin GPIO_PIN_12
#define WRITE_EN_GPIO_Port GPIOA
#define ADDR_10_Pin GPIO_PIN_10
#define ADDR_10_GPIO_Port GPIOC
#define ADDR_11_Pin GPIO_PIN_11
#define ADDR_11_GPIO_Port GPIOC
#define ADDR_12_Pin GPIO_PIN_12
#define ADDR_12_GPIO_Port GPIOC
#define ADDR_19_Pin GPIO_PIN_4
#define ADDR_19_GPIO_Port GPIOB
#define ADDR_20_Pin GPIO_PIN_5
#define ADDR_20_GPIO_Port GPIOB
#define WDI_Pin GPIO_PIN_8
#define WDI_GPIO_Port GPIOB
#define M1_CE_Pin GPIO_PIN_9
#define M1_CE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
