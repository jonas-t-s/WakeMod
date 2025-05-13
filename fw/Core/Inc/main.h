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
#include "stm32u0xx_hal.h"

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
#define SYS_SDN_Pin GPIO_PIN_0
#define SYS_SDN_GPIO_Port GPIOA
#define SYS_SDN_EXTI_IRQn EXTI0_1_IRQn
#define WUR_CLK_Pin GPIO_PIN_1
#define WUR_CLK_GPIO_Port GPIOA
#define WUR_IRQ_Pin GPIO_PIN_2
#define WUR_IRQ_GPIO_Port GPIOA
#define DISABLE_IN_SW_Pin GPIO_PIN_5
#define DISABLE_IN_SW_GPIO_Port GPIOA
#define OOK_SDA_Pin GPIO_PIN_7
#define OOK_SDA_GPIO_Port GPIOA
#define ANT_CTRL_Pin GPIO_PIN_1
#define ANT_CTRL_GPIO_Port GPIOB
#define WUR_RST_Pin GPIO_PIN_8
#define WUR_RST_GPIO_Port GPIOA
#define WUR_CS_Pin GPIO_PIN_10
#define WUR_CS_GPIO_Port GPIOA
#define WUR_MISO_Pin GPIO_PIN_11
#define WUR_MISO_GPIO_Port GPIOA
#define WUR_MOSI_Pin GPIO_PIN_12
#define WUR_MOSI_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SYS_SCL_Pin GPIO_PIN_3
#define SYS_SCL_GPIO_Port GPIOB
#define SYS_SDA_Pin GPIO_PIN_4
#define SYS_SDA_GPIO_Port GPIOB
#define SYS_IRQ_Pin GPIO_PIN_5
#define SYS_IRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
