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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2_protocol.h"
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
void Error_Handler(void);
void PS2_SetLeds(PS2_HandleTypeDef *dev);
void PS2_PINS_Output_OD_High(void);
void BSP_PS2_Init(void);

// This Function Provides Delay In Microseconds Using DWT
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

/* Exported variables --------------------------------------------------------*/
extern PS2_HandleTypeDef ps2_1;
extern PS2_HandleTypeDef ps2_2;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ONBOARD_LED_Pin GPIO_PIN_13
#define ONBOARD_LED_GPIO_Port GPIOC
#define KB1_Pin GPIO_PIN_1
#define KB1_GPIO_Port GPIOA
#define MOUSE1_Pin GPIO_PIN_2
#define MOUSE1_GPIO_Port GPIOA
#define KB2_Pin GPIO_PIN_3
#define KB2_GPIO_Port GPIOA
#define MOUSE2_Pin GPIO_PIN_4
#define MOUSE2_GPIO_Port GPIOA
#define CLOCK1_Pin GPIO_PIN_15
#define CLOCK1_GPIO_Port GPIOA
#define CLOCK1_EXTI_IRQn EXTI15_10_IRQn
#define DATA1_Pin GPIO_PIN_3
#define DATA1_GPIO_Port GPIOB
#define CLOCK2_Pin GPIO_PIN_4
#define CLOCK2_GPIO_Port GPIOB
#define CLOCK2_EXTI_IRQn EXTI4_IRQn
#define DATA2_Pin GPIO_PIN_6
#define DATA2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
