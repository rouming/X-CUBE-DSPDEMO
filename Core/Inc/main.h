/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "defs.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define DAC_FREQ    13125

extern size_t adc_samples;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void ADC_set_size_and_restart(size_t nb_samples);
void DAC_set_freq_and_restart(uint32_t freq);
void BSP_Background(void);

void     BACKUP_SaveParameter(uint32_t address, uint32_t data);
uint32_t BACKUP_RestoreParameter(uint32_t address);

static inline void enable_cycles_counter(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	DWT->CYCCNT = 0;
}

static inline uint32_t cycles_counter(void)
{
	return DWT->CYCCNT;
}

static inline float cycles_counter_to_s(uint32_t cycnt)
{
	return (float)cycnt / SystemCoreClock;
}

static inline float cycles_counter_to_us(uint32_t cycnt)
{
	return (float)cycnt / SystemCoreClock * 1e6;
}

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RIGHT_BUTTON_Pin GPIO_PIN_3
#define RIGHT_BUTTON_GPIO_Port GPIOE
#define LEFT_BUTTON_Pin GPIO_PIN_4
#define LEFT_BUTTON_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define TC_PEN_Pin GPIO_PIN_5
#define TC_PEN_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_1
#define LCD_BL_GPIO_Port GPIOB
#define TOUCH_CS_Pin GPIO_PIN_12
#define TOUCH_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
