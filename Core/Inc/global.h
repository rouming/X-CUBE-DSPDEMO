/**
  ******************************************************************************
  * @file    STM32F429_DSPDEMO/Inc/global.h 
  * @author  MCD Application Team
  * @brief   Header for global.c module
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_H
#define __GLOBAL_H
#define __FPU_PRESENT             1U       /*!< FPU present */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"

#include "GUI.h"
#include "WM.h"

#include "calibration.h"

#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include <stdlib.h>
#include <string.h>

//
// Recommended memory to run the sample with adequate performance
//
#define RECOMMENDED_MEMORY (1024L * 30)
#define GRID_DIST_X      	25
#define GRID_DIST_Y      	10

#define FIR_PROCESS 0
#define FFT_PROCESS 1

#define Float32 0
#define Q15 		1
#define Q31 		2
#define LPF			0
#define HPF			1

#define FFT_INVERSE_FLAG           ((uint8_t)0)
#define FFT_DC_NEG_POS_ORDER_FLAG  ((uint8_t)0)
#define FFT_DC_POS_NEG_ORDER_FLAG  ((uint8_t)1)

#define SINE_SAMPLES		64
#define ADC_BUF_SZ			2048
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);

void LCD_OUTPUT_Cycles(uint16_t,uint16_t, uint32_t);
void LCD_OUTPUT_DURATION(uint16_t,uint16_t, uint32_t);
/* Exported functions ------------------------------------------------------- */
extern uint16_t adc_buf[ADC_BUF_SZ];
extern const uint16_t sine_12bit[SINE_SAMPLES];

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#endif /* __GLOBAL_H */
/************************ (C) COPYRIGHT STMicroelectronics ************************/
