/**
  ******************************************************************************
  * @file    STM32F429_DSPDEMO/Src/fft_processing.c
  * @author  MCD Application Team
  * @brief   FFT calculation Service Routines
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

/* Includes ------------------------------------------------------------------*/
#include "arm_const_structs.h"
#include "fft.h"
#include "display.h"
#include "interpolation.h"

float32_t fft_interpolated_out_f32[GRAPH_WIDTH];

float32_t FFT_Input_f32[FFT_Length_Tab*2];
float32_t FFT_Output_f32[FFT_Length_Tab];

q15_t FFT_Input_Q15[FFT_Length_Tab*2];
q15_t FFT_Output_Q15[FFT_Length_Tab];

q31_t FFT_Input_Q31[FFT_Length_Tab*2];
q31_t FFT_Output_Q31[FFT_Length_Tab];

size_t adc_samples = 64;
uint16_t adc_buf[ADC_BUF_SZ];

/**
  * @brief  This function Calculate FFT in Q15.
  * @param  FFT Length : 1024, 256, 64
  * @retval None
  */
void FFT_PROCESSING_Q15Process(uint32_t FFT_Length)
{
	const arm_cfft_instance_q15 *cfft_q15 = NULL;
	const float32_t *fft_out_f32;

	q15_t maxValue;    /* Max FFT value is stored here */
	uint32_t maxIndex;    /* Index in Output array where max value is */

	size_t i, fft_len_div2, nb_cycles, duration_us;

	for (i = 0; i < FFT_Length; i++) {
		float32_t sample;

		sample = adc_buf[i];
		/* Convert to [0, 1] range */
		sample /= 4096;
		/* Remove DC */
		sample -= 0.5;

		/* Real part */
		FFT_Input_f32[i * 2] = sample;
		/* Imaginary part */
		FFT_Input_f32[i * 2 + 1] = 0;
	}

	switch (FFT_Length) {
	case 64:
		cfft_q15 = &arm_cfft_sR_q15_len64;
		break;
	case 256:
		cfft_q15 = &arm_cfft_sR_q15_len256;
		break;
	case 1024:
		cfft_q15 = &arm_cfft_sR_q15_len1024;
		break;
	}

	arm_float_to_q15(FFT_Input_f32, FFT_Input_Q15, FFT_Length*2);

	nb_cycles = cycles_counter();
	/* Run FFT */
	arm_cfft_q15(cfft_q15, FFT_Input_Q15, FFT_INVERSE_FLAG, FFT_DC_POS_NEG_ORDER_FLAG);
	nb_cycles = cycles_counter() - nb_cycles;

	GUI_Clear();
	LCD_OUTPUT_Cycles(5, 305, nb_cycles);
	duration_us = (uint32_t)cycles_counter_to_us(nb_cycles);
	LCD_OUTPUT_DURATION(120, 305, duration_us);

	/* We are interested only in DC and positive frequencies */
	fft_len_div2 = FFT_Length/2;

	/* Calculate magnitude */
	arm_cmplx_mag_q15(FFT_Input_Q15, FFT_Output_Q15, fft_len_div2);

	/* Calculates maxValue and returns corresponding value */
	arm_max_q15(FFT_Output_Q15, fft_len_div2, &maxValue, &maxIndex);
	maxValue = 0;

	/* Convert to float for further processing */
	arm_q15_to_float(FFT_Output_Q15, FFT_Output_f32, fft_len_div2);

	for (i = 0; i < adc_samples; i++) {
		GRAPH_DATA_YT_AddValue(aGraph_Data[1], adc_buf[i] / 50 + 50);
	}

	if (fft_len_div2 < GRAPH_WIDTH) {
		BUILD_BUG_ON(GRAPH_WIDTH != ARRAY_SIZE(fft_interpolated_out_f32));
		interpolate(FFT_Output_f32, fft_len_div2,
					fft_interpolated_out_f32, GRAPH_WIDTH,
					lin_interpolation);
		fft_out_f32 = fft_interpolated_out_f32;
	} else {
		fft_out_f32 = FFT_Output_f32;
	}

	for (i = 0; i < GRAPH_WIDTH; i++) {
		GRAPH_DATA_YT_AddValue(aGraph_Data[0], fft_out_f32[i] * 600 + 10);
	}
}

/**
  * @brief  This function Calculate FFT in F32.
  * @param  FFT Length : 1024, 256, 64
  * @retval None
  */
void FFT_PROCESSING_F32Process(uint32_t FFT_Length)
{
	const arm_cfft_instance_f32 *cfft_f32 = NULL;
	const float32_t *fft_out_f32;

	float32_t maxValue;    /* Max FFT value is stored here */
	uint32_t maxIndex;    /* Index in Output array where max value is */

	size_t i, fft_len_div2, nb_cycles, duration_us;

	for (i = 0; i < FFT_Length; i++) {
		float32_t sample;

		sample = adc_buf[i];
		/* Convert to [0, 1] range */
		sample /= 4096;
		/* Remove DC */
		sample -= 0.5;

		/* Real part */
		FFT_Input_f32[i * 2] = sample;
		/* Imaginary part */
		FFT_Input_f32[i * 2 + 1] = 0;
	}

	switch (FFT_Length) {
	case 64:
		cfft_f32 = &arm_cfft_sR_f32_len64;
		break;
	case 256:
		cfft_f32 = &arm_cfft_sR_f32_len256;
		break;
	case 1024:
		cfft_f32 = &arm_cfft_sR_f32_len1024;
		break;
	}

	nb_cycles = cycles_counter();
	/* Run FFT */
	arm_cfft_f32(cfft_f32, FFT_Input_f32, FFT_INVERSE_FLAG, FFT_DC_POS_NEG_ORDER_FLAG);
	nb_cycles = cycles_counter() - nb_cycles;

	GUI_Clear();
	LCD_OUTPUT_Cycles(5, 305, nb_cycles);
	duration_us = (uint32_t)cycles_counter_to_us(nb_cycles);
	LCD_OUTPUT_DURATION(120, 305, duration_us);

	/* We are interested only in DC and positive frequencies */
	fft_len_div2 = FFT_Length/2;

	/* Calculate magnitude */
	arm_cmplx_mag_f32(FFT_Input_f32, FFT_Output_f32, fft_len_div2);

	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(FFT_Output_f32, fft_len_div2, &maxValue, &maxIndex);
	maxValue = 0;

	for (i = 0; i < adc_samples; i++) {
		GRAPH_DATA_YT_AddValue(aGraph_Data[1], adc_buf[i] / 50 + 50);
	}

	if (fft_len_div2 < GRAPH_WIDTH) {
		BUILD_BUG_ON(GRAPH_WIDTH != ARRAY_SIZE(fft_interpolated_out_f32));
		interpolate(FFT_Output_f32, fft_len_div2,
					fft_interpolated_out_f32, GRAPH_WIDTH,
					lin_interpolation);
		fft_out_f32 = fft_interpolated_out_f32;
	} else {
		fft_out_f32 = FFT_Output_f32;
	}

	for (i = 0; i < GRAPH_WIDTH; i++) {
		GRAPH_DATA_YT_AddValue(aGraph_Data[0], fft_out_f32[i] * 5 + 10);
	}
}

/**
  * @brief  This function Calculate FFT in Q31.
  * @param  FFT Length : 1024, 256, 64
  * @retval None
  */
void FFT_PROCESSING_Q31Process(uint32_t FFT_Length)
{
	const arm_cfft_instance_q31 *cfft_q31 = NULL;
	const float32_t *fft_out_f32;

	q31_t maxValue;    /* Max FFT value is stored here */
	uint32_t maxIndex;    /* Index in Output array where max value is */

	size_t i, fft_len_div2, nb_cycles, duration_us;

	for (i = 0; i < FFT_Length; i++) {
		float32_t sample;

		sample = adc_buf[i];
		/* Convert to [0, 1] range */
		sample /= 4096;
		/* Remove DC */
		sample -= 0.5;

		/* Real part */
		FFT_Input_f32[i * 2] = sample;
		/* Imaginary part */
		FFT_Input_f32[i * 2 + 1] = 0;
	}

	switch (FFT_Length) {
	case 64:
		cfft_q31 = &arm_cfft_sR_q31_len64;
		break;
	case 256:
		cfft_q31 = &arm_cfft_sR_q31_len256;
		break;
	case 1024:
		cfft_q31 = &arm_cfft_sR_q31_len1024;
		break;
	}

	arm_float_to_q31(FFT_Input_f32, FFT_Input_Q31, FFT_Length*2);

	nb_cycles = cycles_counter();
	/* Run FFT */
	arm_cfft_q31(cfft_q31, FFT_Input_Q31, FFT_INVERSE_FLAG, FFT_DC_POS_NEG_ORDER_FLAG);
	nb_cycles = cycles_counter() - nb_cycles;

	GUI_Clear();
	LCD_OUTPUT_Cycles(5, 305, nb_cycles);
	duration_us = (uint32_t)cycles_counter_to_us(nb_cycles);
	LCD_OUTPUT_DURATION(120, 305, duration_us);

	/* We are interested only in DC and positive frequencies */
	fft_len_div2 = FFT_Length/2;

	/* Calculate magnitude */
	arm_cmplx_mag_q31(FFT_Input_Q31, FFT_Output_Q31, fft_len_div2);

	/* Calculates maxValue and returns corresponding value */
	arm_max_q31(FFT_Output_Q31, fft_len_div2, &maxValue, &maxIndex);
	maxValue = 0;

	/* Convert to float for further processing */
	arm_q31_to_float(FFT_Output_Q31, FFT_Output_f32, fft_len_div2);

	for (i = 0; i < adc_samples; i++) {
		GRAPH_DATA_YT_AddValue(aGraph_Data[1], adc_buf[i] / 50 + 50);
	}

	if (fft_len_div2 < GRAPH_WIDTH) {
		BUILD_BUG_ON(GRAPH_WIDTH != ARRAY_SIZE(fft_interpolated_out_f32));
		interpolate(FFT_Output_f32, fft_len_div2,
					fft_interpolated_out_f32, GRAPH_WIDTH,
					lin_interpolation);
		fft_out_f32 = fft_interpolated_out_f32;
	} else {
		fft_out_f32 = FFT_Output_f32;
	}

	for (i = 0; i < GRAPH_WIDTH; i++) {
		GRAPH_DATA_YT_AddValue(aGraph_Data[0], fft_out_f32[i] * 600 + 10);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics ************************/
