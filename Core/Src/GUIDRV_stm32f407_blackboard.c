/*********************************************************************
*          Portions COPYRIGHT 2013 STMicroelectronics                *
*          Portions SEGGER Microcontroller GmbH & Co. KG             *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2013  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.22 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to STMicroelectronics International
N.V. a Dutch company with a Swiss branch and its headquarters in Plan-
les-Ouates, Geneva, 39 Chemin du Champ des Filles, Switzerland for the
purposes of creating libraries for ARM Cortex-M-based 32-bit microcon_
troller products commercialized by Licensee only, sublicensed and dis_
tributed under the terms and conditions of the End User License Agree_
ment supplied by STMicroelectronics International N.V.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

/**
  ******************************************************************************
  * @file    LCDConf_stm3240g_eval.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    11-October-2013
  * @brief   Driver for STM3240G-EVAL board LCD
  ******************************************************************************
  * @attention
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "GUI.h"
#include "GUIDRV_FlexColor.h"
#include "main.h"

/* defines ------------------------------------------------------------*/
// Physical display size
#define XSIZE_PHYS  240
#define YSIZE_PHYS  320

/*********************************************************************
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   GUICC_565
  #error Color conversion not defined!
#endif
#ifndef   GUIDRV_FLEXCOLOR
  #error No display driver defined!
#endif

/*********************************************************************
*       FSCM 16 bit data - address pointer
*
**********************************************************************
*/
#define LCD_BASE           ((uint32_t)(0x60000000)
#define LCD_REG_ADDRESS   (*(volatile U16*)(0x60000000))
#define LCD_DATA_ADDRESS  (*(volatile U16*)(0x60080000))

/* Private Functions -----------------------------------------------------*/

/********************************************************************
*
*       LCD_IO_Init
*
* Function description:
*   Inits LCD
*/
void LCD_IO_Init(void)
{
	static uint32_t Is_LCD_IO_Initialized;

	if (Is_LCD_IO_Initialized == 0) {
		Is_LCD_IO_Initialized = 1;
	}
}

/********************************************************************
*
*       LCD_IO_WriteReg
*
* Function description:
*   Sets display register
*/
void LCD_IO_WriteReg(U16 Data)
{
  LCD_REG_ADDRESS = Data;
}

/********************************************************************
*
*       LCD_IO_WriteData
*
* Function description:
*   Writes a value to a display register
*/
void LCD_IO_WriteData(U16 Data)
{
  LCD_DATA_ADDRESS = Data;
}

/********************************************************************
*
*       LCD_IO_WriteMultipleData
*
* Function description:
*   Writes multiple values to a display register.
*/
void LCD_IO_WriteMultipleData(U16 * pData, int NumItems)
{
  while (NumItems--) {
    LCD_DATA_ADDRESS = *pData++;
  }
}

/********************************************************************
*
*       LCD_IO_ReadMultipleData
*
* Function description:
*   Reads multiple values from a display register.
*/
void LCD_IO_ReadMultipleData(U16 * pData, int NumItems)
{
  while (NumItems--) {
    *pData++ = LCD_DATA_ADDRESS;
  }
}

/********************************************************************
*
*       LCD_Delay
*
* Function description:
*   Delays execution by delay ms
*/
void LCD_Delay(uint32_t delay)
{
	HAL_Delay(delay);
}

/*
 * LCD_Backlight_On() - Turns on backlight
 */
void LCD_Backlight_On(void)
{
	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
}

/*
 * LCD_Backlight_Off() - Turns off backlight
 */
void LCD_Backlight_Off(void)
{
	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
}

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Function description:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/

void LCD_X_Config(void) {
  GUI_DEVICE * pDevice;
  CONFIG_FLEXCOLOR Config = {0};
  GUI_PORT_API PortAPI = {0};

  //
  // Set display driver and color conversion
  //
  pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, GUICC_565, 0, 0);
  //
  // Display driver configuration, required for Lin-driver
  //
  LCD_SetSizeEx (0, XSIZE_PHYS , YSIZE_PHYS);
  LCD_SetVSizeEx(0, VXSIZE_PHYS, VYSIZE_PHYS);

  //
  // Orientation
  //
  Config.Orientation = GUI_MIRROR_Y; // Landscape

  GUIDRV_FlexColor_Config(pDevice, &Config);
  //
  // Set controller and operation mode
  //
  PortAPI.pfWrite16_A0  = LCD_IO_WriteReg;
  PortAPI.pfWrite16_A1  = LCD_IO_WriteData;
  PortAPI.pfWriteM16_A1 = LCD_IO_WriteMultipleData;
  PortAPI.pfReadM16_A1  = LCD_IO_ReadMultipleData;
  GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B16);
}



/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Function description:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  (void) LayerIndex;
  (void) pData;

  switch (Cmd) {
  case LCD_X_INITCONTROLLER: {
    return 0;
  }
  default:
    r = -1;
  }
  return r;
}

/*************************** End of file ****************************/

