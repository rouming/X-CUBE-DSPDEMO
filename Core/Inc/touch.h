#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  uint16_t TouchDetected;
  uint16_t X;
  uint16_t Y;
  uint16_t Z;
} TS_StateTypeDef;

typedef enum
{
  TS_OK       = 0x00,
  TS_ERROR    = 0x01,
  TS_TIMEOUT  = 0x02
} TS_StatusTypeDef;

/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State);

#endif
