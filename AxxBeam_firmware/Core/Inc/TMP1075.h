#ifndef INC_TMP1075_H
#define INC_TMP1075_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32g4xx_hal.h"

/* Defines ------------------------------------------------------------------*/
// I2C address of the TMP1075 sensor (7-bit address)
#define TMP1075_I2C_ADDRESS (0x48 << 1)  // 8-bit address for HAL - IC address is 0x49
#define TMP1075_TEMP_REGISTER 0x00  // Temperature register address

/* Function prototypes -------------------------------------------------------*/
float TMP1075_Read_Temperature(I2C_HandleTypeDef *hi2c);

#endif
