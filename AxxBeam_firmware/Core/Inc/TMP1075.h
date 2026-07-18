#ifndef INC_TMP1075_H
#define INC_TMP1075_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32g4xx_hal.h"

/* Defines ------------------------------------------------------------------*/
// I2C address of the TMP1075 sensor (7-bit address 0x48, shifted for HAL)
#define TMP1075_I2C_ADDRESS (0x48 << 1)
#define TMP1075_TEMP_REGISTER 0x00  // Temperature register address
#define TMP1075_I2C_TIMEOUT_MS 100  // Never block the main loop on a stuck I2C bus
#define TMP1075_READ_ERROR -1000.0f // Returned when the I2C transaction fails

/* Function prototypes -------------------------------------------------------*/
float TMP1075_Read_Temperature(I2C_HandleTypeDef *hi2c);

#endif
