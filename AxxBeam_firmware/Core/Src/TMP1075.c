/*
 * TMP1075.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Axel Johansson
 */


/* Includes ------------------------------------------------------------------*/
#include "TMP1075.h"


// Read the temperature from the TMP1075
float TMP1075_Read_Temperature(I2C_HandleTypeDef *hi2c) {
	uint8_t temp_reg_addr = TMP1075_TEMP_REGISTER;	// Temperature register address in the temperature IC
	uint8_t temp_raw_data[2] = {0}; 		// Array to hold the raw temperature data (MSB + LSB)
    int16_t temp_raw;  						// 16-bit variable to hold the combined raw temperature value
    float temperature;  					// Variable to hold the calculated temperature

    // Send the temperature register address
    if (HAL_I2C_Master_Transmit(hi2c, TMP1075_I2C_ADDRESS, (uint8_t*) &temp_reg_addr, 1, TMP1075_I2C_TIMEOUT_MS) != HAL_OK) {
        // Transmission error
        return TMP1075_READ_ERROR;
    }

    // Receive 2 bytes of temperature data from the TMP1075
    if (HAL_I2C_Master_Receive(hi2c, TMP1075_I2C_ADDRESS, temp_raw_data, 2, TMP1075_I2C_TIMEOUT_MS) != HAL_OK) {
        // Reception error
        return TMP1075_READ_ERROR;
    }

    // Combine the MSB and LSB into a 16-bit value
    temp_raw = temp_raw_data[0] << 8 ;
    temp_raw = temp_raw | temp_raw_data[1] ;

    // Right shift the result for 12-bit resolution (TMP1075 in default 12-bit mode)
    temp_raw >>= 4;

    // Convert the raw temperature to Celsius (TMP1075 is 0.0625°C/LSB)
    temperature = temp_raw * 0.0625;

    return temperature;
}
