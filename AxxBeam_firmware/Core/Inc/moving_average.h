#ifndef INC_MOVING_AVERAGE_H
#define INC_MOVING_AVERAGE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Definitions ---------------------------------------------------------------*/
#define Max_WindowLength 200

/* TypeDefs ------------------------------------------------------------------*/
typedef struct{
	float WindowLength;
	float History[Max_WindowLength]; /*Array to store values of filter window*/
	float Sum;	/* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
}FilterTypeDef;

/* Function prototypes -------------------------------------------------------*/
void Moving_Average_Init(FilterTypeDef* filter_struct, uint32_t window_length);
float Moving_Average_Compute(float raw_data, FilterTypeDef* filter_struct);
void Moving_Average_Set_Value(float raw_data, FilterTypeDef* filter_struct);

#endif
