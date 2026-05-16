/* Includes ------------------------------------------------------------------*/
#include "moving_average.h"

/**
  * @brief  This function initializes filter's data structure.
	* @param  filter_struct : Data structure
  * @retval None.
  */
void Moving_Average_Init(FilterTypeDef* filter_struct, uint32_t window_length)
{
	filter_struct->WindowLength = window_length;

	filter_struct->Sum = 0;
	filter_struct->WindowPointer = 0;

	for(uint32_t i=0; i<(uint32_t)filter_struct->WindowLength; i++)
	{
		filter_struct->History[i] = 0;
	}
}

/**
  * @brief  This function filters data with moving average filter.
	* @param  raw_data : input raw sensor data.
	* @param  filter_struct : Data structure
  * @retval Filtered value.
  */
float Moving_Average_Compute(float raw_data, FilterTypeDef* filter_struct)
{
	filter_struct->Sum += raw_data;
	filter_struct->Sum -= filter_struct->History[filter_struct->WindowPointer];
	filter_struct->History[filter_struct->WindowPointer] = raw_data;
	if(filter_struct->WindowPointer < (uint32_t)(filter_struct->WindowLength - 1.0f))
	{
		filter_struct->WindowPointer += 1;
	}
	else
	{
		filter_struct->WindowPointer = 0;
	}
	return (float)filter_struct->Sum/(float)filter_struct->WindowLength;
}

/* Fill moving average filter history with raw_data */
void Moving_Average_Set_Value(float raw_data, FilterTypeDef* filter_struct){
	filter_struct->Sum = raw_data*filter_struct->WindowLength;
	filter_struct->WindowPointer = 0;

	for(uint32_t i=0; i<(uint32_t)filter_struct->WindowLength; i++)
	{
		filter_struct->History[i] = raw_data;
	}
}

