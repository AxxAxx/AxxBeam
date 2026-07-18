#include <flash.h>
#include "main.h"
#include <math.h>
#include <stdio.h>
#include "stdlib.h"

uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (numberofwords--)
	{
		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 8; // one 32-bit word is stored per 64-bit flash double word
		RxBuf++;
	}
}

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;

		uint32_t PageError;
	int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	   uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords*8 - 1; // one 32-bit word per 64-bit double word
	  uint32_t EndPage = GetPage(EndPageAdress);

		 /* Clear OPTVERR bit set on virgin samples */
		  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	   /* Fill EraseInit structure*/
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Page        = StartPage;
		EraseInitStruct.NbPages     = (EndPage - StartPage) + 1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  HAL_FLASH_Lock();
		  return HAL_FLASH_GetError();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 8;//8  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 HAL_FLASH_Lock();
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}
