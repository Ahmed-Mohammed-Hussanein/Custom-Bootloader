/*
 * stm32f103c6_CRC32_Driver.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Ahmed
 */


// =============================================
// ================== Includes =================
// =============================================

#include "Platform_Types.h"
#include "stm32f103c6_BitField.h"
#include "stm32f10xxx_device_header.h"
#include "stm32f103c6_RCC_Driver.h"
#include "stm32f103c6_CRC32_Driver.h"

/*
 * =====================================================================================
 * ================================= APIs Function Definition ==========================
 * =====================================================================================
 */

void MCAL_CRC_clockEnable(void)
{
	MCAL_RCC_AHB_enableClock(RCC_AHB_PERIPHERAL_CRC);
}

uint32_t MCAL_CRC_Calculate(uint8_t pBuffer[], uint32_t length, uint8_t dataSize)
{
	uint32_t index;
	uint32_t temp;

	// reset the CRC peripheral
	SET_MASK(CRC->CR, CRC_CR_RESET);

	// Calculation
	for(index = 0; index < length; index += dataSize)
	{
		if(dataSize == 1)
		{
			CRC->DR	=	*((uint8_t*)pBuffer + index);
		}
		else if(dataSize == 2)
		{
			CRC->DR	=	*((uint16_t*)pBuffer + index);
		}
		else if(dataSize == 4)
		{
			CRC->DR	=	*((uint32_t*)pBuffer + index);
		}
		else
		{
			CRC->DR	=	*((uint32_t*)pBuffer + index);
			CRC->DR	=	*((uint32_t*)pBuffer + index + 4);
		}
		//		CRC->DR	=	pBuffer[index];
	}

	// read the result
	temp = CRC->DR;

	// return the value
	return temp;
}

uint32_t MCAL_CRC_Accumelate(uint8_t pBuffer[], uint32_t length, uint8_t dataSize)
{
	uint32_t index;
	uint32_t temp;

	// Calculation
	for(index = 0; index < length; index += dataSize)
	{
		if(dataSize == 1)
		{
			CRC->DR	=	*((uint8_t*)pBuffer + index);
		}
		else if(dataSize == 2)
		{
			CRC->DR	=	*((uint16_t*)pBuffer + index);
		}
		else if(dataSize == 4)
		{
			CRC->DR	=	*((uint32_t*)pBuffer + index);
		}
		else
		{
			CRC->DR	=	*((uint32_t*)pBuffer + index);
			CRC->DR	=	*((uint32_t*)pBuffer + index + 4);
		}
		//		CRC->DR	=	pBuffer[index];
	}

	// read the result
	temp = CRC->DR;

	// return the value
	return temp;
}
