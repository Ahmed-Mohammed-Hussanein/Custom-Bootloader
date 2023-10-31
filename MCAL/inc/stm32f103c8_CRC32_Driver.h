/*
 * stm32f103c6_CRC32_Driver.h
 *
 *  Created on: Oct 16, 2023
 *      Author: Ahmed
 */

#ifndef INC_STM32F103C6_CRC32_DRIVER_H_
#define INC_STM32F103C6_CRC32_DRIVER_H_

// =============================================
// ================== Includes =================
// =============================================

#include "Platform_Types.h"

// ==========================================================
// ================== User Type Definitions =================
// ==========================================================


// ===================================================================
// ================== Macros Configuration Reference =================
// ===================================================================

#define CRC_DATA_SIZE_BYTE					0x1
#define CRC_DATA_SIZE_HALF_WORD				0x2
#define CRC_DATA_SIZE_WORD					0x4
#define CRC_DATA_SIZE_DOUBLE_WORD			0x8

// ===================================================
// ================== APIs Functions =================
// ===================================================

void MCAL_CRC_clockEnable(void);

uint32_t MCAL_CRC_Calculate(uint8_t pBuffer[], uint32_t length, uint8_t dataSize);
uint32_t MCAL_CRC_Accumelate(uint8_t pBuffer[], uint32_t length, uint8_t dataSize);

#endif /* INC_STM32F103C6_CRC32_DRIVER_H_ */
