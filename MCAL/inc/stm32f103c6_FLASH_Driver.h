/*
 * stm32f103c6_FLASH_Driver.h
 *
 *  Created on: Jun 23, 2023
 *      Author: Ahmed
 */

#ifndef INC_STM32F103C6_FLASH_DRIVER_H_
#define INC_STM32F103C6_FLASH_DRIVER_H_

// =============================================
// ================== Includes =================
// =============================================

#include "Platform_Types.h"
#include "Utils.h"

#include "stm32f10xxx_device_header.h"

// ==========================================================
// ================== User Type Definitions =================
// ==========================================================

typedef struct
{




}	FPEC_Config_t;


// ===================================================================
// ================== Macros Configuration Reference =================
// ===================================================================

#define FLASH_PROGRAMMING_TYPE_HALF_WORD			1
#define FLASH_PROGRAMMING_TYPE_WORD					2
#define FLASH_PROGRAMMING_TYPE_DOUBLE_WORD			3

/* The difference between 2 consecutive pages */
#define FLASH_PAGES_OFFSET							0x400


#define FLASH_PAGE_ADDRESS_MAP(pageNumber)		(FLASH_BASE + FLASH_PAGES_OFFSET * pageNumber)


// ===================================================
// ================== APIs Functions =================
// ===================================================
void MCAL_FLASH_MassErase(void);
void MCAL_FLASH_PageErase(uint32_t startPage, uint32_t numberOfPages);

void MCAL_FLASH_Programming(uint32_t address, uint64_t Data, uint8_t programmingType);
void MCAL_FLASH_WriteHalfWord(uint32_t address, uint16_t _half_word);
void MCAL_FLASH_WriteWord(uint32_t address, uint32_t _word);

void MCAL_Flash_UnLock(void);
void MCAL_Flash_Lock(void);

void MCAL_FLASH_READ(uint32_t address, uint32_t *_word);

#endif /* INC_STM32F103C6_FLASH_DRIVER_H_ */
