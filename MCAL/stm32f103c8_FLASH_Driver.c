/*
 * stm32f103c6_FLASH_Driver.c
 *
 *  Created on: Jun 23, 2023
 *      Author: Ahmed
 */


// =============================================
// ================== Includes =================
// =============================================

#include <stm32f103c8_FLASH_Driver.h>
#include <stm32f103c8_RCC_Driver.h>
#include "Platform_Types.h"


/*
 * =====================================================================================
 * ================================= Generic Macros ====================================
 * =====================================================================================
 */

/* ACR */
#define PRFTBS				5
#define PRFTBE				4
#define HLFCYA				3
#define LATENCY				0


/* SR */
#define EOP					5
#define WRPRTERR			4
#define PGERR				2
#define BSY					0


/* CR */
#define EOPIE				12
#define ERRIE				10
#define OPTWRE				9
#define LOCK				7
#define STRT				6
#define OPTER				5
#define OPTPG				4
#define MER					2
#define PER					1
#define PG					0

/* OBR */
#define Data1				18
#define Data0				10
#define nRST_STDBY			4
#define nRST_STOP			3
#define WDG_SW				2
#define RDPRT				1
#define OPTERR				0

/*
 * =====================================================================================
 * ================================= Private Function Definition ==========================
 * =====================================================================================
 */

#define KEY1				0x45670123UL
#define KEY2				0xCDEF89ABUL

void MCAL_Flash_UnLock(void)
{
	if(READ_BIT(FLASH->CR, LOCK) != 0)
	{
		FLASH->KEYR = KEY1;
		FLASH->KEYR = KEY2;
	}
}

void MCAL_Flash_Lock(void)
{
	if(READ_BIT(FLASH->CR, LOCK) != 1)
	{
		SET_BIT(FLASH->CR, LOCK);
	}
}

static void flash_program_halfWord(uint16_t *address, uint16_t halfWord)
{
	// Set PG to select programming the flash
	SET_BIT(FLASH->CR, PG);

	// write the address
	*address = halfWord;

	// wait until the operation performed
	while(READ_BIT(FLASH->SR, BSY) == 1)
	{

	}

	// Clear PG
	CLEAR_BIT(FLASH->CR, PG);
}

/*
 * =====================================================================================
 * ================================= APIs Function Definition ==========================
 * =====================================================================================
 */

/**================================================================
 * @Fn				- MCAL_FLASH_MassErase
 * @brief			- This is used to Erase all user flash.
 * @param [in] 		- None.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_FLASH_MassErase(void)
{
	// Unlock Flash
	MCAL_Flash_UnLock();

	// check if there is no operation --> check BSY flag.
	while(READ_BIT(FLASH->SR, BSY) == 1)
	{

	}

	// Set the MER bit in the FLASH_CR register.
	SET_BIT(FLASH->CR, MER);

	// Set the STRT bit in the FLASH_CR register.
	SET_BIT(FLASH->CR, STRT);

	// Wait for the BSY bit to be reset.
	while(READ_BIT(FLASH->SR, BSY) == 1)
	{

	}

	// clear the MER bit in the FLASH_CR register.
	CLEAR_BIT(FLASH->CR, MER);

	// Clear EOP
	SET_BIT(FLASH->SR, EOP);

	// lock the flash
	MCAL_Flash_Lock();
}

/**================================================================
 * @Fn				- MCAL_FLASH_PageErase
 * @brief			- This is used to Erase a page of the user flash.
 * @param [in] 		- address: address of page to Erase.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_FLASH_PageErase(uint32_t startPage, uint32_t numberOfPages)
{
	uint32_t index;
	uint32_t stopCondition;

	// Unlock Flash
	MCAL_Flash_UnLock();

	// check if there is no operation --> check BSY flag.
	while(READ_BIT(FLASH->SR, BSY) == 1)
	{

	}

	// Set the PER bit in the FLASH_CR register.
	SET_BIT(FLASH->CR, PER);

	stopCondition = numberOfPages + startPage;
	for(index = startPage; index < stopCondition; index++)
	{
		// Program the FLASH_AR register to select a page to erase.
		FLASH->AR	=	FLASH_PAGE_ADDRESS_MAP(index);

		// Set the STRT bit in the FLASH_CR register.
		SET_BIT(FLASH->CR, STRT);

		// Wait for the BSY bit to be reset.
		while(READ_BIT(FLASH->SR, BSY) == 1)
		{

		}
	}

	// Clear the PER bit in the FLASH_CR register.
	CLEAR_BIT(FLASH->CR, PER);

	// lock the flash
	MCAL_Flash_Lock();
}

void MCAL_FLASH_PageErase2(uint32_t startPage)
{
	// Unlock Flash
	MCAL_Flash_UnLock();

	// check if there is no operation --> check BSY flag.
	while(READ_BIT(FLASH->SR, BSY) == 1)
	{

	}

	// Set the PER bit in the FLASH_CR register.
	SET_BIT(FLASH->CR, PER);

	// Program the FLASH_AR register to select a page to erase.
	FLASH->AR = page_adress_map(startPage);

	// Set the STRT bit in the FLASH_CR register.
	SET_BIT(FLASH->CR, STRT);

	// Wait for the BSY bit to be reset.
	while((READ_BIT(FLASH->SR, BSY) == 1))
	{

	}

	// Clear the PER bit in the FLASH_CR register.
	CLEAR_BIT(FLASH->CR, PER);

	// lock the flash
	MCAL_Flash_Lock();
}

/**================================================================
 * @Fn				- MCAL_FLASH_Programming
 * @brief			- This is used to Write half word, word, or double word into the flash.
 * @param [in] 		- address: address in the flash to write the data.
 * 					- Data	 : the data wanted to be written.
 * @retval 			- None.
 * Note				- You must first use HAL_FLASH_UnLock API.
 */
void MCAL_FLASH_Programming(uint16_t *address, uint64_t Data, uint8_t programmingType)
{
	uint8_t nIterations = (programmingType == FLASH_PROGRAMMING_TYPE_HALF_WORD) ? 1 :
			(programmingType == FLASH_PROGRAMMING_TYPE_WORD) ? 2 : 4;

	uint16_t *pAddress = address;

	uint8_t i;
	for(i = 0; i < nIterations; i++)
	{
		flash_program_halfWord(pAddress++, (Data >> (i * 16U)));
	}

	// Clear EOP
	//	SET_BIT(FLASH->SR, EOP);
}

/**================================================================
 * @Fn				- MCAL_FLASH_Read
 * @brief			- This is used to read a word from the flash.
 * @param [in] 		- address: address in the flash to read the word.
 * 					- _word	 : a pointer to variable to store the read word.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_FLASH_Read(uint32_t address, uint32_t *_word)
{
	*_word = *(vuint32_t*)address;
}
