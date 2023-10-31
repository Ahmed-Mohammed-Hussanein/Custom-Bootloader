/*
 * stm32f103c8_RCC_Driver.c
 *
 *  Created on: Mar 2, 2023
 *      Author: Ahmed
 */


// =============================================
// ================== Includes =================
// =============================================


#include <stm32f103c8_RCC_Driver.h>
#include "Platform_Types.h"
#include "Utils.h"

#include "stm32f10xxx_device_header.h"



/*
 * =====================================================================================
 * ================================= Generic Macros ====================================
 * =====================================================================================
 */

#define GET_FLASH_LATENCY(FREQ)		(((FREQ) < 24000000UL) ? (0UL) : (((FREQ) < 48000000UL) ? (1UL) : (2UL)))

/* CR */
#define PLLRDY				25
#define PLLON				24
#define CSS					19
#define HSEBYP				18
#define HSERDY				17
#define HSEON				16
#define HSIRDY				1
#define HSION				0

/* CFGR */
#define PLLMUL				18
#define PLLXTPRE			17
#define PLLSRC				16
#define PPRE2				11
#define PPRE1				8
#define HPRE				4
#define SWS					2
#define SW					0


/*
 * =====================================================================================
 * ================================= Private Variables =================================
 * =====================================================================================
 */

static uint32_t SYSCLK = 0;
static uint32_t AHB_CLK	= 0;
static uint32_t APB1_CLK = 0;
static uint32_t APB2_CLK = 0;


/*
 * =====================================================================================
 * ================================= APIs Function Definition ==========================
 * =====================================================================================
 */

/**================================================================
 * @Fn				- MCAL_RCC_Init
 * @brief			- This is used to configure and initialize the clock for buses.
 * @param [in] 		- clkConfig: This carries the configuration of the clock.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_Init(RCC_Config_t *clkConfig)
{
	// calculate SYSCLK frequency
	if(clkConfig->RCC_SYSCLK == RCC_SYSCLK_HSI)
	{
		SYSCLK = HSI_OSCILLATOR;
	}
	else if(clkConfig->RCC_SYSCLK == RCC_SYSCLK_HSE)
	{
		SYSCLK = HSE_OSCILLATOR;
	}
	else if(clkConfig->RCC_SYSCLK == RCC_SYSCLK_PLLCLK)
	{
		if(clkConfig->RCC_PLLSRC == RCC_PLL_SRC_HSI_DIV_2)
		{
			SYSCLK = (HSI_OSCILLATOR / 2) * (clkConfig->RCC_PLLMUL + 2);
		}
		else if(clkConfig->RCC_PLLSRC == RCC_PLL_SRC_HSE_DIV_2)
		{
			SYSCLK = (HSE_OSCILLATOR / 2) * (clkConfig->RCC_PLLMUL + 2);
		}
		else if(clkConfig->RCC_PLLSRC == RCC_PLL_SRC_HSE)
		{
			SYSCLK = (HSE_OSCILLATOR) * (clkConfig->RCC_PLLMUL + 2);
		}
		else
		{

		}
	}
	else
	{

	}

	if(SYSCLK > 72000000UL)
	{
		return;
	}

	// calculate AHB frequency
	if(clkConfig->RCC_AHBPrescaler < RCC_HPRE_DIV_64)
	{
		AHB_CLK = SYSCLK / (uint32_t)(1 << (clkConfig->RCC_AHBPrescaler - 7));
	}
	else if((clkConfig->RCC_AHBPrescaler >= RCC_HPRE_DIV_64) && (clkConfig->RCC_AHBPrescaler <= RCC_HPRE_DIV_512))
	{
		AHB_CLK = SYSCLK / (uint32_t)(1 << (clkConfig->RCC_AHBPrescaler - 6));
	}
	else
	{

	}

	// calculate APB1 frequency
	APB1_CLK = AHB_CLK / (uint32_t)(1 << (clkConfig->RCC_APB1Prescaler - 3));

	if(APB1_CLK > 36000000UL)
	{
		return;
	}

	// calculate APB2 frequency
	APB2_CLK = AHB_CLK / (uint32_t)(1 << (clkConfig->RCC_APB2Prescaler - 3));





	// enable clock source
	// Enable HSI Oscillator
	if((clkConfig->RCC_PLLSRC == RCC_PLL_SRC_HSI_DIV_2)
			|| (clkConfig->RCC_SYSCLK == RCC_SYSCLK_HSI))
	{
		SET_BIT(RCC->CR, HSION);
		while((READ_BIT(RCC->CR, HSIRDY)) != 1){}

	}
	// Enable HSE Oscillator
	else if ((clkConfig->RCC_PLLSRC == RCC_PLL_SRC_HSE_DIV_2)
			|| (clkConfig->RCC_PLLSRC == RCC_PLL_SRC_HSE)
			|| (clkConfig->RCC_SYSCLK == RCC_SYSCLK_HSE))
	{
		SET_BIT(RCC->CR, HSEON);
		while((READ_BIT(RCC->CR, HSERDY)) != 1){}

		// set security system
		// SET_BIT(RCC->CR, CSS);

		// check if failure happened or not

	}
	else
	{

	}

	// set PLL Configuration.
	if(clkConfig->RCC_SYSCLK == RCC_SYSCLK_PLLCLK)
	{
		// set PLL clock source
		switch(clkConfig->RCC_PLLSRC)
		{
		case RCC_PLL_SRC_HSI_DIV_2:
			CLEAR_BIT(RCC->CFGR, PLLSRC);
			break;

		case RCC_PLL_SRC_HSE_DIV_2:
			SET_BIT(RCC->CFGR, PLLSRC);
			SET_BIT(RCC->CFGR, PLLXTPRE);
			break;

		case RCC_PLL_SRC_HSE:
			SET_BIT(RCC->CFGR, PLLSRC);
			CLEAR_BIT(RCC->CFGR, PLLXTPRE);
			break;

		default:
			break;
		}

		// set PLL multiplier
		RCC->CFGR |= (clkConfig->RCC_PLLMUL << PLLMUL);

		// set PLL source.
		SET_BIT(RCC->CR, PLLON);
		while((READ_BIT(RCC->CR, PLLRDY)) != 1){}
	}

	// set AHB prescaler
	WRITE_MASK_POS(RCC->CFGR, 0xF, HPRE, clkConfig->RCC_AHBPrescaler);

	// set APB1 prescaler
	WRITE_MASK_POS(RCC->CFGR, 0x7, PPRE1, clkConfig->RCC_APB1Prescaler);

	// set APB2 prescaler
	WRITE_MASK_POS(RCC->CFGR, 0x7, PPRE2, clkConfig->RCC_APB2Prescaler);

	// set flash latency
	FLASH->ACR |= GET_FLASH_LATENCY(SYSCLK);

	// set SYSCLK source
	WRITE_MASK_POS(RCC->CFGR, 0x3, SW, clkConfig->RCC_SYSCLK);
	while(READ_MASK_POS(RCC->CFGR, 0x3, SWS) != clkConfig->RCC_SYSCLK){}

	return;
}

/**================================================================
 * @Fn				- MCAL_RCC_APB2_resetPeripheral
 * @brief			- This is used to reset a peripheral on the APB2 Bus.
 * @param [in] 		- APB2PeripheralReset: This must be a value of @ref RCC_APB2_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_APB2_resetPeripheral(uint32_t APB2PeripheralReset)
{
	SET_MASK(RCC->APB2RSTR, APB2PeripheralReset);
	CLEAR_MASK(RCC->APB2RSTR, APB2PeripheralReset);
}

/**================================================================
 * @Fn				- MCAL_RCC_APB2_enableClock
 * @brief			- This is used to enable the clock of a peripheral on the APB2 Bus.
 * @param [in] 		- APB2PeripheralEnable: This must be a value of @ref RCC_APB2_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_APB2_enableClock(uint32_t APB2PeripheralEnable)
{
	SET_MASK(RCC->APB2ENR, APB2PeripheralEnable);
}

/**================================================================
 * @Fn				- MCAL_RCC_APB2_disableClock
 * @brief			- This is used to disable the clock of a peripheral on the APB2 Bus.
 * @param [in] 		- APB2PeripheralDisable: This must be a value of @ref RCC_APB2_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_APB2_disableClock(uint32_t APB2PeripheralDisable)
{
	CLEAR_MASK(RCC->APB2ENR, APB2PeripheralDisable);
}

/**================================================================
 * @Fn				- MCAL_RCC_APB1_resetPeripheral
 * @brief			- This is used to reset a peripheral on the APB1 Bus.
 * @param [in] 		- APB1PeripheralReset: This must be a value of @ref RCC_APB1_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_APB1_resetPeripheral(uint32_t APB1PeripheralReset)
{
	SET_MASK(RCC->APB1RSTR, APB1PeripheralReset);
	CLEAR_MASK(RCC->APB1RSTR, APB1PeripheralReset);
}

/**================================================================
 * @Fn				- MCAL_RCC_APB1_enableClock
 * @brief			- This is used to enable the clock of a peripheral on the APB1 Bus.
 * @param [in] 		- APB1PeripheralEnable: This must be a value of @ref RCC_APB1_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_APB1_enableClock(uint32_t APB1PeripheralEnable)
{
	SET_MASK(RCC->APB1ENR, APB1PeripheralEnable);
}

/**================================================================
 * @Fn				- MCAL_RCC_APB1_disableClock
 * @brief			- This is used to disable the clock of a peripheral on the APB1 Bus.
 * @param [in] 		- APB1PeripheralDisable: This must be a value of @ref RCC_APB1_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_APB1_disableClock(uint32_t APB1PeripheralDisable)
{
	CLEAR_MASK(RCC->APB1ENR, APB1PeripheralDisable);
}

/**================================================================
 * @Fn				- MCAL_RCC_AHB_resetPeripheral
 * @brief			- This is used to reset a peripheral on the AHB Bus.
 * @param [in] 		- AHBPeripheralReset: This must be a value of @ref RCC_AHB_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_AHB_resetPeripheral(uint32_t AHBPeripheralReset)
{
	CLEAR_MASK(RCC->AHBENR, AHBPeripheralReset);
	SET_MASK(RCC->AHBENR, AHBPeripheralReset);
}

/**================================================================
 * @Fn				- MCAL_RCC_AHB_enableClock
 * @brief			- This is used to enable the clock of a peripheral on the AHB Bus.
 * @param [in] 		- AHBPeripheralEnable: This must be a value of @ref RCC_AHB_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_AHB_enableClock(uint32_t AHBPeripheralEnable)
{
	SET_MASK(RCC->AHBENR, AHBPeripheralEnable);
}

/**================================================================
 * @Fn				- MCAL_RCC_AHB_disableClock
 * @brief			- This is used to disable the clock of a peripheral on the AHB Bus.
 * @param [in] 		- AHBPeripheralDisable: This must be a value of @ref RCC_AHB_PERIPHERAL_define.
 * @retval 			- None.
 * Note				- None.
 */
void MCAL_RCC_AHB_disableClock(uint32_t AHBPeripheralDisable)
{
	CLEAR_MASK(RCC->AHBENR, AHBPeripheralDisable);
}

/**================================================================
 * @Fn				- MCAL_RCC_getSYSCLK
 * @brief			- This is used to return the clock value of SYS Bus.
 * @param [in] 		- None.
 * @retval 			- clock value of Sys clock in Hz.
 * Note				- None.
 */
uint32_t MCAL_RCC_getSYSCLK(void)
{
	return SYSCLK;
}

/**================================================================
 * @Fn				- MCAL_RCC_getAHBCLK
 * @brief			- This is used to return the clock value of AHB Bus.
 * @param [in] 		- None.
 * @retval 			- clock value of AHB clock in Hz.
 * Note				- None.
 */
uint32_t MCAL_RCC_getAHBCLK(void)
{
	return AHB_CLK;
}

/**================================================================
 * @Fn				- MCAL_RCC_getAPB1CLK
 * @brief			- This is used to return the clock value of APB1 Bus.
 * @param [in] 		- None.
 * @retval 			- clock value of APB1 clock in Hz.
 * Note				- None.
 */
uint32_t MCAL_RCC_getAPB1CLK(void)
{
	return APB1_CLK;
}

/**================================================================
 * @Fn				- MCAL_RCC_getAPB2CLK
 * @brief			- This is used to return the clock value of APB2 Bus.
 * @param [in] 		- None.
 * @retval 			- clock value of APB2 clock in Hz.
 * Note				- None.
 */
uint32_t MCAL_RCC_getAPB2CLK(void)
{
	return APB2_CLK;
}
