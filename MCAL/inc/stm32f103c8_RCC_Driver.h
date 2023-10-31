/*
 * stm32f103c8_RCC_Driver.h
 *
 *  Created on: Mar 2, 2023
 *      Author: Ahmed
 */

#ifndef INC_STM32F103C8_RCC_DRIVER_H_
#define INC_STM32F103C8_RCC_DRIVER_H_

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
	uint8_t RCC_SYSCLK;						/* This is used to select SYSCLK source.
												Max frequency for Sys CLK is 72 MHz.
												This parameter must be a value of @ref RCC_SYSCLK_define. */

	uint8_t RCC_PLLMUL;						/* This specifies PLL multiplication factor.
											  	This parameter must be a value of @ref RCC_PLL_MUL_define. */

	uint8_t RCC_PLLSRC;						/* This is used to select PLL clock source.
												  	This parameter must be a value of @ref RCC_PLL_SRC_define. */

	uint8_t RCC_AHBPrescaler;					/* This is used to control the division factor of the AHB clock.
												Max frequency for AHB is 72 MHz.
												This parameter must be a value of @ref RCC_HPRE_define. */

	uint8_t RCC_APB1Prescaler;				/* This is used to control the division factor of the APB low-speed clock (PCLK1).
												Max frequency for APB1 is 36 MHz.
												This parameter must be a value of @ref RCC_PPRE1_define. */

	uint8_t RCC_APB2Prescaler;				/* This is used to control the division factor of the APB high-speed clock (PCLK2).
												Max frequency for APB2 is 72 MHz.
												This parameter must be a value of @ref RCC_PPRE2_define. */

}	RCC_Config_t;


#define HSI_OSCILLATOR				8000000UL
#define HSE_OSCILLATOR				8000000UL


// ===================================================================
// ================== Macros Configuration Reference =================
// ===================================================================

// @ref RCC_SYSCLK_define.
#define RCC_SYSCLK_HSI							0x0
#define RCC_SYSCLK_HSE							0x1
#define RCC_SYSCLK_PLLCLK						0x2


// @ref RCC_PLL_MUL_define.
#define RCC_PLL_MUL_x2							0x0
#define RCC_PLL_MUL_x3							0x1
#define RCC_PLL_MUL_x4							0x2
#define RCC_PLL_MUL_x5							0x3
#define RCC_PLL_MUL_x6							0x4
#define RCC_PLL_MUL_x7							0x5
#define RCC_PLL_MUL_x8							0x6
#define RCC_PLL_MUL_x9							0x7
#define RCC_PLL_MUL_x10							0x8
#define RCC_PLL_MUL_x11							0x9
#define RCC_PLL_MUL_x12							0xA
#define RCC_PLL_MUL_x13							0xB
#define RCC_PLL_MUL_x14							0xC
#define RCC_PLL_MUL_x15							0xD
#define RCC_PLL_MUL_x16							0xE

// @ref RCC_PLL_SRC_define.
#define RCC_PLL_SRC_HSI_DIV_2					0x0
#define RCC_PLL_SRC_HSE_DIV_2					0x1
#define RCC_PLL_SRC_HSE							0x2


// @ref RCC_HPRE_define.
#define RCC_HPRE_NOTDIV							0x7		/* SYSCLK not divided */
#define RCC_HPRE_DIV_2							0x8		/* SYSCLK divided by 2 */
#define RCC_HPRE_DIV_4							0x9		/* SYSCLK divided by 4 */
#define RCC_HPRE_DIV_8							0xA		/* SYSCLK divided by 8 */
#define RCC_HPRE_DIV_16							0xB		/* SYSCLK divided by 16 */
#define RCC_HPRE_DIV_64							0xC		/* SYSCLK divided by 64 */
#define RCC_HPRE_DIV_128						0xD		/* SYSCLK divided by 128 */
#define RCC_HPRE_DIV_256						0xE		/* SYSCLK divided by 256 */
#define RCC_HPRE_DIV_512						0xF		/* SYSCLK divided by 512 */

// @ref RCC_PPRE1_define.
#define RCC_PPRE1_NOTDIV						0x3		/* HCLK not divided */
#define RCC_PPRE1_DIV_2							0x4		/* HCLK divided by 2 */
#define RCC_PPRE1_DIV_4							0x5		/* HCLK divided by 4 */
#define RCC_PPRE1_DIV_8							0x6		/* HCLK divided by 8 */
#define RCC_PPRE1_DIV_16						0x7		/* HCLK divided by 16 */

// @ref RCC_PPRE2_define.
#define RCC_PPRE2_NOTDIV						0x3		/* HCLK not divided */
#define RCC_PPRE2_DIV_2							0x4		/* HCLK divided by 2 */
#define RCC_PPRE2_DIV_4							0x5		/* HCLK divided by 4 */
#define RCC_PPRE2_DIV_8							0x6		/* HCLK divided by 8 */
#define RCC_PPRE2_DIV_16						0x7		/* HCLK divided by 16 */

// @ref RCC_APB2_PERIPHERAL_define.
#define RCC_APB2_PERIPHERAL_AFIO				(1<<0)
#define RCC_APB2_PERIPHERAL_GPIOA				(1<<2)
#define RCC_APB2_PERIPHERAL_GPIOB				(1<<3)
#define RCC_APB2_PERIPHERAL_GPIOC				(1<<4)
#define RCC_APB2_PERIPHERAL_GPIOD				(1<<5)
#define RCC_APB2_PERIPHERAL_ADC1				(1<<9)
#define RCC_APB2_PERIPHERAL_ADC2				(1<<10)
#define RCC_APB2_PERIPHERAL_TIM1				(1<<11)
#define RCC_APB2_PERIPHERAL_SPI1				(1<<12)
#define RCC_APB2_PERIPHERAL_TIM8				(1<<13)
#define RCC_APB2_PERIPHERAL_USART1				(1<<14)
#define RCC_APB2_PERIPHERAL_TIM9				(1<<19)
#define RCC_APB2_PERIPHERAL_TIM10				(1<<20)
#define RCC_APB2_PERIPHERAL_TIM11				(1<<21)

// @ref RCC_APB1_PERIPHERAL_define.
#define RCC_APB1_PERIPHERAL_TIM2				(1<<0)
#define RCC_APB1_PERIPHERAL_TIM3				(1<<1)
#define RCC_APB1_PERIPHERAL_TIM4				(1<<2)
#define RCC_APB1_PERIPHERAL_SPI2				(1<<14)
#define RCC_APB1_PERIPHERAL_USART2				(1<<17)
#define RCC_APB1_PERIPHERAL_USART3				(1<<18)
#define RCC_APB1_PERIPHERAL_I2C1				(1<<21)
#define RCC_APB1_PERIPHERAL_I2C2				(1<<22)
#define RCC_APB1_PERIPHERAL_CAN					(1<<25)

// @ref RCC_AHB_PERIPHERAL_define.
#define RCC_AHB_PERIPHERAL_CRC					(1<<6)


// ===================================================
// ================== APIs Functions =================
// ===================================================
void MCAL_RCC_Init(RCC_Config_t *clkConfig);

void MCAL_RCC_APB2_resetPeripheral(uint32_t APB2PeripheralReset);
void MCAL_RCC_APB2_enableClock(uint32_t APB2PeripheralEnable);
void MCAL_RCC_APB2_disableClock(uint32_t APB2PeripheralDisable);

void MCAL_RCC_APB1_resetPeripheral(uint32_t APB1PeripheralReset);
void MCAL_RCC_APB1_enableClock(uint32_t APB1PeripheralEnable);
void MCAL_RCC_APB1_disableClock(uint32_t APB1PeripheralDisable);

void MCAL_RCC_AHB_resetPeripheral(uint32_t AHBPeripheralReset);
void MCAL_RCC_AHB_enableClock(uint32_t AHBPeripheralEnable);
void MCAL_RCC_AHB_disableClock(uint32_t AHBPeripheralDisable);

uint32_t MCAL_RCC_getSYSCLK(void);
uint32_t MCAL_RCC_getAHBCLK(void);
uint32_t MCAL_RCC_getAPB1CLK(void);
uint32_t MCAL_RCC_getAPB2CLK(void);

#endif /* INC_STM32F103C8_RCC_DRIVER_H_ */
