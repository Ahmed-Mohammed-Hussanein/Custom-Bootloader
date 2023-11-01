/*
 * stm32f10xxx_device_header.h
 *
 *  Created on: Nov 4, 2022
 *      Author: Ahmed
 */

#ifndef STM32F10XXX_DEVICE_HEADER_H_
#define STM32F10XXX_DEVICE_HEADER_H_

// =============================================
// ================== Includes =================
// =============================================

#include "Platform_Types.h"
#include "Utils.h"

// ==========================================================
// ================== Memory Base Addresses =================
// ==========================================================
#define FLASH_BASE				0x08000000UL
#define SYSTEM_MEMORY_BASE		0x1FFFF000UL
#define SRAM_BASE				0x20000000UL

// ==============================================================
// ================== Peripheral Base Addresses =================
// ==============================================================

#define NVIC_BASE				0xE000E100UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* CRC32 -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define CRC_BASE				0x40023000UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* AFIO -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define	AFIO_BASE				0x40010000UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* EXTI -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define EXTI_BASE				0x40010400UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* GPIO -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define	GPIOA_BASE				0x40010800UL
#define	GPIOB_BASE				0x40010C00UL
#define	GPIOC_BASE				0x40011000UL
#define	GPIOD_BASE				0x40011400UL
#define	GPIOE_BASE				0x40011800UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* bxCAN -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define CAN_CSR_BASE					0x40006400UL
#define CAN_TXMBR0_BASE					(0x40006400UL + 0x180UL)
#define CAN_TXMBR1_BASE					(0x40006400UL + 0x190UL)
#define CAN_TXMBR2_BASE					(0x40006400UL + 0x1A0UL)
#define CAN_RXFIFO0_BASE				(0x40006400UL + 0x1B0UL)
#define CAN_RXFIFO1_BASE				(0x40006400UL + 0x1C0UL)
#define CAN_FLTR_CONFIG_BASE			(0x40006400UL + 0x200UL)
#define CAN_FILTERS_BASE				(0x40006400UL + 0x240UL)

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* ADC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define ADC1_BASE						(0x40012400UL)
#define ADC2_BASE						(0x40012800UL)

// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* TIMER 2 to 5 -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
#define TIM2_BASE						(0x40000000UL)
#define TIM3_BASE						(0x40000400UL)
#define TIM4_BASE						(0x40000800UL)
#define TIM5_BASE						(0x40000C00UL)


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* RCC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define RCC_BASE				0x40021000UL


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* FPEC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define FPEC_BASE				0x40022000UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* USART -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define USART1_BASE				0x40013800UL
#define USART2_BASE				0x40004400UL
#define USART3_BASE				0x40004800UL


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* SPI -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define SPI1_BASE				0x40013000UL
#define SPI2_BASE				0x40003800UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* I2C -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define I2C1_BASE				0x40005400UL
#define I2C2_BASE				0x40005800UL

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* DBG -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define DBG_BASE				0xE0042000UL


// =========================================================
// ================== Peripheral Registers =================
// =========================================================

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* USART -*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		SR;
	vuint32_t 		DR;
	vuint32_t		BRR;
	vuint32_t		CR1;
	vuint32_t		CR2;
	vuint32_t		CR3;
	vuint32_t		GTPR;

} USART_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* SPI -*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		CR1;
	vuint32_t 		CR2;
	vuint32_t		SR;
	vuint32_t		DR;
	vuint32_t		CRCPR;
	vuint32_t		RXCRCR;
	vuint32_t		TXCRCR;
	vuint32_t		I2SCFGR;
	vuint32_t		I2SPR;

} SPI_TypeDef;

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* I2C -*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		CR1;
	vuint32_t 		CR2;
	vuint32_t		OAR1;
	vuint32_t		OAR2;
	vuint32_t		DR;
	vuint32_t		SR1;
	vuint32_t		SR2;
	vuint32_t		CCR;
	vuint32_t		TRISE;

} I2C_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* NVIC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		ISER0;
	vuint32_t 		ISER1;
	vuint32_t		ISER2;
	vuint32_t		reserved1[116];
	vuint32_t		ICER0;
	vuint32_t		ICER1;
	vuint32_t		ICER2;
	vuint32_t		reserved2[116];
	vuint32_t		ISPR0;
	vuint32_t		ISPR1;
	vuint32_t		ISPR2;
	vuint32_t		reserved3[116];
	vuint32_t		ICPR0;
	vuint32_t		ICPR1;
	vuint32_t		ICPR2;
	vuint32_t		reserved4[116];
	vuint32_t		IABR0;

} NVIC_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* EXTI -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		IMR;
	vuint32_t 		EMR;
	vuint32_t		RTSR;
	vuint32_t		FTSR;
	vuint32_t		SWIER;
	vuint32_t		PR;

} EXTI_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* AFIO -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		EVCR;
	vuint32_t 		MAPR;
	vuint32_t		EXTICR1;
	vuint32_t		EXTICR2;
	vuint32_t		EXTICR3;
	vuint32_t		EXTICR4;
	vuint32_t		reserved;
	vuint32_t 		MAPR2;

} AFIO_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* GPIO -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		CRL;
	vuint32_t 		CRH;
	vuint32_t		IDR;
	vuint32_t		ODR;
	vuint32_t		BSRR;
	vuint32_t		BRR;
	vuint32_t 		LCKR;

} GPIO_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* bxCAN -*-*-*-*-* APB1
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		MCR;
	vuint32_t 		MSR;
	vuint32_t		TSR;
	vuint32_t		RF0R;
	vuint32_t		RF1R;
	vuint32_t		IER;
	vuint32_t 		ESR;
	vuint32_t 		BTR;

} CAN_CSR_TypeDef;

typedef struct
{
	vuint32_t		TIxR;
	vuint32_t 		TDTxR;
	vuint32_t		TDLxR;
	vuint32_t		TDHxR;

} CAN_TXMBRx_TypeDef;

typedef struct
{
	vuint32_t		RIxR;
	vuint32_t		RDTxR;
	vuint32_t		RDLxR;
	vuint32_t		RDHxR;

} CAN_RXFIFOx_TypeDef;

typedef struct
{
	vuint32_t		FMR;
	vuint32_t 		FM1R;
	vuint32_t		reserved1;
	vuint32_t		FS1R;
	vuint32_t		reserved2;
	vuint32_t		FFA1R;
	vuint32_t		reserved3;
	vuint32_t		FA1R;

} CAN_FLTR_Config_TypeDef;

typedef struct
{
	vuint32_t		FiRx[14];
	//vuint32_t 	F0R2;

	/*vuint32_t		F1R1;
	vuint32_t		F1R2;

	vuint32_t		F2R1;
	vuint32_t		F2R2;

	vuint32_t		F3R1;
	vuint32_t		F3R2;

	vuint32_t		F4R1;
	vuint32_t 	F4R2;

	vuint32_t		F5R1;
	vuint32_t		F5R2;

	vuint32_t		F6R1;
	vuint32_t		F6R2;

	vuint32_t		F7R1;
	vuint32_t		F7R2;*/

} CAN_Filters_TypeDef;

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* ADC -*-*-*-*-* APB2
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t		SR;
	vuint32_t 	CR1;
	vuint32_t		CR2;
	vuint32_t		SMPR1;
	vuint32_t		SMPR2;
	vuint32_t		JOFR1;
	vuint32_t		JOFR2;
	vuint32_t		JOFR3;
	vuint32_t		JOFR4;
	vuint32_t		HTR;
	vuint32_t		LTR;
	vuint32_t		SQR1;
	vuint32_t		SQR2;
	vuint32_t		SQR3;
	vuint32_t		JSQR;
	vuint32_t		JDR1;
	vuint32_t		JDR2;
	vuint32_t		JDR3;
	vuint32_t		JDR4;
	vuint32_t		DR;

} ADC_TypeDef;

// *-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
// -*-*-*-*-* TIMER 2 to 5 -*-*-*-*-* APB2
// *-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
typedef struct
{
	vuint32_t		CR1;
	vuint32_t 	CR2;
	vuint32_t		SMCR;
	vuint32_t		DIER;
	vuint32_t		SR;
	vuint32_t		EGR;
	vuint32_t		CCMR1;
	vuint32_t		CCMR2;
	vuint32_t		CCER;
	vuint32_t		CNT;
	vuint32_t		PSC;
	vuint32_t		ARR;
	vuint32_t		reserved1;
	vuint32_t		CCR1;
	vuint32_t		CCR2;
	vuint32_t		CCR3;
	vuint32_t		CCR4;
	vuint32_t		reserved2;
	vuint32_t		DCR;
	vuint32_t		DMAR;

} TIMx_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* RCC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t 	CR;
	vuint32_t 	CFGR;
	vuint32_t 	CIR;
	vuint32_t 	APB2RSTR;
	vuint32_t 	APB1RSTR;
	vuint32_t 	AHBENR;
	vuint32_t 	APB2ENR;
	vuint32_t 	APB1ENR;
	vuint32_t 	BDCR;
	vuint32_t 	CSR;

} RCC_TypeDef;

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* CRC32 -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t 	DR;
	vuint32_t 	IDR;
	vuint32_t 	CR;

} CRC_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* FPEC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t 	ACR;
	vuint32_t 	KEYR;
	vuint32_t 	OPTKEYR;
	vuint32_t 	SR;
	vuint32_t 	CR;
	vuint32_t 	AR;
	vuint32_t 	reserved;
	vuint32_t 	OBR;
	vuint32_t 	WRPR;

} FPEC_TypeDef;


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* DBG -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
typedef struct
{
	vuint32_t	MCU_IDCODE;
	vuint32_t	MCU_CR;

} DBG_TypeDef;

// =========================================================
// ================== Peripheral Instances =================
// =========================================================

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* EXTI -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define EXTI		((EXTI_TypeDef*)EXTI_BASE)


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* AFIO -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define AFIO		((AFIO_TypeDef*)AFIO_BASE)


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* GPIO -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define GPIOA		((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB		((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC		((GPIO_TypeDef*)GPIOC_BASE)
#define GPIOD		((GPIO_TypeDef*)GPIOD_BASE)
#define	GPIOE		((GPIO_TypeDef*)GPIOE_BASE)




// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* RCC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define	RCC			((RCC_TypeDef*)RCC_BASE)


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* FPEC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define	FLASH			((FPEC_TypeDef*)FPEC_BASE)


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* bxCAN -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define CAN_CSR					((CAN_CSR_TypeDef*)CAN_CSR_BASE)
#define CAN_TXMBR0				((CAN_TXMBRx_TypeDef*)CAN_TXMBR0_BASE)
#define CAN_TXMBR1				((CAN_TXMBRx_TypeDef*)CAN_TXMBR1_BASE)
#define CAN_TXMBR2				((CAN_TXMBRx_TypeDef*)CAN_TXMBR2_BASE)
#define CAN_RXFIFO0				((CAN_RXFIFOx_TypeDef*)CAN_RXFIFO0_BASE)
#define CAN_RXFIFO1				((CAN_RXFIFOx_TypeDef*)CAN_RXFIFO1_BASE)
#define CAN_FLTR_CONFIG			((CAN_FLTR_Config_TypeDef*)CAN_FLTR_CONFIG_BASE)
#define CAN_FILTERS				((CAN_Filters_TypeDef*)CAN_FILTERS_BASE)

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* ADC -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define ADC1						((ADC_TypeDef*)ADC1_BASE)
#define ADC2						((ADC_TypeDef*)ADC2_BASE)

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* USART -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define USART1					((USART_TypeDef*)USART1_BASE)
#define USART2					((USART_TypeDef*)USART2_BASE)
#define USART3					((USART_TypeDef*)USART3_BASE)

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* SPI -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define SPI1					((SPI_TypeDef*)SPI1_BASE)
#define SPI2					((SPI_TypeDef*)SPI2_BASE)

// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* I2C -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define I2C1					((I2C_TypeDef*)I2C1_BASE)
#define I2C2					((I2C_TypeDef*)I2C2_BASE)


// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* TIMER 2 to 5 -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
#define TIM2						((TIMx_TypeDef*)TIM2_BASE)
#define IM3							((TIMx_TypeDef*)TIM3_BASE)
#define TIM4						((TIMx_TypeDef*)TIM4_BASE)
#define TIM5						((TIMx_TypeDef*)TIM5_BASE)



// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* CRC32 -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define CRC							((CRC_TypeDef*)CRC_BASE)


// *-*-*-*-*-*-*-*-*-*-*-*-*-
// -*-*-*-*-* DBG -*-*-*-*-*
// *-*-*-*-*-*-*-*-*-*-*-*-*-
#define DBG							((DBG_TypeDef*)DBG_BASE)

// =========================================================
// ================== Generic Macros =======================
// =========================================================
#define RCC_GPIOA_CLK_EN()	(RCC->APB2ENR |= 1<<2)
#define RCC_GPIOB_CLK_EN()	(RCC->APB2ENR |= 1<<3)
#define RCC_GPIOC_CLK_EN()	(RCC->APB2ENR |= 1<<4)


#define RCC_GPIOA_CLK_DS()	(RCC->APB2ENR &= ~(1<<2))
#define RCC_GPIOB_CLK_DS()	(RCC->APB2ENR &= ~(1<<3))
#define RCC_GPIOC_CLK_DS()	(RCC->APB2ENR &= ~(1<<4))


#define RCC_GPIOA_RESET()	(RCC->APB2RSTR |= 1<<2);\
							(RCC->APB2RSTR &= ~(1<<2))

#define RCC_GPIOB_RESET()	(RCC->APB2RSTR |= 1<<3);\
							(RCC->APB2RSTR &= ~(1<<3))

#define RCC_GPIOC_RESET()	(RCC->APB2RSTR |= 1<<4);\
							(RCC->APB2RSTR &= ~(1<<4))


#define RCC_AFIO_CLK_EN()	(RCC->APB2ENR |= 1<<0)

#define RCC_AFIO_RESET()	(RCC->APB2RSTR |= 1<<0);\
							(RCC->APB2RSTR &= ~(1<<0))

#define RCC_AFIO_CLK_DS()	(RCC->APB2ENR &= ~(1<<0))

#define RCC_CAN_CLK_EN()	(RCC->APB1ENR |= 1<<25)

#define RCC_USART1_CLK_EN()	(RCC->APB2ENR |= 1<<14)
#define RCC_USART2_CLK_EN()	(RCC->APB1ENR |= 1<<17)
#define RCC_USART3_CLK_EN()	(RCC->APB1ENR |= 1<<18)

#define RCC_USART1_CLK_DS()	(RCC->APB2ENR &= ~(1<<14))
#define RCC_USART2_CLK_DS()	(RCC->APB1ENR &= ~(1<<17))
#define RCC_USART3_CLK_DS()	(RCC->APB1ENR &= ~(1<<18))

#define RCC_USART1_RESET()	(RCC->APB2RSTR |= (1<<14));\
							(RCC->APB2RSTR &= ~(1<<14))
#define RCC_USART2_RESET()	(RCC->APB1RSTR |= (1<<17));\
							(RCC->APB1RSTR &= ~(1<<17))
#define RCC_USART3_RESET()	(RCC->APB1RSTR |= (1<<18));\
							(RCC->APB1RSTR &= ~(1<<18))



#define NVIC				((NVIC_TypeDef*)NVIC_BASE)


#endif /* STM32F10XXX_DEVICE_HEADER_H_ */
