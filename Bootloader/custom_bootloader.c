/*
 * custom_bootloader.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Ahmed
 */





#include "Platform_Types.h"
#include "stm32f10xxx_device_header.h"

#include "stm32f103c6_USART_Driver.h"
#include "stm32f103c6_CRC32_Driver.h"

#include "custom_bootloader.h"

#ifdef DEBUG_MODE

#include <stdarg.h>

#define _ATTRIBUTE(attrs) __attribute__ (attrs)
#define __VALIST __gnuc_va_list

int	vsprintf (char *__restrict, const char *__restrict, __VALIST)
_ATTRIBUTE ((__format__ (__printf__, 2, 0)));

void debug_Init(void)
{
	/* enable clock for debug UART */
	MCAL_RCC_APB2_enableClock(RCC_APB2_PERIPHERAL_AFIO);
	MCAL_RCC_APB2_enableClock(RCC_APB2_PERIPHERAL_GPIOA);
	MCAL_USART_clockEnable(DEBUG_UART);

	/* configure UART for debug */
	USART_Config_t debug_uart = {USART_BAUDRATE_115200, USART_MODE_TX, USART_WORD_8BITS,
			USART_PARITY_DISABLE, USART_STOP_1BIT, USART_FLOW_CONTROL_NONE, USART_IRQ_ENABLE_NONE};

	MCAL_USART_Init(DEBUG_UART, &debug_uart);

	MCAL_USART_GPIO_setPins(DEBUG_UART);

	MCAL_USART_Start(DEBUG_UART);
}

void debug_message(uint8_t *format, ... )
{
	uint8_t debug_buffer[512] = {0};
	va_list args;

	va_start( args, format );

	vsprintf( (char*)debug_buffer, (const char*)format, args );

	MCAL_USART_sendString(DEBUG_UART, debug_buffer);

	va_end( args ) ;
}

#endif


void BL_Init(void)
{
	/* Clock Init */
	RCC_Config_t clock;
	clock.RCC_SYSCLK		=	RCC_SYSCLK_HSI;
	clock.RCC_PLLSRC		=	RCC_PLL_SRC_HSE;
	clock.RCC_PLLMUL		=	RCC_PLL_MUL_x4;
	clock.RCC_AHBPrescaler	=	RCC_HPRE_NOTDIV;
	clock.RCC_APB1Prescaler	=	RCC_PPRE1_NOTDIV;
	clock.RCC_APB2Prescaler =	RCC_PPRE2_NOTDIV;

	MCAL_RCC_Init(&clock);

#ifdef DEBUG_MODE

	debug_Init();

#endif

	/* enable clock for host UART */
	MCAL_RCC_APB2_enableClock(RCC_APB2_PERIPHERAL_AFIO);
	MCAL_RCC_APB2_enableClock(RCC_APB2_PERIPHERAL_GPIOA);
	MCAL_USART_clockEnable(HOST_UART);

	/* configure UART for host */
	USART_Config_t host_uart = {USART_BAUDRATE_115200, USART_MODE_TX_RX, USART_WORD_8BITS,
			USART_PARITY_DISABLE, USART_STOP_1BIT, USART_FLOW_CONTROL_NONE, USART_IRQ_ENABLE_NONE};

	MCAL_USART_Init(HOST_UART, &host_uart);

	MCAL_USART_GPIO_setPins(HOST_UART);

	MCAL_USART_Start(HOST_UART);

	MCAL_CRC_clockEnable();
}

void BL_listenToHost(void)
{
	

}
