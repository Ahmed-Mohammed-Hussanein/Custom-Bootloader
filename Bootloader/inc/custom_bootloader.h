/*
 * custom_bootloader.h
 *
 *  Created on: Oct 16, 2023
 *      Author: Ahmed
 */

#ifndef INC_CUSTOM_BOOTLOADER_H_
#define INC_CUSTOM_BOOTLOADER_H_


#include "Platform_Types.h"
#include "stm32f10xxx_device_header.h"

#include "stm32f103c6_RCC_Driver.h"
#include "stm32f103c6_USART_Driver.h"


//#define DEBUG_MODE

#ifdef DEBUG_MODE

#include <stdarg.h>
#define DEBUG_UART			USART2

void debug_message(uint8_t *format, ... );

#endif

#define HOST_UART						USART1



void BL_Init(void);
void BL_listenToHost(void);

#endif /* INC_CUSTOM_BOOTLOADER_H_ */
