/*
 * custom_bootloader.h
 *
 *  Created on: Oct 16, 2023
 *      Author: Ahmed
 */

#ifndef INC_CUSTOM_BOOTLOADER_H_
#define INC_CUSTOM_BOOTLOADER_H_


#include <stm32f103c8_RCC_Driver.h>
#include <stm32f103c8_USART_Driver.h>
#include "Platform_Types.h"
#include "stm32f10xxx_device_header.h"



#define DEBUG_MODE

#ifdef DEBUG_MODE

#include <stdarg.h>
#define DEBUG_UART			USART2

void debug_message(uint8_t *format, ... );

#endif

#define HOST_UART						USART1

#define BL_HOST_BUFFER_SIZE				256

#define BL_CRC_VERIFICATION_FAILED		0x0
#define BL_CRC_VERIFICATION_PASSED		0x1

#define BL_ACK							0xCD
#define BL_NACK							0x7F

#define INVALID_SECTOR_NUMBER			0x00
#define VALID_SECTOR_NUMBER				0x01
#define UNSUCCESSFUL_ERASE				0x02
#define SUCCESSFUL_ERASE				0x03

#define FLASH_PAYLOAD_WRITE_FAILED   	0x00
#define FLASH_PAYLOAD_WRITE_PASSED   	0x01


#define BL_VENDOR_ID          	         100
#define BL_SW_MAJOR_VERSION   	       	 1
#define BL_SW_MINOR_VERSION   	         2
#define BL_SW_PATCH_VERSION   	         3


/* Bootloader Commands */
#define BL_COMMANDS_NUMBER				0xC
#define BL_GET_VERSION					0x51
#define BL_GET_HELP						0x52
#define BL_GET_CID						0x53
#define BL_GET_RDP_STATUS				0x54
#define BL_GO_TO_ADDR					0x55
#define BL_FLASH_ERASE					0x56
#define BL_MEM_WRITE					0x57
#define BL_MEM_READ						0x58
#define BL_READ_SECTOR_STATUS			0x59
#define BL_EN_R_W_PROTECT				0x5A
#define BL_DIS_R_W_PROTECT				0x5B
#define BL_OTP_READ						0x5C


void BL_Init(void);
void BL_listenToHost(void);

#endif /* INC_CUSTOM_BOOTLOADER_H_ */
