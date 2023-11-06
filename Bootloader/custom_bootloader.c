/*
 * custom_bootloader.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Ahmed
 */


#include "Utils.h"


#include <stm32f103c8_CRC32_Driver.h>
#include <stm32f103c8_USART_Driver.h>
#include "Platform_Types.h"
#include "stm32f10xxx_device_header.h"
#include "stm32f103c8_FLASH_Driver.h"

#include "custom_bootloader.h"

#ifdef DEBUG_MODE

#include <stdarg.h>

#define _ATTRIBUTE(attrs) __attribute__ (attrs)
#define __VALIST __gnuc_va_list

int	vsprintf (char *__restrict, const char *__restrict, __VALIST)
_ATTRIBUTE ((__format__ (__printf__, 2, 0)));


void debug_message(uint8_t *format, ... )
{
	uint8_t debug_buffer[512] = {0};
	va_list args;

	va_start( args, format );

	vsprintf( (char*)debug_buffer, (const char*)format, args );

	MCAL_USART_sendString(DEBUG_UART, debug_buffer);

	va_end( args ) ;
}


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

	debug_message((uint8_t*)"UART Debuger is Initialized Successfully.\r\n");
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

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"UART HOST is Initialized Successfully.\r\n");
	debug_message((uint8_t*)"Bootloader is Started Successfully.\r\n\r\n");
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif
}

static uint8_t BL_CRC_Verify(uint8_t pBuffer[], uint32_t length, uint32_t receivedCRC)
{
	uint8_t CRC_state = BL_CRC_VERIFICATION_FAILED;

	uint32_t calculatedCRC = MCAL_CRC_Calculate(pBuffer, length, CRC_DATA_SIZE_BYTE);

	if(calculatedCRC == receivedCRC)
	{
		CRC_state = BL_CRC_VERIFICATION_PASSED;
	}

	return CRC_state;
}

static void BL_sendACK(uint32_t length)
{
	uint8_t BL_ACK_buffer[2] = {BL_ACK, length};

	MCAL_USART_sendBuffer(HOST_UART, BL_ACK_buffer, 2);
}

static void BL_sendNACK(void)
{
	uint8_t BL_NACK_buffer[1] = {BL_NACK};

	MCAL_USART_sendBuffer(HOST_UART, BL_NACK_buffer, 1);
}

static void BL_getVersion(void)
{
	uint8_t BL_getVersion_buffer[4] = {BL_VENDOR_ID, BL_SW_MAJOR_VERSION,
			BL_SW_MINOR_VERSION, BL_SW_PATCH_VERSION};

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"Bootloader Get Version.\n\r");
	debug_message((uint8_t*)"Bootloader Vendor ID: %d.\n\r", BL_VENDOR_ID);
	debug_message((uint8_t*)"Bootloader Version: %d.%d.%d\n\r", BL_VENDOR_ID
			, BL_SW_MINOR_VERSION, BL_SW_PATCH_VERSION);
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif

	/* Send Ack */
	BL_sendACK(4);

	/* Send Reply */
	MCAL_USART_sendBuffer(HOST_UART, BL_getVersion_buffer, 4);
}

static void BL_getHelp(void)
{
	uint8_t BL_getHelp_buffer[BL_COMMANDS_NUMBER] = {BL_GET_VERSION, BL_GET_HELP,
			BL_GET_CID, BL_GET_RDP_STATUS, BL_GO_TO_ADDR, BL_FLASH_ERASE, BL_MEM_WRITE, BL_EN_R_W_PROTECT,
			BL_MEM_READ, BL_READ_SECTOR_STATUS, BL_OTP_READ, BL_DIS_R_W_PROTECT};

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"Bootloader Get Help For Supported Commands.\n\r");
	debug_message((uint8_t*)"Bootloader Get Version						: 0x%X.\n\r", BL_GET_VERSION);
	debug_message((uint8_t*)"Bootloader Get Help						: 0x%X.\n\r", BL_GET_HELP);
	debug_message((uint8_t*)"Bootloader Get Chip ID						: 0x%X.\n\r", BL_GET_CID);
	debug_message((uint8_t*)"Bootloader Flash Read Protection Level		: 0x%X.\n\r", BL_GET_RDP_STATUS);
	debug_message((uint8_t*)"Bootloader Go To Address					: 0x%X.\n\r", BL_GO_TO_ADDR);
	debug_message((uint8_t*)"Bootloader Flash Erase						: 0x%X.\n\r", BL_FLASH_ERASE);
	debug_message((uint8_t*)"Bootloader Flash Write						: 0x%X.\n\r", BL_MEM_WRITE);
	debug_message((uint8_t*)"Bootloader Enable Read Write protection	: 0x%X.\n\r", BL_EN_R_W_PROTECT);
	debug_message((uint8_t*)"Bootloader Flash Read						: 0x%X.\n\r", BL_MEM_READ);
	debug_message((uint8_t*)"Bootloader Read Sector Status				: 0x%X.\n\r", BL_READ_SECTOR_STATUS);
	debug_message((uint8_t*)"Bootloader Read OTP Contents				: 0x%X.\n\r", BL_OTP_READ);
	debug_message((uint8_t*)"Bootloader Disable Read Write protection	: 0x%X.\n\r", BL_DIS_R_W_PROTECT);
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif

	/* Send Ack */
	BL_sendACK(BL_COMMANDS_NUMBER);

	/* Send Reply */
	MCAL_USART_sendBuffer(HOST_UART, BL_getHelp_buffer, BL_COMMANDS_NUMBER);
}

static void BL_getCID(void)
{
	uint8_t BL_getCID_buffer[2] = {0};
	BL_getCID_buffer[0]	=	DBG->MCU_IDCODE & 0xFF;
	BL_getCID_buffer[1]	=	(DBG->MCU_IDCODE >> 8) & 0xF;

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"Bootloader Get Chip ID.\n\r");
	debug_message((uint8_t*)"Bootloader Chip ID: 0x%X.\n\r", *(uint16_t*)&BL_getCID_buffer[0]);
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif

	/* Send Ack */
	BL_sendACK(2);

	/* Send Reply */
	MCAL_USART_sendBuffer(HOST_UART, BL_getCID_buffer, 2);
}


static void BL_flashErase(uint8_t pBuffer[])
{
	uint32_t startPage;
	uint32_t numberOfPages;
	uint8_t BL_flashErase_buffer[1] = {0};
	BL_flashErase_buffer[0]	=	SUCCESSFUL_ERASE;

	startPage		= pBuffer[0];
	numberOfPages 	= (pBuffer[1] > (128 - startPage)) ? (128 - startPage) : pBuffer[1];

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"Bootloader Flash Erase.\n\r");
	debug_message((uint8_t*)"The Page Number		: %d.\n\r", startPage);
	debug_message((uint8_t*)"The Number of pages	: %d.\n\r", numberOfPages);
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif

	MCAL_FLASH_PageErase(startPage, numberOfPages);

	/* Send Ack */
	BL_sendACK(1);

	/* Send Reply */
	MCAL_USART_sendBuffer(HOST_UART, BL_flashErase_buffer, 1);
}

static void BL_flashWrite(uint8_t pBuffer[])
{
	uint32_t index;
	uint16_t *pPayload		= (uint16_t*)&pBuffer[5];
	uint16_t *baseAddress 	= (uint16_t*)(*(uint32_t*)&pBuffer[0]);
	uint32_t payloadLength	= pBuffer[4];

	uint8_t BL_flashErase_buffer[1] = {0};
	BL_flashErase_buffer[0]	=	FLASH_PAYLOAD_WRITE_PASSED;

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"Bootloader Flash Write.\n\r");
	debug_message((uint8_t*)"The Base Address		: 0x%x.\n\r", baseAddress);
	debug_message((uint8_t*)"The Payload Length		: %u.\n\r", payloadLength);
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif

	payloadLength /= 2;

	MCAL_Flash_UnLock();
	for(index = 0; index < payloadLength; index++)
	{
		MCAL_FLASH_Programming(baseAddress, *pPayload, FLASH_PROGRAMMING_TYPE_HALF_WORD);
		baseAddress++;
		pPayload++;
	}
	MCAL_Flash_Lock();

#ifdef DEBUG_MODE

	debug_message((uint8_t*)"Data is Written Successfully.\n\r");
	debug_message((uint8_t*)"\r\n==================================================================\n\r");

#endif

	/* Send Ack */
	BL_sendACK(1);

	/* Send Reply */
	MCAL_USART_sendBuffer(HOST_UART, BL_flashErase_buffer, 1);
}

void BL_listenToHost(void)
{
	uint8_t BL_HOST_Buffer[BL_HOST_BUFFER_SIZE];
	uint8_t length_to_follow;
	uint8_t checkedCRC;

	/* Receive the length of the command */
	MCAL_USART_receiveBuffer(HOST_UART, BL_HOST_Buffer, 1);
	length_to_follow = BL_HOST_Buffer[0];

	/* Receive the remaining of the command */
	MCAL_USART_receiveBuffer(HOST_UART, &BL_HOST_Buffer[1], length_to_follow);

	/* CRC Verification */
	checkedCRC = BL_CRC_Verify(BL_HOST_Buffer, length_to_follow - 4 + 1, *(uint32_t*)&BL_HOST_Buffer[length_to_follow - 4 + 1]);

	if(checkedCRC == BL_CRC_VERIFICATION_FAILED)
	{

#ifdef DEBUG_MODE

		debug_message((uint8_t*)"CRC Verification Failed. 0x%X 0x%X\n\r\n\r", checkedCRC, *(uint32_t*)&BL_HOST_Buffer[length_to_follow - 4 + 1]);

#endif

		/* Send Not Ack */
		BL_sendNACK();
		return;
	}


#ifdef DEBUG_MODE

	debug_message((uint8_t*)"CRC Verification Pass.\n\r\n\r");

#endif

	switch(BL_HOST_Buffer[1])
	{
	case BL_GET_VERSION:
		BL_getVersion();
		break;

	case BL_GET_HELP:
		BL_getHelp();
		break;

	case BL_GET_CID:
		BL_getCID();
		break;

	case BL_GET_RDP_STATUS:
		break;

	case BL_GO_TO_ADDR:
		break;

	case BL_FLASH_ERASE:
		BL_flashErase(&BL_HOST_Buffer[2]);
		break;

	case BL_MEM_WRITE:
		BL_flashWrite(&BL_HOST_Buffer[2]);
		break;

	case BL_EN_R_W_PROTECT:
		break;

	case BL_MEM_READ:
		break;

	case BL_READ_SECTOR_STATUS:
		break;

	case BL_OTP_READ:
		break;

	case BL_DIS_R_W_PROTECT:
		break;
	}

}
