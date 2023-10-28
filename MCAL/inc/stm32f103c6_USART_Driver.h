/*
 * stm32f103_USART_Driver.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ahmed
 */

#ifndef INC_STM32F103C6_USART_DRIVER_H_
#define INC_STM32F103C6_USART_DRIVER_H_

// =============================================
// ================== Includes =================
// =============================================

#include "stm32f10xxx_device_header.h"
#include "Platform_Types.h"



// ==========================================================
// ================== User Type Definitions =================
// ==========================================================

typedef struct
{
	uint32_t USART_baudRate; 			/* This specifies the buad rate of the uart.
					     	 	 	 	 	 	This parameter must be a value of @ref USART_BAUDRATE_define. */

	uint8_t USART_Mode; 				/* This enables or disable Tx, Rx or both.
					     	 	 	 	 	This parameter must be a value of @ref USART_MODE_define. */

	uint8_t USART_wordLength; 			/* This specifies the length of the USART packet to be 8 or 9 bits.
	     	 	 	 	 	 	 	 	   This parameter must be a value of @ref USART_WORD_define. */

	uint8_t USART_parityBit; 			/* This specifies the parity bit to be even, odd, or disabled.
		     	 	 	 	 	 	 	 	This parameter must be a value of @ref USART_PARITY_define. */

	uint8_t USART_stopBit; 				/* This specifies the stop bit to be 0.5, 1, 1.5, or 2 bits.
			     	 	 	 	 	 	 	This parameter must be a value of @ref USART_STOP_define. */

	uint8_t USART_hardwareFlowControl; 	/* This specifies to enable or disable cts, rts, or both.
					     	 	 	 	 	This parameter must be a value of @ref USART_FLOW_CONTROL_define. */

	uint8_t USART_interruptEnable; 		/* This specifies to enable 4 interrupts .
						     	 	 	 	This parameter must be a value of @ref USART_IRQ_ENABLE_define. */

} USART_Config_t;



// ===================================================================
// ================== Macros Configuration Reference =================
// ===================================================================

// @ref USART_MODE_define.
#define USART_MODE_RX					0x1
#define USART_MODE_TX					0x2
#define USART_MODE_TX_RX				0x3

// @ref USART_WORD_define.
#define USART_WORD_8BITS 				0x0
#define USART_WORD_9BITS 				0x1


// @ref USART_PARITY_define.
#define USART_PARITY_DISABLE			0x0
#define USART_PARITY_EVEN				0x2
#define USART_PARITY_ODD				0x3

// @ref USART_STOP_define.
#define USART_STOP_1BIT					0x0
#define USART_STOP_HALF_BIT				0x1
#define USART_STOP_2BIT					0x2
#define USART_STOP_1_HALF_BIT			0x3

// @ref USART_BAUDRATE_define.
#define USART_BAUDRATE_2400				2400
#define USART_BAUDRATE_9600				9600
#define USART_BAUDRATE_19200			19200
#define USART_BAUDRATE_57600			57600
#define USART_BAUDRATE_115200			115200
#define USART_BAUDRATE_230400			230400
#define USART_BAUDRATE_460800			460800
#define USART_BAUDRATE_921600			921600
#define USART_BAUDRATE_2250000			2250000

// @ref USART_FLOW_CONTROL_define.
#define USART_FLOW_CONTROL_NONE			0x0
#define USART_FLOW_CONTROL_RTS			0x1
#define USART_FLOW_CONTROL_CTS			0x2
#define USART_FLOW_CONTROL_RTS_CTS		0x3

// @ref USART_IRQ_ENABLE_define.
#define USART_IRQ_ENABLE_NONE			0x0
#define USART_IRQ_ENABLE_TXE			(1<<7)
#define USART_IRQ_ENABLE_TC				(1<<6)
#define USART_IRQ_ENABLE_RXNE			(1<<5)
#define USART_IRQ_ENABLE_PE				(1<<8)
#define USART_IRQ_ENABLE_ALL			((1<<7) | (1<<6) | (1<<5) | (1<<8))




// ===================================================
// ================== APIs Functions =================
// ===================================================

void MCAL_USART_Init(USART_TypeDef *USARTx, USART_Config_t *config);
void MCAL_USART_DeInit(USART_TypeDef *USARTx);

void MCAL_USART_clockEnable(USART_TypeDef *USARTx);

void MCAL_USART_Start(USART_TypeDef *USARTx);
void MCAL_USART_Stop(USART_TypeDef *USARTx);

void MCAL_USART_GPIO_setPins(USART_TypeDef *USARTx);

void MCAL_USART_sendData_IT(USART_TypeDef *USARTx, uint8_t *txBuffer);
void MCAL_USART_sendBuffer(USART_TypeDef *USARTx, uint8_t *pTxBuffer, uint32_t size);
void MCAL_USART_sendString(USART_TypeDef *USARTx, uint8_t *pString);

void MCAL_USART_receiveData(USART_TypeDef *USARTx, uint8_t *pRxBuffer);
void MCAL_USART_receiveBuffer(USART_TypeDef *USARTx, uint8_t *pRxBuffer, uint32_t size);


void MCAL_UART_WAIT_TransmitComplete(USART_TypeDef* USARTx);

// CallBack Functions
void USART1_TransmitBufferEmpty_CallBack(void);
void USART1_TransmissionComplete_CallBack(void);
void USART1_ReceiveBufferNotEmpty_CallBack(void);
void USART1_ParityError_CallBack(void);

void USART2_TransmitBufferEmpty_CallBack(void);
void USART2_TransmissionComplete_CallBack(void);
void USART2_ReceiveBufferNotEmpty_CallBack(void);
void USART2_ParityError_CallBack(void);

void USART3_TransmitBufferEmpty_CallBack(void);
void USART3_TransmissionComplete_CallBack(void);
void USART3_ReceiveBufferNotEmpty_CallBack(void);
void USART3_ParityError_CallBack(void);


// TODO MCAL_USART_LIN_Init()	// LIN
// TODO MCAL_USART_Init()		// Synchronous
// TODO MCAL_USART_DMA_Init()	// Multi-buffer communication

#endif /* INC_STM32F103C6_USART_DRIVER_H_ */
