/*
 * stm32f103_USART_Driver.c
 *
 *  Created on: Sep 4, 2023
 *      Author: Ahmed
 */


// =============================================
// ================== Includes =================
// =============================================

#include <stm32f103c8_RCC_Driver.h>
#include <stm32f103c8_USART_Driver.h>
#include "Platform_Types.h"

#include "stm32f10xxx_device_header.h"
#include "GPIO_Peripheral_Pins.h"
#include "stm32f103c8_GPIO_Driver.h"

/*
 * =====================================================================================
 * ================================= Private Macros ====================================
 * =====================================================================================
 */

#define NULL (void*)0

/* SR */
#define CTS				9U
#define LBD				8U
#define TXE				7U
#define TC				6U
#define RXNE			5U
#define IDLE			4U
#define ORE				3U
#define NE				2U
#define FE				1U
#define PE				0U


/* CR1 */
#define UE				13U
#define M				12U
#define WAKE			11U
#define PCE				10U
#define PS				9U
#define PEIE			8U
#define TXEIE			7U
#define TCIE			6U
#define RXNEIE			5U
#define IDLEIE			4U
#define TE				3U
#define RE				2U
#define RWU				1U
#define SBK				0U

/* CR2 */
#define LINEN			14U
#define STOP1			13U
#define STOP0			12U
#define CLKEN			11U
#define CPOL			10U
#define CPHA			9U
#define LBCL			8U
#define LBDIE			6U
#define LBDL			5U
#define ADD3			3U
#define ADD2			2U
#define ADD1			1U
#define ADD0			0U

/* CR3 */
#define CTSIE			10U
#define CTSE			9U
#define RTSE			8U
#define DMAT			7U
#define DMAR			6U
#define SCEN			5U
#define NACK			4U
#define HDSEL			3U
#define IRPL			2U
#define IREN			1U
#define EIE				0U



#define USARTDIV(_PCLK_, _BAUD_)				(uint32_t) ((_PCLK_)/(16U * (_BAUD_) ))
#define USARTDIV_MUL100(_PCLK_, _BAUD_)			(uint32_t) ((25U * (_PCLK_) ) / (4U * (_BAUD_)))
#define Mantissa_MUL100(_PCLK_, _BAUD_)			(uint32_t) (USARTDIV(_PCLK_, _BAUD_) * 100U)
#define Mantissa(_PCLK_, _BAUD_)				(uint32_t) (USARTDIV(_PCLK_, _BAUD_ ))
#define DIV_Fraction(_PCLK_, _BAUD_)			(uint32_t) (((USARTDIV_MUL100(_PCLK_, _BAUD_) - Mantissa_MUL100(_PCLK_, _BAUD_) )*16U) / 100U)
#define UART_BRR_Register(_PCLK_, _BAUD_)		(( Mantissa (_PCLK_, _BAUD_ ) ) << 4U )|((DIV_Fraction(_PCLK_, _BAUD_)) & 0xFU )

/*
 * =====================================================================================
 * ================================= APIs Function Definition ==========================
 * =====================================================================================
 */

/**================================================================
 * @Fn				- MCAL_USART_Init
 * @brief			- Initialize UART Asynchronous  only.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- config: This carries the configuration for the USART.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_Init(USART_TypeDef *USARTx, USART_Config_t *config)
{
	uint32_t pclk;
	uint32_t BRR;

	// configure the word length.
	WRITE_BIT(USARTx->CR1, M, config->USART_wordLength);

	// configure the parity bit.
	WRITE_MASK_POS(USARTx->CR1, 0x3U, PS, config->USART_parityBit);

	// configure the stop bit.
	WRITE_MASK_POS(USARTx->CR2, 0x3U, STOP0, config->USART_stopBit);

	// configure the hardware flow control.
	WRITE_MASK_POS(USARTx->CR3, 0x3U, RTSE, config->USART_hardwareFlowControl);

	// configure the interrupts
	if(config->USART_interruptEnable != USART_IRQ_ENABLE_NONE)
	{
		WRITE_MASK(USARTx->CR1, USART_IRQ_ENABLE_ALL, config->USART_interruptEnable);

		if(USARTx == USART1)
		{
			SET_BIT(NVIC->ISER1, 37U-32U);
		}
		else if(USARTx == USART2)
		{
			SET_BIT(NVIC->ISER1, 38U-32U);
		}
		else if(USARTx == USART3)
		{
			SET_BIT(NVIC->ISER1, 39U-32U);
		}
		else
		{

		}
	}

	// configure the baud rate
	// USART1 --> APB2
	// USART2,3 --> APB1
	pclk 			= (USARTx == USART1) ? MCAL_RCC_getAPB2CLK() : MCAL_RCC_getAPB1CLK() ;
	BRR 			= UART_BRR_Register(pclk, config->USART_baudRate);
	USARTx->BRR 	= BRR;

	// configure USART MODE
	WRITE_MASK_POS(USARTx->CR1, 0x3U, RE, config->USART_Mode);
}

/**================================================================
 * @Fn				- MCAL_USART_DeInit
 * @brief			- DeInitialize USART.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_DeInit(USART_TypeDef *USARTx)
{
	if(USARTx == USART1)
	{
		MCAL_RCC_APB2_resetPeripheral(RCC_APB2_PERIPHERAL_USART1);
	}
	else if(USARTx == USART2)
	{
		MCAL_RCC_APB1_resetPeripheral(RCC_APB1_PERIPHERAL_USART2);
	}
	else if(USARTx == USART3)
	{
		MCAL_RCC_APB1_resetPeripheral(RCC_APB1_PERIPHERAL_USART3);
	}
	else
	{

	}
}

void MCAL_USART_clockEnable(USART_TypeDef *USARTx)
{
	if(USARTx == USART1)
	{
		MCAL_RCC_APB2_enableClock(RCC_APB2_PERIPHERAL_USART1);
	}
	else if(USARTx == USART2)
	{
		MCAL_RCC_APB1_enableClock(RCC_APB1_PERIPHERAL_USART2);
	}
	else if(USARTx == USART3)
	{
		MCAL_RCC_APB1_enableClock(RCC_APB1_PERIPHERAL_USART3);
	}
	else
	{

	}
}

/**================================================================
 * @Fn				- MCAL_USART_GPIO_setPins
 * @brief			- Configure the GPIO pins for USART.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_GPIO_setPins(USART_TypeDef *USARTx)
{
	GPIO_PinConfig_t pin;
	if(USARTx == USART1)
	{
		// configure TX PA9
		pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
		pin.GPIO_PinNumber		= USART1_TX;
		MCAL_GPIO_Init(USART1_PORT, &pin);

		// configure RX PA10
		pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
		pin.GPIO_PinNumber		= USART1_RX;
		MCAL_GPIO_Init(USART1_PORT, &pin);

		uint8_t HW_flowControl = READ_MASK_POS(USARTx->CR3, 0x3U, RTSE);
		if((HW_flowControl == USART_FLOW_CONTROL_RTS) || (HW_flowControl == USART_FLOW_CONTROL_RTS_CTS))
		{
			// configure RTS PA12
			pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
			pin.GPIO_PinNumber		= USART1_RTS;
			MCAL_GPIO_Init(USART1_PORT, &pin);
		}

		if((HW_flowControl == USART_FLOW_CONTROL_CTS) || (HW_flowControl == USART_FLOW_CONTROL_RTS_CTS))
		{
			// configure CTS PA11
			pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
			pin.GPIO_PinNumber		= USART1_CTS;
			MCAL_GPIO_Init(USART1_PORT, &pin);
		}
	}

	else if(USARTx == USART2)
	{
		// configure TX PA2
		pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
		pin.GPIO_PinNumber		= USART2_TX;
		MCAL_GPIO_Init(USART2_PORT, &pin);

		// configure RX PA3
		pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
		pin.GPIO_PinNumber		= USART2_RX;
		MCAL_GPIO_Init(USART2_PORT, &pin);

		uint8_t HW_flowControl = READ_MASK_POS(USARTx->CR3, 0x3U, RTSE);
		if((HW_flowControl == USART_FLOW_CONTROL_RTS) || (HW_flowControl == USART_FLOW_CONTROL_RTS_CTS))
		{
			// configure RTS PA1
			pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
			pin.GPIO_PinNumber		= USART2_RTS;
			MCAL_GPIO_Init(USART2_PORT, &pin);
		}

		if((HW_flowControl == USART_FLOW_CONTROL_CTS) || (HW_flowControl == USART_FLOW_CONTROL_RTS_CTS))
		{
			// configure CTS PA0
			pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
			pin.GPIO_PinNumber		= USART2_CTS;
			MCAL_GPIO_Init(USART2_PORT, &pin);
		}
	}

	else if(USARTx == USART3)
	{
		// configure TX PB10
		pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
		pin.GPIO_PinNumber		= USART3_TX;
		MCAL_GPIO_Init(USART3_PORT, &pin);

		// configure RX PB11
		pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
		pin.GPIO_PinNumber		= USART3_RX;
		MCAL_GPIO_Init(USART3_PORT, &pin);

		uint8_t HW_flowControl = READ_MASK_POS(USARTx->CR3, 0x3U, RTSE);
		if((HW_flowControl == USART_FLOW_CONTROL_RTS) || (HW_flowControl == USART_FLOW_CONTROL_RTS_CTS))
		{
			// configure RTS PB14
			pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
			pin.GPIO_PinNumber		= USART3_RTS;
			MCAL_GPIO_Init(USART3_PORT, &pin);
		}

		if((HW_flowControl == USART_FLOW_CONTROL_CTS) || (HW_flowControl == USART_FLOW_CONTROL_RTS_CTS))
		{
			// configure CTS PB13
			pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
			pin.GPIO_PinNumber		= USART3_CTS;
			MCAL_GPIO_Init(USART3_PORT, &pin);
		}
	}
	else
	{

	}
}

/**================================================================
 * @Fn				- MCAL_SPI_Start
 * @brief			- start USART module to work.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_Start(USART_TypeDef *USARTx)
{
	// enable USART
	SET_BIT(USARTx->CR1, UE);
}

/**================================================================
 * @Fn				- MCAL_USART_Stop
 * @brief			- stop USART module from work.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_Stop(USART_TypeDef *USARTx)
{
	// disable USART
	CLEAR_BIT(USARTx->CR1, UE);
}

/**================================================================
 * @Fn				- MCAL_USART_sendData_IT
 * @brief			- send one byte using interrupt technique.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pTxBuffer: the byte to be sent.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_sendData_IT(USART_TypeDef *USARTx, uint8_t *pTxBuffer)
{
	/* check if 9 bit is enabled */
	if(READ_BIT(USARTx->CR1, M) == 1)
	{
		USARTx->DR			= *(uint16_t*)pTxBuffer & 0x1FF;
	}
	else
	{
		USARTx->DR			= *(uint8_t*)pTxBuffer & 0xFF;
	}
}

/**================================================================
 * @Fn				- MCAL_USART_sendBuffer
 * @brief			- send bytes of certain size using polling technique.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pTxBuffer: a pointer to buffer to be sent.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_sendBuffer(USART_TypeDef *USARTx, uint8_t *pTxBuffer, uint32_t size)
{
	/* check if 9 bit is enabled */
	uint8_t  *p8BitsData = NULL;
	uint16_t *p9BitsData = NULL;
	uint32_t index;


	if(READ_BIT(USARTx->CR1, M) == 1)
	{
		p9BitsData = (uint16_t*)pTxBuffer;
	}
	else
	{
		p8BitsData = pTxBuffer;
	}

	for(index = 0; index < size; index++)
	{
		while(!READ_BIT(USARTx->SR, TXE))
		{

		}

		if(p9BitsData == NULL)
		{
			USARTx->DR = (*p8BitsData & 0xFF);
			p8BitsData++;
		}
		else
		{
			USARTx->DR = (*p9BitsData & 0x1FF);
			p9BitsData++;
		}
	}
}

/**================================================================
 * @Fn				- MCAL_USART_receiveData_IT
 * @brief			- receive one byte using interrupt technique.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pRxBuffer: a pointer to the byte to be received.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_receiveData_IT(USART_TypeDef *USARTx, uint8_t *pRxBuffer)
{
	if(READ_BIT(USARTx->CR1, M) == 1)
	{
		*(uint16_t*)pRxBuffer = USARTx->DR & ((~(READ_BIT(USARTx->CR1, PCE) << 8)) & 0x1FF );
	}
	else
	{
		*(uint8_t*)pRxBuffer = USARTx->DR & ((~(READ_BIT(USARTx->CR1, PCE) << 7)) & 0xFF);
	}
}

/**================================================================
 * @Fn				- MCAL_USART_receiveBuffer
 * @brief			- receive bytes with a certain size using Polling technique.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pRxBuffer: a pointer to the buffer to receive in.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_receiveBuffer(USART_TypeDef *USARTx, uint8_t *pRxBuffer, uint32_t size)
{
	uint32_t index;
	uint16_t *p9BitsData = NULL;
	uint8_t *p8BitsData = NULL;

	if(READ_BIT(USARTx->CR1, M) == 1)
	{
		p9BitsData = (uint16_t*)pRxBuffer;
	}
	else
	{
		p8BitsData = pRxBuffer;
	}

	for(index = 0; index < size; index++)
	{
		while(!READ_BIT(USARTx->SR, RXNE))
		{

		}

		if(NULL != p8BitsData)
		{
			*p8BitsData	=	USARTx->DR;
			p8BitsData++;
		}
		else
		{
			*p9BitsData	=	USARTx->DR;
			p9BitsData++;
		}
		//		pRxBuffer[index]			= USARTx->DR;
	}
}

/**================================================================
 * @Fn				- MCAL_UART_WAIT_TransmitComplete
 * @brief			- wait until the transmission done.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_UART_WAIT_TransmitComplete(USART_TypeDef* USARTx)
{
	while(!READ_BIT(USARTx->SR, TC))
	{

	}
}

/**================================================================
 * @Fn				- MCAL_USART_sendString
 * @brief			- send 8 bit string.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pString: a pointer to the string.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_sendString(USART_TypeDef *USARTx, uint8_t *pString)
{
	while(*pString != '\0')
	{
		while(!READ_BIT(USARTx->SR, TXE))
		{

		}

		USARTx->DR = (*pString & 0xFF);
		pString++;
		//		MCAL_USART_sendData_IT(USARTx, *pString++);
	}
}

/*
 * =====================================================================================
 * ================================= ISR ===============================================
 * =====================================================================================
 */

void USART1_IRQHandler(void)
{
	if(READ_BIT(USART1->CR1, TXEIE) == 1)
	{
		USART1_TransmitBufferEmpty_CallBack();
	}

	if(READ_BIT(USART1->CR1, TCIE) == 1)
	{
		USART1_TransmissionComplete_CallBack();
	}

	if(READ_BIT(USART1->CR1, RXNEIE) == 1)
	{
		USART1_ReceiveBufferNotEmpty_CallBack();
	}

	if(READ_BIT(USART1->CR1, PEIE) == 1)
	{
		USART1_ParityError_CallBack();
	}
}

void USART2_IRQHandler(void)
{
	if(READ_BIT(USART2->CR1, TXE) == 1)
	{
		USART2_TransmitBufferEmpty_CallBack();
	}

	if(READ_BIT(USART2->CR1, TC) == 1)
	{
		USART2_TransmissionComplete_CallBack();
	}

	if(READ_BIT(USART2->CR1, RXNE) == 1)
	{
		USART2_ReceiveBufferNotEmpty_CallBack();
	}

	if(READ_BIT(USART2->CR1, PE) == 1)
	{
		USART2_ParityError_CallBack();
	}
}

void USART3_IRQHandler(void)
{
	if(READ_BIT(USART3->CR1, TXE) == 1)
	{
		USART3_TransmitBufferEmpty_CallBack();
	}

	if(READ_BIT(USART3->CR1, TC) == 1)
	{
		USART3_TransmissionComplete_CallBack();
	}

	if(READ_BIT(USART3->CR1, RXNE) == 1)
	{
		USART3_ReceiveBufferNotEmpty_CallBack();
	}

	if(READ_BIT(USART3->CR1, PE) == 1)
	{
		USART3_ParityError_CallBack();
	}
}

/*
 * =====================================================================================
 * ================================= CALL BACK =========================================
 * =====================================================================================
 */

// =============================================
// =================== USART1 ==================
// =============================================
__attribute__((weak)) void USART1_TransmitBufferEmpty_CallBack(void)
{
	while(1)
	{

	}
}

__attribute__((weak)) void USART1_TransmissionComplete_CallBack(void)
{
	while(1)
	{

	}
}

__attribute__((weak)) void USART1_ReceiveBufferNotEmpty_CallBack(void)
{
	while(1)
	{

	}
}

__attribute__((weak)) void USART1_ParityError_CallBack(void)
{
	while(1)
	{

	}
}


// =============================================
// =================== USART2 ==================
// =============================================
__attribute__((weak)) void USART2_TransmitBufferEmpty_CallBack(void)
{
	while(1)
	{
		
	}
}

__attribute__((weak)) void USART2_TransmissionComplete_CallBack(void)
{
	while(1)
	{
		
	}
}

__attribute__((weak)) void USART2_ReceiveBufferNotEmpty_CallBack(void)
{
	while(1)
	{
		
	}
}

__attribute__((weak)) void USART2_ParityError_CallBack(void)
{
	while(1)
	{
		
	}
}


// =============================================
// =================== USART3 ==================
// =============================================
__attribute__((weak)) void USART3_TransmitBufferEmpty_CallBack(void)
{
	while(1)
	{
		
	}
}

__attribute__((weak)) void USART3_TransmissionComplete_CallBack(void)
{
	while(1)
	{
		
	}
}

__attribute__((weak)) void USART3_ReceiveBufferNotEmpty_CallBack(void)
{
	while(1)
	{
		
	}
}

__attribute__((weak)) void USART3_ParityError_CallBack(void)
{
	while(1)
	{
		
	}
}
