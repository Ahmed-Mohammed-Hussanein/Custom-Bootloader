/*
 * stm32f103_USART_Driver.c
 *
 *  Created on: Sep 4, 2023
 *      Author: Ahmed
 */


// =============================================
// ================== Includes =================
// =============================================

#include "Platform_Types.h"

#include "stm32f10xxx_device_header.h"
#include "GPIO_Peripheral_Pins.h"
#include "stm32f103c6_GPIO_Driver.h"
#include "stm32f103c6_RCC_Driver.h"
#include "stm32f103c6_USART_Driver.h"

/*
 * =====================================================================================
 * ================================= Private Macros ====================================
 * =====================================================================================
 */

#define NULL (void*)0

/* SR */
#define CTS				9
#define LBD				8
#define TXE				7
#define TC				6
#define RXNE			5
#define IDLE			4
#define ORE				3
#define NE				2
#define FE				1
#define PE				0


/* CR1 */
#define UE				13
#define M				12
#define WAKE			11
#define PCE				10
#define PS				9
#define PEIE			8
#define TXEIE			7
#define TCIE			6
#define RXNEIE			5
#define IDLEIE			4
#define TE				3
#define RE				2
#define RWU				1
#define SBK				0

/* CR2 */
#define LINEN			14
#define STOP1			13
#define STOP0			12
#define CLKEN			11
#define CPOL			10
#define CPHA			9
#define LBCL			8
#define LBDIE			6
#define LBDL			5
#define ADD3			3
#define ADD2			2
#define ADD1			1
#define ADD0			0

/* CR3 */
#define CTSIE			10
#define CTSE			9
#define RTSE			8
#define DMAT			7
#define DMAR			6
#define SCEN			5
#define NACK			4
#define HDSEL			3
#define IRPL			2
#define IREN			1
#define EIE				0



#define USARTDIV(_PCLK_, _BAUD_)				(uint32_t) (_PCLK_/(16 * _BAUD_ ))
#define USARTDIV_MUL100(_PCLK_, _BAUD_)			(uint32_t) ((25 * _PCLK_ ) / (4 * _BAUD_))
#define Mantissa_MUL100(_PCLK_, _BAUD_)			(uint32_t) (USARTDIV(_PCLK_, _BAUD_) * 100)
#define Mantissa(_PCLK_, _BAUD_)				(uint32_t) (USARTDIV(_PCLK_, _BAUD_ ))
#define DIV_Fraction(_PCLK_, _BAUD_)			(uint32_t) (((USARTDIV_MUL100(_PCLK_, _BAUD_) - Mantissa_MUL100(_PCLK_, _BAUD_) )*16) / 100)
#define UART_BRR_Register(_PCLK_, _BAUD_)		(( Mantissa (_PCLK_, _BAUD_ ) ) << 4 )|((DIV_Fraction(_PCLK_, _BAUD_)) & 0xF )

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
	uint32_t pclk, BRR;

	// configure the word length.
	WRITE_BIT(USARTx->CR1, M, config->USART_wordLength);

	// configure the parity bit.
	WRITE_MASK_POS(USARTx->CR1, 0x3, PS, config->USART_parityBit);

	// configure the stop bit.
	WRITE_MASK_POS(USARTx->CR2, 0x3, STOP0, config->USART_stopBit);

	// configure the hardware flow control.
	WRITE_MASK_POS(USARTx->CR3, 0x3, RTSE, config->USART_hardwareFlowControl);

	// configure the interrupts
	if(config->USART_interruptEnable != USART_IRQ_ENABLE_NONE)
	{
		WRITE_MASK(USARTx->CR1, USART_IRQ_ENABLE_ALL, config->USART_interruptEnable);

		if(USARTx == USART1)
		{
			SET_BIT(NVIC->ISER1, 37-32);
		}
		else if(USARTx == USART2)
		{
			SET_BIT(NVIC->ISER1, 38-32);
		}
		else if(USARTx == USART3)
		{
			SET_BIT(NVIC->ISER1, 39-32);
		}
	}

	// configure the baud rate
	// USART1 --> APB2
	// USART2,3 --> APB1
	pclk 			= (USARTx == USART1) ? MCAL_RCC_getAPB2CLK() : MCAL_RCC_getAPB1CLK() ;
	BRR 			= UART_BRR_Register(pclk, config->USART_baudRate);
	USARTx->BRR 	= BRR;

	// configure USART MODE
	WRITE_MASK_POS(USARTx->CR1, 0x3, RE, config->USART_Mode);
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
		pin.GPIO_PinNumber		= GPIO_PIN_9;
		MCAL_GPIO_Init(GPIOA, &pin);

		// configure RX PA10
		pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
		pin.GPIO_PinNumber		= GPIO_PIN_10;
		MCAL_GPIO_Init(GPIOA, &pin);

		uint8_t HW_flowControl = READ_MASK_POS(USARTx->CR3, 0x3, RTSE);
		if(HW_flowControl == USART_FLOW_CONTROL_RTS || HW_flowControl == USART_FLOW_CONTROL_RTS_CTS)
		{
			// configure RTS PA12
			pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
			pin.GPIO_PinNumber		= GPIO_PIN_12;
			MCAL_GPIO_Init(GPIOA, &pin);
		}

		if(HW_flowControl == USART_FLOW_CONTROL_CTS || HW_flowControl == USART_FLOW_CONTROL_RTS_CTS)
		{
			// configure CTS PA11
			pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
			pin.GPIO_PinNumber		= GPIO_PIN_11;
			MCAL_GPIO_Init(GPIOA, &pin);
		}
	}

	else if(USARTx == USART2)
	{
		// configure TX PA2
		pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
		pin.GPIO_PinNumber		= GPIO_PIN_2;
		MCAL_GPIO_Init(GPIOA, &pin);

		// configure RX PA3
		pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
		pin.GPIO_PinNumber		= GPIO_PIN_3;
		MCAL_GPIO_Init(GPIOA, &pin);

		uint8_t HW_flowControl = READ_MASK_POS(USARTx->CR3, 0x3, RTSE);
		if(HW_flowControl == USART_FLOW_CONTROL_RTS || HW_flowControl == USART_FLOW_CONTROL_RTS_CTS)
		{
			// configure RTS PA1
			pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
			pin.GPIO_PinNumber		= GPIO_PIN_1;
			MCAL_GPIO_Init(GPIOA, &pin);
		}

		if(HW_flowControl == USART_FLOW_CONTROL_CTS || HW_flowControl == USART_FLOW_CONTROL_RTS_CTS)
		{
			// configure CTS PA0
			pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
			pin.GPIO_PinNumber		= GPIO_PIN_0;
			MCAL_GPIO_Init(GPIOA, &pin);
		}
	}

	else if(USARTx == USART3)
	{
		// configure TX PB10
		pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
		pin.GPIO_PinNumber		= GPIO_PIN_10;
		MCAL_GPIO_Init(GPIOB, &pin);

		// configure RX PB11
		pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
		pin.GPIO_PinNumber		= GPIO_PIN_11;
		MCAL_GPIO_Init(GPIOB, &pin);

		uint8_t HW_flowControl = READ_MASK_POS(USARTx->CR3, 0x3, RTSE);
		if(HW_flowControl == USART_FLOW_CONTROL_RTS || HW_flowControl == USART_FLOW_CONTROL_RTS_CTS)
		{
			// configure RTS PB14
			pin.GPIO_PinMode		= GPIO_MODE_OUTPUT_AF_PP_2MHz;
			pin.GPIO_PinNumber		= GPIO_PIN_14;
			MCAL_GPIO_Init(GPIOB, &pin);
		}

		if(HW_flowControl == USART_FLOW_CONTROL_CTS || HW_flowControl == USART_FLOW_CONTROL_RTS_CTS)
		{
			// configure CTS PB13
			pin.GPIO_PinMode		= GPIO_MODE_INPUT_FLOATING;
			pin.GPIO_PinNumber		= GPIO_PIN_13;
			MCAL_GPIO_Init(GPIOB, &pin);
		}
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
 * @Fn				- MCAL_USART_sendData
 * @brief			- send one byte.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pTxBuffer: the byte to be sent.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_sendData_IT(USART_TypeDef *USARTx, uint8_t *pTxBuffer)
{
	/* check if 9 bit is enabled */
	if(READ_BIT(USARTx->CR1, M))
	{
		USARTx->DR			= *(uint16_t*)pTxBuffer & 0x1FF;
	}
	else
	{
		USARTx->DR			= *(uint8_t*)pTxBuffer & 0xFF;
	}
}

void MCAL_USART_sendBuffer(USART_TypeDef *USARTx, uint8_t *pTxBuffer, uint32_t size)
{
	/* check if 9 bit is enabled */
	uint8_t  *p8BitsData = NULL;
	uint16_t *p9BitsData = NULL;
	uint32_t index;


	if(READ_BIT(USARTx->CR1, M))
	{
		p9BitsData = (uint16_t*)pTxBuffer;
	}
	else
	{
		p8BitsData = pTxBuffer;
	}

	for(index = 0; index < size; index++)
	{
		while(!READ_BIT(USARTx->SR, TXE));

		if(p9BitsData == NULL)
		{
			USARTx->DR = (*p8BitsData++ & 0xFF);
		}
		else
		{
			USARTx->DR = (*p9BitsData++ & 0x1FF);
		}
	}
}

/**================================================================
 * @Fn				- MCAL_USART_receiveData
 * @brief			- receive one byte.
 * @param [in] 		- USARTx: This specifies which USART is used for configuration where x = 1, 2, or 3.
 * @param [in] 		- pRxBuffer: a pointer to the byte to be received.
 * @retval 			- None.
 * Note				- supports only asynchronous mode.
 */
void MCAL_USART_receiveData_IT(USART_TypeDef *USARTx, uint8_t *pRxBuffer)
{
	if(READ_BIT(USARTx->CR1, M))
	{
		*(uint16_t*)pRxBuffer = USARTx->DR & ((~(READ_BIT(USARTx->CR1, PCE) << 8)) & 0x1FF );
	}
	else
	{
		*(uint8_t*)pRxBuffer = USARTx->DR & ((~(READ_BIT(USARTx->CR1, PCE) << 7)) & 0xFF);
	}
}


void MCAL_USART_receiveBuffer(USART_TypeDef *USARTx, uint8_t *pRxBuffer, uint32_t size)
{
	uint32_t index;
	uint16_t *p9BitsData = NULL;
	uint8_t *p8BitsData = NULL;

	if(READ_BIT(USARTx->CR1, M))
	{
		p9BitsData = (uint16_t*)pRxBuffer;
	}
	else
	{
		p8BitsData = pRxBuffer;
	}

	for(index = 0; index < size; index++)
	{
		while(!READ_BIT(USARTx->SR, RXNE));

		if(NULL != p8BitsData)
		{
			*p8BitsData++	=	USARTx->DR;
		}
		else
		{
			*p9BitsData++	=	USARTx->DR;
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
	while(!READ_BIT(USARTx->SR, TC));
}


void MCAL_USART_sendString(USART_TypeDef *USARTx, uint8_t *pString)
{
	while(*pString != '\0')
	{
		while(!READ_BIT(USARTx->SR, TXE));

		USARTx->DR = (*pString++ & 0xFF);
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
	if(READ_BIT(USART1->CR1, TXEIE))
	{
		USART1_TransmitBufferEmpty_CallBack();
	}

	if(READ_BIT(USART1->CR1, TCIE))
	{
		USART1_TransmissionComplete_CallBack();
	}

	if(READ_BIT(USART1->CR1, RXNEIE))
	{
		USART1_ReceiveBufferNotEmpty_CallBack();
	}

	if(READ_BIT(USART1->CR1, PEIE))
	{
		USART1_ParityError_CallBack();
	}
}

void USART2_IRQHandler(void)
{
	if(READ_BIT(USART2->CR1, TXE))
	{
		USART2_TransmitBufferEmpty_CallBack();
	}

	if(READ_BIT(USART2->CR1, TC))
	{
		USART2_TransmissionComplete_CallBack();
	}

	if(READ_BIT(USART2->CR1, RXNE))
	{
		USART2_ReceiveBufferNotEmpty_CallBack();
	}

	if(READ_BIT(USART2->CR1, PE))
	{
		USART2_ParityError_CallBack();
	}
}

void USART3_IRQHandler(void)
{
	if(READ_BIT(USART3->CR1, TXE))
	{
		USART3_TransmitBufferEmpty_CallBack();
	}

	if(READ_BIT(USART3->CR1, TC))
	{
		USART3_TransmissionComplete_CallBack();
	}

	if(READ_BIT(USART3->CR1, RXNE))
	{
		USART3_ReceiveBufferNotEmpty_CallBack();
	}

	if(READ_BIT(USART3->CR1, PE))
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
	while(1);
}

__attribute__((weak)) void USART1_TransmissionComplete_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART1_ReceiveBufferNotEmpty_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART1_ParityError_CallBack(void)
{
	while(1);
}


// =============================================
// =================== USART2 ==================
// =============================================
__attribute__((weak)) void USART2_TransmitBufferEmpty_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART2_TransmissionComplete_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART2_ReceiveBufferNotEmpty_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART2_ParityError_CallBack(void)
{
	while(1);
}


// =============================================
// =================== USART3 ==================
// =============================================
__attribute__((weak)) void USART3_TransmitBufferEmpty_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART3_TransmissionComplete_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART3_ReceiveBufferNotEmpty_CallBack(void)
{
	while(1);
}

__attribute__((weak)) void USART3_ParityError_CallBack(void)
{
	while(1);
}
