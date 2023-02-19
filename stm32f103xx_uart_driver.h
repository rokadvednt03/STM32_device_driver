/**
  ******************************************************************************
  * @file    		stm32f103xx_uart_driver.h
  * @author  		Vedant A. Rokad
  * @processor 	ARM Cortex-M3
	* @controller STM32F103C8T8
  * @date    		27-Augest-2022
  * @brief   		Device_Driver header file
  ******************************************************************************
 **/
 
#include "stm32f10x.h"
#include <stdint.h>

#define FLAG_SET 1
#define FLAG_RESET 0



/****************************************************************************************
*                                                                                       *
*                                     USART_MACROS                                      *
*                                                                                       *
/***************************************************************************************/
//@USART_FLAG

#define USART_FLAG_CTS 			   (1 << 9)
#define USART_FLAG_LINBREAK    (1 << 8)
#define USART_FLAG_TXE  			 (1 << 7)
#define USART_FLAG_TC 				 (1 << 6)
#define USART_FLAG_RXNE 			 (1 << 5)
#define USART_FLAG_IDLE 			 (1 << 4)
#define USART_FLAG_ORE 				 (1 << 3)
#define USART_FLAG_NE 				 (1 << 2)
#define USART_FLAG_FE 				 (1 << 1)
#define USART_FLAG_PE 				 (1 << 0)


 //@USART_Mode
 
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

// *@USART_Baud
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 			115200
#define USART_STD_BAUD_230400 			230400
#define USART_STD_BAUD_460800 			460800
#define USART_STD_BAUD_921600 			921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

 //*@USART_ParityControl
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0


// *@USART_WordLength
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

 //*@USART_NoOfStopBits
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

// *@USART_HWFlowControl
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3





/*********************************USART_Configuration************************************
*                                                                                       *
*                                   USART_Config_t                                      *
*                                                                                       *
/***************************************************************************************/
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;






/********************************USART_HANDLE_STRUCTURE**********************************
*                                                                                       *
*                                    USART_Handle_t                                     *
*                                                                                       *
/***************************************************************************************/
typedef struct
{
	USART_TypeDef *pUSARTx;
	USART_Config_t   USART_Config;
	
}USART_Handle_t;






/****************************************************************************************
*                                                                                       *
*                              USART_APIs_&_HELPING_FUNCTION                            *
*                                                                                       *
/***************************************************************************************/

void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi);
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_Interrupt_IRQ_Control(USART_TypeDef *pusart,uint16_t Enordi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);


void USART_Set_Baudrate(USART_TypeDef *pUSARTx , uint32_t baudrate);

void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi);
void USART_ClearFlag(USART_TypeDef *pUSARTx, uint16_t StatusFlagName);
uint8_t USART_GETFlagStatus(uint16_t flagname , USART_TypeDef *pUSARTx);

void USART_Enable(USART_TypeDef *pUSARTx , uint8_t EnorDi);
