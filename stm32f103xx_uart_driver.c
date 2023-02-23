/**
  ******************************************************************************
  * @file    		stm32f103xx_uart_driver.c
  * @author  		Vedant A. Rokad
  * @processor 	ARM Cortex-M3
	* @controller STM32F103C8T8
  * @date    		27-Augest-2022
  * @brief   		Device_Driver source file
  ******************************************************************************
 **/
 
#include "stm32f10x.h"
#include "stm32f103xx_uart_driver.h"
#include <stdint.h>


/****************************************************************************************
*                                                                                       *
*                                USART_PERIPHERAL_CLOCK                                 *
*                                                                                       *
/***************************************************************************************/
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
{
		//check whether peripharal is enable or not
		if(EnorDi == ENABLE)
			{
					//enable the usart peripheral 
					if(pUSARTx == USART1)
							RCC->APB2ENR |= (1<<14);
					else if(pUSARTx == USART2)
							RCC->APB1ENR |= (1<<17);
					else if(pUSARTx == USART3)
							RCC->APB1ENR |= (1<<18);
			}
		else
			{
					//disable the usart peripheral 
					if(pUSARTx == USART1)
							RCC->APB1ENR &= ~(1<<14);
					else if(pUSARTx == USART2)
							RCC->APB2ENR &= ~(1<<17);
					else if(pUSARTx == USART3)
							RCC->APB2ENR &= ~(1<<18);
			}
}




/****************************************************************************************
*                                                                                       *
*                                   USART_INITIALIZE                                    *
*                                                                                       *
/***************************************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// select the USART mode
	// 1.USART_MODE_ONLY_Transmission
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
			pUSARTHandle->pUSARTx->CR1 |= (1<<3);
			pUSARTHandle->pUSARTx->CR1 &= ~(1<<2);
	}
	//2.USART_MODE_ONLY_Receiver
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
			pUSARTHandle->pUSARTx->CR1 &= ~(1<<3);
			pUSARTHandle->pUSARTx->CR1 |= (1<<2);
	}
	//3.USART_MODE_Transmission_Receiver
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
			pUSARTHandle->pUSARTx->CR1 |= (1<<3);
			pUSARTHandle->pUSARTx->CR1 |= (1<<2);
	}
	
			
	//Set the baudrate
	USART_Set_Baudrate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
	
	//Configure Number of STOP bit 
	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << 12);


	// USART_WordLength;
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
					pUSARTHandle->pUSARTx->CR1 &= ~(1<<12);
		else
				pUSARTHandle->pUSARTx->CR1 |= (1<<12);
	
		
	//USART_Parity_Control
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			pUSARTHandle->pUSARTx->CR1 &= ~(1<<10);
	else
	{
		//USART_EVEN_PARITY
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
		{
				pUSARTHandle->pUSARTx->CR1 |= (1<<10);
				pUSARTHandle->pUSARTx->CR1 &= ~(1<<9);
		}
		//USART_ODD_PARITY
		else
		{
				pUSARTHandle->pUSARTx->CR1 |= (1<<10);
				pUSARTHandle->pUSARTx->CR1 |= (1<<9);
		}
		
	}
	
	
	
	//USART_Hardware_Flow_Control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE)
	{
		pUSARTHandle->pUSARTx->CR3 &= ~(1<<8);
		pUSARTHandle->pUSARTx->CR3 &= ~(1<<9);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		pUSARTHandle->pUSARTx->CR3 &= ~(1<<8);
		pUSARTHandle->pUSARTx->CR3 |= (1<<9);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1<<8);
		pUSARTHandle->pUSARTx->CR3 &= ~(1<<9);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1<<8);
		pUSARTHandle->pUSARTx->CR3 |= (1<<9);
	}
}





/****************************************************************************************
*                                                                                       *
*                                   USART_SEND_DATA                                     *
*                                                                                       *
/***************************************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len--)
	{
	while(! USART_GETFlagStatus(USART_FLAG_TXE,pUSARTHandle->pUSARTx));
	uint16_t *pdata;
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		pdata = (uint16_t*)pTxBuffer;
		pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
		{
			pTxBuffer++;
			pTxBuffer++;
		}
		else{
			pTxBuffer++;
		}
	}
	else
	{
		pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
		pTxBuffer++;
	}
	while(! USART_GETFlagStatus(USART_FLAG_TC , pUSARTHandle->pUSARTx));

	}
}




/****************************************************************************************
*                                                                                       *
*                                   USART_GET_FLAG_STATUS                               *
*                                                                                       *
/***************************************************************************************/
uint8_t USART_GETFlagStatus(uint16_t flagname , USART_TypeDef *pUSARTx)
{
	if(pUSARTx->SR & flagname)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

/****************************************************************************************
*                                                                                       *
*                                   USART_ClearFlag			                                *
*                                                                                       *
/***************************************************************************************/
void USART_ClearFlag(USART_TypeDef *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}


/****************************************************************************************
*                                                                                       *
*                                   USART_SET_BAUDRATE                                  *
*                                                                                       *
/***************************************************************************************/
void USART_Set_Baudrate(USART_TypeDef *pUSARTx , uint32_t baudrate)
{
	uint32_t M_Part  , F_Part ;
	uint32_t temp;
	uint32_t pclk1 = 36000000 ;
	uint32_t pclk2 = 72000000 ;
	
	if(pUSARTx == USART1)
	{
			temp = ((pclk2 * 25) / (4 * 9600) ) ;
	}
	else
	{
		temp = ((pclk1 * 25 ) / (4 * baudrate) ) ;
	}
	
	M_Part = temp / 100 ;
	F_Part = ((temp - (M_Part*100))*16)/100 ;
	
	pUSARTx->BRR |= (M_Part << 4);
	pUSARTx->BRR |= (F_Part << 0);
	
}




/****************************************************************************************
*                                                                                       *
*                                   USART_ENABLE		                                    *
*                                                                                       *
/***************************************************************************************/
void USART_Enable(USART_TypeDef *pUSARTx , uint8_t EnorDi)
{
		if(EnorDi == ENABLE)
		{
				pUSARTx->CR1 |= (1 << 13 );
		}
		else
		{
				pUSARTx->CR1 &= ~(1 << 13 );
		}
}

/****************************************************************************************
*                                                                                       *
*                             USART_Interrupt_IRQ_Control		                            *
*                                                                                       *
/***************************************************************************************/
void USART_Interrupt_IRQ_Control(USART_TypeDef *pusart,uint16_t Enordi) 
{
	if(Enordi == ENABLE)
	{
			if(pusart == USART1)
				NVIC_EnableIRQ(37);
			else if(pusart == USART1)
				NVIC_EnableIRQ(38);
			else if(pusart == USART1)
				NVIC_EnableIRQ(39);
	}
	else
	{
			if(pusart == USART1)
				NVIC_DisableIRQ(37);
			else if(pusart == USART1)
				NVIC_DisableIRQ(38);
			else if(pusart == USART1)
				NVIC_DisableIRQ(39);
	}
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	NVIC_SetPriority(IRQNumber,IRQPriority);
}
