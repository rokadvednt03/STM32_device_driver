#include "stm32f10x.h"
#include "stm32f103xx_uart_driver.h"
#include <stdint.h>


void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			RCC->APB1ENR |= (1<<14);
		}
		else if(pUSARTx == USART2)
		{
			RCC->APB2ENR |= (1<<17);
		}
		else if(pUSARTx == USART3)
		{
			RCC->APB2ENR |= (1<<18);
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			RCC->APB1ENR &= ~(1<<14);
		}
		else if(pUSARTx == USART2)
		{
			RCC->APB2ENR &= ~(1<<17);
		}
		else if(pUSARTx == USART3)
		{
			RCC->APB2ENR &= ~(1<<18);
		}
	}
	
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// USART_Mode;
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<3);
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<2);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<3);
		pUSARTHandle->pUSARTx->CR1 |= (1<<2);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<3);
		pUSARTHandle->pUSARTx->CR1 |= (1<<2);
	}
	
		
	
	// USART_Baud;
	
	// USART_NoOfStopBits;
	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << 12);
	// USART_WordLength;
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<12);
	}
	else
	{
		pUSARTHandle->pUSARTx->CR1 |= (1<<12);
	}
	// USART_ParityControl;
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1<<10);
	}
	else
	{
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
		{
				pUSARTHandle->pUSARTx->CR1 |= (1<<10);
				pUSARTHandle->pUSARTx->CR1 &= ~(1<<9);
		}
		else
		{
				pUSARTHandle->pUSARTx->CR1 |= (1<<10);
				pUSARTHandle->pUSARTx->CR1 |= (1<<9);
		}
		
	}
	// USART_HWFlowControl;
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
