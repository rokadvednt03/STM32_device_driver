#include "stm32f10x.h"
#include "stm32f103xx_uart_driver.h"
#include <stdint.h>


void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			RCC->APB2ENR |= (1<<14);
		}
		else if(pUSARTx == USART2)
		{
			RCC->APB1ENR |= (1<<17);
		}
		else if(pUSARTx == USART3)
		{
			RCC->APB1ENR |= (1<<18);
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

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
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
}

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

uint32_t GetRCC_PCLK1(void)
{
	uint32_t RCC_PCLK1;
	uint32_t sysclk ;
	uint32_t temp;
	sysclk = 8000000;
	uint32_t ahb,apb1,apb2;
	uint32_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
	uint32_t APB1_PreScaler[4] = {2,4,8,16};
	
	temp = (RCC->CFGR >> 4 ) & (0xF);
	if(temp < 8 )
	{
		ahb = 1;
	}
	else
	{
		ahb = AHB_PreScaler[temp-8];
	}
	
	temp = ((RCC->CFGR) >> 8 ) & (0x07);
	if(temp < 4 )
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB1_PreScaler[temp-4];
	}
	RCC_PCLK1 = (sysclk/ahb)/apb1;
	return RCC_PCLK1;
}


uint32_t GetRCC_PCLK2(void)
{
	uint32_t RCC_PCLK2;
	uint32_t sysclk ;
	uint32_t temp;
	sysclk = 8000000;
	uint32_t ahb,apb2;
	uint32_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
	uint32_t APB2_PreScaler[4] = {2,4,8,16};
	
	temp = (RCC->CFGR >> 4 ) & (0xF);
	if(temp < 8 )
	{
		ahb = 1;
	}
	else
	{
		ahb = AHB_PreScaler[temp-8];
	}
	
	temp = ((RCC->CFGR) >> 11 ) & (0x07);
	if(temp < 4 )
	{
		apb2 = 1;
	}
	else
	{
		apb2 = APB2_PreScaler[temp-4];
	}
	RCC_PCLK2 = (sysclk/ahb)/apb2;
	return RCC_PCLK2;
}
