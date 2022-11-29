#include "stm32f10x.h"
#include "stm32f103xx_uart_driver.h"
#include <stdint.h>


/****************************************************************************************
*                                                                                       *
*                                TIMER_PERIPHERAL_CLOCK                                 *
*                                                                                       *
/***************************************************************************************/
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
{
		if(EnorDi == ENABLE)
			{
					if(pUSARTx == USART1)
							RCC->APB2ENR |= (1<<14);
					else if(pUSARTx == USART2)
							RCC->APB1ENR |= (1<<17);
					else if(pUSARTx == USART3)
							RCC->APB1ENR |= (1<<18);
			}
		else
			{
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
	USART_Set_Baudrate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
	
	// USART_NoOfStopBits;
	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << 12);


	// USART_WordLength;
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
					pUSARTHandle->pUSARTx->CR1 &= ~(1<<12);
		else
				pUSARTHandle->pUSARTx->CR1 |= (1<<12);
	
		
		// USART_ParityControl;
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			pUSARTHandle->pUSARTx->CR1 &= ~(1<<10);
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
