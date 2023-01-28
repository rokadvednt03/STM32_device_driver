/*
 * spi.c
 *
 *  Created on: Jul 25, 2022
 *      Author: rokad
 */


#include "stm32f10x.h"
#include "spi_driver.h"
#include <stdint.h>
#include <stddef.h>

static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_ERRIE_IT_Handle(SPI_Handle_t *pSPIHandle);


void SPI_PeriClockControl(SPI_TypeDef *pSPIx , uint8_t EnorDi)
{
		if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				RCC->APB2ENR |= (1<<12);
			}
			else if(pSPIx == SPI2)
			{
				RCC->APB1ENR |= (1<<14);
			}
			else if(pSPIx == SPI3)
			{
				RCC->APB1ENR |= (1<<15);
			}

		}
		else
		{
			if(pSPIx == SPI1)
			{
				RCC->APB2ENR &= ~(1<<12);
			}
			else if(pSPIx == SPI2)
			{
				RCC->APB1ENR &= ~(1<<14);
			}
			else if(pSPIx == SPI3)
			{
				RCC->APB1ENR &= ~(1<<15);
			}
		}

}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint16_t tempreg = 0;
	//1.SPI_DeviceMode
	tempreg |= ( pSPIHandle->SPIConfig.SPI_DeviceMode << 2 );

	//2.SPI_BusConfig
		if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << 15);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << 15);
		//RXONLY bit must be set
		tempreg |= ( 1 << 10);
	}

	//3.SPI_SCLKSpeed

		tempreg |= (pSPIHandle->SPIConfig.SPI_SCLKSpeed << 3);

	//4.SPI_DFF
	 tempreg |= ( pSPIHandle->SPIConfig.SPI_DFF << 11 );

	//5.SPI_CPOL
	tempreg |= ( pSPIHandle->SPIConfig.SPI_CPOL << 1 );
	//6.SPI_CPHA
	tempreg |= ( pSPIHandle->SPIConfig.SPI_CPHA << 0 );
	//7.SPI_SSM
	tempreg |= ( pSPIHandle->SPIConfig.SPI_SSM << 9 );

	pSPIHandle->pSPIx->CR1 = tempreg;
}

uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx ,uint8_t FlagName)
{
		if(pSPIx->SR & FlagName)
		{
			return FLAG_SET;
		}
		else
		{
		return FLAG_RESET;
		}
}

void SPI_SendData(SPI_TypeDef *pSPIx , uint8_t *pTxBuffer , uint32_t len)
{
	uint16_t *pdata;
	
		while(len>0)
		{
			//1.TXE buffer
			while( SPI_GetFlagStatus(SPI2,TXE_FLAG) == FLAG_RESET);
			//2. check DFF bit
			if(pSPIx->CR1 & (1<<11))
			{
				pdata = (uint16_t*)pTxBuffer;
				//1.16bit transfer
				pSPIx->DR = (*pdata & 0xFFFF);
				len--;
				len--;
				pdata++;
			}
			else
			{
			//2.8bit data transfer
				pSPIx->DR = *(pTxBuffer);
				len--;
				pTxBuffer++;
			}

		}
}

void SPI_ReceiveData(SPI_TypeDef *pSPIx , uint8_t *pRxBuffer , uint32_t len)
{
	while(len>0)
		{
			//1.TXE buffer
			while( SPI_GetFlagStatus(SPI2,TXE_FLAG) == FLAG_RESET);
			//2. check DFF bit
			if(pSPIx->CR1 & (1<<11))
			{
				//1.16bit transfer
				 *((uint16_t*)pRxBuffer) = pSPIx->DR;
				len--;
				len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
			//2.8bit data transfer
			  *(pRxBuffer) = 	pSPIx->DR;
				len--;
				pRxBuffer++;
			}

		}
}



void SPI_IRQ_Interrupt_ENConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		NVIC_EnableIRQ(IRQNumber);
	}
	else
	{
		NVIC_DisableIRQ(IRQNumber);
	}
	
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t len)
{

	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1.Save TXE buffer addr and len in some global variable
				pSPIHandle->pTxBuffer = pTxBuffer ;
				pSPIHandle->TxLen = len ;
		//2.mark the SPI state as busy in transmission so that no other code can over same SPI peripheral untill transmission is over
				pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3.Enable TXEIE so that when TXE flag is set get interrupt
				pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE) ;
		//4.Data Transmission will be handled by the ISR code(will be implemented)
	}
	return state;
}



uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pRxBuffer , uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1.Save TXE buffer addr and len in some global variable
				pSPIHandle->pRxBuffer = pRxBuffer ;
				pSPIHandle->RxLen = len ;
		//2.mark the SPI state as busy in transmission so that no other code can over same SPI peripheral untill transmission is over
				pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3.Enable TXEIE so that when TXE flag is set get interrupt
				pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE) ;
		//4.Data Transmission will be handled by the ISR code(will be implemented)
	}
	return state;
}



void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
		uint8_t temp1,temp2;
		//check TXE
		temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
		temp2 = pSPIHandle->TxState & (1 << SPI_CR2_TXEIE);
		
		if(temp1 && temp2)
		{
			SPI_TXE_IT_Handle(pSPIHandle);
		}
			
		//check RXNE
		temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
		temp2 = pSPIHandle->RxState & (1 << SPI_CR2_RXNEIE);
		
		if(temp1 && temp2)
		{
			SPI_RXNE_IT_Handle(pSPIHandle);
		}
		
		//check OVR
		
		temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
		temp2 = pSPIHandle->TxState & (1 << SPI_CR2_ERRIE);
		
		if(temp1 && temp2)
		{
			SPI_ERRIE_IT_Handle(pSPIHandle);
		}
}

static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle)
{
		if(pSPIHandle->pSPIx->CR1 & (1<<11))
			{
				//1.16bit transfer
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else
			{
			//2.8bit data transfer
				pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			if(! pSPIHandle->TxLen)
			{
				CloseTransmission(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
			}
}

static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<11))
			{
				//1.16bit transfer
				 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->RxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
			else
			{
			//2.8bit data transfer
			 *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;
			}
			if(! pSPIHandle->RxLen)
			{
				CloseReception(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
			}
}

static void SPI_ERRIE_IT_Handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void CloseTransmission(SPI_Handle_t *pSPIHandle )
{
				pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
				pSPIHandle->pTxBuffer = NULL;
				pSPIHandle->TxLen = 0;
				pSPIHandle->TxState = SPI_READY;
}

void CloseReception(SPI_Handle_t *pSPIHandle )
{
				pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
				pSPIHandle->pRxBuffer = NULL;
				pSPIHandle->RxLen = 0;
				pSPIHandle->RxState = SPI_READY;
}

void CloseOVR(SPI_TypeDef *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__attribute__((weak))void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation.
	
}


void SPI_SSOE_Config(SPI_TypeDef *pSPIx , uint8_t EnOrDi)
{
		if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << 2) ;
		}
		else
		{
			pSPIx->CR2 &= ~(1 << 2) ;
		}
}

void SPI_PeripharalControl(SPI_TypeDef *pSPIx , uint8_t EnorDi)
{
		if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << 6) ;
		}
		else
		{
			pSPIx->CR1 &= ~(1 << 6) ;
		}
}

void SPI_SSIControl(SPI_TypeDef *pSPIx , uint8_t EnOrDi)
{
		if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << 8) ;
		}
		else
		{
			pSPIx->CR1 &= ~(1 << 8) ;
		}
}
