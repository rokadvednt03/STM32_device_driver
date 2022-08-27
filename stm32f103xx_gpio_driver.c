#include<stdint.h>
#include "stm32f10x.h"
#include "stm32f1030xx_gpio_driver.h"


void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			RCC->APB2ENR |=(1<<2);
		}
		if(pGPIOx == GPIOB)
		{
			RCC->APB2ENR |=(1<<3);
		}
		if(pGPIOx == GPIOC)
		{
			RCC->APB2ENR |=(1<<4);
		}
		if(pGPIOx == GPIOD)
		{
			RCC->APB2ENR |=(1<<5);
		}
		if(pGPIOx == GPIOE)
		{
			RCC->APB2ENR |=(1<<6);
		}
		if(pGPIOx == GPIOF)
		{
			RCC->APB2ENR |=(1<<7);
		}
		if(pGPIOx == GPIOG)
		{
			RCC->APB2ENR |=(1<<8);
		}

	}

	else if(EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			RCC->APB2ENR &= ~(1<<2);
		}
		if(pGPIOx == GPIOB)
		{
			RCC->APB2ENR &= ~(1<<3);
		}
		if(pGPIOx == GPIOC)
		{
			RCC->APB2ENR &= ~(1<<4);
		}
		if(pGPIOx == GPIOD)
		{
			RCC->APB2ENR &= ~(1<<5);
		}
		if(pGPIOx == GPIOE)
		{
			RCC->APB2ENR &= ~(1<<6);
		}
		if(pGPIOx == GPIOF)
		{
			RCC->APB2ENR &= ~(1<<7);
		}
		if(pGPIOx == GPIOG)
		{
			RCC->APB2ENR &= ~(1<<8);
		}

	}
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	if(pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut <= 3)
	{
		uint32_t temp1 , temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		if(temp1 == 0)
		{
			pGPIOHandle->pGPIOx->CRL &= ~(0xF<<(temp2*4));
			pGPIOHandle->pGPIOx->CRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut<<(temp2*4));
			pGPIOHandle->pGPIOx->CRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<((temp2*4)+2));		
		}

		else
		{
			pGPIOHandle->pGPIOx->CRH &= ~(0xF<<(temp2*4));
			pGPIOHandle->pGPIOx->CRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut<<(temp2*4));
			pGPIOHandle->pGPIOx->CRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<((temp2*4)+2));
		}
	}
	else if(pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut > 3)
	{
		
		RCC->APB2ENR |= (1<<0);
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		
		if(pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_ModeInOut == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		
		uint8_t var1 =0 , var2 =0;
		var1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		var2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		AFIO->EXTICR[var1] |= (portcode<<(var2*4));
		
		
	}

}


void GPIO_IRQ_Interrupt_ENConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


void GPIO_IRQ_PR_Config(uint8_t IRQNumber, uint8_t IRQ_Priority)
{
	NVIC_SetPriority(IRQNumber,IRQ_Priority);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
		if(EXTI->PR & (1<<PinNumber))
		{
			EXTI->PR |= (1<<PinNumber);
		}
}
