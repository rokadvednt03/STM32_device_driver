#include "stm32f1030xx_gpio_driver.h"
#include "stm32f103xx_uart_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
void USART1_IRQHandler(void);
int main()
{
	
	uint8_t i ,j;
	char data[] = "The world is full of wonders, waiting to be explored and discovered. From the vastness of space to the depths of the oceans, there are endless mysteries and marvels that captivate the human mind. Every day, we are presented with new challenges and opportunities to learn, grow, and make a difference in the world. Whether it's through scientific research, artistic expression, or simply connecting with other people, each of us has the power to shape the world in our own unique way. Life is an adventure, full of ups and downs, but it's the journey that counts, not just the destination. So let's make the most of each moment and embrace the beauty and complexity of this incredible world we live in.";
	GPIO_Handle_t tx ;
	USART_Handle_t trans;
	
	tx.pGPIOx = GPIOA ;
	tx.GPIO_PinConfig.GPIO_ModeInOut = GPIO_OUT_10MHZ ;
	tx.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN_OUT_PP;
	tx.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9 ;
	
	trans.pUSARTx = USART1;
	trans.USART_Config.USART_Baud = USART_STD_BAUD_9600 ;
	trans.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	trans.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1 ; 
	trans.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	trans.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	
	GPIO_PeriClockControl(GPIOA , ENABLE );
	USART_PeriClockControl(USART1 , ENABLE);
	GPIO_Init(&tx);
	
	tx.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10 ;
	GPIO_Init(&tx);
	
	USART_Init(&trans);
		USART1->CR1 |= (1<<6);
		NVIC_EnableIRQ(37);
	
	USART_Enable(USART1,ENABLE);	
	
	for( i = 0 ; i < 3 ; i ++ )
	{
		
	USART_SendData(&trans,(uint8_t*)data , sizeof(data));
		
	}	
	
	USART_Enable(USART1,DISABLE);
} 

void USART1_IRQHandler(void)
{
	GPIO_Handle_t led;
	led.pGPIOx = GPIOC;
	led.GPIO_PinConfig.GPIO_PinNumber = 13;
	led.GPIO_PinConfig.GPIO_ModeInOut = GPIO_OUT_10MHZ;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&led);
}
