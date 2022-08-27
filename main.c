#include "stm32f1030xx_gpio_driver.h"
#include "stm32f103xx_uart_driver.h"

int main()
{
	/*
	USART_Handle_t aa;
	aa.pUSARTx = USART1;
	aa.USART_Config.USART_Mode = USART_MODE_TXRX;
	aa.USART_Config.USART_WordLength = USART_WORDLEN_9BITS;
	aa.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1_5;
	aa.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_CTS_RTS;
	*/
	
	USART_PeriClockControl(USART1,ENABLE);
	USART_PeriClockControl(USART2,ENABLE);
	
	
	//USART_Init(&aa);USART_Init(&aa);
}
