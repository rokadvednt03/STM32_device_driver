#include <stdint.h>
#include "stm32f10x.h"



#define GPIO_MODE_ANALOGIN	     0
#define GPIO_MODE_FLOATIN 		 1
#define GPIO_MODE_IN_PUPD		 2
#define GPIO_MODE_RESERVED 		 3
#define GPIO_MODE_IT_FT			 4
#define GPIO_MODE_IT_RT			 5
#define GPIO_MODE_IT_RFT		 6


#define GPIO_MODE_OUT_PP 		 0
#define GPIO_MODE_OUT_OD 		 1
#define GPIO_MODE_ALTFN_OUT_PP   2
#define GPIO_MODE_ALTFN_OUT_OD 	 3

#define GPIO_IN 				 0
#define GPIO_OUT_10MHZ  		 1
#define GPIO_OUT_2MHZ   		 2
#define GPIO_OUT_50MHZ   		 3



#define GPIO_PIN_NO_0  0
#define GPIO_PIN_NO_1  1
#define GPIO_PIN_NO_2  2
#define GPIO_PIN_NO_3  3
#define GPIO_PIN_NO_4  4
#define GPIO_PIN_NO_5  5
#define GPIO_PIN_NO_6  6
#define GPIO_PIN_NO_7  7
#define GPIO_PIN_NO_8  8
#define GPIO_PIN_NO_9  9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


#define GPIO_BASEADDR_TO_CODE(x) 	 ((x == GPIOA) ? 0 :\
																		(x == GPIOB) ? 1 :\
																		(x == GPIOC) ? 2 :\
																		(x == GPIOD) ? 3 :\
																		(x == GPIOE) ? 4 :\
																		(x == GPIOF) ? 5 :\
																		(x == GPIOG) ? 6 : 0 )



typedef struct
{
	uint32_t GPIO_PinNumber;
	uint32_t GPIO_PinMode; 
	uint32_t GPIO_ModeInOut ;
	uint32_t GPIO_PinSpeed;
	uint32_t GPIO_PinPdControl;
	uint32_t GPIO_Pintype;
	uint32_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;



typedef struct
{
	GPIO_TypeDef *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;




void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);



void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx , uint8_t EnorDi);



uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx , uint8_t PinNumber);
uint8_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx , uint8_t PinNumber , uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx , uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx , uint8_t PinNumber);

void GPIO_IRQ_Interrupt_ENConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQ_PR_Config(uint8_t IRQNumber, uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t PinNumber);
