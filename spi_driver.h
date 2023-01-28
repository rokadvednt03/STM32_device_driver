#include "stm32f10x.h"
#include <stdint.h>
#include	<stddef.h>


#define FLAG_SET 1
#define FLAG_RESET 0


typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	
}SPI_Config_t;


typedef struct
{
	SPI_TypeDef *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
	
}SPI_Handle_t;


/*
*@SPI_Application_State
*/
#define SPI_READY						0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/*
* @SPI_FLAG_STATUS
*/
#define TXE_FLAG		(1<<1)
#define RXNE_FLAG		(1<<0)
#define SPI_BUSY_FLAG		(1<<7)
/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3

/*
 * @SPI_SCLKSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128            	6
#define SPI_SCLK_SPEED_DIV256            	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 		0
#define SPI_DFF_16BITS 		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH 		1
#define SPI_CPOL_LOW 			0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH 		1
#define SPI_CPHA_LOW 			0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN   		  1
#define SPI_SSM_DI   		  0




void SPI_PeriClockControl(SPI_TypeDef *pSPIx , uint8_t EnorDi);


void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);


void SPI_SendData(SPI_TypeDef *pSPIx , uint8_t *pTxBuffer , uint32_t len);
void SPI_ReceiveData(SPI_TypeDef *pSPIx , uint8_t *pRxBuffer , uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pRxBuffer , uint32_t len);

void SPI_IRQ_Interrupt_ENConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQ_PR_Config(uint8_t IRQNumber, uint8_t IRQ_Priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx ,uint8_t FlagName );

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

void CloseTransmission(SPI_Handle_t *pSPIHandle );
void CloseReception(SPI_Handle_t *pSPIHandle );
void CloseOVR(SPI_TypeDef *pSPIx);

void SPI_SSIControl(SPI_TypeDef *pSPIx , uint8_t EnOrDi);
void SPI_SSOE_Config(SPI_TypeDef *pSPIx , uint8_t EnOrDi);
void SPI_PeripharalControl(SPI_TypeDef *pSPIx , uint8_t EnorDi);
