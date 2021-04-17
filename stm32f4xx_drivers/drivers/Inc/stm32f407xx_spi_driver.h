/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 26 Mar 2021
 *      Author: emine
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
typedef struct
{

		uint8_t SPI_DeviceMode;		/*@MASTER OR SLAVE MODES */
		uint8_t SPI_BusConfig;		/*@FULL-DUPLEX OR SIMPLEX */
		uint8_t SPI_DFF;			/*@8 bits or 16 bits */
		uint8_t SPI_CPHA;			/*@PHASE=0(data sample happens first edge of the clock), =1(second edge)*/
		uint8_t SPI_CPOL;			/*@POLARITY=0 ( idle state is low), =1( idle state high)*/
		uint8_t SPI_SSM;			/*@SW -HW */
		uint8_t SPI_SclkSpeed;			/*@SPEEDS */

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;

#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

#define SPI_CPHA_FIRST_EDGE			0
#define SPI_CPHA_SECOND_EDGE			1

#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

#define SPI_SSM_EN			1
#define SPI_SSM_DI			0

/*SPI RELATED STATUS FLAGS DEFINITIONS */
#define SPI_TXE_FLAG	( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG	( 1<< SPI_SR_RXNE )
#define SPI_BUSY_FLAG	( 1<< SPI_SR_BSY )

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

 //SPI application states
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2


//possible SPI Apllication EVENTS

#define SPI_EVENT_TX_CMPLT 				1
#define SPI_EVENT_RX_CMPLT 				2
#define SPI_EVENT_OVR_ERR 				3
#define SPI_EVENT_CRC_ERR 				4

/*APIs supported by this driver */

/*Peripheral Clock Setup */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*Init and De-Init */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Data Send and Receive */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len); //uint32_t Len is standard practice

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len); //uint32_t Len is standard practice

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*IRQ Configurations and ISR Handling */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*Other Peripherals and Control APIs */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//Application callback

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
