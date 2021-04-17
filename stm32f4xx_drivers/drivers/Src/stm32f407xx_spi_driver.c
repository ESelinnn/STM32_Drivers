/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 26 Mar 2021
 *      Author: emine
 */

#include "stm32f407xx_spi_driver.h"
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx==SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx==SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
	else
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_DN();
		}else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DN();
		}else if(pSPIx==SPI3)
		{
			SPI3_PCLK_DN();
		}

	}

}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//SPI Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle -> pSPIx, ENABLE);
	uint32_t tempreg=0; // we use one variable for one register called SPI_CR1
	//1. configure the device mode
	tempreg |=pSPIHandle-> SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;
	//2. configure the bus config
	if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//tempreg &= ~(1<<SPI_CR1_RXONLY); //SELİN
	}else if (pSPIHandle-> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
		//tempreg &= ~(1<<SPI_CR1_RXONLY); //SELİN

	}else if (pSPIHandle-> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	//3.configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle-> SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

	//4. configure the DDF
	tempreg |= pSPIHandle-> SPIConfig.SPI_DFF<<SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle-> SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle-> SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
		{
			SPI1_REG_RESET();
		}else if(pSPIx==SPI2)
		{
			SPI2_REG_RESET();
		}else if(pSPIx==SPI3)
		{
			SPI3_REG_RESET();
		}

}


void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pSPIx -> CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pSPIx -> CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx -> CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx ->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

}


/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call search polling code !!!!!
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) //uint32_t Len is standard practice
{
	while(Len>0)
	{
		//1. wait untill TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		if( (pSPIx -> CR1 &( 1 << SPI_CR1_DFF ) ) )
		{
			//16 bit DFF
			//1. load the data in the DR
			pSPIx -> DR = *((uint16_t*)pTxBuffer);


			//2. Decrement LEN and increment buffer addr
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;


		}else
		{
			//8 bit DFF
			//1. load the data to the DR
			pSPIx -> DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}



}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//1. wait untill RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);
		if( (pSPIx -> CR1 &( 1 << SPI_CR1_DFF ) ) )
		{
			//16 bit DFF
			//1. load the data in the DR
			*((uint16_t*)pRxBuffer) = pSPIx ->DR;
			//2. Decrement LEN and increment buffer addr
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;


		}else
		{
			//8 bit DFF
			//1. load the data to the DR
			*pRxBuffer = pSPIx -> DR ;
			Len--;
			pRxBuffer++;
		}

	}

}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
// these functions should not be called by user thereforethey are not defined in .h file
//and functions indicades with static so if user try to call them there will be an error
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_Err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle -> pSPIx -> SR & (1<< SPI_SR_TXE);
	temp2 =pHandle -> pSPIx -> CR2 & (1<< SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//lets check for RXNE
	temp1 = pHandle -> pSPIx -> SR & (1<< SPI_SR_RXNE);
	temp2 =pHandle -> pSPIx -> CR2 & (1<< SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxe_interrupt_handle(pHandle);
	}

	//check for OVR FLAG
	temp1 = pHandle -> pSPIx -> SR & (1<< SPI_SR_OVR);
	temp2 =pHandle -> pSPIx -> CR2 & (1<< SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle OVR
		spi_ovr_Err_interrupt_handle(pHandle);
	}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( (pSPIHandle->pSPIx -> CR1 &( 1 << SPI_CR1_DFF ) ) )
		{
			//16 bit DFF
			//1. load the data in the DR
			pSPIHandle->pSPIx -> DR = *((uint16_t*)pSPIHandle->pTxBuffer);


			//2. Decrement LEN and increment buffer addr
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;


	}else
	{
		    //8 bit DFF
			//1. load the data to the DR
			pSPIHandle->pSPIx -> DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle-> TxLen)
	{
		//TxLen is zero, so close the spi transmission and inform the application that TX is over
		//this prevents interrupts from setting up of TXE flag

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);

	}
}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if( (pSPIHandle->pSPIx -> CR1 &( 1 << SPI_CR1_DFF ) ) )
			{
				//16 bit DFF
				//1. load the data in the DR
				*((uint16_t*)pSPIHandle->pRxBuffer)= (uint16_t)pSPIHandle->pSPIx -> DR ;


				//2. Decrement LEN and increment buffer addr
				pSPIHandle->RxLen -= 2;
				pSPIHandle->pRxBuffer++;
				pSPIHandle->pRxBuffer++;


		}else
		{
			    //8 bit DFF
				//1. load the data to the DR
			 	*(pSPIHandle->pRxBuffer)= (uint8_t)pSPIHandle->pSPIx -> DR ;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;
		}

		if(! pSPIHandle-> RxLen)
		{
			//TxLen is zero, so close the spi transmission and inform the application that TX is over
			//this prevents interrupts from setting up of TXE flag

			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

		}

}
static void spi_ovr_Err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle -> TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp; //if we dont use this temp varible we can write this
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//this is a weak implemantation the app mac override this function
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~(1<< SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle-> TxLen =0;
	pSPIHandle-> TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~(1<< SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle-> RxLen =0;
	pSPIHandle-> RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx ->DR;
	temp = pSPIx ->SR;
	(void)temp;
}
/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle-> TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. SAVE THE TX BUFFER ADDR AND LEN info in some global variables
		pSPIHandle -> pTxBuffer = pTxBuffer;
		pSPIHandle -> TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle -> TxState = SPI_BUSY_IN_TX;
		// 3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx ->CR2 |= (1<< SPI_CR2_TXEIE);
		// 4. data transmission will be handled by the other ISR code (will implement later)

	}
	return state;
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle-> RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. SAVE THE TX BUFFER ADDR AND LEN info in some global variables
		pSPIHandle -> pRxBuffer = pRxBuffer;
		pSPIHandle -> RxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle -> RxState = SPI_BUSY_IN_RX;
		// 3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx ->CR2 |= (1<< SPI_CR2_RXNEIE);
		// 4. data transmission will be handled by the other ISR code (will implement later)

	}
	return state;
}

