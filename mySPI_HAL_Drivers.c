#include "mySPI_HAL_Drivers.h"

HAL_StatusTypeDef myHAL_SPI_DeviceMasterSet(SPI_TypeDef *SPIx, uint32_t masterMode) {
	
	if(masterMode == SPI_MASTER_MODE_SEL)
		SPIx->CR1 |= SPI_REG_CR1_MSTR;
	else if(masterMode == SPI_SLAVE_MODE_SEL)
		SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
	else
		return HAL_ERROR;
	
	return HAL_OK;

}

HAL_StatusTypeDef myHAL_SPI_DevicePhaseSet(SPI_TypeDef *SPIx, uint32_t Phase) {

		if(Phase == SPI_SECOND_CLOCK_TRANS)
			SPIx->CR1 |= SPI_REG_CR1_CPHA;
		else if(Phase == SPI_FIRST_CLOCK_TRANS)
			SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
		else
			return HAL_ERROR;
		
		return HAL_OK;
			
}

HAL_StatusTypeDef myHAL_SPI_DevicePolaritySet(SPI_TypeDef *SPIx, uint32_t polarity) {

	if(polarity == SPI_CPOL_LOW)
		SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
	else if(polarity == SPI_CPOL_HIGH)
		SPIx->CR1 |= SPI_REG_CR1_CPOL;
	else
		return HAL_ERROR;
	
	return HAL_OK;
	
}

HAL_StatusTypeDef myHAL_SPI_DeviceDataSizeSet(SPI_TypeDef *SPIx, uint32_t dataSize) {

	if(dataSize == SPI_8BIT_DF_ENABLE)
		SPIx->CR1 &= ~SPI_REG_CR1_DFF;
	else if(dataSize == SPI_16_BIT_DF_ENABLE)
		SPIx->CR1 |= SPI_REG_CR1_DFF;
	else
		return HAL_ERROR;
	
	return HAL_OK;
	
}

HAL_StatusTypeDef myHAL_SPI_DeviceBitSet(SPI_TypeDef *SPIx, uint32_t bitDirection) {	

	if(bitDirection == SPI_TX_MSB_FIRST) 
		SPIx->CR1 &= ~SPI_CR1_LSBFIRST;
	else if(bitDirection == SPI_TX_LSB_FIRST)
		SPIx->CR1 |= SPI_CR1_LSBFIRST;
	else
		return HAL_ERROR;
	
	return HAL_OK;
	
}
HAL_StatusTypeDef hal_spi_init(spi_handle_t *spi_handle) {
	
	//Enable the peripheral clock
	__HAL_RCC_SPI2_CLK_ENABLE();

	//Setting the SPI directional mode
	spi_handle->Instance->CR1 |= spi_handle->Init.Mode;

	//Setting the SPI data frame format
	spi_handle->Instance->CR1 |= spi_handle->Init.DataSize;
	
	//Setting the SPI slave management
	spi_handle->Instance->CR1 |= spi_handle->Init.NSS;
	
	//Setting the SPI SSI
	//spi_handle->Instance->CR1 |= SPI_REG_CR1_SSI;
	
	//Setting the Master or Slave mode
	spi_handle->Instance->CR1 |= spi_handle->Init.Direction;
	
	//Setting the CPHA
	spi_handle->Instance->CR1 |= spi_handle->Init.CLKPhase;
	
	//Setting the CPOL
	spi_handle->Instance->CR1 |= spi_handle->Init.CLKPolarity;

	//Setting the first bit whether LSB or MSB
	spi_handle->Instance->CR1 |= spi_handle->Init.FirstBit;
	
	spi_handle->State |= HAL_SPI_STATE_READY;
	
	return HAL_OK;
	
}

HAL_StatusTypeDef hal_spi_master_tx(spi_handle_t *spi_handle,uint8_t *buffer, uint32_t len) {

	//Setting the parameter buffer
	spi_handle->pTxBuffPtr = buffer;
	
	//Setting the parameter length
	spi_handle->TxXferSize = len;
	
	//Check the state of SPI
	if(spi_handle->State == HAL_SPI_STATE_ERROR) {
		return HAL_ERROR;
	}
	
	spi_handle->Instance->CR2 &= SPI_REG_CR2_TXEIE_ENABLE;
	
	
	while( !( (spi_handle->Instance->SR) & SPI_REG_SR_TXE_FLAG) ) {
	
		spi_handle->State &= HAL_SPI_STATE_RESET;	
		
		spi_handle->State |= HAL_SPI_STATE_BUSY_TX;
	
	}
	
	spi_handle->Instance->DR = *buffer;
	
	spi_handle->State |= HAL_SPI_STATE_READY;
	
	return HAL_OK;
	
}



