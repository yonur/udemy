#include "asdxl345_stm32f446xx.h"

void adxl345Write_SPI(SPI_HandleTypeDef *spi, uint8_t reg, uint8_t *data_buf, uint8_t data_length ) {

	uint8_t* packet;
	
	packet[0] &= ~(1 << 7);
	
	if(data_length > 1) {
	
		packet[0] |= (1 << 6);
		
		packet[0] |= reg;
		
		packet++; 
		
		for(uint8_t i = 0; i < data_length; i++) {
		
				*packet = data_buf[i];
				
				 packet++;
			
		}
		
	} else {
	
		packet[0] &= ~(1 << 6);
		
		packet[0] |= reg;
		
		packet++;
		
		*packet = *data_buf;
		
	}
	  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	
	HAL_SPI_Transmit(spi, packet, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	
}

void adxl345Read_SPI(SPI_HandleTypeDef *spi, uint8_t *reg, uint8_t *tx_data_buf, uint8_t *rx_data_buf, uint8_t data_length) {

	uint8_t* packet;
	uint8_t spiData;
	
	*reg |= (1 << 7);
	
	if(data_length > 1)
	 *reg |= (1 << 6);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	
	HAL_SPI_Transmit(spi, reg, 2, 100);
	
	while(__HAL_SPI_GET_FLAG(spi, HAL_SPI_STATE_BUSY));
	
	spiData = HAL_SPI_Receive(spi, rx_data_buf, 2, 100);
	
	while(__HAL_SPI_GET_FLAG(spi, HAL_SPI_STATE_BUSY));
	
	spiData = HAL_SPI_Transmit(spi, 0x00, 1, 100);
	
	while(__HAL_SPI_GET_FLAG(spi, HAL_SPI_STATE_BUSY));
	
	spiData = HAL_SPI_Receive(spi, rx_data_buf, 2, 100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	
	HAL_SPI_TransmitReceive(spi,reg,rx_data_buf,2,100);
}
