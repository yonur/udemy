#include "adxl345.h"

void adxl345_Read(SPI_HandleTypeDef spi, uint8_t reg, uint8_t *data_buf, uint8_t data_len) {
	
	//This read command
	reg |= (1 << 7);
	
	if(data_len > 1) 
		reg |= (1 << 6);

	data_buf[0] = reg;
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&spi, data_buf, data_len + 1, 100);
	
	HAL_SPI_Receive(&spi, data_buf, data_len + 1, 100);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	

	HAL_Delay(1);
	
}

void adxl345_Write(SPI_HandleTypeDef spi,  uint8_t reg, uint8_t *value, uint8_t data_len) {

	uint8_t *data_buf;
	
	reg &= ~(1 << 7);
	
	if(data_len > 1)
		reg |= (1 << 6);
	
	data_buf[0] = reg;
	
	for(uint8_t i = 0; i < data_len; i++) {
	
		data_buf[i + 1] = value[i];
		
	}
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&spi, data_buf, (data_len + 1), 100);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	HAL_Delay(1);
	
}

void adxl345_LowPowerMode(SPI_HandleTypeDef spi) {
	
  adxl345_Write(spi, POWER_CTL,(uint8_t *) LOW_POWER, 1);
	
}

void adxl345_StandbyMode(SPI_HandleTypeDef spi) {
		
}

void adxl345_devID(SPI_HandleTypeDef spi) {


}
