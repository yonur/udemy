#include "asdxl345_stm32f446xx.h"

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

//ADXL345 güç ayarlari
void adxl345_PowerOn(SPI_HandleTypeDef spi) {
	
	adxl345_Write(spi, POWER_CTL, 0, 1);  //Wakeup
	
	adxl345_Write(spi, POWER_CTL, 16, 1); //Auto sleep
	
	adxl345_Write(spi, POWER_CTL, 8, 1);  //Measure

}

//Accelerasyon degerini okumak için
void adxl345_ReadAcceleration(SPI_HandleTypeDef spi, uint8_t * x, uint8_t *y, uint8_t *z) {

	adxl345_Read(spi, DATAX0, rec_buf, 6); 

	*x = ( rec_buf[1] << 8 ) | rec_buf[0];
	
	*y = ( rec_buf[3] << 8 ) | rec_buf[2];
	
	*z = ( rec_buf[5] << 8 ) | rec_buf[4];
	
}

//Istenen registerin istenen bitini birlemek veya sifirlamak için kullanilan method
void adxl345_SetRegisterBit(SPI_HandleTypeDef *spi,  uint8_t reg, uint8_t bit_pos, uint8_t state) {

	uint8_t *reg_val;
	
	adxl345_Read(*spi, reg, reg_val, 1);
	
	if(state) {
	
		*reg_val |= (1 << bit_pos);
	
	} else {
		
		*reg_val &= ~(1 << bit_pos);
	
	}
	
	adxl345_Write(*spi, reg, reg_val, 1);

}

//Istenen registerin istenen bitinin durumunu ögrenmek için kullanilan method
uint8_t adxl345_GetRegisterBit(SPI_HandleTypeDef *spi,  uint8_t reg, uint8_t bit_pos) {

	uint8_t *reg_val;
	
	adxl345_Read(*spi, reg, reg_val, 1);
	
	return ((*reg_val >> bit_pos) & 1);

} 

//
void adxl345_SetInterruptMapping(SPI_HandleTypeDef *spi, uint8_t interrupt_bit, uint8_t interrupt_pin) {
	
	adxl345_SetRegisterBit(spi, ADXL345_INT_MAP, interrupt_pin, interrupt_bit);

}

uint8_t adxl345_GetInterruptSource(SPI_HandleTypeDef *spi, uint8_t interruptBitPos) {

	adxl345_GetRegisterBit(spi, ADXL345_INT_SOURCE, interruptBitPos);

}

void adxl345_SetImportantInterruptMapping(SPI_HandleTypeDef *spi, uint8_t single_tap, uint8_t double_tap, uint8_t free_fall, uint8_t activity, uint8_t inactivity) {
		
		if(single_tap == 1) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(single_tap == 2) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(double_tap == 1) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(double_tap == 2) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(free_fall == 1) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT1_PIN );}
	else if(free_fall == 2) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT2_PIN );}

	if(activity == 1) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(activity == 2) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT2_PIN );}

	if(inactivity == 1) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(inactivity == 2) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT2_PIN );}

}

uint8_t adxl345_IsInterruptEnabled(SPI_HandleTypeDef *spi, uint8_t bit_pos) {

	return adxl345_GetRegisterBit(spi, ADXL345_INT_ENABLE, bit_pos);
	
}



																						
