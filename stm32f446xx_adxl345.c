#include "adxl345_stm32f446xx.h"

void adxl345_PowerOn(SPI_HandleTypeDef spi) {
	
	adxl345_Write(spi, POWER_CTL, ADXL345_POWER_CTL_WAKEUP_8HZ_VAL, 1);  //Wakeup
	
	adxl345_Write(spi, POWER_CTL, (uint8_t *)ADXL345_POWER_CTL_AUTO_SLEEP_VAL, 1); //Auto sleep
	
	adxl345_Write(spi, POWER_CTL, (uint8_t *)ADXL345_POWER_CTL_MEASURE_VAL, 1);  //Measure

}

void adxl345_ReadAcceleration(SPI_HandleTypeDef spi, uint8_t * x, uint8_t *y, uint8_t *z) {

	adxl345_Read(spi, DATAX0, rec_buf, 6); 

	*x = ( rec_buf[1] << 8 ) | rec_buf[0];
	
	*y = ( rec_buf[3] << 8 ) | rec_buf[2];
	
	*z = ( rec_buf[5] << 8 ) | rec_buf[4];
	
}

void adxl345_GetGxyz(SPI_HandleTypeDef *spi, uint8_t *x, uint8_t *y, uint8_t *z) {

	*x = (*x) * gains[0];
	
	*y = (*y) * gains[1];
	
	*z = (*z) * gains[2];

}

//This function is used to send data to the dedicated register of ADXL345
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

//This function is used to read data from the dedicated register of ADXL345
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

void adxl345_GetRangeSettings(SPI_HandleTypeDef *spi, uint8_t *range_settings) {
	
	uint8_t buf;

	adxl345_Read(*spi, ADXL345_DATA_FORMAT, &buf, SET);

	*range_settings = (buf & ADXL345_DATA_FORMAT_RANGE_MSK);

}
																						
void adxl345_SetRangeSettings(SPI_HandleTypeDef *spi, uint8_t val) {
	
	uint8_t rangeSetted = 0x00;
	
	adxl345_Read(*spi, ADXL345_DATA_FORMAT, &rangeSetted, 1);
	
	MODIFY_REG(rangeSetted, ADXL345_DATA_FORMAT_RANGE_MSK, val);

	adxl345_Write(*spi, ADXL345_DATA_FORMAT, &rangeSetted, 1);
	
}

uint8_t adxl345_GetSelfTestBit(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_SELF_TEST_BIT);

}

void adxl345_SetSelfTestBit(SPI_HandleTypeDef *spi, uint8_t self_testbit_control) {

	adxl345_SetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_SELF_TEST_BIT, self_testbit_control);

}

uint8_t adxl345_GetSPI_State(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_SPI_BIT);
	
}

void adxl345_SetSPIState(SPI_HandleTypeDef *spi, uint8_t spi_state_control) {

	adxl345_SetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_SPI_BIT, spi_state_control);

}

uint8_t adxl345_GetInterruptLevelBit(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_INT_INVERT_BIT);

}

void adxl345_SetInterruptLevelBit(SPI_HandleTypeDef *spi, uint8_t int_invert_control) {
	
	adxl345_SetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_INT_INVERT_BIT, int_invert_control);
	
}

uint8_t adxl345_GetFullResBitState(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_FULL_RES_BIT);

}

void adxl345_SetFullResBitState(SPI_HandleTypeDef *spi, uint8_t full_res_bit_control) {

	adxl345_SetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_FULL_RES_BIT, full_res_bit_control);
}

uint8_t adxl345_GetJustifyBitState(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_JUSTIFY_BIT);

}

void adxl345_SetJustifyBitState(SPI_HandleTypeDef *spi, uint8_t justify_bit_control) {

	adxl345_SetRegisterBit(spi, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_JUSTIFY_BIT, justify_bit_control);

}

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
//Should Set Between 0 and 255
//Scale Factor is 62.5 mg/LSB
//A Value of 0 May Result in Undesirable Behavior
void adxl345_SetTapThreshold(SPI_HandleTypeDef *spi, uint8_t tap_threshold) {
	
	adxl345_Write(*spi, ADXL345_THRESH_TAP, &tap_threshold, 1);

}

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
uint8_t adxl345_GetTapThreshold(SPI_HandleTypeDef *spi) {

	uint8_t tap_threshold_val;
	
	adxl345_Read(*spi, ADXL345_THRESH_TAP, &tap_threshold_val, 1);
	
	return tap_threshold_val;
	
}

void adxl345_SetAxisGains(double *_gains) {

	for(uint8_t i = 0; i < 3; i++) {
	
		gains[i] = _gains[i];
	
	}
	
}

void adxl345_GetAxisGains(double *_gains) {

	for(uint8_t i = 0; i < 3; i++) {
	
		_gains[i] = gains[i];	
		
	}

}

void adxl345_GetAxisOffset(SPI_HandleTypeDef *spi, uint8_t *x, uint8_t *y, uint8_t *z) {

	adxl345_Read(*spi, ADXL345_OFSX, x, 1);
	
	adxl345_Read(*spi, ADXL345_OFSY, y, 1);
	
	adxl345_Read(*spi, ADXL345_OFSZ, z, 1);

}

void adxl345_SetAxisOffset(SPI_HandleTypeDef *spi, uint8_t *x, uint8_t *y, uint8_t *z) {

	adxl345_Write(*spi, ADXL345_OFSX, x, 1);
	
	adxl345_Write(*spi, ADXL345_OFSY, y , 1);
	
	adxl345_Write(*spi, ADXL345_OFSZ, z, 1);

}

void adxl345_SetTapDuration(SPI_HandleTypeDef *spi, uint8_t tap_duration) {
	
	adxl345_Write(*spi, ADXL345_DUR, &tap_duration, 1);
	
}

uint8_t adxl345_GetTapDuration(SPI_HandleTypeDef *spi) {

	uint8_t tap_dur_val;
	
	adxl345_Read(*spi, ADXL345_DUR, &tap_dur_val, 1);
	
	return tap_dur_val;
	
}

void adxl345_SetDoubleTapLatency(SPI_HandleTypeDef *spi, uint8_t double_tap_latency) {

	adxl345_Write(*spi, ADXL345_LATENT, &double_tap_latency, 1);

}

uint8_t adxl345_GetDoubleTapLatency(SPI_HandleTypeDef *spi) {

	uint8_t double_tap_latency;
	
	adxl345_Read(*spi, ADXL345_LATENT, &double_tap_latency, 1);
	
	return double_tap_latency;

}

void adxl345_SetDoubleTapWindow(SPI_HandleTypeDef *spi, uint8_t double_tap_window) {

	adxl345_Write(*spi, ADXL345_WINDOW, &double_tap_window, 1);

}

uint8_t adxl34_GetDoubleTapWindow(SPI_HandleTypeDef *spi) {

	uint8_t double_tap_val;
	
	adxl345_Read(*spi, ADXL345_WINDOW, &double_tap_val, 1);
	
	return double_tap_val;

}

void adxl345_SetActivityThreshold(SPI_HandleTypeDef *spi, uint8_t activity_threshold) {
	
	adxl345_Write(*spi, ADXL345_THRESH_ACT, &activity_threshold, 1);

}

uint8_t adxl345_GetActivityThreshold(SPI_HandleTypeDef *spi) {

	uint8_t activity_threshold_val;
	
	adxl345_Read(*spi, ADXL345_THRESH_ACT, &activity_threshold_val, 1);
	
	return activity_threshold_val;

}

void adxl345_SetInactivityThreshold(SPI_HandleTypeDef *spi, uint8_t inactivity_threshold) {
	
	adxl345_Write(*spi, ADXL345_THRESH_INACT, &inactivity_threshold, 1);

}
	
uint8_t adxl345_GetInactivityThreshold(SPI_HandleTypeDef *spi) {

	uint8_t inactivity_threshold_val;
	
	adxl345_Read(*spi, ADXL345_THRESH_INACT, &inactivity_threshold_val, 1);
	
	return inactivity_threshold_val;

}

void adxl345_SetTimeInactivity(SPI_HandleTypeDef *spi, uint8_t inactivity_time) {
	
	adxl345_Write(*spi, ADXL345_TIME_INACT, &inactivity_time, 1);

}

uint8_t adxl345_GetTimeInactivity(SPI_HandleTypeDef *spi) {

	uint8_t inactivity_time_val;
	
	adxl345_Read(*spi, ADXL345_TIME_INACT, &inactivity_time_val, 1);
	
	return inactivity_time_val;

}

void adxl345_SetFreeFallThreshold(SPI_HandleTypeDef *spi, uint8_t free_fall_threshold) {

	adxl345_Write(*spi, ADXL345_THRESH_FF, &free_fall_threshold, 1);

}

uint8_t adxl345_GetFreeFallThreshold(SPI_HandleTypeDef *spi) {

	uint8_t free_fall_val;
	
	adxl345_Read(*spi, ADXL345_THRESH_FF, &free_fall_val, 1);
	
	return free_fall_val;

}

void adx345_SetFreeFallDuration(SPI_HandleTypeDef *spi, uint8_t free_fall_time) {

	adxl345_Write(*spi, ADXL345_TIME_FF, &free_fall_time, 1);

}

uint8_t adx345_GetFreeFallDuration(SPI_HandleTypeDef *spi) {

	uint8_t free_fall_duration_val;
	
	adxl345_Read(*spi, ADXL345_TIME_FF, &free_fall_duration_val, 1);
	
	return free_fall_duration_val;

}

uint8_t adxl345_IsActivityX_Enabled(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_X_EN);

}

uint8_t adxl345_IsActivityY_Enabled(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_Y_EN);

}

uint8_t adxl345_IsActivityZ_Enabled(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_Z_EN);

}

uint8_t adxl345_IsInactivityX_Enabled(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_X_EN);

}

uint8_t adxl345_IsInactivityY_Enabled(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Y_EN);

}

uint8_t adxl345_IsInactivityZ_Enabled(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Z_EN);

}

void adxl345_SetActivityX(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_X_EN, state);

}

void adxl345_SetActivityY(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_Y_EN, state);

}

void adxl345_SetActivityZ(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_Z_EN, state);

}

void adxl345_SetInactivityX(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_X_EN, state);

}

void adxl345_SetInactivitytY(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Y_EN	, state);

}

void adxl345_SetInActivityZ(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Z_EN, state);

}
 
void adxl345_SetActivityXYZ(SPI_HandleTypeDef *spi, uint8_t stateX, uint8_t stateY, uint8_t stateZ) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_X_EN, stateX);

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_Y_EN, stateY);

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_Z_EN, stateZ);	

}

void adxl345_SetInactivityXYZ(SPI_HandleTypeDef *spi, uint8_t stateX, uint8_t stateY, uint8_t stateZ) {
	
	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Z_EN, stateX);

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Z_EN, stateY);

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_Z_EN, stateZ);

}

uint8_t adxl345_IsActivityAc(SPI_HandleTypeDef *spi) {
	
	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_AC_DC);

}

uint8_t adxl345_IsInactivityAc(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_AC_DC);

}

void adxl345_SetActivityAc(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_ACT_AC_DC, state);

}

void adxl345_SetInactivityAc(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_ACT_INACT_CTL, ADXL345_ACT_INACT_CTL_INACT_AC_DC, state);

}

uint8_t adxl345_GetSuppressBit(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_SUPPRESS);

}

void adxl345_SetSuppressBit(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_SUPPRESS, state);

}

/**************************** TAP BITS ******************************/
/*                                                                  */
uint8_t adxl345_IsTapdetectionOnX(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_X_EN);

}

uint8_t adxl345_IsTapDetectionOnY(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_Y_EN);

}

uint8_t adxl345_IsTapDetectionOnZ(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_Z_EN);

}

void adxl345_SetTapDetectionOnX(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_X_EN, state);

}

void adxl345_SetTapDetectionOnY(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_Y_EN, state);
	
}

void adxl345_SetTapDetectionOnZ(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_Z_EN, state);

}

void adxl345_SetTapDetectionOnXYZ(SPI_HandleTypeDef *spi, uint8_t stateX, uint8_t stateY, uint8_t stateZ) {

	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_X_EN, stateX);
	
	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_Y_EN, stateY);
	
	adxl345_SetRegisterBit(spi, ADXL345_TAP_AXES, ADXL345_TAP_AXES_Z_EN, stateZ);

} 

uint8_t adxl345_IsActivitySourceOnX(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_ACT_X_SRC);

}

uint8_t adxl345_IsActivitySourceOnY(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_ACT_Y_SRC);

}

uint8_t adxl345_IsActivitySourceOnZ(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_ACT_Z_SRC);

}

uint8_t adxl345_IsTapSourceOnX(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_TAP_X_SRC);
	
}

uint8_t adxl345_IsTapSourceOnY(SPI_HandleTypeDef *spi) {


	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_TAP_Y_SRC);
	
}

uint8_t adxl345_IsTapSourceOnZ(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_TAP_Z_SRC);

}

uint8_t adxl345_IsAsleep(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_ACT_TAP_STATUS, ADXL345_ACT_TAP_ASLEEP);

}

uint8_t adxl345_GetLowPowerMode(SPI_HandleTypeDef *spi) {

	return adxl345_GetRegisterBit(spi, ADXL345_BW_RATE, ADXL345_BW_RATE_LOW_POWER);

}

void adxl345_SetLowPowerMode(SPI_HandleTypeDef *spi, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_BW_RATE, ADXL345_BW_RATE_LOW_POWER, state);

}

uint8_t adxl345_GetBwRate(SPI_HandleTypeDef *spi) {

	uint8_t 	bw_val;
	
	adxl345_Read( *spi, ADXL345_BW_RATE, &bw_val, 1 );
	
	bw_val = READ_BIT( bw_val, ADXL345_BW_RATE_MSK );
	
	return ( pow( 2, ( bw_val - 6 ) ) * 6.25 );

}

void adxl345_SetBwRate(SPI_HandleTypeDef *spi, double bw_rate) {

	uint8_t int_rate = bw_rate / 6.25;
	
	uint8_t bw_rate_count = 0;
	
	uint8_t bw_bit_val;
	
	while(int_rate >>= 1) {
	
		bw_rate_count++;
	
	}
	
	if(bw_rate_count >= 9) {
	
		adxl345_Read(*spi, ADXL345_BW_RATE, &bw_bit_val, 1);
		
		MODIFY_REG(bw_bit_val, ADXL345_BW_RATE_MSK, bw_rate_count);
		
		adxl345_Write(*spi, ADXL345_BW_RATE, &bw_bit_val, 1);
		
	}

}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
uint8_t adxl345_SetBw(SPI_HandleTypeDef *spi, uint8_t bw_val) {

	if( ( bw_val < ADXL345_BW_0_05 ) || ( bw_val > ADXL345_BW_1600 ) ) {
	
		return HAL_ERROR;
	
	} else {
	
		adxl345_Write(*spi, ADXL345_BW_RATE, &bw_val, 1);
	
		return HAL_OK;
		
	}

}

uint8_t adxl345_GetBW(SPI_HandleTypeDef *spi) {

	uint8_t bw_val;
	
	adxl345_Read(*spi, ADXL345_BW_RATE, &bw_val, 1);
	
	return bw_val;

}

uint8_t adxl345_Triggered(uint8_t interrupts, uint8_t mask) {

	return ( (interrupts >> mask) & 1 );

}

/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */

uint8_t adxl345_GetInterruptSource(SPI_HandleTypeDef *spi, uint8_t interrupt_bit) {

	return adxl345_GetRegisterBit(spi, ADXL345_INT_SOURCE, interrupt_bit);

}

uint8_t adxl345_GetInterruptMapping(SPI_HandleTypeDef *spi, uint8_t interrupt_bit) {

	return adxl345_GetRegisterBit(spi, ADXL345_INT_MAP, interrupt_bit);

}

void adxl345_SetInterruptMapping(SPI_HandleTypeDef *spi, uint8_t interrupt_bit, uint8_t interrupt_pin) {
	
	adxl345_SetRegisterBit(spi, ADXL345_INT_MAP, interrupt_bit, interrupt_pin);

}

void adxl345_SetImportantInterruptMapping(SPI_HandleTypeDef *spi, uint8_t single_tap, uint8_t double_tap, uint8_t free_fall, uint8_t activity, uint8_t inactivity) {
		
		if(single_tap == ADXL345_SEL_INT1_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(single_tap == ADXL345_SEL_INT2_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(double_tap == ADXL345_SEL_INT1_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(double_tap == ADXL345_SEL_INT2_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(free_fall == ADXL345_SEL_INT1_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT1_PIN );}
	else if(free_fall == ADXL345_SEL_INT2_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT2_PIN );}

	if(activity == ADXL345_SEL_INT1_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(activity == ADXL345_SEL_INT2_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT2_PIN );}

	if(inactivity == ADXL345_SEL_INT1_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(inactivity == ADXL345_SEL_INT2_PIN) {
		adxl345_SetInterruptMapping(spi, ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT2_PIN );}

}

uint8_t adxl345_IsInterruptEnabled(SPI_HandleTypeDef *spi, uint8_t interrupt_bit) {

	return adxl345_GetRegisterBit(spi, ADXL345_INT_ENABLE, interrupt_bit);
	
}

void adxl345_SetInterrupt(SPI_HandleTypeDef *spi, uint8_t interrupt_bit, uint8_t state) {

	adxl345_SetRegisterBit(spi, ADXL345_INT_ENABLE, interrupt_bit, state);

}

void adxl345_SingleTapINT(SPI_HandleTypeDef *spi, uint8_t status) {

	if(status)
		adxl345_SetInterrupt(spi, ADXL345_INT_SINGLE_TAP_BIT, SET);
	else
		adxl345_SetInterrupt(spi, ADXL345_INT_SINGLE_TAP_BIT, RESET);
	
}

void adxl345_DoubleTapINT(SPI_HandleTypeDef *spi, uint8_t status) {

	if(status)
		adxl345_SetInterrupt(spi, ADXL345_INT_DOUBLE_TAP_BIT, SET);
	else
		adxl345_SetInterrupt(spi, ADXL345_INT_DOUBLE_TAP_BIT, RESET);
	
}

void adxl345_FreeFallINT(SPI_HandleTypeDef *spi, uint8_t status) {

	if(status)
		adxl345_SetInterrupt(spi, ADXL345_INT_FREE_FALL_BIT, SET);
	else
		adxl345_SetInterrupt(spi, ADXL345_INT_FREE_FALL_BIT, RESET);
	
}

void adxl345_ActivityINT(SPI_HandleTypeDef *spi, uint8_t status) {

	if(status)
		adxl345_SetInterrupt(spi, ADXL345_INT_ACTIVITY_BIT , SET);
	else
		adxl345_SetInterrupt(spi, ADXL345_INT_ACTIVITY_BIT, RESET);
	
}

void adxl345_InactivityINT(SPI_HandleTypeDef *spi, uint8_t status) {

	if(status)
		adxl345_SetInterrupt(spi, ADXL345_INT_INACTIVITY_BIT , SET);
	else
		adxl345_SetInterrupt(spi, ADXL345_INT_INACTIVITY_BIT, RESET);
	
}

//Istenen registerin istenen bitini birlemek veya sifirlamak için kullanilan method
void adxl345_SetRegisterBit(SPI_HandleTypeDef *spi,  uint8_t reg, uint8_t bit_pos, uint8_t state) {

	uint8_t *reg_val;
	
	adxl345_Read(*spi, reg, reg_val, 1);
	
	if(state) {
	
		*reg_val |= (1 << bit_pos); // Forces nth Bit of reg_val to 1. Other Bits Unchanged.
	
	} else {
		
		*reg_val &= ~(1 << bit_pos); // Forces nth Bit of reg_val to 1. Other Bits Unchanged.
	
	}
	
	adxl345_Write(*spi, reg, reg_val, 1);

}

//Istenen registerin istenen bitinin durumunu ögrenmek için kullanilan method
uint8_t adxl345_GetRegisterBit(SPI_HandleTypeDef *spi,  uint8_t reg, uint8_t bit_pos) {

	uint8_t *reg_val;
	
	adxl345_Read(*spi, reg, reg_val, 1);
	
	return ((*reg_val >> bit_pos) & 1);

} 


