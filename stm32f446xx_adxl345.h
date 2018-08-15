#ifndef ASDXL345_STM32F446XX_H
#define ASDXL345_STM32F446XX_H

#include "stm32f4xx_hal.h"
#include <stdint.h> 

#define DEVID          0x00  //R   11100101  Device ID 
#define THRESH_TAP     0x1D  //R/W 00000000  Tap threshold 
#define OFSX           0x1E  //R/W 00000000  X-axis offset 
#define OFSY           0x1F  //R/W 00000000  Y-axis offset 
#define OFSZ           0x20  //R/W 00000000  Z-axis offset 
#define DUR            0x21  //R/W 00000000  Tap duration 
#define Latent         0x22  //R/W 00000000  Tap latency 
#define Window         0x23  //R/W 00000000  Tap window 
#define THRESH_ACT     0x24  //R/W 00000000  Activity threshold 
#define THRESH_INACT   0x25  //R/W 00000000  Inactivity threshold 
#define TIME_INACT     0x26  //R/W 00000000  Inactivity time 
#define ACT_INACT_CTL  0x27  //R/W 00000000  Axis enable control for activity and inactivity detection 
#define THRESH_FF      0x28  //R/W 00000000  Free-fall threshold 
#define TIME_FF        0x29  //R/W 00000000  Free-fall time 
#define TAP_AXES       0x2A  //R/W 00000000  Axis control for single tap/double tap 
#define ACT_TAP_STATUS 0x2B  //R   00000000  Source of single tap/double tap 
#define BW_RATE        0x2C  //R/W 00001010  Data rate and power mode control 
#define POWER_CTL      0x2D  //R/W 00000000  Power-saving features control 
#define INT_ENABLE     0x2E  //R/W 00000000  Interrupt enable control 
#define INT_MAP        0x2F  //R/W 00000000  Interrupt mapping control 
#define INT_SOURCE     0x30  //R   00000010  Source of interrupts 
#define DATA_FORMAT    0x31  //R/W 00000000  Data format control 
#define DATAX0         0x32  //R   00000000  X-Axis Data 0 
#define DATAX1         0x33  //R   00000000  X-Axis Data 1 
#define DATAY0         0x34  //R   00000000  Y-Axis Data 0 
#define DATAY1         0x35  //R   00000000  Y-Axis Data 1 
#define DATAZ0         0x36  //R   00000000  Z-Axis Data 0 
#define DATAZ1         0x37  //R   00000000  Z-Axis Data 1 
#define FIFO_CTL       0x38  //R/W 00000000  FIFO control 
#define FIFO_STATUS    0x39  //R   00000000  FIFO status 

/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID			0x00		// Device ID
#define ADXL345_RESERVED1		0x01		// Reserved. Do Not Access. 
#define ADXL345_THRESH_TAP		0x1D		// Tap Threshold. 
#define ADXL345_OFSX			0x1E		// X-Axis Offset. 
#define ADXL345_OFSY			0x1F		// Y-Axis Offset.
#define ADXL345_OFSZ			0x20		// Z- Axis Offset.
#define ADXL345_DUR				0x21		// Tap Duration.
#define ADXL345_LATENT			0x22		// Tap Latency.
#define ADXL345_WINDOW			0x23		// Tap Window.
#define ADXL345_THRESH_ACT		0x24		// Activity Threshold
#define ADXL345_THRESH_INACT	0x25		// Inactivity Threshold
#define ADXL345_TIME_INACT		0x26		// Inactivity Time
#define ADXL345_ACT_INACT_CTL	0x27		// Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF		0x28		// Free-Fall Threshold.
#define ADXL345_TIME_FF			0x29		// Free-Fall Time.
#define ADXL345_TAP_AXES		0x2A		// Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS	0x2B		// Source of Tap/Double Tap
#define ADXL345_BW_RATE			0x2C		// Data Rate and Power mode Control
#define ADXL345_POWER_CTL		0x2D		// Power-Saving Features Control
#define ADXL345_INT_ENABLE		0x2E		// Interrupt Enable Control
#define ADXL345_INT_MAP			0x2F		// Interrupt Mapping Control
#define ADXL345_INT_SOURCE		0x30		// Source of Interrupts
#define ADXL345_DATA_FORMAT		0x31		// Data Format Control
#define ADXL345_DATAX0			0x32		// X-Axis Data 0
#define ADXL345_DATAX1			0x33		// X-Axis Data 1
#define ADXL345_DATAY0			0x34		// Y-Axis Data 0
#define ADXL345_DATAY1			0x35		// Y-Axis Data 1
#define ADXL345_DATAZ0			0x36		// Z-Axis Data 0
#define ADXL345_DATAZ1			0x37		// Z-Axis Data 1
#define ADXL345_FIFO_CTL		0x38		// FIFO Control
#define ADXL345_FIFO_STATUS		0x39		// FIFO Status

#define ADXL345_BW_1600			0xF			// 1111		IDD = 40uA
#define ADXL345_BW_800			0xE			// 1110		IDD = 90uA
#define ADXL345_BW_400			0xD			// 1101		IDD = 140uA
#define ADXL345_BW_200			0xC			// 1100		IDD = 140uA
#define ADXL345_BW_100			0xB			// 1011		IDD = 140uA 
#define ADXL345_BW_50			0xA			// 1010		IDD = 140uA
#define ADXL345_BW_25			0x9			// 1001		IDD = 90uA
#define ADXL345_BW_12_5		    0x8			// 1000		IDD = 60uA 
#define ADXL345_BW_6_25			0x7			// 0111		IDD = 50uA
#define ADXL345_BW_3_13			0x6			// 0110		IDD = 45uA
#define ADXL345_BW_1_56			0x5			// 0101		IDD = 40uA
#define ADXL345_BW_0_78			0x4			// 0100		IDD = 34uA
#define ADXL345_BW_0_39			0x3			// 0011		IDD = 23uA
#define ADXL345_BW_0_20			0x2			// 0010		IDD = 23uA
#define ADXL345_BW_0_10			0x1			// 0001		IDD = 23uA
#define ADXL345_BW_0_05			0x0			// 0000		IDD = 23uA

#define ADXL345_RANGE_2G		0x00
#define ADXL345_RANGE_4G		0x01
#define ADXL345_RANGE_8G		0x02
#define ADXL345_RANGE_16G		0x03

#define ADXL345_DATA_FORMAT_SELF_TEST_BIT  0x80
#define ADXL345_DATA_FORMAT_SPI_BIT				 0x40
#define ADXL345_DATA_FORMAT_INT_INVERT_BIT 0x20
#define ADXL345_DATA_FORMAT_FULL_RES_BIT	 0x08
#define ADXL345_DATA_FORMAT_JUSTIFY_BIT		 0x04

#define ADXL345_DATA_FORMAT_ENABLE_SELF_TEST_FORCE  0x01
#define ADXL345_DATA_FORMAT_DISABLE_SELF_TEST_FORCE 0x00
#define ADXL345_DATA_FORMAT_SPI_3_WIRE							0x01
#define ADXL345_DATA_FORMAT_SPI_4_WIRE							0x00
#define ADXL345_DATA_FORMAT_INT_INVERT_ACTIVE_LOW   0x01
#define ADXL345_DATA_FORMAT_INT_INVERT_ACTIVE_HIGH  0x00
#define ADXL345_DATA_FORMAT_FULL_RES								0x01
#define ADXL345_DATA_FORMAT_10_BIT_MODE							0x00
#define ADXL345_DATA_FORMAT_JUSTIFY_MSB							0x01
#define ADXL345_DATA_FORMAT_JUSTIFY_LSB							0x00

 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN		0x00		//INT1: 0
#define ADXL345_INT2_PIN		0x01		//INT2: 1

/********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT		0x07
#define ADXL345_INT_SINGLE_TAP_BIT		0x06
#define ADXL345_INT_DOUBLE_TAP_BIT		0x05
#define ADXL345_INT_ACTIVITY_BIT		0x04
#define ADXL345_INT_INACTIVITY_BIT		0x03
#define ADXL345_INT_FREE_FALL_BIT		0x02
#define ADXL345_INT_WATERMARK_BIT		0x01
#define ADXL345_INT_OVERRUNY_BIT		0x00

#define ADXL345_DATA_READY				0x07
#define ADXL345_SINGLE_TAP				0x06
#define ADXL345_DOUBLE_TAP				0x05
#define ADXL345_ACTIVITY				0x04
#define ADXL345_INACTIVITY				0x03
#define ADXL345_FREE_FALL				0x02
#define ADXL345_WATERMARK				0x01
#define ADXL345_OVERRUNY				0x00


 /****************************** ERRORS ******************************/
#define ADXL345_OK			1		// No Error
#define ADXL345_ERROR		0		// Error Exists

#define ADXL345_NO_ERROR	0		// Initial State
#define ADXL345_READ_ERROR	1		// Accelerometer Reading Error
#define ADXL345_BAD_ARG		2		// Bad Argument

//Action bits
#define LOW_POWER      0x10

#define CONSTRAIN(VAL, LOW, HIGH) ( ( VAL < LOW ) ? LOW : ( ( VAL > HIGH ) ? HIGH : VAL ) )

uint8_t rec_buf[6];

//Set the LOW_POWER bit in the BW_RATE register
void adxl345_LowPowerMode(SPI_HandleTypeDef spi);

//Clear the Measure bit in the POWER_CTL register
void adxl345_StandbyMode(SPI_HandleTypeDef spi);

//If clear SPI bit in the DATA_FORMAT register -> 4-wire mode
//If set SPI bit in the DATA_FORMAT register -> 3-wire mode
void adxl345_SetSpi_Mode(void);

//To get device id
void adxl345_devID(SPI_HandleTypeDef spi);
	
void adxl345_Write(SPI_HandleTypeDef spi, uint8_t reg, uint8_t *value, uint8_t data_len);

void adxl345_Read(SPI_HandleTypeDef spi, uint8_t reg, uint8_t *data_buf, uint8_t data_len);

//ADXL345 güç ayarlari
void adxl345_PowerOn(SPI_HandleTypeDef spi);

//Accelerasyon degerini okumak için
void adxl345_ReadAcceleration(SPI_HandleTypeDef spi, uint8_t * x, uint8_t *y, uint8_t *z); 

//Istenen registerin istenen bitini birlemek veya sifirlamak için kullanilan method
void adxl345_SetRegisterBit(SPI_HandleTypeDef *spi, uint8_t reg, uint8_t bit_pos, uint8_t state);

//Istenen registerin istenen bitinin durumunu ögrenmek için kullanilan method
uint8_t adxl345_GetRegisterBit(SPI_HandleTypeDef *spi,  uint8_t reg, uint8_t bit_pos);

//Interrupt source'u ögrenmek için
uint8_t adxl345_GetInterruptSource(SPI_HandleTypeDef *spi, uint8_t interruptBitPos);

/**********************INTERRUPT MAPPING************************************/
//Interrupt mapping'i pin 1 yada pin 2'ye set etmek için kullanilir
void adxl345_SetInterruptMapping(SPI_HandleTypeDef *spi, uint8_t interrupt_bit, uint8_t interrupt_pin);

//Interrupt mapping'teki interruptlari tek tek pin 1 yada pin 2'ye seçmek için kullanilir.
void adxl345_SetImportantInterruptMapping(SPI_HandleTypeDef *spi, uint8_t single_tap, uint8_t double_tap, uint8_t free_fall, uint8_t activity, uint8_t inactivity);

//To get the g range
void adxl345_GetRangeSettings(SPI_HandleTypeDef *spi, uint8_t *range_settings);

//To set the g range
void adxl345_SetRangeSettings(SPI_HandleTypeDef *spi, uint8_t val);

//To get the self test bit in the DATA_FORMAT register
uint8_t adxl345_GetSelfTestBit(SPI_HandleTypeDef *spi);

//To control self test
void adxl345_SetSelfTestBit(SPI_HandleTypeDef *spi, uint8_t self_testbit_control);

//To get SPI state (3-Wire or 4-Wire)
uint8_t adxl345_GetSPI_State(SPI_HandleTypeDef *spi);

//To set SPI state (3-Wire or 4-Wire)
void adxl345_SetSPIState(SPI_HandleTypeDef *spi, uint8_t spi_state_control);

//To get the INT_INVERT state
uint8_t adxl345_GetIntInvertState(SPI_HandleTypeDef *spi);

//To set the INT_INVERT state
void adxl345_SetIntInvertState(SPI_HandleTypeDef *spi, uint8_t int_invert_control);

//To get the full resolution bit state
uint8_t adxl345_GetFullResBitState(SPI_HandleTypeDef *spi);

//To set the full resolution bit state
void adxl345_SetFullResBitState(SPI_HandleTypeDef *spi, uint8_t full_res_bit_control);

//To get the justify bit state
uint8_t adxl345_GetJustifyBitState(SPI_HandleTypeDef *spi);

//To set the justify bit state
void adxl345_SetJustifyBitState(SPI_HandleTypeDef *spi, uint8_t justify_bit_control);

// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
void adxl345_SetTapThreshold(SPI_HandleTypeDef *spi, uint8_t tap_threshold);

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
uint8_t adxl345_GetTapThreshold(SPI_HandleTypeDef *spi);

#endif
