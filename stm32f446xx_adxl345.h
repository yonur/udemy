#ifndef __ADXL345_STM32F446XX_H
#define __ADXL345_STM32F446XX_H

#include "stm32f4xx_hal.h"
#include <stdint.h> 
#include <math.h>

/**********************************Macro Functions***********************************************/
#define CONSTRAIN(VAL, LOW, HIGH) ( ( VAL < LOW ) ? LOW : ( ( VAL > HIGH ) ? HIGH : VAL ) )

/***************************************Registers************************************************/
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

/*************************Register Areas and Checks******************************/
#define ADXL345_RANGE_2G		0x00
#define ADXL345_RANGE_4G		0x01
#define ADXL345_RANGE_8G		0x02
#define ADXL345_RANGE_16G		0x03

#define ADXL345_DATA_FORMAT_SELF_TEST_BIT  0x07
#define ADXL345_DATA_FORMAT_SPI_BIT				 0x06
#define ADXL345_DATA_FORMAT_INT_INVERT_BIT 0x05
#define ADXL345_DATA_FORMAT_FULL_RES_BIT	 0x03
#define ADXL345_DATA_FORMAT_JUSTIFY_BIT		 0x02

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

/********* ACTIVATION INACTIVATION CONTROL BIT POSITIONS **************/
#define ADXL345_ACT_INACT_CTL_ACT_AC_DC 								0x07
#define ADXL345_ACT_INACT_CTL_INACT_AC_DC 							0x03
#define ADXL345_ACT_INACT_CTL_ACT_X_EN									0x06
#define ADXL345_ACT_INACT_CTL_ACT_Y_EN									0x05
#define ADXL345_ACT_INACT_CTL_ACT_Z_EN									0x04
#define ADXL345_ACT_INACT_CTL_INACT_X_EN								0x02
#define ADXL345_ACT_INACT_CTL_INACT_Y_EN								0x01
#define ADXL345_ACT_INACT_CTL_INACT_Z_EN								0x00

/******************** TAP_AXES BIT POSITIONS *************************/
#define ADXL345_TAP_AXES_SUPPRESS												0x03
#define ADXL345_TAP_AXES_X_EN														0x02
#define ADXL345_TAP_AXES_Y_EN														0x01
#define ADXL345_TAP_AXES_Z_EN														0x00

/******************* ACT_TAP_STATUS BIT POSITIONS *******************/
#define ADXL345_ACT_TAP_ACT_X_SRC												0x06
#define ADXL345_ACT_TAP_ACT_Y_SRC												0x05
#define ADXL345_ACT_TAP_ACT_Z_SRC												0x04
#define ADXL345_ACT_TAP_ASLEEP													0x03
#define ADXL345_ACT_TAP_TAP_X_SRC												0x02
#define ADXL345_ACT_TAP_TAP_Y_SRC												0x01
#define ADXL345_ACT_TAP_TAP_Z_SRC												0x00

/******************** BW_RATE BIT POSITIONS *************************/
#define ADXL345_BW_RATE_LOW_POWER												0x04
#define ADXL345_BW_RATE_MSK												0x0F

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


/******************************* ERRORS ******************************/
#define ADXL345_OK			1		// No Error
#define ADXL345_ERROR		0		// Error Exists

#define ADXL345_NO_ERROR	0		// Initial State
#define ADXL345_READ_ERROR	1		// Accelerometer Reading Error
#define ADXL345_BAD_ARG		2		// Bad Argument

//Action bits
#define LOW_POWER      0x10

/****************************Variables**********************************************************/
uint8_t rec_buf[6];
double gains[3] = {0.00376390, 0.00376009, 0.00349265};

/***************************Functions***********************************************************/

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

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
//To get the justify bit state
uint8_t adxl345_GetJustifyBitState(SPI_HandleTypeDef *spi);

//To set the justify bit state
void adxl345_SetJustifyBitState(SPI_HandleTypeDef *spi, uint8_t justify_bit_control);

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
void adxl345_SetTapThreshold(SPI_HandleTypeDef *spi, uint8_t tap_threshold);

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
uint8_t adxl345_GetTapThreshold(SPI_HandleTypeDef *spi);

/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */

//To set axis gain
void adxl345_SetAxisGains(double *_gains);

//to get axis gain
void adxl345_GetAxisGains(double *_gains);

/********************* OFSX, OFSY and OFSZ BYTES ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 15.6mg/LSB
void adxl345_GetAxisOffset(SPI_HandleTypeDef *spi, uint8_t *x, uint8_t *y, uint8_t *z);

void adxl345_SetAxisOffset(SPI_HandleTypeDef *spi, uint8_t x, uint8_t y, uint8_t z);

/****************************** DUR BYTE ****************************/
/*                           ~ SET & GET                            */
// DUR Byte: Contains an Unsigned Time Value Representing the Max Time 
//  that an Event must be Above the THRESH_TAP Threshold to qualify 
//  as a Tap Event
// The scale factor is 625µs/LSB
// Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
void adxl345_SetTapDuration(SPI_HandleTypeDef *spi, uint8_t tap_duration);

uint8_t adxl345_GetTapDuration(SPI_HandleTypeDef *spi);

/************************** LATENT REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains Unsigned Time Value Representing the Wait Time from the Detection
//  of a Tap Event to the Start of the Time Window (defined by the Window 
//  Register) during which a possible Second Tap Even can be Detected.
// Scale Factor is 1.25ms/LSB. 
// A Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void adxl345_SetDoubleTapLatency(SPI_HandleTypeDef *spi, uint8_t double_tap_latency);

uint8_t adxl345_GetDoubleTapLatency(SPI_HandleTypeDef *spi);

/************************** WINDOW REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains an Unsigned Time Value Representing the Amount of Time 
//  After the Expiration of the Latency Time (determined by Latent register)
//  During which a Second Valid Tape can Begin. 
// Scale Factor is 1.25ms/LSB. 
// Value of 0 Disables the Double Tap Function. 
// It Accepts a Maximum Value of 255.
void adxl345_SetDoubleTapWindow(SPI_HandleTypeDef *spi, uint8_t double_tap_window);

uint8_t adxl34_GetDoubleTapWindow(SPI_HandleTypeDef *spi);

/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared 
//  with the Value is Compared with the Value in the THRESH_ACT Register. 
// The Scale Factor is 62.5mg/LSB. 
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
void adxl345_SetActivityThreshold(SPI_HandleTypeDef *spi, uint8_t activity_threshold);

uint8_t adxl345_GetActivityThreshold(SPI_HandleTypeDef *spi);

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is 
//  Compared with the value in the THRESH_INACT Register. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
void adxl345_SetInactivityThreshold(SPI_HandleTypeDef *spi, uint8_t inactivity_threshold);

uint8_t adxl345_GetInactivityThreshold(SPI_HandleTypeDef *spi);

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared. 
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB. 
// Value Must Be Between 0 and 255.
void adxl345_SetTimeInactivity(SPI_HandleTypeDef *spi, uint8_t inactivity_time);

uint8_t adxl345_GetTimeInactivity(SPI_HandleTypeDef *spi);

/*********************** THRESH_FF Register *************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// The Acceleration on all Axes is Compared with the Value in THRES_FF to
//  Determine if a Free-Fall Event Occurred. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// Accepts a Maximum Value of 255.
void adxl345_SetFreeFallThreshold(SPI_HandleTypeDef *spi, uint8_t free_fall_threshold);

uint8_t adxl345_GetFreeFallThreshold(SPI_HandleTypeDef *spi);

/************************ TIME_FF Register **************************/
/*                          ~ SET & GET                             */
// Stores an Unsigned Time Value Representing the Minimum Time that the Value 
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
void adx345_SetFreeFallDuration(SPI_HandleTypeDef *spi, uint8_t free_fall_time);

uint8_t adx345_GetFreeFallDuration(SPI_HandleTypeDef *spi);

/************************** ACTIVITY BITS ***************************/
/*                                                                  */
uint8_t adxl345_IsActivityX_Enabled(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsActivityY_Enabled(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsActivityZ_Enabled(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsInactivityX_Enabled(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsInactivityY_Enabled(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsInactivityZ_Enabled(SPI_HandleTypeDef *spi);

void adxl345_SetActivityX(SPI_HandleTypeDef *spi, uint8_t state);

void adxl345_SetActivityY(SPI_HandleTypeDef *spi, uint8_t state); 
	
void adxl345_SetActivityZ(SPI_HandleTypeDef *spi, uint8_t state); 
	
void adxl345_SetInactivityX(SPI_HandleTypeDef *spi, uint8_t state); 
	
void adxl345_SetInactivityY(SPI_HandleTypeDef *spi, uint8_t state);
	
void adxl345_SetInactivityZ(SPI_HandleTypeDef *spi, uint8_t state);

void adxl345_SetActivityXYZ(SPI_HandleTypeDef *spi, uint8_t stateX, uint8_t stateY, uint8_t stateZ);

void adxl345_SetInactivityXYZ(SPI_HandleTypeDef *spi, uint8_t stateX, uint8_t stateY, uint8_t stateZ);

uint8_t adxl345_IsActivityAc(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsInactivityAc(SPI_HandleTypeDef *spi);

void adxl345_SetActivityAc(SPI_HandleTypeDef *spi, uint8_t state);

void adxl345_SetInactivityAc(SPI_HandleTypeDef *spi, uint8_t state);

/************************* SUPPRESS BITS ****************************/
/*                                                                  */
uint8_t adxl345_GetSuppressBit(SPI_HandleTypeDef *spi);

void adxl345_SetSuppressBit(SPI_HandleTypeDef *spi, uint8_t state);

/**************************** TAP BITS ******************************/
/*                                                                  */
uint8_t adxl345_IsTapdetectionOnX(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsTapDetectionOnY(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsTapDetectionOnZ(SPI_HandleTypeDef *spi);

void adxl345_SetTapDetectionOnX(SPI_HandleTypeDef *spi, uint8_t state);

void adxl345_SetTapDetectionOnY(SPI_HandleTypeDef *spi, uint8_t state);

void adxl345_SetTapDetectionOnZ(SPI_HandleTypeDef *spi, uint8_t state);

void adxl345_SetTapDetectionOnXYZ(SPI_HandleTypeDef *spi, uint8_t stateX, uint8_t stateY, uint8_t stateZ);

uint8_t adxl345_IsActivitySourceOnX(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsActivitySourceOnY(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsActivitySourceOnZ(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsTapSourceOnX(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsTapSourceOnY(SPI_HandleTypeDef *spi);

uint8_t adxl345_IsTapSourceOnZ(SPI_HandleTypeDef *spi);

/*************************** ASLEEP BIT *****************************/
/*                                                                  */
uint8_t adxl345_IsAsleep(SPI_HandleTypeDef *spi);

/************************** LOW POWER BIT ***************************/
/*                                                                  */
uint8_t adxl345_GetLowPowerMode(SPI_HandleTypeDef *spi);

void adxl345_SetLowerPowerMode(SPI_HandleTypeDef *spi, uint8_t state);

/*************************** RATE BITS ******************************/
/*                                                                  */


#endif
