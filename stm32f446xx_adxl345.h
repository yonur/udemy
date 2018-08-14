#ifndef __ADXL345_H
#define __ADXL345_H

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

//Action bits
#define LOW_POWER      0x10

#include "stm32f4xx_hal.h"
#include <stdint.h> 

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


#endif
