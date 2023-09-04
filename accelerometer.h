#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdio.h>

#include "stm32f4xx.h"
#include "uart.h"

/* SPI buffer size (Rx and Tx are the same size) */
#define SPI_MAX_BUFFER_LEN (16)

/* LIS3DSH Commands */
#define LIS3DSH_READ_BIT (0x80)
#define LIS3DSH_MULTI_BYTE (0x40) // read or write multiple bytes

/* LIS3DSH Registers */
#define INFO1_ADDR (0x0D)
#define INFO2_ADDR (0x0E)
#define WHO_AM_I_ADDR (0x0F)
#define OFFSET_X_ADDR (0x10)
#define OFFSET_Y_ADDR (0x11)
#define OFFSET_Z_ADDR (0x12)
#define STAT_ADDR (0x18)
#define CTRL_REG4 (0x20)
#define CTRL_REG1 (0x21)
#define CTRL_REG2 (0x22)
#define CTRL_REG3 (0x23)
#define CTRL_REG5 (0x24)
#define CTRL_REG6 (0x25)
#define STATUS_ADDR (0x27)
#define OUT_X_ACCEL_L (0x28)
#define OUT_X_ACCEL_H (0x29)
#define OUT_Y_ACCEL_L (0x2A)
#define OUT_Y_ACCEL_H (0x2B)
#define OUT_Z_ACCEL_L (0x2C)
#define OUT_Z_ACCEL_H (0x2D)
#define FIFO_CTRL_ADDR (0x2E)
#define FIFO_SRC_ADDR (0x2F)

#define MULTIBYTE_ACCEL_READ_LEN (6)

/* SPI send data return code. */
typedef enum
{
  SPI_TX_NO_ERROR,
  /* Tx buffer would be full if the data was added OR
   * tx buffer is already full. Try again later.
   * Can also increase TX buffer size.
   */
  SPI_TX_BUFFER_FULL,
} SPIResponseCode_t;

typedef enum
{
  BYPASS_MODE = 0x00,
  FIFO_MODE = 0x20,
  STREAM_MODE = 0x40
} AccelFIFOMode_t;

typedef struct
{
  float x, y, z;
} acceleration_t;

typedef union
{
  int16_t s16[MULTIBYTE_ACCEL_READ_LEN / 2];
  uint16_t u16[MULTIBYTE_ACCEL_READ_LEN / 2];
  uint8_t u8[MULTIBYTE_ACCEL_READ_LEN];
  int8_t s8[MULTIBYTE_ACCEL_READ_LEN];
  char c8[MULTIBYTE_ACCEL_READ_LEN];
} accel_data;

uint8_t InitAccelerometer(void);
void ReadAcceleration(acceleration_t *accel);
void EXTI1_IRQHandler(void);
void SPI1_IRQHandler(void);

#endif // ACCELEROMETER_H