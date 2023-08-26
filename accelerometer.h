#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdio.h>

#include "stm32f4xx.h"
#include "uart.h"

/* LIS3DSH Commands */
#define LIS3DSH_READ_BIT (0x80)
#define LIS3DSH_MULTI_BYTE (0x40) // read or write multiple bytes

/* LIS3DSH Registers */
#define INFO1_ADDR (0x0D)
#define INFO2_ADDR (0x0E)
#define WHO_AM_I_ADDR (0x0F)
#define STAT_ADDR (0x18)
#define CTRL_REG4 (0x20)
#define CTRL_REG1 (0x21)
#define CTRL_REG2 (0x22)
#define CTRL_REG3 (0x23)
#define CTRL_REG5 (0x24)
#define CTRL_REG6 (0x25)
#define STATUS_ADDR (0x27)
#define OUT_X_ACCEL (0x28)
#define OUT_Y_ACCEL (0x2A)
#define OUT_Z_ACCEL (0x2C)
#define FIFO_CTRL_ADDR (0x2E)
#define FIFO_SRC_ADDR (0x2F)

#define MULTIBYTE_ACCEL_READ_LEN (6)

typedef enum
{
  BYPASS_MODE = 0x00,
  FIFO_MODE = 0x20,
  STREAM_MODE = 0x40
} AccelFIFOMode_t;

typedef struct
{
  int16_t x, y, z;
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
/*
void ReadTemperature(uint8_t* temp);
void ReadStatReg(uint8_t* status);
void ReadStatusReg(uint8_t* status);
void ReadOutsReg(uint8_t* status);
*/

#endif // ACCELEROMETER_H