#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "uart.h"

/* Butterworth Parameters */
#define Y_COEFFS (2)
#define X_COEFFS (3)

#define ACCEL_BUFFER_SIZE (16U)

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

#define MULTIBYTE_ACCEL_READ_LEN (6U)

/* state machine for the SPI interrupt */
typedef enum
{
  start_spi, /* start */
  t0,        /* transmit byte 0 */
  r0,        /* receive byte 0 */
  t1,        /* transmit byte 1 */
  r1,        /* and so on... */
  t2,
  r2,
  t3,
  r3,
  t4,
  r4,
  t5,
  r5,
  t6,
  r6,
  end_spi,
} SpiStateMachine_t;

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
} accel_data_t;

uint8_t InitAccelerometer(void);
void EXTI0_IRQHandler(void);
void SPI1_IRQHandler(void);

/* Retreives acceleration data via interrupts */
void GetAccelerationData(acceleration_t *accel);

/* Retreives acceleration data via polling/blocking */
void ReadAcceleration(acceleration_t *accel);

/* global variables */
extern uint32_t SPI_comms;
extern uint32_t SPI_missed;
extern uint32_t SPI_failed;

#endif // ACCELEROMETER_H