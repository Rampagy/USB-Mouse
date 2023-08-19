#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


#include "stm32f4xx.h"


/* LIS3DSH Commands */
#define LIS3DSH_READ_BIT 0x80
#define LIS3DSH_MULTI_BYTE 0x40 // read or write multiple bytes

/* LIS3DSH Registers */
#define WHO_AM_I_ADDR (0x0F)
#define CTRL_REG3 (0x23)
#define CTRL_REG4 (0x20)
#define CTRL_REG5 (0x24)
#define OUT_X_ACCEL (0x28)
#define OUT_Y_ACCEL (0x2A)
#define OUT_Z_ACCEL (0x2C)

#define MULTIBYTE_ACCEL_READ_LEN (6)


typedef struct {
  float x, y, z;
} acceleration_t;

typedef union {
  uint16_t u16[MULTIBYTE_ACCEL_READ_LEN >> 1];
  uint8_t u8[MULTIBYTE_ACCEL_READ_LEN];
} accel_data;


uint8_t InitAccelerometer(void);
void ReadAcceleration(acceleration_t* accel);


#endif // ACCELEROMETER_H