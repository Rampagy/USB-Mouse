#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "stm32f4xx.h"

/* LIS3DSH Commands */
#define LIS3DSH_READ_BIT 0x80
#define LIS3DSH_MULTI_BYTE 0x40 // read or write multiple bytes

/* LIS3DSH Registers */
#define WHO_AM_I_ADDR 0x0F
#define CTRL_REG4 0x20


uint8_t InitAccelerometer(void);

#endif // ACCELEROMETER_H