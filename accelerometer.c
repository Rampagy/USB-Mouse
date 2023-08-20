#include "accelerometer.h"


static uint8_t SPI1_SendByte(uint8_t byte);
void SPI1_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void SPI1_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/*
void ReadOutsReg(uint8_t* status)
{
  SPI1_Read(&(*status), OUTS1_ADDR, 1);
}
*/

void ReadStatusReg(uint8_t* status)
{
  SPI1_Read(&(*status), STATUS_ADDR, 1);
}

void ReadFIFOStatusReg(uint8_t* status)
{
  SPI1_Read(&(*status), FIFO_SRC_ADDR, 1);
}

/*
void ReadStatReg(uint8_t* status)
{
  SPI1_Read(&(*status), STAT_ADDR, 1);
}


void ReadTemperature(uint8_t* temp)
{
  SPI1_Read(&(*temp), TEMPERATURE_ADDR, 1);
}
*/

void ReadAcceleration(acceleration_t* accel)
{
  accel_data reg;
  uint8_t status0 = 0;
  uint8_t status1 = 0;

  /* Wait for new data to arrive before reading accel data */
  //while ((status0 & 0x80) == 0x00 && (status0 & 0x40) == 0x00 && (status0 & 0x1F) < 1 && status1 == 0x00)
  //{
  //  ReadFIFOStatusReg(&status0);
  //  ReadStatusReg(&status1);
  //}

  // make sure we read at least one x, y, and z
  //if ((status0 & 0x1F) == 0)
  //{
  //  status0 = 1;
  //}

  /* Clear out the FIFO buffer */
  //while ((status0 & 0x1F) > 0)
  {
    /* Read 6 bytes: x acceleration, y acceleration, and z acceleration. */
    SPI1_Read(&reg.u8[0], OUT_X_ACCEL, MULTIBYTE_ACCEL_READ_LEN);
    //status0 = (status0 & 0x1F) - 1;
  }

  //status1 = BYPASS_MODE | 0x0A;
  //SPI1_Write(&status1, FIFO_CTRL_ADDR, 1);

  /* Enable FIFO mode*/
  //status1 = 0x70;
  //SPI1_Write(&status1, CTRL_REG6, 1);

  //status1 = STREAM_MODE | 0x0A;
  //SPI1_Write(&status1, FIFO_CTRL_ADDR, 1);

  /* Parse the results into floats with units of g */
  //int16_t x = (int16_t)(((uint16_t)reg.u8[1] << 8) | (uint16_t)reg.u8[0]);
  //int16_t y = (int16_t)(((uint16_t)reg.u8[3] << 8) | (uint16_t)reg.u8[2]);
  //int16_t z = (int16_t)(((uint16_t)reg.u8[5] << 8) | (uint16_t)reg.u8[4]);

  //accel->x = (float)(x) / 32767.5f;
  //accel->y = (float)(y) / 32767.5f;
  //accel->z = (float)(z) / 32767.5f;

  accel->x = ((int16_t)reg.s8[1] << 8) | (int16_t)reg.s8[0];
  accel->y = ((int16_t)reg.s8[3] << 8) | (int16_t)reg.s8[2];
  accel->z = ((int16_t)reg.s8[5] << 8) | (int16_t)reg.s8[4];
}


uint8_t InitAccelerometer(void)
{
  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 100000; ++i);

  uint8_t tmpreg = 0x67; // 100Hz, continuous update, x, y, z enabled
  SPI1_Write(&tmpreg, CTRL_REG4, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 100000; ++i);

  /* 200 hz bandwidth filter */
  tmpreg = 0x40;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 100000; ++i);

  /* Read accel */
  acceleration_t accels_baseline;
  ReadAcceleration(&accels_baseline);

  /* Positive self test */
  tmpreg = 0x42;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 100000; ++i);

  acceleration_t accels_positive;
  ReadAcceleration(&accels_positive);

  /* Negative self test */
  tmpreg = 0x44;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 100000; ++i);

  acceleration_t accels_negative;
  ReadAcceleration(&accels_negative);

  /* 200 hz bandwidth filter, self test is off */
  tmpreg = 0x40;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  //tmpreg = 0x70;
  //SPI1_Write(&tmpreg, CTRL_REG6, 1);

  //tmpreg = STREAM_MODE | 0x0A;
  //SPI1_Write(&tmpreg, FIFO_CTRL_ADDR, 1);

  /* Read the WHO_AM_I reg to verify functionality */
  uint8_t who_am_i = 0;
  SPI1_Read(&who_am_i, WHO_AM_I_ADDR, 1);

  uint8_t init_success_flag = 0;
  if ((who_am_i == 0x3F) &&
      (0.06f*(accels_positive.x - accels_baseline.x) > 50.0f) &&
      (0.06f*(accels_positive.y - accels_baseline.y) > 50.0f) &&
      (0.06f*(accels_positive.z - accels_baseline.z) > 200.0f) &&
      (0.06f*(accels_negative.x - accels_baseline.x) < -50.0f) &&
      (0.06f*(accels_negative.y - accels_baseline.y) < -50.0f) &&
      (0.06f*(accels_negative.z - accels_baseline.z) < -200.0f))
  {
    init_success_flag = 1;
  }

  return init_success_flag;
}


void SPI1_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
     - When 0, the address will remain unchanged in multiple read/write commands.
     - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)LIS3DSH_MULTI_BYTE;
  }

  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  
  /* Send the Address of the indexed register */
  (void)SPI1_SendByte(WriteAddr);

  /* Send the data that will be written into the device (MSB First) */
  while (NumByteToWrite > 0x00)
  {
    (void)SPI1_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}


void SPI1_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(LIS3DSH_READ_BIT | LIS3DSH_MULTI_BYTE);
  }
  else
  {
    ReadAddr |= (uint8_t)LIS3DSH_READ_BIT;
  }

  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  
  /* Send the Address of the indexed register */
  (void)SPI1_SendByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
    *pBuffer = SPI1_SendByte(0x00);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */ 
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}


static uint8_t SPI1_SendByte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /* Wait to receive a Byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}