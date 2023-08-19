#include "accelerometer.h"


static uint8_t SPI1_SendByte(uint8_t byte);
void SPI1_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void SPI1_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


void ReadOutsReg(uint8_t* status)
{
  SPI1_Read(&(*status), OUTS1_ADDR, 1);
}


void ReadStatusReg(uint8_t* status)
{
  SPI1_Read(&(*status), STATUS_ADDR, 1);
}


void ReadStatReg(uint8_t* status)
{
  SPI1_Read(&(*status), STAT_ADDR, 1);
}


void ReadTemperature(uint8_t* temp)
{
  SPI1_Read(&(*temp), TEMPERATURE_ADDR, 1);
}


void ReadAcceleration(acceleration_t* accel)
{
  accel_data reg;

  /* Read 6 bytes: x acceleration, y acceleration, and z acceleration. */
  SPI1_Read(&reg.u8[0], OUT_X_ACCEL, MULTIBYTE_ACCEL_READ_LEN);

  /* Parse the results into floats with units of g */
  accel->x = 2.0f*((float)reg.u8[0]-99.0f) / 99.0f;
  accel->y = 2.0f*((float)reg.u8[2]-67.0f) / 67.0f;
  accel->z = 2.0f*((float)reg.u8[4]-99.0f) / 99.0f;
}

uint8_t InitAccelerometer(void)
{
  /* Set ctrl reg 4 to 400Hz, regs not updated until MSB/LSB are read 
   *   Z-axis enabled, Y-axis enabled, and X-axis enabled
   */
  uint8_t tmpreg = 0x97;
  SPI1_Write(&tmpreg, CTRL_REG4, 1);

  /* Enable the FIFO buffer and auto address incrementing */
  tmpreg = 0x50;
  SPI1_Write(&tmpreg, CTRL_REG6, 1);

  /* Set to bypass mode */
  tmpreg = 0x00;
  SPI1_Write(&tmpreg, FIFO_CTRL, 1);

  /* Read interrupt flags */
  SPI1_Read(&tmpreg, OUTS1_ADDR, 1);

  /* Read the WHO_AM_I reg to verify functionality */
  uint8_t who_am_i = 0;
  SPI1_Read(&who_am_i, WHO_AM_I_ADDR, 1);

  return (who_am_i == 0x3F);
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