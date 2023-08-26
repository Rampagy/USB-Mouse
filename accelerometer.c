#include "accelerometer.h"

static uint8_t SPI1_SendByte(uint8_t byte);
void SPI1_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void SPI1_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/*
void ReadOutsReg(uint8_t* status)
{
  SPI1_Read(&(*status), OUTS1_ADDR, 1);
}
*/

void ReadStatusReg(uint8_t *status)
{
  SPI1_Read(status, STATUS_ADDR, 1);
}

void ReadFIFOStatusReg(uint8_t *status)
{
  SPI1_Read(&(*status), FIFO_SRC_ADDR, 1);
}

void ReadStatReg(uint8_t *status)
{
  SPI1_Read(&(*status), STAT_ADDR, 1);
}

/*
void ReadTemperature(uint8_t* temp)
{
  SPI1_Read(&(*temp), TEMPERATURE_ADDR, 1);
}
*/

void ReadAcceleration(acceleration_t *accel)
{
  accel_data reg;
  // uint8_t status0 = 0;
  uint8_t status1 = 0;

  /* Wait for new data to arrive before reading accel data */
  // while ((status1 & 0x08) == 0x00)
  {
    //  ReadFIFOStatusReg(&status0);
    // TIM_SetCompare1(TIM4, 10500);
    ReadStatusReg(&status1);
    // TIM_SetCompare1(TIM4, 0);
  }

  // make sure we read at least one x, y, and z
  // if ((status0 & 0x1F) == 0)
  //{
  //  status0 = 1;
  //}

  /* Clear out the FIFO buffer */
  // while ((status0 & 0x1F) > 0)
  {
    /* Read 6 bytes: x acceleration, y acceleration, and z acceleration. */
    SPI1_Read(&reg.u8[0], OUT_X_ACCEL, MULTIBYTE_ACCEL_READ_LEN);
    // status0 = (status0 & 0x1F) - 1;
  }

  /* Enable FIFO mode*/
  // status1 = 0x70;
  // SPI1_Write(&status1, CTRL_REG6, 1);

  // status1 = STREAM_MODE | 0x0A;
  // SPI1_Write(&status1, FIFO_CTRL_ADDR, 1);

  // status1 = BYPASS_MODE | 0x0A;
  // SPI1_Write(&status1, FIFO_CTRL_ADDR, 1);

  /* Parse the results into floats with units of g */
  // int16_t x = (int16_t)(((uint16_t)reg.u8[1] << 8) | (uint16_t)reg.u8[0]);
  // int16_t y = (int16_t)(((uint16_t)reg.u8[3] << 8) | (uint16_t)reg.u8[2]);
  // int16_t z = (int16_t)(((uint16_t)reg.u8[5] << 8) | (uint16_t)reg.u8[4]);

  // accel->x = (float)(x) / 32767.5f;
  // accel->y = (float)(y) / 32767.5f;
  // accel->z = (float)(z) / 32767.5f;

  accel->x = (int16_t)((reg.s8[1] << 8) | reg.s8[0]);
  accel->y = (int16_t)((reg.s8[3] << 8) | reg.s8[2]);
  accel->z = (int16_t)((reg.s8[5] << 8) | reg.s8[4]);

  char accel_str[64] = {'\0'};
  (void)sprintf(accel_str, "x: %d y: %d z: %d\r\n", accel->x, accel->y, accel->z);
  UARTQueueData(accel_str);
}

uint8_t self_test(void)
{
  uint8_t tmpreg = 0;

  /* Read accel */
  acceleration_t accels_baseline;
  ReadAcceleration(&accels_baseline);

  /* Positive self test */
  tmpreg = 0x42;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 10000000; ++i)
    ;

  acceleration_t accels_positive;
  ReadAcceleration(&accels_positive);

  /* Negative self test */
  tmpreg = 0x44;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 10000000; ++i)
    ;

  acceleration_t accels_negative;
  ReadAcceleration(&accels_negative);

  /* 200 hz bandwidth filter, self test is off */
  tmpreg = 0x40;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  /* Read the WHO_AM_I reg to verify functionality */
  uint8_t who_am_i = 0;
  SPI1_Read(&who_am_i, WHO_AM_I_ADDR, 1);

  uint8_t info1 = 0;
  SPI1_Read(&info1, INFO1_ADDR, 1);

  uint8_t info2 = 0;
  SPI1_Read(&info2, INFO2_ADDR, 1);

  {
    char init_regs[64] = {'\0'};
    sprintf(&init_regs[0], "INFO1 %02X, INFO2 %02X, WhoAmI %02X\r\n", info1, info2, who_am_i);
    (void)UARTQueueData(&init_regs[0]);
  }

  uint8_t init_success_flag = 0;
  if ((who_am_i == 0x3F) && (info1 == 0x21) && (info2 == 0x00))
  {
    init_success_flag |= 0x01;
  }

  if (((0.06f * ((float)accels_positive.x - (float)accels_baseline.x)) > 0.0f) &&
      ((0.06f * ((float)accels_positive.y - (float)accels_baseline.y)) > 0.0f) &&
      ((0.06f * ((float)accels_positive.z - (float)accels_baseline.z)) > 0.0f) &&
      ((0.06f * ((float)accels_negative.x - (float)accels_baseline.x)) < 0.0f) &&
      ((0.06f * ((float)accels_negative.y - (float)accels_baseline.y)) < 0.0f) &&
      ((0.06f * ((float)accels_negative.z - (float)accels_baseline.z)) < 0.0f))
  {
    init_success_flag |= 0x02;
  }

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 10000000; ++i)
    ;

  {
    char init_regs[64] = {'\0'};
    sprintf(&init_regs[0], "SF %02X\r\n", init_success_flag);
    (void)UARTQueueData(&init_regs[0]);
  }

  return init_success_flag;
}

uint8_t InitAccelerometer(void)
{
  uint8_t tmpreg = 0;
  uint8_t reg_read[6] = {0};

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 1000000; ++i)
    ;

  tmpreg = 0x67; // 100Hz, continuous update, x, y, z enabled
  SPI1_Write(&tmpreg, CTRL_REG4, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 1000000; ++i)
    ;

  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG1, 1);

  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG2, 1);

  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG3, 1);

  /* 200 hz bandwidth filter */
  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  tmpreg = 0x10;
  SPI1_Write(&tmpreg, CTRL_REG6, 1);

  // tmpreg = 0x70;
  // SPI1_Write(&tmpreg, CTRL_REG6, 1);

  // tmpreg = STREAM_MODE | 0x0A;
  // SPI1_Write(&tmpreg, FIFO_CTRL_ADDR, 1);

  SPI1_Read(&reg_read[0], STAT_ADDR, 1);
  SPI1_Read(&reg_read[1], CTRL_REG4, 1);
  SPI1_Read(&reg_read[2], CTRL_REG3, 1);
  SPI1_Read(&reg_read[3], CTRL_REG5, 1);
  SPI1_Read(&reg_read[4], CTRL_REG6, 1);
  SPI1_Read(&reg_read[5], STATUS_ADDR, 1);

  {
    char init_regs[64] = {'\0'};
    sprintf(&init_regs[0], "STAT %02X, REG4 %02X, REG3 %02X, REG5 %02X, REG6 %02X, STATUS %02X\r\n", reg_read[0], reg_read[1], reg_read[2], reg_read[3], reg_read[4], reg_read[5]);
    (void)UARTQueueData(&init_regs[0]);
  }

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 10000000; ++i)
    ;

  SPI1_Read(&reg_read[0], FIFO_CTRL_ADDR, 1);
  SPI1_Read(&reg_read[1], FIFO_SRC_ADDR, 1);
  SPI1_Read(&reg_read[2], INFO1_ADDR, 1);
  SPI1_Read(&reg_read[3], INFO2_ADDR, 1);
  SPI1_Read(&reg_read[4], WHO_AM_I_ADDR, 1);

  {
    char init_regs[64] = {'\0'};
    sprintf(&init_regs[0], "FIFO_CTRL %02X, FIFO SRC %02X, INFO1 %02X, INFO2 %02X, WhoAmI %02X\r\n", reg_read[0], reg_read[1], reg_read[2], reg_read[3], reg_read[4]);
    (void)UARTQueueData(&init_regs[0]);
  }

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 10000000; ++i)
    ;

  return self_test();
}

void SPI1_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
     - When 0, the address will remain unchanged in multiple read/write commands.
     - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if (NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)LIS3DSH_MULTI_BYTE;
  }

  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);

  for (volatile uint32_t i = 0; i < 1000; ++i)
    ;

  /* Send the Address of the indexed register */
  (void)SPI1_SendByte(WriteAddr);

  /* Send the data that will be written into the device (MSB First) */
  while (NumByteToWrite > 0x00)
  {
    (void)SPI1_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  for (volatile uint32_t i = 0; i < 1000; ++i)
    ;

  /* Set chip select High at the end of the transmission */
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

void SPI1_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if (NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(LIS3DSH_READ_BIT | LIS3DSH_MULTI_BYTE);
  }
  else
  {
    ReadAddr |= (uint8_t)LIS3DSH_READ_BIT;
  }

  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);

  for (volatile uint32_t i = 0; i < 1000; ++i)
    ;

  /* Send the Address of the indexed register */
  (void)SPI1_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while (NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
    *pBuffer = SPI1_SendByte(0x00);
    NumByteToRead--;
    pBuffer++;
  }

  for (volatile uint32_t i = 0; i < 1000; ++i)
    ;

  /* Set chip select High at the end of the transmission */
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

static uint8_t SPI1_SendByte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    ;

  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /* Wait to receive a Byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    ;

  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}