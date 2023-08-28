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

  /* Read 6 bytes: x acceleration, y acceleration, and z acceleration. */
  SPI1_Read(&reg.u8[0], OUT_X_ACCEL_L, 1);
  SPI1_Read(&reg.u8[1], OUT_X_ACCEL_H, 1);
  SPI1_Read(&reg.u8[2], OUT_Y_ACCEL_L, 1);
  SPI1_Read(&reg.u8[3], OUT_Y_ACCEL_H, 1);
  SPI1_Read(&reg.u8[4], OUT_Z_ACCEL_L, 1);
  SPI1_Read(&reg.u8[5], OUT_Z_ACCEL_H, 1);

  accel->x = 0.061f * ((float)reg.s16[0]); //  mg
  accel->y = 0.061f * ((float)reg.s16[1]); //  mg
  accel->z = 0.061f * ((float)reg.s16[2]); //  mg

  char accel_str[64] = {'\0'};
  (void)sprintf(accel_str, "x: %dmg y: %dmg z: %dmg\r\n", (int16_t)(accel->x), (int16_t)(accel->y), (int16_t)(accel->z));
  UARTQueueData(accel_str);
}

uint8_t self_test(void)
{
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

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 100000; ++i)
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

  /* Reset accelerometer before configuring it */
  tmpreg = 0x80;
  SPI1_Write(&tmpreg, CTRL_REG6, 1);

  /* Wait until reboot is finished */
  tmpreg = 0U;
  while (tmpreg & 0x80)
  {
    /* Poor man's delay */
    for (volatile uint32_t i = 0; i < 10000; ++i)
      ;

    SPI1_Read(&tmpreg, CTRL_REG6, 1);
  }

  /* Soft reset the accelerometer before configuring it */
  tmpreg = 0x01;
  SPI1_Write(&tmpreg, CTRL_REG3, 1);

  /* Wait until soft reset is finished */
  tmpreg = 0U;
  while (tmpreg & 0x01)
  {
    /* Poor man's delay */
    for (volatile uint32_t i = 0; i < 10000; ++i)
      ;

    SPI1_Read(&tmpreg, CTRL_REG3, 1);
  }

  tmpreg = 0x67; // 100Hz, continuous update, x, y, z enabled
  SPI1_Write(&tmpreg, CTRL_REG4, 1);

  /* Poor man's delay */
  for (volatile uint32_t i = 0; i < 10000; ++i)
    ;

  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG1, 1);

  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG2, 1);

  tmpreg = 0x00;
  SPI1_Write(&tmpreg, CTRL_REG3, 1);

  /* 200 hz bandwidth filter */
  tmpreg = 0x40;
  SPI1_Write(&tmpreg, CTRL_REG5, 1);

  tmpreg = 0x10;
  SPI1_Write(&tmpreg, CTRL_REG6, 1);

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
  for (volatile uint32_t i = 0; i < 100000; ++i)
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
  for (volatile uint32_t i = 0; i < 100000; ++i)
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