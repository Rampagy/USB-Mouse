#include "accelerometer.h"

static accel_data acceleration_data_buffer = {0U};

/* SPI buffer variables */
static uint8_t spi_rx_buffer[MULTIBYTE_ACCEL_READ_LEN] = {0};
static const uint8_t spi_tx_buffer[MULTIBYTE_ACCEL_READ_LEN + 1] = {
    OUT_X_ACCEL_L | LIS3DSH_READ_BIT,
    OUT_X_ACCEL_H | LIS3DSH_READ_BIT,
    OUT_Y_ACCEL_L | LIS3DSH_READ_BIT,
    OUT_Y_ACCEL_H | LIS3DSH_READ_BIT,
    OUT_Z_ACCEL_L | LIS3DSH_READ_BIT,
    OUT_Z_ACCEL_H | LIS3DSH_READ_BIT,
    0x00,
};
static uint8_t tx_buffer_count = 0;
static uint8_t rx_buffer_count = 0;

static uint8_t SPI1_SendByte(uint8_t byte);
static void SPI1_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
static void SPI1_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
static void SPIQueueData(void);
static void InterpretAccelData(accel_data *reg, acceleration_t *accel);

void GetAccelerationData(acceleration_t *accel)
{
  /* Returns the acceleration data of the x,y, and z axis
   *   with units of milli-g
   *   -1000mg is equivalent to -9.81 m/s/s
   */
  taskENTER_CRITICAL();
  InterpretAccelData(&acceleration_data_buffer, accel);
  taskEXIT_CRITICAL();
}

static void SPISendQueuedData(void)
{
  SPI_I2S_SendData(SPI1, spi_tx_buffer[tx_buffer_count]);
  ++tx_buffer_count;
}

static void SPIQueueData(void)
{
  /* Start the SPI transfers */
  SPISendQueuedData();

  /* Enable the TXE interrupts */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
}

void EXTI0_IRQHandler(void)
{
  /* Handles the data ready interrupt on PE1
   *   Enables SPI interrupts to read the acceleration data
   */

  /* Disable interrupts and other tasks from running during this interrupt. */
  UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  /* If pin 0 is currently set, the data is ready to read */
  if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == Bit_SET)
  {
    /* Start of transmission, reset chip select */
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);

    /* Clear interrupt flag */
    EXTI_ClearITPendingBit(EXTI_Line0);

    /* Start reading the acceleration data from SPI */
    SPIQueueData();
  }

  /* Re-enable interrupts and other tasks. */
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void SPI1_IRQHandler(void)
{
  /* Disable interrupts and other tasks from running during this interrupt. */
  UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  /* Tx register is empty (data needs to be sent) */
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET && tx_buffer_count <= 1)
  {
    /* Disable TXE interrupts */
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
    SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);

    /* Enable RXNE interrupts */
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);

    /* Send the next queued data */
    SPISendQueuedData();
  }
  /* Rx register is not empty (data has been received) */
  else if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)
  {
    SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
    spi_rx_buffer[rx_buffer_count] = (uint8_t)SPI_I2S_ReceiveData(SPI1);
    ++rx_buffer_count;

    /* Communication complete when 6 bytes are read */
    if (rx_buffer_count >= MULTIBYTE_ACCEL_READ_LEN)
    {
      SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);

      /* Copy the received data into an acceleration buffer */
      for (uint16_t i = 0; i < MULTIBYTE_ACCEL_READ_LEN; ++i)
      {
        acceleration_data_buffer.u8[i] = spi_rx_buffer[i];
        spi_rx_buffer[i] = 0;
      }

      rx_buffer_count = 0;
      tx_buffer_count = 0;
      GPIO_SetBits(GPIOE, GPIO_Pin_3);
    }
    else
    {
      /* Send the next queued data */
      SPISendQueuedData();
    }
  }

  /* Re-enable interrupts and other tasks */
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void InterpretAccelData(accel_data *reg, acceleration_t *accel)
{
  /* Takes reg 6 bytes in from reg and returns them as floats
   *   in accel
   * The return data is in units of milli-g's
   *   -1000mg is equivalent to -9.81 m/s/s
   */

  accel->x = ((float)reg->s16[0]) / 16.38375f; // mg
  accel->y = ((float)reg->s16[1]) / 16.38375f; // mg
  accel->z = ((float)reg->s16[2]) / 16.38375f; // mg
}

void ReadAcceleration(acceleration_t *accel)
{
  accel_data reg;

  /* Read 6 bytes: x acceleration, y acceleration, and z acceleration. */
  SPI1_Read(&reg.u8[0], OUT_X_ACCEL_L, 6);

  InterpretAccelData(&reg, accel);
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
    (void)UARTQueueData(&init_regs[0], 0U);
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
    (void)UARTQueueData(&init_regs[0], 0U);
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

  /* Enable data ready interrupt (active high) on interrupt pin 1 */
  tmpreg = 0xC8;
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
    (void)UARTQueueData(&init_regs[0], 0U);
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
    (void)UARTQueueData(&init_regs[0], 0U);
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
  ReadAddr |= (uint8_t)LIS3DSH_READ_BIT;

  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);

  for (volatile uint32_t i = 0; i < 1000; ++i)
    ;

  /* Send the Address of the indexed register */
  (void)SPI1_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while (NumByteToRead > 0x00)
  {
    if (NumByteToRead > 0x01)
    {
      /* Set the next address to read */
      ++ReadAddr;
    }
    else
    {
      /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
      ReadAddr = 0x00;
    }

    *pBuffer = SPI1_SendByte(ReadAddr);
    --NumByteToRead;
    ++pBuffer;
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