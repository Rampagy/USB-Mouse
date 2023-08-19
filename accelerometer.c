#include "accelerometer.h"

uint8_t SendSPIMessage(void)
{
  for (volatile uint32_t i = 0; i < 3000000; i++);
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  //if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == SET) TIM_SetCompare1(TIM4, 0);

  /* Wait until Tx buffer is empty */
  while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET );

  /* Send data */
  SPI_I2S_SendData(SPI1, 0x8F);

  /* Wait until the RX buffer is not empty */
  while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET );



  uint16_t data = SPI_I2S_ReceiveData(SPI1);

  GPIO_SetBits(GPIOE, GPIO_Pin_3);
  
  if (((uint8_t)data) == 0xFF) // == 0x3F)
  {
    TIM_SetCompare2(TIM4, 10498);
  }
  else
  {
    TIM_SetCompare2(TIM4, 0);
  }

  return 1;
}