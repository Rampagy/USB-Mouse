#include "uart.h"

static uint8_t uart_buffer[UART_MAX_BUFFER_LEN] = {0};
static uint16_t uart_tx_buffer_size = 0;

void UARTSendData(void)
{
  USART_SendData(USART3, uart_buffer[0]);

  /* Shift all the bytes up in the queue */
  for (uint16_t i = 0; i < uart_tx_buffer_size - 1; ++i)
  {
    uart_buffer[i] = uart_buffer[i + 1];
  }
  uart_buffer[uart_tx_buffer_size] = 0;

  uart_tx_buffer_size--;
}

UARTResponseCode_t UARTQueueData(char *buffer)
{
  /* Add bytes to the uart transmit queue
   *   Input must have a null terminating zero at the end
   */

  UARTResponseCode_t return_code = UART_TX_NO_ERROR;
  uint16_t buffer_len = 0;

  /* Count the length of the input buffer */
  while (*(buffer + buffer_len) != '\0')
  {
    buffer_len++;
  }

  if (uart_tx_buffer_size + buffer_len > UART_MAX_BUFFER_LEN)
  {
    return_code = UART_TX_TX_BUFFER_FULL;
  }
  else
  {
    /* Add the data to the queue */
    taskENTER_CRITICAL();

    for (uint16_t i = 0; i < buffer_len; ++i)
    {
      uart_buffer[uart_tx_buffer_size] = *(buffer + i);
      uart_tx_buffer_size++;
    }

    taskEXIT_CRITICAL();

    /* Check if transmit mode needs to be enabled */
    if (!(USART3->CR1 & USART_Mode_Tx))
    {
      /* Add data to the data register */
      UARTSendData();

      /* Turn Tx mode on */
      USART3->CR1 |= USART_Mode_Tx;
    }
  }

  return return_code;
}

void USART3_IRQHandler(void)
{
  /* Handles the UART Interrupt.
   * On a transfer complete interrupt, populates the data register
   * with the next byte to send
   */

  /* Disable interrupts and other tasks from running during this interrupt. */
  UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if (USART_GetFlagStatus(USART3, USART_FLAG_TC))
  {
    /* If the current index is less than the size of the buffer send the byte */
    if (uart_tx_buffer_size > 0)
    {
      /* Add data to the data register */
      UARTSendData();
    }
    /* There are no more bytes to transfer */
    else
    {
      /* Turn Tx mode off, clear the data register */
      USART3->CR1 &= ~USART_Mode_Tx;
      USART_SendData(USART3, 0);
    }
  }

  /* Re-enable interrupts and other tasks. */
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

  return;
}
