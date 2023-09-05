#ifndef UART_H
#define UART_H

/* Includes */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

/* Max size of tx buffer in bytes */
#define UART_MAX_BUFFER_LEN (128)

/* UART send data return code. */
typedef enum
{
  UART_TX_NO_ERROR,
  /* Tx buffer would be full if the data was added OR
   * tx buffer is already full. Try again later.
   * Can also increase TX buffer size.
   */
  UART_TX_TX_BUFFER_FULL,
} UARTResponseCode_t;

UARTResponseCode_t UARTQueueData(char *buffer, uint8_t from_isr);
void USART3_IRQHandler(void);

#endif