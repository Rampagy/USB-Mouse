#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "accelerometer.h"
#include "uart.h"
#include "fast_math_functions.h"

// Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

#define ACCEL_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 10)

StackType_t fpuTaskStack[ACCEL_TASK_STACK_SIZE] CCM_RAM; // Put task stack in CCM
StaticTask_t fpuTaskBuffer CCM_RAM;                      // Put TCB in CCM

void init_peripherals(void);
void AccelerometerTask(void *p);

int main(void)
{
  /* Update the MCU and peripheral clock frequencies */
  SystemCoreClockUpdate();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  init_peripherals();

  // Create a task
  // Stack and TCB are placed in CCM of STM32F4
  // The CCM block is connected directly to the core, which leads to zero wait states
  /* Spawn the tasks. */
  /*                             Task,   Task Name,            Stack Size, parameters, priority, task stack  */
  xTaskCreateStatic(AccelerometerTask, "AccelTask", ACCEL_TASK_STACK_SIZE, NULL, 1, fpuTaskStack, &fpuTaskBuffer);

  vTaskStartScheduler(); // should never return

  for (;;)
    ;
}

void vApplicationTickHook(void)
{
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void)
{
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
  (void)pcTaskName;
  (void)pxTask;
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}

StaticTask_t xIdleTaskTCB CCM_RAM;
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] CCM_RAM;

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
  state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task's stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
  Note that, as the array is necessarily of type StackType_t,
  configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB CCM_RAM;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] CCM_RAM;

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void AccelerometerTask(void *p)
{
  (void)p;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5;
  acceleration_t accels;
  uint32_t taskCount = 5001;

  /* Initialize the xLastWakeTime variable with the current time. */
  xLastWakeTime = xTaskGetTickCount();

  (void)UARTQueueData("\r\n\0", 0U);

  while (1)
  {
    GetAccelerationData(&accels);

    /* Set LED brightness based on acceleration. */
    if (accels.x > 100.0f)
    {
      /* Turn green on proportional to the accel, turn red off */
      TIM_SetCompare1(TIM4, (uint32_t)((accels.x - 100.0f) * 10.5f));
      TIM_SetCompare3(TIM4, 0);
    }
    else if (accels.x < -100.0f)
    {
      /* Turn red on proportional to the accel, turn green off */
      TIM_SetCompare3(TIM4, (uint32_t)((-accels.x - 100.0f) * 10.5f));
      TIM_SetCompare1(TIM4, 0);
    }

    if (accels.y > 100.0f)
    {
      /* Turn orange on proportional to the accel, turn blue off */
      TIM_SetCompare2(TIM4, 0);
      TIM_SetCompare4(TIM4, (uint32_t)((accels.y - 100.0f) * 10.5f));
    }
    else if (accels.y < -100.0f)
    {
      /* Turn blue on proportional to the accel, turn orange off */
      TIM_SetCompare4(TIM4, 0);
      TIM_SetCompare2(TIM4, (uint32_t)((-accels.y - 100.0f) * 10.5f));
    }

    /* https://www.digikey.com/en/articles/using-an-accelerometer-for-inclination-sensing */
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;

    {
      float intermediate = 0.0f;
      (void)arm_sqrt_f32(accels.x * accels.x + accels.z * accels.z, &intermediate);
      (void)arm_atan2_f32(accels.y, intermediate, &pitch);

      intermediate = 0.0f;
      (void)arm_sqrt_f32(accels.y * accels.y + accels.z * accels.z, &intermediate);
      (void)arm_atan2_f32(accels.x, intermediate, &roll);

      intermediate = 0.0f;
      (void)arm_sqrt_f32(accels.x * accels.x + accels.y * accels.y, &intermediate);
      (void)arm_atan2_f32(intermediate, accels.z, &yaw);
    }

    /* Send accel data every 50 ms */
    if ((uint32_t)taskCount * (uint32_t)xFrequency >= (uint32_t)50)
    {
      char accel_str[64] = {'\0'};
      (void)sprintf(accel_str, "%d,%d,%d,%d,%d,%d\r\n", (int16_t)accels.x, (int16_t)accels.y, (int16_t)accels.z,
                    (int16_t)(pitch * 180.0f / 3.14159f), (int16_t)(roll * 180.0f / 3.14159f), (int16_t)(yaw * 180.0f / 3.14159f));
      UARTResponseCode_t response = UARTQueueData(accel_str, 0U);
      taskCount = 0;

      if (response != UART_TX_NO_ERROR)
      {
        char response_code[4] = {'f', '\r', '\n', '\0'};
        (void)UARTQueueData(response_code, 0U);
      }
    }

    ++taskCount;

    /* Wait for the next cycle. */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

  vTaskDelete(NULL);
}

void init_peripherals(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  SPI_I2S_DeInit(SPI1);
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOD);
  GPIO_DeInit(GPIOE);
  TIM_DeInit(TIM4);
  USART_DeInit(USART3);
  EXTI_DeInit();

  /* Enable the GPIOA, GPIOD, and GPIOE Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE); // 168 MHz

  /* Enable the SPI1 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_SYSCFG, ENABLE); // 84 MHz

  /* Enable the TIM4, and USART3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_USART3, ENABLE); // 42 MHz? or 84 MHz?

  /* Enable the USB clock */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE); // 168 MHz? 84 MHz? 42 MHz?

  /* Enable the PWR clock */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Configure the LED pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; /* GREEN, ORANGE, RED, BLUE */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect TIM4 pins to AF (alternate function) */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

  /* Initialize TIM4 for use with the LEDs */
  TIM_InitStructure.TIM_Period = 10499; /* 10ms period */
  TIM_InitStructure.TIM_Prescaler = 9;  /* Implicit +1 makes 10 */
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_RepetitionCounter = 0; /* not used */
  TIM_TimeBaseInit(TIM4, &TIM_InitStructure);

  /* Initialize the output compare module for use with TIM4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure); /* Green */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure); /* Orange */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure); /* Red */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure); /* Blue */
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Start TIM4 */
  TIM_Cmd(TIM4, ENABLE);

  /* Initialize UART pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; /* GREEN, ORANGE, RED, BLUE */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect UART Pins to GPIO */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

  /* Initialize UART peripheral */
  USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_Mode = USART_Mode_Rx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART3, &USART_InitStructure);

  /* Enable UART interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART3->SR = 0x00;
  USART3->DR = 0x00;
  USART_ITConfig(USART3, USART_IT_TC, ENABLE);
  USART_Cmd(USART3, ENABLE); // Enable UART3

  /* Initialize SCLK, MISO, MOSI module */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; /* SCLK, MISO, MOSI */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect SPI pins to AF (alternate function) */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

  /* Initialize SPI module */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          /* software management of slave select (chip select) */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; /* 84 MHz / 8 = 10.50 MHz */
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Initialize CS (chip select) module */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; /* CS (chip select) */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Start SPI */
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
  SPI_Cmd(SPI1, ENABLE);

  /* Send a new line to specify a restart */
  (void)UARTQueueData("\r\n\0", 0U);

  /* Initialize the accelerometer in blocking/polling mode */
  if (InitAccelerometer() != 1U)
  {
    (void)UARTQueueData("Accelerometer Init failed\r\n\0", 0U);
    while (1)
    {
      TIM_SetCompare1(TIM4, 10500);
      TIM_SetCompare2(TIM4, 10500);
      TIM_SetCompare3(TIM4, 10500);
      TIM_SetCompare4(TIM4, 10500);

      /* Poor man's delay */
      for (volatile uint32_t i = 0; i < 100000; ++i)
        ;

      TIM_SetCompare1(TIM4, 0);
      TIM_SetCompare2(TIM4, 0);
      TIM_SetCompare3(TIM4, 0);
      TIM_SetCompare4(TIM4, 0);

      /* Poor man's delay */
      for (volatile uint32_t i = 0; i < 100000; ++i)
        ;
    }
  }
  else
  {
    (void)UARTQueueData("Accelerometer Init passed\r\n\0", 0U);
  }

  /* Disable SPI until interrupts are enabled */
  SPI_Cmd(SPI1, DISABLE);
  GPIO_SetBits(GPIOE, GPIO_Pin_3);

  /* Initialize data ready pin (PE0) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; /* INT1 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Configure data ready interrupt line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable data ready interrupt (PE1) */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable SPI interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  for (uint32_t i = 0; i < 1000000; ++i)
    ;

  /* Re-enable SPI now that the accelerometer is initialized and interrupt mode is configured */
  SPI_Cmd(SPI1, ENABLE);

  /* Connect PE0 to the EXTI line 0 */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

  /* If the pin is already set (missed the rising edge), generate an interrupt */
  if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == Bit_SET)
  {
    EXTI_GenerateSWInterrupt(EXTI_PinSource0);
  }

  (void)UARTQueueData("SPI Interrupts enabled\r\n\0", 0U);

  /* Initialize USB pins: SOF VBUS DM DP */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect pins USB full speed */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_OTG1_FS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_OTG1_FS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);

  /* Configure ID line debug */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect pins to USB full speed */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_OTG1_FS);
}