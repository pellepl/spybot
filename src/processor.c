/*
 * processor.c
 *
 *  Created on: Aug 1, 2012
 *      Author: petera
 */

#include "processor.h"
#include "system.h"
#include "gpio.h"

static void RCC_config() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  #ifdef CONFIG_UART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#ifdef CONFIG_UART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
#ifdef CONFIG_UART3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
#ifdef CONFIG_UART4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);


  /* PCLK1 = HCLK/1 */
  RCC_PCLK1Config(RCC_HCLK_Div1);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(STM32_SYSTEM_TIMER_RCC, ENABLE);

#ifdef CONFIG_SPI
#ifdef CONFIG_SPI1
  /* Enable SPI1_MASTER clock and GPIO clock for SPI1_MASTER */
  RCC_APB2PeriphClockCmd(SPI1_MASTER_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(SPI1_MASTER_CLK, ENABLE);

  /* Enable SPI1_MASTER DMA clock */
  RCC_AHBPeriphClockCmd(SPI1_MASTER_DMA_CLK, ENABLE);
#endif
#ifdef CONFIG_SPI2
  /* Enable SPI2_MASTER clock and GPIO clock for SPI2_MASTER */
  RCC_APB2PeriphClockCmd(SPI2_MASTER_GPIO_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(SPI2_MASTER_CLK, ENABLE);

  /* Enable SPI2_MASTER DMA clock */
  RCC_AHBPeriphClockCmd(SPI2_MASTER_DMA_CLK, ENABLE);
#endif
#endif

#ifdef CONFIG_I2C
  RCC_APB1PeriphClockCmd(I2C1_CLK, ENABLE);
#endif

#ifdef CONFIG_ADC
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
#endif

  // spybot cvideo
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

static void NVIC_config(void)
{
  // Configure the NVIC Preemption Priority Bits
  // use 3 bits for preemption and 1 bit for  subgroup
  u8_t prioGrp = 8 - __NVIC_PRIO_BITS;
  // use 4 bits for preemption and 0 subgroups
  //u8_t prioGrp = 8 - __NVIC_PRIO_BITS - 1;
  NVIC_SetPriorityGrouping(prioGrp);


  // Config systick interrupt
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioGrp, 1, 1));

  // Config pendsv interrupt, lowest
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(prioGrp, 7, 1));

  // Config & enable TIM interrupt
  NVIC_SetPriority(STM32_SYSTEM_TIMER_IRQn, NVIC_EncodePriority(prioGrp, 1, 0));
  NVIC_EnableIRQ(STM32_SYSTEM_TIMER_IRQn);

  // Config & enable uarts interrupt
#ifdef CONFIG_UART2
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(prioGrp, 2, 0));
  NVIC_EnableIRQ(USART2_IRQn);
#endif

#ifdef CONFIG_SPI
  // Config & enable the SPI-DMA interrupt
#ifdef CONFIG_SPI1
  NVIC_SetPriority(SPI1_MASTER_Rx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 3, 0));
  NVIC_EnableIRQ(SPI1_MASTER_Rx_IRQ_Channel);
  NVIC_SetPriority(SPI1_MASTER_Tx_IRQ_Channel, NVIC_EncodePriority(prioGrp, 3, 1));
  NVIC_EnableIRQ(SPI1_MASTER_Tx_IRQ_Channel);
#endif
#endif

#ifdef CONFIG_I2C
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(prioGrp, 1, 1));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(prioGrp, 1, 1));
  NVIC_EnableIRQ(I2C2_ER_IRQn);
#endif

#ifdef CONFIG_ADC
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(prioGrp, 6, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);
#endif

  // range sensor
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(prioGrp, 2, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  // extis
  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(prioGrp, 1, 0)); // radio data ready
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(prioGrp, 1, 0)); // radio carrier detect
  NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(prioGrp, 1, 0));
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(prioGrp, 1, 0));
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(prioGrp, 1, 0));

  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(prioGrp, 0, 0));   // rover cvideo vsync
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(prioGrp, 0, 1)); // rover cvideo hsync
}

static void UART2_config() {
#ifdef CONFIG_UART2
  gpio_config(PORTA, PIN2, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  gpio_config(PORTA, PIN3, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
#endif
}
static void SPI_config() {
#ifdef CONFIG_SPI
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
#ifdef CONFIG_SPI1
  // SPI1

  /* Configure SPI1_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI1_MASTER_PIN_SCK | SPI1_MASTER_PIN_MOSI | SPI1_MASTER_PIN_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI1_MASTER_GPIO, &GPIO_InitStructure);

  /* SPI1_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // APB2/64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1_MASTER, &SPI_InitStructure);

#ifndef CONFIG_SPI_POLL
  /* Configure SPI DMA common */
  // SPI1_BASE(APB2PERIPH_BASE(PERIPH_BASE(0x40000000) + 0x00010000) + 3000)
  // DataRegister offset = 0x0c = SPI1_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_MASTER_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  /* Configure SPI DMA rx */
  DMA_DeInit(SPI1_MASTER_Rx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI1_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

  /* Configure SPI DMA tx */
  DMA_DeInit(SPI1_MASTER_Tx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI1_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA SPI RX channel transfer complete interrupt */
  DMA_ITConfig(SPI1_MASTER_Rx_DMA_Channel, DMA_IT_TC | DMA_IT_TE, ENABLE);
  // Do not enable DMA SPI TX channel transfer complete interrupt,
  // always use tx/rx transfers and only await DMA RX finished irq
  DMA_ITConfig(SPI1_MASTER_Tx_DMA_Channel, DMA_IT_TE, ENABLE);

  /* Enable SPI_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI1_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , ENABLE);
#endif // CONFIG_SPI_POLL
#endif //CONFIG_SPI1

#ifdef CONFIG_SPI2
  // SPI2

  /* Configure SPI2_MASTER pins: NSS, SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI2_MASTER_PIN_SCK | SPI2_MASTER_PIN_MOSI | SPI2_MASTER_PIN_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI2_MASTER_GPIO, &GPIO_InitStructure);

  /* SPI2_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // APB2/64
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2_MASTER, &SPI_InitStructure);

#ifndef CONFIG_SPI_POLL
  /* Configure SPI DMA common */
  // SPI1_BASE(APB2PERIPH_BASE(PERIPH_BASE(0x40000000) + 0x00010000) + 3000)
  // DataRegister offset = 0x0c = SPI1_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_MASTER_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  /* Configure SPI2 DMA rx */
  DMA_DeInit(SPI2_MASTER_Rx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI2_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

  /* Configure SPI DMA tx */
  DMA_DeInit(SPI2_MASTER_Tx_DMA_Channel);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(SPI2_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

  /* Enable DMA SPI RX channel transfer complete interrupt */
  DMA_ITConfig(SPI2_MASTER_Rx_DMA_Channel, DMA_IT_TC | DMA_IT_TE, ENABLE);
  // Do not enable DMA SPI TX channel transfer complete interrupt,
  // always use tx/rx transfers and only await DMA RX finished irq
  DMA_ITConfig(SPI2_MASTER_Tx_DMA_Channel, DMA_IT_TE, ENABLE);

  /* Enable SPI_MASTER DMA Rx/Tx request */
  SPI_I2S_DMACmd(SPI2_MASTER, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx , ENABLE);
#endif // CONFIG_SPI_POLL
#endif //CONFIG_SPI2
#endif // CONFIG_SPI
}
static void LED_config() {
#ifdef CONFIG_LED
#endif
}

static void TIM_config() {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  u16_t prescaler = 0;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = SYS_CPU_FREQ/SYS_MAIN_TIMER_FREQ;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(STM32_SYSTEM_TIMER, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(STM32_SYSTEM_TIMER, prescaler, TIM_PSCReloadMode_Immediate);

  /* TIM IT enable */
  TIM_ITConfig(STM32_SYSTEM_TIMER, TIM_IT_Update, ENABLE);

  /* TIM enable counter */
  TIM_Cmd(STM32_SYSTEM_TIMER, ENABLE);
}

static void ADC_config() {
#ifdef CONFIG_ADC
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure C05 (ADC Channel15) as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
}

static void I2C_config() {
#ifdef CONFIG_I2C
  gpio_config(PORTB, PIN10, CLK_50MHZ, AF, AF0, OPENDRAIN, NOPULL);
  gpio_config(PORTB, PIN11, CLK_50MHZ, AF, AF0, OPENDRAIN, NOPULL);
#endif
}

static void SERVO_config() {
  gpio_config(PORTB, PIN6, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  gpio_config(PORTB, PIN7, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  gpio_config(PORTB, PIN8, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  gpio_config(PORTB, PIN9, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);

  // 0xffff => 20ms = 1/50
  // 1/(50*65535) = 1/(72000000/x) => x = 72000000/(50*65535) => 22
  // 72000000/22 = 3272727
  // 3272727 / 65535 = 49.94
  // 3277 ~= 1ms
  // 4915 ~= 1.5ms
  // 6554 ~= 2ms

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = 22-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OCInitStructure.TIM_Pulse = 2500-1;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_Pulse = 5000-1;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_Pulse = 7500-1;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_Pulse = 4915;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(TIM4, ENABLE);


  TIM_Cmd(TIM4, ENABLE);
}

static void RANGE_SENS_config() {
}

static void CVIDEO_config() {
  // config spi & dma
  SPI_InitTypeDef  SPI_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;

  /* SPI2_MASTER configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // APB2/16
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  // DataRegister offset = 0x0c = SPI2_BASE + 0x0c
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE + 0x0c);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  // Configure SPI DMA tx
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Channel5, DMA_IT_TE, DISABLE);

  /* Enable SPI_MASTER DMA Tx request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx , ENABLE);


  // config io
  // B15 video overlay (spi mosi)
  gpio_config(PORTB, PIN15, CLK_50MHZ, OUT, AF0, PUSHPULL, NOPULL);
  gpio_disable(PORTB, PIN15);
  // B8 vsync
  gpio_config(PORTB, PIN8, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
  // B14 hsync
  gpio_config(PORTB, PIN14, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
  gpio_interrupt_mask_disable(PORTB, PIN14);
  gpio_interrupt_mask_disable(PORTB, PIN8);
}

// ifc

void PROC_base_init() {
  RCC_config();
  NVIC_config();
}

void PROC_periph_init() {
  DBGMCU_Config(STM32_SYSTEM_TIMER_DBGMCU, ENABLE);
  gpio_init();

  UART2_config();
  LED_config();
  SPI_config();
  TIM_config();
  ADC_config();
  I2C_config();

  SERVO_config();
  RANGE_SENS_config();
  CVIDEO_config();
}

