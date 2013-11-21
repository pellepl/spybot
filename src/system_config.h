/*
 * system_config.h
 *
 *  Created on: Jul 24, 2012
 *      Author: petera
 */

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

#include "config_header.h"
#include "types.h"
#include "stm32f10x.h"


#define APP_NAME "SPYBOT"


/****************************************/
/***** Functionality block settings *****/
/****************************************/


// enable uart1
#define CONFIG_UART2

#define CONFIG_UART_CNT   1 // update according to enabled uarts

#define CONFIG_SPI1
//#define CONFIG_SPI2

/*********************************************/
/***** Hardware build time configuration *****/
/*********************************************/

/** Processor specifics **/

#ifndef USER_HARDFAULT
// enable user hardfault handler
#define USER_HARDFAULT 1
#endif

// hardware debug (blinking leds etc)
#define HW_DEBUG


/** General **/

// internal flash start address
#define FLASH_START       FLASH_BASE
// internal flash page erase size
#define FLASH_PAGE_SIZE   0x400 // md
// internal flash protection/unprotection for firmware
#define FLASH_PROTECT     FLASH_WRProt_AllPages
// internal flash total size in bytes
#define FLASH_TOTAL_SIZE  (64*1024) // md

/** UART **/

#ifdef CONFIG_UART2
#define UART2_GPIO_PORT       GPIOA
#define UART2_GPIO_RX         GPIO_Pin_3
#define UART2_GPIO_TX         GPIO_Pin_2
#endif

/** SPI **/

#ifdef CONFIG_SPI

// make SPI driver use polling method, otherwise DMA requests are used
// warning - polling method should only be used for debugging and may be
// unstable. Do not sent multitudes of data using this config
//#define CONFIG_SPI_POLL

// on some stm32f1s it seems to be a bad idea closing down the SPI between
// operations
#define CONFIG_SPI_KEEP_RUNNING

#ifdef CONFIG_SPI1

#define SPI1_MASTER_GPIO              GPIOA
#define SPI1_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SPI1_MASTER_PIN_SCK           GPIO_Pin_5
#define SPI1_MASTER_PIN_MISO          GPIO_Pin_6
#define SPI1_MASTER_PIN_MOSI          GPIO_Pin_7

#define SPI1_MASTER                   SPI1
#define SPI1_MASTER_BASE              SPI1_BASE
#define SPI1_MASTER_CLK               RCC_APB2Periph_SPI1
#define SPI1_MASTER_DMA               DMA1
#define SPI1_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI1_MASTER_Rx_DMA_Channel    DMA1_Channel2
#define SPI1_MASTER_Tx_DMA_Channel    DMA1_Channel3
#define SPI1_MASTER_Rx_IRQ_Channel    DMA1_Channel2_IRQn
#define SPI1_MASTER_Tx_IRQ_Channel    DMA1_Channel3_IRQn

#endif // CONFIG_SPI1

#ifdef CONFIG_SPI2

#define SPI2_MASTER_GPIO              GPIOB
#define SPI2_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOB
#define SPI2_MASTER_PIN_SCK           GPIO_Pin_13
#define SPI2_MASTER_PIN_MISO          GPIO_Pin_14
#define SPI2_MASTER_PIN_MOSI          GPIO_Pin_15

#define SPI2_MASTER                   SPI2
#define SPI2_MASTER_BASE              SPI2_BASE
#define SPI2_MASTER_CLK               RCC_APB1Periph_SPI2
#define SPI2_MASTER_DMA               DMA1
#define SPI2_MASTER_DMA_CLK           RCC_AHBPeriph_DMA1
// according to userguide table 78
#define SPI2_MASTER_Rx_DMA_Channel    DMA1_Channel4
#define SPI2_MASTER_Tx_DMA_Channel    DMA1_Channel5
#define SPI2_MASTER_Rx_IRQ_Channel    DMA1_Channel4_IRQn
#define SPI2_MASTER_Tx_IRQ_Channel    DMA1_Channel5_IRQn

#endif // CONFIG_SPI2

#endif // CONFIG_SPI

/** I2C **/

#ifdef CONFIG_I2C

#define I2C1_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define I2C1_CLK                      RCC_APB1Periph_I2C2
#define I2C1_GPIO_PORT                GPIOB
#define I2C1_SCL_GPIO_PIN_SOURCE      GPIO_PinSource10
#define I2C1_SDA_GPIO_PIN_SOURCE      GPIO_PinSource11
#define I2C1_SCL_GPIO_PIN             GPIO_Pin_10
#define I2C1_SDA_GPIO_PIN             GPIO_Pin_11
#define I2C1_PORT                     I2C2

#define I2C_MAX_ID                    1

#endif

/** ADC **/

// TODO #define CONFIG_ADC


/** RADIO **/

#ifndef NRF905_DATA_READY_PORT
#define NRF905_DATA_READY_PORT        PORTA
#endif
#ifndef NRF905_DATA_READY_PIN
#define NRF905_DATA_READY_PIN         PIN0
#endif

#ifndef NRF905_CARRIER_DETECT_PORT
#define NRF905_CARRIER_DETECT_PORT    PORTA
#endif
#ifndef NRF905_CARRIER_DETECT_PIN
#define NRF905_CARRIER_DETECT_PIN     PIN1
#endif

#ifndef NRF905_PWR_PORT
#define NRF905_PWR_PORT               PORTA
#endif
#ifndef NRF905_PWR_PIN
#define NRF905_PWR_PIN                PIN10
#endif

#ifndef NRF905_CS_PORT
#define NRF905_CS_PORT                PORTA
#endif
#ifndef NRF905_CS_PIN
#define NRF905_CS_PIN                 PIN11
#endif

#ifndef NRF905_TRX_CE_PORT
#define NRF905_TRX_CE_PORT            PORTA
#endif
#ifndef NRF905_TRX_CE_PIN
#define NRF905_TRX_CE_PIN             PIN12
#endif

#ifndef NRF905_TX_EN_PORT
#define NRF905_TX_EN_PORT             PORTB
#endif
#ifndef NRF905_TX_EN_PIN
#define NRF905_TX_EN_PIN              PIN5
#endif



/****************************************************/
/******** Application build time configuration ******/
/****************************************************/

/** TICKER **/
// STM32 system timer
#define CONFIG_STM32_SYSTEM_TIMER   2
// system timer frequency
#define SYS_MAIN_TIMER_FREQ   40000
// system timer counter type
typedef u16_t system_counter_type;
// system tick frequency
#define SYS_TIMER_TICK_FREQ   1000
// os ticker cpu clock div
#define SYS_OS_TICK_DIV       8

/** UART **/

#define UARTSTDIN       0
#define UARTSTDOUT      0

#define UART2_SPEED 460800

#define USE_COLOR_CODING

/** IO **/
#define CONFIG_IO_MAX   1

#define IOSTD        0
#define IODBG        IOSTD


/** DEBUG **/

//#define DBG_RADIO

// disable all asserts
//#define ASSERT_OFF

// disable all debug output
//#define DBG_OFF

#define CONFIG_DEFAULT_DEBUG_MASK     (0xffffffff)

// enable or disable tracing
#define DBG_TRACE_MON
#define TRACE_SIZE            (64)

// enable os thread led blinky
#define DBG_OS_THREAD_BLINKY

#define VALID_RAM(x) \
  (((void*)(x) >= RAM_BEGIN && (void*)(x) < RAM_END))

#define VALID_FLASH(x) \
  ((void*)(x) >= (void*)FLASH_BEGIN && (void*)(x) < (void*)(FLASH_END))

#define VALID_DATA(x) \
  (VALID_RAM(x) || VALID_FLASH(x))

#define OS_DBG_BADR(x) \
    (!VALID_RAM(x))


#endif /* SYSTEM_CONFIG_H_ */
