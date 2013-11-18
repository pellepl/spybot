/*
 * nrf905_driver.h
 *
 *  Created on: Nov 17, 2013
 *      Author: petera
 */

#ifndef NRF905_DRIVER_H_
#define NRF905_DRIVER_H_

#include "system.h"
#include "spi_dev.h"

#define NRF905_OK                         0
#define NRF905_ERR_BUSY                   -51
#define NRF905_ERR_BAD_CONFIG             -52
#define NRF905_ERR_ILLEGAL_STATE          -53

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

#ifndef NRF905_SPI_BUS
#define NRF905_SPI_BUS                0
#endif

typedef enum {
  NRF905_POWERDOWN = 0,
  NRF905_STANDBY,
  NRF905_CONFIG,
  NRF905_READ_CONFIG,
  NRF905_RX,
  NRF905_TX,
} nrf905_state;

typedef enum {
  NRF905_CFG_HFREQ_433 = 0,
  NRF905_CFG_HFREQ_868_915 = 1,
} nrf905_cfg_hfreq;

typedef enum {
  NRF905_CFG_PA_PWR_m10 = 0b00,
  NRF905_CFG_PA_PWR_m2 = 0b01,
  NRF905_CFG_PA_PWR_6 = 0b10,
  NRF905_CFG_PA_PWR_10 = 0b11
} nrf905_cfg_pa_pwr;

typedef enum {
  NRF905_CFG_RX_REDUCED_POWER_OFF = 0,
  NRF905_CFG_RX_REDUCED_POWER_ON,
} nrf905_cfg_rx_reduced_power;

typedef enum {
  NRF905_CFG_AUTO_RETRAN_OFF = 0,
  NRF905_CFG_AUTO_RETRAN_ON,
} nrf905_cfg_auto_retransmit;

typedef enum {
  NRF905_CFG_ADDRESS_FIELD_WIDTH_1 = 0b001,
  NRF905_CFG_ADDRESS_FIELD_WIDTH_4 = 0b100,
} nrf905_cfg_address_field_width;

typedef enum {
  NRF905_CFG_OUT_CLK_FREQ_4MHZ = 0b00,
  NRF905_CFG_OUT_CLK_FREQ_2MHZ = 0b01,
  NRF905_CFG_OUT_CLK_FREQ_1MHZ = 0b10,
  NRF905_CFG_OUT_CLK_FREQ_0_5MHZ = 0b11,
} nrf905_cfg_out_clk_freq;

typedef enum {
  NRF905_CFG_OUT_CLK_OFF = 0,
  NRF905_CFG_OUT_CLK_ON = 1
} nrf905_cfg_out_clk_enable;

typedef enum {
  NRF905_CFG_XTAL_FREQ_4MHZ = 0b000,
  NRF905_CFG_XTAL_FREQ_8MHZ = 0b001,
  NRF905_CFG_XTAL_FREQ_12MHZ = 0b010,
  NRF905_CFG_XTAL_FREQ_16MHZ = 0b011,
  NRF905_CFG_XTAL_FREQ_20MHZ = 0b100,
} nrf905_cfg_xtal_freq;

typedef enum {
  NRF905_CFG_CRC_OFF = 0,
  NRF905_CFG_CRC_ON = 1
} nrf905_cfg_crc;

typedef enum {
  NRF905_CFG_CRC_MODE_8BIT = 0,
  NRF905_CFG_CRC_MODE_16BIT = 1,
} nrf905_cfg_crc_mode;


typedef struct {
  u16_t channel_freq : 9;
  nrf905_cfg_hfreq hfreq_pll : 1;
  nrf905_cfg_pa_pwr pa_pwr : 2;
  nrf905_cfg_rx_reduced_power rx_reduced_power : 1;
  nrf905_cfg_auto_retransmit auto_retransmit : 1;
  nrf905_cfg_address_field_width rx_address_field_width : 3;
  nrf905_cfg_address_field_width tx_address_field_width : 3;
  u8_t rx_payload_width : 6;
  u8_t tx_payload_width : 6;
  u32_t rx_address;
  nrf905_cfg_out_clk_freq out_clk_freq : 2;
  nrf905_cfg_out_clk_enable out_clk_enable : 1;
  nrf905_cfg_xtal_freq crystal_frequency : 3;
  nrf905_cfg_crc crc_en : 1;
  nrf905_cfg_crc_mode crc_mode : 1;
} __attribute__ (( packed )) nrf905_config;

#define NRF905_STATE_TRANS_POWDOW_STDBY_US    30000
#define NRF905_STATE_TRANS_STDBY_RXTX_US      650
#define NRF905_STATE_TRANS_RXTX_TXRX_US       550

#define NRF905_SPI_W_CONFIG(r)          ( (0b0000 << 4) | ((r) & 0xf) )
#define NRF905_SPI_R_CONFIG(r)          ( (0b0001 << 4) | ((r) & 0xf) )
#define NRF905_SPI_W_TX_PAYLOAD         ( 0b00100000 )
#define NRF905_SPI_R_TX_PAYLOAD         ( 0b00100001 )
#define NRF905_SPI_W_TX_ADDRESS         ( 0b00100010 )
#define NRF905_SPI_R_TX_ADDRESS         ( 0b00100011 )
#define NRF905_SPI_R_RX_PAYLOAD         ( 0b00100100 )
#define NRF905_SPI_CH_CONFIG(ch,pll,pa) ( (0b1000<<12) | ((pa&0x3)<<10) | ((pll&1)<<9) | (ch&0x1ff) )

struct nrf905_s;

typedef void (*nrf905_cb)(struct nrf905_s *nrf, nrf905_state state, int res);

typedef struct nrf905_s {
  volatile bool busy;
  nrf905_state state;
  nrf905_cb cb;
  spi_dev *spi_dev;
  hw_io_port pwr_port;
  hw_io_pin pwr_pin;
  hw_io_port trx_ce_port;
  hw_io_pin trx_ce_pin;
  hw_io_port tx_en_port;
  hw_io_pin tx_en_pin;
  nrf905_config config;
  u8_t _buf[16];
  void *_scratch;
} nrf905;

void NRF905_init(nrf905 *nrf, spi_dev *spi_dev,
    nrf905_cb cb,
    hw_io_port pwr_port, hw_io_pin pwr_pin,
    hw_io_port trx_ce_port, hw_io_pin trx_ce_pin,
    hw_io_port tx_en_port, hw_io_pin tx_en_pin);

int NRF905_powerdown(nrf905 *nrf);

int NRF905_standby(nrf905 *nrf);

int NRF905_config(nrf905 *nrf, nrf905_config *cfg);

int NRF905_read_config(nrf905 *nrf, nrf905_config *cfg);

int NRF905_config_channel(nrf905 *nrf, u16_t channel_freq);

int NRF905_config_pa(nrf905 *nrf, nrf905_cfg_pa_pwr pa_pwr);

int NRF905_config_rx_address(nrf905 *nrf, u32_t rx_address);

int NRF905_signal_data_ready(nrf905 *nrf);

#endif /* NRF905_DRIVER_H_ */
