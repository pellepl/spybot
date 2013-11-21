/*
 * nrf905_impl.c
 *
 *  Created on: Nov 18, 2013
 *      Author: petera
 */

#include "nrf905_impl.h"
#include "spi_dev.h"
#include "gpio.h"
#include "miniutils.h"

static nrf905 nrf;
static spi_dev nrf_spi_dev;
static nrf905_config config;

static void nrf_impl_cb(nrf905 *nrf, nrf905_state state, int res) {
  print("NRF cb state:%i res:%i\n", state, res);
  if (res != NRF905_OK) return;
  switch (state) {
  case NRF905_RX_READ:
    print("NRF got packet\n");
    printbuf(IOSTD, NRF905_RX_PKT_BUFFER(nrf), 16);
    NRF905_rx(nrf);
    break;
  case NRF905_TX:
    print("NRF sent packet\n");
    break;
  case NRF905_READ_CONFIG:
    print(
        "auto retransmit: %i\n"
        "channel:         %i\n"
        "crc enable:      %i\n"
        "crc mode:        %i\n"
        "xtal freq:       %i\n"
        "hfreq pll:       %i\n"
        "out clock enable:%i\n"
        "out clock freq:  %i\n"
        "pa power:        %i\n"
        "rx address:      %08x\n"
        "rx addr width:   %i\n"
        "rx payload len:  %i\n"
        "rx reduced pow:  %i\n"
        "tx addr width:   %i\n"
        "tx payload len:  %i\n"
        ,
        config.auto_retransmit,
        config.channel_freq,
        config.crc_en,
        config.crc_mode,
        config.crystal_frequency,
        config.hfreq_pll,
        config.out_clk_enable,
        config.out_clk_freq,
        config.pa_pwr,
        config.rx_address,
        config.rx_address_field_width,
        config.rx_payload_width,
        config.rx_reduced_power,
        config.tx_address_field_width,
        config.tx_payload_width);
    break;
  default:
    break;
  }
}

static void nrf_impl_data_ready_irq(gpio_pin pin) {
  if (pin == NRF905_DATA_READY_PIN) {
    NRF905_signal_data_ready(&nrf);
  }
}

static void nrf_impl_carrier_detect_irq(gpio_pin pin) {
  if (pin == NRF905_CARRIER_DETECT_PIN) {
    bool cd = gpio_get(NRF905_CARRIER_DETECT_PORT, NRF905_CARRIER_DETECT_PIN);
    print("radio carrier %s\n", cd ? "UP" : "DOWN");
  }
}

void NRF905_IMPL_init(void) {
  SPI_DEV_init(
      &nrf_spi_dev,
      SPIDEV_CONFIG_SPEED_9M | SPIDEV_CONFIG_CPHA_1E | SPIDEV_CONFIG_CPOL_LO | SPIDEV_CONFIG_FBIT_MSB,
      _SPI_BUS(0),
      gpio_get_hw_port(NRF905_CS_PORT), gpio_get_hw_pin(NRF905_CS_PIN),
      SPI_CONF_IRQ_DRIVEN | SPI_CONF_IRQ_CALLBACK
      );

  NRF905_init(&nrf, &nrf_spi_dev,
      nrf_impl_cb,
      gpio_get_hw_port(NRF905_PWR_PORT), gpio_get_hw_pin(NRF905_PWR_PIN),
      gpio_get_hw_port(NRF905_TRX_CE_PORT), gpio_get_hw_pin(NRF905_TRX_CE_PIN),
      gpio_get_hw_port(NRF905_TX_EN_PORT), gpio_get_hw_pin(NRF905_TX_EN_PIN)
      );

  gpio_config(NRF905_PWR_PORT, NRF905_PWR_PIN, CLK_50MHZ, OUT, AF0, PUSHPULL, NOPULL);
  gpio_config(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, CLK_50MHZ, OUT, AF0, PUSHPULL, NOPULL);
  gpio_config(NRF905_TX_EN_PORT, NRF905_TX_EN_PIN, CLK_50MHZ, OUT, AF0, PUSHPULL, NOPULL);
  gpio_config(NRF905_CS_PORT, NRF905_CS_PIN, CLK_50MHZ, OUT, AF0, PUSHPULL, PULLUP);
  gpio_config(NRF905_CARRIER_DETECT_PORT, NRF905_CARRIER_DETECT_PIN, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
  gpio_config(NRF905_DATA_READY_PORT, NRF905_DATA_READY_PIN, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);

  gpio_interrupt_config(NRF905_CARRIER_DETECT_PORT, NRF905_CARRIER_DETECT_PIN,
      nrf_impl_carrier_detect_irq, FLANK_BOTH);
  gpio_interrupt_config(NRF905_DATA_READY_PORT, NRF905_DATA_READY_PIN,
      nrf_impl_data_ready_irq, FLANK_UP);

  gpio_interrupt_mask_enable(NRF905_CARRIER_DETECT_PORT, NRF905_CARRIER_DETECT_PIN, TRUE);
  gpio_interrupt_mask_enable(NRF905_DATA_READY_PORT, NRF905_DATA_READY_PIN, TRUE);

  NRF905_standby(&nrf);

}

void NRF905_IMPL_read_conf(void) {
  NRF905_standby(&nrf);
  int res = NRF905_read_config(&nrf, &config);
  print("nrf read conf %i\n", res);
}

void NRF905_IMPL_set_conf(nrf905_config *c) {
  nrf905_config spybot_conf;
  if (c == NULL) {
    spybot_conf.auto_retransmit = NRF905_CFG_AUTO_RETRAN_OFF;
    spybot_conf.channel_freq = 128;
    spybot_conf.crc_en = NRF905_CFG_CRC_ON;
    spybot_conf.crc_mode = NRF905_CFG_CRC_MODE_16BIT;
    spybot_conf.crystal_frequency = NRF905_CFG_XTAL_FREQ_16MHZ;
    spybot_conf.hfreq_pll = NRF905_CFG_HFREQ_433;
    spybot_conf.out_clk_enable = NRF905_CFG_OUT_CLK_OFF;
    spybot_conf.out_clk_freq = NRF905_CFG_OUT_CLK_FREQ_1MHZ;
    spybot_conf.pa_pwr= NRF905_CFG_PA_PWR_10;
#ifdef DBG_RADIO
    spybot_conf.rx_address = 0x9ce3aed2;
#else
    spybot_conf.rx_address = 0x631c512d;
#endif
    spybot_conf.rx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
    spybot_conf.rx_payload_width = 16;
    spybot_conf.rx_reduced_power = NRF905_CFG_RX_REDUCED_POWER_OFF;
    spybot_conf.tx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
    spybot_conf.tx_payload_width = 16;

    c = &spybot_conf;
  }

  NRF905_standby(&nrf);
  int res = NRF905_config(&nrf, c);
  print("nrf config %i\n", res);
}

void NRF905_IMPL_set_addr(void) {
  u8_t addr[4] = {0x63, 0x1c, 0x51, 0x2d};

  int res = NRF905_config_tx_address(&nrf, addr);
  print("nrf set addr %i\n", res);
}

void NRF905_IMPL_rx(void) {
  NRF905_standby(&nrf);
  int res = NRF905_rx(&nrf);
  print("nrf rx %i\n", res);
}

void NRF905_IMPL_tx(void) {
  static u8_t pkt = 0;
  NRF905_standby(&nrf);
  u8_t pattern[16] = {0x80, 0x91, 0xa2, 0xb3, 0xc4, 0xd5, 0xe6, 0xf7,
                      0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef };
  pattern[0] = pkt++;
  int res = NRF905_tx(&nrf, pattern, 16);
  print("nrf tx %i\n", res);
}
