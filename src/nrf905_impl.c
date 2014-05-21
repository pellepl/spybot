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
#include "comm_config.h"

static struct {
  nrf905 nrf;
  nrf905_rx rx_cb;
  nrf905_tx tx_cb;
  nrf905_cfg cfg_cb;
  nrf905_err err_cb;
  spi_dev spi_dev;
  nrf905_config config;
  bool carrier_on;
  time last_carrier;
} radio;

static void nrf_impl_cb(nrf905 *nrf, nrf905_state state, int res) {
  DBG(D_RADIO, D_WARN, "nrf cb state:%i res:%i\n", state, res);
  if (res != NRF905_OK) {
    if ((state == NRF905_TX_PRIME || state == NRF905_TX) && radio.tx_cb) {
      radio.tx_cb(res);
    }
    if ((state == NRF905_CONFIG || state == NRF905_QUICK_CONFIG ||  state == NRF905_CONFIG_TX_ADDR) && radio.cfg_cb) {
      radio.cfg_cb(res);
    }
    if (radio.err_cb) {
      radio.err_cb(state, res);
    }
    return;
  }
  switch (state) {
  case NRF905_RX_READ:
    DBG(D_RADIO, D_DEBUG, "nrf got packet\n");
    if (radio.rx_cb) {
      radio.rx_cb(NRF905_RX_PKT_BUFFER(nrf), NRF905_RX_PKT_LEN(nrf));
    }
    break;
  case NRF905_TX:
    DBG(D_RADIO, D_DEBUG, "nrf sent packet\n");
    if (radio.tx_cb) {
      radio.tx_cb(res);
    }
    break;
  case NRF905_CONFIG:
  case NRF905_QUICK_CONFIG:
  case NRF905_CONFIG_TX_ADDR:
    DBG(D_RADIO, D_DEBUG, "nrf configged\n");
    if (radio.cfg_cb) {
      radio.cfg_cb(res);
    }
    break;
  case NRF905_READ_CONFIG:
    DBG(D_RADIO, D_INFO,
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
        radio.config.auto_retransmit,
        radio.config.channel_freq,
        radio.config.crc_en,
        radio.config.crc_mode,
        radio.config.crystal_frequency,
        radio.config.hfreq_pll,
        radio.config.out_clk_enable,
        radio.config.out_clk_freq,
        radio.config.pa_pwr,
        radio.config.rx_address,
        radio.config.rx_address_field_width,
        radio.config.rx_payload_width,
        radio.config.rx_reduced_power,
        radio.config.tx_address_field_width,
        radio.config.tx_payload_width);
    break;
  default:
    break;
  }
}

static void nrf_impl_data_ready_irq(gpio_pin pin) {
  if (pin == NRF905_DATA_READY_PIN) {
    NRF905_signal_data_ready(&radio.nrf);
  }
}

static void nrf_impl_carrier_detect_irq(gpio_pin pin) {
  if (pin == NRF905_CARRIER_DETECT_PIN) {
    bool cd = gpio_get(NRF905_CARRIER_DETECT_PORT, NRF905_CARRIER_DETECT_PIN);
    DBG(D_RADIO, D_DEBUG, "nrf carrier %s\n", cd ? "UP" : "DOWN");
    radio.carrier_on = cd;
    if (!cd) {
      radio.last_carrier = SYS_get_time_ms();
    }
  }
}

void NRF905_IMPL_init(nrf905_rx rx_cb, nrf905_tx tx_cb, nrf905_cfg cfg_cb, nrf905_err err_cb) {
  memset(&radio, 0, sizeof(radio));

  radio.rx_cb = rx_cb;
  radio.tx_cb = tx_cb;
  radio.cfg_cb = cfg_cb;
  radio.err_cb = err_cb;

  SPI_DEV_init(
      &radio.spi_dev,
      SPIDEV_CONFIG_SPEED_9M | SPIDEV_CONFIG_CPHA_1E | SPIDEV_CONFIG_CPOL_LO | SPIDEV_CONFIG_FBIT_MSB,
      _SPI_BUS(0),
      gpio_get_hw_port(NRF905_CS_PORT), gpio_get_hw_pin(NRF905_CS_PIN),
      SPI_CONF_IRQ_DRIVEN | SPI_CONF_IRQ_CALLBACK
      );

  NRF905_init(&radio.nrf, &radio.spi_dev,
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

  NRF905_standby(&radio.nrf);

}

void NRF905_IMPL_status(void) {
  print("NRF905 status\n");
  print("  gpio PWR: %s\n", gpio_get(NRF905_PWR_PORT, NRF905_PWR_PIN) ? "HI":"LO");
  print("  gpio TRX: %s\n", gpio_get(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN) ? "HI":"LO");
  print("  gpio TXE: %s\n", gpio_get(NRF905_TX_EN_PORT, NRF905_TX_EN_PIN) ? "HI":"LO");
  print("  gpio CD : %s\n", gpio_get(NRF905_CARRIER_DETECT_PORT, NRF905_CARRIER_DETECT_PIN) ? "HI":"LO");
  print("  gpio DR : %s\n", gpio_get(NRF905_DATA_READY_PORT, NRF905_DATA_READY_PIN) ? "HI":"LO");
  print("  state   : %i\n", NRF905_get_state(&radio.nrf));
  print("  carrier : %s (last dect:%i ms ago)\n", radio.carrier_on ? "UP  " : "DOWN", SYS_get_time_ms() - radio.last_carrier);
}

bool NRF905_IMPL_is_standby(void) {
  return NRF905_get_state(&radio.nrf) == NRF905_STANDBY;
}

void NRF905_IMPL_read_conf(void) {
  NRF905_standby(&radio.nrf);
  int res = NRF905_read_config(&radio.nrf, &radio.config);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf read conf %i\n", res);
}

const nrf905_config *NRF905_IMPL_get_config(void) {
  return &radio.config;
}

int NRF905_IMPL_conf(nrf905_config *c, bool force) {
  nrf905_config spybot_conf;
  if (c == NULL) {
    spybot_conf.auto_retransmit = NRF905_CFG_AUTO_RETRAN_OFF;
    spybot_conf.channel_freq = 128; // 435.227MHz
    spybot_conf.crc_en = NRF905_CFG_CRC_ON;
    spybot_conf.crc_mode = NRF905_CFG_CRC_MODE_16BIT;
    spybot_conf.crystal_frequency = NRF905_CFG_XTAL_FREQ_16MHZ;
    spybot_conf.hfreq_pll = NRF905_CFG_HFREQ_433;
    spybot_conf.out_clk_enable = NRF905_CFG_OUT_CLK_OFF;
    spybot_conf.out_clk_freq = NRF905_CFG_OUT_CLK_FREQ_1MHZ;
    spybot_conf.pa_pwr= NRF905_CFG_PA_PWR_m2;
#ifdef SECONDARY
    spybot_conf.rx_address = 0x9ce3aed2;
#else
    spybot_conf.rx_address = 0x631c512d;
#endif
    spybot_conf.rx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
    spybot_conf.rx_payload_width = COMM_LNK_MAX_DATA;
    spybot_conf.rx_reduced_power = NRF905_CFG_RX_REDUCED_POWER_OFF;
    spybot_conf.tx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
    spybot_conf.tx_payload_width = COMM_LNK_MAX_DATA;

    c = &spybot_conf;
  }

  if (force && NRF905_get_state(&radio.nrf) != NRF905_STANDBY) {
    (void)NRF905_standby(&radio.nrf);
  }
  int res = NRF905_config(&radio.nrf, c);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf config %i\n", res);
  return res;
}

int NRF905_IMPL_conf_tx_addr(u8_t *addr, bool force) {
  if (force && NRF905_get_state(&radio.nrf) != NRF905_STANDBY) {
    (void)NRF905_standby(&radio.nrf);
  }
  int res = NRF905_config_tx_address(&radio.nrf, addr);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf set addr %i\n", res);
  return res;
}

int NRF905_IMPL_conf_channel(u16_t channel_freq, bool force) {
  if (force && NRF905_get_state(&radio.nrf) != NRF905_STANDBY) {
    (void)NRF905_standby(&radio.nrf);
  }
  int res = NRF905_quick_config_channel(&radio.nrf, channel_freq);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf set channel %i\n", res);
  return res;
}

int NRF905_IMPL_conf_pa(nrf905_cfg_pa_pwr pa, bool force) {
  if (force && NRF905_get_state(&radio.nrf) != NRF905_STANDBY) {
    (void)NRF905_standby(&radio.nrf);
  }
  int res = NRF905_quick_config_pa(&radio.nrf, pa);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf set PA %i\n", res);
  return res;
}

int NRF905_IMPL_rx(void) {
  int res = NRF905_OK;
  nrf905_state st = NRF905_get_state(&radio.nrf);
  if (st == NRF905_CONFIG || st == NRF905_QUICK_CONFIG ||
      st == NRF905_CONFIG_TX_ADDR || st == NRF905_TX_PRIME ||
      st == NRF905_TX) {
    return NRF905_ERR_BUSY;
  }

  if (st != NRF905_RX_LISTEN && st != NRF905_RX_READ) {
    NRF905_standby(&radio.nrf);
    res = NRF905_rx(&radio.nrf);
    if (res == NRF905_OK) {
      radio.last_carrier = SYS_get_time_ms();
    }
    if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf rx %i\n", res);
  }

  return res;
}

int NRF905_IMPL_tx(u8_t *data, u8_t len) {
  int res = NRF905_OK;
  nrf905_state st = NRF905_get_state(&radio.nrf);
  if (st == NRF905_CONFIG || st == NRF905_QUICK_CONFIG ||
      st == NRF905_CONFIG_TX_ADDR || st == NRF905_TX_PRIME ||
      st == NRF905_TX || st == NRF905_RX_READ) {
    return NRF905_ERR_BUSY;
  }

  NRF905_standby(&radio.nrf);
  res = NRF905_tx(&radio.nrf, data, len);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf tx %i\n", res);
  return res;
}

bool NRF905_IMPL_listening(void) {
  nrf905_state st = NRF905_get_state(&radio.nrf);
  return st == NRF905_POWERDOWN || st == NRF905_STANDBY || st == NRF905_RX_LISTEN;
}


int NRF905_IMPL_carrier(void) {
  int res = NRF905_OK;
  nrf905_state st = NRF905_get_state(&radio.nrf);
  if (st == NRF905_CONFIG || st == NRF905_QUICK_CONFIG ||
      st == NRF905_CONFIG_TX_ADDR || st == NRF905_TX_PRIME ||
      st == NRF905_TX || st == NRF905_RX_READ) {
    return NRF905_ERR_BUSY;
  }

  NRF905_standby(&radio.nrf);
  res = NRF905_tx_carrier(&radio.nrf);
  if (res != NRF905_OK) DBG(D_RADIO, D_WARN, "nrf carrier %i\n", res);
  return res;
}

bool NRF905_IMPL_lbt_check_rts(u32_t ms) {
  nrf905_state st = NRF905_get_state(&radio.nrf);
  time now = SYS_get_time_ms();
//  print("lbt: st %i [%i] carrier:%s delta:%i [>%i]",
//      st, NRF905_RX_LISTEN,
//          radio.carrier_on ? "UP" : "DOWN",
//              now - radio.last_carrier,
//              ms);
  if (st != NRF905_RX_LISTEN || radio.carrier_on) {
//    print(" FAIL state:%i\n", st);
    return FALSE;
  }
//  print(" OK\n");
  return (now - radio.last_carrier) >= ms;
}

