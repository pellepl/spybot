/*
 * nrf905_driver.c
 *
 *  Created on: Nov 17, 2013
 *      Author: petera
 */

#include "nrf905_driver.h"

static void nrf905_spi_dev_cb(spi_dev *d, int res) {
  nrf905 *nrf = (nrf905 *)d->user_data;
  ASSERT(nrf);

  nrf905_state st = nrf->state;
  NRF905_standby(nrf);

  if (res != SPI_OK) {
    if (nrf->cb) {
      nrf->cb(nrf, st, res);
    }
    return;
  }

  switch (st) {

  case NRF905_CONFIG:
    if (nrf->cb) {
      nrf->cb(nrf, st, NRF905_OK);
    }
    break;

  case NRF905_READ_CONFIG: {
    // data in nrf->_buf[1..10], cfg struct in nrf->_scratch
    nrf905_config *cfg = (nrf905_config *)nrf->_scratch;
    u8_t *b = &nrf->_buf[0];

    cfg->channel_freq = b[1] | ((b[2] & 1)<<8);
    cfg->hfreq_pll = (b[2] & 0b00000010) >> 1;
    cfg->pa_pwr = (b[2] & 0b00001100) >> 2;
    cfg->rx_reduced_power = (b[2] & 0b00010000) >> 4;
    cfg->auto_retransmit = (b[2] & 0b00100000) >> 5;
    cfg->tx_address_field_width = (b[3] & 0xf0) >> 4;
    cfg->rx_address_field_width = (b[3] & 0x0f);
    cfg->rx_payload_width = b[4];
    cfg->tx_payload_width = b[5];
    cfg->rx_address = (b[6]<<24) | (b[7]<<16) | (b[8]<<8) | (b[9]);
    cfg->out_clk_freq = (b[10] & 0b00000011);
    cfg->out_clk_enable = (b[10] & 0b00000100) >> 2;
    cfg->crystal_frequency = (b[10] & 0b00111000) >> 3;
    cfg->crc_en = (b[10] & 0b01000000) >> 6;
    cfg->crc_mode = (b[10] & 0b10000000) >> 7;

    if (nrf->cb) {
      nrf->cb(nrf, st, NRF905_OK);
    }
    break;
  }
  default:
    if (nrf->cb) {
      nrf->cb(nrf, st, NRF905_ERR_ILLEGAL_STATE);
    }
    break;
  }
}

int NRF905_powerdown(nrf905 *nrf) {
  if (nrf->state != NRF905_STANDBY) {
    return NRF905_ERR_ILLEGAL_STATE;
  }
  GPIO_set(nrf->pwr_port, 0, nrf->pwr_pin);
  GPIO_set(nrf->trx_ce_port, 0, nrf->trx_ce_pin);
  GPIO_set(nrf->tx_en_port, 0, nrf->tx_en_pin);
  nrf->state = NRF905_POWERDOWN;

  return NRF905_OK;
}

int NRF905_standby(nrf905 *nrf) {
  GPIO_set(nrf->pwr_port, nrf->pwr_pin, 0);
  GPIO_set(nrf->trx_ce_port, 0, nrf->trx_ce_pin);
  GPIO_set(nrf->tx_en_port, 0, nrf->tx_en_pin);
  if (nrf->state == NRF905_POWERDOWN) {
    SYS_hardsleep_us(NRF905_STATE_TRANS_POWDOW_STDBY_US);
  }
  nrf->state = NRF905_STANDBY;
  return NRF905_OK;
}

int NRF905_config(nrf905 *nrf, nrf905_config *cfg) {
  if (cfg->rx_payload_width == 0 || cfg->rx_payload_width > 32 ||
      cfg->tx_payload_width == 0 || cfg->tx_payload_width > 32) {
    return NRF905_ERR_BAD_CONFIG;
  }
  if (nrf->state != NRF905_STANDBY) {
    return NRF905_ERR_ILLEGAL_STATE;
  }

  nrf->state = NRF905_CONFIG;

  memcpy(&nrf->config, cfg, sizeof(nrf905_config));
  memset(&nrf->_buf, 0, 11);

  nrf->_buf[0] = NRF905_SPI_W_CONFIG(0);

  nrf->_buf[1] =
      (cfg->channel_freq & 0xff);
  nrf->_buf[2] =
      (cfg->auto_retransmit << 5) |
      (cfg->rx_reduced_power << 4) |
      (cfg->pa_pwr << 2) |
      (cfg->hfreq_pll << 1) |
      ((cfg->channel_freq>>8) & 0x1);
  nrf->_buf[3] =
      (cfg->tx_address_field_width << 4) |
      (cfg->rx_address_field_width);
  nrf->_buf[4] =
      (cfg->rx_payload_width);
  nrf->_buf[5] =
      (cfg->tx_payload_width);
  nrf->_buf[6] =
      ((cfg->rx_address>>24) & 0xff);
  nrf->_buf[7] =
      ((cfg->rx_address>>16) & 0xff);
  nrf->_buf[8] =
      ((cfg->rx_address>>8) & 0xff);
  nrf->_buf[9] =
      (cfg->rx_address & 0xff);
  nrf->_buf[10] =
      (cfg->crc_mode << 7) |
      (cfg->crc_en << 6) |
      (cfg->crystal_frequency << 3) |
      (cfg->out_clk_enable << 2) |
      (cfg->out_clk_freq);

  SPI_DEV_open(nrf->spi_dev);
  int res = SPI_DEV_txrx(nrf->spi_dev, &nrf->_buf[0], 11, 0, 0);
  if (res != SPI_OK) {
    SPI_DEV_close(nrf->spi_dev);
  }

  return res;
}

int NRF905_read_config(nrf905 *nrf, nrf905_config *cfg) {
  if (nrf->state != NRF905_STANDBY) {
    return NRF905_ERR_ILLEGAL_STATE;
  }

  nrf->state = NRF905_READ_CONFIG;
  memset(&nrf->_buf, 0, 11);
  nrf->_scratch = cfg;

  nrf->_buf[0] = NRF905_SPI_R_CONFIG(0);

  SPI_DEV_open(nrf->spi_dev);
  int res = SPI_DEV_txrx(nrf->spi_dev, &nrf->_buf[0], 1, &nrf->_buf[1], 10);
  if (res != SPI_OK) {
    SPI_DEV_close(nrf->spi_dev);
  }

  return res;
}

void NRF905_init(nrf905 *nrf, spi_dev *spi_dev,
    nrf905_cb cb,
    hw_io_port pwr_port, hw_io_pin pwr_pin,
    hw_io_port trx_ce_port, hw_io_pin trx_ce_pin,
    hw_io_port tx_en_port, hw_io_pin tx_en_pin) {
  memset(nrf, 0, sizeof(nrf905));
  nrf->spi_dev = spi_dev;
  nrf->cb = cb;
  nrf->pwr_port = pwr_port;
  nrf->pwr_pin = pwr_pin;
  nrf->trx_ce_port = trx_ce_port;
  nrf->trx_ce_pin = trx_ce_pin;
  nrf->tx_en_port = tx_en_port;
  nrf->tx_en_pin = tx_en_pin;

  GPIO_set(pwr_port, 0, pwr_pin);
  GPIO_set(trx_ce_port, 0, trx_ce_pin);
  GPIO_set(tx_en_port, 0, tx_en_pin);

  nrf->state = NRF905_POWERDOWN;

  SPI_DEV_set_callback(spi_dev, nrf905_spi_dev_cb);
  SPI_DEV_set_user_data(spi_dev, nrf);
}
