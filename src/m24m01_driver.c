/*
 * m24m01_driver.c
 *
 *  Created on: Jan 13, 2014
 *      Author: petera
 */

#include "m24m01_driver.h"

#define M24M01_SIZE         (128*1024)
#define M24M01_HIGH_START   (M24M01_SIZE/2)
#define M24M01_MAX_QUERIES  3

static int _m24m01_read(m24m01_dev *dev, u32_t addr, u8_t *buf, u32_t len);

static void m24m01_cb(i2c_dev *idev, int res) {
  m24m01_dev *dev = (m24m01_dev *)I2C_DEV_get_user_data(idev);

  // check query states
  if (dev->state == M24M01_READ_QUERY || dev->state == M24M01_WRITE_QUERY) {
    if (res < I2C_OK) {
      // query fail
      dev->query_tries++;
      if (dev->query_tries >= M24M01_MAX_QUERIES) {
        res = I2C_ERR_M24M01_UNRESPONSIVE;
        // all queries failed, goto fail handler below
      } else {
        // query again
        res = I2C_DEV_query(idev);
        if (res >= I2C_OK) {
          // query request ok, await next callback
          return;
        } else {
          // query request failed, goto fail handler below
        }
      }
    } else {
      // query ok
    }
  }

  if (res < I2C_OK) {
    // error
    dev->state = M24M01_IDLE;
    if (dev->callback) dev->callback(dev, res);
    return;
  }

  dev->addr += dev->oplen;
  dev->data += dev->oplen;
  dev->len -= dev->oplen;

  if (dev->len == 0) {
    // finished
    dev->state = M24M01_IDLE;
    if (dev->callback) dev->callback(dev, I2C_OK);
    return;
  }

  if (dev->state == M24M01_READ_QUERY || dev->state == M24M01_READ) {
    dev->state = M24M01_READ;
    res = _m24m01_read(dev, dev->addr, dev->data, dev->len);
  } else if (dev->state == M24M01_WRITE_QUERY) {
    // TODO
  } else if (dev->state == M24M01_WRITE) {
    // TODO
  } else {
    res = I2C_ERR_M24M01_BAD_STATE;
  }

  if (res != I2C_OK) {
    // error on next op
    dev->state = M24M01_IDLE;
    if (dev->callback) dev->callback(dev, res);
    return;
  }
}

void m24m01_open(m24m01_dev *dev, i2c_bus *bus, bool e1, bool e2,
    void (*m24m01_callback)(m24m01_dev *dev, int res)) {
  memset(dev, 0, sizeof(m24m01_dev));
  dev->state = M24M01_IDLE;
  const u32_t clock = 100000;
  dev->dev_addr = 0b10100000;
  dev->dev_addr |= e2 ? (0b10000) : 0;
  dev->dev_addr |= e1 ? (0b01000) : 0;
  I2C_DEV_init(&dev->i2c_h, clock, bus, dev->dev_addr | 0b10);
  I2C_DEV_init(&dev->i2c_l, clock, bus, dev->dev_addr);
  I2C_DEV_set_user_data(&dev->i2c_h, dev);
  I2C_DEV_set_user_data(&dev->i2c_l, dev);
  I2C_DEV_set_callback(&dev->i2c_h, m24m01_cb);
  I2C_DEV_set_callback(&dev->i2c_l, m24m01_cb);
  I2C_DEV_open(&dev->i2c_h);
  I2C_DEV_open(&dev->i2c_l);
}

void m24m01_close(m24m01_dev *dev) {
  I2C_DEV_close(&dev->i2c_h);
  I2C_DEV_close(&dev->i2c_l);
}

static int _m24m01_read(m24m01_dev *dev, u32_t addr, u8_t *buf, u32_t len) {
  if (addr > M24M01_SIZE || (addr+len) > M24M01_SIZE) {
    return I2C_ERR_M24M01_ADDRESS_RANGE_BAD;
  }

  int res;

  i2c_dev *adev;
  if (addr >= M24M01_HIGH_START) {
    addr -= M24M01_HIGH_START;
    adev = &dev->i2c_h;
    dev->oplen = len;
  } else {
    adev = &dev->i2c_l;
    dev->oplen = MIN(M24M01_HIGH_START - addr, len);
  }

  if (dev->state == M24M01_READ_QUERY) {
    res = I2C_DEV_query(adev);
  } else if (dev->state == M24M01_READ) {
    dev->tmp[0] = addr >> 8;
    dev->tmp[0] = addr & 0xff;

    dev->seq[0].dir = I2C_DEV_TX;
    dev->seq[0].buf = &dev->tmp[0];
    dev->seq[0].len = 2;
    dev->seq[0].gen_stop = I2C_DEV_RESTART;

    dev->seq[1].dir = I2C_DEV_RX;
    dev->seq[1].buf = buf;
    dev->seq[1].len = dev->oplen;
    dev->seq[1].gen_stop = I2C_DEV_STOP;

    res = I2C_DEV_sequence(adev, &dev->seq[0], 2);
  }

  return res;
}

int m24m01_read(m24m01_dev *dev, u32_t addr, u8_t *buf, u32_t len) {
  if (addr > M24M01_SIZE || (addr+len) > M24M01_SIZE) {
    return I2C_ERR_M24M01_ADDRESS_RANGE_BAD;
  }
  if (dev->state != M24M01_IDLE) {
    return I2C_ERR_DEV_BUSY;
  }
  int res;

  dev->addr = addr;
  dev->data = buf;
  dev->len = len;
  dev->oplen = 0;
  dev->state = M24M01_READ_QUERY;
  dev->query_tries = 0;

  res = _m24m01_read(dev, addr, buf, len);

  if (res != I2C_OK) {
    dev->state = M24M01_IDLE;
  }

  return res;
}

int m24m01_write(m24m01_dev *dev, u32_t addr, u8_t *buf, u32_t len) {
  if (addr > M24M01_SIZE || (addr+len) > M24M01_SIZE) {
    return I2C_ERR_M24M01_ADDRESS_RANGE_BAD;
  }
  if (dev->state != M24M01_IDLE) {
    return I2C_ERR_DEV_BUSY;
  }
  int res;

  dev->addr = addr;
  dev->data = buf;
  dev->len = len;
  dev->oplen = 0;
  dev->state = M24M01_WRITE;
  dev->query_tries = 0;

  // TODO

  if (res != I2C_OK) {
    dev->state = M24M01_IDLE;
  }

  return res;
}
