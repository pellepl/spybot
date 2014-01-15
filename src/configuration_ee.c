/*
 * configuration.c
 *
 *  Created on: Jan 11, 2014
 *      Author: petera
 */

#include "configuration.h"
#include "app.h"
#include "m24m01_driver.h"
#include "taskq.h"
#include "configuration_ee.h"

#define CFG_EE_ADDRESS    0x00000000
#define CFG_EE_LEN        (16*1024)
#define CFG_EE_BLOCK_LEN  1024
#define CFG_EE_MAGIC_OK   0xfa1105a8
#define CFG_EE_MAGIC_DEL  0xb08ed2d7
#define CFG_EE_MAGIC_UPD  0xdeadc0de
#define CFG_EE_NO_ADDR    0xffffffff

typedef struct {
  u32_t magic;
  configuration_t cfg;
} ee_cfg;

static struct {
  cfg_ee_cb ee_cb;
  u32_t ee_cur_state_addr;
  cfg_ee_state ee_cur_state;
  ee_cfg ee_struct;
  u32_t ee_arg;
  m24m01_dev *ee_dev;
  u32_t ee_cur_addr;
  u32_t ee_len;
  u8_t *ee_buf;
} cfg;

static void cfg_ee_read(u32_t addr, u8_t *buf, u32_t len);

static void cfg_ee_read_task_fn(u32_t arg, void *argp) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    print("cfg_ee read stalled\n");
    return;
  }
  print("cfg_ee read exe %08x %i\n", cfg.ee_cur_addr, cfg.ee_len);
  int res = m24m01_read(cfg.ee_dev, cfg.ee_cur_addr, cfg.ee_buf, cfg.ee_len);
  if (res != I2C_OK) {
    TASK_mutex_unlock(&i2c_mutex);
    print("cfg_ee read failed %i\n", res);
    if (cfg.ee_cb) {
      cfg.ee_cb(cfg.ee_cur_state, res);
    }
  }
}


static void cfg_ee_cb_irq(m24m01_dev *dev, int res) {
  TASK_mutex_unlock(&i2c_mutex);
  print("cfg_ee cb res %i\n", res);

  if (res < I2C_OK) {
    if (cfg.ee_cb) {
      cfg.ee_cb(cfg.ee_cur_state, res);
    }
    return;
  }

  switch (cfg.ee_cur_state) {
  case CFG_EE_FIND_MAGIC:
    if (cfg.ee_struct.magic == cfg.ee_arg) {
      cfg.ee_cur_state_addr = cfg.ee_cur_addr;
      if (cfg.ee_cb) {
        cfg.ee_cb(cfg.ee_cur_state, CFG_OK);
      }
    } else {
      cfg.ee_cur_addr += CFG_EE_BLOCK_LEN;

      if (cfg.ee_cur_addr >= CFG_EE_ADDRESS + CFG_EE_LEN) {
        if (cfg.ee_cb) {
          cfg.ee_cb(cfg.ee_cur_state, ERR_CFG_NOT_FOUND);
        }
        cfg.ee_cur_state_addr = CFG_EE_NO_ADDR;
      } else {
        // keep same params, addr advanced
        cfg_ee_read(cfg.ee_cur_addr, cfg.ee_buf, cfg.ee_len);
      }
    }
    break;
  default:
    ASSERT(FALSE);
    break;
  }
}

static void cfg_ee_read(u32_t addr, u8_t *buf, u32_t len) {
  cfg.ee_cur_addr = addr;
  cfg.ee_buf = buf;
  cfg.ee_len = len;
  print("cfg_ee read request %08x %i\n", addr, len);
  task *t = TASK_create(cfg_ee_read_task_fn, 0);
  ASSERT(t);
  TASK_run(t, 0, 0);
}

static void cfg_find_magic(u32_t magic) {
  cfg.ee_cur_state = CFG_EE_FIND_MAGIC;
  cfg.ee_arg = magic;
  cfg_ee_read(CFG_EE_ADDRESS, (u8_t *)&cfg.ee_struct, sizeof(ee_cfg));
}

void CFG_init(m24m01_dev *ee_dev, cfg_ee_cb cb) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.ee_cb = cb;
  cfg.ee_dev = ee_dev;
  cfg.ee_cur_state_addr = CFG_EE_NO_ADDR;
  m24m01_open(cfg.ee_dev, _I2C_BUS(0), FALSE, FALSE, cfg_ee_cb_irq);
}

void CFG_load_settings(void) {
  cfg_find_magic(CFG_EE_MAGIC_OK);
}
