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
#define CFG_EE_BLOCK_LEN  256
#define CFG_EE_MAGIC_OK   0xfa1105a8
#define CFG_EE_MAGIC_DEL  0xb08ed2d7
#define CFG_EE_MAGIC_UPD  0xdeadc0de
#define CFG_EE_NO_ADDR    0xffffffff

#define EE_DBG(x, ...) DBG(D_APP, D_DEBUG, "CFGEE:"x, ## __VA_ARGS__)

const static u32_t magic_del[1] = {CFG_EE_MAGIC_DEL};
const static u32_t magic_ok[1] = {CFG_EE_MAGIC_OK};

typedef struct {
  u32_t magic;
  configuration_t cfg;
} ee_cfg;

static struct {
  cfg_state state;
  cfg_ee_state ee_state;
  cfg_ee_cb cb;
  u32_t cur_cfg_addr;
  u32_t new_cfg_addr;
  ee_cfg config;
  m24m01_dev *ee_dev;

  u32_t ee_magic_arg;
  u32_t ee_cur_addr;
  u32_t ee_len;
  u8_t *ee_buf;
} cfg;

static void cfg_ee_read(u32_t addr, u8_t *buf, u32_t len);
static void cfg_ee_write(u32_t addr, const u8_t *buf, u32_t len);

static void cfg_ee_read_task_fn(u32_t arg, void *argp) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    EE_DBG("read stalled\n");
    return;
  }
  EE_DBG("read exe %08x %i\n", cfg.ee_cur_addr, cfg.ee_len);
  int res = m24m01_read(cfg.ee_dev, cfg.ee_cur_addr, cfg.ee_buf, cfg.ee_len);
  if (res != I2C_OK) {
    TASK_mutex_unlock(&i2c_mutex);
    EE_DBG("read failed %i\n", res);
    if (cfg.cb) {
      cfg.cb(cfg.state, cfg.ee_state, res);
    }
  }
}

static void cfg_ee_write_task_fn(u32_t arg, void *argp) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    EE_DBG("write stalled\n");
    return;
  }
  EE_DBG("write exe %08x %i\n", cfg.ee_cur_addr, cfg.ee_len);
  int res = m24m01_write(cfg.ee_dev, cfg.ee_cur_addr, cfg.ee_buf, cfg.ee_len);
  if (res != I2C_OK) {
    TASK_mutex_unlock(&i2c_mutex);
    EE_DBG("write failed %i\n", res);
    if (cfg.cb) {
      cfg.cb(cfg.state, cfg.ee_state, res);
    }
  }
}


static void cfg_ee_cb_irq(m24m01_dev *dev, int res) {
  TASK_mutex_unlock(&i2c_mutex);
  EE_DBG("cb res %i, state %i/%i\n", res, cfg.state, cfg.ee_state);

  if (res < I2C_OK) {
    if (cfg.cb) {
      cfg.cb(cfg.state, cfg.ee_state, res);
    }
    return;
  }

  switch (cfg.ee_state) {
  case CFG_EE_FIND_MAGIC:
    if (cfg.config.magic == cfg.ee_magic_arg) {
      // found
      cfg.cur_cfg_addr = cfg.ee_cur_addr;
      EE_DBG("cb found %08x\n", cfg.cur_cfg_addr);
      if (cfg.cb) {
        cfg.cb(cfg.state, cfg.ee_state, CFG_OK);
      }
      cfg.ee_state = CFG_EE_IDLE;
    } else {
      cfg.ee_cur_addr += CFG_EE_BLOCK_LEN;

      if (cfg.ee_cur_addr >= CFG_EE_ADDRESS + CFG_EE_LEN) {
        // not found
        cfg.cur_cfg_addr = CFG_EE_NO_ADDR;
        if (cfg.state == LOAD_CFG) {
          // tried load current setting, failed, so create default and store
          CFG_store_default();
        } else {
          // magic not found, fail
          if (cfg.cb) {
            cfg.cb(cfg.state, cfg.ee_state, ERR_CFG_NOT_FOUND);
          }
          cfg.ee_state = CFG_EE_IDLE;
        }
      } else {
        // keep same params, addr advanced
        cfg_ee_read(cfg.ee_cur_addr, cfg.ee_buf, cfg.ee_len);
      }
    }
    break;
  case CFG_EE_WRITE_CONFIG:
    EE_DBG("cb finalize %08x\n", cfg.new_cfg_addr);
    cfg.ee_state = CFG_EE_WRITE_MAGIC_FINALIZE;
    cfg_ee_write(cfg.new_cfg_addr, (u8_t *)&magic_ok, sizeof(magic_ok));
    break;
  case CFG_EE_WRITE_MAGIC_FINALIZE:
    if (cfg.cur_cfg_addr != CFG_EE_NO_ADDR) {
      EE_DBG("cb delete %08x\n", cfg.cur_cfg_addr);
      cfg.ee_state = CFG_EE_WRITE_MAGIC_DELETE;
      cfg_ee_write(cfg.cur_cfg_addr, (u8_t *)&magic_del, sizeof(magic_del));
      break;
    } else {
      EE_DBG("cb no old cfg to delete\n");
    }
    //no break
  case CFG_EE_WRITE_MAGIC_DELETE:
    EE_DBG("cb new cfg stored @ %08x\n", cfg.new_cfg_addr);
    cfg.cur_cfg_addr = cfg.new_cfg_addr;
    cfg.new_cfg_addr = CFG_EE_NO_ADDR;
    if (cfg.cb) {
      cfg.cb(cfg.state, cfg.ee_state, CFG_OK);
    }
    cfg.ee_state = CFG_EE_IDLE;
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
  EE_DBG("read request %08x %i\n", addr, len);
  task *t = TASK_create(cfg_ee_read_task_fn, 0);
  ASSERT(t);
  TASK_run(t, 0, 0);
}

static void cfg_ee_write(u32_t addr, const u8_t *buf, u32_t len) {
  cfg.ee_cur_addr = addr;
  cfg.ee_buf = (u8_t *)buf;
  cfg.ee_len = len;
  EE_DBG("write request %08x %i\n", addr, len);
  task *t = TASK_create(cfg_ee_write_task_fn, 0);
  ASSERT(t);
  TASK_run(t, 0, 0);
}

static void cfg_find_magic(u32_t magic) {
  cfg.ee_state = CFG_EE_FIND_MAGIC;
  cfg.ee_magic_arg = magic;
  cfg_ee_read(CFG_EE_ADDRESS, (u8_t *)&cfg.config, sizeof(ee_cfg));
}

static void cfg_set_default(void) {
  cfg.config.cfg.main.cam_pan_adjust = 0;
  cfg.config.cfg.main.cam_tilt_adjust = 0;
  cfg.config.cfg.main.radar_adjust = 0;
  cfg.config.cfg.main.steer_adjust = 0;
  cfg.config.cfg.main.common = 0;

  cfg.config.cfg.accel.acc_x_min = -1064;
  cfg.config.cfg.accel.acc_x_max = 1051;
  cfg.config.cfg.accel.acc_y_min = -1003;
  cfg.config.cfg.accel.acc_y_max = 1041;
  cfg.config.cfg.accel.acc_z_min = -1012;
  cfg.config.cfg.accel.acc_z_max = 1143;

  cfg.config.cfg.magneto.mag_x_min = -780;
  cfg.config.cfg.magneto.mag_x_max = 346;
  cfg.config.cfg.magneto.mag_y_min = -542;
  cfg.config.cfg.magneto.mag_y_max = 547;
  cfg.config.cfg.magneto.mag_z_min = -542;
  cfg.config.cfg.magneto.mag_z_max = 426;

  cfg.config.cfg.radio.radio_channel = 128;
}

void CFG_init(m24m01_dev *ee_dev, cfg_ee_cb cb) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.cb = cb;
  cfg.ee_dev = ee_dev;
  cfg.cur_cfg_addr = CFG_EE_NO_ADDR;
  m24m01_open(cfg.ee_dev, _I2C_BUS(0), FALSE, FALSE, cfg_ee_cb_irq);
}

void CFG_load_config(void) {
  cfg.cur_cfg_addr = CFG_EE_NO_ADDR;
  cfg.state = LOAD_CFG;
  cfg_find_magic(CFG_EE_MAGIC_OK);
}

void CFG_store_config(void) {
  cfg.state = STORE_CFG;
  u32_t addr;
  if (cfg.cur_cfg_addr == CFG_EE_NO_ADDR) {
    // indicates there is no config stored yet, set default
    addr = CFG_EE_ADDRESS;
  } else {
    // pick next config slot aside to current
    addr = cfg.cur_cfg_addr + CFG_EE_BLOCK_LEN;
    if (addr >= CFG_EE_ADDRESS + CFG_EE_LEN) {
      addr -= CFG_EE_LEN;
    }
  }

  cfg.config.magic = CFG_EE_MAGIC_UPD;
  cfg.new_cfg_addr = addr;
  cfg.ee_state = CFG_EE_WRITE_CONFIG;
  cfg_ee_write(addr, (u8_t *)&cfg.config, sizeof(ee_cfg));
}

void CFG_set_config(configuration_t *c) {
  memcpy(&cfg.config.cfg, c, sizeof(configuration_t));
}

int CFG_get_config(configuration_t *c) {
  if (cfg.cur_cfg_addr == CFG_EE_NO_ADDR) {
    return ERR_CFG_NOT_LOADED;
  }
  memcpy(c, &cfg.config.cfg, sizeof(configuration_t));
  return CFG_OK;
}

void CFG_store_default(void) {
  cfg_set_default();
  CFG_store_config();
}
