/*
 * configuration_ee.h
 *
 *  Created on: Jan 14, 2014
 *      Author: petera
 */

#ifndef CONFIGURATION_EE_H_
#define CONFIGURATION_EE_H_

#include "system.h"
#include "configuration.h"

#define CFG_OK              0
#define ERR_CFG_NOT_FOUND   -1250
#define ERR_CFG_NOT_LOADED  -1251

typedef enum {
  IDLE = 0,
  LOAD_CFG,
  STORE_CFG,
} cfg_state;

typedef enum {
  CFG_EE_IDLE = 0,
  CFG_EE_FIND_MAGIC,
  CFG_EE_WRITE_CONFIG,
  CFG_EE_WRITE_MAGIC_FINALIZE,
  CFG_EE_WRITE_MAGIC_DELETE,
} cfg_ee_state;

typedef void (*cfg_ee_cb)(cfg_state state, cfg_ee_state ee_state, int res);

void CFG_EE_init(m24m01_dev *ee_dev, cfg_ee_cb cb);

int CFG_EE_get_config(configuration_t *c);

void CFG_EE_set_config(configuration_t *c);

void CFG_EE_load_config(void);

void CFG_EE_store_config(void);

void CFG_EE_store_default(void);



#endif /* CONFIGURATION_EE_H_ */
