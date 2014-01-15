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

#define CFG_OK            0
#define ERR_CFG_NOT_FOUND -1250

typedef enum {
  CFG_EE_IDLE = 0,
  CFG_EE_FIND_MAGIC,
} cfg_ee_state;

typedef void (*cfg_ee_cb)(cfg_ee_state state, int res);

void CFG_init(m24m01_dev *ee_dev, cfg_ee_cb cb);

void CFG_load_settings(void);


#endif /* CONFIGURATION_EE_H_ */
