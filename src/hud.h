/*
 * hud.h
 *
 *  Created on: Dec 10, 2013
 *      Author: petera
 */

#ifndef HUD_H_
#define HUD_H_

#include "system.h"
#include "gfx_bitmap.h"
#include "lsm303_driver.h"


typedef enum {
  HUD_INIT = 0,
  HUD_MAIN,
  HUD_DBG
} hud_state;

void HUD_init(gcontext *ctx);
void HUD_paint(lsm303_dev *lsm);
void HUD_dbg_print(char *str);
void HUD_state(hud_state state);

#endif /* HUD_H_ */
