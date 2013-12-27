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
  HUD_CONFIG,
  HUD_DBG
} hud_state;

void HUD_init(gcontext *ctx, lsm303_dev *lsm_dev);
void HUD_paint(void);
void HUD_dbg_print(gcontext *ctx, char *str);
void HUD_state(hud_state state);
void HUD_vbl(void);

void hud_paint_main(gcontext *ctx, lsm303_dev *lsm_dev, bool init);
void hud_paint_dbg(gcontext *ctx, bool init);
void hud_paint_config(gcontext *ctx, bool init);


#endif /* HUD_H_ */
