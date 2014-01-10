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
#include "input.h"

typedef enum {
  HUD_INIT = 0,
  HUD_MAIN,
  HUD_CONFIG,
  HUD_DBG,
  HUD_TEST
} hud_state;

void HUD_init(gcontext *ctx);
void HUD_paint(void);
void HUD_dbg_print(gcontext *ctx, char *str);
void HUD_state(hud_state state);
void HUD_vbl(void);
void HUD_input(input_type i, bool change);
hud_state HUD_get_state(void);

void hud_paint_main(gcontext *ctx, bool init);
void hud_paint_dbg(gcontext *ctx, bool init);
void hud_paint_config(gcontext *ctx, bool init);



#endif /* HUD_H_ */
