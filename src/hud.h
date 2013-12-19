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

void HUD_init(gcontext *ctx);
void HUD_paint(gcontext *ctx, lsm303_dev *lsm);

#endif /* HUD_H_ */
