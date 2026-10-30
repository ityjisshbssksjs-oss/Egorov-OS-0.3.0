#ifndef EGOROV_CALC_H
#define EGOROV_CALC_H

#include "types.h"

/* Calculator window content size. */
#define CALC_WIN_W 272
#define CALC_WIN_H 312

void calc_reset(void);

/* Draw display + 5x4 button grid inside the window body.
 *   wx, wy = window top-left (NOT body — body starts at wy + 36).
 *   hover_btn  = button index 0..19 currently hovered, or -1
 *   pressed_btn = button index 0..19 currently pressed (mouse down), or -1 */
void calc_draw(int wx, int wy, int hover_btn, int pressed_btn);

/* Returns button index 0..19 if mouse is over a calc button, else -1. */
int  calc_hit_test(int wx, int wy, int mx, int my);

/* Apply a button click. */
void calc_click(int btn);

/* React to a keyboard character: 0..9, '.', '+', '-', '*', '/',
 * '=' or '\n', 'c'/'C' or 27 (Esc) for clear, 8 (BS) for backspace. */
void calc_key(uint32_t k);

#endif
