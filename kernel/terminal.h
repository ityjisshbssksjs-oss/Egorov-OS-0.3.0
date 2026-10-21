#ifndef EGOROV_TERMINAL_H
#define EGOROV_TERMINAL_H

#include "types.h"

#define TERMINAL_WIN_W 600
#define TERMINAL_WIN_H 400

void terminal_init(void);
void terminal_draw(int wx, int wy);
void terminal_key(uint32_t k);
void terminal_click(int wx, int wy, int mx, int my);

#endif
