#ifndef EGOROV_NOTEPAD_H
#define EGOROV_NOTEPAD_H

#include "types.h"

#define NOTEPAD_WIN_W  480
#define NOTEPAD_WIN_H  340

void notepad_init(void);
void notepad_clear(void);
void notepad_select_all(void);
void notepad_set_hdd(bool hdd);
void notepad_save_hdd(void);
void notepad_open_hdd(void);
void notepad_draw(int wx, int wy, int hover_btn, int pressed_btn);
int  notepad_hit_btn(int wx, int wy, int mx, int my);
void notepad_key(uint32_t k);
void notepad_mouse_click(int wx, int wy, int mx, int my);

#endif
