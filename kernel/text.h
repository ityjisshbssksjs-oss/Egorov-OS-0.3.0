#ifndef EGOROV_TEXT_H
#define EGOROV_TEXT_H

#include "types.h"

void text_draw_char(int x, int y, uint32_t cp, uint32_t color);
int  text_draw_str(int x, int y, const char *utf8, uint32_t color);
int  text_width(const char *utf8);
void text_draw_str_shadow(int x, int y, const char *utf8, uint32_t color, uint32_t shadow);

/* Render text scaled by an integer factor (each glyph pixel becomes scale x scale). */
void text_draw_str_scaled(int x, int y, const char *utf8, uint32_t color, int scale);
void text_draw_str_scaled_shadow(int x, int y, const char *utf8,
                                 uint32_t color, uint32_t shadow, int scale);

#endif
