#include "text.h"
#include "font.h"
#include "fb.h"

static int find_glyph(uint32_t cp) {
    for (int i = 0; i < FONT_LOOKUP_COUNT; i++) {
        if (font_lookup_cp[i] == cp) return font_lookup_idx[i];
    }
    return -1;
}

void text_draw_char(int x, int y, uint32_t cp, uint32_t color) {
    int idx = find_glyph(cp);
    if (idx < 0) idx = find_glyph('?');
    if (idx < 0) return;
    const uint8_t *g = font_glyphs[idx];
    for (int yy = 0; yy < FONT_GLYPH_H; yy++) {
        uint8_t row = g[yy];
        for (int xx = 0; xx < FONT_GLYPH_W; xx++) {
            if (row & (0x80 >> xx)) fb_pixel(x + xx, y + yy, color);
        }
    }
}

/* Decode one UTF-8 codepoint, advance *p. Returns 0 on end. */
static uint32_t utf8_next(const char **p) {
    const uint8_t *s = (const uint8_t*)(*p);
    if (!*s) return 0;
    uint32_t cp;
    int extra;
    if (s[0] < 0x80) { cp = s[0]; extra = 0; }
    else if ((s[0] & 0xE0) == 0xC0) { cp = s[0] & 0x1F; extra = 1; }
    else if ((s[0] & 0xF0) == 0xE0) { cp = s[0] & 0x0F; extra = 2; }
    else if ((s[0] & 0xF8) == 0xF0) { cp = s[0] & 0x07; extra = 3; }
    else { cp = '?'; extra = 0; }
    s++;
    for (int i = 0; i < extra; i++) {
        if ((s[0] & 0xC0) != 0x80) { cp = '?'; break; }
        cp = (cp << 6) | (s[0] & 0x3F);
        s++;
    }
    *p = (const char*)s;
    return cp;
}

int text_draw_str(int x, int y, const char *utf8, uint32_t color) {
    int dx = x;
    while (*utf8) {
        uint32_t cp = utf8_next(&utf8);
        if (!cp) break;
        text_draw_char(dx, y, cp, color);
        dx += FONT_GLYPH_W;
    }
    return dx - x;
}

int text_width(const char *utf8) {
    int n = 0;
    while (*utf8) {
        uint32_t cp = utf8_next(&utf8);
        if (!cp) break;
        n++;
    }
    return n * FONT_GLYPH_W;
}

void text_draw_str_shadow(int x, int y, const char *utf8, uint32_t color, uint32_t shadow) {
    text_draw_str(x + 1, y + 1, utf8, shadow);
    text_draw_str(x, y, utf8, color);
}

static void draw_char_scaled(int x, int y, uint32_t cp, uint32_t color, int scale) {
    int idx = find_glyph(cp);
    if (idx < 0) idx = find_glyph('?');
    if (idx < 0) return;
    const uint8_t *g = font_glyphs[idx];
    for (int yy = 0; yy < FONT_GLYPH_H; yy++) {
        uint8_t row = g[yy];
        for (int xx = 0; xx < FONT_GLYPH_W; xx++) {
            if (row & (0x80 >> xx)) {
                int px = x + xx * scale;
                int py = y + yy * scale;
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        fb_pixel(px + sx, py + sy, color);
            }
        }
    }
}

void text_draw_str_scaled(int x, int y, const char *utf8, uint32_t color, int scale) {
    if (scale < 1) scale = 1;
    int dx = x;
    while (*utf8) {
        uint32_t cp = utf8_next(&utf8);
        if (!cp) break;
        draw_char_scaled(dx, y, cp, color, scale);
        dx += FONT_GLYPH_W * scale;
    }
}

void text_draw_str_scaled_shadow(int x, int y, const char *utf8,
                                 uint32_t color, uint32_t shadow, int scale) {
    if (scale < 1) scale = 1;
    text_draw_str_scaled(x + scale, y + scale, utf8, shadow, scale);
    text_draw_str_scaled(x, y, utf8, color, scale);
}
