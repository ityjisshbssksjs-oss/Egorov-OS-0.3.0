#include "gui.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "cursor.h"

void gui_window(const window_t *w) {
    fb_rect(w->x + 4, w->y + 4, w->w, w->h, rgb(0, 0, 0));      /* тень */

    fb_rect_outline(w->x, w->y, w->w, w->h, COL_WINDOW_BORDER);
    fb_rect_outline(w->x + 1, w->y + 1, w->w - 2, w->h - 2, COL_WINDOW_BORDER);

    int title_h = 26;
    fb_grad_v(w->x + 2, w->y + 2, w->w - 4, title_h, COL_TITLE_TOP, COL_TITLE_BOT);
    fb_hline(w->x + 2, w->y + 2, w->w - 4, rgb(0x80, 0xB0, 0xF0));

    if (w->title)
        text_draw_str_shadow(w->x + 10, w->y + 6, w->title, COL_WHITE, COL_BLACK);

    int body_y = w->y + 2 + title_h;
    fb_rect(w->x + 2, body_y, w->w - 4, w->h - 4 - title_h, COL_WINDOW_BG);
}

void gui_button(int x, int y, int w, int h, const char *label, bool hover, bool pressed) {
    uint32_t face = pressed ? rgb(0xD2, 0xCE, 0xBE)
                  : (hover   ? rgb(0xFC, 0xF6, 0xDE) : rgb(0xF0, 0xEC, 0xDC));
    fb_rect(x, y, w, h, face);

    /* Фаска: свет сверху и слева, тень снизу и справа. */
    uint32_t hi = rgb(0xFF, 0xFF, 0xFF);
    uint32_t lo = rgb(0x6E, 0x6A, 0x5C);
    fb_hline(x, y, w, hi);
    fb_vline(x, y, h, hi);
    fb_hline(x, y + h - 1, w, lo);
    fb_vline(x + w - 1, y, h, lo);
    fb_rect_outline(x, y, w, h, rgb(0x40, 0x3C, 0x34));

    int tx = x + (w - text_width(label)) / 2;
    int ty = y + (h - FONT_GLYPH_H) / 2;
    if (pressed) { tx++; ty++; }
    text_draw_str(tx, ty, label, COL_TEXT);
}

void gui_panel_inset(int x, int y, int w, int h) {
    fb_rect(x, y, w, h, rgb(0xFF, 0xFF, 0xFF));
    fb_rect_outline(x, y, w, h, rgb(0x80, 0x80, 0x80));
    fb_hline(x, y, w, rgb(0x40, 0x40, 0x40));
    fb_vline(x, y, h, rgb(0x40, 0x40, 0x40));
}

void gui_taskbar(int fb_w, int fb_h, bool start_hover, const char *clock_text,
                 const char *mouse_text, bool mouse_missing) {
    int ty = fb_h - GUI_TASKBAR_H;

    fb_grad_v(0, ty, fb_w, GUI_TASKBAR_H, COL_TASKBAR_TOP, COL_TASKBAR_BOT);
    fb_hline(0, ty, fb_w, COL_TASKBAR_HIGH);
    fb_hline(0, ty + 1, fb_w, rgb(0x40, 0x70, 0xE0));

    /* Кнопка «Пуск». */
    int sw = 100, sh = GUI_TASKBAR_H - 4;
    int sx = 4, sy = ty + 2;
    uint32_t st = start_hover ? COL_START_HOVER_T : COL_START_TOP;
    uint32_t sb = start_hover ? COL_START_HOVER_B : COL_START_BOT;
    fb_grad_v(sx, sy, sw, sh, st, sb);
    fb_hline(sx, sy, sw, rgb(0x8F, 0xD8, 0x8F));
    fb_rect_outline(sx, sy, sw, sh, rgb(0x10, 0x40, 0x10));
    int btn_tx = sx + (sw - text_width("Пуск")) / 2;
    text_draw_str_shadow(btn_tx, sy + (sh - FONT_GLYPH_H) / 2, "Пуск", COL_WHITE, COL_BLACK);

    /* Кармашек с часами. */
    int cw = 106;
    int cx = fb_w - cw - 4;
    fb_grad_v(cx, sy, cw, sh, rgb(0x10, 0x4F, 0xCC), rgb(0x05, 0x2A, 0x80));
    fb_rect_outline(cx, sy, cw, sh, rgb(0x05, 0x21, 0x6E));
    const char *ct = clock_text ? clock_text : "EgorovOS";
    text_draw_str_shadow(cx + (cw - text_width(ct)) / 2,
                         sy + (sh - FONT_GLYPH_H) / 2, ct, COL_WHITE, COL_BLACK);

    /* Статус мыши слева от часов. */
    if (mouse_text) {
        uint32_t mc = mouse_missing ? rgb(0xFF, 0x70, 0x60) : COL_WHITE;
        int mw = text_width(mouse_text);
        int mx = cx - 12 - mw;
        if (mx < sw + 16) mx = sw + 16;
        text_draw_str_shadow(mx, sy + (sh - FONT_GLYPH_H) / 2, mouse_text, mc, COL_BLACK);
    }
}

void gui_cursor(int x, int y) {
    for (int yy = 0; yy < CURSOR_H; yy++) {
        uint16_t m = cursor_mask[yy];
        for (int xx = 0; xx < CURSOR_W; xx++)
            if (m & (1 << (11 - xx))) fb_pixel(x + xx, y + yy, COL_BLACK);
    }
    for (int yy = 0; yy < CURSOR_H; yy++) {
        uint16_t m = cursor_inner[yy];
        for (int xx = 0; xx < CURSOR_W; xx++)
            if (m & (1 << (11 - xx))) fb_pixel(x + xx, y + yy, COL_WHITE);
    }
}

void gui_progress(int x, int y, int w, int h, int percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    fb_rect(x, y, w, h, rgb(0xE8, 0xE8, 0xE8));
    fb_rect_outline(x, y, w, h, rgb(0x40, 0x40, 0x40));
    int fw = (w - 4) * percent / 100;
    int seg = 12;
    int sx = x + 2, sy = y + 2, sh = h - 4;
    for (int i = 0; i < fw / seg; i++)
        fb_grad_v(sx + i * seg, sy, seg - 2, sh,
                  rgb(0x66, 0xC8, 0x66), rgb(0x1F, 0x70, 0x1F));
}

void gui_textinput(int x, int y, int w, int h, const char *buf, bool focused) {
    fb_rect(x, y, w, h, COL_WHITE);
    fb_rect_outline(x, y, w, h, focused ? COL_TITLE_TOP : rgb(0x80, 0x80, 0x80));
    if (focused)
        fb_rect_outline(x + 1, y + 1, w - 2, h - 2, rgb(0x80, 0xA8, 0xE0));

    int ty = y + (h - FONT_GLYPH_H) / 2;
    text_draw_str(x + 4, ty, buf, COL_TEXT);

    if (focused) {
        int cx = x + 4 + text_width(buf);
        if (cx < x + w - 4) fb_rect(cx, ty, 2, FONT_GLYPH_H, rgb(0x10, 0x10, 0x10));
    }
}
