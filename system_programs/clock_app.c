/* Clock app — large digital time + date inside its own window. */
#include "clock_app.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "rtc.h"
#include "string.h"

void clock_app_draw(int wx, int wy) {
    rtc_time_t t;
    rtc_read(&t);

    /* Body starts at wy + 36 (under titlebar). */
    int by = wy + 36;
    int bx = wx + 8;
    int bw = CLOCK_WIN_W - 16;
    int bh = CLOCK_WIN_H - 36 - 8;

    /* Inset display area */
    fb_rect(bx, by + 4, bw, bh - 4, rgb(0x16, 0x3D, 0x7B));
    fb_rect_outline(bx, by + 4, bw, bh - 4, rgb(0x05, 0x21, 0x6E));

    /* Big HH:MM:SS — scale 4 (32x64 per char). */
    char hms[12];
    rtc_format_hms(&t, hms);
    int scale = 4;
    int tw = text_width(hms) * scale;
    int tx = bx + (bw - tw) / 2;
    int ty = by + 18;
    text_draw_str_scaled_shadow(tx, ty, hms,
        rgb(0xFF, 0xFF, 0xFF),
        rgb(0x00, 0x00, 0x00), scale);

    /* Date line: Weekday, DD month YYYY */
    char date[12];
    rtc_format_date(&t, date);

    /* Compose: "Воскресенье, 29.04.2026" */
    char line[64];
    const char *wd = rtc_weekday_ru(&t);
    int p = 0;
    while (wd[p - 0] && p < 30) { line[p] = wd[p]; p++; }
    int wlen = p;
    line[p++] = ','; line[p++] = ' ';
    for (int i = 0; date[i]; i++) line[p++] = date[i];
    line[p] = 0;
    (void)wlen;

    int dline_w = text_width(line);
    int dx = bx + (bw - dline_w) / 2;
    int dy = ty + FONT_GLYPH_H * scale + 14;
    text_draw_str_shadow(dx, dy, line, rgb(0xCC, 0xE0, 0xFF), COL_BLACK);
}
