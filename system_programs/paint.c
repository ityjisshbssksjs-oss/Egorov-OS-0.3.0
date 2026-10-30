/* Paint sketchpad — draw with the mouse, 8-color palette, 3 brush sizes,
 * persistent canvas held in BSS as 8-bit palette indices. */
#include "paint.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "string.h"
#include "io.h"   // для serial_puts

/* Canvas dimensions. 400x300 = 120,000 bytes BSS. */
#define CANVAS_W 400
#define CANVAS_H 300

#define TOOLBAR_H 36
#define SWATCH_W  24
#define SWATCH_H  24

#define PAINT_INNER_PAD 8

#define N_COLORS 8
#define N_TOOLS  12   /* 0..7 colors, 8..10 brush sizes, 11 clear */

/* Palette */
static const uint32_t palette[N_COLORS] = {
    0x000000, 0xFFFFFF, 0xC81818, 0x18A018,
    0x1858E8, 0xF0C800, 0xC008C0, 0x18C8C8
};

static const char *brush_label[3] = { "1", "2", "3" };
static const int   brush_size[3]  = { 1, 3, 6 };

static uint8_t canvas_idx[CANVAS_H][CANVAS_W];
static int  cur_color = 0;     /* palette index, default black */
static int  cur_brush = 1;     /* default medium */
static int  prev_x = -1, prev_y = -1;

void paint_init(void) {
    /* Index 1 = white background. */
    memset(canvas_idx, 1, sizeof(canvas_idx));
    cur_color = 0;
    cur_brush = 1;
    prev_x = -1;
    prev_y = -1;
}

static void canvas_xy(int wx, int wy, int *cx, int *cy) {
    *cx = wx + PAINT_INNER_PAD;
    *cy = wy + 36 + PAINT_INNER_PAD + TOOLBAR_H + PAINT_INNER_PAD;
}

bool paint_in_canvas(int wx, int wy, int mx, int my) {
    int cx, cy; canvas_xy(wx, wy, &cx, &cy);
    return mx >= cx && mx < cx + CANVAS_W && my >= cy && my < cy + CANVAS_H;
}

static void put_pixel_canvas(int px, int py, uint8_t color_idx, int brush) {
    int r = brush;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy > r * r) continue;
            int x = px + dx, y = py + dy;
            if (x < 0 || x >= CANVAS_W || y < 0 || y >= CANVAS_H) continue;
            canvas_idx[y][x] = color_idx;
        }
    }
}

/* Bresenham between (x0,y0)-(x1,y1) on canvas-local coords. */
static void line_canvas(int x0, int y0, int x1, int y1, uint8_t c, int brush) {
    int dx = x1 - x0; if (dx < 0) dx = -dx;
    int dy = y1 - y0; if (dy < 0) dy = -dy;
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int safety = 2000;
    while (safety-- > 0) {
        put_pixel_canvas(x0, y0, c, brush);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

void paint_paint(int wx, int wy, int mx, int my, bool dragging) {
    int cx, cy; canvas_xy(wx, wy, &cx, &cy);
    int lx = mx - cx;
    int ly = my - cy;
    if (lx < 0 || lx >= CANVAS_W || ly < 0 || ly >= CANVAS_H) {
        prev_x = -1; prev_y = -1;
        return;
    }
    if (dragging && prev_x >= 0) {
        line_canvas(prev_x, prev_y, lx, ly,
                    (uint8_t)cur_color, brush_size[cur_brush]);
    } else {
        put_pixel_canvas(lx, ly, (uint8_t)cur_color, brush_size[cur_brush]);
    }
    prev_x = lx; prev_y = ly;
}

void paint_release(void) {
    prev_x = -1;
    prev_y = -1;
}

static void tool_xy(int wx, int wy, int t, int *tx, int *ty, int *tw, int *th) {
    int x0 = wx + PAINT_INNER_PAD;
    int y0 = wy + 36 + PAINT_INNER_PAD;
    if (t < N_COLORS) {
        *tx = x0 + t * (SWATCH_W + 4);
        *ty = y0 + 6;
        *tw = SWATCH_W;
        *th = SWATCH_H;
    } else if (t < N_COLORS + 3) {
        int j = t - N_COLORS;
        *tx = x0 + N_COLORS * (SWATCH_W + 4) + 16 + j * 32;
        *ty = y0 + 6;
        *tw = 28;
        *th = SWATCH_H;
    } else {
        *tw = 110;
        *th = 24;
        *tx = wx + PAINT_WIN_W - 8 - *tw;
        *ty = y0 + 6;
    }
}

int paint_hit_tool(int wx, int wy, int mx, int my) {
    for (int t = 0; t < N_TOOLS; t++) {
        int tx, ty, tw, th;
        tool_xy(wx, wy, t, &tx, &ty, &tw, &th);
        if (mx >= tx && mx < tx + tw && my >= ty && my < ty + th) return t;
    }
    return -1;
}

void paint_tool_click(int tool) {
    if (tool < 0) return;
    if (tool < N_COLORS) cur_color = tool;
    else if (tool < N_COLORS + 3) cur_brush = tool - N_COLORS;
    else if (tool == N_COLORS + 3) {
        memset(canvas_idx, 1, sizeof(canvas_idx));
    }
}

void paint_draw(int wx, int wy, int hover_tool, int pressed_tool) {
    int tbx = wx + PAINT_INNER_PAD;
    int tby = wy + 36 + PAINT_INNER_PAD;
    int tbw = PAINT_WIN_W - 2 * PAINT_INNER_PAD;
    fb_rect(tbx, tby, tbw, TOOLBAR_H, rgb(0xD8, 0xD4, 0xC0));
    fb_rect_outline(tbx, tby, tbw, TOOLBAR_H, rgb(0x60, 0x60, 0x60));

    for (int i = 0; i < N_COLORS; i++) {
        int sx, sy, sw, sh;
        tool_xy(wx, wy, i, &sx, &sy, &sw, &sh);
        fb_rect(sx, sy, sw, sh, palette[i]);
        fb_rect_outline(sx, sy, sw, sh, COL_BLACK);
        if (i == cur_color) {
            fb_rect_outline(sx - 2, sy - 2, sw + 4, sh + 4, rgb(0xFF, 0xCC, 0x00));
            fb_rect_outline(sx - 1, sy - 1, sw + 2, sh + 2, rgb(0xFF, 0xCC, 0x00));
        }
        if (i == hover_tool) {
            fb_rect_outline(sx - 1, sy - 1, sw + 2, sh + 2, COL_WHITE);
        }
    }

    for (int j = 0; j < 3; j++) {
        int t = N_COLORS + j;
        int sx, sy, sw, sh;
        tool_xy(wx, wy, t, &sx, &sy, &sw, &sh);
        bool hover   = (t == hover_tool);
        bool pressed = (t == pressed_tool) || (j == cur_brush);
        gui_button(sx, sy, sw, sh, brush_label[j], hover, pressed);
    }

    {
        int t = N_COLORS + 3;
        int sx, sy, sw, sh;
        tool_xy(wx, wy, t, &sx, &sy, &sw, &sh);
        bool hover   = (t == hover_tool);
        bool pressed = (t == pressed_tool);
        gui_button(sx, sy, sw, sh, "Очистить", hover, pressed);
    }

    int cx, cy; canvas_xy(wx, wy, &cx, &cy);
    fb_rect_outline(cx - 1, cy - 1, CANVAS_W + 2, CANVAS_H + 2,
                    rgb(0x40, 0x40, 0x40));

    for (int y = 0; y < CANVAS_H; y++) {
        int dy = cy + y;
        if (dy < 0 || (uint32_t)dy >= fb.height) continue;
        uint32_t *row = fb.pixels + (uint32_t)dy * fb.width + cx;
        const uint8_t *src = canvas_idx[y];
        for (int x = 0; x < CANVAS_W; x++) {
            int dx = cx + x;
            if (dx < 0 || (uint32_t)dx >= fb.width) continue;
            row[x] = palette[src[x] & 7];
        }
    }
}
