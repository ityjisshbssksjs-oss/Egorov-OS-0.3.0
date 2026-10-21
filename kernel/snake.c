/* Змейка — классическая игра для Egorov OS.
   Управление: WASD.
   Цель: собирать еду (красный квадрат), не врезаться в стену и в себя.
   Нажмите пробел / Enter / кнопку «Начать заново» для рестарта после смерти. */

#include "snake.h"
#include "fb.h"
#include "text.h"
#include "gui.h"
#include "ps2.h"
#include "string.h"
#include "io.h"

/* ── Directions ── */
#define DIR_UP    0
#define DIR_DOWN  1
#define DIR_LEFT  2
#define DIR_RIGHT 3

/* ── Snake segment ── */
typedef struct { int8_t x, y; } seg_t;

#define MAX_SEG (SNAKE_COLS * SNAKE_ROWS)

static seg_t  g_body[MAX_SEG];
static int    g_len;
static int    g_dir;
static int    g_next_dir;
static int8_t g_food_x, g_food_y;
static bool   g_dead;
static int    g_score;

/* Simple LCG pseudo-random (seed advances each tick). */
static uint32_t g_rand_state = 0xABCD1234u;
static uint32_t rand_next(void) {
    g_rand_state = g_rand_state * 1664525u + 1013904223u;
    return g_rand_state;
}

/* Кнопка «Начать заново» — размер под текст (13 симв. * 8 px = 104 px,
 * кнопка 100 px была тесной). стоит ПОД счётом и подсказкой,
 * на 36 px выше низа окна (28 кнопка + 8 отступ), одна геометрия на
 * отрисовку и клик. */
#define SNK_BTN_W   148
#define SNK_BTN_H   28
#define SNK_BTN_BY(wy) ((wy) + SNAKE_WIN_H - 36)
static int snk_btn_x(int wx) { return wx + (SNAKE_WIN_W - SNK_BTN_W) / 2; }

/* Step counter: we only move the snake every N frames to control speed. */
static uint32_t g_tick_counter;
#define TICKS_PER_STEP 18u   /* lower = faster snake */

static void place_food(void) {
    for (int attempt = 0; attempt < 200; attempt++) {
        int8_t fx = (int8_t)(rand_next() % (uint32_t)SNAKE_COLS);
        int8_t fy = (int8_t)(rand_next() % (uint32_t)SNAKE_ROWS);
        bool ok = true;
        for (int i = 0; i < g_len; i++) {
            if (g_body[i].x == fx && g_body[i].y == fy) { ok = false; break; }
        }
        if (ok) { g_food_x = fx; g_food_y = fy; return; }
    }
    /* Fallback: upper-left corner */
    g_food_x = 0; g_food_y = 0;
}

void snake_init(void) {
    g_len  = 3;
    g_dir  = DIR_RIGHT;
    g_next_dir = DIR_RIGHT;
    g_dead = false;
    g_score = 0;
    g_tick_counter = 0;

    /* Start in the middle */
    int sx = SNAKE_COLS / 2, sy = SNAKE_ROWS / 2;
    g_body[0].x = (int8_t)sx;       g_body[0].y = (int8_t)sy;
    g_body[1].x = (int8_t)(sx-1);   g_body[1].y = (int8_t)sy;
    g_body[2].x = (int8_t)(sx-2);   g_body[2].y = (int8_t)sy;

    g_rand_state ^= (uint32_t)sx * 7919u + (uint32_t)sy;
    place_food();
}

bool snake_tick(void) {
    if (g_dead) return false;
    g_tick_counter++;
    if (g_tick_counter < TICKS_PER_STEP) return false;
    g_tick_counter = 0;

    /* Commit direction (prevent 180-degree reversal). */
    int nd = g_next_dir;
    if ((nd == DIR_UP    && g_dir == DIR_DOWN)  ||
        (nd == DIR_DOWN  && g_dir == DIR_UP)    ||
        (nd == DIR_LEFT  && g_dir == DIR_RIGHT) ||
        (nd == DIR_RIGHT && g_dir == DIR_LEFT)) {
        nd = g_dir; /* ignore */
    }
    g_dir = nd;

    /* Compute new head position. */
    int8_t hx = g_body[0].x, hy = g_body[0].y;
    if (g_dir == DIR_UP)    hy--;
    if (g_dir == DIR_DOWN)  hy++;
    if (g_dir == DIR_LEFT)  hx--;
    if (g_dir == DIR_RIGHT) hx++;

    /* Wall collision. */
    if (hx < 0 || hx >= SNAKE_COLS || hy < 0 || hy >= SNAKE_ROWS) {
        g_dead = true; return true;
    }

    /* Self collision. */
    for (int i = 0; i < g_len - 1; i++) {
        if (g_body[i].x == hx && g_body[i].y == hy) {
            g_dead = true; return true;
        }
    }

    /* Eat food? */
    bool ate = (hx == g_food_x && hy == g_food_y);

    /* Shift body (tail drops unless ate). */
    int new_len = ate ? (g_len < MAX_SEG ? g_len + 1 : g_len) : g_len;
    for (int i = new_len - 1; i > 0; i--) g_body[i] = g_body[i-1];
    g_body[0].x = hx;
    g_body[0].y = hy;
    g_len = new_len;

    if (ate) {
        g_score++;
        place_food();
    }

    return true;
}

void snake_key(uint32_t k) {
    if (g_dead) {
        if (k == ' ' || k == '\n') snake_init();
        return;
    }
    if (k == 'w' || k == 'W') g_next_dir = DIR_UP;
    if (k == 's' || k == 'S') g_next_dir = DIR_DOWN;
    if (k == 'a' || k == 'A') g_next_dir = DIR_LEFT;
    if (k == 'd' || k == 'D') g_next_dir = DIR_RIGHT;
    if (k == ' ' || k == '\n') { /* pause/restart on Enter/Space when alive — do nothing */ }
}

static bool btn_hover;

void snake_click(int wx, int wy, int mx, int my) {
    /* Restart button (always shown, especially useful after death). */
    int bx = snk_btn_x(wx);
    int by = SNK_BTN_BY(wy);
    if (mx >= bx && mx < bx + SNK_BTN_W && my >= by && my < by + SNK_BTN_H) {
        snake_init();
    }
    (void)wx; (void)wy;
}

/* ── Cell palette ── */
static uint32_t body_color(int idx) {
    /* Head is bright green, rest fades slightly. */
    if (idx == 0) return rgb(0x10, 0xC0, 0x10);
    int r = 0x10, g_c = 0x80 + (idx & 1) * 0x20, b = 0x10;
    return rgb(r, g_c, b);
}

void snake_draw(int wx, int wy) {
    int gx = wx + 12;  /* grid origin X */
    int gy = wy + 36;  /* grid origin Y (below title bar) */

    /* Black background for grid. */
    fb_rect(gx, gy, SNAKE_COLS * SNAKE_CELL, SNAKE_ROWS * SNAKE_CELL, rgb(0x10,0x10,0x10));
    fb_rect_outline(gx, gy, SNAKE_COLS * SNAKE_CELL, SNAKE_ROWS * SNAKE_CELL, rgb(0x50,0x50,0x50));

    /* Draw food */
    fb_rect(gx + g_food_x * SNAKE_CELL + 2,
            gy + g_food_y * SNAKE_CELL + 2,
            SNAKE_CELL - 4, SNAKE_CELL - 4, rgb(0xFF,0x30,0x30));
    fb_rect_outline(gx + g_food_x * SNAKE_CELL + 2,
                    gy + g_food_y * SNAKE_CELL + 2,
                    SNAKE_CELL - 4, SNAKE_CELL - 4, rgb(0xFF,0x80,0x80));

    /* Draw snake */
    for (int i = g_len - 1; i >= 0; i--) {
        int cx = gx + g_body[i].x * SNAKE_CELL + 1;
        int cy = gy + g_body[i].y * SNAKE_CELL + 1;
        fb_rect(cx, cy, SNAKE_CELL - 2, SNAKE_CELL - 2, body_color(i));
        if (i == 0) {
            /* Eyes on head based on direction. */
            int ex1 = cx + 3, ey1 = cy + 3;
            int ex2 = cx + SNAKE_CELL - 7, ey2 = cy + 3;
            if (g_dir == DIR_DOWN)  { ey1 = ey2 = cy + SNAKE_CELL - 7; }
            if (g_dir == DIR_LEFT)  { ex1 = ex2 = cx + 3; ey1 = cy+3; ey2 = cy+SNAKE_CELL-7; }
            if (g_dir == DIR_RIGHT) { ex1 = ex2 = cx+SNAKE_CELL-7; ey1=cy+3; ey2=cy+SNAKE_CELL-7; }
            fb_rect(ex1, ey1, 3, 3, COL_BLACK);
            fb_rect(ex2, ey2, 3, 3, COL_BLACK);
        }
    }

    /* Score line */
    int sy = gy + SNAKE_ROWS * SNAKE_CELL + 8;
    char score_str[32];
    {
        const char *s1 = "Счёт: ";
        int p = 0;
        while (s1[p]) { score_str[p] = s1[p]; p++; }
        char num[12]; itoa(g_score, num, 10);
        int i = 0; while (num[i]) { score_str[p++] = num[i++]; }
        score_str[p] = 0;
    }
    text_draw_str(wx + 12, sy, score_str, rgb(0x10,0xC0,0x10));

    /* Help / death overlay */
    if (g_dead) {
        text_draw_str(wx + 12, sy + 18, "ИГРА ОКОНЧЕНА! Пробел = рестарт", rgb(0xFF,0x40,0x40));
    } else {
        text_draw_str(wx + 12, sy + 18, "WASD для управления", rgb(0x90,0x90,0x90));
    }

    /* Restart button */
    int bx = snk_btn_x(wx);
    int by2 = SNK_BTN_BY(wy);

    /* Compute hover */
    mouse_state_t *ms = ps2_mouse();
    btn_hover = (ms->x >= bx && ms->x < bx + SNK_BTN_W &&
                 ms->y >= by2 && ms->y < by2 + SNK_BTN_H);
    gui_button(bx, by2, SNK_BTN_W, SNK_BTN_H,
               "Начать заново", btn_hover, ms->left && btn_hover);
}
