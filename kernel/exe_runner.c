/* EXE Runner for Egorov OS.
 * Simulates a Windows PE loader console window.
 * Known EXEs (calc, clock, paint, notepad, snake) launch real built-in apps.
 * Unknown EXEs show a compatibility notice.
 */
#include "exe_runner.h"
#include "fb.h"
#include "text.h"
#include "gui.h"
#include "string.h"

#define TOTAL_TICKS   52
#define PHASE1_END    13
#define PHASE2_END    26
#define PHASE3_END    39

/* Console palette */
#define CON_BG    rgb(0x0C, 0x0C, 0x0C)
#define CON_FRAME rgb(0x40, 0x40, 0x40)
#define CON_TEXT  rgb(0xCC, 0xCC, 0xCC)
#define CON_GREEN rgb(0x16, 0xC6, 0x0C)
#define CON_CYAN  rgb(0x3A, 0x96, 0xDD)
#define CON_WARN  rgb(0xF9, 0xF1, 0xA5)
#define CON_ERR   rgb(0xE7, 0x48, 0x56)

static char         g_name[36];
static exe_target_t g_target;
static int          g_tick = 0;
static bool         g_done = false;

/* PE metadata per target */
static const char *pe_entry_tbl[] = {
    "0x00401A80",  /* calc    */
    "0x00401340",  /* clock   */
    "0x00402B10",  /* paint   */
    "0x00401200",  /* notepad */
    "0x00401050",  /* snake   */
    "0x00401000",  /* unknown */
};
static const char *pe_size_tbl[] = {
    "38 КБ", "22 КБ", "54 КБ", "31 КБ", "19 КБ", "--"
};

void exe_runner_start(const char *name, exe_target_t target) {
    int i = 0;
    while (name[i] && i < 35) { g_name[i] = name[i]; i++; }
    g_name[i]   = 0;
    g_target    = target;
    g_tick      = 0;
    g_done      = false;
}

void exe_runner_tick(void) {
    if (!g_done) {
        g_tick++;
        if (g_tick >= TOTAL_TICKS) g_done = true;
    }
}

bool         exe_runner_done(void)       { return g_done; }
exe_target_t exe_runner_get_target(void) { return g_target; }

/* Helper: append src to dst (null-terminated), max = dst buffer size */
static void str_append(char *dst, const char *src, int max) {
    int i = 0;
    while (dst[i]) i++;
    for (int j = 0; src[j] && i < max - 1; j++, i++)
        dst[i] = src[j];
    dst[i] = 0;
}

/* Draw one console row and advance cy */
static void con_row(int cx, int *cy, const char *text, uint32_t col) {
    text_draw_str(cx, *cy, text, col);
    *cy += 13;
}

void exe_runner_draw(int wx, int wy) {
    int ww = EXE_WIN_W;
    int wh = EXE_WIN_H;

    /* Console body */
    fb_rect(wx + 2, wy + 28, ww - 4, wh - 30, CON_BG);
    fb_rect_outline(wx + 2, wy + 28, ww - 4, wh - 30, CON_FRAME);

    int tgt = (g_target < EXE_TARGET_UNKNOWN) ? (int)g_target : 5;
    int cx = wx + 8;
    int cy = wy + 33;

    /* Header line */
    {
        char hdr[80]; hdr[0] = 0;
        str_append(hdr, "C:\\Программы\\", 80);
        str_append(hdr, g_name, 80);
        con_row(cx, &cy, hdr, CON_CYAN);
    }
    cy += 2;

    /* Static PE info */
    {
        char ln[64]; ln[0] = 0;
        str_append(ln, "[PE] Файл:        ", 64);
        str_append(ln, g_name, 64);
        con_row(cx, &cy, ln, CON_TEXT);
    }
    {
        char ln[64]; ln[0] = 0;
        str_append(ln, "[PE] Размер:      ", 64);
        str_append(ln, pe_size_tbl[tgt], 64);
        con_row(cx, &cy, ln, CON_TEXT);
    }
    con_row(cx, &cy, "[PE] Платформа:   i386  PE32", CON_TEXT);
    {
        char ln[64]; ln[0] = 0;
        str_append(ln, "[PE] Точка входа: ", 64);
        str_append(ln, pe_entry_tbl[tgt], 64);
        con_row(cx, &cy, ln, CON_TEXT);
    }
    con_row(cx, &cy, "[PE] Секции:      .text .data .rdata .rsrc", CON_TEXT);
    cy += 3;

    /* Phase-dependent lines */
    if (g_tick >= 0) {
        uint32_t c = (g_tick < PHASE1_END) ? CON_WARN : CON_GREEN;
        con_row(cx, &cy, "[LD] Чтение PE-заголовка .......  OK", c);
    }
    if (g_tick >= PHASE1_END) {
        uint32_t c = (g_tick < PHASE2_END) ? CON_WARN : CON_GREEN;
        con_row(cx, &cy, "[LD] Загрузка секций .text .data . OK", c);
    }
    if (g_tick >= PHASE2_END) {
        uint32_t c = (g_tick < PHASE3_END) ? CON_WARN : CON_GREEN;
        con_row(cx, &cy, "[LD] Инициализация импортов .....  OK", c);
    }
    if (g_tick >= PHASE3_END) {
        if (g_target == EXE_TARGET_UNKNOWN) {
            con_row(cx, &cy, "[!!] Win32 API не поддерживается.", CON_ERR);
        } else {
            uint32_t c = g_done ? CON_GREEN : CON_WARN;
            con_row(cx, &cy, "[LD] Передача управления ........  OK", c);
        }
    }

    /* Progress bar + label */
    int pct = g_tick * 100 / TOTAL_TICKS;
    if (pct > 100) pct = 100;

    int bar_y = wy + wh - 44;
    fb_hline(wx + 4, bar_y - 6, ww - 8, rgb(0x30, 0x30, 0x30));

    if (g_done && g_target != EXE_TARGET_UNKNOWN) {
        text_draw_str(cx, bar_y - 4, "Программа запущена!", CON_GREEN);
    } else if (g_done && g_target == EXE_TARGET_UNKNOWN) {
        text_draw_str(cx, bar_y - 4,
            "Ошибка: несовместимая программа (Win32 API не реализован).", CON_ERR);
    } else {
        char pbuf[32];
        int p = 0;
        const char *s = "Загрузка:  ";
        while (s[p]) { pbuf[p] = s[p]; p++; }
        char num[8]; itoa(pct, num, 10);
        for (int i = 0; num[i]; i++) pbuf[p++] = num[i];
        pbuf[p++] = '%'; pbuf[p] = 0;
        text_draw_str(cx, bar_y - 4, pbuf, CON_TEXT);
    }

    gui_progress(wx + 4, bar_y + 8, ww - 8, 14, pct);
}
