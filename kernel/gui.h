#ifndef EGOROV_GUI_H
#define EGOROV_GUI_H

#include "types.h"

/* Палитра в духе Windows XP (тема Luna). */
#define COL_TASKBAR_TOP     rgb(0x24, 0x5E, 0xDC)
#define COL_TASKBAR_BOT     rgb(0x05, 0x36, 0xA8)
#define COL_TASKBAR_HIGH    rgb(0x59, 0x86, 0xE2)
#define COL_START_TOP       rgb(0x3D, 0x95, 0x3D)
#define COL_START_BOT       rgb(0x12, 0x60, 0x12)
#define COL_START_HOVER_T   rgb(0x6F, 0xC1, 0x6F)
#define COL_START_HOVER_B   rgb(0x1F, 0x80, 0x1F)
#define COL_TITLE_TOP       rgb(0x32, 0x6B, 0xCE)
#define COL_TITLE_BOT       rgb(0x05, 0x36, 0xA8)
#define COL_WINDOW_BG       rgb(0xEC, 0xE9, 0xD8)
#define COL_WINDOW_BORDER   rgb(0x05, 0x36, 0xA8)
#define COL_WHITE           rgb(0xFF, 0xFF, 0xFF)
#define COL_BLACK           rgb(0x00, 0x00, 0x00)
#define COL_TEXT            rgb(0x10, 0x10, 0x10)

/* Высота панели задач; одна константа для gui_taskbar() и рабочего стола. */
#define GUI_TASKBAR_H 32

typedef struct {
    int x, y, w, h;
    const char *title;
} window_t;

/* Окно с тенью, рамкой и заголовком; кнопка закрытия рисуется отдельно
 * (desktop.c), чтобы подсветить наведение и нажатие. */
void gui_window(const window_t *w);

void gui_button(int x, int y, int w, int h, const char *label, bool hover, bool pressed);

/* Панель задач: «Пуск», статус мыши и часы.  mouse_text NULL — не рисовать;
 * mouse_missing — статус красится тревожным цветом. */
void gui_taskbar(int fb_w, int fb_h, bool start_hover, const char *clock_text,
                 const char *mouse_text, bool mouse_missing);

void gui_cursor(int x, int y);
void gui_progress(int x, int y, int w, int h, int percent);
void gui_panel_inset(int x, int y, int w, int h);

/* Однострочное поле ввода; focused — рисовать курсор. */
void gui_textinput(int x, int y, int w, int h, const char *buf, bool focused);

#endif
