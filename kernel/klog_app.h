#ifndef EGOROV_KLOG_APP_H
#define EGOROV_KLOG_APP_H

#include "types.h"

/* Окно «Журнал ядра (COM1)» — экранная диагностика без serial-порта.
 * Показывает хвост klog-кольца: строки MOUSEHW/POWER с подсветкой. */
#define KLOG_WIN_W 560
#define KLOG_WIN_H 420

/* Нарисовать содержимое окна в позиции (wx, wy) — внутри рамки gui_window. */
void klog_app_draw(int wx, int wy);

/* Клавиши прокрутки: KEY_PGUP/KEY_PGDN/KEY_UP/KEY_DOWN/KEY_HOME/KEY_END.
 * Вызывается из desktop.c, когда окно активно.  Не блокирует. */
void klog_app_key(uint32_t key);

#endif
