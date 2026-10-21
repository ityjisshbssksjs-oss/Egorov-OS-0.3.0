#ifndef EGOROV_SNAKE_H
#define EGOROV_SNAKE_H

#include "types.h"

#define SNAKE_COLS   20
#define SNAKE_ROWS   15
#define SNAKE_CELL   16

/* +36 заголовок, +120 нижняя панель: счёт, подсказка и кнопка «Начать
 * заново» — каждая на своей строке, кнопка НЕ налезает на поле. */
#define SNAKE_WIN_W  (SNAKE_COLS * SNAKE_CELL + 24)
#define SNAKE_WIN_H  (SNAKE_ROWS * SNAKE_CELL + 120)

void snake_init(void);
void snake_draw(int wx, int wy);
void snake_key(uint32_t k);
/* Returns true if the game state changed (step occurred). */
bool snake_tick(void);
/* Mouse clicks on the window (restart button). */
void snake_click(int wx, int wy, int mx, int my);

#endif
