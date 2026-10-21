#ifndef EGOROV_EXPLORER_H
#define EGOROV_EXPLORER_H

#include "types.h"

#define EXPLORER_WIN_W  600
#define EXPLORER_WIN_H  400

/* Set by explorer_click when a .exe file is double-clicked.
 * -1 = nothing launched; >= 0 = exe_target_t value (see exe_runner.h).
 * Desktop resets this to -1 after handling. */
extern int  explorer_launched_exe;
extern char explorer_exe_name[36];

void explorer_init(bool booted_from_hdd);
void explorer_open_trash(void);
void explorer_draw(int wx, int wy, int mx, int my, bool left_down);
int  explorer_hit_close(int wx, int wy, int mx, int my);
void explorer_click(int wx, int wy, int mx, int my);

#endif
