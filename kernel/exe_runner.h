#pragma once
#include "types.h"

#define EXE_WIN_W 450
#define EXE_WIN_H 240

typedef enum {
    EXE_TARGET_CALC     = 0,
    EXE_TARGET_CLOCK    = 1,
    EXE_TARGET_PAINT    = 2,
    EXE_TARGET_NOTEPAD  = 3,
    EXE_TARGET_SNAKE    = 4,
    EXE_TARGET_UNKNOWN  = 5,
} exe_target_t;

void         exe_runner_start(const char *name, exe_target_t target);
void         exe_runner_draw(int wx, int wy);
void         exe_runner_tick(void);
bool         exe_runner_done(void);
exe_target_t exe_runner_get_target(void);
