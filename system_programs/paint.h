#ifndef EGOROV_PAINT_H
#define EGOROV_PAINT_H

#include "types.h"

#define PAINT_WIN_W 460
#define PAINT_WIN_H 380

void paint_init(void);

/* Hover/pressed are tool slot indices (0..N), or -1.  Tool slots:
 *   0..7  : color swatches
 *   8..10 : brush sizes (small/medium/large)
 *   11    : "Очистить" (clear)
 */
void paint_draw(int wx, int wy, int hover_tool, int pressed_tool);

/* Returns the tool slot under (mx, my) inside the toolbar, or -1. */
int  paint_hit_tool(int wx, int wy, int mx, int my);

/* Trigger a tool action. */
void paint_tool_click(int tool);

/* Returns true if (mx, my) is inside the canvas drawable area. */
bool paint_in_canvas(int wx, int wy, int mx, int my);

/* Paint at (mx, my). If `dragging` is true, draws a line from the previous
 * paint point (Bresenham). Call once per frame while mouse is held. */
void paint_paint(int wx, int wy, int mx, int my, bool dragging);

/* Reset previous-point tracking (call when mouse button is released or canvas
 * leaves focus). */
void paint_release(void);

#endif
