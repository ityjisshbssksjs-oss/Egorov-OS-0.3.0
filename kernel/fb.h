#ifndef EGOROV_FB_H
#define EGOROV_FB_H

#include "types.h"

typedef struct {
    uint32_t *pixels;   /* software back-buffer (RAM) — all draws go here  */
    uint32_t *vram;     /* real VBE framebuffer — written only by fb_present */
    uint32_t  width;
    uint32_t  height;
    uint32_t  pitch;    /* back-buffer row stride in bytes (= width * 4)    */
    uint8_t   bpp;
} framebuffer_t;

extern framebuffer_t fb;

void fb_init(uint64_t addr, uint32_t pitch, uint32_t w, uint32_t h, uint8_t bpp);

/* Blit the back-buffer to VRAM. Call once per rendered frame.
 * Because every pixel write goes to fast RAM first and only one blit
 * reaches the slow VBE framebuffer, the user never sees a partial frame —
 * eliminating the flicker/tearing visible when mouse movement triggers a
 * full-screen redraw. */
void fb_present(void);

static inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}
void fb_pixel(int x, int y, uint32_t c);
void fb_fill(uint32_t c);
void fb_rect(int x, int y, int w, int h, uint32_t c);
void fb_rect_outline(int x, int y, int w, int h, uint32_t c);
void fb_hline(int x, int y, int w, uint32_t c);
void fb_vline(int x, int y, int h, uint32_t c);
void fb_grad_v(int x, int y, int w, int h, uint32_t top, uint32_t bot);
void fb_grad_h(int x, int y, int w, int h, uint32_t left, uint32_t right);
void fb_blit_mask(int x, int y, int w, int h, const uint8_t *mask, uint32_t c);
void fb_blit_rgba(int x, int y, int w, int h, const uint32_t *src);

#endif
