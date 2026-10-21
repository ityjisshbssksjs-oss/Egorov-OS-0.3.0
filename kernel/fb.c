#include "fb.h"

/* COM1-лог в kernel.c - нужен для маркера «FB: shadow-present включён». */
extern void serial_puts(const char *s);

/* -----------------------------------------------------------------------
 * Software double-buffer
 *
 * All drawing primitives write to fb_backbuf (plain RAM, ~3 MB in BSS).
 * fb_present() then copies the completed frame to the real VBE framebuffer
 * in a single sequential sweep.
 *
 * Why this eliminates flickering:
 *   Without a back-buffer every fb_rect/fb_grad_v call writes directly to
 *   the VBE aperture.  The CPU can draw only a few MB/s to the PCI-mapped
 *   framebuffer, so a full 1024×768×32 redraw takes several milliseconds
 *   that are clearly visible as a flicker whenever the mouse moves.
 *
 *   With a back-buffer the slow VBE write happens exactly once per frame,
 *   after the entire scene is composited in fast cache-warm RAM.  From the
 *   user's perspective the screen jumps atomically from one fully-drawn
 *   frame to the next.
 * ----------------------------------------------------------------------- */
#define FB_MAX_W 1024
#define FB_MAX_H 768

/* [#312] выравнивание 4К: backbuf и тень лежат в .bss с выровненной
 * базы (DMA-дружелюбные адреса, готовность к MTRR/крупным страницам). */
static uint32_t fb_backbuf[FB_MAX_W * FB_MAX_H] __attribute__((aligned(4096)));   /* ~3 MB in .bss */
/* [#306] ТЕНЕВАЯ копия последнего слитого в VRAM кадра: fb_present
 * сравнивает её с backbuf построчно и пишет в VRAM только изменившиеся
 * хвосты строк.  Проблема реального железа: без MTRR write-combining
 * запись 3 МБ в VRAM занимает ~10-30 мс - полная копия КАЖДЫЙ кадр
 * душила USB-поллинг (мышь дёргалась).  Идле-кадр теперь = 3 МБ чтения
 * из кэшируемой ОЗУ (~0.3-1 мс) и НОЛЬ записей в VRAM; движение мыши
 * = десятки строк.  Инвариант: VRAM пишет ТОЛЬКО fb_present (grep:
 * других обращений к fb.vram в ядре нет), поэтому shadow==backbuf на
 * слове гарантирует, что VRAM на этом слове уже актуален.  Ещё +3 МБ
 * в .bss: линкер BSS не ограничивает; objcopy -O binary NOBITS не
 * пишет (размер egorov.bin не меняется); boot.S чистит _bss_start..
 * _bss_end по символам линкер-скрипта - новый массив занулён
 * автоматически (GRUB - через p_memsz ELF-сегмента). */
static uint32_t fb_shadow[FB_MAX_W * FB_MAX_H] __attribute__((aligned(4096)));
static uint32_t fb_vram_pitch;   /* real VBE row stride (may include padding) */

framebuffer_t fb;

void fb_init(uint64_t addr, uint32_t pitch, uint32_t w, uint32_t h, uint8_t bpp) {
    /* [#309] VBE LFB практически всегда ниже 4 Г; если BIOS/GRUB отдал
     * выше - 32-битный указатель его не адресует: предупреждаем и
     * продолжаем с младшими (лучше смотреть «что-то», чем чёрный экран;
     * multiboot-путь kernel.c адреса >4Г отсекает и сам). */
    if (addr > 0xFFFFFFFFull)
        serial_puts("FB: VRAM выше 4Г - не поддерживается VBE\n");
    fb.vram       = (uint32_t*)(uint32_t)addr;
    fb_vram_pitch = pitch;

    /* Point all drawing calls at the back-buffer. */
    fb.pixels = fb_backbuf;
    fb.width  = (w < FB_MAX_W) ? w : (uint32_t)FB_MAX_W;
    fb.height = (h < FB_MAX_H) ? h : (uint32_t)FB_MAX_H;
    fb.pitch  = fb.width * 4;   /* back-buffer: tight packing, no row padding */
    fb.bpp    = bpp;
}

void fb_present(void) {
    /* [#306] shadow-present (см. комментарий у fb_shadow): построчное
     * сравнение 4-байтными словами, копия ХВОСТА строки от первого
     * расхождения.  Первый вызов - полная копия с синхронизацией тени
     * (VRAM ещё хранит мусор BIOS/GRUB, нулевая тень ни о чём не
     * говорит - в т.ч. для полностью чёрных строк).  Фолбэк на старый
     * полный путь - если размер вылез за максимум (fb_init клампит,
     * путь недостижим, но страховка бесплатна).  Сравнение 3 МБ из
     * кэш-теплой ОЗУ на пустом кадре ~0.3-1 мс - в десятки раз дешевле
     * записи в VRAM, и USB-поллинг больше не душится. */
    static bool s_first = true;
    const uint32_t line = fb.width * 4;                  /* строка backbuf, байт */
    /* [#310/311] guard от pitch=0 от кривого BIOS VBE остаётся. */
    const uint32_t vram_stride = fb_vram_pitch ? fb_vram_pitch : line;

    if (fb.width > (uint32_t)FB_MAX_W || fb.height > (uint32_t)FB_MAX_H) {
        /* страховка (недостижимо): старый полный проход */
        const uint8_t *src = (const uint8_t*)fb_backbuf;
        uint8_t       *dst = (uint8_t*)fb.vram;
        for (uint32_t y = 0; y < fb.height; y++) {
            const uint32_t *s = (const uint32_t*)src;
            uint32_t       *d = (uint32_t*)dst;
            for (uint32_t x = 0; x < fb.width; x++) d[x] = s[x];
            src += line;
            dst += vram_stride;
        }
        return;
    }

    if (s_first) {
        s_first = false;
        serial_puts("FB: shadow-present включён\n");
        for (uint32_t y = 0; y < fb.height; y++) {
            const uint32_t *s  = &fb_backbuf[y * fb.width];
            uint32_t       *sh = &fb_shadow[y * fb.width];
            uint32_t       *d  = (uint32_t*)((uint8_t*)fb.vram + y * vram_stride);
            for (uint32_t x = 0; x < fb.width; x++) { d[x] = s[x]; sh[x] = s[x]; }
        }
        return;
    }

    for (uint32_t y = 0; y < fb.height; y++) {
        const uint32_t *s  = &fb_backbuf[y * fb.width];
        uint32_t       *sh = &fb_shadow[y * fb.width];
        uint32_t        x  = 0;
        while (x < fb.width && sh[x] == s[x]) x++;   /* до первого расхождения */
        if (x == fb.width) continue;                 /* строка не менялась */
        uint32_t *d = (uint32_t*)((uint8_t*)fb.vram + y * vram_stride);
        for (; x < fb.width; x++) { d[x] = s[x]; sh[x] = s[x]; }
    }
}

/* -----------------------------------------------------------------------
 * Drawing primitives — identical to before, but now targeting fb.pixels
 * (the back-buffer) instead of VRAM directly.
 * ----------------------------------------------------------------------- */

void fb_pixel(int x, int y, uint32_t c) {
    /* [#307] беззнаковая проверка границ: одно приведение к unsigned
     * закрывает и отрицательные координаты, и выход за край. */
    if ((unsigned)x >= fb.width || (unsigned)y >= fb.height) return;
    uint8_t *row = (uint8_t*)fb.pixels + y * fb.pitch;
    ((uint32_t*)row)[x] = c;
}

void fb_fill(uint32_t c) {
    for (uint32_t y = 0; y < fb.height; y++) {
        uint32_t *row = (uint32_t*)((uint8_t*)fb.pixels + y * fb.pitch);
        for (uint32_t x = 0; x < fb.width; x++) row[x] = c;
    }
}

void fb_rect(int x, int y, int w, int h, uint32_t c) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > (int)fb.width)  w = (int)fb.width  - x;
    if (y + h > (int)fb.height) h = (int)fb.height - y;
    if (w <= 0 || h <= 0) return;
    for (int yy = 0; yy < h; yy++) {
        uint32_t *row = (uint32_t*)((uint8_t*)fb.pixels + (y + yy) * fb.pitch);
        for (int xx = 0; xx < w; xx++) row[x + xx] = c;
    }
}

void fb_hline(int x, int y, int w, uint32_t c) { fb_rect(x, y, w, 1, c); }
void fb_vline(int x, int y, int h, uint32_t c) { fb_rect(x, y, 1, h, c); }

void fb_rect_outline(int x, int y, int w, int h, uint32_t c) {
    fb_hline(x, y, w, c);
    fb_hline(x, y + h - 1, w, c);
    fb_vline(x, y, h, c);
    fb_vline(x + w - 1, y, h, c);
}

void fb_grad_v(int x, int y, int w, int h, uint32_t top, uint32_t bot) {
    int tr = (top >> 16) & 0xFF, tg = (top >> 8) & 0xFF, tb = top & 0xFF;
    int br = (bot >> 16) & 0xFF, bg = (bot >> 8) & 0xFF, bb = bot & 0xFF;
    for (int yy = 0; yy < h; yy++) {
        int r = tr + (br - tr) * yy / (h ? h : 1);
        int g = tg + (bg - tg) * yy / (h ? h : 1);
        int b = tb + (bb - tb) * yy / (h ? h : 1);
        uint32_t c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
        fb_hline(x, y + yy, w, c);
    }
}

void fb_grad_h(int x, int y, int w, int h, uint32_t left, uint32_t right) {
    int lr = (left >> 16) & 0xFF, lg = (left >> 8) & 0xFF, lb = left & 0xFF;
    int rr = (right >> 16) & 0xFF, rg = (right >> 8) & 0xFF, rb = right & 0xFF;
    for (int xx = 0; xx < w; xx++) {
        int r = lr + (rr - lr) * xx / (w ? w : 1);
        int g = lg + (rg - lg) * xx / (w ? w : 1);
        int b = lb + (rb - lb) * xx / (w ? w : 1);
        uint32_t c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
        fb_vline(x + xx, y, h, c);
    }
}

void fb_blit_mask(int x, int y, int w, int h, const uint8_t *mask, uint32_t c) {
    int row_bytes = (w + 7) / 8;
    for (int yy = 0; yy < h; yy++) {
        for (int xx = 0; xx < w; xx++) {
            uint8_t b = mask[yy * row_bytes + (xx >> 3)];
            if (b & (0x80 >> (xx & 7))) fb_pixel(x + xx, y + yy, c);
        }
    }
}

void fb_blit_rgba(int x, int y, int w, int h, const uint32_t *src) {
    for (int yy = 0; yy < h; yy++) {
        for (int xx = 0; xx < w; xx++) {
            uint32_t p = src[yy * w + xx];
            if (p & 0xFF000000) fb_pixel(x + xx, y + yy, p & 0xFFFFFF);
        }
    }
}
