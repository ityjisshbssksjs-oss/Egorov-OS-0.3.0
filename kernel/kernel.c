/* Egorov OS kernel entry point.
 *
 * Receives multiboot1 magic + info pointer, sets up framebuffer, then enters
 * either the desktop (Live) or the installer based on the kernel command line.
 *
 * FIX (triple faults / text+cursor disappear after HDD install):
 *   The 8259A PIC, after BIOS init, maps IRQ 0-7 to INT 0x08-0x0F.
 *   INT 0x08 is the CPU double-fault vector — so the PIT timer (IRQ 0,
 *   18.2 Hz) collides with #DF in the IDT.  With no IDT set up by the OS,
 *   any hardware interrupt reaching the CPU caused a triple fault (→ VM
 *   reset).  VirtualBox statistics showed 243 double-faults in one session.
 *
 *   After a triple fault VirtualBox resets the guest CPU but the VBE VRAM
 *   retains its last pixel state.  The background was painted first, text
 *   and the cursor were scheduled next — so the user saw a desktop
 *   background with no text or cursor, and no further input was possible.
 *
 *   Fix applied here (two layers):
 *   1. Reprogram the 8259A PICs so IRQ 0-15 map to INT 0x20-0x2F, safely
 *      above all CPU exception vectors, then mask every IRQ (OCW1 = 0xFF).
 *      This OS uses polled I/O throughout, so hardware interrupts are never
 *      needed and can remain masked for the entire session.
 *   2. Load a minimal 256-entry IDT whose every gate points to a one-byte
 *      IRET stub.  Any stray NMI or spurious interrupt now returns silently
 *      instead of triple-faulting the machine.
 */
#include "types.h"
#include "io.h"
#include "fb.h"
#include "text.h"
#include "font.h"    /* FONT_GLYPH_W/H для текстовой консоли */
#include "gui.h"
#include "ps2.h"
#include "usb_mouse.h"
#include "mouse_hw.h"
#include "ehci.h"
#include "usbmap.h"
#include "usbdev.h"
#include "usb_uhci_kbd.h"
#include "ata.h"
#include "ahci.h"
#include "usb.h"
#include "rtc.h"      /* авто-выход textlog по RTC */
#include "power.h"
#include "timer.h"    /* PIT-калибровка задержек (kdelay_*) */
#include "version.h"  /* [#473] ЭТАП 9: номер СБОРКИ (EGOROV_OS_BUILD_S) */
#include "logstick.h" /* [LOGS] ЭТАП 11: EGOROV.LOG на разделе LOGS */

#include "string.h"

/* Глубокий аудит (G4, п.9c): тег версии собран в ОДНОМ месте — все
 * серийные баннеры/подписи ниже склеиваются из EGOROV_TAG.  Результиру
 * текст НЕИЗМЕНЕН (побайтно те же строки): «Egorov OS 0.3.0» ЗАМОРОЖЕН,
 * e2e-маркеры не задеты (os/tests/e2e_audit.py версию не матчит —
 * проверено grep'ом).
 * [#473] ЭТАП 9: к баннерам ДОБАВЛЕН номер СБОРКИ из kernel/version.h
 * («Сборка 1», «Сборка 2», …) — по нему видно, какой образ реально
 * загрузился с флешки (Rufus/DD мог не перезаписать).  e2e-маркеры
 * номер сборки не матчат — проверено. */
#define EGOROV_TAG "0.3.0"


#define MULTIBOOT_BOOTLOADER_MAGIC 0x2BADB002

struct multiboot_info {
    uint32_t flags;
    uint32_t mem_lower;
    uint32_t mem_upper;
    uint32_t boot_device;
    uint32_t cmdline;
    uint32_t mods_count;
    uint32_t mods_addr;
    uint32_t syms[4];
    uint32_t mmap_length;
    uint32_t mmap_addr;
    uint32_t drives_length;
    uint32_t drives_addr;
    uint32_t config_table;
    uint32_t boot_loader_name;
    uint32_t apm_table;
    uint32_t vbe_control_info;
    uint32_t vbe_mode_info;
    uint16_t vbe_mode;
    uint16_t vbe_interface_seg;
    uint16_t vbe_interface_off;
    uint16_t vbe_interface_len;
    uint64_t framebuffer_addr;
    uint32_t framebuffer_pitch;
    uint32_t framebuffer_width;
    uint32_t framebuffer_height;
    uint8_t  framebuffer_bpp;
    uint8_t  framebuffer_type;
    uint8_t  color_info[6];
} __attribute__((packed));

extern void desktop_run(bool booted_from_hdd);
extern void installer_run(void);

/* ── Interrupt infrastructure ─────────────────────────────────────────────── */

/* Remap the 8259A PIC pair so that IRQ 0-7 → INT 0x20-0x27 and
   IRQ 8-15 → INT 0x28-0x2F, then mask every single IRQ.
   This must be done before the IDT is loaded so that even if an
   interrupt slips through between the CLI and the LIDT, the vector
   number is safe (≥ 0x20, well above all CPU exception vectors). */
static void pic_remap_and_mask(void) {
    /* ICW1 – start initialisation sequence (cascade, ICW4 needed). */
    outb(0x20, 0x11);
    outb(0xA0, 0x11);
    /* ICW2 – new base vectors. */
    outb(0x21, 0x20);   /* master: IRQ 0-7  → INT 0x20-0x27 */
    outb(0xA1, 0x28);   /* slave:  IRQ 8-15 → INT 0x28-0x2F */
    /* ICW3 – cascade wiring (master pin 2 / slave ID 2). */
    outb(0x21, 0x04);
    outb(0xA1, 0x02);
    /* ICW4 – 8086/88 mode. */
    outb(0x21, 0x01);
    outb(0xA1, 0x01);
    /* OCW1 – mask all IRQs on both chips. */
    outb(0x21, 0xFF);
    outb(0xA1, 0xFF);
}

/* One-byte IRET stub used as the catch-all interrupt handler.
   Declared volatile so the compiler doesn't remove it as dead code. */
static volatile uint8_t isr_stub[1] = { 0xCF }; /* IRET */

/* 256 × 8-byte interrupt gate descriptors. */
static uint64_t g_idt[256];

/* IDTR pseudo-descriptor (6 bytes: 2-byte limit + 4-byte base). */
static struct { uint16_t limit; uint32_t base; } __attribute__((packed)) g_idtr;

static void idt_install(void) {
    uint32_t h = (uint32_t)isr_stub;
    int i;
    for (i = 0; i < 256; i++) {
        /* 32-bit interrupt gate:
             bits 15:0  = handler offset [15:0]
             bits 31:16 = segment selector (0x08 = kernel CS from GDT)
             bits 39:32 = 0 (reserved)
             bits 47:40 = 0x8E (present | DPL=0 | 32-bit interrupt gate)
             bits 63:48 = handler offset [31:16]                          */
        g_idt[i] = ((uint64_t)(h & 0xFFFFu))
                 | ((uint64_t)0x0008u   << 16)
                 | ((uint64_t)0x8Eu     << 40)
                 | ((uint64_t)(h >> 16) << 48);
    }
    g_idtr.limit = (uint16_t)(sizeof(g_idt) - 1);
    g_idtr.base  = (uint32_t)g_idt;
    __asm__ volatile("lidt %0" :: "m"(g_idtr) : "memory");
}

/* ── Serial port (COM1) debug output ──────────────────────────────────────── */

#define COM1 0x3F8

void serial_init(void) {
    outb(COM1 + 1, 0x00);   /* disable interrupts */
    outb(COM1 + 3, 0x80);   /* enable DLAB */
    outb(COM1 + 0, 0x01);   /* baud divisor lo (115200) */
    outb(COM1 + 1, 0x00);   /* baud divisor hi */
    outb(COM1 + 3, 0x03);   /* 8N1 */
    outb(COM1 + 2, 0xC7);   /* FIFO on */
    outb(COM1 + 4, 0x0B);   /* RTS+DTR+IRQ */
}

static void serial_putc(char c) {
    while (!(inb(COM1 + 5) & 0x20)) {} /* wait for transmit empty */
    outb(COM1, (uint8_t)c);
}

/* ── TEXTLOG — текстовая консоль «test on errors» ──────────────────────────
 * Отдельный пункт меню GRUB передаёт ядру параметр «textlog» (как «install»).
 * Тогда КАЖДЫЙ байт serial-журнала (те же строки, что печатает COREINFO)
 * в живую дублируется на экран в текстовом виде — как dmesg в Linux.
 * GUI не запускается вовсе: после инициализации железа ядро остаётся в
 * консоли, опрашивает мышь/клавиатуру (как desktop) и печатает новые
 * MOUSEHW-ошибки по мере появления.  До готовности fb строки копятся в
 * ОЗУ (tc_active=0) и вываливаются на экран одной пачкой из tc_start(). */
static int g_textlog = 0;          /* режим textlog включён */
static int g_sticklog = 0;         /* пункт 2 GRUB: журнал → EGOROV.LOG на
                                     * разделе LOGS флешки ([LOGS] ЭТАП 11) */

#define TC_MAX_ROWS 48
#define TC_MAX_COLS 127            /* 1024 px / 8 px на символ - запас */
static char tc_buf[TC_MAX_ROWS][TC_MAX_COLS + 1];
static int  tc_rows = TC_MAX_ROWS, tc_cols = TC_MAX_COLS;
static int  tc_row, tc_col;
static int  tc_active = 0;         /* fb готов — можно рисовать */

static void tc_draw_row(int r) {
    fb_rect(0, r * FONT_GLYPH_H, (int)fb.width, FONT_GLYPH_H, rgb(0, 0, 0));
    text_draw_str(4, r * FONT_GLYPH_H, tc_buf[r], rgb(0xE8, 0xE8, 0xE8));
}

static void tc_scroll(void) {
    int r, c;
    for (r = 1; r < tc_rows; r++)
        for (c = 0; c <= TC_MAX_COLS; c++)
            tc_buf[r - 1][c] = tc_buf[r][c];
    for (c = 0; c <= TC_MAX_COLS; c++) tc_buf[tc_rows - 1][c] = 0;
    if (tc_active) {
        for (r = 0; r < tc_rows; r++) tc_draw_row(r);
        fb_present();
    }
}

static void tc_newline(void) {
    tc_col = 0;
    if (tc_row + 1 >= tc_rows) {
        tc_row = tc_rows - 1;
        tc_scroll();
    } else {
        tc_row++;
        if (tc_active) fb_present();   /* строка видна по завершении */
    }
}

static void tc_putc(char c) {
    if (c == '\n') { tc_newline(); return; }
    if (c == '\r') { tc_col = 0; return; }
    if (tc_col >= tc_cols) tc_newline();          /* перенос длинной строки */
    if (tc_row >= tc_rows) tc_row = tc_rows - 1;
    tc_buf[tc_row][tc_col] = c;
    tc_col++;
    tc_buf[tc_row][tc_col] = 0;
    if (tc_active)   /* символ дорисовывается в задний буфер (без present) */
        text_draw_str(4 + (tc_col - 1) * FONT_GLYPH_W, tc_row * FONT_GLYPH_H,
                      tc_buf[tc_row] + tc_col - 1, rgb(0xE8, 0xE8, 0xE8));
}

/* Включить консоль: вызывается сразу после fb_init в режиме textlog.
 * Подрезает накопившийся хвост под фактическое число строк экрана. */
static void tc_start(void) {
    int r;
    tc_cols = (int)((fb.width - 8) / FONT_GLYPH_W);
    tc_rows = (int)((fb.height - 4) / FONT_GLYPH_H);
    if (tc_cols > TC_MAX_COLS) tc_cols = TC_MAX_COLS;
    if (tc_rows > TC_MAX_ROWS) tc_rows = TC_MAX_ROWS;
    if (tc_cols < 20) tc_cols = 20;
    if (tc_rows < 5)  tc_rows = 5;
    while (tc_row >= tc_rows) {              /* хвост не влез — сдвинуть */
        int r2, c;
        for (r2 = 1; r2 < TC_MAX_ROWS; r2++)
            for (c = 0; c <= TC_MAX_COLS; c++)
                tc_buf[r2 - 1][c] = tc_buf[r2][c];
        tc_row--;
    }
    fb_rect(0, 0, (int)fb.width, (int)fb.height, rgb(0, 0, 0));
    tc_active = 1;
    for (r = 0; r <= tc_row && r < tc_rows; r++) tc_draw_row(r);
    fb_present();
}


/* ── KLOG: зеркало serial-вывода в RAM-кольцо (Журнал ядра, F12) ────────────
 * На реальном ПК COM1 часто недоступен, а флешка (dd) файлов не отдаёт —
 * каждый байт, ушедший в serial, дублируется в буфер 8 КБ.  Окно
 * Терминал (terminal.c, команда COREINFO, F12) печатает хвост: MOUSEHW/POWER
 * видны прямо на экране целевой машины — фото экрана = диагностика.
 *
 * [LOGS] РАННИЙ БУФЕР (build 4): кольцо 8 КБ затирало начало загрузки ДО
 * того, как ls_stick_prepare() находил флешку — обещание «весь журнал с
 * первого байта» не выполнялось.  Теперь ПЕРВЫЕ байты журнала пишутся
 * и в ЛИНЕЙНЫЙ буфер ранней загрузки (без затирания): он держит старт
 * до готовности носителя.  Полный поток для EGOROV.LOG = ранний буфер
 * [0..E) + кольцо [oldest..total); дыра между ними (если обе ёмкости
 * переполнены) честно считается в байтах и отмечается маркером в файле.
 * Размер — cmdline logearly=N (КБ, по умолчанию 128, минимум 1);
 * эффективный размер кольца — cmdline logring=N (КБ, по умолчанию 64). */
#define KLOG_EARLY_SIZE 131072u  /* 128 КБ линейного буфера старта */
#define KLOG_SIZE 65536u   /* кольцо: 64 КБ (степень двойки; [build 4] было 8К —
                             * мгновенный взрыв диагностики (СНИМКи ~20К) успевал
                             * затереть журнал между тиками выгрузки).  Буфер —
                             * 64 КБ всегда, ЭФФЕКТИВНЫЙ размер — cmdline
                             * logring=N КБ (8..64, степень 2; по умолчанию 64) */
#define KLOG2_SIZE 8192u   /* близнец-канарейка: 8 КБ (конец .bss) — не растёт */
static uint32_t klog_ring_sz = KLOG_SIZE;  /* эффективный размер кольца */
static char     klog_buf[KLOG_SIZE];
static uint32_t klog_pos;          /* всего записано байт */
static char     klog_early_buf[KLOG_EARLY_SIZE];  /* старт журнала  */
static uint32_t klog_early_n;     /* заполнено раннего буфера (<= max) */
static uint32_t klog_early_max = KLOG_EARLY_SIZE; /* лимит (logearly) */

/* близнец-зеркало журнала — 16 КБ область _klog2_start, которую
 * linker.ld кладёт в САМЫЙ КОНЕЦ .bss (за всеми данными и 3 МБ fb_backbuf).
 * Бродячий DMA остановленного xHCI или выход за границу буфера со стороны
 * стека должны переписать миллионы байт «жертвенной» памяти, прежде чем
 * добраться до близнеца.  COREINFO показывает основное кольцо; если оно
 * повреждено (klog_pos < 16 — живой баннер ядра уже даёт ~60+),
 * показывается близнец с предупреждением (симптом: «COREINFO: 1: 2»).
 * Раскладка области: [0..8191] кольцо, [8192..8195] позиция. */
extern char _klog2_start[];

static uint32_t klog2_getpos(void) {
    return *(volatile uint32_t *)(_klog2_start + KLOG2_SIZE);
}

static void klog2_setpos(uint32_t v) {
    *(volatile uint32_t *)(_klog2_start + KLOG2_SIZE) = v;
}

static void klog_putc(char c) {
    if (g_textlog) tc_putc(c);   /* живой вывод журнала на экран */
    klog_buf[klog_pos & (klog_ring_sz - 1u)] = c;
    klog_pos++;
    /* [LOGS] ранний буфер: линейный, НЕ затирается; заполняется пока есть
     * место — старт журнала доживает до выгрузки на флешку */
    if (klog_early_n < klog_early_max)
        klog_early_buf[klog_early_n++] = c;
    {
        uint32_t p2 = klog2_getpos();
        _klog2_start[p2 & (KLOG2_SIZE - 1u)] = c;
        klog2_setpos(p2 + 1);
    }
}

/* Скопировать хвост журнала (последние min(pos, dstsz-1) байт). */
void klog_read_tail(char *dst, int dstsz) {
    uint32_t total = klog_pos;
    uint32_t n = (dstsz > 0) ? (uint32_t)dstsz - 1u : 0u;
    if (n > total) n = total;
    uint32_t start = total - n;
    for (uint32_t i = 0; i < n; i++)
        dst[i] = klog_buf[(start + i) & (klog_ring_sz - 1u)];
    dst[n] = 0;
}

uint32_t klog_total(void) { return klog_pos; }

/* [LOGS] СТАРЫЙ ЖИВОЙ байт журнала: всё, что записано раньше, уже
 * затёрто кольцом (переполнение журнала).  Выгрузчик EGOROV.LOG
 * обязан начать СЮДА, а не с нуля — иначе в файл попали бы «дыры»
 * кольца, читающиеся как мусор. */
uint32_t klog_oldest(void) {
    return (klog_pos > klog_ring_sz) ? klog_pos - klog_ring_sz : 0u;
}

/* [LOGS] РАННИЙ БУФЕР: сколько байт старта захвачено (линейно, без
 * затирания).  Пока early_len == klog_total() — журнал ПОЛНЫЙ с
 * первого байта; после переполнения — старт до early_len. */
uint32_t klog_early_len(void) { return klog_early_n; }

/* [LOGS] РАННИЙ БУФЕР: копия байт [from..) линейного буфера (не более
 * dstsz).  Возвращает число скопированных байт (0 — from за концом). */
uint32_t klog_early_copy(uint32_t from, char *dst, uint32_t dstsz) {
    if (dst == 0 || dstsz == 0 || from >= klog_early_n) return 0u;
    uint32_t n = klog_early_n - from;
    if (n > dstsz) n = dstsz;
    for (uint32_t i = 0; i < n; i++)
        dst[i] = klog_early_buf[from + i];
    return n;
}

/* [LOGS] СНИМОК ЖУРНАЛА для выгрузки на накопитель.
 * Копирует байты [from .. from+*plen) в dst, продвигая *plen на число
 * реально скопированных байт.  Три защиты:
 *   1) from за концом (журнал короче) → 0 байт, безопасный no-op;
 *   2) ПЕРЕПОЛНЕНИЕ кольца: from < klog_oldest() → from подтягивается
 *      к старейшему живому байту (затёртое не выгружается вовсе);
 *   3) размер буфера: копируется не больше *plen байт.
 * Возвращает ИТОГОВОЕ число скопированных байт (может быть меньше
 * запрошенного — либо буфер меньше, либо журнал закончился). */
uint32_t klog_snapshot(uint32_t from, char *dst, uint32_t dstsz) {
    if (dst == 0 || dstsz == 0) return 0u;
    uint32_t total = klog_pos;
    if (from >= total) return 0u;                 /* журнал короче      */
    uint32_t oldest = (total > klog_ring_sz) ? total - klog_ring_sz : 0u;
    if (from < oldest) from = oldest;             /* затёрто кольцом    */
    uint32_t n = total - from;
    if (n > dstsz) n = dstsz;                     /* размер буфера      */
    for (uint32_t i = 0; i < n; i++)
        dst[i] = klog_buf[(from + i) & (klog_ring_sz - 1u)];
    return n;
}

/* близнец-зеркало — чтение хвоста из копии в конце.bss (см. выше). */
uint32_t klog_mirror_total(void) { return klog2_getpos(); }

void klog_mirror_tail(char *dst, int dstsz) {
    uint32_t total = klog2_getpos();
    uint32_t n = (dstsz > 0) ? (uint32_t)dstsz - 1u : 0u;
    if (n > total) n = total;
    uint32_t start = total - n;
    for (uint32_t i = 0; i < n; i++)
        dst[i] = _klog2_start[(start + i) & (KLOG2_SIZE - 1u)];
    dst[n] = 0;
}

void serial_puts(const char *s) {
    for (; *s; s++) {
        if (*s == '\n') serial_putc('\r');
        serial_putc(*s);
        klog_putc(*s);
    }
}

void serial_puthex(uint32_t v) {
    const char *h = "0123456789ABCDEF";
    serial_puts("0x");
    for (int i = 28; i >= 0; i -= 4) {
        serial_putc(h[(v >> i) & 0xF]);
        klog_putc(h[(v >> i) & 0xF]);
    }
}

/* ── Bochs VBE (BochsDispi) protected-mode framebuffer init ──────────────── */

/* The BochsDispi interface works via I/O ports 0x01CE/0x01CF and is supported
 * by QEMU (-vga std), VirtualBox, and Bochs.  Unlike INT 10h, it can be called
 * directly from 32-bit protected mode — no BIOS trampoline required.
 * The linear framebuffer is always mapped at 0xE0000000.
 *
 * On real hardware GRUB sets the VBE mode via INT 10h and passes framebuffer
 * info in multiboot_info (flag bit 12, framebuffer_type == 1).  We try that
 * first and fall back to BochsDispi for QEMU environments where GRUB cannot
 * switch VBE modes (e.g. headless / SDL-on-Xvfb). */

#define BOCHS_VBE_INDEX 0x01CE
#define BOCHS_VBE_DATA  0x01CF

#define VBE_IDX_ID      0
#define VBE_IDX_XRES    1
#define VBE_IDX_YRES    2
#define VBE_IDX_BPP     3
#define VBE_IDX_ENABLE  4
#define VBE_IDX_BANK    5
#define VBE_IDX_VIRT_W  6
#define VBE_IDX_VIRT_H  7
#define VBE_IDX_X_OFF   8
#define VBE_IDX_Y_OFF   9

#define VBE_DIS_DISABLED    0x00
#define VBE_DIS_ENABLED     0x01
#define VBE_DIS_LFB         0x40

static void bochs_vbe_w(uint16_t idx, uint16_t val) {
    outw(BOCHS_VBE_INDEX, idx);
    outw(BOCHS_VBE_DATA, val);
}
static uint16_t bochs_vbe_r(uint16_t idx) {
    outw(BOCHS_VBE_INDEX, idx);
    return inw(BOCHS_VBE_DATA);
}

/* ── PCI config space helpers ─────────────────────────────────────────────── */

static uint32_t pci_read32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg) {
    uint32_t addr = 0x80000000u
                  | ((uint32_t)bus << 16)
                  | ((uint32_t)dev << 11)
                  | ((uint32_t)fn  <<  8)
                  | (reg & 0xFC);
    outl(0xCF8, addr);
    return inl(0xCFC);
}

/* Scan all PCI devices for VGA controller (class 0x03) and return BAR0.
 * Returns 0 if not found.  Шина сканируется ДО 256: VGA-контроллер может
 * сидеть за PCI-мостом на старших шинах (AMD FCH, ноутбучные чипсеты) —
 * 256×32×8 конфиг-чтений занимают единицы миллисекунд. */
static uint32_t pci_find_vga_bar0(void) {
    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t dev = 0; dev < 32; dev++) {
            uint32_t id = pci_read32(bus, dev, 0, 0);
            if (id == 0xFFFFFFFFu || id == 0u) continue;
            uint32_t cc = pci_read32(bus, dev, 0, 8);
            uint8_t base_class = (uint8_t)(cc >> 24);
            uint8_t sub_class  = (uint8_t)(cc >> 16);
            uint16_t vendor    = (uint16_t)(id & 0xFFFF);
            uint16_t device_id = (uint16_t)(id >> 16);
            serial_puts("PCI "); serial_puthex(bus); serial_puts(":");
            serial_puthex(dev); serial_puts(" ");
            serial_puthex(vendor); serial_puts(":"); serial_puthex(device_id);
            serial_puts(" class="); serial_puthex(base_class);
            serial_puts(":"); serial_puthex(sub_class); serial_puts("\n");
            if (base_class == 0x03) {  /* display controller */
                uint32_t bar0 = pci_read32(bus, dev, 0, 0x10);
                serial_puts("VGA BAR0="); serial_puthex(bar0); serial_puts("\n");
                bar0 &= ~0xFu;  /* strip type/prefetchable bits */
                if (bar0 >= 0x10000000u) return bar0;
            }
        }
    }
    return 0;
}

/* Returns 1 and fills out_* if BochsDispi is available and mode was set.
 * Returns 0 if the hardware doesn't have BochsDispi. */
static int bochs_vbe_set(uint16_t w, uint16_t h, uint16_t bpp,
                          uint64_t *out_addr, uint32_t *out_pitch) {
    uint16_t id = bochs_vbe_r(VBE_IDX_ID);
    serial_puts("BochsVBE id="); serial_puthex(id); serial_puts("\n");
    if (id < 0xB0C0 || id > 0xB0CF) return 0;

    bochs_vbe_w(VBE_IDX_ENABLE, VBE_DIS_DISABLED);
    bochs_vbe_w(VBE_IDX_XRES,   w);
    bochs_vbe_w(VBE_IDX_YRES,   h);
    bochs_vbe_w(VBE_IDX_BPP,    bpp);
    bochs_vbe_w(VBE_IDX_VIRT_W, w);
    bochs_vbe_w(VBE_IDX_VIRT_H, h);
    bochs_vbe_w(VBE_IDX_X_OFF,  0);
    bochs_vbe_w(VBE_IDX_Y_OFF,  0);
    bochs_vbe_w(VBE_IDX_ENABLE, VBE_DIS_ENABLED | VBE_DIS_LFB);

    /* Verify settings were accepted */
    uint16_t chk = bochs_vbe_r(VBE_IDX_XRES);
    serial_puts("BochsVBE XRES readback="); serial_puthex(chk); serial_puts("\n");
    uint16_t en  = bochs_vbe_r(VBE_IDX_ENABLE);
    serial_puts("BochsVBE ENABLE readback="); serial_puthex(en); serial_puts("\n");

    /* Find the actual LFB address via PCI scan */
    serial_puts("Scanning PCI for VGA BAR0...\n");
    uint32_t bar0 = pci_find_vga_bar0();
    if (!bar0) {
        serial_puts("PCI VGA not found, using 0xE0000000\n");
        bar0 = 0xE0000000u;
    }

    /* Quick sanity: write a test dword and read it back */
    volatile uint32_t *lfb = (volatile uint32_t *)bar0;
    lfb[0] = 0xDEADBEEFu;
    uint32_t rb = lfb[0];
    serial_puts("LFB test @"); serial_puthex(bar0);
    serial_puts(": wrote 0xDEADBEEF, read "); serial_puthex(rb); serial_puts("\n");

    *out_addr  = (uint64_t)bar0;
    *out_pitch = (uint32_t)w * (bpp >> 3);
    serial_puts("BochsVBE set "); serial_puthex(w); serial_puts("x");
    serial_puthex(h); serial_puts(" @"); serial_puthex(bar0);
    serial_puts(" pitch="); serial_puthex(*out_pitch); serial_puts("\n");
    return 1;
}

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static bool cmdline_contains(const char *cmd, const char *kw) {
    if (!cmd) return false;
    int klen = 0; while (kw[klen]) klen++;
    for (int i = 0; cmd[i]; i++) {
        int j = 0;
        while (j < klen && cmd[i + j] == kw[j]) j++;
        if (j == klen) return true;
    }
    return false;
}

/* [LOGS] ЭТАП 11: числовой аргумент cmdline «kw=NNN» (0 = нет/мусор).
 * Используется для logcap=N — лимит EGOROV.LOG в КБ (тест переполнения). */
static uint32_t cmdline_int(const char *cmd, const char *kw) {
    if (!cmd) return 0;
    int klen = 0; while (kw[klen]) klen++;
    for (int i = 0; cmd[i]; i++) {
        int j = 0;
        while (j < klen && cmd[i + j] == kw[j]) j++;
        if (j == klen && cmd[i + klen] == '=') {
            uint32_t v = 0;
            for (int k = i + klen + 1; cmd[k] >= '0' && cmd[k] <= '9'; k++)
                v = v * 10u + (uint32_t)(cmd[k] - '0');
            return v;
        }
    }
    return 0;
}

static void panic_no_fb(uint32_t magic) {
    /* Fall back to VGA text mode panic. */
    volatile uint16_t *vga = (uint16_t*)0xB8000;
    const char *msg = "Egorov OS: framebuffer not available from bootloader.";
    for (int i = 0; msg[i]; i++) vga[i] = (0x4F00 | (uint8_t)msg[i]);
    (void)magic;
    for (;;) __asm__ volatile("hlt");
}

/* ── Откуда загрузились: установленная система или Live? ──
 * Свой HDD-загрузчик (hdd_boot.asm) строит минимальный mbi только с
 * битами 1 (boot_device) и 12 (framebuffer) и НИКОГДА не заполняет
 * бит 9 (boot_loader_name).  GRUB бит 9 ставит всегда.
 * Старая проверка «BIOS-диск 0x80..0x8F = HDD» ложно срабатывала для
 * Live-загрузки с USB-флешки: BIOS грузит флешку тоже как диск 0x80,
 * и ОС писала «установленная система (HDD)» прямо в Live-режиме. */
static bool detect_hdd_boot(struct multiboot_info *mbi) {
    serial_puts("bootdev: mbi->flags=");
    serial_puthex(mbi->flags);
    serial_puts("\n");
    if (!(mbi->flags & (1 << 1))) {              /* нет поля boot_device */
        serial_puts("bootdev: no boot_device field -> Live\n");
        return false;
    }
    serial_puts("bootdev: boot_device=");
    serial_puthex(mbi->boot_device);
    serial_puts("\n");
    if (mbi->flags & (1 << 9)) {                 /* boot_loader_name -> GRUB */
        serial_puts("bootdev: bootloader name present (GRUB) -> Live\n");
        return false;
    }
    /* boot_device: биты 31-24 = номер BIOS-диска (0x80 = первый HDD). */
    uint8_t drive = (uint8_t)((mbi->boot_device >> 24) & 0xFF);
    bool hdd = (drive >= 0x80 && drive <= 0x8F);
    serial_puts(hdd ? "bootdev: own HDD bootloader -> installed\n"
                    : "bootdev: not a HDD drive -> Live\n");
    return hdd;
}

/* ── ФИНАЛЬНАЯ ДИАГНОСТИКА — САМЫЙ КОНЕЦ экрана ─────────────────────────────
 * В самом низу textlog печатаются ОБА СНИМКа: xHCI (мышь) и OHCI
 * (флешка/мышь-обход) - одно фото нижней части экрана = всё состояние
 * USB.  После них идёт [LOGS] ИТОГ выгрузки EGOROV.LOG (ls_flush_final):
 * «ЗАПИСАНО N байт» или честная причина отказа — тоже на фото. */
static void diag_final(void) {
    serial_puts("\n==== ДИАГНОСТИКА USB (Egorov OS " EGOROV_TAG " Сборка " EGOROV_OS_BUILD_S ", самый конец, фото) ====\n");
    mouse_hw_xhci_snapshot();
    mouse_hw_ohci_snapshot();
    serial_puts("==== КОНЕЦ ДИАГНОСТИКИ (дальше лог не идёт) ====\n");
}


/* ── Entry point ──────────────────────────────────────────────────────────── */

void kernel_main(uint32_t magic, struct multiboot_info *mbi) {
    /* ── раннее определение режима textlog (пункт GRUB «test on
     * errors»): с этого момента весь журнал идёт и на экран.  fb ещё
     * нет — строки копятся в ОЗУ до tc_start(). */
    g_textlog = cmdline_contains(
        (mbi->flags & (1 << 2)) ? (const char *)mbi->cmdline : 0,
        "textlog");
    /* sticklog всегда через текстовую консоль - весь процесс виден
     * на экране. */
    g_sticklog = cmdline_contains(
        (mbi->flags & (1 << 2)) ? (const char *)mbi->cmdline : 0,
        "sticklog");
    if (g_sticklog) g_textlog = 1;
    /* [LOGS] logearly=N (КБ): размер раннего буфера журнала — для
     * проверки честности при переполнении (тест L13) и экономии ОЗУ.
     * Читается ДО первого serial_puts: баннер уже должен попасть в буфер. */
    {
        uint32_t ekb = cmdline_int(
            (mbi->flags & (1 << 2)) ? (const char *)mbi->cmdline : 0,
            "logearly");
        if (ekb == 0)                 ekb = 128u;   /* по умолчанию */
        else if (ekb > KLOG_EARLY_SIZE / 1024u) ekb = KLOG_EARLY_SIZE / 1024u;
        klog_early_max = ekb * 1024u;
    }
    /* [LOGS] logring=N (КБ): ЭФФЕКТИВНЫЙ размер кольца журнала (8..64,
     * степень двойки; по умолчанию 64 — взрывы диагностики не затирают
     * журнал между выгрузками).  Тест L13 (logring=8) проверяет честные
     * маркеры потери.  Парсится ДО первого serial_puts. */
    {
        uint32_t rkb = cmdline_int(
            (mbi->flags & (1 << 2)) ? (const char *)mbi->cmdline : 0,
            "logring");
        if (rkb == 0) rkb = 64u;
        if (rkb < 8u) rkb = 8u;
        if (rkb > KLOG_SIZE / 1024u) rkb = KLOG_SIZE / 1024u;
        uint32_t sz = 8u;                      /* вниз к степени двойки */
        while (sz * 2u <= rkb) sz *= 2u;
        klog_ring_sz = sz * 1024u;
    }

    /* ── Step 1: silence all hardware interrupts ──────────────────────────
     * Done before ANY other work so that the PIT, RTC and other peripherals
     * cannot fire a spurious interrupt while the IDT is still absent.
     * pic_remap_and_mask() sends EOI-safe ICW sequences; it is safe to call
     * even with IF=0 (CLI from boot.S / hdd_boot.asm is still in effect). */
    pic_remap_and_mask();

    /* ── Step 2: install minimal IDT ──────────────────────────────────────
     * Loads a 256-gate IDT where every entry points to a 1-byte IRET stub.
     * This makes the CPU safe against NMIs and any other non-maskable event
     * that cannot be blocked by the PIC or by CLI alone. */
    idt_install();

    /* ── Serial debug (COM1 at 115200 8N1) — helps diagnose headless QEMU */
    serial_init();
    /* Тег сборки в баннере: первая строка журнала сразу показывает,
     * собрана ли флешка из нового архива. */
    serial_puts("\n=== Egorov OS " EGOROV_TAG " Сборка " EGOROV_OS_BUILD_S " kernel_main ===\n");
    serial_puts("magic="); serial_puthex(magic); serial_puts("\n");
    serial_puts("klog: зеркало serial x2 (близнец в .klogsec) включено (Пуск -> Терминал: COREINFO)\n");
    if (g_textlog)
        serial_puts("TEXTLOG: текстовая консоль (журнал дублируется на экран)\n");

    /* ── Step 2.5: калибровка задержек по PIT (канал 2) ────────────────────
     * Драйверы (ata, ahci, USB-стек, mouse_hw, ehci) используют kdelay_us
     * и kdelay_ms: калибровка обязана пройти ДО любого обращения к железу
     * с таймингами — то есть до ata_init и всей USB-инициализации ниже.
     * До вызова kdelay_ms работают в консервативном фолбэке 1 итер/мкс.
     * Глубокий аудит (G4, п.9a): порядок проверен — timer_init (ЗДЕСЬ)
     * стоит до ata_init/ahci_init/ps2_init и всей USB-цепочки (этап 4);
     * нарушений нет, не трогаем. */
    timer_init();
    serial_puts("timer_init done\n");

    /* ── Step 3: validate multiboot environment ───────────────────────────*/
    if (magic != MULTIBOOT_BOOTLOADER_MAGIC) {
        serial_puts("BAD MAGIC - panic\n");
        panic_no_fb(magic);
    }

    serial_puts("mbi->flags="); serial_puthex(mbi->flags); serial_puts("\n");
    serial_puts("mbi->framebuffer_addr=");
    serial_puthex((uint32_t)mbi->framebuffer_addr); serial_puts("\n");
    serial_puts("mbi->framebuffer_type=");
    serial_puthex(mbi->framebuffer_type); serial_puts("\n");
    serial_puts("mbi->framebuffer_width=");
    serial_puthex(mbi->framebuffer_width); serial_puts("\n");
    serial_puts("mbi->framebuffer_height=");
    serial_puthex(mbi->framebuffer_height); serial_puts("\n");

    /* ── Step 4: force BochsDispi VBE ──────────────────────────────────── */
    uint64_t fb_addr  = 0;
    uint32_t fb_pitch = 0;
    uint32_t fb_w     = 0;
    uint32_t fb_h     = 0;
    uint8_t  fb_bpp   = 0;

    serial_puts("FB: forcing BochsDispi VBE\n");
    if (bochs_vbe_set(1024, 768, 32, &fb_addr, &fb_pitch)) {
        fb_w   = 1024;
        fb_h   = 768;
        fb_bpp = 32;
        serial_puts("FB: BochsDispi OK\n");
    } else {
        /* v1.7: real hardware has no BochsDispi — accept the mode GRUB
           set via INT 10h and handed over in multiboot_info instead of
           panicking into VGA text. */
        serial_puts("FB: BochsDispi failed, trying multiboot framebuffer\n");
        if ((mbi->flags & (1u << 12)) && mbi->framebuffer_type == 1 &&
            mbi->framebuffer_bpp == 32 &&
            mbi->framebuffer_addr != 0 &&
            (mbi->framebuffer_addr >> 32) == 0 &&
            mbi->framebuffer_width > 0 && mbi->framebuffer_height > 0 &&
            mbi->framebuffer_pitch > 0) {
            fb_addr  = mbi->framebuffer_addr;
            fb_pitch = mbi->framebuffer_pitch;
            fb_w     = mbi->framebuffer_width;
            fb_h     = mbi->framebuffer_height;
            fb_bpp   = 32;
            serial_puts("FB: multiboot framebuffer accepted\n");
        } else {
            serial_puts("FB: no usable framebuffer\n");
            panic_no_fb(magic);
        }
    }

    serial_puts("FB: init addr="); serial_puthex((uint32_t)fb_addr);
    serial_puts(" w="); serial_puthex(fb_w);
    serial_puts(" h="); serial_puthex(fb_h); serial_puts("\n");

    fb_init(fb_addr, fb_pitch, fb_w, fb_h, fb_bpp);
    serial_puts("fb_init done\n");

    if (g_textlog) {
        /* текстовый режим — без приветственного экрана. */
        tc_start();
        serial_puts("TEXTLOG: консоль активна (накопленный журнал показан)\n");
    } else {
    fb_grad_v(0, 0, fb.width, fb.height,
              rgb(0x05, 0x21, 0x6E), rgb(0x32, 0x6B, 0xCE));
    serial_puts("fb_grad_v done\n");

    const char *t1 = "Egorov OS";
    const char *t2 = "Добро пожаловать в Egorov Os";
    const char *t3 = EGOROV_TAG " Сборка " EGOROV_OS_BUILD_S;  /* [#473]: версия заморожена, СБОРКА растёт */
    int title_scale = 3;
    int x1 = ((int)fb.width  - text_width(t1) * title_scale) / 2;
    int ty = (int)fb.height  / 2 - 60;
    serial_puts("drawing text t1...\n");
    text_draw_str_scaled_shadow(x1, ty, t1, COL_WHITE, COL_BLACK, title_scale);
    serial_puts("t1 done\n");
    int x2 = ((int)fb.width  - text_width(t2)) / 2;
    text_draw_str(x2, ty + 80, t2, COL_WHITE);
    serial_puts("t2 done\n");
    int x3 = ((int)fb.width  - text_width(t3)) / 2;
    text_draw_str(x3, ty + 110, t3, rgb(0xC0, 0xD8, 0xFF));
    serial_puts("t3 done, calling fb_present...\n");
    fb_present();
    serial_puts("fb_present done\n");
    } /* конец else — обычный приветственный экран */

    /* ── Step 5: hardware initialisation ─────────────────────────────────*/
    serial_puts("ata_init...\n");
    ata_init();
    serial_puts("ata_init done, present=");
    serial_puts(ata_primary_master.present ? "YES" : "NO"); serial_puts("\n");
    ahci_init();   /* AHCI SATA fallback: no-op if ATA already found a drive */
    serial_puts("ahci_init done\n");

    /* USB оптическая мышь (HID boot protocol, UHCI).
       usb_init() [mass storage] намеренно оставлен выключен: при
       отсутствии USB-накопителя контроллер UHCI остаётся свободен
       для мыши. */
    ps2_init(fb.width, fb.height);
    serial_puts("ps2_init done\n");

    /* ── «ЭТАП 2» (порты 2.0 на реальном железе): ПОРЯДОК ИНИЦИАЛИЗАЦИИ
     * USB ПЕРЕДЕЛАН ЕЩЁ РАЗ.  История: OHCI-компаньоны зондировались
     * ВНУТРИ mouse_hw_init — ДО ehci_init.  Полноценный EHCI потом ставил
     * CONFIGFLAG=1 и ЗАБИРАЛ порты обратно у компаньонов — уже
     * перечисленная мышь из порта 2.0 срывалась, и всё висело на
     * «повезёт/не повезёт» с CSC на следующем секундном опросе.
     *
     * Новый порядок:
     *   1) mouse_hw_init       — ТОЛЬКО xHCI-скан (3.0) + handoff;
     *   2) ehci_init           — порты 2.0: HS-устройства напрямую, FS/LS
     *                            — отпускаются компаньонам (Port Owner);
     *                            ДВА контроллера, каждый со своим
     *                            расписанием (AMD FCH: 0:12.2 и 0:13.2);
     *   3) mouse_hw_init_ohci  — OHCI-компаньоны зондятся ПОСЛЕ EHCI:
     *                            LS/FS-порты уже им отданы навсегда;
     *   4) usb_mouse_init      — UHCI (QEMU piix3);
     *   5) usb_uhci_kbd_init   — клавиатура UHCI (shared-захват).
     *
     * «Этап 2» в kernel/ehci.c закрыл остальное: DATA-тоггл DATA1 в
     * контрольных передачах (реальные устройства отклоняли DATA0 —
     * клавиатура 2.0 не перечислялась ВООБЩЕ), пер-инстансные
     * расписания, ретраи неперечисленных портов, сброс PED-портов BIOS. */
    const char *cmd_usb = (mbi->flags & (1 << 2)) ? (const char*)mbi->cmdline : 0;
    /* Глубокий аудит (G4, п.9b, #11/28-класс): force_usb — ТОЛЬКО явный
     * cmdline «usbmouse»/пункт 2 (g_sticklog).  Это НЕ автодетект и НЕ
     * принудительное переключение: PS/2-инициализация выше прошла в любом
     * случае; в mouse_hw.c g_force_usb влияет лишь на диагностику/печать
     * (скан выполняется ВСЕГДА), а в ehci.c он только запрещает ОТКАТ
     * CONFIGFLAG.  Живая PS/2-клавиатура (ps2_kbd_alive) ни при каком
     * force_usb не отключается — оба guard'а построены на живости PS/2.
     * Автодетект пути: без клавы -> USB-натив; с клавой -> USB только для
     * мыши/доп-портов. */
    bool force_usb = cmdline_contains(cmd_usb, "usbmouse") || g_sticklog;
    mouse_hw_force_usb(force_usb ? 1 : 0);
    if (!ps2_kbd_alive()) {
        serial_puts("USB: PS/2-клавиатуры нет - беру USB под контроль:\n");
        serial_puts("USB: клавиатуру обслужит нативный драйвер (UHCI/EHCI/xHCI).\n");
    }

    /* [#454] usbmap_boot_print ДО mouse_hw_init: его первый вызов делает
     * memset(g_picks) (СМАЗ-защита) — раньше он стирал записи выборов,
     * которые xHCI-часть уже успела сделать в mouse_hw_init. */
    usbmap_boot_print();
    usbdev_boot_print();   /* 0.3.0: шесть драйверов + реестр */

    /* 1) xHCI (USB 3.0): скан + BIOS-handoff + перечисление мыши/клавы */
    mouse_hw_init(fb.width, fb.height);
    serial_puts("mouse_hw_init done, xhci=");
    serial_puts(mouse_hw_present() ? "YES" : "NO"); serial_puts("\n");

    /* 2) EHCI (USB 2.0): HS-мышь/клава напрямую; FS/LS — компаньонам */
    ehci_init(fb.width, fb.height, force_usb ? 1 : 0);
    serial_puts("ehci_init done, mouse=");
    serial_puts(ehci_mouse_active() ? "YES" : "NO");
    serial_puts(" kbd=");
    serial_puts(ehci_kbd_present() ? "YES" : "NO");
    serial_puts("\n");

    /* 3) «ЭТАП 2»: OHCI-компаньоны — СТРОГО ПОСЛЕ ehci_init: EHCI уже
       раздал LS/FS-порты компаньонам навсегда (Port Owner) и перечислил
       HS-устройства сам.  Мыши/клавы full/low-speed из портов 2.0
       зондятся здесь в финальном состоянии. */
    mouse_hw_init_ohci();
    serial_puts("mouse_hw_init_ohci done\n");

    /* 4) UHCI-мышь (HID boot, port 1.x и «2.0-порты у компаньонов»).
       usb_init() [mass storage] намеренно оставлен выключен: при
       отсутствии USB-накопителя контроллер UHCI остаётся свободен
       для мыши. */
    usb_mouse_init(fb.width, fb.height);
    serial_puts("usb_mouse_init done, usb=");
    serial_puts(usb_mouse_present() ? "YES" : "NO");
    serial_puts(" ps2=");
    serial_puts(ps2_mouse_present() ? "YES" : "NO"); serial_puts("\n");

    /* [#412] USB-КАРТА ПОРТОВ: после подъёма ВСЕХ USB-контроллеров и
       ВСЕХ транспортов (xHCI → EHCI → OHCI → UHCI-мышь → sticklog →
       UHCI-клавиатура) печатаем ПОЛНЫЙ список портов — какой это порт
       (USB2/USB3), занят/пуст, чем занят и работает ли.  [#455] печатаем
       ПОСЛЕ usb_uhci_kbd_init: статус UHCI-клавиатуры тоже виден.
       Запрос пользователя: «программа ищет все порты и пишет в логе,
       какой это порт и какой» — вся картина одной таблицей. Дальше карту
       обновляет горячий путь [#451] (привязка/отключение). */
    usb_uhci_kbd_init();
    serial_puts("usb_uhci_kbd_init done, kbd=");
    serial_puts(usb_uhci_kbd_present() ? "YES" : "NO");
    serial_puts("\n");
    mouse_hw_portmap();

    /* ── sticklog — [LOGS] ЭТАП 11: ПОЛНАЯ запись журнала на флешку.
     * Пункт 2 GRUB «LOGS to USB stick» раньше ВРАЛ: носитель искался,
     * но запись была отключена.  Теперь: usb_init() поднимает MSD на
     * UHCI (на OHCI/xHCI накопитель уже перечислен выше мышиной
     * инициализацией), затем ls_stick_prepare() находит накопитель на
     * ЛЮБОМ из трёх транспортов, разбирает MBR/GPT, ищет раздел LOGS
     * (FAT32 с меткой тома «LOGS») и открывает/создаёт EGOROV.LOG.
     * Периодическая выгрузка — ls_tick() из usbdev_tick() (1 Гц);
     * финальная — ls_flush_final() при выходе из textlog.  Отказ любого
     * шага — честная строка на экран/COM1; ОС продолжает работать. ── */
    if (g_sticklog) {
        serial_puts("LOGS: usb_init (подъём MSD на UHCI для sticklog)...\n");
        usb_init();
        serial_puts("LOGS: usb_init done, MSD(UHCI)=");
        serial_puts(usb_drive.present ? "YES" : "NO");
        serial_puts("\n");
        {
            uint32_t cap_kb = cmdline_int(
                (mbi->flags & (1 << 2)) ? (const char *)mbi->cmdline : 0,
                "logcap");
            ls_stick_prepare(cap_kb ? cap_kb * 1024u : 0u);
        }
    }

    /* ── UHCI-клавиатура (HID boot protocol): uhci_core/uhci_ctl/uhci_kbd
       + порт usb_uhci_kbd.c. Запускается ПОСЛЕДНИМ из USB-транспортов:
       свободный UHCI забирает целиком, а контроллер с UHCI-мышью ДЕЛИТ
       с ней — QH клавиатуры подцепляется за QH мыши горизонтальной
       ссылкой (usb_mouse_sched_chain), мышь и клава живут на одном
       контроллере. Отчёты идут в usb_kbd_parse_boot("UHCI") -> очередь
       ps2.c -> терминал/Блокнот/рабочий стол.  [#455] вызов ПЕРЕСЁН выше
       (до mouse_hw_portmap), здесь осталась только подпись. ── */
    serial_puts("hw_init done\n");

    /* ── Step 6: choose mode ──────────────────────────────────────────────*/
    bool from_hdd = detect_hdd_boot(mbi);

    const char *cmd = (mbi->flags & (1 << 2)) ? (const char*)mbi->cmdline : 0;
    serial_puts("cmdline="); serial_puts(cmd ? cmd : "(null)"); serial_puts("\n");

    /* ── TEXTLOG — консоль отладки с выходом в GUI ───────────────────────
     * Рабочий стол НЕ запускается, пока не нажаты ESC или Enter: новые
     * MOUSEHW-ошибки (cc=...) печатаются на экран вживую, консоль не
     * тупик. */
    if (g_textlog) {
        serial_puts("\nTEXTLOG: GRUB: 1 LIVE / 2 ДИАГНОСТИКА (экран) / 3 INSTALL.\n");
        serial_puts("TEXTLOG: мышь xHCI жива (порт 3, слот 1), СНИМКи честные.\n");
        serial_puts("TEXTLOG: Блокнот: Enter вставляет новую строку - клавиша\n");
        serial_puts("TEXTLOG: раньше застревала в диспетчере и до Блокнота не\n");
        serial_puts("TEXTLOG: доходила (печатались буквы, Enter молчал).\n");
        serial_puts("TEXTLOG: Курсор на рабочем столе: при любой живой мыши (USB\n");
        serial_puts("TEXTLOG: или PS/2) старт сразу в режиме мыши; Ctrl+J\n");
        serial_puts("TEXTLOG: переключает режим (видно в трее).\n");
        serial_puts("TEXTLOG: Режим 1 LIVE: USB-скан выполняется ВСЕГДА - та же\n");
        serial_puts("TEXTLOG: мышь, что в пункте 2; захват USB клавиатуру не\n");
        serial_puts("TEXTLOG: убивает - печать в Блокноте работает при активной мыши.\n");
        serial_puts("TEXTLOG: Hotplug: мышь в ЛЮБОЙ порт - определится заново без\n");
        serial_puts("TEXTLOG: перезагрузки, и в режиме 1 тоже.\n");
        serial_puts("TEXTLOG: Вертикаль мыши: Y больше не дёргается и не уезжает.\n");
        serial_puts("TEXTLOG: Оси X/Y читаются по report descriptor (смещение+размер,\n");
        serial_puts("TEXTLOG: 8/12/16 бит, report ID) - 12-битные приёмники едут ровно.\n");
        serial_puts("TEXTLOG: «Диалог с устройствами»: опрашивается КАЖДЫЙ порт:\n");
        serial_puts("TEXTLOG: клавиатура? - привязать и искать мышь дальше; мышь? -\n");
        serial_puts("TEXTLOG: занять; ввода нет? - пометить и пропустить.  Клавиатура\n");
        serial_puts("TEXTLOG: больше НЕ занимает место мыши в ЛЮБОМ порту, мышь ищется\n");
        serial_puts("TEXTLOG: по ВСЕМ портам, подключённая позже - находится опросом\n");
        serial_puts("TEXTLOG: портов (и при работающей мыши).\n");
        serial_puts("TEXTLOG: ТАБЛИЦА УСТРОЙСТВ USB (как в Windows/Linux): каждая\n");
        serial_puts("TEXTLOG: запись - «паспорт» vid/pid и роль (bInterfaceProtocol:\n");
        serial_puts("TEXTLOG: 1=клава, 2=мышь); подключение и отключение - события\n");
        serial_puts("TEXTLOG: в журнале, таблица печатается СНИМКами.\n");
        serial_puts("TEXTLOG: «Переподключил - мыши нет»: кольца TRB сбрасываются на\n");
        serial_puts("TEXTLOG: КАЖДОЕ перечисление (канон Linux xhci-ring.c: enqueue =\n");
        serial_puts("TEXTLOG: fresh dequeue); у мыши ЛИЧНЫЙ контекст, перечисление\n");
        serial_puts("TEXTLOG: клавиатуры после мыши её не портит; скан по ВСЕМ портам,\n");
        serial_puts("TEXTLOG: вторая мышь помечается, быстрая замена ловится по CSC.\n");
        serial_puts("TEXTLOG: Защита от СМАЗа ОЗУ: порт/слот мыши под штампом-тенью\n");
        serial_puts("TEXTLOG: (СМАЗ -> восстановление), фиктивное «устройство отключено»\n");
        serial_puts("TEXTLOG: отменяется, канарейки - в СНИМКах.  OHCI:\n");
        serial_puts("TEXTLOG: HcLSThreshold=0x628 (BIOS мог оставить 0: LS-транзакции\n");
        serial_puts("TEXTLOG: не выпускались, cc=5), recovery 100 мс + ретраи для LS.\n");
        serial_puts("TEXTLOG: Шрифт Terminus 8x16 (глифы полного роста), переработан\n");
        serial_puts("TEXTLOG: рабочий стол (фон, меню Пуск, кнопки).\n");
        serial_puts("TEXTLOG: Пуск без лишней линии, «Перезагрузка» в футере рядом с\n");
        serial_puts("TEXTLOG: выключением; модификаторы USB-клавы идут состоянием\n");
        serial_puts("TEXTLOG: отчёта (не потоком): «о» печатается, Ctrl не залипает.\n");
        serial_puts("TEXTLOG: Клаву и мышь различает LED-проба: «зажги NumLock+CapsLock»\n");
        serial_puts("TEXTLOG: - кто смог, тот клава, кто ответил STALL - мышь.  Клава\n");
        serial_puts("TEXTLOG: без мыши и в одиночестве больше не ломается.\n");
        serial_puts("TEXTLOG: ПЛЮС дроби в калькуляторе, крупные кнопки в блокноте и змейке.\n");
        serial_puts("TEXTLOG: Кнопка змейки опущена ПОД счёт и подсказку (она закрывала\n");
        serial_puts("TEXTLOG: поле), окно выше; «минус» в калькуляторе ставится и ДО\n");
        serial_puts("TEXTLOG: цифр - «±», потом число.\n");
        serial_puts("TEXTLOG: Классификация HID: дескриптор интерфейса -> report\n");
        serial_puts("TEXTLOG: descriptor -> LED-проба -> неоднозначно = КЛАВА.\n");
        serial_puts("TEXTLOG: «HID: addr=N port=.. proto=.. via=..» - видно, что выбрано.\n");
        serial_puts("TEXTLOG: На OHCI каждому устройству - свой USB-адрес (два на\n");
        serial_puts("TEXTLOG: адресе 0 больше не «каша»).  Устройства не теряем.\n");
        serial_puts("TEXTLOG: build " EGOROV_TAG " Сборка " EGOROV_OS_BUILD_S " - скан не останавливается: если на порту\n");
        serial_puts("TEXTLOG: устройство есть, но мышь не нашлась, оно перечисляется\n");
        serial_puts("TEXTLOG: заново БЕЗ перетыкания (фото: p4/p5=0x303, cc setup=5 -\n");
        serial_puts("TEXTLOG: два устройства на адресе 0 глушили друг друга; чужие\n");
        serial_puts("TEXTLOG: порты притушаются.  Мышь и клава работают как раньше.\n");
serial_puts("TEXTLOG: инициализация завершена; ожидание мыши.\n");
        serial_puts("TEXTLOG: ESC или Enter - рабочий стол сразу; 90 сек без единой\n");
        serial_puts("TEXTLOG: клавиши - рабочий стол откроется сам.\n");
        if (g_sticklog) {
            serial_puts("TEXTLOG: пункт 2: ВСЕ USB-накопители перечисляются,\n");
            serial_puts("TEXTLOG: запись идёт ТОЛЬКО на тот, где найден раздел\n");
            serial_puts("TEXTLOG: LOGS (FAT32, метка тома) - модель и ёмкость\n");
            serial_puts("TEXTLOG: выбранного видны выше. Журнал пишется с\n");
            serial_puts("TEXTLOG: ПЕРВОГО байта (ранний буфер 128К, кольцо 64К); потеря\n");
            serial_puts("TEXTLOG: байт при переполнении честно указывается.\n");
            serial_puts("TEXTLOG: выгрузка ~раз в секунду, финальная - перед\n");
            serial_puts("TEXTLOG: рабочим столом; итог - ниже).\n");
        }
        diag_final();   /* оба СНИМКа - в самом низу экрана */
        {
            rtc_time_t rt;
            uint8_t  last_s;
            int      waited = 0;
            uint32_t skip   = 0;
            rtc_read(&rt);
            last_s = rt.sec;
            for (;;) {
                mouse_state_t m;
                uint32_t tk;
                ps2_poll(&m);
                /* пока мышь не найдена — hotplug РЕАЛЬНО следит
                 * (xHCI/OHCI): «обнаружено новое устройство!» появляется
                 * живьём, без перезагрузки. */
                if (!mouse_hw_active()) mouse_hw_poll(&m);
                tk = ps2_poll_key();
                if (tk == KEY_ESC || tk == KEY_ENTER) break;  /* вручную */
                if (tk != 0) waited = 0;   /* активность - таймер заново */
                /* Страховка от «вечной» консоли: если клавиатура на
                 * реальном ПК молчит, через 90 секунд рабочий стол
                 * открывается сам.  RTC сверяем раз в ~4096 итераций. */
                if ((++skip & 0xFFFu) == 0u) {
                    rtc_read(&rt);
                    if (rt.sec != last_s) {
                        last_s = rt.sec;
                        if (++waited >= 90) {
                            serial_puts("TEXTLOG: 90 сек без клавиш - открывается рабочий стол\n");
                            break;
                        }
                    }
                }
            }
        }
        /* [LOGS] ЭТАП 11: финальная выгрузка журнала в EGOROV.LOG +
         * честный ИТОГ (ЗАПИСАНО N байт / НЕ ЗАПИСАНО: причина) — строка
         * видна на экране и в COM1 до открытия рабочего стола. */
        if (g_sticklog)
            ls_flush_final();
        /* sticklog-ветки больше нет: пункт 2 = диагностика в конце
         * экрана + рабочий стол (мышь проверяется сразу после ESC/Enter). */
        /* Журнал больше не дублируется на экран: иначе свежие serial-строки
         * (paint_release и т.п.) рисовали бы текст ПОВЕРХ рабочего стола. */
        g_textlog = 0;
        serial_puts("TEXTLOG: -> рабочий стол (desktop_run)\n");
    }

    if (cmdline_contains(cmd, "install")) {
        serial_puts("-> installer_run\n");
        installer_run();
    } else {
        serial_puts("-> desktop_run\n");
        desktop_run(from_hdd);
    }
    for (;;) __asm__ volatile("hlt");
}

