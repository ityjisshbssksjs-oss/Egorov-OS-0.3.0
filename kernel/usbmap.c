/* usbmap.c — сводка «тип порта -> список драйверов» для Egorov OS 0.3.0.
 *
 * Один журнал на ВСЕ шины: UHCI/OHCI/EHCI/xHCI-драйверы вызывают
 * usbmap_pick() после связывания устройства, PS/2 в списке обозначен
 * отдельным типом порта (не-USB).  Сам выбор устройства остаётся в
 * своих драйверах - здесь только список, печатается он и собирается
 * сводка выборов (для журнала и СНИМКа).
 */

#include "usbmap.h"
#include "string.h"

extern void serial_puts(const char *s);

/* prog-if -> имя контроллера/типа порта */
static const char *um_ctrl_name(int prog_if) {
    switch (prog_if & 0xF0u) {
        case 0x00: return "UHCI";
        case 0x10: return "OHCI";
        case 0x20: return "EHCI";
        case 0x30: return "xHCI";
        default:   return "USB";
    }
}

/* роль -> подпись в журнале */
static const char *um_role_name(const char *role) {
    if (!role[0]) return "?";
    if (role[0] == 'k') return "клавиатура";
    if (role[0] == 'm' && role[1] == 's') return "флешка";
    if (role[0] == 'm') return "мышь";
    return "?";   /* [#452] нераспознанная роль — честный «?», не «мышь» */
}

/* ограниченные копирование/склейка (в string.h их нет) */
static void um_copy(char *dst, int cap, const char *src) {
    int i = 0;
    while (src[i] && i < cap - 1) { dst[i] = src[i]; i++; }
    dst[i] = 0;
}
static void um_append(char *dst, int cap, const char *src) {
    int n = 0;
    while (dst[n]) n++;
    um_copy(dst + n, cap - n, src);
}

/* журнал выборов: роль+скорость+ШИНА+порт (Г11: шина участвовала в
 * матчинге — «клава xHCI#1 порт 1» и «клава xHCI#2 порт 1» больше
 * НЕ сливаются в одну запись; 12 записей вместо 8) */
#define UM_NPICK 12
typedef struct {
    uint8_t  used;
    char     role[12];
    char     speed[8];
    uint8_t  port;
    uint8_t  bus;
    uint16_t n;              /* сколько раз выбирался (переподключения) */
} um_pick_t;
static um_pick_t g_picks[UM_NPICK];

static bool g_boot_printed;

void usbmap_boot_print(void) {
    /* СМАЗ-защита: журнал выборов и флаг печати инициализируются явно
     * при первом вызове - ОЗУ под статиками не обязана быть нулём. */
    static uint32_t init_magic;
    const uint32_t  MAGIC_OK = 0x554D4130u;   /* "UMA0" */
    if (init_magic != MAGIC_OK) {
        init_magic = MAGIC_OK;
        g_boot_printed = false;
        memset(g_picks, 0, sizeof(g_picks));
    }
    if (g_boot_printed) return;
    g_boot_printed = true;
    serial_puts("USB: список драйверов ПО ТИПУ ПОРТА (версию спрашивает ОС):\n");
    serial_puts("USB:   порт «3.0» (xHCI)         : клавиатура 3.0, мышь 3.0\n");
    serial_puts("USB:   порт «2.0» (EHCI/компаньон): клавиатура 2.0, мышь 2.0\n");
    serial_puts("USB:   порт «1.x» (UHCI/OHCI)    : клавиатура 1.x, мышь 1.x\n");
    serial_puts("USB:   PS/2 — отдельные не-USB порты (к USB-драйверам не относятся)\n");
}

void usbmap_pick(int prog_if, const char *role, int port, const char *speed) {
    const char *rl = role ? role : "?";
    const char *sp = speed ? speed : "?";
    uint8_t bus = (uint8_t)(prog_if & 0xF0u);
    int slot = -1;
    for (int i = 0; i < UM_NPICK; i++) {
        if (g_picks[i].used &&
            !strcmp(g_picks[i].role, rl) &&
            !strcmp(g_picks[i].speed, sp) &&
            g_picks[i].bus == bus &&          /* Г11: ШИНА в матче */
            g_picks[i].port == (uint8_t)port) {
            slot = i;
            break;
        }
        if (slot < 0 && !g_picks[i].used) slot = i;
    }
    if (slot < 0) {
        /* Г11: журнал полон — затираем ЗАПИСЬ С НАИМЕНЬШИМ счётчиком
         * (самая неценная), а не всегда нулевую (живую мышь). */
        int victim = 0;
        for (int i = 1; i < UM_NPICK; i++)
            if (g_picks[i].n < g_picks[victim].n) victim = i;
        slot = victim;
    }
    um_pick_t *p = &g_picks[slot];
    if (p->used) p->n++;
    else         { p->used = 1; p->n = 1; }
    um_copy(p->role, (int)sizeof(p->role), rl);
    um_copy(p->speed, (int)sizeof(p->speed), sp);
    p->port = (uint8_t)port;
    p->bus  = bus;

    /* 0.3.0: печать выбора делает диспетчер usbdev (реестр, «ПОДОШЁЛ»,
     * «уже занят», «НЕИЗВЕСТНОЕ УСТРОЙСТВО»); здесь — только журнал. */
}

void usbmap_summary(char *out, int outsz) {
    if (!out || outsz < 2) return;
    out[0] = 0;
    int used = 0;
    for (int i = 0; i < UM_NPICK; i++) {
        if (!g_picks[i].used) continue;
        char nb[8];
        char line[96];
        line[0] = 0;
        if (used) um_append(line, (int)sizeof(line), "; ");
        um_append(line, (int)sizeof(line), um_role_name(g_picks[i].role));
        um_append(line, (int)sizeof(line), "-");
        um_append(line, (int)sizeof(line), g_picks[i].speed);
        um_append(line, (int)sizeof(line), " порт ");
        um_append(line, (int)sizeof(line), itoa((uint32_t)g_picks[i].port, nb, 10));
        um_append(line, (int)sizeof(line), " (x");
        um_append(line, (int)sizeof(line), itoa((uint32_t)g_picks[i].n, nb, 10));
        um_append(line, (int)sizeof(line), ")");
        if ((int)strlen(out) + (int)strlen(line) >= outsz - 1) break;
        um_append(out, outsz, line);
        used++;
    }
    if (!used) um_copy(out, outsz, "выборов драйверов ещё не было");
}
