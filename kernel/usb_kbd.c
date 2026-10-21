/* usb_kbd.c — настоящая USB-клавиатура (HID boot protocol) для Egorov OS 0.3.0.
 *
 * Один разбор boot-отчёта на все шины: модификаторы отдаются СОСТОЯНИЕМ
 * (ps2_kbd_mods - boot-протокол кладёт полный байт в каждый отчёт, поэтому
 * Ctrl/Shift залипнуть не могут), клавиши - диффом массива usage
 * (были-ушли = break, появились = make) с инъекцией в общую очередь ps2.c.
 * Таблица HID -> набор 1 - 1-в-1 с mouse_hw.c (oh_kbd_process): одна
 * раскладка на все шины.
 */

#include "usb_kbd.h"
#include "ps2.h"
#include "ehci.h"
#include "mouse_hw.h"
#include "usb_uhci_kbd.h"
#include "usbdev.h"   /* [#462] usbdev_note_report — живость для КАРТЫ ГНЕЗД */
#include "string.h"

extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);
/* [#446] LED UHCI-клавиатуры (uhci_kbd.c, прототип — uhci_kbd.h) */
extern int uhci_kbd_set_leds(int num_lock, int caps_lock, int scroll_lock);

/* ── состояние разбора (один отчет - одно сравнение с прошлым) ── */
static uint8_t  g_prev[6];        /* usage из прошлого отчёта            */
static uint32_t g_pkts;           /* счётчик отчётов (первый - в журнал) */

void usb_kbd_init(void) {
    memset(g_prev, 0, sizeof(g_prev));
    g_pkts = 0;
}

/* HID usage -> сканкод набора 1 (make). *ext=1 - код с префиксом E0
 * (инжектится двумя байтами: 0xE0, код).  0 = клавиша не поддерживается. */
static uint8_t hid_to_set1(uint8_t u, int *ext) {
    *ext = 0;
    static const uint8_t letters[26] = {   /* 0x04=a ... 0x1D=z */
        0x1E,0x30,0x2E,0x20,0x12,0x21,0x22,0x23,0x17,0x24,0x25,0x26,0x32,
        0x31,0x18,0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11,0x2D,0x15,0x2C
    };
    if (u >= 0x04 && u <= 0x1D) return letters[u - 0x04];
    if (u >= 0x1E && u <= 0x27) return (uint8_t)(0x02u + (u - 0x1Eu)); /* 1..0 */
    if (u >= 0x3A && u <= 0x43) return (uint8_t)(0x3Bu + (u - 0x3Au)); /* F1..F10 */
    switch (u) {                              /* F11/F12: в наборе 1 = 0x57/0x58 */
        case 0x44: return 0x57;
        case 0x45: return 0x58;
        default:   break;
    }
    switch (u) {
        case 0x28: return 0x1C;   /* Enter      */
        case 0x29: return 0x01;   /* Esc        */
        case 0x2A: return 0x0E;   /* Backspace  */
        case 0x2B: return 0x0F;   /* Tab        */
        case 0x2C: return 0x39;   /* Space      */
        case 0x2D: return 0x0C;   /* - _        */
        case 0x2E: return 0x0D;   /* = +        */
        case 0x2F: return 0x1A;   /* [ {        */
        case 0x30: return 0x1B;   /* ] }        */
        case 0x31: return 0x2B;   /* \ |        */
        case 0x33: return 0x27;   /* ; :        */
        case 0x34: return 0x28;   /* ' "        */
        case 0x35: return 0x29;   /* ` ~        */
        case 0x36: return 0x33;   /* , <        */
        case 0x37: return 0x34;   /* . >        */
        case 0x38: return 0x35;   /* / ?        */
        case 0x39: return 0x3A;   /* CapsLock   */
        case 0x53: return 0x45;   /* NumLock    */
        case 0x47: return 0x46;   /* ScrollLock */
        case 0x55: return 0x37;   /* KP *       */
        case 0x56: return 0x4A;   /* KP -       */
        case 0x57: return 0x4E;   /* KP +       */
        case 0x59: return 0x4F;   /* KP 1       */
        case 0x5A: return 0x50;   /* KP 2       */
        case 0x5B: return 0x51;   /* KP 3       */
        case 0x5C: return 0x4B;   /* KP 4       */
        case 0x5D: return 0x4C;   /* KP 5       */
        case 0x5E: return 0x4D;   /* KP 6       */
        case 0x5F: return 0x47;   /* KP 7       */
        case 0x60: return 0x48;   /* KP 8       */
        case 0x61: return 0x49;   /* KP 9       */
        case 0x62: return 0x52;   /* KP 0       */
        case 0x63: return 0x53;   /* KP .       */
        /* E0-навигация */
        case 0x49: *ext = 1; return 0x52;   /* Insert */
        case 0x4A: *ext = 1; return 0x47;   /* Home   */
        case 0x4B: *ext = 1; return 0x49;   /* PgUp   */
        case 0x4C: *ext = 1; return 0x53;   /* Delete */
        case 0x4D: *ext = 1; return 0x4F;   /* End    */
        case 0x4E: *ext = 1; return 0x51;   /* PgDn   */
        case 0x4F: *ext = 1; return 0x4D;   /* Right  */
        case 0x50: *ext = 1; return 0x4B;   /* Left   */
        case 0x51: *ext = 1; return 0x50;   /* Down   */
        case 0x52: *ext = 1; return 0x48;   /* Up     */
        case 0x54: *ext = 1; return 0x35;   /* KP /   */
        case 0x58: *ext = 1; return 0x1C;   /* KP Enter */
        default:   return 0;
    }
}

/* инжект make/break набора 1 (с E0-префиксом при необходимости). */
static void usb_inject_sc(uint8_t sc, int ext, int is_break) {
    if (ext) ps2_kbd_inject(0xE0);
    ps2_kbd_inject(is_break ? (uint8_t)(0x80u | sc) : sc);
}

/* сброс: все зажатые клавиши отпускаются break-ами, модификаторы гасятся. */
static void usb_kbd_reset(void) {
    for (int i = 0; i < 6; i++) {
        uint8_t u = g_prev[i];
        if (!u) continue;
        int ext = 0;
        uint8_t sc = hid_to_set1(u, &ext);
        if (sc) usb_inject_sc(sc, ext, 1);
    }
    memset(g_prev, 0, sizeof(g_prev));
    ps2_kbd_mods(0);
    ps2_kbd_detached();      /* «зажатый Enter» больше не держит ЛКМ */
}

void usb_kbd_parse_boot(const uint8_t *pkt, int len, const char *bus) {
    if (!pkt || len < 1) { usb_kbd_reset(); return; }

    if (g_pkts == 0) {
        serial_puts("USB-KBD: первый отчёт (");
        if (bus) serial_puts(bus);
        serial_puts(", ");
        serial_puthex((uint32_t)len);
        serial_puts(" байт):");
        for (int i = 0; i < len && i < 8; i++) {
            serial_puts(" ");
            serial_puthex(pkt[i]);
        }
        serial_puts("\n");
    }
    g_pkts++;
    usbdev_note_report(USBDEV_ROLE_KBD);   /* [#462] живость для КАРТЫ ГНЕЗД */

    /* Клавиатура с Report-ID префиксом шлёт 9 байт [ID][mods][rsv][k1..k6]
     * (честный boot-отчёт всегда 8).  Раньше ID «угадывался» по pkt[1]!=0:
     * при пустом отчёте [ID][00][00...] сдвига не было, и ID принимался за
     * модификаторы - фантомный Ctrl (j -> Ctrl+J) или Shift (ЗАГЛАВНЫЕ).
     * Теперь решение по ДЛИНЕ: 9+ байт = префикс есть (boot уже включён
     * SET_PROTOCOL'ом на всех шинах). */
    if (len >= 9) { pkt++; len--; }
    if (len < 1) return;

    /* модификаторы - СОСТОЯНИЕМ, а не потоком make/break: залипать
     * нечему (см. ps2_kbd_mods). */
    ps2_kbd_mods(pkt[0]);

    uint8_t cur[6];
    for (int i = 0; i < 6; i++) {
        cur[i] = (2 + i < len) ? pkt[2 + i] : 0;
        if (cur[i] <= 3) cur[i] = 0;      /* 0/rollover/POST-ошибки        */
    }

    /* отпускания (были в прошлом отчёте, нет в текущем) */
    for (int i = 0; i < 6; i++) {
        uint8_t u = g_prev[i];
        if (!u) continue;
        bool gone = true;
        for (int j = 0; j < 6; j++) if (cur[j] == u) { gone = false; break; }
        if (gone) {
            int ext = 0;
            uint8_t sc = hid_to_set1(u, &ext);
            if (sc) usb_inject_sc(sc, ext, 1);
        }
    }
    /* нажатия (есть в текущем, не было в прошлом) */
    for (int i = 0; i < 6; i++) {
        uint8_t u = cur[i];
        if (!u) continue;
        bool fresh = true;
        for (int j = 0; j < 6; j++) if (g_prev[j] == u) { fresh = false; break; }
        if (fresh) {
            int ext = 0;
            uint8_t sc = hid_to_set1(u, &ext);
            if (sc) usb_inject_sc(sc, ext, 0);
        }
    }
    memcpy(g_prev, cur, sizeof(g_prev));
}

/* ── транспорт EHCI (2.0) + UHCI (1.x): опрос qTD/TD клавиатуры ── */
void usb_kbd_poll(void) {
    usb_uhci_kbd_poll();     /* UHCI-клавиатура (usb_uhci_kbd.c) — первой */
    ehci_kbd_poll();
}

/* жива ли USB-клавиатура хотя бы на одной шине */
bool usb_kbd_present(void) {
    if (ehci_kbd_present()) return true;
    if (usb_uhci_kbd_present()) return true;   /* UHCI (usb_uhci_kbd.c) */
    return mouse_hw_kbd_present();   /* OHCI/xHCI-клава (mouse_hw.c) */
}

/* [#446] ДИСПЕТЧЕР LED ВСЕХ USB-клавиатур: вызывается ps2.c при каждом
 * тумблере NumLock/CapsLock (kbd_set_leds).  HID-порядок бит: 0 NumLock,
 * 1 CapsLock, 2 ScrollLock.  Раньше на тумблеры реагировала только
 * PS/2-клава и UHCI-путь; индикаторы EHCI/OHCI/xHCI-клавиатур не
 * менялись НИКОГДА (а проба перечисления зажигала 0x03 навечно). */
void usb_kbd_leds_all(bool num, bool caps) {
    uint8_t hid = (uint8_t)((num ? 1u : 0u) | (caps ? 2u : 0u));
    mouse_hw_kbd_set_leds(hid);              /* xHCI + OHCI (mouse_hw.c) */
    ehci_kbd_set_leds(hid);                  /* EHCI (ehci.c) */
    (void)uhci_kbd_set_leds(num ? 1 : 0, caps ? 1 : 0, 0);  /* UHCI */
}
