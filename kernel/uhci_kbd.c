/* ─────────────────────────────────────────────────────────────────────────
 *  uhci_kbd.c — HID boot-протокол клавиатуры.
 *
 *  Формат boot-отчёта (8 байт, фиксирован протоколом):
 *    byte 0 — модификаторы: бит0 LCtl, 1 LShift, 2 LAlt, 3 LGui,
 *                            4 RCtl, 5 RShift, 6 RAlt, 7 RGui;
 *    byte 1 — зарезервировано (OEM);
 *    bytes 2..7 — до шести usage id нажатых клавиш (0 = пусто,
 *                            1 = rollover-ошибка).
 *
 *  Опрос: раз в uhci_kbd_poll() проверяется TD прерываний. Если TD
 *  завершён без ошибок — в DMA-пуле лежит свежий отчёт: инвертируем
 *  data toggle, переармливаем TD, разбираем нажатия (сравнение с
 *  предыдущим отчётом), обновляем LED при переключении Num/Caps/Scroll.
 *
 *  LED-индикаторы отправляются управляющим запросом SET_REPORT
 *  (bmReqType 0x21, wValue 0x0200 = output-отчёт №0, 1 байт маски:
 *  бит0 NumLock, бит1 CapsLock, бит2 ScrollLock).
 * ───────────────────────────────────────────────────────────────────────── */

#include "uhci_int.h"

/* ── Модификаторы ── */
#define MOD_CTRL  0x11u
#define MOD_SHIFT 0x22u
#define MOD_ALT   0x44u
#define MOD_GUI   0x88u

/* ── Таблицы перевода usage id → ASCII ── */
static const uint8_t k_plain[256] = {
    [0x04] = 'a', [0x05] = 'b', [0x06] = 'c', [0x07] = 'd',
    [0x08] = 'e', [0x09] = 'f', [0x0A] = 'g', [0x0B] = 'h',
    [0x0C] = 'i', [0x0D] = 'j', [0x0E] = 'k', [0x0F] = 'l',
    [0x10] = 'm', [0x11] = 'n', [0x12] = 'o', [0x13] = 'p',
    [0x14] = 'q', [0x15] = 'r', [0x16] = 's', [0x17] = 't',
    [0x18] = 'u', [0x19] = 'v', [0x1A] = 'w', [0x1B] = 'x',
    [0x1C] = 'y', [0x1D] = 'z',
    [0x1E] = '1', [0x1F] = '2', [0x20] = '3', [0x21] = '4',
    [0x22] = '5', [0x23] = '6', [0x24] = '7', [0x25] = '8',
    [0x26] = '9', [0x27] = '0',
    [0x2C] = ' ',
    [0x2D] = '-', [0x2E] = '=', [0x2F] = '[', [0x30] = ']',
    [0x31] = '\\', [0x32] = '#',
    [0x33] = ';', [0x34] = '\'', [0x35] = '`',
    [0x36] = ',', [0x37] = '.', [0x38] = '/',
    [0x54] = '/', [0x55] = '*', [0x56] = '-', [0x57] = '+',
    [0x59] = '1', [0x5A] = '2', [0x5B] = '3', [0x5C] = '4',
    [0x5D] = '5', [0x5E] = '6', [0x5F] = '7', [0x60] = '8',
    [0x61] = '9', [0x62] = '0', [0x63] = '.',
    [0x64] = '\\',
};

static const uint8_t k_shift[256] = {
    [0x04] = 'A', [0x05] = 'B', [0x06] = 'C', [0x07] = 'D',
    [0x08] = 'E', [0x09] = 'F', [0x0A] = 'G', [0x0B] = 'H',
    [0x0C] = 'I', [0x0D] = 'J', [0x0E] = 'K', [0x0F] = 'L',
    [0x10] = 'M', [0x11] = 'N', [0x12] = 'O', [0x13] = 'P',
    [0x14] = 'Q', [0x15] = 'R', [0x16] = 'S', [0x17] = 'T',
    [0x18] = 'U', [0x19] = 'V', [0x1A] = 'W', [0x1B] = 'X',
    [0x1C] = 'Y', [0x1D] = 'Z',
    [0x1E] = '!', [0x1F] = '@', [0x20] = '#', [0x21] = '$',
    [0x22] = '%', [0x23] = '^', [0x24] = '&', [0x25] = '*',
    [0x26] = '(', [0x27] = ')',
    [0x2C] = ' ',
    [0x2D] = '_', [0x2E] = '+', [0x2F] = '{', [0x30] = '}',
    [0x31] = '|', [0x32] = '~',
    [0x33] = ':', [0x34] = '"', [0x35] = '~',
    [0x36] = '<', [0x37] = '>', [0x38] = '?',
    [0x54] = '/', [0x55] = '*', [0x56] = '-', [0x57] = '+',
    [0x59] = '1', [0x5A] = '2', [0x5B] = '3', [0x5C] = '4',
    [0x5D] = '5', [0x5E] = '6', [0x5F] = '7', [0x60] = '8',
    [0x61] = '9', [0x62] = '0', [0x63] = '.',
    [0x64] = '|',
};

/* Имена непечатаемых клавиш (для колбэка on_key с ascii = -1). */
static const char *const k_name[256] = {
    [0x28] = "Enter", [0x29] = "Esc", [0x2A] = "Backspace",
    [0x2B] = "Tab",
    [0x39] = "CapsLock",
    [0x3A] = "F1",  [0x3B] = "F2",  [0x3C] = "F3",  [0x3D] = "F4",
    [0x3E] = "F5",  [0x3F] = "F6",  [0x40] = "F7",  [0x41] = "F8",
    [0x42] = "F9",  [0x43] = "F10", [0x44] = "F11", [0x45] = "F12",
    [0x46] = "PrintScreen", [0x47] = "ScrollLock", [0x48] = "Pause",
    [0x49] = "Insert", [0x4A] = "Home", [0x4B] = "PageUp",
    [0x4C] = "Delete", [0x4D] = "End",  [0x4E] = "PageDown",
    [0x4F] = "Right", [0x50] = "Left", [0x51] = "Down", [0x52] = "Up",
    [0x53] = "NumLock", [0x58] = "KeypadEnter",
    [0x59] = "End", [0x5A] = "Down", [0x5B] = "PageDown",
    [0x5C] = "Left", [0x5D] = "Keypad5", [0x5E] = "Right",
    [0x5F] = "Home", [0x60] = "Up", [0x61] = "PageUp",
    [0x62] = "Insert", [0x63] = "Delete",
    [0x65] = "Application", [0x66] = "Power",
    [0x67] = "Keypad=", [0x68] = "F13", [0x69] = "F14",
    [0x6A] = "F15", [0x6B] = "F16", [0x6C] = "F17", [0x6D] = "F18",
    [0x6E] = "F19", [0x6F] = "F20", [0x70] = "F21", [0x71] = "F22",
    [0x72] = "F23", [0x73] = "F24",
};

/* ── Состояние модуля ── */
static uhci_kbd_key_fn    g_key_fn;
static uhci_kbd_report_fn g_report_fn;

static uint8_t k_prev[8];                /* предыдущий отчёт            */
static uint8_t k_led_num, k_led_caps, k_led_scroll;

/* ── Регистрация колбэков ── */
void uhci_kbd_on_key(uhci_kbd_key_fn fn)         { g_key_fn = fn; }
void uhci_kbd_on_report(uhci_kbd_report_fn fn)   { g_report_fn = fn; }

/* ── Вооружить/переармить TD прерываний ──
 * Перезаписываем и статус, и token (data toggle), и elem_link QH:
 * аппаратура продвигает elem_link по мере завершения TD, а по NAK на
 * реальном железе TD вообще деактивируется — в обоих случаях расписание
 * нужно восстановить явно. */
static void k_arm_td(void)
{
    volatile uhci_td_t *td =
        (volatile uhci_td_t *)(void *)(uk_pool() + UK_OFF_TD_KBD);
    volatile uhci_qh_t *qh =
        (volatile uhci_qh_t *)(void *)(uk_pool() + UK_OFF_QH_INT);

    td->link = UK_LINK_TERM;
    td->token = UK_TD_TOKEN(UK_PID_IN, uk.dev_addr, uk.kbd_ep,
                            uk.kbd_toggle, uk.kbd_mps);
    td->buffer = uk_phys((const void *)(uk_pool() + UK_OFF_REPORT));
    td->ctrl_sts = UK_TD_ACTIVE | UK_TD_CERR3 |
                   (uk.low_speed ? UK_TD_LS : 0u);
    qh->elem_link = UK_LINK_TD(uk_phys((const void *)td));
    uk_dma();          /* [#403] свежий TD/QH — в ОЗУ до старта    */
}

/* ── Привязка/отвязка ── */
int uk_kbd_bind(int port, uint8_t iface, uint8_t ep, uint8_t mps)
{
    uk.kbd_port = port;
    uk.kbd_iface = iface;
    uk.kbd_ep = ep;
    uk.kbd_mps = mps;
    uk.kbd_toggle = 0;                    /* после SET_CONFIG — DATA0 */
    uk.kbd_errs = 0;
    uk.kbd_bound = 1;
    memset(k_prev, 0, sizeof k_prev);
    k_led_num = k_led_caps = k_led_scroll = 0;
    k_arm_td();
    return 0;
}

void uk_kbd_unbind(const char *why)
{
    volatile uhci_qh_t *qh;

    if (!uk.kbd_bound)
        return;
    hal_log("UHCI-KBD: клавиатура отвязана:");
    hal_log(why);
    qh = (volatile uhci_qh_t *)(void *)(uk_pool() + UK_OFF_QH_INT);
    qh->elem_link = UK_LINK_TERM;
    uk.kbd_bound = 0;
}

int uhci_kbd_port(void) { return uhci_kbd_present() ? (int)uk.kbd_port : -1; }

/* ── Публичный: порт привязанной клавиатуры на ДАННОМ контроллере ──
 * Аудит #234 (coord): драйвер UHCI-мыши (usb_mouse.c, um_quiesce_others)
 * гасит соседние порты на время перечисления — порт РАБОТАЮЩЕЙ
 * клавиатуры трогать нельзя (PE обратно вернёт только сброс порта,
 * который клавиатурный драйвер уже не делает).  Возвращает 0..3, если
 * клавиатура привязана на контроллере с I/O-базой io, иначе -1. */
int uhci_kbd_bound_port(uint16_t io)
{
    if (!uk.attached || !uk.kbd_bound)
        return -1;
    if (uk.io != (uint32_t)io)
        return -1;
    if (uk.kbd_port < 0 || uk.kbd_port >= 4)
        return -1;
    return uk.kbd_port;
}

int uhci_kbd_present(void)
{
    return (uk.attached && uk.kbd_bound) ? 1 : 0;
}

uint32_t uhci_kbd_int_qh_link(void)
{
    if (!uk.attached)
        return 0;
    return UK_LINK_QH(uk_phys((const void *)(uk_pool() + UK_OFF_QH_INT)));
}

/* ── LED-индикаторы ── */
static void k_send_leds(void)
{
    if (!uk.kbd_bound)
        return;
    {
        uint8_t *b = (uint8_t *)(void *)(uk_pool() + UK_OFF_LED);
        b[0] = (uint8_t)((k_led_num ? 1u : 0u) |
                         (k_led_caps ? 2u : 0u) |
                         (k_led_scroll ? 4u : 0u));
        if (uk_ctl_transfer(USB_RT_OUT | USB_REQTYPE_CLASS |
                            USB_RECIPIENT_INTERFACE,
                            HID_REQ_SET_REPORT,
                            0x0200u, (uint16_t)uk.kbd_iface, 1, b) < 0 &&
            uhci_kbd_verbose)
            hal_log("UHCI-KBD: SET_REPORT (LED) не прошёл");
    }
}

int uhci_kbd_set_leds(int num_lock, int caps_lock, int scroll_lock)
{
    k_led_num = num_lock ? 1 : 0;
    k_led_caps = caps_lock ? 1 : 0;
    k_led_scroll = scroll_lock ? 1 : 0;
    k_send_leds();
    return 0;
}

/* ── Разбор отчёта ── */
static int k_held(uint8_t u)
{
    for (int i = 2; i < 8; i++)
        if (k_prev[i] == u)
            return 1;
    return 0;
}

/* ASCII-перевод с учётом Shift, CapsLock и NumLock. */
static int k_translate(uint8_t u, uint8_t mods)
{
    int shift = (mods & MOD_SHIFT) != 0;
    uint8_t c = shift ? k_shift[u] : k_plain[u];

    if (!c)
        return -1;
    if (k_led_caps && u >= 0x04 && u <= 0x1D)
        c = shift ? k_plain[u] : k_shift[u];
    if (u >= 0x59 && u <= 0x63 && !k_led_num)
        return -1;                        /* keypad без NumLock — навигация */
    return (int)c;
}

static void k_handle_report(const uint8_t *r, int len)
{
    uint8_t mods;
    int i;

    /* [#437] до 16 байт (было 8): клавиатура, оставшаяся в report-протоколе
     * с Report-ID, шлёт [ID][mods][rsv][k1..k5] — 9 байт.  Кап на 8 не
     * давал эвристике Report-ID в usb_kbd_parse_boot сработать НИКОГДА,
     * и первый байт читался как «модификаторы». */
    if (len > 16)
        len = 16;
    if (len < 3)
        return;
    mods = r[0];

    if (g_report_fn)
        g_report_fn(r, len);

    for (i = 2; i < len && i < 8; i++) {   /* разбор — только boot-часть */
        uint8_t u = r[i];

        if (u < 4)
            continue;                     /* 0 пусто, 1 rollover, 2-3 rsvd */
        if (k_held(u))
            continue;                     /* удержание — не событие */

        if (u == 0x39) { k_led_caps ^= 1;   k_send_leds(); }
        if (u == 0x53) { k_led_num ^= 1;    k_send_leds(); }
        if (u == 0x47) { k_led_scroll ^= 1; k_send_leds(); }

        if (g_key_fn)
            g_key_fn(u, mods, k_translate(u, mods));
    }

    for (i = 0; i < 8; i++)
        k_prev[i] = (i < len) ? r[i] : 0;
}

/* ── Публичное: опрос ── */
void uhci_kbd_poll(void)
{
    volatile uhci_td_t *td;
    uint32_t sts;
    uint32_t len;

    if (!uk.attached || !uk.kbd_bound)
        return;

    /* Раз в ~16 опросов — не отключили ли устройство от порта.
     * (Чтение PORTSC — две инструкции in; на кадровой частоте рабочего
     * стола это даёт реакцию на отключение ~1-2 сек.) */
    if ((++uk.poll_cnt & 0x0Fu) == 0) {
        if (!(uk_rd16(UHCI_PORTSC(uk.kbd_port)) & PORTSC_CCS)) {
            uk_kbd_unbind("устройство отключено от порта");
            return;
        }
    }

    td = (volatile uhci_td_t *)(void *)(uk_pool() + UK_OFF_TD_KBD);
    uk_dma();              /* [#403] статус/отчёт пишет контроллер */
    sts = td->ctrl_sts;

    if (sts & UK_TD_ACTIVE)
        return;                           /* TD ждёт — новых событий нет */

    if (sts & UK_TD_ERR_MASK) {
        if (++uk.kbd_errs > 2000) {
            uk_kbd_unbind("серия ошибок interrupt-TD — устройство умерло?");
            return;
        }
        if (uhci_kbd_verbose)
            hal_log_hex("UHCI-KBD: ошибка interrupt-TD, sts=", sts);
        if (sts & UK_TD_STALLED) {
            /* Endpoint встал: снять halt и вернуть toggle в DATA0. */
            uk_ctl_transfer(USB_RT_OUT | USB_REQTYPE_STANDARD |
                            USB_RECIPIENT_ENDPOINT,
                            USB_REQ_CLEAR_FEATURE, USB_FEATURE_ENDPOINT_HALT,
                            (uint16_t)(0x80u | uk.kbd_ep), 0, NULL);
            uk.kbd_toggle = 0;
        }
        k_arm_td();
        return;
    }

    if (sts & UK_TD_NAK) {
        /* Реальное железо завершает TD по NAK, когда данных нет. */
        k_arm_td();
        return;
    }

    /* Успешный отчёт: забрать данные, инвертировать toggle, переармить.
     * Аудит #245 (проверено, ок): разбор (k_handle_report -> колбэк
     * g_report_fn -> usb_kbd_parse_boot в порту usb_uhci_kbd.c) идёт
     * ТОЛЬКО при реально пришедшем отчёте (len>=3, actlen от TD) —
     * NAK-путь выше просто переармливает TD, пустышки не парсятся. */
    uk.kbd_errs = 0;
    len = uk_td_actlen(sts);
    if (len > uk.kbd_mps)
        len = uk.kbd_mps;

    if (len >= 3) {
        uint8_t tmp[8];
        const volatile uint8_t *src = uk_pool() + UK_OFF_REPORT;
        uint32_t i;
        for (i = 0; i < len && i < 8; i++)
            tmp[i] = src[i];
        k_handle_report(tmp, (int)len);
    }

    uk.kbd_toggle ^= 1u;
    k_arm_td();
}

/* ── Публичное: имя клавиши ── */
const char *uhci_kbd_key_name(uint8_t usage)
{
    if (usage < 4)
        return NULL;
    return k_name[usage];
}
