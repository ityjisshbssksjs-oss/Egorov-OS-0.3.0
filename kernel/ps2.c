#include "ps2.h"
#include "usb_mouse.h"
#include "mouse_hw.h"
#include "ehci.h"
#include "usbdev.h"
#include "usb_kbd.h"
#include "io.h"
#include "string.h"

/* COM1-лог реализован в kernel.c (QEMU -serial stdio).
   «ЭТАП 5» (G3): модуль печатает ОДИН новый e2e-маркер - первый
   синтез софт-автоповтора USB-клавиши (KBD-TYPEMATIC, см. блок
   typematic ниже); прочей печати отсюда по-прежнему нет. */
extern void serial_puts(const char *s);
/* [#446] диспетчер LED USB-клавиатур — реализован в usb_kbd.c */
extern void usb_kbd_leds_all(bool num, bool caps);

#define PS2_DATA   0x60
#define PS2_STATUS 0x64
#define PS2_CMD    0x64

static mouse_state_t g_mouse;
static int g_fb_w, g_fb_h;
static uint8_t mouse_packet[3];
static int mouse_phase = 0;
static int mouse_phase_stall = 0; /* тиков без байтов при недобранном пакете */
static bool mouse_present = false;
static bool prev_left = false, prev_right = false;
/* физическое состояние Enter (make 0x1C / break 0x9C).
   Покойные кадры без событий не сбрасывают флаг - только break. */
static bool enter_held = false;
/* тихий итог проверки клавиатуры (ACK на сброс 0xFF).
 * ACK есть  -> настоящая PS/2-клава: USB-контроллеры можно трогать.
 * ACK нет   -> клавиатура работает через BIOS-эмуляцию (внутренняя USB):
 *              USB-контроллеры трогать НЕЛЬЗЯ, handoff убивает эмуляцию -
 *              так «Клава отваливалась» на реальном ПК после захвата
 *              xHCI/EHCI/OHCI.  Режим «usbmouse» (cmdline) снимает защиту
 *              осознанно - тогда клавиатуру берёт на себя mouse_hw (HID).
 * Сам «проверятор» (KBD-диагностики, счётчик байт, ИТОГ, пульс)
 * удалён - остался только тихий детект. */
static bool kbd_alive = false;
/* [#446] состояние LED вынесено на уровень файла: опросчик клавиш
 * (раньше — локальные static) и опрашиватель USB-клавиатур (SET_REPORT)
 * должны видеть ОДНИ И ТЕ ЖЕ тумблеры Num/Caps. */
static bool numlock_on  = false;
static bool capslock_on = false;

/* очередь сканкодов (набор 1) от USB HID-клавиатуры (mouse_hw.c,
 * OHCI).  ps2_poll_key() разбирает её ПЕРВОЙ, поэтому USB-клавиатура
 * неотличима от PS/2 для всего остального кода ОС (Ctrl+J, Enter,
 * текстлог, терминал). */
#define KBDQ_SZ 64
static volatile uint8_t kbdq[KBDQ_SZ];
/* снапшот модификаторов USB-клавы НА МОМЕНТ инъекции байта: к моменту
 * разбора очереди Shift мог уже отпустить (быстрый набор) - регистр
 * букв не должен «прыгать».  Раньше брался СВЕЖИЙ отчёт. */
static volatile uint8_t kbdq_mod[KBDQ_SZ];
static volatile int     kbdq_r = 0, kbdq_w = 0;
static uint8_t kbd_live_mods = 0;
/* модификаторы для РАЗБИРАЕМОГО сейчас байта очереди (восстанавливается
 * из kbdq_get); декларация выше kbdq_get - там присваивается. */
static uint8_t g_usb_mods = 0;
/* «ЭТАП 5»: учёт USB-потока для софт-автоповтора (определён ниже,
 * в блоке typematic - после очереди, которой он пользуется). */
static void tm_track(uint8_t b);
void ps2_kbd_inject(uint8_t sc) {
    /* [#299] kbdq: r/w двигаются по +1 mod KBDQ_SZ (64 = степень двойки,
     * но и честный % корректен): переполнение детектится nw==r, байт
     * теряется осознанно, «затирания» живых байт не бывает - оба
     * указателя ходят только вперёд по кругу. */
    int nw = (kbdq_w + 1) % KBDQ_SZ;
    if (nw == kbdq_r) return;               /* очередь полна - байт теряем */
    kbdq[kbdq_w] = sc;
    kbdq_mod[kbdq_w] = kbd_live_mods;       /* модификаторы на момент клавиши */
    kbdq_w = nw;
}
static int kbdq_get(void) {
    if (kbdq_r == kbdq_w) return -1;
    uint8_t b = kbdq[kbdq_r];
    g_usb_mods = kbdq_mod[kbdq_r];          /* восстановить снапшот */
    kbdq_r = (kbdq_r + 1) % KBDQ_SZ;
    tm_track(b);      /* «ЭТАП 5»: make/break USB -> учёт автоповтора */
    return b;
}

/* ══ «ЭТАП 5» (G3): софт-typematic USB-клавиатур - «как в Windows» ══════
 * У PS/2-клавиатур автоповтор делает САМ контроллер (make-и льются из
 * порта 0x60, пока клавиша зажата). У USB HID повтора НЕТ: отчёт просто
 * держит usage зажатой клавиши, диф-разбор даёт одно нажатие. Windows
 * для USB-клавиатур делает ПРОГРАММНЫЙ повтор - делаем так же.
 *
 * ИСТОЧНИК - ТОЛЬКО USB: состояние ведётся из kbdq_get (очередь
 * USB-клавиатур); байты PS/2-порта этот путь не проходят, поэтому у
 * PS/2 не бывает двойного повтора (аппаратный + софт).
 *
 * ВРЕМЯ = КАДРЫ. Гонок очередей нет: вся ОС однопоточна, IRQ замаскиро-
 * ваны, put/get выполняются из одного цикла опроса. МЕСТО инкремента
 * tm_frames - ps2_poll(): её зовёт РОВНО ОДИН раз за итерацию КАЖДЫЙ
 * потребитель ввода (desktop.c, installer.c, текстлог kernel.c -
 * других циклов с ps2_poll_key нет: grep). В desktop.c при удержании
 * любой USB-клавиши цикл спит 15 мс (ps2_typematic_held в условии
 * сна), поэтому «кадр» = 15 мс и тайминги = Windows: задержка ~500 мс,
 * повтор ~30 мс. В циклах без сна (инсталлятор/текстлог) повторы идут
 * с частотой их итераций - терпимо: блокнот/терминал/змейка/калькулятор
 * получают клавиши ТОЛЬКО из desktop-цикла, а инсталлятор перерисовыва-
 * ет весь экран на каждой итерации (кадр там и так ~десятки мс).
 *
 * НЕ повторяем: модификаторы (USB-путь и так не инжектит их make - они
 * идут состоянием ps2_kbd_mods; фильтр оставлен как страховка), тумб-
 * леры Caps/Num/Scroll (поворачивали бы светодиоды 30 р/с) и Ctrl+J
 * (глобальный тумблер режима курсора не должен «машать» от повтора -
 * отсечён в месте синтеза). E0-расширенные НЕ-модификаторы (стрелки,
 * Delete и пр.) повторяются - с восстановлением E0-префикса. */
#define TM_DELAY 35u   /* кадров до первого повтора (~500 мс при кадре 15 мс) */
#define TM_RATE  2u    /* кадров между повторами (~30 мс - дефолт Windows)    */

static uint8_t  tm_key = 0;         /* зажатая USB-клавиша (make-код, 0 = нет) */
static uint8_t  tm_ext = 0;         /* она пришла с E0-префиксом                */
static uint8_t  tm_ext_next = 0;    /* в очереди встречен 0xE0: следующий байт
                                      * расширенный (префикс инжектится парой) */
static uint8_t  tm_mods = 0;        /* снапшот модификаторов на момент make    */
static uint32_t tm_frames = 0;      /* кадров удержания (двигает ps2_poll)     */
static uint32_t tm_last_repeat = 0; /* tm_frames момента последнего синтеза    */
static bool     tm_logged = false;  /* маркер KBD-TYPEMATIC - раз на удержание */

/* клавиши, автоповтор которых запрещён (см. комментарий блока выше). */
static bool tm_repeatable(uint8_t sc) {
    switch (sc) {
        case 0x1D: case 0x2A: case 0x36: case 0x38:   /* Ctrl/Shift/Alt     */
        case 0x3A: case 0x45: case 0x46:              /* Caps/Num/Scroll    */
            return false;
        default:
            return true;
    }
}

/* учёт потока USB-клавиатуры (вызывается из kbdq_get). Break инжектится
 * как 0x80|make (usb_kbd.c/mouse_hw.c - одна конвенция набора 1). */
static void tm_track(uint8_t b) {
    if (b == 0xE0) { tm_ext_next = 1; return; }   /* префикс: ждём код */
    uint8_t was_ext = tm_ext_next;   /* принадлежность ЭТОГО байта - до сброса */
    tm_ext_next = 0;
    if (b & 0x80u) {
        /* break: снимаем удержание ТОЛЬКО той же клавиши (другие зажатые
         * не должны терять повтор). E0- и numpad-варианты стрелок делят
         * один make-код - совпадения по коду достаточно. */
        if ((uint8_t)(b & 0x7Fu) == tm_key) { tm_key = 0; tm_logged = false; }
        return;
    }
    /* make: кандидат на автоповтор (последняя нажатая - как в Windows) */
    tm_key  = tm_repeatable(b) ? b : 0;
    tm_ext  = was_ext;
    tm_mods = g_usb_mods;    /* снапшот модификаторов именно этого байта */
    tm_frames = 0;
    tm_last_repeat = 0;
    tm_logged = false;
}

/* пора синтезировать автоповтор? Очередь USB обязана быть ПУСТОЙ (сначала
 * расходуются реальные события), клавиша удержана дольше задержки, с
 * прошлого синтеза прошло не меньше периода. */
bool ps2_typematic_due(void) {
    if (tm_key == 0) return false;
    if (kbdq_r != kbdq_w) return false;
    if (tm_frames < TM_DELAY) return false;
    return (uint32_t)(tm_frames - tm_last_repeat) >= TM_RATE;
}

/* удержана ли USB-клавиша? Аксессор для desktop-цикла: пока true, кадр
 * выравнивается до ~15 мс (hal_msleep), иначе скорость повтора зависела
 * бы от скорости пустого цикла (на реальном железе - тысячи итераций/с). */
bool ps2_typematic_held(void) {
    return tm_key != 0;
}

/* свежий байт модификаторов USB-клавиатуры (boot-протокол кладёт его
 * в КАЖДЫЙ пакет целиком): заливается в kbd_live_mods, а в очередь -
 * снапшотом на момент каждой клавиши (kbdq_mod).  Синтетический
 * make/break-поток раньше терял break - Ctrl «залипал». */
void ps2_kbd_mods(uint8_t mods) {
    kbd_live_mods = mods;    /* снапшот кладётся в kbdq при инъекции */
}

/* есть ли входной байт? Своя очередь USB-клавы или клавиатурный
 * байт порта 0x60; aux-байты (мышь, бит 5 статуса) ожиданием не считаются,
 * чтобы поток пакетов мыши не крутил цикл разбора на месте. */
bool ps2_key_pending(void) {
    if (kbdq_r != kbdq_w) return true;
    if (ps2_typematic_due()) return true;   /* «ЭТАП 5»: готов синтез повтора */
    uint8_t st = inb(PS2_STATUS);
    return (st & 1) != 0 && (st & 0x20) == 0;
}

static const char scancode_ascii[128] = {
    0,   27, '1','2','3','4','5','6','7','8','9','0','-','=', 8, '\t',
    'q','w','e','r','t','y','u','i','o','p','[',']','\n', 0,  'a','s',
    'd','f','g','h','j','k','l',';','\'','`', 0, '\\','z','x','c','v',
    'b','n','m',',','.','/', 0,  '*', 0,  ' ', 0,
};

static void wait_in(void) {
    for (int i = 0; i < 100000; i++) {
        if (!(inb(PS2_STATUS) & 2)) return;
    }
}
static void wait_out(void) {
    for (int i = 0; i < 100000; i++) {
        if (inb(PS2_STATUS) & 1) return;
    }
}

static void mouse_write(uint8_t v) {
    wait_in();
    outb(PS2_CMD, 0xD4);
    wait_in();
    outb(PS2_DATA, v);
}
static uint8_t mouse_read(void) {
    wait_out();
    return inb(PS2_DATA);
}

/* Ждём байт ИМЕННО от мыши (бит 5 статуса = данные aux-порта).
   Байты клавиатуры, пришедшие попутно, выбрасываются.  Возвращает -1 по
   таймауту: на реальной машине без PS/2 мыши контроллер молчит — старый
   способ (дождаться любого байта и назвать его ACK) давал ложное
   «мышь установлена», а курсор при этом не двигался. */
static int mouse_resp(int timeout_iters) {
    for (int i = 0; i < timeout_iters; i++) {
        uint8_t st = inb(PS2_STATUS);
        if (st & 1) {
            if (st & 0x20) return inb(PS2_DATA);
            (void)inb(PS2_DATA);
        }
    }
    return -1;
}

/* записать байт клавиатуре - порт 0x60 БЕЗ 0xD4 (0xD4 - это мышь). */
static void kbd_write(uint8_t v) {
    wait_in();
    outb(PS2_DATA, v);
}
/* ждём байт ИМЕННО от клавиатуры (бит 5 статуса = 0), aux-байты
 * пропускаем.  Возвращает -1 по таймауту: на машине без PS/2-клавиатуры
 * контроллер молчит - так мы и отличаем «живую Клаву» от эмуляции. */
static int kbd_resp(int timeout_iters) {
    for (int i = 0; i < timeout_iters; i++) {
        uint8_t st = inb(PS2_STATUS);
        if (st & 1) {
            uint8_t b = inb(PS2_DATA);
            if (!(st & 0x20)) { return b; }
        }
    }
    return -1;
}

/* Синхронизация LED с PS/2-клавиатурой (команда 0xED).  USB-клавы
 * игнорируем: их LED - SET_REPORT, вне рамок boot-протокола.
 * Вызывается из ps2_init (после сброса все LED погашены) и при
 * тумблере NumLock/CapsLock. */
static void kbd_set_leds(bool num, bool caps) {
    if (kbd_alive) {
        kbd_write(0xED);
        (void)kbd_resp(100000);              /* ACK */
        kbd_write((uint8_t)((num ? 1u : 0u) | (caps ? 2u : 0u)));
        (void)kbd_resp(100000);              /* ACK */
    }
    /* [#446] USB-клавиатуры (UHCI/EHCI/OHCI/xHCI) — тем же тумблером:
     * раньше их LED синхронизировал ТОЛЬКО UHCI-путь, а проба
     * перечисления зажигала NumLock|CapsLock НАВСЕГДА. */
    usb_kbd_leds_all(num, caps);
}

/* [#446] текущее состояние тумблеров — для проб LED при привязке
 * USB-клавиатур (см. mouse_hw.c). */
void ps2_kbd_led_state(int *num, int *caps) {
    if (num)  *num  = numlock_on  ? 1 : 0;
    if (caps) *caps = capslock_on ? 1 : 0;
}

void ps2_init(int fb_w, int fb_h) {
    g_fb_w = fb_w; g_fb_h = fb_h;
    g_mouse.x = fb_w / 2;
    g_mouse.y = fb_h / 2;
    g_mouse.left = g_mouse.right = false;
    g_mouse.moved = false;
    g_mouse.clicked_left = false;
    g_mouse.clicked_right = false;
    mouse_phase = 0;
    prev_left = false;
    prev_right = false;

    for (int i = 0; i < 32; i++) {
        if (inb(PS2_STATUS) & 1) (void)inb(PS2_DATA);
        else break;
    }

    wait_in();
    outb(PS2_CMD, 0xA8);

    wait_in();
    outb(PS2_CMD, 0x20);
    wait_out();
    uint8_t cfg = inb(PS2_DATA);
    cfg &= ~(1 << 5);         /* aux-часы ВКЛ (бит 5 = «выключены») */
    cfg &= ~(1 << 1);         /* IRQ12 off - поллинг */
    cfg &= ~(1 << 0);         /* IRQ1  off - поллинг */
    cfg |=  (1u << 6);        /* ТРАНСЛЯЦИЯ set2 -> set1 (бит 0x40!):
                               * раскладка ниже - таблица НАБОРА 1. BIOS
                               * обычно ставит бит сам, но UEFI-CSM и
                               * ноутбучные EC после холодного сброса - нет:
                               * без трансляции каждая клавиша декодируется
                               * неверно (a=Enter, двойные символы от
                               * break-префикса 0xF0).  Включаем сами. */
    wait_in();
    outb(PS2_CMD, 0x60);
    wait_in();
    outb(PS2_DATA, cfg);

    /* ── Детект присутствия: сброс aux-устройства (0xFF) ──
     * Настоящая PS/2 мышь отвечает 0xFA (ACK), 0xAA (самотест пройден),
     * 0x00 (device id).  При отсутствии устройства контроллер молчит.
     * v1.8: детект только по полному сбросу — без ложных срабатываний. */
    mouse_present = false;
    mouse_write(0xFF);
    int r = mouse_resp(200000);
    if (r == 0xFE) {                    /* контроллер просит повторить */
        mouse_write(0xFF);
        r = mouse_resp(200000);
    }
    if (r == 0xFA) {
        int st = mouse_resp(1000000);   /* 0xAA = BAT: часть мышей до 1 с
                                          * (500K браковал медленные
                                          * KVM/USB-адаптерные мыши) */
        (void)mouse_resp(200000);       /* 0x00 = device id */
        mouse_present = (st == 0xAA);
    }

    if (mouse_present) {
        mouse_write(0xF6);              /* defaults */
        (void)mouse_resp(200000);
        mouse_write(0xF4);              /* enable reporting */
        (void)mouse_resp(200000);
    }

    for (int i = 0; i < 32; i++) {
        if (inb(PS2_STATUS) & 1) (void)inb(PS2_DATA);
        else break;
    }

    /* ── тихий детект «Клавы» (БЕЗ печатного проверятора) ────────────────
     * Ничего не ломаем и ничего не печатаем: 0xAE посылает даже GRUB
     * (безопасно), клавиатуре - стандартный сброс 0xFF, ждём ACK 0xFA.
     *   ACK есть -> настоящая PS/2-клава: USB-контроллеры разрешены;
     *   ACK нет  -> клавиатура = BIOS-эмуляция внутренней USB: USB не
     *               трогаем (защита Клавы), мышь - пункт GRUB
     *               «USB mouse test» (cmdline usbmouse). */
    wait_in();
    outb(PS2_CMD, 0xAE);                  /* включить интерфейс клавиатуры */
    kbd_write(0xFF);                      /* сброс клавиатуре -> ждём ACK */
    int kr = kbd_resp(1200000);
    if (kr == 0xFE) { kbd_write(0xFF); kr = kbd_resp(1200000); }
    /* Считаем клавиатуру живой при ЛЮБОМ из ответов:
     *   0xFA - ACK на сброс (настоящая PS/2-клава);
     *   0xAA - BAT-самотест вместо ACK (часть адаптеров шлёт сразу его);
     *   0xFE - RESEND: KVM и USB->PS/2-адаптеры отвечают на 0xFF только
     *          бесконечными 0xFE - порт физически есть, клава за ним
     *          живая.  Раньше только 0xFA: KVM-машины считались «без
     *          PS/2-клавы» и теряли защиту от USB-handoff. */
    kbd_alive = (kr == 0xFA || kr == 0xAA || kr == 0xFE);
    if (kbd_alive) {
        (void)kbd_resp(1200000);          /* 0xAA самотест - результат не важен */
        kbd_write(0xF4);                  /* включить сканирование */
        (void)kbd_resp(400000);
        /* после сброса 0xFF ВСЕ LED погашены: numlock_on/capslock_on
         * стартуют false (см. ps2_poll_key) - гасим их и физически,
         * чтобы состояние драйвера и клавиатуры совпадали. */
        kbd_set_leds(false, false);
    }
    for (int i = 0; i < 32; i++) {
        if (inb(PS2_STATUS) & 1) (void)inb(PS2_DATA);
        else break;
    }
}

mouse_state_t *ps2_mouse(void) {
    return &g_mouse;
}

bool ps2_mouse_present(void) {
    return mouse_present;
}

/* Enter удерживается? Состояние ведёт ps2_poll_key (make/break). */
bool ps2_enter_down(void) {
    return enter_held;
}

/* USB-клавиатура отключилась (mouse_hw.c): её scанкоды больше не
 * придут, включая break Enter - иначе зажатый Enter навсегда держит
 * «ЛКМ» (ps2_enter_down).  Сбрасываем всё, что могло остаться от неё. */
void ps2_kbd_detached(void) {
    enter_held = false;
    /* «ЭТАП 5»: клавиша ушла вместе с клавиатурой - софт-автоповтор
     * обязан остановиться, иначе «призрак» печатал бы вечно. */
    tm_key = 0;
    tm_logged = false;
}
/* клавиатура отвечала на сброс (ACK 0xFA)? По этому флагу
 * kernel.c и mouse_hw.c решают, можно ли трогать USB-контроллеры: без
 * PS/2-клавиатуры Клава работает через BIOS-эмуляцию, которую handoff
 * убивает (кроме режима «usbmouse» - там клавиатуру берёт mouse_hw). */
bool ps2_kbd_alive(void) {
    return kbd_alive;
}

/* Применить один пакет PS/2 мыши. Невалидные байты (бит 3 = 0,
   переполнение) молча отбрасываются. */
static void ps2_apply(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t flags = b0;
    int dx = (int)(int8_t)b1;
    int dy = (int)(int8_t)b2;

    if (!(flags & 0x08)) {
        return;
    }

    /* Переполнение (биты 6-7): счётчики dx/dy недостоверны - движение
     * отбрасываем, а КНОПКИ применяем (клик в момент рывка не теряется).
     * Раньше пакет выбрасывался целиком вместе с кнопками. */
    if (flags & 0xC0) {
        dx = 0;
        dy = 0;
    }

    if (dx > 20) dx = 20;
    if (dx < -20) dx = -20;
    if (dy > 20) dy = 20;
    if (dy < -20) dy = -20;

    int nx = g_mouse.x + dx;
    int ny = g_mouse.y - dy;
    if (nx < 0) nx = 0;
    if (nx >= g_fb_w) nx = g_fb_w - 1;
    if (ny < 0) ny = 0;
    if (ny >= g_fb_h) ny = g_fb_h - 1;
    if (nx != g_mouse.x || ny != g_mouse.y) {
        g_mouse.moved = true;
    }
    g_mouse.x = nx;
    g_mouse.y = ny;

    bool nl = (flags & 1) != 0;
    bool nr = (flags & 2) != 0;

    if (nl && !prev_left) {
        g_mouse.clicked_left = true;
    }
    if (nr && !prev_right) {
        g_mouse.clicked_right = true;
    }
    prev_left = nl;
    prev_right = nr;
    g_mouse.left = nl;
    g_mouse.right = nr;
}

/* [#435] дренаж aux-байтов PS/2-мыши.  При активной USB-мыши ранний
 * return в ps2_poll оставлял OBF полным: PS/2-мышь, включённая отчётами
 * (0xF4) в init, продолжала слать байты, и ps2_key_pending() не видел
 * клавиатурных — PS/2-КЛАВИАТУРА глохла, пока жива USB-мышь. */
static void ps2_drain_aux(void) {
    int safety = 64;
    while (safety-- > 0) {
        uint8_t st = inb(PS2_STATUS);
        if (!(st & 1u) || !(st & 0x20u)) break;   /* нет данных / не aux */
        (void)inb(PS2_DATA);                      /* байт мыши — в никуда */
    }
}

void ps2_poll(mouse_state_t *out) {
    /* «ЭТАП 5» (G3): тик софт-typematic. ps2_poll зовётся каждым
     * потребителем ввода РОВНО один раз за итерацию цикла (см.
     * комментарий у блока typematic) - это и есть «кадр» (~15 мс в
     * desktop-цикле при удержании клавиши). Выше ставить нельзя:
     * до USB-опросов счётчик двигать бессмысленно, ниже - ветки мыши
     * возвращаются раньше. */
    if (tm_key != 0) tm_frames++;

    /* ── v0.3.1: USB-клавиатуры опрашиваются КАЖДЫЙ кадр ──────────────
     * ps2_key_pending() видит только очередь kbdq и PS/2-порт, а очередь
     * USB-клавиатуры наполняется САМ ОПРОСОМ (uhci_kbd_poll): пока
     * клавиш нет, desktop не звал ps2_poll_key() — и USB-клавиатура
     * «замерзала» на рабочем столе.  Поэтому опрос всех USB-клавиатур
     * стоит ЗДЕСЬ, до веток мыши (те возвращаются раньше — при активной
     * мыши нижний код недостижим).  Повторный вызов из ps2_poll_key()
     * безвреден [#281 - ПРОВЕРЕНО «ЭТАП 5»]: mouse_hw_kbd_poll дренерует
     * event ring ДЕСТРУКТИВНО (evt_get двигает ERDP - второй вызов за
     * кадр не видит уже потреблённых событий), OHCI-путь ограждён
     * HcFmNumber (ohci_poll_reports_inst: один раз на кадр шины) и
     * done-список пере-постановкой TD; usb_kbd_poll - переармленный/
     * неактивный TD сразу выходит (этапы 1-4). Дублей сканкодов НЕТ. */
    usb_kbd_poll();          /* UHCI (usb_uhci_kbd.c) + EHCI (ehci.c) */
    mouse_hw_kbd_poll();     /* OHCI/xHCI-клавиатуры (mouse_hw.c) */
    usbdev_tick();           /* 0.3.0: раз в секунду — опрос ВСЕХ портов
                              * ВСЕХ контроллеров, независимо ни от чего */

    /* ── USB optical mouse takeover ──
     * Когда на USB-шине жива HID мышь (boot protocol), она полностью
     * заменяет PS/2 мышь: desktop.c и installer.c опрашивают ввод через
     * ps2_poll, поэтому вызывающему коду не важно, откуда пришли события.
     * Если USB мышь исчезает (watchdog в usb_mouse.c), управление
     * автоматически возвращается к PS/2. */
    if (usb_mouse_active()) {
        usb_mouse_poll(out);
        ps2_drain_aux();   /* [#435] PS/2-мышь не глушит PS/2-клавиатуру */
        return;
    }

    /* ── MOUSEHW (дополнение): мышь на реальном железе (xHCI) ──
     * Сюда доходим только когда UHCI-драйвер неактивен: в QEMU с
     * работающей usb-mouse ветка выше уже вернула управление, поведение
     * не меняется.  На xHCI-машине здесь отдаётся указатель.
     */
    if (mouse_hw_active()) {
        mouse_hw_poll(out);
        ps2_drain_aux();   /* [#435] */
        return;
    }

    /* EHCI (USB 2.0): mouse on the high-speed bus. Branch AFTER
     * UHCI/xHCI/OHCI: if a mouse is already bound on another bus,
     * its driver keeps serving it; EHCI joins only when empty. */
    if (ehci_mouse_active()) {
        ehci_mouse_poll(out);
        ps2_drain_aux();   /* [#435] */
        return;
    }

    /* 0.3.2: USB-мыши нет — периодически перепроверяем СВОБОДНЫЕ
     * UHCI-контроллеры: мышь могли воткнуть позже или в порт 2.0,
     * отданный EHCI компаньону уже после init-скана. */
    usb_mouse_rescan();

    g_mouse.clicked_left = false;
    g_mouse.clicked_right = false;
    g_mouse.moved = false;

    int got = 0;
    int safety = 64;
    while (safety-- > 0) {
        uint8_t st = inb(PS2_STATUS);
        if (!(st & 1)) break;
        if (!(st & 0x20)) break;

        uint8_t b = inb(PS2_DATA);
        got = 1;
        if (mouse_phase == 0 && !(b & 0x08)) {
            continue;
        }
        mouse_packet[mouse_phase++] = b;
        if (mouse_phase >= 3) {
            mouse_phase = 0;
            ps2_apply(mouse_packet[0], mouse_packet[1], mouse_packet[2]);
        }
    }

    /* Таймаут недобранного пакета: если пакет не собрался за 3 байта
     * (мышь выдернута посреди пакета, помехи) - сброс фазы.  Без этого
     * фаза застревает на 1-2 и курсор замирает до переподключения. */
    if (mouse_phase > 0 && !got) {
        if (++mouse_phase_stall > 8) {
            mouse_phase = 0;
            mouse_phase_stall = 0;
        }
    } else if (got) {
        mouse_phase_stall = 0;
    }

    if (out) *out = g_mouse;
}

uint32_t ps2_poll_key(void) {
    static bool shift_held   = false;
    static bool extended_key = false;
    /* после сброса 0xFF у клавиатуры ВСЕ LED выключены: NumLock
     * АППАРАТНО off (numpad шлёт навигационные коды).  Раньше драйвер
     * считал true - первый тумблер вводил вечный рассинхрон
     * «цифры вместо стрелок».  (состояние — файловые переменные [#446]) */
    /* Pause/Break - E1-последовательность (E1 1D 45 E1 9D C5):
     * без пропуска 0x1D зажимал Ctrl, а 0x45 щёлкал NumLock при
     * каждом нажатии Pause. */
    static int  e1_skip      = 0;
    /* состояние Ctrl - нужно для глобального хоткея Ctrl+J.
     * Раньше Ctrl не отслеживался вовсе (make 0x1D/break 0x9D тихо
     * возвращали 0 - поведение клавиатуры не меняется). */
    static bool ctrl_held    = false;

    /* сначала очередь USB-клавиатуры (mouse_hw.c) - она неотличимо
     * встраивается в тот же поток сканкодов набора 1.
 * Shift/Ctrl у USB-клавиатуры берём из свежего отчёта
     * (ps2_kbd_mods), у PS/2 - из потока make/break порта 0x60.
     * Две клавиатуры ничего не знают друг о друге и не мешают. */
    /* poll USB keyboards of all buses (EHCI transport; OHCI/xHCI
     * are polled inside mouse_hw_poll): fresh reports land in the
     * queue below and are drained first - keys are never stale. */
    usb_kbd_poll();
    /* Клавиатуры OHCI/xHCI живут в mouse_hw_poll, но при активной
     * мыши ДРУГОГО драйвера (UHCI) тот не вызывается (ps2_poll
     * возвращается раньше, desktop - тоже) - клавиатура замерзала.
     * Опрашиваем клавиатурную часть из общего пути разбора клавиш. */
    mouse_hw_kbd_poll();

    int inj = kbdq_get();
    uint8_t sc;
    bool from_usb = (inj >= 0);
    if (from_usb) {
        sc = (uint8_t)inj;
    } else {
        uint8_t st = inb(PS2_STATUS);
        if ((st & 0x21u) == 0x01u) {
            sc = inb(PS2_DATA);       /* байт клавиатуры из PS/2-порта */
        } else if (ps2_typematic_due()) {
            /* «ЭТАП 5»: очередь USB пуста, порт 0x60 молчит (нет данных
             * ВООБЩЕ или ждёт только мышь) - синтезируем АВТОПОВТОР
             * зажатой USB-клавиши как свежий make: снапшот модификаторов
             * из tm_mods, E0-префикс - через extended_key (строка ниже
             * прочитает и погасит его). Ctrl+J НЕ повторяем: глобальный
             * тумблер режима курсора не должен «машать» 30 р/с (слот
             * повтора всё равно израсходован - цикл разбора не крутится
             * на месте). */
            tm_last_repeat = tm_frames;
            if (tm_key == 0x24 && (tm_mods & 0x11u)) return 0;   /* 'j'+Ctrl */
            sc = tm_key;
            from_usb = true;
            g_usb_mods = tm_mods;
            if (tm_ext) extended_key = true;
            if (!tm_logged) {
                tm_logged = true;
                serial_puts("KBD-TYPEMATIC: автоповтор клавиши\n");
            }
        } else {
            return 0;
        }
    }
    bool shift_now = from_usb ? (g_usb_mods & 0x22u) != 0 : shift_held;
    bool ctrl_now  = from_usb ? (g_usb_mods & 0x11u) != 0 : ctrl_held;

    if (sc == 0xE0) {
        extended_key = true;
        return 0;
    }

    bool ext = extended_key;
    extended_key = false;

    if (sc == 0xE1) { e1_skip = 5; return 0; }  /* Pause: пропускаем хвост */
    if (e1_skip > 0) { e1_skip--; return 0; }

    if (!ext && (sc == 0x2A || sc == 0x36)) { shift_held = true;  return 0; }
    if (!ext && (sc == 0xAA || sc == 0xB6)) { shift_held = false; return 0; }
    if (!ext && sc == 0x45) {               /* NumLock: тумблер + LED */
        numlock_on = !numlock_on;
        kbd_set_leds(numlock_on, capslock_on);
        return 0;
    }
    if (!ext && sc == 0x3A) {               /* CapsLock: тумблер + LED */
        capslock_on = !capslock_on;
        kbd_set_leds(numlock_on, capslock_on);
        return 0;
    }
    if (!ext && sc == 0x1D) { ctrl_held = true;  return 0; }  /* Ctrl down */
    if (!ext && sc == 0x9D) { ctrl_held = false; return 0; }  /* Ctrl up   */

    /* трекинг Enter для «зажатия» ЛКМ (ps2_enter_down). Break 0x9C
     * раньше просто отбрасывался проверкой (sc & 0x80) - теперь он
     * поднимает enter_held.  Make 0x1C ставит флаг и, как и прежде,
     * возвращается ниже из таблицы как '\n' (возвраты не менялись). */
    if (!ext && sc == 0x9C) { enter_held = false; return 0; } /* Enter up   */
    if (!ext && sc == 0x1C) { enter_held = true;  }           /* Enter down */

    /* 0xF0-костыль удалён: трансляция set2->set1 теперь включается
     * явно (бит 0x40 конфиг-байта в ps2_init) - break-префикс набора 2
     * из порта 0x60 прийти не может. */

    /* «ЭТАП 3»: break-коды стрелок больше НЕ выбрасываются — режим
     * «курсор с клавиатуры» (Ctrl+J) должен видеть ОТПУСКАНИЕ стрелки,
     * иначе непрерывное движение не остановить. Остальные break-коды
     * по-прежнему отбрасываются (0). Стрелки бывают E0-навигационные
     * (break = E0 0xC8/0xD0/0xCB/0xCD) и numpad-стрелки без NumLock
     * (break = 0xC8/0xD0/0xCB/0xCD без префикса). */
    if (sc & 0x80u) {
        uint8_t mk = (uint8_t)(sc & 0x7Fu);
        if (mk == 0x48u) return KEY_UP    | KEY_BREAK;
        if (mk == 0x50u) return KEY_DOWN  | KEY_BREAK;
        if (mk == 0x4Bu) return KEY_LEFT  | KEY_BREAK;
        if (mk == 0x4Du) return KEY_RIGHT | KEY_BREAK;
        return 0;
    }

    /* Расширенные клавиши (0xE0) */
    if (ext) {
        switch (sc) {
            case 0x48: return KEY_UP;
            case 0x50: return KEY_DOWN;
            case 0x4B: return shift_now ? KEY_SHIFT_LEFT  : KEY_LEFT;
            case 0x4D: return shift_now ? KEY_SHIFT_RIGHT : KEY_RIGHT;
            /* Навигационные клавиши */
            case 0x47: return KEY_HOME;
            case 0x4F: return KEY_END;
            case 0x49: return KEY_PGUP;
            case 0x51: return KEY_PGDN;
            case 0x53: return KEY_DEL;      /* Delete (E0 53) - раньше 0 */
            default:   return 0;
        }
    }

    /* Обычные клавиши (без 0xE0) */
    if (sc < 128 && scancode_ascii[sc]) {
        char c = scancode_ascii[sc];
        /* Ctrl+J перехватывается до shift-преобразований и до
         * доставки в приложения - это глобальный хоткей курса.
 * Ctrl проверяется по СВОЕМУ источнику байта, поэтому
         * фантомный Ctrl чужого потока букву не съедает. */
        if (ctrl_now && (c == 'j' || c == 'J')) return KEY_CTRL_J;
        /* регистр букв = Shift XOR CapsLock (CapsLock заработал) */
        if (c >= 'a' && c <= 'z' && (shift_now != capslock_on))
            return (uint32_t)(uint8_t)(c - 'a' + 'A');
        if (shift_now) {
            switch (c) {
                case '1': return '!'; case '2': return '@';
                case '3': return '#'; case '4': return '$';
                case '5': return '%'; case '6': return '^';
                case '7': return '&'; case '8': return '*';
                case '9': return '('; case '0': return ')';
                case '-': return '_'; case '=': return '+';
                case '[': return '{'; case ']': return '}';
                case '\\':return '|'; case ';': return ':';
                case '\'':return '"'; case ',': return '<';
                case '.': return '>'; case '/': return '?';
                case '`': return '~';
                default:  break;
            }
        }
        return (uint32_t)(uint8_t)c;
    }

    /* Клавиши NumPad (без E0) – возвращаем цифры или навигацию в зависимости от NumLock */
    switch (sc) {
        case 0x47: return numlock_on ? '7' : KEY_HOME;
        case 0x48: return numlock_on ? '8' : KEY_UP;
        case 0x49: return numlock_on ? '9' : KEY_PGUP;
        case 0x4A: return '-';
        case 0x4B: return numlock_on ? '4' : (shift_now ? KEY_SHIFT_LEFT : KEY_LEFT);
        case 0x4C: return numlock_on ? '5' : 0;
        case 0x4D: return numlock_on ? '6' : (shift_now ? KEY_SHIFT_RIGHT : KEY_RIGHT);
        case 0x4E: return '+';
        case 0x4F: return numlock_on ? '1' : KEY_END;
        case 0x50: return numlock_on ? '2' : KEY_DOWN;
        case 0x51: return numlock_on ? '3' : KEY_PGDN;
        case 0x52: return numlock_on ? '0' : 0;
        case 0x53: return numlock_on ? '.' : KEY_BS;
        default: break;
    }

    /* Функциональные клавиши F1–F12 */
    if (sc >= 0x3B && sc <= 0x44) return KEY_F1 + (sc - 0x3B);
    if (sc == 0x57) return KEY_F1 + 10;
    if (sc == 0x58) return KEY_F1 + 11;

    return 0;
}

void ps2_init_usb_hid(void) {
    /* Мост к настоящему драйверу USB HID мыши (usb_mouse.c).
       kernel.c вызывает usb_mouse_init() напрямую; здесь вызов оставлен
       для совместимости — usb_mouse_init() защищён от повторного запуска. */
    usb_mouse_init(g_fb_w, g_fb_h);
}

