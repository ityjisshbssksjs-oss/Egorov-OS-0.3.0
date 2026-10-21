/* usbdev.c — ДИСПЕТЧЕР УСТРОЙСТВ USB (шесть драйверов + реестр + 1 Гц опрос).
 * Egorov OS 0.3.0.  См. usbdev.h — там полная модель.
 *
 * Здесь: реестр занятости (КЛАВИАТУРА/МЫШЬ), подбор драйвера по версии
 * порта, «НЕИЗВЕСТНОЕ УСТРОЙСТВО», и секундный тик (RTC), который
 * НЕЗАВИСИМО ОТ ОБСТОЯТЕЛЬСТВ обходит ВСЕ порты ВСЕХ контроллеров
 * через hotplug-функции драйверов шин.
 *
 * Глубокий аудит #205: сверх секундного ПОЛНОГО обхода добавлен ЛЁГКИЙ
 * обход ~4 раза в секунду (каждые ~250 мс).  Полный (раз в секунду)
 * трогает всё, включая тяжёлые rescan-попытки EHCI (порт-ресеты
 * ненашых портов); лёгкий — только CHANGE/edge-фильтры дешёвых шин
 * (xHCI/OHCI/UHCI: чтение PORTSC и реакция на CSC — копейки), поэтому
 * реакция на hotplug ускоряется с ~1 с до ~250 мс без тяжеления цикла. */

#include "types.h"
#include "string.h"
#include "io.h"
#include "usbdev.h"
#include "logstick.h"   /* [LOGS] ЭТАП 11: ls_tick() — периодическая выгрузка */

extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);
extern char *itoa(uint32_t v, char *buf, int base);

/* [#451] горячее обновление USB-КАРТЫ: привязка/отключение ставят флаг,
 * секундный тик печатает свежую карту (реализация — mouse_hw.c). */
extern void mouse_hw_portmap_mark(void);
extern void mouse_hw_portmap_flush(void);

/* hotplug-обходы шин (реализованы в драйверах контроллеров) */
extern void mouse_hw_hotplug_sweep(void);   /* xHCI (все инстансы) + OHCI (все инстансы) */
extern void ehci_hotplug_sweep(void);       /* EHCI: все порты всех контроллеров */
extern void usb_mouse_rescan_now(void);     /* UHCI: мышь, порты 0/1 всех свободных контроллеров */
extern void usb_uhci_kbd_poll(void);        /* UHCI: клавиатура (свои кулдауны внутри) */

/* ── Реестр: ровно два слота — КЛАВИАТУРА и МЫШЬ ──────────────────────────
 * Занятый слот держит имя драйвера (роль+версия порта) и место, где
 * устройство живёт.  Драйвер «израсходован»: к ДРУГИМ устройствам он
 * больше не подбирается, пока устройство не выдернуто (release). */
typedef struct {
    int      used;
    int      ver;                /* 1/2/3 — версия порта привязки */
    char     bus[12];            /* «xHCI#1» / «OHCI#2» / «EHCI#1» / «UHCI#1» */
    int      port;
    unsigned vid, pid;
    uint32_t binds;              /* сколько привязок через этот слот (переподкл.) */
} usbdev_slot_t;

static usbdev_slot_t g_kbd_slot_reg;
static usbdev_slot_t g_ms_slot_reg;

static int  g_ehci_seen;         /* на плате есть EHCI → UHCI/OHCI-порты МОГУТ быть компаньонами [#463] */
static int  g_inited;
static int  g_sweeps;            /* счётчик секундных обходов */

/* [#462] Счётчики ЖИВЫХ HID-отчётов — «ЗАНЯТ» в карте должен
 * доказываться реальным потоком данных, а не только привязкой слота
 * (диагноз пользователя: «статус ЗАНЯТ не доказывает, что устройство
 * передаёт данные»). Инкремент — из ВСЕХ путей разбора отчётов:
 *   мышь:  xHCI event-ring (mouse_hw.c), OHCI done-queue (mouse_hw.c),
 *          EHCI eh_poll_dev (ehci.c), UHCI um_poll (usb_mouse.c);
 *   клава: oh_kbd_process (xHCI+OHCI) и usb_kbd_parse_boot (EHCI+UHCI).
 * Сброс в 0 — при новой привязке слота (try_bind) и при release: после
 * переподключения счёт снова честный «с нуля». */
static uint32_t g_rep_kbd;       /* HID-отчёты текущей клавиатуры   */
static uint32_t g_rep_ms;        /* HID-отчёты текущей мыши         */

static void ud_copy(char *dst, int cap, const char *src) {
    int i = 0;
    while (src[i] && i < cap - 1) { dst[i] = src[i]; i++; }
    dst[i] = 0;
}

static const char *ud_ver_str(int ver) {
    switch (ver) {
        case 3:  return "3.0";
        case 2:  return "2.0";
        default: return "1.x";
    }
}

static const char *ud_role_str(int role) {
    switch (role) {
        case USBDEV_ROLE_KBD:   return "клавиатуры";
        case USBDEV_ROLE_MOUSE: return "мыши";
        default:                return "?";
    }
}

/* полное имя драйвера: «драйвер клавиатуры 3.0» */
static void ud_drv_name(int role, int ver, char *out, int cap) {
    ud_copy(out, cap, "драйвер ");
    /* ограниченная склейка */
    const char *r = ud_role_str(role);
    int n = 0;
    while (out[n]) n++;
    int i = 0;
    while (r[i] && n < cap - 8) out[n++] = r[i++];
    out[n++] = ' ';
    const char *v = ud_ver_str(ver);
    i = 0;
    while (v[i] && n < cap - 1) out[n++] = v[i++];
    out[n] = 0;
}

/* ── Печать списка шести драйверов при загрузке ─────────────────────────── */
void usbdev_boot_print(void) {
    serial_puts("USBDEV: ДИСПЕТЧЕР УСТРОЙСТВ: всего ШЕСТЬ драйверов —\n");
    serial_puts("USBDEV:   клавиатура 3.0 | мышь 3.0   (порты xHCI)\n");
    serial_puts("USBDEV:   клавиатура 2.0 | мышь 2.0   (порты EHCI/компаньоны)\n");
    serial_puts("USBDEV:   клавиатура 1.x | мышь 1.x   (порты UHCI/OHCI)\n");
    serial_puts("USBDEV: реестр: каждый драйвер привязывается к ОДНОМУ устройству;\n");
    serial_puts("USBDEV: неподошедшие ни к одному драйверу = НЕИЗВЕСТНОЕ УСТРОЙСТВО.\n");
    serial_puts("USBDEV: опрос ВСЕХ портов КАЖДУЮ СЕКУНДУ (переподключения).\n");
}

/* ── «Порт, кто ты?» ─────────────────────────────────────────────────────── */
void usbdev_ask_port(const char *bus, int ver, int port) {
    char nb[8];
    serial_puts("USBDEV: ");
    serial_puts(bus ? bus : "?");
    serial_puts(" порт ");
    serial_puts(itoa((uint32_t)port, nb, 10));
    serial_puts(" спрашивает: кто ты? — «USB ");
    serial_puts(ud_ver_str(ver));
    serial_puts("»\n");
}

/* ── Ядро модели: подбор драйвера для нового устройства ──────────────────── */
int usbdev_try_bind(int ver, int kbd_ok, int mouse_ok,
                     const char *bus, int port,
                     unsigned vid, unsigned pid) {
    char drv[48];

    serial_puts("USBDEV: новое устройство vid=");
    serial_puthex(vid);
    serial_puts(" pid=");
    serial_puthex(pid);
    serial_puts(" (");
    serial_puts(bus ? bus : "?");
    serial_puts(" порт ");
    {
        char nb[8];
        serial_puts(itoa((uint32_t)port, nb, 10));
    }
    serial_puts("): клавиатура? ");
    serial_puts(kbd_ok ? "ДА" : "нет");
    serial_puts(", мышь? ");
    serial_puts(mouse_ok ? "ДА" : "нет");
    serial_puts("\n");

    /* 1) сначала — драйвер КЛАВИАТУРЫ этой версии порта */
    ud_drv_name(USBDEV_ROLE_KBD, ver, drv, (int)sizeof(drv));
    if (g_kbd_slot_reg.used) {
        serial_puts("USBDEV: «");
        serial_puts(drv);
        serial_puts("» уже занят (");
        serial_puts(g_kbd_slot_reg.bus);
        serial_puts(" порт ");
        {
            char nb[8];
            serial_puts(itoa((uint32_t)g_kbd_slot_reg.port, nb, 10));
        }
        serial_puts(") — больше не подбираю его ни к кому\n");
    } else if (kbd_ok) {
        serial_puts("USBDEV: пробую «");
        serial_puts(drv);
        serial_puts("»… ПОДОШЁЛ\n");
        g_kbd_slot_reg.used = 1;
        g_kbd_slot_reg.ver  = ver;
        ud_copy(g_kbd_slot_reg.bus, (int)sizeof(g_kbd_slot_reg.bus),
                bus ? bus : "?");
        g_kbd_slot_reg.port = port;
        g_kbd_slot_reg.vid  = vid;
        g_kbd_slot_reg.pid  = pid;
        g_kbd_slot_reg.binds++;
        serial_puts("USBDEV: реестр: КЛАВИАТУРА = ");
        serial_puts(g_kbd_slot_reg.bus);
        serial_puts(" порт ");
        {
            char nb[8];
            serial_puts(itoa((uint32_t)port, nb, 10));
        }
        serial_puts(" («");
        serial_puts(drv);
        serial_puts("»)\n");
        g_rep_kbd = 0;             /* [#462] новое устройство — счёт отчётов с нуля */
        mouse_hw_portmap_mark();   /* [#451] карта устарела: клава привязана */
        return USBDEV_ROLE_KBD;
    } else {
        serial_puts("USBDEV: пробую «");
        serial_puts(drv);
        serial_puts("»… устройство не клавиатура\n");
    }

    /* 2) затем — драйвер МЫШИ этой версии порта */
    ud_drv_name(USBDEV_ROLE_MOUSE, ver, drv, (int)sizeof(drv));
    if (g_ms_slot_reg.used) {
        serial_puts("USBDEV: «");
        serial_puts(drv);
        serial_puts("» уже занят (");
        serial_puts(g_ms_slot_reg.bus);
        serial_puts(" порт ");
        {
            char nb[8];
            serial_puts(itoa((uint32_t)g_ms_slot_reg.port, nb, 10));
        }
        serial_puts(") — больше не подбираю его ни к кому\n");
    } else if (mouse_ok) {
        serial_puts("USBDEV: пробую «");
        serial_puts(drv);
        serial_puts("»… ПОДОШЁЛ\n");
        g_ms_slot_reg.used = 1;
        g_ms_slot_reg.ver  = ver;
        ud_copy(g_ms_slot_reg.bus, (int)sizeof(g_ms_slot_reg.bus),
                bus ? bus : "?");
        g_ms_slot_reg.port = port;
        g_ms_slot_reg.vid  = vid;
        g_ms_slot_reg.pid  = pid;
        g_ms_slot_reg.binds++;
        serial_puts("USBDEV: реестр: МЫШЬ = ");
        serial_puts(g_ms_slot_reg.bus);
        serial_puts(" порт ");
        {
            char nb[8];
            serial_puts(itoa((uint32_t)port, nb, 10));
        }
        serial_puts(" («");
        serial_puts(drv);
        serial_puts("»)\n");
        g_rep_ms = 0;              /* [#462] новое устройство — счёт отчётов с нуля */
        mouse_hw_portmap_mark();   /* [#451] карта устарела: мышь привязана */
        return USBDEV_ROLE_MOUSE;
    } else {
        serial_puts("USBDEV: пробую «");
        serial_puts(drv);
        serial_puts("»… устройство не мышь\n");
    }

    /* 3) никакой из шести драйверов не подошёл */
    serial_puts("USBDEV: НЕИЗВЕСТНОЕ УСТРОЙСТВО (");
    serial_puts(bus ? bus : "?");
    serial_puts(" порт ");
    {
        char nb[8];
        serial_puts(itoa((uint32_t)port, nb, 10));
    }
    serial_puts("): ни один из 6 драйверов не подошёл\n");
    return USBDEV_ROLE_NONE;
}

int usbdev_kbd_bound(void)   { return g_kbd_slot_reg.used; }
int usbdev_mouse_bound(void) { return g_ms_slot_reg.used; }

/* ── Освобождение слота (detach / неудачная привязка) ───────────────────── */
void usbdev_release(int role, const char *why) {
    usbdev_slot_t *s = (role == USBDEV_ROLE_KBD) ? &g_kbd_slot_reg :
                       (role == USBDEV_ROLE_MOUSE) ? &g_ms_slot_reg : 0;
    if (!s || !s->used) return;
    serial_puts("USBDEV: устройство отключено (");
    serial_puts(s->bus);
    serial_puts(" порт ");
    {
        char nb[8];
        serial_puts(itoa((uint32_t)s->port, nb, 10));
    }
    serial_puts(") → «драйвер ");
    serial_puts(ud_role_str(role));
    serial_puts(" ");
    serial_puts(ud_ver_str(s->ver));
    serial_puts("» свободен (");
    serial_puts(why ? why : "detach");
    serial_puts(")\n");
    s->used = 0;
    s->bus[0] = 0;
    if (role == USBDEV_ROLE_KBD)        g_rep_kbd = 0;   /* [#462] */
    else if (role == USBDEV_ROLE_MOUSE) g_rep_ms  = 0;   /* [#462] */
    mouse_hw_portmap_mark();   /* [#451] карта устарела: устройство ушло */
}

/* ── Версия порта по типу контроллера ──────────────────────────────────────
 * [#461] ЧЕСТНОЕ поколение ШИНЫ — как Linux называет шину «usb1/usb2/usb3»:
 * UHCI/OHCI = 1.x ВСЕГДА (устройство на этой шине — USB 1.x-устройство,
 * даже если физическое гнездо «USB 2.0-компаньон»). Прежний глобальный
 * флаг g_ehci_seen врал, что КАЖДЫЙ UHCI/OHCI-порт — компаньон: наличие
 * EHCI не доказывает этого ни для одного конкретного порта. Способность
 * ГНЕЗДА (компаньон или нет) — вопрос КАРТЫ ГНЕЗД [#459], там печатается
 * «USB 2.0, если компаньон — соответствие без ACPI неизвестно»; драйвер
 * же честно называется по шине. Сверка: Linux drivers/usb/core — шина
 * OHCI всегда «usb1», независимо от наличия EHCI на плате. */
int usbdev_port_version(int prog_if) {
    switch (prog_if & 0xF0u) {
        case 0x30: return 3;                 /* xHCI: по протоколу порта (xh_port_ver) */
        case 0x20: return 2;                 /* EHCI: шина USB 2.0 */
        default:   return 1;                 /* UHCI/OHCI: шина USB 1.x — честно */
    }
}

/* [#463] На плате есть EHCI → UHCI/OHCI-порты МОГУТ оказаться гнёздами-
 * компаньонами (макс USB 2.0). Использует КАРТА ГНЕЗД для честной
 * формулировки «если компаньон — соответствие неизвестно». */
int usbdev_companion_possible(void) { return g_ehci_seen; }

void usbdev_set_ehci_seen(int seen) { g_ehci_seen = seen; }

/* ── [#462] Живость: счётчики HID-отчётов текущих устройств ────────────── */
void usbdev_note_report(int role) {
    if (role == USBDEV_ROLE_KBD) {
        if (g_rep_kbd == 0) mouse_hw_portmap_mark();  /* [#462] первый отчёт — перепечать карту с «ДАННЫЕ ИДУТ» */
        g_rep_kbd++;
    } else if (role == USBDEV_ROLE_MOUSE) {
        if (g_rep_ms == 0) mouse_hw_portmap_mark();
        g_rep_ms++;
    }
}

uint32_t usbdev_report_count(int role) {
    return (role == USBDEV_ROLE_KBD) ? g_rep_kbd : g_rep_ms;
}

int usbdev_bound(int role) {
    if (role == USBDEV_ROLE_KBD)   return g_kbd_slot_reg.used;
    if (role == USBDEV_ROLE_MOUSE) return g_ms_slot_reg.used;
    return 0;
}

/* ── Сводка реестра ─────────────────────────────────────────────────────── */
void usbdev_summary(char *out, int outsz) {
    if (!out || outsz < 2) return;
    out[0] = 0;
    int n = 0;
    const char *lbl[2] = { "клавиатура: ", "мышь: " };
    usbdev_slot_t *sl[2] = { &g_kbd_slot_reg, &g_ms_slot_reg };
    for (int i = 0; i < 2; i++) {
        if (!sl[i]->used) continue;
        if (n && n < outsz - 2) { out[n++] = ';'; out[n++] = ' '; }
        for (const char *p = lbl[i]; *p && n < outsz - 1; ) out[n++] = *p++;
        for (const char *p = sl[i]->bus; *p && n < outsz - 1; ) out[n++] = *p++;
        if (n < outsz - 10) {
            char nb[8];
            const char *a = " порт ";
            for (; *a && n < outsz - 1; ) out[n++] = *a++;
            itoa((uint32_t)sl[i]->port, nb, 10);
            for (const char *p = nb; *p && n < outsz - 1; ) out[n++] = *p++;
            out[n++] = ' ';
            out[n++] = '(';
            const char *v = ud_ver_str(sl[i]->ver);
            for (; *v && n < outsz - 1; ) out[n++] = *v++;
            out[n++] = ')';
        }
    }
    out[n] = 0;
    if (!n) ud_copy(out, outsz, "реестр USB пуст");
}

/* ── Секундный тик: RTC-секунды → полный обход всех портов ──────────────── */
static uint8_t cmos_read(uint8_t reg) {
    outb(0x70, reg);
    return inb(0x71);
}

static int rtc_seconds(void) {
    uint8_t stb = cmos_read(0x0B);
    uint8_t s   = cmos_read(0x00);
    if (!(stb & 0x04u))                      /* BCD -> двоичное */
        s = (uint8_t)((s & 0x0Fu) + ((s >> 4) * 10u));
    return (int)(s % 60u);
}

/* Полный обход: ВСЕ контроллеры, включая тяжёлый ehci_rescan_unclaimed
 * (повторные перечисления ненашых EHCI-портов с порт-ресетами). */
static void ud_sweep_full(void) {
    mouse_hw_hotplug_sweep();     /* xHCI (все инстансы) + OHCI (все инстансы) */
    ehci_hotplug_sweep();         /* EHCI: порты + rescan ненашых */
    usb_mouse_rescan_now();       /* UHCI: мышь на свободных контроллерах */
    usb_uhci_kbd_poll();          /* UHCI: клавиатура */
}

/* Лёгкий обход (#205): только дешёвые CHANGE/edge-фильтры.  Все три
 * вызова в покое стоят несколько чтений PORTSC; дорогое внутри них
 * случается ЛИШЬ при реальном событии порта — что и требуем.  EHCI
 * (его rescan_unclaimed молотит порт-ресетами) сюда сознательно НЕ
 * входит — он остаётся на секундном полном обходе. */
static void ud_sweep_light(void) {
    mouse_hw_hotplug_sweep();     /* xHCI+OHCI: CSC-фильтры внутри */
    usb_mouse_rescan_now();       /* UHCI: edge-детект свежих подключений */
    usb_uhci_kbd_poll();          /* UHCI-клава: yield-логика на 250 мс */
}

void usbdev_tick(void) {
    static int last_sec = -1;
    /* #205: счётчик вызовов — единственный суб-секундный механизм времени
     * (RTC даёт только целые секунды, IRQ/PIT-тик в системе нет).  Раз в
     * RTC-секунду калибруемся: сколько вызовов уложилось в секунду — тем
     * самым лёгкий обход режется на четверть секунды.  Честная оговорка:
     * интервал ПРИБЛИЗИТЕЛЬНЫЙ и зависит от частоты главного цикла
     * (ps2_poll зовётся каждый кадр/итерацию desktop-цикла). */
    static uint32_t ud_calls;        /* всего вызовов с старта              */
    static uint32_t ud_calls_at_sec; /* вызовов на последней смене секунды */
    static uint32_t ud_calls_per_sec;/* вызовов за прошлую секунду (калибр) */
    static uint32_t ud_last_light;   /* вызовов на последнем лёгком обходе */
    ud_calls++;

    if (!g_inited) {                          /* первый вызов: стартовая секунда */
        g_inited = 1;
        last_sec = rtc_seconds();
        ud_calls_at_sec = ud_calls;
        ud_last_light   = ud_calls;
        serial_puts("USBDEV: секундный опрос портов ЗАПУЩЕН (все контроллеры,\n");
        serial_puts("USBDEV: все порты, независимо от обстоятельств)\n");
        serial_puts("USBDEV: плюс лёгкий обход по CHANGE-битам ~4 раза в секунду\n");
        /* стартовый обход сразу: устройства, воткнутые до загрузки,
         * ловятся и без ожидания первой секунды */
        ud_sweep_full();
        return;
    }
    int s = rtc_seconds();
    if (s != last_sec) {                      /* секунда сменилась */
        ud_calls_per_sec = ud_calls - ud_calls_at_sec;
        ud_calls_at_sec  = ud_calls;
        last_sec = s;
        g_sweeps++;

        /* РАЗ В СЕКУНДУ: опросить ВСЕ порты ВСЕХ контроллеров.  Порядок
         * фиксирован; каждый обход неблокирующий (CSC-фильтры внутри). */
        ud_sweep_full();
        ud_last_light = ud_calls;   /* полный только что был — лёгкий не нужен */
        mouse_hw_portmap_flush();   /* [#451] были события hotplug — свежая карта */

        /* [LOGS] ЭТАП 11: периодическая выгрузка журнала в EGOROV.LOG
         * (активна только в режиме sticklog после успешной подготовки;
         * бюджет секторов ограничен — вызов не вешает систему). */
        ls_tick();

        /* сердцебиение в журнал раз в минуту — видно, что опрос жив */
        if ((g_sweeps % 60u) == 0u) {
            char nb[8];
            serial_puts("USBDEV: опрос портов: ");
            serial_puts(itoa((uint32_t)(g_sweeps / 60u), nb, 10));
            serial_puts(" мин (каждую секунду, все контроллеры)\n");
        }
        return;
    }

    /* #205: лёгкий обход ~каждые 250 мс — каждые max(1, calls_per_sec/4)
     * вызовов.  До первой калибровки (первая секунда) не тикаем: стартовый
     * полный обход только что прошёл, а частоту цикла ещё не знаем.
     * Фактическая частота 2.5-4 Гц (интервал 250-400 мс): целочисленное
     * деление плюс синхронизация с секундной сменой — модель проверена
     * симуляцией, реакция на hotplug улучшена с ~1 с до ~0.3 с. */
    if (ud_calls_per_sec == 0u) return;      /* калибровка не выполнена */
    uint32_t every = ud_calls_per_sec / 4u;   /* ~250 мс */
    if (every == 0u) every = 1u;              /* цикл медленнее 4 Гц — каждый вызов */
    if (ud_calls - ud_last_light < every) return;
    ud_last_light = ud_calls;
    ud_sweep_light();
}
