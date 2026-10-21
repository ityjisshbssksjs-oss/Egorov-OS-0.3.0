/* ehci.c — EHCI-драйвер (USB 2.0) для Egorov OS 0.3.0
 *
 * «ПОРТЫ 2.0, ЭТАП 2» — что убивало порты 2.0 на реальном железе
 * (AMD FCH: 2×xHCI + 2×EHCI + 2×OHCI-компаньона) и что исправлено здесь:
 *
 *   1) DATA-стадия контрольной передачи уходила с тогглом DATA0
 *      (спека USB: SETUP=DATA0 → DATA=DATA1 → STATUS=DATA1). QEMU не
 *      проверяет тогглы контрольных труб — все e2e проходили, а РЕАЛЬНАЯ
 *      клавиатура high-speed отклоняла DATA0 в GET_DESCRIPTOR и
 *      перечисление умирало на первом же запросе. Теперь DATA-qTD
 *      несёт QTD_DT (бит 31 токена) — как STATUS-qTD.
 *
 *   2) ДВА EHCI на одной машине делили ОДНИ расписания/буферы/g_op:
 *      инициализация второго контроллера затирала расписание РАБОТАЮЩЕГО
 *      первого (два HC ходили по одним qH/qTD), а опрос портов после
 *      переезда g_op видел только второй. Теперь всё состояние
 *      ПЕР-ИНСТАНСНОЕ (массивы _a[2], селектор g_ei) — как xHCI/OHCI
 *      в mouse_hw.c. Порты 2.0, висящие на ВТОРОМ EHCI, больше не
 *      «мертвая зона».
 *
 *   3) Порядок инициализации: раньше mouse_hw_init зондировал OHCI-
 *      компаньонов ДО ehci_init, а ehci_init ставил CONFIGFLAG=1 и
 *      ЗАБИРАЛ порты обратно у компаньонов — уже перечисленная мышь
 *      срывалась. Теперь ehci_init идёт ПЕРЕД mouse_hw_init_ohci():
 *      EHCI сам раздаёт LS/FS-порты компаньонам, и OHCI зондит их
 *      уже в финальном состоянии (см. kernel.c).
 *
 *   4) Устройство, чьё перечисление ОДИН раз не прошло, больше никогда
 *      не пробовалось (CCS без CSC не перечитывался). Теперь каждую
 *      секунду ehci_rescan_unclaimed() даёт порту до 3 попыток, потом
 *      ждёт переподключения — как OHCI.
 *
 *   5) Порт с PED=1 (BIOS успел перечислить устройство до нас) НЕ
 *      перечислялся «как есть» — устройство сидело на чужом адресе и
 *      молчало. Теперь любой НЕНАШ подключённый порт получает честный
 *      port reset перед перечислением.
 *
 *   6) Handoff: USBLEGSUP (биты 16/24) + глушение USBLEGCTLSTS (SMI-
 *      источники BIOS) — SM-обработчик больше не стреляет по нашему
 *      контроллеру; таймаут рукопожатия сокращен до ~1 с.
 *
 *   7) Опрос DMA-памяти (токены qTD) — только через volatile-чтения
 *      (eh_dma32): GCC -O2 выносил обычные чтения из цикла ожидания,
 *      и передача «зависала» при живом контроллере.
 *
 *   8) Перестройка периодической цепочки (hotplug) — с выключением
 *      PSE на время записи (USBCMD.PSE=0 → дождаться USBSTS.PSS=0):
 *      контроллер не читает qH, пока мы их переписываем.
 *
 * Архитектура прежняя (0.3.0):
 *   • PCI-скан prog-if 0x20 (до ДВУХ контроллеров), MMIO;
 *   • halt → HCRESET → async-кольцо + периодическая таблица 1024;
 *   • CONFIGFLAG=1 ставится только после запуска расписаний; откат
 *     (устройств нет, PS/2-клавиатуры нет) возвращает порты
 *     компаньонам — HS-клавиатура за компаньоном поднимется как
 *     full-speed через OHCI;
 *   • HS-устройства (PED после сброса): перечисление напрямую, HID
 *     мышь + клавиатура через «диспетчер usbdev» (6 драйверов);
 *   • FS/LS-устройства: порт отдаётся компаньону (Port Owner);
 *     без компаньона — прямая попытка (QEMU);
 *   • поллинг без прерываний, опрос каждую секунду из usbdev_tick().
 *
 * Раскладки битов сверены со спецификацией EHCI 1.0 (qTD/qH/PORTSC,
 * USBCMD/USBSTS/CONFIGFLAG) и с Linux ehci-hcd.
 *
 * «ЭТАП F2» (внешний аудит) — что исправлено здесь, поверх этапов 2-3:
 *   • откат CONFIGFLAG=0 + kdelay_ms(100) на физический возврат портов
 *     компаньонам (VIA держит ~50 мс);
 *   • STALL: CLEAR_FEATURE(ENDPOINT_HALT) + сброс тоггла (контрольная
 *     труба — с повтором запроса, interrupt-EP — с DT=DATA0);
 *   • переарм int-TD БЕЗ полного eh_sched_rebuild (мышь ~60 опросов/с,
 *     rebuild = 1024 записи + wbinvd + стоп PSE);
 *   • PP 100 мс/порт, reset-таймаут 500 мс, буфер дескрипторов 512 Б с
 *     клампом, prev-кнопки мыши ПЕР-УСТРОЙСТВО, EH_NINST=4 (ASMedia),
 *     eh_delay → kdelay_us (PIT-калибровка, timer.h).
 *
 * «ЭТАП G2» (глубокий фикс, второй аудит 186–330) — что исправлено здесь:
 *   • doorbell() возвращает bool: при таймауте IAA — рестарт async-
 *     механизма (ASE=0 → ASS=0 → каноническое кольцо → ASE=1), а не
 *     молчаливая модификация кольца «как будто прозвенело» (#227);
 *   • адреса устройств — БИТМАПА битов 1..127 с возвратом при detach
 *     (#193/#258): инкрементный счётчик «тёк» на hotplug-циклах;
 *   • бит STS_PSS исправлен на 14 (было 15 = ASS): eh_sched_rebuild
 *     теперь честно ждёт останова ИМЕННО периодического расписания;
 *   • OCC-порты (#270), липкий PED после disconnect (#271), пропущенный
 *     CSC при быстром выдер/втыке — кэш conn-состояния (#264/#265),
 *     HS EP0 MPS=64 (#12e), валидация bDescriptorType и кламп конфига
 *     по реально прочитанной длине — короткому пакету (#12d).
 */

#include "ehci.h"
#include "usb_kbd.h"
#include "usbmap.h"
#include "usbdev.h"
#include "mouse_hw.h"  /* [#459] portmap_pm_n/portmap_pm_alive для КАРТЫ ГНЕЗД */
#include "io.h"
#include "string.h"
#include "timer.h"    /* F2-15: kdelay_us/kdelay_ms (PIT-калибровка) */

extern void serial_puthex(uint32_t v);

/* ── PCI config-space access ── */
#define PCI_ADDR 0xCF8u
#define PCI_DATA 0xCFCu

static uint32_t pci_read32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off) {
    uint32_t addr = (1u << 31) | ((uint32_t)bus << 16) |
                    ((uint32_t)dev << 11) | ((uint32_t)fn << 8) | (off & 0xFC);
    outl(PCI_ADDR, addr);
    return inl(PCI_DATA);
}
static void pci_write32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off, uint32_t v) {
    uint32_t addr = (1u << 31) | ((uint32_t)bus << 16) |
                    ((uint32_t)dev << 11) | ((uint32_t)fn << 8) | (off & 0xFC);
    outl(PCI_ADDR, addr);
    outl(PCI_DATA, v);
}
static uint8_t pci_read8(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off) {
    uint32_t d = pci_read32(bus, dev, fn, off & 0xFC);
    return (uint8_t)(d >> ((off & 3) * 8));
}

/* ── MMIO helpers ── */
static void eh_w32(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static uint32_t eh_r32(uint32_t a)         { return *(volatile uint32_t *)a; }
static void eh_dma(void) { __asm__ volatile("wbinvd" ::: "memory"); }
/* F2-15: тело — калиброванная kdelay_us (PIT): старый цикл inb(0x80) жил
 * ~20 нс на QEMU и ~1 мкс на реальном железе — USB-тайминги (дебаунс,
 * recovery, reset) плыли на порядок. timer_init() вызывается в kernel.c
 * ДО ehci_init() (порядок инициализации — агент F4). */
static void eh_delay(uint32_t us) { kdelay_us(us); }
/* чтение памяти, которую правит HC через DMA (токены qTD): обычное
 * чтение GCC -O2 выносит из цикла ожидания — как oh_dma32 в OHCI */
static uint32_t eh_dma32(const uint32_t *p) {
    return *(const volatile uint32_t *)p;
}

/* ── EHCI register offsets (от операционных регистров) ── */
#define R_USBCMD     0x00u
#define R_USBSTS     0x04u
#define R_USBINTR    0x08u
#define R_FRINDEX    0x0Cu
#define R_CTRLDSSEG  0x10u
#define R_PERIODIC   0x14u
#define R_ASYNC      0x18u
#define R_CONFIGFLAG 0x40u
#define R_PORTSC     0x44u

#define CMD_RS    (1u << 0)
#define CMD_HCR   (1u << 1)
#define CMD_PSE   (1u << 4)      /* periodic schedule enable (USBCMD.4) */
#define CMD_ASE   (1u << 5)      /* async schedule enable    (USBCMD.5) */
#define CMD_IAAD  (1u << 6)      /* async advance doorbell             */

#define STS_HALT  (1u << 12)
#define STS_IAA   (1u << 5)
/* G2 (собственная находка): биты расписаний в USBSTS по спеке EHCI 1.0
 * (Table 2-10; ср. Linux STS_PSS=1<<14 / STS_ASS=1<<15): периодика —
 * бит 14, async — бит 15. Прежний STS_PSS=1<<15 был на самом деле ASS:
 * eh_sched_rebuild «ждал останова периодики» по async-биту — на живом
 * HC ASS=1 всегда, ожидание выжигало ~20 мс таймаутом, и eh_fl
 * переписывался при РАБОТАЮЩЕМ периодическом расписании. */
#define STS_PSS   (1u << 14)     /* periodic schedule status           */
#define STS_ASS   (1u << 15)     /* async schedule status              */

/* PORTSC.  Поля СКОРОСТИ у EHCI в PORTSC НЕТ (это не xHCI): включённый
 * порт (PED=1) = high-speed; иначе скорость видна по линии (биты 10-11). */
#define PS_CCS    (1u << 0)
#define PS_CSC    (1u << 1)
#define PS_PED    (1u << 2)
#define PS_PEC    (1u << 3)
#define PS_OCC    (1u << 5)
#define PS_PR     (1u << 8)
#define PS_LINE   (0x3u << 10)   /* 01=K(LS), 10=J(FS) — до сброса          */
#define PS_PP     (1u << 12)
#define PS_PO     (1u << 13)     /* port owner: 1 = компаньон               */

/* qTD */
#define QTD_ACTIVE (1u << 7)
#define QTD_HALT   (1u << 6)
#define QTD_END    1u            /* terminator of qTD/qH link pointers */
#define QTD_ERRM   ((1u << 6) | (1u << 5) | (1u << 4) | (1u << 3))
/* F2-3: STALL = HALT БЕЗ ошибок шины (DBE/Babble/XactErr): устройство
 * отклонило запрос; лечится CLEAR_FEATURE(ENDPOINT_HALT). */
#define QTD_ERRBITS ((1u << 5) | (1u << 4) | (1u << 3))
#define QTD_PID_OUT   0u
#define QTD_PID_IN    (1u << 8)
#define QTD_PID_SETUP (2u << 8)
/* qTD token: PID 9:8, Cerr 11:10, C_Page 15:12, Tbytes 31:16, DT 31 */
#define QTD_CERR3     (3u << 10)
/* DT (Data Toggle) - бит 31 токена qTD (EHCI 1.0, Table 3-13).
 * ФИКС «этапа 2»: DATA-стадия контрольной передачи обязана идти с
 * DATA1 (SETUP=DATA0 → DATA=DATA1 → STATUS=DATA1). Без QTD_DT реальное
 * устройство отклоняло чтение дескрипторов, а QEMU тогглы контрольных
 * труб не проверял — дефект жил незаметно со времён 0.3.2. */
#define QTD_DT       (1u << 31)     /* DATA1                              */
#define QTD_LEN(n)    (((uint32_t)(n) & 0x7FFFu) << 16)
#define QTD_REM(tok)  (((tok) >> 16) & 0x7FFFu)
#define QH_DTC       (1u << 14)     /* бит DTC в dword1 qH             */
#define QH_H         (1u << 15)     /* голова async-кольца (reclaim)   */

/* qH dword1 (EP characteristics) */
#define QH_EPCHAR(addr, ep, spd, mps)                        \
    (((uint32_t)(addr) & 0x7Fu) |                            \
     (((uint32_t)(ep) & 0xFu) << 8) |                        \
     (((uint32_t)(spd) & 0x3u) << 12) | (1u << 14) |          /* DTC=1 */ \
     (1u << 15) |                                /* H - head reclaim */ \
     (((uint32_t)(mps) & 0x7FFu) << 16))

#define QH_HL_TERM  1u
#define QH_HL_QH(x) ((uint32_t)(x) | 2u)
#define QH_HL_TD(x) ((uint32_t)(x))

/* qH dword2 (EP capabilities): S-mask 7:0, C-mask 15:8 */
#define QH_CAP_HS      0x01u              /* uframe 0, раз в 1 мс        */
#define QH_CAP_FSLS    (0x01u | (0x1Cu << 8))  /* split: start 0, cpl 2-4 */

/* ── Структуры в ОЗУ ── */
typedef struct { uint32_t next, alt, token, buf[5]; } eh_qtd_t;

/* устройства (мышь + клавиатура + запас) */
#define EH_NDEV 4
typedef struct {
    bool     used, alive;
    uint8_t  addr, port, spd;   /* spd: 0=FS 1=LS 2=HS */
    uint8_t  role;              /* 1=мышь 2=клава      */
    uint8_t  ep, mps;
    uint16_t vid, pid;
    int      iface;
    bool     first_pkt;         /* первый отчёт - в журнал   */
    uint8_t  mps0;             /* F2-3: bMaxPacketSize0 (clear-halt по ep0) */
    bool     armed;            /* F2-5: qH уже в расписании (переарм без rebuild) */
    bool     prev_l, prev_r;   /* F2-13: prev-кнопки ПЕР-УСТРОЙСТВО, не глобально:
                                 * две мыши/инстанса не порождают фантомные клики */
} eh_dev_t;

/* ═══ ПЕР-ИНСТАНСНОЕ состояние (ФИКС «этапа 2», п.2) ══════════════════
 * AMD FCH несёт ДВА EHCI (0:12.2 и 0:13.2) — порты 2.0 разъезжаются по
 * обоим. До сих пор единственные eh_fl/eh_head/g_op означали: второй
 * контроллер затирает расписание первого и «уводит» опрос портов. */
/* F2-14: до ЧЕТЫРЁХ EHCI (платы с ASMedia несут 3-4). Все пер-инстансные
 * массивы неинициализированы → лежат в .bss: размер плоского бинарника
 * (лимит HDD-загрузчика 256 КБ) НЕ растёт, расход ~5 КБ ОЗУ на инстанс. */
#define EH_NINST 4
static int g_ei;                                   /* текущий инстанс     */

static uint32_t eh_fl_a[EH_NINST][1024] __attribute__((aligned(4096)));
/* ФИКС «этапа 2»: строка 16 dword (64 Б), а НЕ 12 (48): указатели EHCI
 * несут только биты 31:5 — КАЖДЫЙ qH обязан лежать на границе 32 байта.
 * При шаге 48 второй инстанс попадал на 16-байтовую границу (0x...9f0),
 * контроллер обрезал ссылку 0x...9f2 до 0x...9e0 и читал «qH» из хвоста
 * ПЕРВОГО инстанса — порт 2.0 на ВТОРОМ EHCI не работал ВООБЩЕ (QEMU
 * usb_ehci_qh_ptrs: "QH @ 0x...9e0: next 0" — мусор). */
static uint32_t eh_head_a[EH_NINST][16]  __attribute__((aligned(32)));
static uint32_t eh_qhc_a[EH_NINST][16]   __attribute__((aligned(32)));
/* F2-11: 6 qTD — с запасом: контрольная передача несёт максимум 3
 * (SETUP+DATA+STATUS), DATA-стадия — ОДИН qTD до 20 КБ (5 страниц). */
static eh_qtd_t eh_ctd_a[EH_NINST][6]    __attribute__((aligned(32)));
static uint8_t  eh_setup_a[EH_NINST][4][8] __attribute__((aligned(4)));
/* F2-10: 512 Б (было 280): composite-конфигурации длиннее 255 (клава+
 * тачпад+consumer); кратно 32, при пересечении границы 4К qtd_bufs сам
 * раскладывает страницы. Границу контролирует кламп в eh_ctrl(). */
static uint8_t  eh_data_a[EH_NINST][512] __attribute__((aligned(32)));
static eh_dev_t g_dv_a[EH_NINST][EH_NDEV];
/* КРИТИЧНО: строка 16 dword (64 байта) — указатели EHCI несут только
 * биты 31:5, каждый qH обязан лежать на границе 32 байта. */
static uint32_t  g_qhi_a[EH_NINST][EH_NDEV][16] __attribute__((aligned(32)));
static eh_qtd_t  g_qtdi_a[EH_NINST][EH_NDEV]    __attribute__((aligned(32)));
static uint8_t   g_pkt_a[EH_NINST][EH_NDEV][64] __attribute__((aligned(8)));   /* [#450] HS-мыши дают MPS 32/64: буфер 16 байт = переполнение DMA */
static uint8_t   g_fail_a[EH_NINST][16];   /* неудачи перечисления порта */
/* G2 #193/#258: адреса устройств — БИТМАПА (биты 1..127), не счётчик:
 * g_next_addr только рос и не возвращал адреса при detach — hotplug
 * «тёк» адресами (адрес USB 7-битный, максимум 127). g_next_addr
 * остаётся КУРСОРОМ поиска первого свободного бита. */
static uint8_t   eh_addr_bm_a[EH_NINST][16];
/* G2 #264/#265: кэш conn-состояния порта — ловит disconnect, чей CSC
 * погас между опросами (поллинг раз в ~1 с пропускает быстрый выдер/втык). */
static bool      g_conn_a[EH_NINST][16];
/* G2 #12(d): фактическая длина последней IN-передачи (short packet) */
static uint16_t  g_rxlen_a[EH_NINST];

static uint32_t g_mmio_a[EH_NINST], g_op_a[EH_NINST];
static uint8_t  g_nports_a[EH_NINST];
static bool     g_running_a[EH_NINST];
static uint8_t  g_next_addr_a[EH_NINST];
static char     g_diag_a[EH_NINST][96];
static uint8_t  g_bd_a[EH_NINST][3];      /* bus/dev/fn наших контроллеров */

#define eh_fl      (eh_fl_a[g_ei])
#define eh_head    (eh_head_a[g_ei])
#define eh_qhc     (eh_qhc_a[g_ei])
#define eh_ctd     (eh_ctd_a[g_ei])
#define eh_setup   (eh_setup_a[g_ei])
#define eh_data    (eh_data_a[g_ei])
#define g_dv       (g_dv_a[g_ei])
#define g_qhi      (g_qhi_a[g_ei])
#define g_qtdi     (g_qtdi_a[g_ei])
#define g_pkt      (g_pkt_a[g_ei])
#define g_fail     (g_fail_a[g_ei])
#define g_mm       (g_mmio_a[g_ei])
#define g_op       (g_op_a[g_ei])
#define g_nports   (g_nports_a[g_ei])
#define g_running  (g_running_a[g_ei])
#define g_next_addr (g_next_addr_a[g_ei])
#define g_diag     (g_diag_a[g_ei])
#define eh_addr_bm (eh_addr_bm_a[g_ei])
#define g_conn     (g_conn_a[g_ei])
#define g_rxlen    (g_rxlen_a[g_ei])

/* ── Общее состояние драйвера ── */
static int      g_ours_n;                  /* сколько инстансов поднято   */
static int      g_fb_w, g_fb_h;
static bool     g_have_companion;          /* найден OHCI/UHCI-компаньон */
static uint8_t  g_ncc_a[EH_NINST];         /* [#432] СВОЙ N_CC (HCSPARAMS [15:12]) — глобальный скан не привязан к инстансу */
#define g_ncc (g_ncc_a[g_ei])
static uint32_t g_ehci_pkts;               /* счётчик отчётов            */
static mouse_state_t g_ms;    /* F2-13: prev-кнопки переехали в eh_dev_t */

static void eh_diag_set(const char *s) {
    uint32_t i = 0;
    while (s[i] && i < sizeof(g_diag) - 1u) { g_diag[i] = s[i]; i++; }
    g_diag[i] = 0;
}
static void eh_diag_set2(const char *a, const char *b) {
    eh_diag_set(a);
    uint32_t i = 0;
    while (g_diag[i]) i++;
    for (uint32_t j = 0; b[j] && i < sizeof(g_diag) - 1u; j++)
        g_diag[i++] = b[j];
    g_diag[i] = 0;
}

static const char *eh_bus_label(void) {
    /* F2-14/F2-17: метки на ВСЕ инстансы (было 2 + маска &1 — при
     * NINST>2 инстансы 3-4 получали бы чужие метки в журнал). */
    static const char *lbl[EH_NINST] =
        { "EHCI#1", "EHCI#2", "EHCI#3", "EHCI#4" };
    return lbl[g_ei & 3];
}

/* ── G2 #193/#258: адреса устройств — битмапа битов 1..127 ────────────
 * alloc — первый свободный бит, поиск с курсора g_next_addr (только что
 * освобождённый адрес не переиспользуется немедленно — на шине мог
 * остаться «неопознанный» житель со старым адресом); free — во ВСЕХ
 * путях detach (eh_kill_dev + провалы eh_enum_dev через out_free). */
static uint8_t eh_addr_alloc(void) {
    for (int t = 0; t < 127; t++) {
        uint8_t cand = (uint8_t)(1 + ((int)g_next_addr - 1 + t) % 127);
        if (!(eh_addr_bm[cand >> 3] & (1u << (cand & 7)))) {
            eh_addr_bm[cand >> 3] |= (uint8_t)(1u << (cand & 7));
            g_next_addr = (uint8_t)(cand >= 127 ? 1 : cand + 1);
            return cand;
        }
    }
    return 0;                        /* все 127 адресов заняты          */
}
static void eh_addr_free(uint8_t a) {
    if (a == 0 || a > 127) return;
    eh_addr_bm[a >> 3] &= (uint8_t)~(1u << (a & 7));
}

/* ── qTD: заполнить указатели страниц (до 5 страниц = 20 КБ) ── */
static void qtd_bufs(eh_qtd_t *t, uint32_t phys) {
    t->buf[0] = phys;
    for (int i = 1; i < 5; i++)
        t->buf[i] = (phys + 4096u * (uint32_t)i) & 0xFFFFF000u;
}

/* ── Передать управление qTD-цепочке через async-расписание ──
 * Один контрольный трансфер за раз: qh_head -> qh_ctrl -> head.
 * G2 #227 (КРИТИЧЕСКИЙ): true = дверь «прозвенела» (IAA пришёл).
 * При ТАЙМАУТЕ старый код продолжал модифицировать async-кольцо как
 * ни в чём не бывало — гонка с живым HC на реальном железе. Теперь:
 * ASE=0 → дождаться ASS=0 → вернуть кольцо в каноническое состояние
 * (голова сама на себя) → ASE=1 — и только потом вернуть false
 * («передача не стартовала»). HCRESET сознательно не делаем — слишком
 * грубо: снёс бы оба расписания и адреса всех устройств инстанса. */
/* ── Поднять async-расписание и ДОЖДАТЬСЯ ASS=1 ──────────────────────
 * [#404, фото «doorbell timeout» ×3] IAAD-звонок имеет смысл ТОЛЬКО
 * при работающем кольце: на AMD FCH звонок в остановленный/не
 * поднятый async не даёт IAA НИКОГДА — все три попытки GET_DESCRIPTOR
 * умирали на doorbell. Перед звонком/вставкой поднимаем ASE и ждём
 * подтверждения STS_ASS (спека 4.8: расписание активно только при
 * ASS=1). Возврат false = кольцо мертво, передачу стартовать нельзя. */
static bool eh_ensure_async(void) {
    if (eh_r32(g_op + R_USBSTS) & STS_ASS) return true;
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) | CMD_ASE);
    int t = 20000;                          /* ~200 мс на подъём     */
    while (!(eh_r32(g_op + R_USBSTS) & STS_ASS) && t-- > 0) eh_delay(10);
    if (eh_r32(g_op + R_USBSTS) & STS_ASS) return true;
    serial_puts("EHCI: async-расписание не поднимается (ASS=0), inst=");
    { char nb[8]; serial_puts(itoa((uint32_t)(g_ei + 1), nb, 10)); }
    serial_puts(" cmd=");
    serial_puthex(eh_r32(g_op + R_USBCMD));
    serial_puts(" sts=");
    serial_puthex(eh_r32(g_op + R_USBSTS));
    serial_puts("\n");
    return false;
}

static bool doorbell(void) {
    if (!eh_ensure_async()) return false;   /* звонок в мёртвое кольцо */
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) | CMD_IAAD);
    int t = 4000;                           /* ~150 мс (реально - мкс)  */
    while (!(eh_r32(g_op + R_USBSTS) & STS_IAA) && t-- > 0) eh_delay(10);
    eh_w32(g_op + R_USBSTS, STS_IAA);       /* R/WC */
    if (t >= 0) return true;                /* IAA пришёл вовремя       */
    /* рестарт async-механизма — единственный безопасный путь дальше */
    serial_puts("EHCI: doorbell timeout, async restart inst=");
    char nb[8];
    serial_puts(itoa((uint32_t)(g_ei + 1), nb, 10));
    serial_puts("\n");
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) & ~CMD_ASE);
    int t2 = 4000;
    while ((eh_r32(g_op + R_USBSTS) & STS_ASS) && t2-- > 0) eh_delay(10);
    eh_head[0] = QH_HL_QH(eh_head);         /* каноническое кольцо      */
    eh_dma();
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) | CMD_ASE);
    return false;
}

static bool eh_ctrl_run(int ntd) {
    /* вставить control-QH в кольцо после головы */
    eh_qhc[0] = QH_HL_QH(eh_head);
    eh_head[0] = QH_HL_QH(eh_qhc);
    eh_dma();
    /* [#405] ВСТАВКА — БЕЗ doorbell-гейта: Linux ehci-hcd тоже просто
     * вшивает QH в кольцо (iaa_watchdog нужен только для безопасного
     * ИЗЪЯТИЯ). HC с PARK=0 обходит кольцо непрерывно и подбирает
     * новый QH на следующем же обходе. Гейт «дождись IAA, иначе
     * считай провалом» (фото: «doorbell timeout» на AMD FCH, где IAA
     * вообще не звонит) убивал ВСЕ контрольные передачи EHCI: кольцо
     * реально работало, но драйвер отказывался ждать токен. Теперь:
     * подняли ASS — и сразу ждём сам qTD. */
    if (!eh_ensure_async()) {
        eh_head[0] = QH_HL_QH(eh_head);    /* кольцо назад в канон     */
        eh_dma();
        return false;
    }

    int t = 4000;                           /* ~200 мс: контрольный     */
    bool done = false;                      /* трансфер HS идёт единицы */
    while (t-- > 0) {                       /* мс; ретраев на уровне    */
        eh_dma();                           /* [#429] токен пишет HC — читаем из ОЗУ, не из кэша: иначе строка
                                               «вечно ACTIVE» → xfer timeout на КАЖДОЙ передаче (остаток #332) */
        if (!(eh_dma32(&eh_ctd[ntd - 1].token) & QTD_ACTIVE)) { done = true; break; }
        eh_delay(50);
    }
    if (!done) {
        serial_puts("EHCI-DBG: xfer timeout inst=");
        serial_puthex((uint32_t)g_ei);
        serial_puts(" cmd=");
        serial_puthex(eh_r32(g_op + R_USBCMD));
        serial_puts(" sts=");
        serial_puthex(eh_r32(g_op + R_USBSTS));
        serial_puts(" async=");
        serial_puthex(eh_r32(g_op + R_ASYNC));
        serial_puts(" p1=");
        serial_puthex(eh_r32(g_op + R_PORTSC));
        serial_puts(" qh0=");
        serial_puthex(eh_dma32(&eh_qhc[0]));
        serial_puts(" qh1=");
        serial_puthex(eh_dma32(&eh_qhc[1]));
        serial_puts(" qh3=");
        serial_puthex(eh_dma32(&eh_qhc[3]));
        serial_puts(" qh4=");
        serial_puthex(eh_dma32(&eh_qhc[4]));
        serial_puts(" qh6=");
        serial_puthex(eh_dma32(&eh_qhc[6]));
        serial_puts(" td0=");
        serial_puthex(eh_dma32(&eh_ctd[0].token));
        serial_puts(" td1=");
        serial_puthex(eh_dma32(&eh_ctd[1].token));
        serial_puts(" td2=");
        serial_puthex(eh_dma32(&eh_ctd[2].token));
        serial_puts("\n");
    }
    bool ok = done;
    eh_dma();                               /* [#429] свежие токены ВСЕХ стадий */
    for (int i = 0; i < ntd; i++)
        if (eh_dma32(&eh_ctd[i].token) & (QTD_ERRM | QTD_HALT)) ok = false;

    /* вынуть control-QH из кольца */
    eh_head[0] = QH_HL_QH(eh_head);
    eh_dma();
    /* G2 #227: при таймауте рестарт внутри doorbell сам вернул кольцо
     * в каноническое состояние и поднял ASE — на ИСХОД уже ЗАВЕРШИВШЕЙСЯ
     * передачи это не влияет, результат сознательно игнорируем. */
    (void)doorbell();
    return ok;
}

/* контрольная передача устройству addr (ep0 mps = mps) */
static bool eh_ctrl(uint8_t addr, uint8_t mps, uint8_t bm, uint8_t req,
                    uint16_t wv, uint16_t wi, uint16_t wlen, void *data) {
    (void)addr;   /* адрес живёт в qH epchar (FA) */
    g_rxlen = 0;  /* G2 #12(d): факт по последней IN-передаче */
    static uint8_t slot;
    uint8_t *su = eh_setup[slot & 3];
    slot++;
    su[0] = bm;  su[1] = req;
    su[2] = (uint8_t)wv;     su[3] = (uint8_t)(wv >> 8);
    su[4] = (uint8_t)wi;     su[5] = (uint8_t)(wi >> 8);
    su[6] = (uint8_t)wlen;   su[7] = (uint8_t)(wlen >> 8);

    /* EP0 mps: HS=64, FS=8..64, LS=8 */
    if (mps == 0 || mps > 64) mps = 64;
    /* F2-10: контроль границы DATA-буфера: запрос длиннее eh_data
     * урезается, а не переполняет буфер (и memcpy ниже). */
    if (wlen > sizeof(eh_data)) wlen = (uint16_t)sizeof(eh_data);

    /* Адрес/скорость/mps уже стоят в eh_qhc[1]/[2]: их ставит eh_ctrl_dev.
     * Здесь только чистка оверлея (токен неактивен) и голова TD-цепочки. */
    for (int i = 4; i < 16; i++) eh_qhc[i] = 0;
    eh_qhc[3] = QH_HL_TD(&eh_ctd[0]);

    memset(&eh_ctd[0], 0, sizeof(eh_ctd));
    /* SETUP: 8 байт, DATA0 */
    eh_ctd[0].next  = QH_HL_TD(&eh_ctd[1]);
    eh_ctd[0].alt   = QH_HL_TD(&eh_ctd[2]);
    eh_ctd[0].token = QTD_ACTIVE | QTD_PID_SETUP | QTD_CERR3 | QTD_LEN(8);
    qtd_bufs(&eh_ctd[0], (uint32_t)su);

    int ntd = 1;
    if (wlen > 0 && data) {
        memset(eh_data, 0, sizeof(eh_data));
        if (!(bm & 0x80)) memcpy(eh_data, data, wlen);
        eh_ctd[0].next = QH_HL_TD(&eh_ctd[1]);
        eh_ctd[1].next  = QH_HL_TD(&eh_ctd[2]);
        eh_ctd[1].alt   = QH_HL_TD(&eh_ctd[2]);
        /* DATA-стадия: DATA1 (ФИКС «этапа 2» — см. шапку файла, п.1).
         * Без QTD_DT реальное устройство отклоняло пакет по тогглу,
         * GET_DESCRIPTOR(8) отваливался и клавиатура 2.0 не работала. */
        eh_ctd[1].token = QTD_ACTIVE | QTD_CERR3 | QTD_DT |
                          (((bm & 0x80) ? QTD_PID_IN : 0u)) |
                          QTD_LEN(wlen);
        qtd_bufs(&eh_ctd[1], (uint32_t)eh_data);
        /* статус: противоположное направление, нулевая длина, DATA1 */
        eh_ctd[2].next  = QTD_END;      /* terminate */
        eh_ctd[2].alt   = QTD_END;
        eh_ctd[2].token = QTD_ACTIVE | QTD_CERR3 | QTD_DT |
                          ((bm & 0x80) ? 0u : QTD_PID_IN) |
                          QTD_LEN(0);
        qtd_bufs(&eh_ctd[2], (uint32_t)eh_data);
        ntd = 3;
    } else {
        /* без стадии данных: SETUP -> статус (IN) */
        eh_ctd[0].next  = QH_HL_TD(&eh_ctd[1]);
        eh_ctd[1].next  = QTD_END;
        eh_ctd[1].alt   = QTD_END;
        eh_ctd[1].token = QTD_ACTIVE | QTD_CERR3 | QTD_DT | QTD_PID_IN |
                          QTD_LEN(0);   /* статус-стадия = DATA1      */
        qtd_bufs(&eh_ctd[1], (uint32_t)eh_data);
        ntd = 2;
    }
    eh_dma();
    /* Оверлей control-QH = копия ПЕРВОГО qTD (токен ACTIVE!), как
     * qh_refresh() в Linux. */
    eh_qhc[4] = eh_ctd[0].next;
    eh_qhc[5] = eh_ctd[0].alt;
    eh_qhc[6] = eh_ctd[0].token;
    for (int k = 0; k < 5; k++) eh_qhc[7 + k] = eh_ctd[0].buf[k];
    eh_dma();
    bool ok = eh_ctrl_run(ntd);
    if (ok && wlen > 0 && data && (bm & 0x80)) {
        eh_dma();                           /* [#429] данные HC — из ОЗУ */
        memcpy(data, eh_data, wlen);
        /* G2 #12(d): сколько реально ПРИШЛО (short packet): HC пишет
         * остаток байт в поле Total Bytes qTD DATA-стадии (eh_ctd[1],
         * трёх-qTD схема). Парсер конфига клампит разбор этим числом. */
        if (ntd == 3) {
            int act = (int)wlen - (int)QTD_REM(eh_dma32(&eh_ctd[1].token));
            if (act < 0) act = 0;
            if (act > (int)wlen) act = (int)wlen;
            g_rxlen = (uint16_t)act;
        }
    }
    return ok;
}

/* F2-3: STALL на контрольной передаче? (HALT без ошибок шины в qTD) */
static bool eh_ctd_stalled(void) {
    for (int i = 0; i < 3; i++) {
        uint32_t tok = eh_dma32(&eh_ctd[i].token);
        if ((tok & QTD_HALT) && !(tok & QTD_ERRBITS)) return true;
    }
    return false;
}

/* контрольная передача с честной скоростью устройства (FS/LS split-поля) */
static bool eh_ctrl_dev(eh_dev_t *d, uint8_t mps, uint8_t bm, uint8_t req,
                        uint16_t wv, uint16_t wi, uint16_t wlen, void *data) {
    /* до SET_ADDRESS устройство на адресе 0 со скоростью ПОРТА */
    uint8_t spd = d->spd, addr = d->addr;
    eh_qhc[1] = QH_EPCHAR(addr, 0, spd, mps) & ~QH_H;   /* H=0: не голова */
    if (spd != 2) {
        eh_qhc[1] |= (1u << 27);               /* C: control через TT      */
        eh_qhc[2] = QH_CAP_FSLS;
    } else {
        eh_qhc[2] = 0;
    }
    bool ok = eh_ctrl(addr, mps, bm, req, wv, wi, wlen, data);
    if (!ok && eh_ctd_stalled()) {
        /* F2-3: после STALL (noname-клавы STALL'ят SET_IDLE) устройство
         * держит HALT и отклоняет ВСЕ следующие запросы, пока хост не
         * снимет его. CLEAR_FEATURE(ENDPOINT_HALT) на ep0 (это же
         * сбрасывает data-toggle устройства в DATA0) + ОДИН повтор
         * исходного запроса — иначе SET_PROTOCOL за STALL'нутым
         * SET_IDLE тоже падал и boot-протокол не включался. */
        (void)eh_ctrl(addr, mps, 0x02, 0x01, 0, 0, 0, 0);
        ok = eh_ctrl(addr, mps, bm, req, wv, wi, wlen, data);
    }
    return ok;
}

/* ── периодическое расписание: активные qH устройств цепочкой ──
 * ФИКС «этапа 2» (п.8): запись цепочки под выключенным PSE — контроллер
 * не читает qH, пока мы их переписываем (USBSTS.PSS ждёт останова).
 * F2-4 (аудит предлагал IAAD doorbell): IAAD — механизм ТОЛЬКО async-
 * кольца (doorbell при снятии QH уже стоит в eh_ctrl_run); для периодики
 * корректный барьер по спецификации — USBCMD.PSE=0 → USBSTS.PSS=0,
 * что здесь и реализовано (как в Linux ehci-hcd).
 * G2: PSS-ожидание стало РАБОТАТЬ только с исправленным битом 14
 * (см. комментарий у STS_PSS) — раньше «останавливали» async-бит. */
static void eh_sched_rebuild(void) {
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) & ~CMD_PSE);
    int t = 2000;
    while ((eh_r32(g_op + R_USBSTS) & STS_PSS) && t-- > 0) eh_delay(10);
    /* [#433] PSS не снялся (застрявший/quirky HC — бывает на реальном
     * железе): НЕ трогаем eh_fl под работающей периодикой — HC читал бы
     * полу-записанные указатели. Живое расписание остаётся, повторим
     * на следующем вызове. */
    if (t < 0) {
        serial_puts("EHCI: sched_rebuild: PSS не снялся - откладываю (расписание живо)\n");
        return;
    }

    uint32_t first = QTD_END;
    int prev = -1;
    for (int i = 0; i < EH_NDEV; i++) {
        if (!g_dv[i].used || !g_dv[i].alive) continue;
        g_qhi[i][0] = QTD_END;              /* хвост цепочки            */
        if (prev >= 0) g_qhi[prev][0] = QH_HL_QH(&g_qhi[i][0]);
        else           first = (uint32_t)&g_qhi[i][0];
        prev = i;
    }
    for (int i = 0; i < 1024; i++)
        eh_fl[i] = (first == QTD_END) ? QTD_END : QH_HL_QH(first & ~3u);
    eh_dma();
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) | CMD_PSE);
}

/* поставить interrupt-IN qTD устройства в очередь.
 * F2-5: ПЕРЕАРМ живого устройства идёт БЕЗ полного eh_sched_rebuild:
 * старый путь (rebuild на каждом отчёте) переписывал все 1024 записи
 * eh_fl + wbinvd + останавливал PSE десятки раз в секунду (мышь 1000 Гц,
 * опрос ~60 Гц) — катастрофа производительности. Теперь qTD и оверлей
 * переписываются «на месте» (токен неактивен — HC проходит мимо qH),
 * полный rebuild — только при первом арме и kill (bind/unbind). */
static void eh_arm_intr(eh_dev_t *d) {
    int i = (int)(d - g_dv);
    /* DTC=0 в qH: тоггл ведёт контроллер в оверлее qH (DATA0/DATA1
     * чередуются между переармами - как требует USB для interrupt-EP).
     * Бит DT (31) оверлея переживает переарм. */
    g_qtdi[i].next  = QTD_END;
    g_qtdi[i].alt   = QTD_END;
    g_qtdi[i].token = QTD_ACTIVE | QTD_PID_IN | QTD_CERR3 | QTD_LEN(d->mps);
    qtd_bufs(&g_qtdi[i], (uint32_t)g_pkt[i]);
    g_qhi[i][3] = QH_HL_TD(&g_qtdi[i]);
    g_qhi[i][4] = g_qtdi[i].next;
    g_qhi[i][5] = g_qtdi[i].alt;
    if (d->armed) {
        /* qH уже в цепочке: связи [0] и epchar [1]/[2] НЕ трогаем — HC
         * может читать qH прямо сейчас (F2-17); неактивный токен в
         * оверлее = «пропустить qH», характеристики менять нельзя. */
        g_qhi[i][6] = (g_qtdi[i].token & ~QTD_DT) | (g_qhi[i][6] & QTD_DT);
        for (int k = 0; k < 5; k++) g_qhi[i][7 + k] = g_qtdi[i].buf[k];
        eh_dma();
        return;
    }
    /* первый арм: инициализация qH и вставка в расписание */
    g_qhi[i][1] = QH_EPCHAR(d->addr, d->ep & 0xF, d->spd, d->mps) &
                  ~(QH_DTC | QH_H);        /* не голова, тоггл ведёт qH */
    if (d->spd != 2) g_qhi[i][2] = QH_CAP_FSLS;
    else             g_qhi[i][2] = QH_CAP_HS;
    /* F2-17: НОВОЕ использование endpoint (reconnect/повторное занятие
     * слота) обязано стартовать с DATA0 — бит DT оверлея мог остаться
     * DATA1 от прошлой сессии слота, и все отчёты отклонялись. */
    g_qhi[i][6] = g_qtdi[i].token & ~QTD_DT;
    for (int k = 0; k < 5; k++) g_qhi[i][7 + k] = g_qtdi[i].buf[k];
    d->armed = true;
    eh_dma();
    eh_sched_rebuild();
}

/* снять устройство с расписания */
static void eh_kill_dev(eh_dev_t *d, const char *why) {
    if (!d->used) return;
    d->alive = false;
    serial_puts("EHCI: устройство отключено (порт ");
    serial_puthex(d->port);
    serial_puts("): ");
    serial_puts(why);
    serial_puts("\n");
    if (d->role == 2) {
        usb_kbd_parse_boot(0, -1, "EHCI");   /* сброс зажатий клавиатуры */
        ps2_kbd_detached();
        usbdev_release(USBDEV_ROLE_KBD, why);
    } else if (d->role == 1) {
        usbdev_release(USBDEV_ROLE_MOUSE, why);
    }
    d->role = 0;
    /* G2 #193/#258: адрес — обратно в битмапу (hotplug не течёт) */
    eh_addr_free(d->addr);
    d->addr = 0;
    d->used = false;
    d->armed = false;     /* F2-5: следующий арм — с полным rebuild */
    eh_sched_rebuild();
}

/* ── разбор конфигурации: найти HID boot-интерфейсы ──
 * has_mouse/has_kbd + их интерфейс/эп/mps. */
static bool eh_find_hid(const uint8_t *cfg, uint16_t total,
                        uint8_t *m_if, uint8_t *m_ep, uint8_t *m_mps,
                        uint8_t *k_if, uint8_t *k_ep, uint8_t *k_mps) {
    bool hm = false, hk = false;
    uint8_t cur_if = 0;
    int pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len = cfg[pos], type = cfg[pos + 1];
        if (len < 2 || pos + len > (int)total) break;   /* [#431] OOB-гвард: мусорный len у конца cfg[512] читал ЗА буфером (стек) */
        if (type == 0x04 && len >= 9) {          /* interface */
            cur_if = cfg[pos + 2];
            if (cfg[pos + 5] == 0x03) {          /* class HID */
                uint8_t sub = cfg[pos + 6], proto = cfg[pos + 7];
                if (sub == 0x01 && proto == 0x02 && !hm) {
                    *m_if = cur_if; hm = true;
                } else if (sub == 0x01 && proto == 0x01 && !hk) {
                    *k_if = cur_if; hk = true;
                }
            }
        } else if (type == 0x05 && len >= 6) {   /* endpoint */
            uint8_t ea = cfg[pos + 2], et = cfg[pos + 3] & 3u;
            uint8_t mp = cfg[pos + 4];
            if (et == 3 && (ea & 0x80u)) {       /* interrupt IN */
                if (hm && *m_if == cur_if && *m_ep == 0) {
                    *m_ep = ea & 0xFu; *m_mps = mp;
                }
                if (hk && *k_if == cur_if && *k_ep == 0) {
                    *k_ep = ea & 0xFu; *k_mps = mp;
                }
            }
        }
        pos += len;
    }
    return hm || hk;
}

/* ── перечислить устройство на порту p (скорость уже известна) ── */
static void eh_enum_dev(uint8_t p, uint8_t spd) {
    eh_dev_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.spd = spd;
    tmp.port = p;
    /* 0.3.0: ПОРТ представляется диспетчеру (EHCI = «USB 2.0») */
    usbdev_ask_port(eh_bus_label(), 2, (int)p);

    uint8_t dd8[8];
    memset(dd8, 0, sizeof(dd8));
    if (!eh_ctrl_dev(&tmp, 64, 0x80, 0x06, 0x0100, 0, 8, dd8)) {
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": GET_DESCRIPTOR(8) не прошёл\n");
        eh_diag_set("GET_DESCRIPTOR(8) не прошёл");
        return;
    }
    uint8_t mps = dd8[6];              /* bMaxPacketSize0: 8/16/32/64 */
    /* G2 #12(e): у HIGH-SPEED EP0 MPS фиксирован 64 (спека USB 2.0,
     * 5.5.3) — «левое» значение из дескриптора не должно расходиться
     * с пакетизацией DATA-стадии. FS/LS (прямой путь без компаньона) —
     * честные 8..64 из дескриптора с клампом. */
    if (spd == 2) mps = 64;
    if (mps > 64) mps = 64;
    if (mps < 8) mps = 8;
    tmp.mps0 = mps;                    /* F2-3: ep0-mps для clear-halt */

    /* G2 #193/#258: адрес из БИТМАПЫ — первый свободный бит 1..127,
     * поиск с курсора g_next_addr (с живым устройством не совпадёт по
     * построению). Возврат — во ВСЕХ путях выхода ниже (out_free),
     * иначе hotplug снова «течёт» адресами. */
    uint8_t na = eh_addr_alloc();
    if (!na) { eh_diag_set("адреса устройств кончились"); return; }
    tmp.addr = 0;
    if (!eh_ctrl_dev(&tmp, mps, 0x00, 0x05, na, 0, 0, 0)) {
        serial_puts("EHCI: SET_ADDRESS не прошёл (порт ");
        serial_puthex(p); serial_puts(")\n");
        eh_diag_set("SET_ADDRESS не прошёл");
        goto out_free;
    }
    eh_delay(20000);
    tmp.addr = na;

    uint8_t dd[18];
    memset(dd, 0, sizeof(dd));
    if (!eh_ctrl_dev(&tmp, mps, 0x80, 0x06, 0x0100, 0, 18, dd)) {
        eh_diag_set("GET_DESCRIPTOR(18) не прошёл");
        goto out_free;
    }
    tmp.vid = (uint16_t)(dd[8] | ((uint16_t)dd[9] << 8));
    tmp.pid = (uint16_t)(dd[10] | ((uint16_t)dd[11] << 8));

    uint8_t cfg9[9];
    memset(cfg9, 0, sizeof(cfg9));
    if (!eh_ctrl_dev(&tmp, mps, 0x80, 0x06, 0x0200, 0, 9, cfg9)) {
        eh_diag_set("GET_CONFIG(9) не прошёл");
        goto out_free;
    }
    /* G2 #12(d): валидация головы конфига — мусор (зависшее устройство,
     * кривая прошивка) не должен попадать в парсер wTotalLength. */
    if (cfg9[0] < 9 || cfg9[1] != 0x02) {
        eh_diag_set("GET_CONFIG(9): не конфиг-дескриптор");
        goto out_free;
    }
    uint16_t total = (uint16_t)(cfg9[2] | ((uint16_t)cfg9[3] << 8));
    /* F2-10: composite-конфигурации бывают длиннее 255 (клава+тачпад+
     * consumer): читаем до 512 — сколько влезает в eh_data (кламп в
     * eh_ctrl); HID-интерфейсы из хвоста больше не теряются.
     * G2 #4(b): wTotalLength>512 — усечение С ЛОГОМ (рядом с хабом
     * встречаются составные конфиги за 400 Б; парсим что влезло). */
    if (total > sizeof(eh_data)) {
        char nb[8];
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": конфиг ");
        serial_puts(itoa(total, nb, 10));
        serial_puts(" Б - усечён до 512\n");
        total = (uint16_t)sizeof(eh_data);
    }
    uint8_t cfg[512];
    if (!eh_ctrl_dev(&tmp, mps, 0x80, 0x06, 0x0200, 0, total, cfg)) {
        eh_diag_set("GET_CONFIG(full) не прошёл");
        goto out_free;
    }
    /* G2 #12(d): парсим только РЕАЛЬНО прочитанное — короткий пакет
     * (g_rxlen) обрезает хвост; хвост буфера нулевой (memset в eh_ctrl),
     * len<2 корректно завершает обход. Плюс проверка bDescriptorType. */
    if (cfg[0] < 9 || cfg[1] != 0x02) {
        eh_diag_set("GET_CONFIG(full): не конфиг-дескриптор");
        goto out_free;
    }
    if (g_rxlen < total) total = g_rxlen;

    uint8_t m_if = 0, m_ep = 0, m_mps = 0, k_if = 0, k_ep = 0, k_mps = 0;
    if (!eh_find_hid(cfg, total, &m_if, &m_ep, &m_mps, &k_if, &k_ep, &k_mps)) {
        serial_puts("EHCI: HID-интерфейсов нет (vid=");
        serial_puthex(tmp.vid);
        serial_puts(" pid=");
        serial_puthex(tmp.pid);
        serial_puts(")\n");
        /* 0.3.0: ни клава, ни мышь — диспетчер печатает вердикт */
        usbdev_try_bind(2, 0, 0, eh_bus_label(), (int)p, tmp.vid, tmp.pid);
        eh_diag_set("HID-интерфейсов нет (НЕИЗВЕСТНОЕ УСТРОЙСТВО)");
        goto out_free;
    }
    if (!eh_ctrl_dev(&tmp, mps, 0x00, 0x09, cfg9[5], 0, 0, 0)) {
        eh_diag_set("SET_CONFIG не прошёл");
        goto out_free;
    }

    /* взять слот устройства */
    eh_dev_t *d = 0;
    for (int i = 0; i < EH_NDEV; i++)
        if (!g_dv[i].used) { d = &g_dv[i]; break; }
    if (!d) { eh_diag_set("слоты устройств кончились"); goto out_free; }
    *d = tmp;
    d->first_pkt = false;    /* журнал первого отчёта заново */
    d->used = true;          /* слот занят живым устройством */

    /* 0.3.0: «одно устройство = один драйвер из ШЕСТИ» — решение
     * принимает диспетчер usbdev: клавиатура пробуется ПЕРВОЙ, мышь —
     * если слот клавиатуры занят или устройство не клавиатура.  Составное
     * устройство (клава+тачпад) получает драйвер клавиатуры; второй
     * интерфейс подхватит только переподключение. */
    {
        int role = usbdev_try_bind(2, k_ep != 0, m_ep != 0,
                                   eh_bus_label(), (int)p, tmp.vid, tmp.pid);
        if (role == USBDEV_ROLE_KBD && k_ep)        d->role = 2;
        else if (role == USBDEV_ROLE_MOUSE && m_ep) d->role = 1;
        else {
            /* ни один драйвер не подошёл / оба слота реестра заняты */
            serial_puts("EHCI: порт ");
            serial_puthex(p);
            serial_puts(": НЕИЗВЕСТНОЕ УСТРОЙСТВО (см. usbdev выше)\n");
            d->used = false;
            eh_diag_set("НЕИЗВЕСТНОЕ УСТРОЙСТВО");
            goto out_free;
        }
    }
    uint8_t ep = 0, mp = 0, ifc = 0;
    if (d->role == 1) { ep = m_ep; mp = m_mps; ifc = m_if; }
    else              { ep = k_ep; mp = k_mps; ifc = k_if; }
    if (!ep) {
        usbdev_release(d->role == 2 ? USBDEV_ROLE_KBD : USBDEV_ROLE_MOUSE,
                       "interrupt-EP не найден");
        d->used = false;
        eh_diag_set("interrupt-EP не найден");
        goto out_free;
    }
    d->ep = ep;
    d->mps = mp ? mp : 8;
    if (d->mps > 16) d->mps = 16;
    d->iface = ifc;
    d->alive = true;

    (void)eh_ctrl_dev(d, mps, 0x21, 0x0A, 0, ifc, 0, 0);            /* SET_IDLE */
    /* SET_PROTOCOL(boot) нужен ЛЮБОМУ HID-интерфейсу (HID 1.11 §7.2.6:
     * устройства стартуют в report-протоколе). */
    (void)eh_ctrl_dev(d, mps, 0x21, 0x0B, 0, ifc, 0, 0);            /* SET_PROTOCOL boot */

    eh_arm_intr(d);

    serial_puts("HID: addr=");
    char nb[8];
    serial_puts(itoa(d->addr, nb, 10));
    serial_puts(" port=");
    serial_puts(itoa(p, nb, 10));
    serial_puts(" speed=");
    serial_puts(spd == 2 ? "2.0" : "1.x");
    serial_puts(" proto=");
    serial_puts(d->role == 1 ? "mouse" : "keyboard");
    serial_puts(" via=interface iface=");
    serial_puts(itoa(ifc, nb, 10));
    serial_puts("\n");
    serial_puts("EHCI: устройство готово vid=");
    serial_puthex(d->vid);
    serial_puts(" pid=");
    serial_puthex(d->pid);
    serial_puts("\n");

    usbmap_pick(0x20, d->role == 1 ? "mouse" : "keyboard", p, "2.0");
    eh_diag_set("устройство готово");
    return;                    /* успех: адрес закреплён за слотом      */

out_free:
    /* G2 #193/#258: неудачное перечисление — адрес обратно в битмапу
     * (порт позже ресетится, устройство вернётся на адрес 0). */
    eh_addr_free(na);
}

/* ── опрос ОДНОГО порта: подключение/отключение/сброс/перечисление ──
 * force=1 — перечислить подключённое даже без CSC (стартовый обход). */
static void eh_port_service(uint8_t p, int force) {
    uint32_t pa = g_op + R_PORTSC + 4u * (uint32_t)(p - 1);
    uint32_t v = eh_r32(pa);
    bool mine = false;
    for (int i = 0; i < EH_NDEV; i++)
        if (g_dv[i].used && g_dv[i].alive && g_dv[i].port == p) mine = true;

    /* 0.3.2: порт уже отдан компаньону (Port Owner=1) — не трогаем:
     * его CCS/CSC отражает чужую (компаньонскую) логику, а запись
     * сюда может пересбросить состояние чужого устройства. */
    if (v & PS_PO) return;

    /* G2 #270: overcurrent — счистить w1c-битом, устройства порта —
     * вон (как disconnect; по спеке порт сам уходит в disabled, питание
     * рвёт хаб/контроллер — PED руками не трогаем). */
    if (v & PS_OCC) {
        eh_w32(pa, (v & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_OCC);
        serial_puts("EHCI: overcurrent порт ");
        serial_puthex(p);
        serial_puts("\n");
        for (int i = 0; i < EH_NDEV; i++)
            if (g_dv[i].used && g_dv[i].port == p)
                eh_kill_dev(&g_dv[i], "overcurrent");
        g_conn[p] = false;
        g_fail[p] = 0;
        v = eh_r32(pa);
        if (!(v & PS_CCS)) return;      /* порт пуст — ждём подключения */
    }

    /* G2 #264/#265: быстрый выдер/втык МЕЖДУ опросами (~1 с) успевает
     * погасить CSC до нашего чтения. Сверяем CCS с кэшем: соединение
     * пропало БЕЗ change-бита — всё равно честный disconnect, иначе
     * «наше» устройство вечно числится alive, а qTD молчит. */
    if (g_conn[p] && !(v & PS_CCS) && !(v & PS_CSC)) {
        for (int i = 0; i < EH_NDEV; i++)
            if (g_dv[i].used && g_dv[i].port == p)
                eh_kill_dev(&g_dv[i], "CCS пропал (CSC пропущен)");
        g_conn[p] = false;
        g_fail[p] = 0;
        return;
    }

    if (v & PS_CSC) {
        /* G2: в записи гасим ТОЛЬКО CSC (раньше (v&0xFFFF)|PS_CSC заодно
         * съедал PEC/OCC, если те уже стояли) */
        eh_w32(pa, (v & 0xFFFFu & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_CSC);
        v = eh_r32(pa);
        if (!(v & PS_CCS)) {
            /* честное отключение: устройства порта - вон, счётчик - в 0 */
            for (int i = 0; i < EH_NDEV; i++)
                if (g_dv[i].used && g_dv[i].port == p)
                    eh_kill_dev(&g_dv[i], "CSC");
            g_conn[p] = false;          /* G2 #264: кэш conn-состояния  */
            g_fail[p] = 0;
            /* G2 #271: «стойкие» контроллеры держат PED=1 после
             * disconnect (большинство сбрасывает его само). Кто не
             * сбросил за 10 мс — гасим явно: пишем ТЕКУЩЕЕ значение
             * с PED=0 и погашенными change-битами (w1c); запись в
             * RO-биты игнорируется, PP не трогаем — порт готов
             * к следующему hotplug. */
            kdelay_ms(10);
            uint32_t pv = eh_r32(pa);
            if ((pv & PS_PED) && !(pv & PS_CCS))
                eh_w32(pa, (pv & ~(PS_CSC | PS_PEC | PS_OCC)) & ~PS_PED);
            return;
        }
        /* CSC с подключённым устройством: «ЭТАП 3» — дебаунс 50 мс
         * (Linux hub.c): дребезг не должен запускать сброс порта.
         * ЛИБО переподключение (дать ещё один шанс), ЛИБО след нашего
         * собственного port-reset из rescan-попытки (тогда счётчик НЕ
         * трогаем — иначе бэкофф из 3 попыток никогда не срабатывал
         * и порт молотился каждую секунду бесконечно, QEMU поднимал
         * CSC на каждый наш сброс). */
        if (v & PS_CCS) {
            eh_delay(50000);              /* 50 мс стабильности */
            if (!(eh_r32(pa) & PS_CCS)) {
                serial_puts("EHCI: порт ");
                serial_puthex(p);
                serial_puts(": дребезг (CCS пропал) - игнорирую\n");
                return;
            }
            v = eh_r32(pa);
        }
        if (g_fail[p] >= 3) g_fail[p] = 2;   /* переподключение: +1 шанс */
    }
    /* G2 #264/#265: обновляем кэш conn-состояния (после обработки
     * change-битов и дебаунса) — с ним сверяемся на СЛЕДУЮЩЕМ обходе. */
    g_conn[p] = (v & PS_CCS) != 0;
    /* G2 #268 (консервативно): НЕ-ресетим РАБОЧИЙ порт: слот НАШ жив
     * (CCS=1, PED=1, адрес в нашем g_dv) — выходим ДО сброса. Повторный
     * reset рабочего HS-порта (особенно USB3-в-2.0-режиме) может
     * подвесить устройство; агрессивный вариант «GET_DESCRIPTOR сразу
     * без сброса» для BIOS-перечисленных портов неверен — устройство
     * сидит на ЧУЖОМ для нас адресе, сброс обязателен. */
    if (mine) return;                    /* живое наше устройство      */
    if (!(v & PS_CCS)) return;           /* пусто                      */

    /* подключено, но НЕ наше: 3 неудачи — ждём переподключения */
    if (g_fail[p] >= 3) return;
    if (!force) {
        /* не стартовый обход и не CSC: это рескан-попытка (см.
         * ehci_rescan_unclaimed) — печатаем, чтобы было видно в журнале */
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": устройство не заявлено - перечисляю снова\n");
    }

    /* low-speed-устройство (Line Status = K) отдаём компаньону СРАЗУ и
     * БЕЗ порта-реста: EHCI-спека §4.2 прямо требует не гнать
     * high-speed-рест в LS-линию. */
    if (((v >> 10) & 3u) == 1u && g_have_companion) {
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": low-speed устройство - компаньону (без сброса)\n");
        eh_w32(pa, (v & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_PO | PS_PP);
        eh_diag_set("LS - компаньону (без сброса)");
        return;
    }

    /* ФИКС «этапа 2» (п.5): порт с PED=1 (BIOS успел перечислить
     * устройство до нас, оно сидит на чужом адресе) РАНЬШЕ перечислялся
     * «как есть» — устройство молчало. Теперь ЛЮБОЙ ненаш порт получает
     * честный port reset: устройство возвращается на адрес 0. */
    if (!(v & PS_PP)) {
        eh_w32(pa, (v & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_PP);
        /* F2-8: 100 мс (было 20) — часть EHCI (VIA) поднимает питание
         * порта до 100 мс: раньше порт читался «пусто» и устройство
         * сразу попадало в 3-попыточный бэкофф. */
        kdelay_ms(100);
        v = eh_r32(pa);
    }
    eh_w32(pa, (v & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_PR);
    int t = 5000;                       /* F2-9: ~500 мс (было 250): Intel
                                           снимает PR до 200 мс — было
                                           впритык; 500 мс — как Linux */
    while ((eh_r32(pa) & PS_PR) && t-- > 0) eh_delay(100);
    /* Реальный HC снимает PR сам; QEMU ждёт записи гостя. */
    v = eh_r32(pa);
    if (v & PS_PR)
        eh_w32(pa, v & ~(PS_CSC | PS_PEC | PS_OCC | PS_PR));
    eh_delay(50000);                    /* recovery                    */
    v = eh_r32(pa);
    if (!(v & PS_CCS)) {
        eh_diag_set("устройство отвалилось при сбросе");
        return;
    }
    if (v & PS_PED) {
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": устройство high-speed, перечисляю\n");
        eh_enum_dev(p, 2);
        for (int i = 0; i < EH_NDEV; i++)
            if (g_dv[i].used && g_dv[i].alive && g_dv[i].port == p)
                g_fail[p] = 0;
        return;
    }
    /* HS-включения нет: FS/LS устройство (или не ответило) */
    uint32_t line = (v >> 10) & 3u;
    if (g_have_companion) {
        if (g_ncc == 0) {
            /* [#432] компаньон найден ГЛОБАЛЬНЫМ сканом, но у ЭТОГО EHCI
             * своих компаньонов нет (N_CC=0 — дискретная USB2-карта при
             * живом чипсетом): маршрутизация PS_PO ушла бы «в никуда» и
             * устройство потерялось навсегда. Пытаемся напрямую. */
            serial_puts("EHCI: порт ");
            serial_puthex(p);
            serial_puts(": FS/LS, но N_CC=0 (своих компаньонов нет) - прямая попытка\n");
            eh_enum_dev(p, line == 1 ? 1 : 0);
            return;
        }
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": устройство full/low-speed - отдаю компаньону (N_CC=");
        { char nb[8]; serial_puts(itoa((uint32_t)g_ncc, nb, 10)); }
        serial_puts(")\n");
        eh_w32(pa, (v & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_PO | PS_PP);
        eh_diag_set("FS/LS - компаньону");
    } else {
        serial_puts("EHCI: порт ");
        serial_puthex(p);
        serial_puts(": full/low-speed без компаньона - прямая попытка\n");
        eh_enum_dev(p, line == 1 ? 1 : 0);
        for (int i = 0; i < EH_NDEV; i++)
            if (g_dv[i].used && g_dv[i].alive && g_dv[i].port == p)
                g_fail[p] = 0;
    }
}

/* обход всех портов инстанса */
static void eh_ports_poll(int force) {
    if (!g_running) return;
    for (uint8_t p = 1; p <= g_nports; p++)
        eh_port_service(p, force);
}

/* ФИКС «этапа 2» (п.4): устройство, чьё перечисление НЕ ПРОШЛО,
 * раньше не пробовалось НИКОГДА (CCS без CSC не перечитывался —
 * только переподключение). Теперь каждую секунду даём порту до 3
 * попыток (дальше — ждать переподключения), как ohci_rescan_unclaimed. */
static void ehci_rescan_unclaimed(void) {
    if (!g_running) return;
    for (uint8_t p = 1; p <= g_nports && p < 16; p++) {
        if (g_fail[p] >= 3) continue;
        bool mine = false;
        for (int i = 0; i < EH_NDEV; i++)
            if (g_dv[i].used && g_dv[i].alive && g_dv[i].port == p) mine = true;
        if (mine) continue;
        uint32_t v = eh_r32(g_op + R_PORTSC + 4u * (uint32_t)(p - 1));
        if (v & PS_PO) continue;              /* порт у компаньона       */
        if (!(v & PS_CCS)) { g_fail[p] = 0; continue; }
        if (v & PS_CSC) { eh_port_service(p, 0); continue; }
        eh_port_service(p, 0);                /* nенаш - ещё попытка     */
        bool ok = false;
        for (int i = 0; i < EH_NDEV; i++)
            if (g_dv[i].used && g_dv[i].alive && g_dv[i].port == p) ok = true;
        if (ok) g_fail[p] = 0;
        else if (++g_fail[p] == 3) {
            serial_puts("EHCI: порт ");
            serial_puthex(p);
            serial_puts(": не перечислился за 3 попытки - жду переподключения\n");
        }
    }
}

/* ── разобрать interrupt-IN отчёт устройства ── */
static void eh_poll_dev(eh_dev_t *d) {
    int i = (int)(d - g_dv);
    eh_dma();                               /* [#429] токен qTD пишет HC: без инвалидации — застоявшаяся строка */
    uint32_t tok = eh_dma32(&g_qtdi[i].token);
    if (tok & QTD_ACTIVE) return;
    if (tok & (QTD_ERRM | QTD_HALT)) {
        if ((tok & QTD_HALT) && !(tok & QTD_ERRBITS)) {
            /* F2-3: STALL на interrupt-EP: тоггл endpoint'а остаётся
             * битым и все отчёты отклоняются. CLEAR_FEATURE(ENDPOINT_
             * HALT) + сброс тоггла оверлея в DATA0 — следующий арм
             * стартует с DATA0. */
            (void)eh_ctrl_dev(d, d->mps0, 0x02, 0x01, 0,
                              (uint16_t)(0x80u | (d->ep & 0xFu)), 0, 0);
            g_qhi[i][6] &= ~QTD_DT;
        }
        eh_arm_intr(d);                       /* переармим - не убиваем    */
        return;
    }
    int act = (int)d->mps - (int)QTD_REM(tok);
    if (act < 0) act = 0;
    if (act > (int)sizeof(g_pkt[i])) act = (int)sizeof(g_pkt[i]);   /* [#450] до 64 */
    if (act < 1) {
        /* [#475] Ситуация B: ДЛИНА 0 — это НЕ «живые данные»: не считаем
         * отчёт (счётчики живости не растут), курсор/клавиши не трогаем,
         * переводим TRB заново. Троттлинг — как ratelimit в Linux. */
        static uint32_t zl_cnt;
        zl_cnt++;
        if (zl_cnt == 1u || (zl_cnt & 1023u) == 0u)
            serial_puts("EHCI: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)\n");
        eh_arm_intr(d);
        return;
    }
    g_ehci_pkts++;
    if (!d->first_pkt) {        /* первый отчёт КАЖДОГО устройства */
        d->first_pkt = true;
        serial_puts("EHCI: первый отчёт ");
        serial_puts(d->role == 1 ? "мыши" : "клавиатуры");
        serial_puts(" (");
        char nb[8];
        serial_puts(itoa((uint32_t)act, nb, 10));
        serial_puts(" байт):");
        for (int k = 0; k < act; k++) { serial_puts(" "); serial_puthex(g_pkt[i][k]); }
        serial_puts("\n");
    }
    if (d->role == 2) {
        usb_kbd_parse_boot(g_pkt[i], act, "EHCI");
    } else if (d->role == 1 && act >= 3) {
        usbdev_note_report(USBDEV_ROLE_MOUSE);   /* [#462] живость для КАРТЫ ГНЕЗД */
        uint8_t btn = g_pkt[i][0];
        int dx = (int)(int8_t)g_pkt[i][1] * 2;
        int dy = (int)(int8_t)g_pkt[i][2] * 2;
        if (dx >  48) dx =  48;
        if (dx < -48) dx = -48;
        if (dy >  48) dy =  48;
        if (dy < -48) dy = -48;
        int nx = g_ms.x + dx, ny = g_ms.y + dy;
        if (nx < 0) nx = 0;
        if (nx >= g_fb_w) nx = g_fb_w - 1;
        if (ny < 0) ny = 0;
        if (ny >= g_fb_h) ny = g_fb_h - 1;
        if (nx != g_ms.x || ny != g_ms.y) g_ms.moved = true;
        g_ms.x = nx; g_ms.y = ny;
        bool nl = (btn & 1) != 0, nr = (btn & 2) != 0;
        /* F2-13: prev-состояние ПЕР-УСТРОЙСТВО (глобальные g_ms_prev_l/r
         * давали фантомные клики при мышах на разных инстансах EHCI) */
        if (nl && !d->prev_l) g_ms.clicked_left = true;
        if (nr && !d->prev_r) g_ms.clicked_right = true;
        d->prev_l = nl; d->prev_r = nr;
        g_ms.left = nl; g_ms.right = nr;
    }
    eh_arm_intr(d);
}

/* ── PCI-скан: есть ли компаньон (OHCI/UHCI) ── */
static bool eh_scan_companion(void) {
    for (uint16_t bus = 0; bus < 256; bus++)
        for (uint8_t dev = 0; dev < 32; dev++)
            for (uint8_t fn = 0; fn < 8; fn++) {
                uint32_t id = pci_read32((uint8_t)bus, dev, fn, 0x00);
                if ((id & 0xFFFFu) == 0xFFFFu) {
                    if (fn == 0) break;
                    continue;
                }
                uint32_t cc = pci_read32((uint8_t)bus, dev, fn, 0x08);
                if ((cc >> 24) == 0x0Cu && ((cc >> 16) & 0xFFu) == 0x03u) {
                    uint8_t pg = (uint8_t)((cc >> 8) & 0xFFu);
                    if (pg == 0x00u || pg == 0x10u) return true;
                }
                if (fn == 0 && !(pci_read8((uint8_t)bus, dev, fn, 0x0E) & 0x80u))
                    break;
            }
    return false;
}

/* ── init одного контроллера (инстанс g_ei); true - поднят ──────────── */
static bool eh_init_one(uint8_t bus, uint8_t dev, uint8_t fn, int force_usb) {
    serial_puts("EHCI: контроллер ");
    serial_puthex(bus); serial_puts(":");
    serial_puthex(dev); serial_puts(".");
    serial_puthex(fn);
    serial_puts(" (это ");
    serial_puts(eh_bus_label());
    serial_puts(")\n");

    uint32_t bar = pci_read32(bus, dev, fn, 0x10);
    if (bar & 1u) { serial_puts("EHCI: BAR0 - I/O, пропускаю\n"); return false; }
    uint32_t mm = bar & ~0xFu;
    if (mm < 0x1000u) { serial_puts("EHCI: плохой BAR0, пропускаю\n"); return false; }
    uint32_t cmd = pci_read32(bus, dev, fn, 0x04);
    pci_write32(bus, dev, fn, 0x04, cmd | 0x6u);    /* mem + busmaster */

    uint32_t caplen = eh_r32(mm) & 0xFFu;
    if (caplen < 0x20u || caplen > 0xFFu) caplen = 0x20u;
    g_mm = mm;
    g_op = mm + caplen;

    /* BIOS handoff (USBLEGSUP: bit 16 BIOS-owned, bit 24 OS-owned).
     * ФИКС «этапа 2» (п.6): после захвата (или таймаута) глушим
     * USBLEGCTLSTS (EECP+4) — SMI-источники BIOS'а не стреляют по
     * нашему контроллеру; таймаут рукопожатия ~1 с. */
    uint32_t hcc = eh_r32(mm + 0x08u);
    uint32_t eecp = (hcc >> 8) & 0xFFu;
    if (eecp >= 0x40u) {
        uint32_t leg = pci_read32(bus, dev, fn, (uint8_t)eecp);
        if (leg & 0x010000u) {                     /* BIOS owns */
            pci_write32(bus, dev, fn, (uint8_t)eecp, leg | 0x01000000u);
            /* «ЭТАП 3»: ожидание до 1.5 с (было ~0.5) — реальный BIOS
             * (AMD FCH) отдаёт владение не мгновенно; Linux ждёт секунды. */
            int t = 1500;
            while ((pci_read32(bus, dev, fn, (uint8_t)eecp) & 0x010000u) &&
                   t-- > 0)
                eh_delay(1000);
            serial_puts("EHCI: BIOS handoff ");
            serial_puts(t > 0 ? "выполнен\n" : "таймаут (беру силой)\n");
        }
        /* USBLEGCTLSTS = 0: отключить ВСЕ SMI-источники legacy-поддержки */
        pci_write32(bus, dev, fn, (uint8_t)(eecp + 4u), 0u);
    }

    /* halt + reset */
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) & ~CMD_RS);
    int t = 20000;
    while (!(eh_r32(g_op + R_USBSTS) & STS_HALT) && t-- > 0) eh_delay(10);
    eh_w32(g_op + R_USBCMD, CMD_HCR);
    t = 2000;
    while ((eh_r32(g_op + R_USBCMD) & CMD_HCR) && t-- > 0) eh_delay(10);

    /* число портов — из HCSPARAMS (КАПАБИЛИТИ-регистры, mm+4) */
    uint32_t hcs = eh_r32(g_mm + 0x04u);
    g_nports = (uint8_t)(hcs & 0xFu);
    g_ncc    = (uint8_t)((hcs >> 12) & 0xFu);   /* [#432] СВОЙ N_CC компаньонов */
    if (g_nports == 0 || g_nports > 15) g_nports = 4;
    serial_puts("EHCI: портов ");
    serial_puthex(g_nports);
    serial_puts("\n");

    /* расписания (ПЕР-ИНСТАНСНЫЕ - см. шапку, п.2) */
    for (int i = 0; i < 1024; i++) eh_fl[i] = QTD_END;
    eh_head[0] = QH_HL_QH(eh_head);
    eh_head[1] = QH_EPCHAR(0, 0, 2, 64);  /* H=1: ГОЛОВА async-кольца */
    for (int i = 2; i < 16; i++) eh_head[i] = 0;
    eh_head[3] = QTD_END; eh_head[4] = QTD_END; eh_head[5] = QTD_END;
    eh_w32(g_op + R_CTRLDSSEG, 0);   /* G2 #190: 64-битные сегменты не
                                        нужны (вся память < 4 ГБ); пишем
                                        ПОСЛЕ HCRESET и ДО USBCMD.RUN */
    eh_w32(g_op + R_FRINDEX, 0);
    eh_w32(g_op + R_PERIODIC, (uint32_t)eh_fl);
    eh_w32(g_op + R_ASYNC, (uint32_t)eh_head);
    eh_w32(g_op + R_USBINTR, 0);
    eh_dma();

    for (int i = 0; i < EH_NDEV; i++) { g_dv[i].used = false; g_dv[i].alive = false; }
    for (int i = 0; i < 16; i++) g_fail[i] = 0;
    g_next_addr = 1;                     /* G2: курсор поиска адресов  */
    memset(eh_addr_bm, 0, sizeof(eh_addr_bm));
    memset(g_conn, 0, sizeof(g_conn));
    eh_diag_set("расписание поднято, CONFIGFLAG=0");

    /* запуск БЕЗ CONFIGFLAG: порты пока у компаньонов.  [#406] Пишем
     * USBCMD ЧТЕНИЕМ-МОДИФИКАЦИЕЙ: полный перезапись команды сносила
     * биты Interrupt Threshold (биты 23:16, после HCRESET = 0x08):
     * нулевой порог на части реальных чипов ломает механизм IAA. */
    eh_w32(g_op + R_USBSTS, 0x3Fu);   /* R/WC: сброс чужих статусов (IAA!) */
    eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) | CMD_RS | CMD_PSE | CMD_ASE);
    eh_delay(10000);
    if (!(eh_r32(g_op + R_USBSTS) & STS_HALT)) {
        serial_puts("EHCI: контроллер работает (порты ещё у компаньонов)\n");
    } else {
        serial_puts("EHCI: контроллер не завёлся - порты остаются у компаньонов\n");
        eh_diag_set("контроллер не завёлся");
        return false;
    }

    /* CONFIGFLAG=1: порты уходят к нам */
    eh_w32(g_op + R_CONFIGFLAG, 1);
    eh_delay(10000);
    /* «ЭТАП 3»: питание каждому порту с переключаемым питанием (PP=0
     * при включённом порте = порты не отвечают). Дешёвая проверка
     * PORTSC перед первичным опросом. */
    for (uint8_t p = 1; p <= g_nports; p++) {
        uint32_t pa = g_op + R_PORTSC + 4u * (uint32_t)(p - 1);
        uint32_t pv = eh_r32(pa);
        if (!(pv & PS_PP)) {
            eh_w32(pa, (pv & ~(PS_CSC | PS_PEC | PS_OCC)) | PS_PP);
            kdelay_ms(100);              /* F2-8: до 100 мс на порт */
        }
    }
    g_running = true;
    if (g_ours_n < EH_NINST) {
        g_bd_a[g_ours_n][0] = bus;
        g_bd_a[g_ours_n][1] = dev;
        g_bd_a[g_ours_n][2] = fn;
        g_ours_n++;
    }

    /* первичный опрос: подключённые устройства */
    eh_ports_poll(1);

    bool bound = false;
    for (int i = 0; i < EH_NDEV; i++)
        if (g_dv[i].used && g_dv[i].alive) bound = true;
    if (bound) return true;

    /* [#470] ЭТАП 9: ОТКАТ только когда терять НЕЧЕГО — все порты ФИЗИЧЕСКИ
     * пусты (CCS=0 на каждом).  Раньше: первая неудачная попытка
     * перечисления + отсутствие PS/2-клавы = CONFIGFLAG=0 + STOP — и
     * подключённое HS-устройство (флешка 2.0 в переднем порту!) было
     * мертво ДО ПЕРЕЗАГРУЗКИ, а диагностика говорила лишь «устройств
     * не нашёл».  Теперь: есть подключение → контроллер ОСТАЁТСЯ и
     * секундный hotplug-обход пробует ещё (как Linux ehci-hcd, который
     * никогда не выключает контроллер из-за пустого опроса). */
    bool any_conn = false;
    for (uint8_t p = 1; p <= g_nports; p++)
        if (eh_r32(g_op + R_PORTSC + 4u * (uint32_t)(p - 1)) & PS_CCS)
            any_conn = true;

    /* откат: клава в BIOS-эмуляции, мы ничего не нашли, порты ПУСТЫ */
    if (!ps2_kbd_alive() && !force_usb && !any_conn) {
        serial_puts("EHCI: устройств не нашёл, порты пусты - ОТКАТ: порты обратно\n");
        serial_puts("EHCI: компаньонам/BIOS (HS-клава поднимется как FS)\n");
        eh_w32(g_op + R_CONFIGFLAG, 0);
        /* F2-2: порты физически возвращаются компаньону не мгновенно
         * (VIA держит ~50 мс): ждём 100 мс ПРИ РАБОТАЮЩЕМ HC, иначе
         * OHCI-компаньон (инициализируется сразу после ehci_init)
         * зондит порты, ещё висящие на EHCI. */
        kdelay_ms(100);
        eh_w32(g_op + R_USBCMD, eh_r32(g_op + R_USBCMD) & ~CMD_RS);
        t = 20000;
        while (!(eh_r32(g_op + R_USBSTS) & STS_HALT) && t-- > 0) eh_delay(10);
        g_running = false;
        if (g_ours_n > 0) g_ours_n--;
        eh_diag_set("откат (порты пусты): порты компаньонам/BIOS");
        return false;
    }
    if (any_conn)
        serial_puts("EHCI: устройств не нашёл, но на портах ЕСТЬ подключение -\n"
                    "EHCI: ОСТАЮСЬ работать, hotplug попробует снова [#470]\n");
    else
        serial_puts("EHCI: устройств не нашёл (остаюсь работать - hotplug)\n");
    return true;
}

bool ehci_init(int fb_w, int fb_h, int force_usb) {
    /* СМАЗ-защита: флаг «уже запущено» с магией, а не простой int -
     * ОЗУ под статиками до явной инициализации не обязана быть нулём. */
    static uint32_t done_magic;
    const uint32_t  MAGIC_OK = 0x45484332u;   /* "EHC2" (этап 2) */
    if (done_magic == MAGIC_OK) return g_ours_n > 0;
    done_magic = MAGIC_OK;

    /* Явная инициализация ВСЕГО состояния модуля (не полагаемся на BSS). */
    for (int i = 0; i < EH_NINST; i++) {
        g_ei = i;
        g_mm = 0; g_op = 0; g_nports = 0;
        g_running = false;
        g_next_addr = 1;                 /* G2: курсор поиска адресов  */
        memset(g_diag, 0, sizeof(g_diag));
        memset(g_fail, 0, sizeof(g_fail));
        memset(eh_addr_bm, 0, sizeof(eh_addr_bm));
        memset(g_conn, 0, sizeof(g_conn));
        g_rxlen = 0;
        for (int k = 0; k < EH_NDEV; k++) { g_dv[k].used = false; g_dv[k].alive = false; }
    }
    g_ei = 0;
    g_ours_n = 0;
    g_have_companion = false;
    g_ehci_pkts = 0;
    memset(&g_ms, 0, sizeof(g_ms));    /* F2-13: prev-кнопки — в eh_dev_t */

    usb_kbd_init();   /* состояние разбора клавиатуры - с нуля */
    g_fb_w = fb_w; g_fb_h = fb_h;
    g_ms.x = fb_w / 2; g_ms.y = fb_h / 2;
    g_have_companion = eh_scan_companion();
    serial_puts("EHCI: компаньон (OHCI/UHCI) ");
    serial_puts(g_have_companion ? "есть\n" : "не найден\n");

    int n = 0;
    int seen_any = 0;
    for (uint16_t bus = 0; bus < 256 && n < EH_NINST; bus++)
        for (uint8_t dev = 0; dev < 32 && n < EH_NINST; dev++)
            for (uint8_t fn = 0; fn < 8 && n < EH_NINST; fn++) {
                uint32_t id = pci_read32((uint8_t)bus, dev, fn, 0x00);
                if ((id & 0xFFFFu) == 0xFFFFu) {
                    if (fn == 0) break;
                    continue;
                }
                uint32_t cc = pci_read32((uint8_t)bus, dev, fn, 0x08);
                if ((cc >> 24) == 0x0Cu && ((cc >> 16) & 0xFFu) == 0x03u &&
                    ((cc >> 8) & 0xFFu) == 0x20u) {
                    seen_any = 1;
                    /* ФИКС «этапа 2» (п.2): зондируем ВСЕ контроллеры
                     * (до двух), каждый со СВОИМ расписанием — раньше
                     * второй затирал первый и всё ломал. */
                    g_ei = n;
                    if (eh_init_one((uint8_t)bus, dev, fn, force_usb)) {
                        n++;
                    } else {
                        g_mm = 0; g_op = 0;   /* инстанс не поднят     */
                    }
                }
                if (fn == 0 && !(pci_read8((uint8_t)bus, dev, fn, 0x0E) & 0x80u))
                    break;
            }
    g_ei = 0;
    /* 0.3.0: диспетчер знает про EHCI — порты UHCI/OHCI-компаньонов
     * отвечают «USB 2.0» (физический разъём 2.0), без EHCI — «USB 1.x». */
    usbdev_set_ehci_seen(seen_any);
    if (n == 0 && !seen_any) serial_puts("EHCI: контроллеров нет\n");
    if (n == 0 && seen_any)
        serial_puts("EHCI: ни один контроллер не поднялся (порты у компаньонов)\n");
    return g_ours_n > 0;
}

/* 0.3.0: полный обход портов EHCI — каждую секунду из usbdev_tick().
 * «Этап 2»: ОБА инстанса + rescan-попытки ненашых портов. */
void ehci_hotplug_sweep(void) {
    for (int i = 0; i < EH_NINST; i++) {
        if (g_mmio_a[i] == 0) continue;
        g_ei = i;
        eh_ports_poll(0);
        ehci_rescan_unclaimed();
    }
    g_ei = 0;
}

bool ehci_mouse_active(void) {
    for (int i = 0; i < EH_NINST; i++) {
        if (!g_running_a[i]) continue;
        for (int k = 0; k < EH_NDEV; k++)
            if (g_dv_a[i][k].used && g_dv_a[i][k].alive &&
                g_dv_a[i][k].role == 1) return true;
    }
    return false;
}

void ehci_mouse_poll(mouse_state_t *out) {
    g_ms.clicked_left = false;
    g_ms.clicked_right = false;
    g_ms.moved = false;
    for (int i = 0; i < EH_NINST; i++) {
        if (!g_running_a[i]) continue;
        g_ei = i;
        for (int k = 0; k < EH_NDEV; k++)
            if (g_dv_a[i][k].used && g_dv_a[i][k].alive &&
                g_dv_a[i][k].role == 1)
                eh_poll_dev(&g_dv_a[i][k]);
    }
    g_ei = 0;
    if (out) *out = g_ms;
}

void ehci_kbd_poll(void) {
    for (int i = 0; i < EH_NINST; i++) {
        if (!g_running_a[i]) continue;
        g_ei = i;
        for (int k = 0; k < EH_NDEV; k++)
            if (g_dv_a[i][k].used && g_dv_a[i][k].alive &&
                g_dv_a[i][k].role == 2)
                eh_poll_dev(&g_dv_a[i][k]);
    }
    g_ei = 0;
}

bool ehci_kbd_present(void) {
    for (int i = 0; i < EH_NINST; i++)
        for (int k = 0; k < EH_NDEV; k++)
            if (g_dv_a[i][k].used && g_dv_a[i][k].alive &&
                g_dv_a[i][k].role == 2) return true;
    return false;
}

int ehci_is_ours(uint8_t bus, uint8_t dev, uint8_t fn) {
    for (int i = 0; i < g_ours_n; i++)
        if (g_bd_a[i][0] == bus && g_bd_a[i][1] == dev &&
            g_bd_a[i][2] == fn) return 1;
    return 0;
}

/* ── СНИМОК EHCI (для фото, конец экрана) ── */
void ehci_snapshot(void) {
    serial_puts("==== EHCI СНИМОК ====\n");
    bool any = false;
    for (int i = 0; i < EH_NINST; i++) {
        if (g_mmio_a[i] == 0) continue;
        any = true;
        g_ei = i;
        serial_puts(eh_bus_label());
        serial_puts(": cmd=");
        serial_puthex(eh_r32(g_op + R_USBCMD));
        serial_puts(" sts=");
        serial_puthex(eh_r32(g_op + R_USBSTS));
        serial_puts(" cf=");
        serial_puthex(eh_r32(g_op + R_CONFIGFLAG));
        serial_puts(" diag=");
        serial_puts(g_diag);
        serial_puts("\n");
        for (uint8_t p = 1; p <= g_nports; p++) {
            serial_puts("порт ");
            serial_puthex(p);
            serial_puts("=0x");
            serial_puthex(eh_r32(g_op + R_PORTSC + 4u * (uint32_t)(p - 1)));
            serial_puts("\n");
        }
        for (int k = 0; k < EH_NDEV; k++) {
            if (!g_dv[k].used) continue;
            serial_puts("EHCI-устройство: addr=");
            char nb[8];
            serial_puts(itoa(g_dv[k].addr, nb, 10));
            serial_puts(" порт=");
            serial_puts(itoa(g_dv[k].port, nb, 10));
            serial_puts(g_dv[k].role == 1 ? " МЫШЬ" : " КЛАВИАТУРА");
            serial_puts(" vid=");
            serial_puthex(g_dv[k].vid);
            serial_puts(" pid=");
            serial_puthex(g_dv[k].pid);
            serial_puts("\n");
        }
    }
    g_ei = 0;
    if (!any) serial_puts("EHCI: контроллеры не инициализированы\n");
    serial_puts("==== КОНЕЦ EHCI СНИМКА ====\n");
}

/* ═══ [#407] USB-КАРТА ПОРТОВ: блок EHCI ═══════════════════════════════
 * Печатает ВСЕ порты ВСЕХ поднятых инстансов EHCI: тип порта (USB2),
 * скорость устройства по PSPD, хозяин (мы/компаньон), занятость и
 * счётчик неудач перечисления. Вызывается из mouse_hw_portmap() —
 * единая карта по запросу пользователя: «программа ищет все порты и
 * пишет в логе, какой это порт и какой». */
/* [#463] ЕДИНАЯ КАРТА ГНЕЗД: EHCI-часть. Порт EHCI = гнездо USB 2.0
 * (High Speed); FS/LS-устройство в этом же гнезде маршрутизирует САМ
 * ЧИПСЕТ (PORTSC OWNER) компаньону UHCI/OHCI — Linux соответствие
 * портов компаньонов тоже НЕ знает (там ОС просто видит устройство на
 * шине usb1 и показывает его в топологии). N_CC — число контроллеров-
 * компаньонов ЭТОГО EHCI (HCSPARAMS [#432]). [#462] «ЗАНЯТ» печатается
 * со счётчиком живых HID-отчётов. */
void ehci_portmap(int *ctrls, int *lports, int *devs_bound, int *eh_ports,
                  int *sock_no)
{
    for (int i = 0; i < EH_NINST; i++) {
        if (!g_running_a[i] || g_mmio_a[i] == 0) {
            if (g_mmio_a[i] != 0) {
                serial_puts("USB-КАРТА: ── EHCI#");
                portmap_pm_n(i + 1);
                serial_puts(": найден, но не поднят (отдан BIOS/компаньонам) —\n");
                serial_puts("USB-КАРТА:     гнёзда НЕ видны\n");
            }
            continue;
        }
        int saved_ei = g_ei;
        g_ei = i;
        if (ctrls) (*ctrls)++;
        serial_puts("USB-КАРТА: ── EHCI#");
        portmap_pm_n(i + 1);
        serial_puts(" (PCI ");
        portmap_pm_n((int)g_bd_a[i][0]);
        serial_puts(":");
        portmap_pm_n((int)g_bd_a[i][1]);
        serial_puts(".");
        portmap_pm_n((int)g_bd_a[i][2]);
        serial_puts("): портов ");
        portmap_pm_n((int)g_nports_a[i]);
        serial_puts(", компаньонов ");
        portmap_pm_n((int)g_ncc_a[i]);      /* [#432] честный N_CC этого инстанса */
        serial_puts(" (FS/LS линии этих же гнёзд)\n");
        for (uint8_t p = 1; p <= g_nports_a[i] && p <= 15; p++) {
            uint32_t pv = eh_r32(g_op + R_PORTSC + 4u * (uint32_t)(p - 1));
            if (lports)    (*lports)++;
            if (eh_ports)  (*eh_ports)++;
            /* [#472] СКВОЗНОЙ № гнезда + PCI-адрес в каждой строке:
             * «порт 4» у EHCI#1 и «порт 4» у EHCI#2 — РАЗНЫЕ гнёзда. */
            serial_puts("USB-КАРТА: ГНЕЗДО №");
            if (sock_no)   portmap_pm_n(++(*sock_no));
            else           portmap_pm_n((int)p);
            serial_puts(" [EHCI#");
            portmap_pm_n(i + 1);
            serial_puts(" PCI ");
            portmap_pm_n((int)g_bd_a[i][0]);
            serial_puts(":");
            portmap_pm_n((int)g_bd_a[i][1]);
            serial_puts(".");
            portmap_pm_n((int)g_bd_a[i][2]);
            serial_puts(" порт ");
            portmap_pm_n((int)p);
            serial_puts("]: макс USB 2.0 | линия: High Speed 480 Мбит/с,\n");
            serial_puts("USB-КАРТА:     FS/LS — через компаньона (N_CC=");
            portmap_pm_n((int)g_ncc_a[i]);
            serial_puts(")\n");
            if (pv & PS_PO) {
                serial_puts("USB-КАРТА:   состояние: у КОМПАНЬОНА — FS/LS-устройство;\n");
                serial_puts("USB-КАРТА:     ОС увидит его на порту OHCI/UHCI ниже (в следующей\n");
                serial_puts("USB-КАРТА:     секции карты). Маршрутизацию\n");
                serial_puts("USB-КАРТА:     делает ЧИПСЕТ: соответствие портов компаньонов без\n");
                serial_puts("USB-КАРТА:     ACPI не знает и Linux\n");
                continue;
            }
            if (!(pv & PS_CCS)) {
                serial_puts("USB-КАРТА:   состояние: пусто\n");
                continue;
            }
            /* устройство есть: чьё? */
            int role = 0;
            for (int k = 0; k < EH_NDEV; k++)
                if (g_dv_a[i][k].used && g_dv_a[i][k].port == p &&
                    g_dv_a[i][k].alive)
                    role = (int)g_dv_a[i][k].role;
            uint32_t spd = (pv >> 26) & 3u;
            serial_puts("USB-КАРТА:   состояние: ");
            serial_puts(spd == 2 ? "high-speed 480 Мбит/с" :
                        (spd == 0 ? "full-speed 12 Мбит/с" : "low-speed 1.5 Мбит/с"));
            if (role == 1) {
                serial_puts(" — МЫШЬ");
                portmap_pm_alive(USBDEV_ROLE_MOUSE);
                if (devs_bound) (*devs_bound)++;
            } else if (role == 2) {
                serial_puts(" — КЛАВИАТУРА");
                portmap_pm_alive(USBDEV_ROLE_KBD);
                if (devs_bound) (*devs_bound)++;
            } else if (g_fail_a[i][p] >= 3) {
                serial_puts(", устройство ЕСТЬ, перечисление НЕ удалось (");
                portmap_pm_n((int)g_fail_a[i][p]);
                serial_puts(" попыток) - жду переподключения");
            } else {
                serial_puts(", устройство ЕСТЬ, перечисляю...");
            }
            serial_puts("\n");
        }
        g_ei = saved_ei;
    }
}

/* [#446] LED EHCI-клавиатуры: SET_REPORT(output) по текущему состоянию
 * NumLock/CapsLock. Вызывается диспетчером usb_kbd_leds_all() при каждом
 * тумблере — раньше индикаторы EHCI-клавы не менялись НИКОГДА. */
void ehci_kbd_set_leds(uint8_t hid_leds) {
    int saved_ei = g_ei;
    for (int i = 0; i < EH_NINST; i++) {
        if (!g_running_a[i]) continue;
        g_ei = i;
        for (int k = 0; k < EH_NDEV; k++) {
            if (!g_dv[k].used || !g_dv[k].alive || g_dv[k].role != 2) continue;
            (void)eh_ctrl_dev(&g_dv[k], g_dv[k].mps0, 0x21, 0x09 /* SET_REPORT */,
                              0x0200u, g_dv[k].iface, 1, &hid_leds);
        }
    }
    g_ei = saved_ei;
}
