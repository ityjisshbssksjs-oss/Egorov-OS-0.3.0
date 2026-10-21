/*
 * USB HID Optical Mouse driver for Egorov OS — v1.5
 *
 * v1.5 (QEMU usb-tablet: «курсор быстро летает вниз и дёргается»):
 *   • Причина: usb-tablet шлёт АБСОЛЮТНЫЕ 6-байтные отчёты (btn, X 16-bit
 *     LE, Y 16-bit LE, wheel), а poll парсил КАЖДЫЙ отчёт как boot
 *     relative (dX=байт1, dY=байт2): старший байт X (0x40 в центре экрана)
 *     трактовался как dY=+128 на ~1000 отчётов/с → курсор улетал вниз,
 *     прижимался к краю и дёргался.  «Выход из захвата» — штатное
 *     поведение QEMU для absolute-устройств (UI не грабит курсор).
 *   • Авто-детект режима: после SET_CONFIG читается HID report descriptor
 *     (class GET_DESCRIPTOR 0x2200, длина из HID-дескриптора конфига,
 *     буфер um_rpt[192]); минимальный парсер main/global item'ов: INPUT
 *     item Data|Var|Absolute (val&7==0x02) с Report Size 16 →
 *     absolute-режим (планшет).  Fallback при неудаче — по
 *     bInterfaceSubClass (нет boot-подкласса → планшет).
 *   • Масштабирование логического максимума (QEMU: 32767, парсится
 *     LOGICAL_MAX) на весь framebuffer → курсор ОС следует за курсором
 *     хоста 1:1.
 *   • um_handle_tablet_report(): btn=байт0, X=байты1-2 LE, Y=байты3-4 LE
 *     (колесо игнорируется); кнопки через общий um_apply_buttons()
 *     (edge-детект, как в PS/2).
 *   • SET_PROTOCOL(boot) отправляется только boot-совместимым
 *     интерфейсам; SET_IDLE — всем.
 *   • um_control(): memset(um_buf) для IN-стадии — детерминированный
 *     хвост при short packet.
 *   • Порог парсинга в poll: absolute требует act>=5, relative — act>=3.
 *
 * v1.4 (first control transfer failed without a reason code):
 *   • SeaBIOS-style control pipe: ONE QH armed once and left scheduled
 *     for the whole enumeration (no more framelist churn between
 *     stages).  Each stage builds its TD in a FRESH rotating slot — a
 *     TD address is never rewritten while a stale async reference
 *     could still point at it.
 *   • Full failure diagnostics: timeout vs error bits, the raw TD
 *     status, plus USBCMD/USBSTS on timeouts.
 * v1.3 fix (responses lagged one transfer behind):
 *   • Hex dumps proved the device answered each control with the
 *     PREVIOUS request's response: the whole TD chain (SETUP → DATA →
 *     STATUS) executed inside one USB frame and the device only
 *     committed the setup after the IN stage had already read stale
 *     data.  Fix: run every stage SEPARATELY — one TD in the schedule
 *     at a time, each completing (and settling) before the next stage
 *     is armed.  Slow (~6 ms per control) but bulletproof.
 *   • No TD chaining any more: the QH element link is re-pointed at
 *     each single-TD stage.
 * v1.2 fix (wrong descriptors during enumeration):
 *   • Every control transfer reused ONE static QH address.  Hosts that
 *     key internal queue state on the QH address (QEMU does) can merge
 *     a new transfer into a stale queue — the device then saw a previous
 *     setup packet and answered with the wrong descriptor (9-byte config
 *     / HID blob instead of the 18-byte device descriptor).  Fixed by
 *     rotating control slots: a fresh QH + a fresh setup-packet buffer
 *     for every transfer, plus a 2 ms settle gap between transfers.
 *   • Added hex dumps of every descriptor read to the serial log.
 * v1.1 fixes (enumeration was silently timing out):
 *   • The QH was scheduled ONLY in frame-list slot 0, but UHCI reads the
 *     slot indexed by the running FRNUM — slot 0 comes around once per
 *     1024 frames (~1.024 s), while control transfers had a 500 ms
 *     timeout → enumeration failed on every attempt.  Now the QH is
 *     scheduled in ALL 1024 slots (true ~1 kHz scheduling).
 *   • TD→TD links now use the depth-first (Vf) bit so a whole control
 *     transfer completes inside one frame.
 *   • Added step-by-step serial diagnostics for port reset / enumeration.
 *   • usb_mouse_poll() hardened: a TD retired with the NAK bit (some
 *     hosts retire instead of keeping it active) is no longer parsed
 *     as a report and does not flip the data toggle.
 *
 * Implements a polled USB optical mouse using the HID boot protocol on top
 * of a UHCI host controller.  The driver is fully self-contained (own PCI
 * scan, frame list, TD/QH pools) so it does not depend on usb.c — when no
 * mass-storage device is enumerated (usb_init() is currently disabled in
 * kernel.c) the first UHCI controller is free for the mouse.
 *
 * Supports:
 *   • PCI config-space scan for UHCI (class 0x0C, subclass 0x03, prog-if 0x00)
 *   • USB device reset + enumeration (control pipe, polling)
 *   • HID class requests: SET_PROTOCOL(boot) + SET_IDLE(0)
 *   • Persistent interrupt-IN QH scheduled in every frame slot (~1 kHz
 *     polling) — zero CPU cost while the mouse is idle (device NAKs, TD
 *     stays active)
 *   • Boot-protocol reports: [0]=buttons, [1]=dX, [2]=dY (wheel ignored)
 *   • v1.5: ABSOLUTE reports (usb-tablet): [0]=buttons, [1:2]=X,
 *     [3:4]=Y (16-bit LE), scaled by LOGICAL_MAX onto the framebuffer
 *
 * Architecture: 32-bit protected mode, no IRQs (polled), no dynamic memory
 * (all structures in BSS/static).  Tested with QEMU (-device usb-mouse on
 * the default PIIX3 UHCI).
 *
 * Limitations (demo-OS scope):
 *   - Only the first UHCI controller is used
 *   - Only one HID device (best boot-mouse interface) is enumerated
 *   - Cannot share a controller with an already-enumerated USB drive
 *   - Wheel / middle button are ignored (mouse_state_t has 2 buttons)
 *   - If the schedule dies (watchdog) or errors repeat, the driver
 *     deactivates itself and the PS/2 driver takes over automatically
 *     via ps2_poll().
 */

#include "usb_mouse.h"
#include "usbmap.h"
#include "usbdev.h"
#include "io.h"
#include "string.h"
#include "timer.h"      /* kdelay_us(): PIT-калиброванные задержки ( um_delay() ) */

/* COM1 logging is implemented in kernel.c (used for QEMU -serial stdio). */
extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);
extern char *itoa(uint32_t v, char *buf, int base);
/* Ядро UHCI-клавиатуры (uhci_kbd.c): порт привязанной клавиатуры на
 * данном контроллере — um_quiesce_others() его НЕ гасит (аудит #234),
 * um_probe_controller() цепляет её QH за своим (общее расписание). */
extern int uhci_kbd_bound_port(uint16_t io);
extern uint32_t uhci_kbd_int_qh_link(void);

/* Hex dump helper (diagnostics). */
static void um_dump(const char *tag, const uint8_t *buf, int n) {
    serial_puts("USBMOUSE: "); serial_puts(tag); serial_puts(" =");
    for (int i = 0; i < n; i++) { serial_puts(" "); serial_puthex(buf[i]); }
    serial_puts("\n");
}

/* ── PCI config-space access (I/O CF8/CFC mechanism) ── */
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

/* ── UHCI register offsets (I/O base relative) ── */
#define UHCI_CMD    0x00u   /* USBCMD  (16-bit) */
#define UHCI_STS    0x02u   /* USBSTS  (16-bit) */
#define UHCI_INTR   0x04u   /* USBINTR (16-bit) */
#define UHCI_FRNUM  0x06u   /* FRNUM   (16-bit) */
#define UHCI_FLBASE 0x08u   /* FLBASEADD (32-bit) */
#define UHCI_PORT0  0x10u   /* PORTSC0 (16-bit) */
#define UHCI_PORT1  0x12u   /* PORTSC1 (16-bit) */

/* PORTSC bits */
#define UHCI_PORT_CONNECT  (1u << 0)
#define UHCI_PORT_CCHANGE  (1u << 1)
#define UHCI_PORT_ENABLE   (1u << 2)
#define UHCI_PORT_ECHANGE  (1u << 3)
#define UHCI_PORT_LSD      (1u << 8)   /* Low-Speed Device attached (RO) */
#define UHCI_PORT_RESET    (1u << 9)

/* USBCMD bits */
#define UHCI_CMD_RS      (1u << 0)
#define UHCI_CMD_HCRESET (1u << 1)
#define UHCI_CMD_GRESET  (1u << 2)
/* «ЭТАП 3» (критично для реального железа): без Configure Flag UHCI
 * считается «не сконфигурированным» — маршрутизация портов/ответы
 * HC не гарантированы.  SeaBIOS/Linux/наш же клавиатурный драйвер
 * (uhci_core.c:246) ставят RS|CF|MAXP.  QEMU бит CF не моделирует —
 * поэтому в эмуляторе всё работало и без него. */
#define UHCI_CMD_CF      (1u << 6)
#define UHCI_CMD_MAXP    (1u << 7)

/* ── USB request constants ── */
#define USB_REQ_GET_STATUS      0x00u
#define USB_REQ_CLEAR_FEATURE   0x01u
#define USB_REQ_SET_ADDRESS     0x05u
#define USB_REQ_GET_DESCRIPTOR  0x06u
#define USB_REQ_SET_CONFIG      0x09u
#define USB_REQ_SET_INTERFACE   0x0Bu

#define USB_HID_SET_IDLE        0x0Au   /* class request (bmReqType 0x21) */
#define USB_HID_SET_PROTOCOL    0x0Bu   /* class request (bmReqType 0x21) */

#define USB_DT_DEVICE    0x01u
#define USB_DT_CONFIG    0x02u
#define USB_DT_STRING    0x03u
#define USB_DT_HID       0x21u
#define USB_DT_REPORT    0x22u

/* Descriptor types (walking the config blob) */
#define DESC_INTERFACE 0x04u
#define DESC_ENDPOINT  0x05u

/* ── UHCI Transfer Descriptor (TD) — 32 bytes, 16-byte aligned ── */
typedef struct __attribute__((packed)) {
    uint32_t link;      /* link pointer */
    uint32_t ctrl_sts;  /* control/status */
    uint32_t token;     /* PID, address, endpoint, length */
    uint32_t buffer;    /* buffer pointer */
    uint32_t _reserved[4];
} uhci_td_t;

/* ── UHCI Queue Head — 16 bytes, 16-byte aligned ── */
typedef struct __attribute__((packed)) {
    uint32_t head_link;   /* horizontal link */
    uint32_t elem_link;   /* element (first TD) */
    uint32_t _reserved[2];
} uhci_qh_t;

/* TD control/status bits */
#define TD_CTRL_ACTIVE  (1u << 23)
#define TD_CTRL_IOC     (1u << 24)
#define TD_CTRL_LS      (1u << 26)  /* Low-speed */
#define TD_CTRL_3ERRS   (3u << 27)
#define TD_CTRL_SPD     (1u << 29)  /* Short Packet Detect */
#define TD_CTRL_STALLED (1u << 22)
#define TD_CTRL_DBUFERR (1u << 21)
#define TD_CTRL_BABBLE  (1u << 20)
#define TD_CTRL_NAK     (1u << 19)  /* read-only status bit */
#define TD_CTRL_CRCERR  (1u << 18)
#define TD_CTRL_ANY_ERR (TD_CTRL_STALLED | TD_CTRL_DBUFERR | \
                         TD_CTRL_BABBLE  | TD_CTRL_CRCERR)

#define PID_SETUP  0x2Du
#define PID_IN     0x69u
#define PID_OUT    0xE1u

/* Token: pid[7:0] | devaddr[14:8] | endpoint[18:15] | D-toggle[19] | maxlen[31:21] */
#define TD_TOKEN(pid, addr, ep, tog, len) \
    ((uint32_t)(pid) | ((uint32_t)(addr) << 8) | ((uint32_t)(ep) << 15) | \
     ((tog) ? (1u << 19) : 0u) | (((uint32_t)(len) ? (uint32_t)(len) - 1u : 0x7FFu) << 21))

/* ── Frame list — 1024 entries, 4096-byte aligned ── */
static uint32_t  um_frame_list[1024] __attribute__((aligned(4096)));

/* Control-transfer SETUP packets rotate so a stale async reference can
 * never observe rewritten bytes. */
#define UM_CSLOTS 4
static uint8_t   um_csetup[UM_CSLOTS][8] __attribute__((aligned(4)));
static uint8_t   um_cslot;

/* Control pipe (v1.4, SeaBIOS-style): ONE QH, permanently scheduled for
 * the whole enumeration; every stage builds its TD in a FRESH rotating
 * slot — a TD address is never rewritten while a stale async reference
 * could still point at it. */
static uhci_qh_t um_ctrl_qh __attribute__((aligned(16)));
/* «ЭТАП 5 / #191-аналог»: 8 слотов при EP0 MPS=8 хватало лишь на
 * wLen<=56 (1 SETUP + 7 данных + 1 статус) — конфиг-дескриптор 57..255 Б
 * НЕ читался вовсе (um_control молча проваливался на середине дата-стадии).
 * 36 слотов покрывают максимум: 1 + ceil(255/8)=32 + 1 = 34 TD. */
#define UM_CTD 36
static uhci_td_t um_ctds[UM_CTD] __attribute__((aligned(16)));
static uint8_t   um_ctd_i;

/* Dedicated persistent QH + TD for the interrupt-IN endpoint. */
static uhci_td_t um_itd __attribute__((aligned(16)));
static uhci_qh_t um_qh  __attribute__((aligned(16)));

/* Transfer buffers.  «ЭТАП 3»: выравнивание ≥256 при длине ≤256 —
 * TD-буфер UHCI не имеет права пересекать границу 4К-страницы. */
static uint8_t um_buf[256] __attribute__((aligned(256)));  /* control data   */
static uint8_t um_pkt[8]   __attribute__((aligned(64)));   /* interrupt-IN report */

/* ── Driver state ── */
static uint16_t g_io;         /* UHCI I/O base */
static uint8_t  g_um_port_cur = 0;  /* порт текущего перечисления (0.3.0) */
/* «ЭТАП 5 / #232/#233» (addr-leak): адреса больше НЕ счётчик-инкремент —
 * после ~127 переподключений он переполнялся в адрес 0 (broadcast!).
 * Теперь битмапа: бит n = USB-адрес n занят.  Адрес 2 зарезервирован за
 * клавиатурным драйвером (uhci_ctl.c).  Выдача — um_addr_alloc(),
 * возврат — при провале перечисления и при деактивации (um_addr_free). */
#define UM_ADDR_MAX 127
static uint8_t   g_um_addr_bm[(UM_ADDR_MAX + 8) / 8];
static uint8_t   g_um_addr_used;    /* адрес, который держит живая мышь (0 = нет) */
static uint8_t  g_port = 0xFF;/* корневой порт, на котором сидит мышь */
static uint8_t  g_devaddr;    /* assigned USB device address */
static uint8_t  g_ep;         /* interrupt-IN endpoint number */
static uint8_t  g_iface;      /* HID interface number */
static uint8_t  g_mps;        /* interrupt-IN max packet size */
static uint8_t  g_toggle;     /* expected DATA0/DATA1 on interrupt EP */
static bool     g_present;    /* device enumerated */
static bool     g_active;     /* USB mouse is the active pointer */
static bool     g_alive_seen; /* HC proved alive (NAK seen or TD retired) */
/* низкоскоростное устройство на порту (PORTSC.LSD): LS-транзакции
 * требуют бита LS в TD (бит 26) и таймингов 1.5 Мбит/с - без него
 * full-speed TD на LS-мыши не получает НИ ОДНОГО ответа (timeout/NAK).
 * Дешёвые USB 1.1-мыши почти всегда low-speed!
 * Аудит #235 (обоснование, НЕ баг): скорость — глобал, а не поле
 * структуры, потому что драйвер обслуживает РОВНО ОДНО устройство на
 * ОДНОМ контроллере (модель «один драйвер = один слот реестра usbdev»);
 * второй мыши здесь появиться не может. */
static bool     g_ls;
static uint32_t g_watchdog;   /* polls with TD active and no life sign */
static uint32_t g_errors;     /* consecutive TD errors */
static bool     g_first_pkt;  /* 0.3.2: первый отчёт - в журнал (диагностика) */
/* «ЭТАП 5 / #234»: бит-маска портов, приглушённых um_quiesce_others();
 * их возвращает um_restore_muted() в конце пробы контроллера. */
static uint8_t  um_muted;

/* ── Разделение контроллера с драйвером клавиатуры (usb_uhci_kbd.c) ──
 * Клавиатурный драйвер подцепляет свой QH прерываний горизонтальной
 * ссылкой за QH мыши (um_qh.head_link). um_arm() обязан СОХРАНЯТЬ эту
 * ссылку при каждом переарме TD мыши, поэтому значение живёт здесь,
 * а не в памяти чужого драйвера. 1 = за QH мыши никого нет. */
static uint32_t g_sched_chain = 1u;

/* v1.5: report mode — relative (boot mouse) vs absolute (tablet). */
static bool     g_absolute;   /* device sends absolute X/Y (tablet)   */
static bool     g_boot_iface; /* interface has the HID boot subclass  */
static uint16_t g_log_max;    /* LOGICAL_MAXIMUM of the absolute axes */
static uint8_t  um_rpt[192] __attribute__((aligned(256))); /* HID report descriptor (TD без пересечения 4К) */

/* «ЭТАП 3»: EP0 max packet size устройства (bMaxPacketSize0, byte 7
 * дескриптора).  Дефолт 8 — безопасен для LS; уточняется после
 * GET_DESCRIPTOR(8).  Чанкование дата-стадии в um_control. */
static uint8_t g_ep0_mps = 8;

/* «ЭТАП 5 / #204/#294»: клампер bMaxPacketSize0 по скорости устройства:
 * LS -> строго 8; FS -> 8/16/32/64; мусор (0,1,3,...) -> безопасные 8.
 * Копия uhci_mps_fix() из uhci_hw.h — включать сюда заголовок
 * клавиатурного стека нельзя: конфликт локальных TD/QH-структур
 * (там volatile 16-байтные, здесь packed 32-байтные). */
static inline uint8_t uhci_mps_fix(uint8_t mps, int ls)
{
    if (ls) return 8;
    if (mps == 8 || mps == 16 || mps == 32 || mps == 64) return mps;
    return 8;
}

static int           g_fb_w, g_fb_h;
static mouse_state_t g_mouse;
static bool          prev_left, prev_right;

/* ── 0.3.2: hotplug-перескан UHCI («мышь воткнули после загрузки или в
 * порт 2.0, отданный EHCI компаньону позже init-скана»). Список всех
 * найденных при init UHCI-контроллеров + контроллер, которым сейчас
 * владеет драйвер клавиатуры (usb_uhci_kbd.c): его не трогаем. */
static uint16_t g_scan_io[4];
static int      g_scan_n;
static uint16_t g_kbd_io;
/* [LOGS] ЭТАП 11: UHCI, занятый mass-storage (usb.c, режим sticklog).
 * rescan мыши не трогает его — раньше хотелплуг-скан заново перечислял
 * флешку (mute/restore порта) и убивал живой MSD-адрес: выгрузка
 * журнала отваливалась «транспортной ошибкой» x3. */
static uint16_t g_msd_io;
static uint32_t g_rescan_cnt;


/* ── Busy delay: PIT-калиброванная (kernel/timer.c; до timer_init()
 * работает консервативный фолбэк — не хуже прежнего цикла inb(0x80)). ── */
static void um_delay(uint32_t us) {
    kdelay_us(us);
}

/* ── «ЭТАП 5 / #232/#233»: alloc/free USB-адресов (битмапа g_um_addr_bm) ── */
static int um_addr_alloc(void)
{
    for (int a = 1; a <= UM_ADDR_MAX; a++) {
        uint8_t m = (uint8_t)(1u << (a & 7));
        if (!(g_um_addr_bm[a >> 3] & m)) {
            g_um_addr_bm[a >> 3] |= m;
            return a;
        }
    }
    return -1;
}

static void um_addr_free(uint8_t addr)
{
    if (addr == 0 || addr > UM_ADDR_MAX)
        return;
    g_um_addr_bm[addr >> 3] &= (uint8_t)~(uint8_t)(1u << (addr & 7));
}

/* Полный сброс битмапы (usb_mouse_init): адрес 2 снова за клавиатурой. */
static void um_addr_release_all(void)
{
    memset(g_um_addr_bm, 0, sizeof g_um_addr_bm);
    g_um_addr_bm[2 >> 3] |= (uint8_t)(1u << (2 & 7));
}

/* Вернуть адрес, который держит (или пыталась получить) мышь. */
static void um_addr_release_used(void)
{
    if (g_um_addr_used) {
        um_addr_free(g_um_addr_used);
        g_um_addr_used = 0;
    }
}

/* ── UHCI low-level helpers ── */
static void uhci_write16(uint16_t off, uint16_t v) { outw((uint16_t)(g_io + off), v); }
static void uhci_write32(uint16_t off, uint32_t v) { outl((uint16_t)(g_io + off), v); }

/* Point every frame-list slot at the same entry.
 * v1.1: slot 0 alone is fetched only when FRNUM wraps (once per 1024
 * frames ≈ 1.024 s) — that made 500 ms control transfers time out.
 * Scheduling in all slots gives a true ~1 kHz execution rate. */
static void um_dma(void);   /* [#428] wbinvd — определена ниже */
static void um_sched_all(uint32_t entry) {
    for (int i = 0; i < 1024; i++) um_frame_list[i] = entry;
    um_dma();   /* [#428] список кадров — в ОЗУ до включения в шедулинг */
}

/* PORTSC биты — см. выше; регистр порта n: 0x10 + 2*n */
#define UHCI_PORT_REG(n)  ((uint16_t)(UHCI_PORT0 + 2u * (uint32_t)(n)))

/* «ЭТАП F1»: число корневых портов UHCI.  Регистра числа портов у UHCI
 * НЕТ — порты сканируют до первого несуществующего PORTSC (так делает
 * Linux uhci-hcd): у PIIX3 их 2, но некоторые контроллеры имеют 4 —
 * раньше порты 3-4 пропадали молча.
 * Два признака несуществующего порта: 0xFFFF (реальное железо) и
 * взведённые reserved-биты 13-15 (QEMU piix3 возвращает 0xFF7F с
 * битом CCS=1 — фантомные «устройства» отравляли рескан и уступку
 * клавиатуры; по спеке UHCI эти биты всегда читаются 0). */
#define UHCI_SCAN_PORTS 4
#define UHCI_PORT_RSVD_MASK 0xE000u   /* reserved-биты 13-15: только 0 */

static int um_port_count(uint16_t io)
{
    int n = 0;
    while (n < UHCI_SCAN_PORTS) {
        uint16_t st = inw((uint16_t)(io + UHCI_PORT_REG(n)));
        if (st == 0xFFFFu || (st & UHCI_PORT_RSVD_MASK) != 0u)
            break;
        n++;
    }
    return n;
}

static void uhci_reset(void) {
    uhci_write16(UHCI_CMD, UHCI_CMD_HCRESET);
    um_delay(50000);
    uhci_write16(UHCI_CMD, 0);
    /* спецификация UHCI: бит HCRESET снимает сам контроллер; запись 0
     * в USBCMD во время ресета на части чипов игнорируется - ждём
     * фактического снятия, иначе следующий USBCMD_RS теряется. */
    int t = 1000;
    while ((inw((uint16_t)(g_io + UHCI_CMD)) & UHCI_CMD_HCRESET) && t-- > 0)
        um_delay(1000);
}

static void uhci_init_framelist(void) {
    /* All frame-list entries point to the terminate bit (nothing scheduled). */
    for (int i = 0; i < 1024; i++) um_frame_list[i] = 1; /* Terminate */
    uhci_write32(UHCI_FLBASE, (uint32_t)um_frame_list);
    uhci_write16(UHCI_FRNUM, 0);
    uhci_write16(UHCI_INTR, 0);          /* disable all interrupts */
    uhci_write16(UHCI_STS, 0x3F);        /* clear status bits */
    /* «ЭТАП 3»: RS|CF|MAXP — см. комментарий у UHCI_CMD_CF. */
    uhci_write16(UHCI_CMD, UHCI_CMD_RS | UHCI_CMD_CF | UHCI_CMD_MAXP);
    um_delay(1000);
}

/* ── Reset + enable a port; returns true if a device attached & enabled ── */
static bool uhci_port_reset(uint16_t port_reg, int portno) {
    uint16_t s = inw((uint16_t)(g_io + port_reg));
    if (!(s & UHCI_PORT_CONNECT)) return false;

    /* «ЭТАП 3»: дебаунс подключения (Linux hub.c: ~100 мс стабильности)
     * ДО сброса: дребезг контактов срывал перечисление на пустом месте.
     * (Клавиатурный драйвер uhci_core.c делает так с самого начала.) */
    {
        uint16_t s1 = inw((uint16_t)(g_io + port_reg));
        um_delay(100000);                          /* 100 мс */
        uint16_t s2 = inw((uint16_t)(g_io + port_reg));
        if (((s1 ^ s2) & UHCI_PORT_CONNECT) != 0) {
            serial_puts("USBMOUSE: порт ");
            serial_puthex((uint32_t)portno);
            serial_puts(": дребезг подключения - пропускаю\n");
            return false;
        }
        if (!(s2 & UHCI_PORT_CONNECT)) return false;
    }

    /* Clear change bits, assert RESET. */
    outw((uint16_t)(g_io + port_reg), UHCI_PORT_CCHANGE | UHCI_PORT_ECHANGE | UHCI_PORT_RESET);
    um_delay(60000);  /* USB spec: ≥50 ms */
    outw((uint16_t)(g_io + port_reg), 0);
    /* «ЭТАП 3»: recovery после сброса ≥50 мс (TRSTRCY; ранее было 5 мс —
     * реальное устройство ещё не готово к первому GET_DESCRIPTOR,
     * срывалось перечисление).  Как в собственном EHCI-драйвере. */
    um_delay(50000);

    /* Enable the port. */
    outw((uint16_t)(g_io + port_reg), UHCI_PORT_ENABLE | UHCI_PORT_CCHANGE | UHCI_PORT_ECHANGE);
    um_delay(10000);
    s = inw((uint16_t)(g_io + port_reg));
    /* скорость устройства из PORTSC.LSD (бит 8): решает LS-бит TD */
    g_ls = (s & UHCI_PORT_LSD) != 0;
    serial_puts("USBMOUSE: port ");
    serial_puthex((uint32_t)portno);
    serial_puts(" after reset: sts=");
    serial_puthex(s);
    serial_puts(g_ls ? " [low-speed]" : " [full-speed]");
    serial_puts("\n");
    return (s & UHCI_PORT_ENABLE) && (s & UHCI_PORT_CONNECT);
}

/* ── «ЭТАП 5 / #234»: возврат приглушённых портов ─────────────────────
 * PE после записи 0 возвращается только сбросом порта — короткий
 * uhci_port_reset снова включит порт (устройство остаётся достижимым
 * для последующих проб: клавиатурный драйвер, MSD usb.c).  Порты без
 * устройства не дёргаем.  Скорость МЫШИ (g_ls) сохраняем: uhci_port_reset
 * соседа перезаписал бы её LSD-битом чужого устройства. */
static void um_restore_muted(void)
{
    bool saved_ls = g_ls;

    if (!um_muted || g_io == 0)
        return;
    for (int p = 0; p < UHCI_SCAN_PORTS; p++) {
        if (!(um_muted & (1u << p)))
            continue;
        if (p == uhci_kbd_bound_port((uint16_t)g_io))
            continue;         /* не мы гасили — на всякий случай */
        if (!(inw((uint16_t)(g_io + UHCI_PORT_REG(p))) & UHCI_PORT_CONNECT))
            continue;         /* устройство выдернули — порт не дёргаем */
        serial_puts("USBMOUSE: возвращаю приглушённый порт ");
        { char nb[8]; serial_puts(itoa((uint32_t)p, nb, 10)); }
        serial_puts(" сбросом\n");
        (void)uhci_port_reset(UHCI_PORT_REG(p), p);
    }
    um_muted = 0;
    g_ls = saved_ls;
}

/* ── Control pipe helpers (v1.4) ── */
static uhci_td_t *um_next_td(void) {
    uhci_td_t *td = &um_ctds[um_ctd_i];
    /* «ЭТАП 5»: UM_CTD больше не степень двойки — вращение по модулю */
    um_ctd_i = (uint8_t)((um_ctd_i + 1u) % UM_CTD);
    return td;
}

/* компиляторный барьер: поля TD/QH пишутся обычными сторами, и без
 * этого GCC -O2 мог поставить запись elem_link (которая ВООРУЖАЕТ TD
 * для HC) РАНЬШЕ токена/буфера — контроллер видел ACTIVE с мусором. */
static void um_barrier(void) { __asm__ volatile("" ::: "memory"); }

/* [#428] кэш-когерентность UHCI-мыши: собственный движок um_* жил БЕЗ
 * единого wbinvd (#403 закрыл только uhci_core.c/uhci_kbd.c — клаву).
 * На реальном железе с write-back кэшем контроллер читает TD/QH/frame-
 * list из ОЗУ со СТАРЫМ мусором, а опрос видит застоявшуюся строку
 * ctrl_sts → «TD timeout»/деактивация.  wbinvd = запись кэша в ОЗУ +
 * инвалидация — закрывает ОБА направления (мы пишем структуры, HC
 * пишет токены/данные). */
static void um_dma(void) { __asm__ volatile("wbinvd" ::: "memory"); }

/* Point the control QH at one TD and wait for it to retire.
 * Full diagnostics on timeout / error.
 * «ЭТАП 3»: NAK-завершение TD БОЛЬШЕ НЕ считается успехом — на
 * реальном UHCI NAK ретайтит TD (uhci_hw.h это прямо документирует),
 * и стадия «успешно» возвращалась БЕЗ данных.  Теперь TD переармливается
 * и ожидание продолжается в рамках общего бюджета (как uhci_core.c). */
static bool um_run_td(volatile uhci_td_t *td, const char *tag) {
    um_barrier();                       /* токен/буфер уже в памяти */
    um_dma();                           /* [#428] и В ОЗУ, а не в кэше */
    um_ctrl_qh.elem_link = (uint32_t)td;
    um_barrier();
    um_dma();
    int timeout = 500; /* 500 ms max */
    int nak_rearm = 0;
    for (;;) {
        um_dma();                       /* [#428] увидеть запись HC в токен */
        if (!(td->ctrl_sts & TD_CTRL_ACTIVE)) break;
        if (timeout-- <= 0) break;
        um_delay(1000);
        /* TD ушёл из ACTIVE с битом NAK: HC сам снял ACTIVE —
         * переармливаем и ждём дальше (устройство просто не готово). */
        if (!(td->ctrl_sts & TD_CTRL_ACTIVE) &&
            (td->ctrl_sts & TD_CTRL_NAK) &&
            !(td->ctrl_sts & TD_CTRL_ANY_ERR) &&
            nak_rearm < 200) {
            nak_rearm++;
            td->ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS |
                           (g_ls ? TD_CTRL_LS : 0u);
            um_barrier();
            um_dma();                   /* [#428] */
            um_ctrl_qh.elem_link = (uint32_t)td;
            um_barrier();
            um_dma();
        }
    }
    uint32_t sts = td->ctrl_sts;
    um_dma();                           /* [#428] финальное состояние — из ОЗУ */
    sts = td->ctrl_sts;
    if (sts & TD_CTRL_ACTIVE) {
        serial_puts("USBMOUSE: TD timeout ("); serial_puts(tag);
        serial_puts(") sts="); serial_puthex(sts);
        serial_puts(" cmd="); serial_puthex(inw((uint16_t)g_io));
        serial_puts(" usbsts="); serial_puthex(inw((uint16_t)(g_io + 2)));
        serial_puts("\n");
        return false;
    }
    if (sts & TD_CTRL_ANY_ERR) {
        serial_puts("USBMOUSE: TD error ("); serial_puts(tag);
        serial_puts(") sts="); serial_puthex(sts);
        serial_puts("\n");
        return false;
    }
    if (sts & TD_CTRL_NAK) {
        /* «ЭТАП 3»: NAK-шторм — устройство так и не приняло запрос
         * (не «успех без данных»!). */
        serial_puts("USBMOUSE: TD NAK-storm ("); serial_puts(tag);
        serial_puts(") sts="); serial_puthex(sts);
        serial_puts("\n");
        return false;
    }
    um_delay(1000);   /* one frame settle */
    return true;
}

/* ── USB CONTROL transfer (setup + optional data + status) ──
 * v1.4: every stage is a single fresh TD on the permanently scheduled
 * control QH; the device fully processes the SETUP before the data
 * stage runs. */
static bool um_control(uint8_t bmReqType, uint8_t bReq, uint16_t wVal,
                       uint16_t wIdx, uint16_t wLen, void *data) {
    /* «ЭТАП 5»: защита аргументов — дата-стадия без буфера не строится */
    if (wLen > 0 && data == NULL)
        return false;
    uint8_t slot = um_cslot;
    um_cslot = (uint8_t)((um_cslot + 1) & (UM_CSLOTS - 1));
    uint8_t *setup = um_csetup[slot];
    setup[0] = bmReqType; setup[1] = bReq;
    setup[2] = (uint8_t)wVal;       setup[3] = (uint8_t)(wVal >> 8);
    setup[4] = (uint8_t)wIdx;       setup[5] = (uint8_t)(wIdx >> 8);
    setup[6] = (uint8_t)wLen;       setup[7] = (uint8_t)(wLen >> 8);

    /* ── Stage 1: SETUP (DATA0) ── */
    uhci_td_t *td = um_next_td();
    td->link     = 1;
    td->ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS |
                   (g_ls ? TD_CTRL_LS : 0u);
    td->token    = TD_TOKEN(PID_SETUP, g_devaddr, 0, 0, 8);
    td->buffer   = (uint32_t)setup;
    if (!um_run_td(td, "setup")) return false;

    /* ── Stage 2: DATA (optional, DATA1 first) ──
     * «ЭТАП 3»: чанк = EP0 MPS (из bMaxPacketSize0), а НЕ 64: у ВСЕХ
     * low-speed мышей EP0=8 — TD на 18 байт завершался коротким пакетом
     * после первых 8, хвост дескриптора читался нулями и um_find_hid
     * проваливался.  (uhci_ctl.c — клавиатурный стек — делает так же.) */
    if (wLen > 0 && data) {
        uint8_t pid = (bmReqType & 0x80) ? PID_IN : PID_OUT;
        if (pid == PID_OUT) memcpy(um_buf, data, wLen);
        else memset(um_buf, 0, sizeof(um_buf)); /* v1.5: deterministic tail */
        uint32_t rem = wLen;
        uint8_t *ptr = (pid == PID_IN) ? um_buf : (uint8_t *)data;
        uint8_t toggle = 1;
        uint8_t ep0 = g_ep0_mps;
        if (ep0 < 8) ep0 = 8;
        if (ep0 > 64) ep0 = 64;
        while (rem > 0) {
            uint32_t chunk = (rem > (uint32_t)ep0) ? ep0 : rem;
            td = um_next_td();
            td->link     = 1;
            td->ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS |
                           (g_ls ? TD_CTRL_LS : 0u);
            td->token    = TD_TOKEN(pid, g_devaddr, 0, toggle, chunk);
            td->buffer   = (uint32_t)ptr;
            if (!um_run_td(td, "data")) return false;
            ptr += chunk; rem -= chunk; toggle ^= 1;
        }
        if (pid == PID_IN) {
            um_dma();                   /* [#428] данные HC — из ОЗУ */
            memcpy(data, um_buf, wLen);
        }
    }

    /* ── Stage 3: STATUS (opposite direction, DATA1, zero length) ── */
    uint8_t status_pid = (bmReqType & 0x80) ? PID_OUT : PID_IN;
    td = um_next_td();
    td->link     = 1;
    td->ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS | TD_CTRL_IOC |
                   (g_ls ? TD_CTRL_LS : 0u);
    td->token    = TD_TOKEN(status_pid, g_devaddr, 0, 1, 0);
    td->buffer   = 0;
    if (!um_run_td(td, "status")) return false;
    return true;
}

/* ── Arm (or re-arm) the persistent interrupt-IN TD ──
 * The QH sits in every frame slot and is executed ~1 kHz.  While the
 * mouse is idle the device NAKs, the TD stays ACTIVE and polling costs
 * nothing.  head_link несёт цепочку клавиатуры (g_sched_chain) —
 * переарм чужую очередь не сносит.
 * Аудит #236 (проверено, ок): переарм пишет только um_itd и um_qh, а
 * затем ставит um_qh в головы кадров; QH клавиатуры достижим ЧЕРЕЗ
 * um_qh.head_link (= g_sched_chain), который восстанавливается ниже —
 * отрыва чужой очереди от головы списка не происходит.
 * «ЭТАП 5»: барьеры — токен/буфер TD обязаны быть в памяти ДО записи
 * elem_link (она вооружает TD) и до появления QH в списке кадров. */
static void um_arm(void) {
    uint8_t len = g_mps;
    if (len > 8) len = 8;
    if (len < 4) len = 4;   /* report is 3..8 bytes; 4 covers boot reports */

    um_itd.link     = 1;    /* terminate */
    um_itd.ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS |
                      (g_ls ? TD_CTRL_LS : 0u);
    um_itd.token    = TD_TOKEN(PID_IN, g_devaddr, g_ep & 0x0Fu, g_toggle, len);
    um_itd.buffer   = (uint32_t)um_pkt;

    um_barrier();                       /* токен/буфер уже в памяти */
    um_qh.head_link = g_sched_chain;   /* сохранить цепочку клавиатуры */
    um_qh.elem_link = (uint32_t)&um_itd;
    um_barrier();                       /* QH готов до входа в список кадров */

    um_sched_all((uint32_t)&um_qh | 2);  /* QH in every slot → ~1 kHz polling */
}

/* Stop scheduling the mouse TD; PS/2 resumes automatically.
 * v0.3.1: расписание НЕ сносим, если за QH мыши подцеплена чужая
 * очередь (клавиатура): QH остаётся в кадрах пустым маршрутизатором
 * и клавиатура продолжает работать. */
static void um_deactivate(const char *why) {
    usbdev_release(USBDEV_ROLE_MOUSE, why);
    /* 0.3.0: g_present = false — ОБЯЗАТЕЛЬНО.  Раньше флаг оставался
     * взведённым, и usb_mouse_rescan() навсегда пропускала контроллер
     * («наш, пусть и на паузе») — переподключённая мышь НЕ находилась
     * ВООБЩЕ НИКОГДА.  Теперь rescan пере-пробирует контроллер, и
     * мышь, воткнутая обратно, поднимается за секунду. */
    g_present = false;
    /* «ЭТАП 5 / #232/#233»: вернуть USB-адрес в битмапу — раньше адрес
     * утекал навсегда (счётчик только рос). */
    um_addr_release_used();
    serial_puts("USBMOUSE: deactivated (");
    serial_puts(why);
    serial_puts("), PS/2 resumes\n");
    um_qh.elem_link = 1;
    if (g_sched_chain == 1u)
        um_sched_all(1);
    g_active = false;
}

static void um_apply_buttons(uint8_t btn);   /* v1.5: forward */

/* ── Apply one boot-protocol report to the shared mouse state ──
 * Report layout (HID boot protocol mouse):
 *   byte 0: bit0 = left, bit1 = right, bit2 = middle
 *   byte 1: dX (signed)
 *   byte 2: dY (signed; USB HID convention: positive = down)
 */
static void um_handle_report(void) {
    uint8_t btn = um_pkt[0];
    int dx = (int)(int8_t)um_pkt[1] * 2;   /* ×2 — comfortable desktop speed */
    int dy = (int)(int8_t)um_pkt[2] * 2;

    if (dx >  48) dx =  48;
    if (dx < -48) dx = -48;
    if (dy >  48) dy =  48;
    if (dy < -48) dy = -48;

    int nx = g_mouse.x + dx;
    int ny = g_mouse.y + dy;
    if (nx < 0) nx = 0;
    if (nx >= g_fb_w) nx = g_fb_w - 1;
    if (ny < 0) ny = 0;
    if (ny >= g_fb_h) ny = g_fb_h - 1;
    if (nx != g_mouse.x || ny != g_mouse.y) {
        g_mouse.moved = true;
    }
    g_mouse.x = nx;
    g_mouse.y = ny;

    um_apply_buttons(btn);
}

/* ── v1.5: button edge detection shared by both report modes ──
 * Same semantics as the PS/2 driver. */
static void um_apply_buttons(uint8_t btn) {
    bool nl = (btn & 1) != 0;
    bool nr = (btn & 2) != 0;

    if (nl && !prev_left)  g_mouse.clicked_left  = true;
    if (nr && !prev_right) g_mouse.clicked_right = true;
    prev_left  = nl;
    prev_right = nr;
    g_mouse.left  = nl;
    g_mouse.right = nr;
}

/* ── v1.5: apply one ABSOLUTE (usb-tablet) report ──
 * Report layout (QEMU usb-tablet, absolute pointing device):
 *   byte 0:   buttons (bit0 = left, bit1 = right)
 *   bytes1-2: X, 16-bit little-endian (0..g_log_max)
 *   bytes3-4: Y, 16-bit little-endian
 *   byte 5+:  wheel / pressure — ignored
 * Logical coordinates are scaled onto the whole framebuffer so the OS
 * cursor follows the host cursor 1:1 (QEMU tablet: LOGICAL_MAX 32767). */
static void um_handle_tablet_report(void) {
    uint8_t  btn = um_pkt[0];
    uint32_t x = (uint32_t)um_pkt[1] | ((uint32_t)um_pkt[2] << 8);
    uint32_t y = (uint32_t)um_pkt[3] | ((uint32_t)um_pkt[4] << 8);

    if (g_log_max == 0) {
        /* Логический максимум не распознан: деление на (0+1) даёт
         * прыжок в правый нижний угол на каждом отчёте (дрожание).
         * Движение отбрасываем, кнопки применяем. */
        um_apply_buttons(btn);
        return;
    }

    int nx = (int)((x * (uint32_t)g_fb_w) / ((uint32_t)g_log_max + 1u));
    int ny = (int)((y * (uint32_t)g_fb_h) / ((uint32_t)g_log_max + 1u));
    if (nx < 0) nx = 0;
    if (nx >= g_fb_w) nx = g_fb_w - 1;
    if (ny < 0) ny = 0;
    if (ny >= g_fb_h) ny = g_fb_h - 1;
    if (nx != g_mouse.x || ny != g_mouse.y) {
        g_mouse.moved = true;
    }
    g_mouse.x = nx;
    g_mouse.y = ny;

    um_apply_buttons(btn);
}

/* ── Public: poll the interrupt endpoint (never blocks) ── */
void usb_mouse_poll(mouse_state_t *out) {
    g_mouse.clicked_left  = false;
    g_mouse.clicked_right = false;
    g_mouse.moved         = false;

    if (g_active) {
        /* «ЭТАП 5»: МГНОВЕННЫЙ детект выдёргивания — PORTSC.CCS (один
         * inw каждые 64 опроса ≈ десятки мс).  Раньше отключение
         * ловилось ТОЛЬКО накоплением 2048 ошибок TD (секунды!):
         * быстрый «выдернул и воткнул обратно» успевал прийти ДО
         * деактивации — kbd-уступка залипала в «уже уступали», и мышь
         * больше НЕ поднималась ВООБЩЕ (reproduced в QEMU: add→del→add
         * при живой клавиатуре).  CCS=0 при активном драйвере = устройство
         * физически выдернуто — деактивируем немедленно; rescan/kbd-yield
         * поднимут его заново при повторном подключении. */
        static uint16_t s_ccs_div;
        if ((uint16_t)(++s_ccs_div & 0x3Fu) == 0u) {
            uint16_t ps = inw((uint16_t)(g_io + UHCI_PORT_REG(g_port)));
            /* ghost-гварды: несуществующий порт (0xFFFF/reserved) не
             * считается отключением — порт реальный и был нашим при
             * привязке, но защита от чтения после чужого HCRESET. */
            if (ps != 0xFFFFu && (ps & 0xE000u) == 0u &&
                !(ps & UHCI_PORT_CONNECT)) {
                serial_puts("USBMOUSE: CCS=0 - мышь выдернута (мгновенный детект)\n");
                um_deactivate("device disconnected (CCS=0)");
                return;
            }
        }

        um_dma();                                   /* [#428] токен/данные HC — из ОЗУ */
        uint32_t sts = *(volatile uint32_t *)&um_itd.ctrl_sts;
        if (!(sts & TD_CTRL_ACTIVE)) {
            /* TD retired: a report arrived, an error occurred, or (on
             * some hosts) the device NAKed and the TD was retired. */
            g_alive_seen = true;
            g_watchdog = 0;
            if (sts & TD_CTRL_ANY_ERR) {
                /* v0.3.0: ошибка TD - НЕ убиваем драйвер: переармим TD
                 * (um_arm ниже) и продолжаем; тоггл сбрасываем в DATA0,
                 * т.к. после сбоя состояние тоггла устройства неизвестно.
                 * Деактивация только после ОЧЕНЬ долгой серии ошибок
                 * (~2 с подряд при опросе ~1 кГц): моргнутое питание или
                 * помеха больше не отключают мышь навсегда. */
                if (sts & TD_CTRL_STALLED) {
                    /* «ЭТАП 3»: STALL на interrupt-EP — снять HALT
                     * (CLEAR_FEATURE ENDPOINT_HALT), как uhci_kbd.c.
                     * Контрольный QH временно в кадрах; um_arm() ниже
                     * вернёт расписание к interrupt-QH. */
                    serial_puts("USBMOUSE: STALL на int-EP - CLEAR_FEATURE(HALT)\n");
                    um_sched_all((uint32_t)&um_ctrl_qh | 2);
                    (void)um_control(0x02, USB_REQ_CLEAR_FEATURE, 0,
                                     (uint16_t)(0x80u | (g_ep & 0x0Fu)), 0, 0);
                }
                g_errors++;
                g_toggle = 0;
                if (g_errors >= 2048) um_deactivate("transfer errors");
            } else if (sts & TD_CTRL_NAK) {
                /* No new report — nothing to parse, keep the toggle. */
                g_errors = 0;
            } else {
                g_errors = 0;
                int act = (int)(sts & 0x7FFu) + 1;   /* ActLen = actual-1 */
                if (act > 8) act = 8;
                if (!g_first_pkt) {
                    g_first_pkt = true;
                    serial_puts("USBMOUSE: первый отчёт (");
                    {
                        char nb[8];
                        serial_puts(itoa((uint32_t)act, nb, 10));
                    }
                    serial_puts(" байт):");
                    for (int k = 0; k < act && k < 8; k++) {
                        serial_puts(" ");
                        serial_puthex(um_pkt[k]);
                    }
                    serial_puts("\n");
                }
                /* v1.5: tablet reports are ≥5 bytes, boot mouse ≥3. */
                int min_act = g_absolute ? 5 : 3;
                if (act >= min_act) {
                    usbdev_note_report(USBDEV_ROLE_MOUSE);   /* [#462] живость для КАРТЫ ГНЕЗД */
                    if (g_absolute) um_handle_tablet_report();
                    else            um_handle_report();
                }
                g_toggle ^= 1;                        /* DATA0 <-> DATA1 */
            }
            um_arm();                                 /* re-arm for next frame */
        } else {
            /* Still waiting: idle (NAK) or the schedule never ran. */
            if (sts & TD_CTRL_NAK) {
                g_alive_seen = true;
                g_watchdog = 0;
                g_errors = 0;
            } else if (!g_alive_seen && ++g_watchdog > 5000u) {
                /* 5 секунд (при опросе ~1 кГц) - раньше было 200000
                 * (~200 с): зависший контроллер полминуты рисовал мёртвый
                 * курсор вместо возврата к PS/2. */
                um_deactivate("watchdog");
            }
        }
    }

    if (out) *out = g_mouse;
}

bool usb_mouse_present(void) { return g_present; }
bool usb_mouse_active(void)  { return g_present && g_active; }

/* ── Walk the config descriptor: pick the best HID interface + int-IN EP ── */
static bool um_find_hid(uint8_t *cfg, uint16_t total) {
    /* Pass 1: choose interface — HID boot mouse preferred.
     * v0.3.1: принимаются ТОЛЬКО интерфейсы протокола «мышь»
     * (bInterfaceProtocol == 2) — раньше любой HID-интерфейс
     * привязывался как мышь, и USB-КЛАВИАТУРА (protocol 1) начинала
     * «двигать курсор» своими boot-отчётами. */
    int best = -1;
    bool best_boot = false;
    int pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        /* «ЭТАП 5 / #296b»: дескриптор ОБРЕЗАН границей прочитанного —
         * поля за total читали бы мусор/ООВ (как uhci_ctl.c: p+len>total). */
        if (pos + len > (int)total) break;
        if (type == DESC_INTERFACE && len >= 9 &&          /* «ЭТАП 5 / #296b»: без len>=9 чтение pos+5/pos+7 — за границей дескриптора */
            cfg[pos + 5] == 0x03 &&                             /* HID class */
            cfg[pos + 7] == 0x02) {                             /* mouse!    */
            bool boot_mouse = (cfg[pos + 6] == 0x01);           /* boot      */
            if (best < 0 || (boot_mouse && !best_boot)) {
                best      = cfg[pos + 2];                         /* bInterfaceNumber */
                best_boot = boot_mouse;
            }
        }
        pos += len;
    }
    if (best < 0) return false;
    g_iface = (uint8_t)best;
    g_boot_iface = best_boot;   /* v1.5 */

    /* Pass 2: interrupt-IN endpoint belonging to that interface. */
    bool in_iface = false;
    pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (pos + len > (int)total) break;   /* «ЭТАП 5 / #296b»: обрезан — стоп */
        if (type == DESC_INTERFACE) {
            in_iface = (cfg[pos + 2] == g_iface);
        } else if (type == DESC_ENDPOINT && len >= 7 && in_iface) {  /* «ЭТАП 5 / #296b»: len>=7 — граница полей EP */
            uint8_t addr = cfg[pos + 2];
            uint8_t attr = cfg[pos + 3];
            if ((addr & 0x80u) && (attr & 0x03u) == 0x03u) {      /* interrupt IN */
                g_ep  = addr & 0x0Fu;
                g_mps = cfg[pos + 4];                              /* wMaxPacketSize lo */
                break;
            }
        }
        pos += len;
    }
    return (g_ep != 0);
}

/* ── v1.5: minimal HID report descriptor parser ──
 * Walks the short items (main/global) and answers true if the descriptor
 * contains an INPUT item with Data|Var|Absolute flags (val&7 == 0x02)
 * declared with Report Size 16 — an absolute pointer axis (usb-tablet).
 * *max receives the LOGICAL_MAXIMUM in effect for that item. */
static bool um_parse_absolute(const uint8_t *r, uint16_t n, uint16_t *max) {
    uint32_t rpt_size = 0;
    uint32_t log_max  = 0;
    uint16_t abs_max  = 0;
    bool abs16 = false;
    int pos = 0;

    while (pos < (int)n) {
        uint8_t item = r[pos++];
        if (item == 0xFE) {                       /* long item — skip */
            if (pos + 1 >= (int)n) break;
            int l = (int)r[pos] | ((int)r[pos + 1] << 8);
            pos += 2 + l;
            continue;
        }
        uint32_t sz = (uint32_t)(item & 0x03);
        if (sz == 3) sz = 4;
        if (pos + (int)sz > (int)n) break;

        uint32_t val = 0;
        for (uint32_t i = 0; i < sz; i++)
            val |= ((uint32_t)r[pos + i]) << (8 * i);
        pos += (int)sz;

        if (item == 0x75) {                       /* REPORT_SIZE (1 byte) */
            rpt_size = val;
        } else if (item == 0x25 || item == 0x26 || item == 0x27) {
            log_max = val;                        /* LOGICAL_MAXIMUM */
        } else if (item == 0x81) {                /* INPUT (flags, 1 byte) */
            if ((val & 7u) == 0x02u && rpt_size == 16u) {
                abs16   = true;                   /* Data|Var|Absolute, 16-bit */
                abs_max = (uint16_t)log_max;
            }
        }
    }

    if (!abs16) return false;
    if (max) {
        if (abs_max == 0) abs_max = 32767u;       /* sane default */
        *max = abs_max;
    }
    return true;
}

/* ── v1.5: absolute (tablet) or relative (boot mouse)? ──
 * Reads the class GET_DESCRIPTOR(0x2200) report descriptor for the
 * enumerated interface (length taken from the HID descriptor inside the
 * config blob) and parses it with um_parse_absolute().  When the
 * descriptor cannot be read, falls back to bInterfaceSubClass:
 * no boot subclass → treat the device as a tablet. */
static bool um_detect_mode(const uint8_t *cfg, uint16_t total) {
    int pos = 0, rpt_len = 0;
    bool in_iface = false;

    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (pos + len > (int)total) break;   /* «ЭТАП 5 / #296b»: обрезан — стоп */
        if (type == DESC_INTERFACE) {
            in_iface = (cfg[pos + 2] == g_iface);
        } else if (type == USB_DT_HID && len >= 9 && in_iface) {  /* «ЭТАП 5 / #296b»: len>=9 — pos+7/pos+8 в границах */
            rpt_len = cfg[pos + 7] | ((int)cfg[pos + 8] << 8);
            break;
        }
        pos += len;
    }

    if (rpt_len <= 0) {
        serial_puts("USBMOUSE: no HID report descriptor length, mode by subclass\n");
        return !g_boot_iface;
    }
    if (rpt_len > (int)sizeof(um_rpt)) rpt_len = (int)sizeof(um_rpt);

    memset(um_rpt, 0, sizeof(um_rpt));
    if (!um_control(0x81, USB_REQ_GET_DESCRIPTOR, (USB_DT_REPORT << 8),
                    g_iface, (uint16_t)rpt_len, um_rpt)) {
        serial_puts("USBMOUSE: report descriptor read failed, mode by subclass\n");
        return !g_boot_iface;
    }
    um_dump("rptdesc", um_rpt, rpt_len > 24 ? 24 : rpt_len);
    return um_parse_absolute(um_rpt, (uint16_t)rpt_len, &g_log_max);
}

/* ── Enumerate the device on the already-reset port as a HID mouse ── */
static bool um_enumerate(void) {
    g_devaddr    = 0;
    g_toggle     = 0;
    g_errors     = 0;
    g_watchdog   = 0;
    g_alive_seen = false;
    g_first_pkt = false;
    g_ep         = 0;
    g_iface      = 0;
    g_mps        = 8;
    g_ep0_mps    = 8;       /* «ЭТАП 3»: уточнится из bMaxPacketSize0 */
    g_boot_iface = false;
    g_absolute   = false;
    g_log_max    = 32767;   /* v1.5: sane default (QEMU tablet) */

    /* v1.4: schedule the control pipe QH — it stays up until um_arm()
     * swaps the schedule to the interrupt QH (or we tear it down).
     * «ЭТАП 5 / #234»: head_link = цепочка клавиатуры (если работаем в
     * общем расписании) — опрос клавиатуры не прерывается даже на время
     * перечисления мыши; без клавиатуры g_sched_chain == 1 (terminate). */
    um_ctrl_qh.head_link = g_sched_chain;
    um_ctrl_qh.elem_link = 1;
    um_sched_all((uint32_t)&um_ctrl_qh | 2);
    um_ctd_i = 0;

    /* GET_DESCRIPTOR (Device, first 8 bytes) at address 0.
     * «ЭТАП 3»: до 3 попыток с паузой 20 мс (как uhci_ctl.c) — реальное
     * устройство после ресета может не ответить на самый первый запрос. */
    uint8_t dev_desc[18];
    memset(dev_desc, 0, sizeof(dev_desc));
    bool got8 = false;
    for (int att = 0; att < 3 && !got8; att++) {
        /* «ЭТАП 5 / #295»: мусорный ответ (нулевой/чужой bDescriptorType)
         * считается НЕудачей — ретрай, а не парсинг случайных байтов */
        got8 = um_control(0x80, USB_REQ_GET_DESCRIPTOR,
                          (USB_DT_DEVICE << 8), 0, 8, dev_desc) &&
               dev_desc[1] == USB_DT_DEVICE;
        if (!got8) um_delay(20000);
    }
    if (!got8) {
        serial_puts("USBMOUSE: enum: GET_DESCRIPTOR(8) failed\n");
        return false;
    }
    um_dump("dev8", dev_desc, 8);
    /* «ЭТАП 5 / #204/#294»: bMaxPacketSize0 клампится по скорости
     * устройства (uhci_mps_fix выше): LS -> строго 8, FS -> 8/16/32/64,
     * мусор (0,1,3,...) -> безопасные 8. */
    g_ep0_mps = uhci_mps_fix(dev_desc[7], g_ls);
    serial_puts("USBMOUSE: EP0 MPS=");
    { char nb[8]; serial_puts(itoa((uint32_t)g_ep0_mps, nb, 10)); }
    serial_puts("\n");

    /* 0.3.0: УНИКАЛЬНЫЙ адрес.  Раньше всегда 1: если соседний порт уже
     * получил адрес 1 (перечисление не-мыши на нём), второе устройство
     * тоже становилось «адресом 1» — коллизия адресов, и дескрипторы
     * читались с ЧУЖОГО устройства («оба в порту не работают»).
     * «ЭТАП 5 / #232/#233»: выдача из битмапы g_um_addr_bm с возвратом
     * при провале/деактивации — счётчик переполнялся в адрес 0
     * (broadcast!) после ~127 переподключений.  Адрес 2 зарезервирован
     * за драйвером клавиатуры (uhci_ctl.c, см. um_addr_release_all). */
    int my_addr = um_addr_alloc();
    if (my_addr < 0) {
        serial_puts("USBMOUSE: enum: свободных USB-адресов нет\n");
        return false;
    }
    g_um_addr_used = (uint8_t)my_addr;
    if (!um_control(0x00, USB_REQ_SET_ADDRESS, (uint16_t)my_addr, 0, 0, 0)) {
        serial_puts("USBMOUSE: enum: SET_ADDRESS failed\n");
        return false;
    }
    g_devaddr = (uint8_t)my_addr;
    /* «ЭТАП 3»: 20 мс на устаканивание адреса (Linux ждёт ~10+ мс;
     * было 5 — реальное устройство могло пропустить след. запрос). */
    um_delay(20000);

    /* GET full Device Descriptor. */
    if (!um_control(0x80, USB_REQ_GET_DESCRIPTOR, (USB_DT_DEVICE << 8), 0, 18, dev_desc)) {
        serial_puts("USBMOUSE: enum: GET_DESCRIPTOR(18) failed\n");
        return false;
    }
    /* «ЭТАП 5 / #295»: тип полного дескриптора устройства */
    if (dev_desc[1] != USB_DT_DEVICE) {
        serial_puts("USBMOUSE: enum: dev18: неверный bDescriptorType\n");
        return false;
    }
    um_dump("dev18", dev_desc, 16);

    serial_puts("USBMOUSE: device id=");
    serial_puthex(((uint32_t)dev_desc[8]) | ((uint32_t)dev_desc[9] << 8));
    serial_puts(":");
    serial_puthex(((uint32_t)dev_desc[10]) | ((uint32_t)dev_desc[11] << 8));
    serial_puts("\n");

    /* GET Configuration Descriptor (short first). */
    uint8_t cfg_hdr[9];
    if (!um_control(0x80, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIG << 8), 0, 9, cfg_hdr)) {
        serial_puts("USBMOUSE: enum: GET_CONFIG(9) failed\n");
        return false;
    }
    um_dump("cfg9", cfg_hdr, 9);
    /* «ЭТАП 5 / #295»: тип конфиг-дескриптора обязан быть CONFIG */
    if (cfg_hdr[1] != USB_DT_CONFIG) {
        serial_puts("USBMOUSE: enum: cfg9: неверный bDescriptorType\n");
        return false;
    }
    uint16_t total_len = (uint16_t)(cfg_hdr[2] | ((uint16_t)cfg_hdr[3] << 8));
    /* «ЭТАП 5 / #296»: wTotalLength в разумных границах; больше 255 Б не
     * читаем (стек-буфер) — усечение ЛОГИРУЕТСЯ, парсинг остаётся строго
     * в пределах прочитанного. */
    if (total_len < 9) {
        serial_puts("USBMOUSE: enum: wTotalLength<9 - мусор\n");
        return false;
    }
    if (total_len > 255) {
        serial_puts("USBMOUSE: конфиг ");
        { char nb[8]; serial_puts(itoa((uint32_t)total_len, nb, 10)); }
        serial_puts(" Б усечён до 255\n");
        total_len = 255;
    }

    uint8_t cfg_full[255];
    if (!um_control(0x80, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIG << 8), 0, total_len, cfg_full)) {
        serial_puts("USBMOUSE: enum: GET_CONFIG(full) failed\n");
        return false;
    }
    um_dump("cfgfull", cfg_full, 16);
    serial_puts("USBMOUSE: cfg total=");
    serial_puthex(total_len);
    serial_puts("\n");

    if (!um_find_hid(cfg_full, total_len)) {
        serial_puts("USBMOUSE: no HID boot-mouse interface\n");
        /* 0.3.0: не мышь — спросить порт и подобрать драйвер из ШЕСТИ.
         * Клавиатурный интерфейс? — его привяжет UHCIKBD (свои вызовы
         * реестра).  Ни клава, ни мышь (флешка/хаб) — диспетчер
         * печатает «НЕИЗВЕСТНОЕ УСТРОЙСТВО». */
        {
            bool kbd_if = false;
            int pos = 0;
            while (pos + 1 < (int)total_len) {
                uint8_t len = cfg_full[pos], type = cfg_full[pos + 1];
                if (len < 2) break;
                if (pos + len > (int)total_len) break;   /* «ЭТАП 5 / #296b»: обрезан — стоп */
                if (type == DESC_INTERFACE && len >= 9 &&
                    cfg_full[pos + 5] == 0x03u &&      /* class HID */
                    cfg_full[pos + 6] == 0x01u &&      /* boot */
                    cfg_full[pos + 7] == 0x01u) {      /* protocol kbd */
                    kbd_if = true;
                    break;
                }
                pos += len;
            }
            if (!kbd_if) {
                int ver = usbdev_port_version(0x00);
                usbdev_ask_port("UHCI", ver, (int)g_um_port_cur + 1);   /* [#458] 1-базированные, как в карте */
                usbdev_try_bind(ver, 0, 0, "UHCI", (int)g_um_port_cur + 1,
                                (unsigned)(dev_desc[8] | ((unsigned)dev_desc[9] << 8)),
                                (unsigned)(dev_desc[10] | ((unsigned)dev_desc[11] << 8)));
                /* вернёт NONE: сообщение «НЕИЗВЕСТНОЕ УСТРОЙСТВО» уже
                 * напечатано диспетчером */
            }
        }
        return false;
    }

    /* 0.3.0: «порт, кто ты?» + драйвер мыши из ШЕСТИ (реестр usbdev).
     * UHCI без EHCI на машине = «USB 1.x»; с EHCI (порты 2.0 ушли к
     * компаньону после handoff) = «USB 2.0».  Слот мыши занят или
     * устройство не мышь — НЕИЗВЕСТНОЕ УСТРОЙСТВО. */
    {
        int ver = usbdev_port_version(0x00);
        usbdev_ask_port("UHCI", ver, (int)g_um_port_cur + 1);   /* [#458] */
        unsigned uv = (unsigned)(dev_desc[8] | ((unsigned)dev_desc[9] << 8));
        unsigned up = (unsigned)(dev_desc[10] | ((unsigned)dev_desc[11] << 8));
        if (usbdev_try_bind(ver, 0, 1, "UHCI", (int)g_um_port_cur + 1, uv, up)
                != USBDEV_ROLE_MOUSE) {
            serial_puts("USBMOUSE: драйвер мыши не подошёл - НЕИЗВЕСТНОЕ УСТРОЙСТВО\n");
            return false;
        }
    }

    /* SET_CONFIGURATION — «ЭТАП 5 / #297»: отказ ФАТАЛЕН (устройство не
     * сконфигурировано, interrupt-EP не отвечает); в отличие от
     * SET_IDLE/SET_PROTOCOL это не «необязательная» настройка. */
    if (!um_control(0x00, USB_REQ_SET_CONFIG, cfg_hdr[5], 0, 0, 0)) {
        serial_puts("USBMOUSE: enum: SET_CONFIG failed\n");
        usbdev_release(USBDEV_ROLE_MOUSE, "SET_CONFIG не прошёл");
        return false;
    }
    um_delay(5000);

    /* v1.5: decide the report mode BEFORE arming the interrupt pipe:
     * read the HID report descriptor and look for a 16-bit absolute
     * pointer axis (usb-tablet); on failure fall back to the boot
     * subclass heuristic (no boot subclass → tablet). */
    g_absolute = um_detect_mode(cfg_full, total_len);

    /* HID class requests: switch a boot-capable mouse to the boot
       protocol (a tablet has no boot subclass and stays in report
       protocol); SET_IDLE for everyone.
       «ЭТАП 5 / #203/#252»: STALL на SET_IDLE/SET_PROTOCOL — НЕ провал
       перечисления: halt на EP0 самоочищается к следующему SETUP
       (CLEAR_FEATURE(ENDPOINT_HALT) для EP0 не нужен), а um_control
       каждый запрос начинает со стадии SETUP=DATA0 — data-toggle EP0
       после STALL консистентен автоматически.  Отказ игнорируется. */
    if (g_boot_iface)
        (void)um_control(0x21, USB_HID_SET_PROTOCOL, 0, g_iface, 0, 0);
    (void)um_control(0x21, USB_HID_SET_IDLE, 0, g_iface, 0, 0);

    serial_puts("USBMOUSE: HID ");
    serial_puts(g_absolute ? "absolute (tablet)" : "relative (boot mouse)");
    serial_puts(" on EP");
    serial_puthex(0x80u | g_ep);
    serial_puts(" iface=");
    serial_puthex(g_iface);
    serial_puts(" mps=");
    serial_puthex(g_mps);
    serial_puts("\n");

    g_present = true;
    g_active  = true;
    um_arm();
    return true;
}

/* ── 0.3.2: проба одного UHCI-контроллера ──────────────────────────────
 * Сброс, расписание, порты 0/1. true = мышь привязана (g_io/g_port
 * выставлены, TD прерываний в расписании); false = контроллер
 * остановлен и освобождён (для клавиатуры или следующего перескана). */
/* 0.3.0: коллизия АДРЕСА 0.  Неадресованное устройство на СОСЕДНЕМ
 * порту слышит те же addr-0 транзакции (root hub адресует все порты):
 * перечисление мыши на одном порту получало дескрипторы КЛАВИАТУРЫ
 * с другого — «оба в порту 2.0/1.x не работают».  Как
 * oh_quiesce_others() в OHCI-драйвере: на время перечисления ВСЕ
 * прочие порты отключаются (PE=0).
 * «ЭТАП F1»: гасим не один «сосед», а все прочие живые порты — на
 * 4-портовых UHCI одного соседа мало.
 * «ЭТАП 5 / #234» (критично): порт, которым ВЛАДЕЕТ драйвер клавиатуры
 * (usb_uhci_kbd.c уже шедулирует свой int-TD), НЕ гасим: PE после
 * записи 0 возвращается только сбросом порта, которого клавиатурный
 * драйвер уже не делает (его init отработал) — работающая клавиатура
 * умирала НАВСЕГДА при перечислении мыши на том же UHCI.  Порт
 * клавиатуры возвращает uhci_kbd_bound_port(io).  Приглушённые порты
 * запоминаем в um_muted — um_probe_controller вернёт их коротким
 * port-reset'ом (успех ИЛИ провал), устройства останутся достижимыми
 * для последующих проб (клавиатура, MSD). */
static void um_quiesce_others(int portno, int nports)
{
    int kport = uhci_kbd_bound_port((uint16_t)g_io);
    for (int p = 0; p < nports; p++) {
        uint16_t reg;
        uint16_t st;
        if (p == portno)
            continue;
        if (p == kport) {
            serial_puts("USBMOUSE: порт ");
            { char nb[8]; serial_puts(itoa((uint32_t)p, nb, 10)); }
            serial_puts(" занят клавиатурой - не приглушаю\n");
            continue;
        }
        reg = UHCI_PORT_REG(p);
        st = inw((uint16_t)(g_io + reg));
        if ((st & UHCI_PORT_CONNECT) && (st & UHCI_PORT_ENABLE)) {
            /* запись 0: PE=0 (выключить порт), suspend=0, w1c-биты не тронуты */
            outw((uint16_t)(g_io + reg), 0);
            um_delay(2000);
            {
                uint16_t rb = inw((uint16_t)(g_io + reg));
                char nb[8];
                serial_puts("USBMOUSE: порт ");
                serial_puts(itoa((uint32_t)p, nb, 10));
                serial_puts(" приглушён (addr-0 коллизия)");
                serial_puts(" sts=");
                serial_puthex(rb);
                serial_puts("\n");
            }
            um_muted |= (uint8_t)(1u << p);
        }
    }
}

static bool um_probe_controller(uint16_t io)
{
    g_io = io;
    um_muted = 0;                /* «ЭТАП 5 / #234»: маска — на эту пробу */
    /* «ЭТАП 5 / #234»: если на контроллере ЖИВЁТ расписание клавиатуры
     * (usb_uhci_kbd привязан), контроллер НЕ сбрасываем (HCRESET уронил
     * бы её int-TD навсегда) — но список кадров ПЕРЕПРОГРАММИРУЕМ себе
     * (uhci_init_framelist БЕЗ HCRESET безопасна на работающем HC:
     * FLBASE вступает в силу со следующего кадра, RS|CF|MAXP idempotent),
     * а QH клавиатуры подцепляем за своим (usb_mouse_sched_chain →
     * um_arm/um_ctrl_qh сохраняют связь) — клавиатура продолжает
     * опрашиваться ЧЕРЕЗ наш список кадров.  Иначе — как раньше:
     * полный сброс и своё расписание. */
    if (uhci_kbd_bound_port(io) >= 0) {
        uint32_t klink = uhci_kbd_int_qh_link();
        /* klink==0 невозможен (bound => attached), но защита от записи
         * нулевого link-указателя в кадры дешевле, чем вера в это. */
        usb_mouse_sched_chain(klink ? klink : 1u);
        serial_puts("USBMOUSE: контроллер занят клавиатурой - общее расписание\n");
        uhci_init_framelist();
        /* «ЭТАП 5 / #234» (собственная находка): сразу вернуть кадры
         * клавиатуре — до um_enumerate() ещё ~250 мс (дебаунс CCS 100 мс
         * + сброс порта 60+50 мс + TRSTRCY): все это время её очередь
         * была бы недостижима (кадры = terminate) и нажатия терялись
         * (HID-отчёты состояния не ретранслируются).  um_enumerate()
         * затем перепрограммирует кадры на um_ctrl_qh, за которым QH
         * клавиатуры подцеплен через g_sched_chain. */
        um_sched_all(klink ? klink : 1u);
    } else {
        usb_mouse_sched_chain(1);   /* снять возможную старую цепочку */
        uhci_reset();
        uhci_init_framelist();
    }

    /* «ЭТАП F1»: порты 0..n-1, n = до первого PORTSC=0xFFFF
     * (Linux uhci-hcd): было жёстко 0/1 — порты 3-4 пропадали молча. */
    {
        int nports = um_port_count(io);
        int kport = uhci_kbd_bound_port(io);   /* «ЭТАП 5 / #234»: порт клавиатуры — не кандидат */
        for (int portno = 0; portno < nports; portno++) {
            if (portno == kport)
                continue;   /* там живая клавиатура: ни приглушение, ни reset */
            g_um_port_cur = (uint8_t)portno;
            um_quiesce_others(portno, nports);
            if (uhci_port_reset(UHCI_PORT_REG(portno), portno) && um_enumerate()) {
                char nb[8];
                g_port = (uint8_t)portno;
                serial_puts("USBMOUSE: ready (порт ");
                serial_puts(itoa((uint32_t)(portno + 1), nb, 10));   /* [#458] 1-базированный, как в карте */
                serial_puts(")\n");
                usbmap_pick(0x00, "mouse", portno + 1,
                            usbdev_port_version(0x00) == 2 ? "2.0" : "1.x");   /* [#449] честное ГНЕЗДО */
                um_restore_muted();      /* «ЭТАП 5 / #234»: успех — тоже вернуть */
                return true;
            }
            /* «ЭТАП 5 / #232/#233»: перечисление не вышло — вернуть адрес */
            um_addr_release_used();
        }
    }

    /* No HID mouse on this controller.
     * «ЭТАП 5 / #234»: сначала вернуть приглушённые порты, затем — только
     * если контроллером не владеет клавиатура — остановить HC; её
     * очередь остаётся жить, а кадры вернём ей. */
    um_restore_muted();
    if (uhci_kbd_bound_port(io) >= 0) {
        uint32_t klink = uhci_kbd_int_qh_link();
        um_sched_all(klink ? klink : 1u);   /* кадры — обратно клавиатуре */
    } else {
        um_sched_all(1);
        uhci_write16(UHCI_CMD, 0);
    }
    usb_mouse_sched_chain(1);   /* «ЭТАП 5»: цепь не должна тащиться в след. пробу */
    g_io = 0;
    g_port = 0xFF;
    return false;
}

/* Каким UHCI сейчас владеет драйвер клавиатуры (usb_uhci_kbd.c)?
 * 0 = никем. rescan эти контроллеры не трогает. */
void usb_mouse_set_kbd_controller(uint16_t io)
{
    g_kbd_io = io;
}

/* [LOGS] ЭТАП 11: регистрация/снятие владения UHCI со стороны MSD. */
void usb_mouse_set_msd_controller(uint16_t io)
{
    g_msd_io = io;
}

uint16_t usb_mouse_msd_controller(void)
{
    return g_msd_io;
}

/* ── 0.3.2: hotplug-перескан (вызывается из ps2_poll, когда USB-мыши
 * нет). Внутренний счётчик ограничивает частоту; полный цикл пробы
 * выполняется только если на каком-то порту СВОБОДНОГО контроллера
 * видно устройство (дешёвый фильтр — два чтения PORTSC). */
/* 0.3.0: обход СЕЙЧАС (без внутреннего троттлинга) — вызывается
 * диспетчером usbdev_tick() каждую секунду, «независимо от
 * обстоятельств».  Дешёвый фильтр по PORTSC внутри. */
/* 0.3.0 (доработка модели): опрос ВСЕХ портов ВСЕХ контроллеров — и
 * когда мышь УЖЕ привязана. Новое устройство ловится по СМЕНЕ
 * CONNECT-бита (защита от повторного перечисления каждую секунду).
 * Чужой контроллер при занятой мыши НЕ перечисляем (статические
 * QH/TD — одна копия на драйвер): по модели диспетчера при занятых
 * обоих слотах устройство = НЕИЗВЕСТНОЕ УСТРОЙСТВО; при свободном
 * клавиатурном слоте его перепривязку пробует UHCIKBD (свой poll). */
static uint16_t g_scan_conn[8];   /* CONNECT-битмап портов (прошлый обход) */

void usb_mouse_rescan_now(void)
{
    if (g_scan_n == 0)
        return;

    for (int i = 0; i < g_scan_n; i++) {
        uint16_t io = g_scan_io[i];
        if (io == 0)
            continue;

        /* Состояние портов пишем ДЛЯ ВСЕХ контроллеров (edge-детект
         * должен видеть подъемы и на «своих» — механизм уступки
         * UHCIKBD переключает контроллер в чужие мгновенно).
         * «ЭТАП F1»: читаем ВСЕ порты (до первого 0xFFFF): было только 0/1
         * — подключение в порты 3-4 не замечалось. */
        uint16_t conn = 0;
        {
            int nports = um_port_count(io);
            for (int p = 0; p < nports; p++) {
                uint16_t st = inw((uint16_t)(io + UHCI_PORT_REG(p)));
                if (st & UHCI_PORT_CONNECT)
                    conn |= (uint16_t)(1u << p);
            }
        }
        uint16_t prev = g_scan_conn[i];
        g_scan_conn[i] = conn;
        uint16_t fresh = (uint16_t)(conn & (uint16_t)~prev);

        if (g_present && g_io == io)
            continue;               /* наш контроллер: события ведёт мышь */
        if (g_kbd_io == io)
            continue;               /* клавиатура: её yield-механизм сам
                                     * позовёт rescan при новом соседе */
        if (g_msd_io == io)
            continue;               /* [LOGS] флешка MSD (usb.c): её адрес
                                     * живёт на порту — НЕ перечисляем */
        if (!fresh)
            continue;               /* новых подключений нет */

        if (g_active) {
            /* мышь уже привязана на другом UHCI: перечислять ЧУЖОЙ
             * контроллер нельзя (общие статические QH/TD).  По модели
             * usbdev занятые слоты никому больше не подбираются. */
            int ver = usbdev_port_version(0x00);
            /* «ЭТАП F1»: биты fresh — все порты (а не только 0/1) */
            for (int p = 0; p < UHCI_SCAN_PORTS; p++) {
                char nb[8];
                if (!(fresh & (1u << p)))
                    continue;
                usbdev_ask_port("UHCI", ver, p);
                if (usbdev_kbd_bound() && usbdev_mouse_bound()) {
                    serial_puts("USBMOUSE: UHCI порт ");
                    serial_puts(itoa((uint32_t)p, nb, 10));
                    serial_puts(": оба слота реестра заняты - НЕИЗВЕСТНОЕ УСТРОЙСТВО\n");
                } else {
                    serial_puts("USBMOUSE: новое устройство в UHCI-порту: мышь уже привязана,\n"
                                "USBMOUSE: слот клавиатуры свободен - перепривязку пробует UHCIKBD\n");
                }
            }
            continue;
        }
        serial_puts("USBMOUSE: hotplug: устройство в UHCI-порту - скан\n");
        if (um_probe_controller(io))
            return;
    }
}

void usb_mouse_rescan(void)
{
    if (g_active || g_scan_n == 0)
        return;
    if (++g_rescan_cnt < 500u)
        return;
    g_rescan_cnt = 0;

    for (int i = 0; i < g_scan_n; i++) {
        uint16_t io = g_scan_io[i];
        if (io == 0)
            continue;
        if (g_present && g_io == io)
            continue;               /* наш контроллер (пусть и на паузе) */
        if (g_kbd_io == io)
            continue;               /* расписание клавиатуры не трогаем */
        if (g_msd_io == io)
            continue;               /* [LOGS] контроллер MSD-флешки не трогаем */

        /* «ЭТАП F1»: CONNECT по ВСЕМ портам (до первого 0xFFFF) */
        uint16_t st = 0;
        {
            int nports = um_port_count(io);
            for (int p = 0; p < nports; p++)
                st |= inw((uint16_t)(io + UHCI_PORT_REG(p)));
        }
        if (!(st & UHCI_PORT_CONNECT))
            continue;               /* пусто — дёшево уходим */

        serial_puts("USBMOUSE: hotplug: устройство в UHCI-порту - скан\n");
        if (um_probe_controller(io))
            return;
    }
}

/* ── Public: PCI scan for UHCI + enumeration ── */
void usb_mouse_init(int fb_w, int fb_h) {
    if (g_present) return;   /* already initialised */
    um_addr_release_all();   /* «ЭТАП 5 / #232»: битмапа адресов; 2 — клавиатура */

    g_fb_w = fb_w;
    g_fb_h = fb_h;
    g_mouse.x = fb_w / 2;
    g_mouse.y = fb_h / 2;
    g_mouse.left = g_mouse.right = false;
    g_mouse.moved = false;
    g_mouse.clicked_left = false;
    g_mouse.clicked_right = false;
    prev_left = prev_right = false;

    for (int i = 0; i < 1024; i++) um_frame_list[i] = 1;   /* terminate */

    serial_puts("USBMOUSE: scanning PCI for UHCI\n");

    /* все 256 PCI-шин (как в mouse_hw.c): раньше сканировались только
     * 0..7 - контроллер за мостом на шине 8+ не находился */
    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t dev = 0; dev < 32; dev++) {
            for (uint8_t fn = 0; fn < 8; fn++) {
                uint32_t id = pci_read32((uint8_t)bus, dev, fn, 0x00);
                if ((id & 0xFFFFu) == 0xFFFFu) {
                    if (fn == 0) break;   /* no function 0 → skip device */
                    continue;
                }
                uint8_t class_code = pci_read8((uint8_t)bus, dev, fn, 0x0B);
                uint8_t subclass   = pci_read8((uint8_t)bus, dev, fn, 0x0A);
                uint8_t prog_if    = pci_read8((uint8_t)bus, dev, fn, 0x09);

                /* UHCI: class=0x0C, sub=0x03, prog-if=0x00 */
                if (class_code == 0x0C && subclass == 0x03 && prog_if == 0x00) {
                    /* BAR4 contains the UHCI I/O base (bit 0 = I/O space).
                     * [#434] fallback на BAR0 (0x10): часть контроллеров
                     * (SiS/VIA) держат базу в BAR0 — usb_uhci_kbd.c умеет
                     * оба, мышь — не умела. */
                    uint32_t bar4 = pci_read32(bus, dev, fn, 0x20);
                    if (!(bar4 & 1u)) {
                        uint32_t bar0 = pci_read32(bus, dev, fn, 0x10);
                        if (bar0 & 1u) bar4 = bar0;
                    }
                    if (!(bar4 & 1u)) continue;
                    /* 0.3.0 ФИКС: база — в ЛОКАЛЬНУЮ переменную, НЕ в g_io!
                     * Раньше каждый найденный контроллер затирал g_io: на
                     * машине с ДВУМЯ UHCI после привязки мыши на первом
                     * g_io указывал на ПОСЛЕДНИЙ найденный — секундный
                     * свип считал его «нашим» и пропускал, а чужие
                     * устройства (флешка = НЕИЗВЕСТНОЕ УСТРОЙСТВО) не
                     * детектировались вовсе. g_io ставит только
                     * um_probe_controller() при реальной пробе. */
                    uint16_t iob = (uint16_t)(bar4 & 0xFFE0u);
                    if (iob == 0) continue;

                    /* Enable bus mastering + I/O. */
                    uint32_t cmd = pci_read32(bus, dev, fn, 0x04);
                    pci_write32(bus, dev, fn, 0x04, cmd | 0x05u);

                    serial_puts("USBMOUSE: UHCI io=");
                    serial_puthex(iob);
                    serial_puts("\n");

                    /* 0.3.2: запомнить контроллер для hotplug-перескана */
                    if (g_scan_n < (int)(sizeof g_scan_io / sizeof g_scan_io[0]))
                        g_scan_io[g_scan_n++] = iob;

                    if (g_present) {
                        /* 0.3.0: мышь уже привязана на другом UHCI —
                         * этот контроллер НЕ трогаем (общие статические
                         * QH/TD: повторный um_probe_controller сломал бы
                         * расписание живой мыши), только запоминаем для
                         * перескана после выдёргивания. */
                        continue;
                    }
                    /* 0.3.0: мыши ещё нет — пробуем (не выходим: пустые
                     * контроллеры не мешают) */
                    (void)um_probe_controller(iob);
                }
                if (fn == 0 && !(pci_read8(bus, dev, fn, 0x0E) & 0x80u))
                    break;   /* single-function device */
            }
        }
    }

    serial_puts("USBMOUSE: no USB mouse found (PS/2 stays active)\n");
}

/* ── Разделение UHCI-контроллера с драйвером клавиатуры ─────────────────
 * Контракт — в usb_mouse.h. Все функции безопасны и когда мыши нет:
 * usb_uhci_kbd.c тогда просто забирает контроллер целиком. */
bool usb_mouse_uhci_busy(void)
{
    return g_io != 0;
}

uint16_t usb_mouse_uhci_io(void)
{
    return g_io;
}

/* [LOGS] ЭТАП 11: каким UHCI сейчас владеет драйвер клавиатуры
 * (usb_uhci_kbd.c)?  0 = никем.  usb_init() (mass storage) обязан
 * скипать ЭТОТ контроллер — раньше сбрасывал живую USB-клавиатуру
 * при поиске флешки в режиме sticklog. */
uint16_t usb_mouse_kbd_controller(void)
{
    return g_kbd_io;
}

int usb_mouse_uhci_port(void)
{
    /* «ЭТАП F1»: портов теперь до 4 (см. um_port_count) — было жёстко
     * «< 2»: мышь на порту 2/3 сообщалась как «-1», и драйвер клавиатуры
     * мог попробовать перечислить её порт. */
    return (g_port < 4u) ? (int)g_port : -1;
}

uint32_t usb_mouse_sched_chain(uint32_t qh_link)
{
    uint32_t old = g_sched_chain;

    g_sched_chain = qh_link ? qh_link : 1u;
    /* Расписание с QH мыши уже стоит (мышь привязана) — применяем
     * связь сразу; иначе её применит ближайший um_arm(). */
    if (g_present && g_io)
        um_qh.head_link = g_sched_chain;
    return old;
}
