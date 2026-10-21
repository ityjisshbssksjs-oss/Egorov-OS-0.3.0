/*
 * USB Mass Storage driver for Egorov OS — v1.0
 *
 * Supports:
 *   • PCI config-space scan for UHCI (class 0x0C, subclass 0x03, prog-if 0x00)
 *   • USB device reset + enumeration (control pipe, polling)
 *   • USB Mass Storage Class / Bulk-Only Transport (BOT)
 *   • SCSI READ(10) / WRITE(10) / READ CAPACITY / INQUIRY commands
 *
 * Architecture: 32-bit protected mode, no IRQs (polled), no dynamic memory
 * (all structures in BSS/static).  Tested with QEMU USB storage emulation.
 *
 * Limitations (demo-OS scope):
 *   - Only first UHCI controller is used
 *   - Only first full-speed device on port 0 or 1 is enumerated
 *   - No high-speed (EHCI) support (EHCI hand-off to UHCI companion is
 *     attempted via EECP legacy support)
 *   - Max 1 USB drive
 */

#include "usb.h"
#include "io.h"
#include "string.h"
#include "timer.h"      /* kdelay_us(): PIT-калиброванные задержки */

/* serial-лог ошибок MSD-пути (реализован в kernel.c). */
extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);

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
static uint16_t pci_read16(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off) {
    uint32_t d = pci_read32(bus, dev, fn, off & 0xFC);
    return (uint16_t)(d >> ((off & 2) * 8));
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
#define UHCI_SOF    0x0Cu   /* SOFMOD  (8-bit)  */
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
#define UHCI_CMD_RS     (1u << 0)
#define UHCI_CMD_HCRESET (1u << 1)
#define UHCI_CMD_GRESET  (1u << 2)
/* «ЭТАП F1»: без CF (Configure Flag) реальный UHCI считается
 * «не сконфигурированным» — маршрутизация портов/ответы HC не
 * гарантированы.  MAXP — 64-байтные пакеты full-speed.  Так ставят
 * SeaBIOS/Linux, наш usb_mouse.c и uhci_core.c.  QEMU бит CF не
 * моделирует — в эмуляторе работало и без него. */
#define UHCI_CMD_CF     (1u << 6)
#define UHCI_CMD_MAXP   (1u << 7)

/* ── USB descriptor / request constants ── */
#define USB_REQ_GET_DESCRIPTOR  0x06u
#define USB_REQ_SET_ADDRESS     0x05u
#define USB_REQ_SET_CONFIG      0x09u
#define USB_REQ_CLEAR_FEATURE   0x01u
#define USB_FEAT_HALT           0x00u

#define USB_DT_DEVICE    0x01u
#define USB_DT_CONFIG    0x02u

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

/* ── Frame list — 1024 entries, 4096-byte aligned ── */
static uint32_t    g_frame_list[1024] __attribute__((aligned(4096)));

/* Static TD/QH pools (enough for control + bulk transfers). */
/* «ЭТАП F1»: 40 TD (было 32) — чтение полного конфиг-дескриптора 255
 * байт при EP0 MPS=8 требует 1+32+1 = 34 TD; пул с запасом. */
#define TD_POOL_SIZE 40
#define QH_POOL_SIZE  4
static uhci_td_t   g_tds[TD_POOL_SIZE] __attribute__((aligned(16)));
static uhci_qh_t   g_qhs[QH_POOL_SIZE] __attribute__((aligned(16)));

/* Transfer buffer (512 bytes + setup packet room). */
/* выравнивание 4096 — UHCI TD не может пересекать границу
 * страницы 4К; 512+16 байт от выровненного начала всегда в одной. */
static uint8_t g_buf[512 + 16] __attribute__((aligned(4096)));

/* ── Globally visible device state ── */
usb_drive_t usb_drive;

/* ── Internal UHCI state ── */
static uint16_t g_io;      /* UHCI I/O base */
static uint8_t  g_devaddr; /* assigned USB device address */
static uint8_t  g_ep_in;   /* bulk-in endpoint */
static uint8_t  g_ep_out;  /* bulk-out endpoint */
static uint8_t  g_toggle_in;
static uint8_t  g_toggle_out;
static uint32_t g_cbw_tag;
/* «ЭТАП F1»: скорость устройства из PORTSC.LSD (бит 8) — LS-транзакции
 * требуют бита LS (бит 26) в ctrl_sts каждого TD; full-speed TD на
 * LS-линии не получает НИ ОДНОГО ответа (timeout/NAK).  Определяется
 * в uhci_port_reset(). */
static bool     g_ls;
/* «ЭТАП F1»: EP0 max packet size (bMaxPacketSize0) — размер чанков
 * дата-стадии контрольной передачи (8/16/32/64; до чтения дескриптора —
 * безопасные 8, как usb_mouse.c). */
static uint8_t  g_ep0_mps = 8;

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

/* ── Busy delay: PIT-калиброванная (kernel/timer.c; до timer_init()
 * работает консервативный фолбэк — не хуже прежнего цикла inb(0x80)). ── */
static void usb_delay(uint32_t us) {
    kdelay_us(us);
}

/* ── UHCI low-level helpers ── */
static void uhci_write16(uint16_t off, uint16_t v) { outw((uint16_t)(g_io + off), v); }
static uint16_t uhci_read16(uint16_t off)           { return inw((uint16_t)(g_io + off)); }
static void uhci_write32(uint16_t off, uint32_t v)  { outl((uint16_t)(g_io + off), v); }

static void uhci_reset(void) {
    uhci_write16(UHCI_CMD, UHCI_CMD_HCRESET);
    usb_delay(50000);
    uhci_write16(UHCI_CMD, 0);
    /* «ЭТАП F1»: бит HCRESET снимает сам контроллер; запись 0 в USBCMD
     * во время сброса часть чипов игнорирует — следующий RS терялся.
     * Ждём фактического снятия бита (паттерн usb_mouse.c/uhci_core.c). */
    int t = 1000;
    while ((inw((uint16_t)(g_io + UHCI_CMD)) & UHCI_CMD_HCRESET) && t-- > 0)
        usb_delay(1000);
}

static void uhci_init_framelist(void) {
    /* All frame-list entries point to the terminate bit (no QH/TD scheduled). */
    for (int i = 0; i < 1024; i++) g_frame_list[i] = 1; /* Terminate */
    uhci_write32(UHCI_FLBASE, (uint32_t)g_frame_list);
    uhci_write16(UHCI_FRNUM, 0);
    uhci_write16(UHCI_INTR, 0);        /* disable all interrupts */
    uhci_write16(UHCI_STS, 0x3F);      /* clear status bits */
    /* «ЭТАП F1»: RS|CF|MAXP — только RS означал «не сконфигурирован»
     * на реальном железе (см. комментарий у UHCI_CMD_CF). */
    uhci_write16(UHCI_CMD, UHCI_CMD_RS | UHCI_CMD_CF | UHCI_CMD_MAXP);
    usb_delay(1000);
}

/* ── Reset + enable a port; returns true if a device attached ── */
static bool uhci_port_reset(uint16_t port_reg) {
    uint16_t s = inw((uint16_t)(g_io + port_reg));
    if (!(s & UHCI_PORT_CONNECT)) return false;

    /* «ЭТАП 5 / #186»: дебаунс подключения ДО сброса (как usb_mouse.c /
     * uhci_core.c): 100 мс стабильности CCS — дребезг контактов срывал
     * перечисление на пустом месте.  Покрывает и перебор портов в
     * usb_init() — он идёт через эту же функцию. */
    {
        uint16_t s1 = inw((uint16_t)(g_io + port_reg));
        usb_delay(100000);                       /* 100 мс */
        uint16_t s2 = inw((uint16_t)(g_io + port_reg));
        if (((s1 ^ s2) & UHCI_PORT_CONNECT) != 0) {
            serial_puts("USBMSD: дребезг подключения - порт не сбрасываю\n");
            return false;
        }
        if (!(s2 & UHCI_PORT_CONNECT)) return false;
    }

    /* Clear change bits, assert RESET. */
    outw((uint16_t)(g_io + port_reg), UHCI_PORT_CCHANGE | UHCI_PORT_ECHANGE | UHCI_PORT_RESET);
    usb_delay(60000);  /* USB spec: ≥50 ms */
    outw((uint16_t)(g_io + port_reg), 0);
    /* «ЭТАП F1»: recovery после сброса ≥50 мс (TRSTRCY; было 5 мс —
     * реальное устройство ещё не готово к первому GET_DESCRIPTOR,
     * перечисление срывалось).  Так же делают usb_mouse.c/uhci_core.c. */
    usb_delay(50000);

    /* Enable the port. */
    outw((uint16_t)(g_io + port_reg), UHCI_PORT_ENABLE | UHCI_PORT_CCHANGE | UHCI_PORT_ECHANGE);
    usb_delay(2000);
    s = inw((uint16_t)(g_io + port_reg));
    /* «ЭТАП F1»: скорость устройства — PORTSC.LSD (бит 8) -> бит LS
     * в каждом TD (см. g_ls). */
    g_ls = (s & UHCI_PORT_LSD) != 0;
    return (s & UHCI_PORT_ENABLE) && (s & UHCI_PORT_CONNECT);
}

/* ── Execute a chain of TDs via a QH ── */
#define TD_CTRL_ACTIVE  (1u << 23)
#define TD_CTRL_IOC     (1u << 24)
#define TD_CTRL_LS      (1u << 26)  /* Low-speed */
#define TD_CTRL_3ERRS   (3u << 27)
#define TD_CTRL_SPD     (1u << 29)  /* Short Packet Detect */
#define TD_CTRL_STALLED (1u << 22)
#define TD_CTRL_DBUFERR (1u << 21)
#define TD_CTRL_CRCERR  (1u << 18)
#define TD_CTRL_ANY_ERR (TD_CTRL_STALLED | TD_CTRL_DBUFERR | TD_CTRL_CRCERR)

#define PID_SETUP  0x2Du
#define PID_IN     0x69u
#define PID_OUT    0xE1u

/* Token: pid[7:0] | devaddr[14:8] | endpoint[18:15] | D-toggle[19] | maxlen[31:21]
 * «ЭТАП F1» (критично): data-toggle живёт в слове TOKEN (бит 19), а НЕ
 * в ctrl_sts — там бит 19 это read-only NAK-статус.  Раньше тоггл писался
 * в ctrl_sts: дата-стадия реально уходила DATA0, и живые устройства
 * отклоняли дескрипторы (QEMU тогглы не проверяет — баг был невидим). */
#define TD_TOKEN(pid, addr, ep, tog, len) \
    ((uint32_t)(pid) | ((uint32_t)(addr) << 8) | ((uint32_t)(ep) << 15) | \
     ((tog) ? (1u << 19) : 0u) | (((uint32_t)(len) ? (uint32_t)(len) - 1u : 0x7FFu) << 21))

/* Schedule QH in every frame-list slot and poll for completion.
 * «ЭТАП F1» (критично): раньше QH вставлялся ТОЛЬКО в слот 0 — а тот
 * исполняется раз в 1024 кадра (~1.024 с) при таймауте 500 мс: передача
 * почти всегда отваливалась по таймауту.  Как usb_mouse.c v1.1 — во ВСЕ
 * слоты (честные ~1 кГц). */
static bool uhci_run_qh(uhci_qh_t *qh, int ntd) {
    qh->head_link = 1; /* terminate */
    for (int i = 0; i < 1024; i++)
        g_frame_list[i] = (uint32_t)qh | 2; /* QH pointer, not Terminate */
    /* «ЭТАП 5»: барьер — список кадров записан до начала опроса TD */
    __asm__ volatile("" ::: "memory");

    /* Wait for all TDs to complete (all inactive or error).
     * Чтение ctrl_sts — volatile: слово меняет контроллер, GCC -O2
     * мог вынести его из цикла (см. uhci_hw.h — там поля volatile). */
    int timeout = 500; /* 500 ms max */
    bool done = false;
    while (timeout-- > 0) {
        usb_delay(1000);
        bool all_done = true;
        for (int i = 0; i < ntd; i++) {
            if (*(volatile uint32_t *)&g_tds[i].ctrl_sts & TD_CTRL_ACTIVE) {
                all_done = false;
                break;
            }
        }
        if (all_done) { done = true; break; }
    }

    bool ok = done;

    /* Check for errors in any TD. */
    for (int i = 0; i < ntd; i++) {
        if (*(volatile uint32_t *)&g_tds[i].ctrl_sts & TD_CTRL_ANY_ERR) {
            ok = false;
            break;
        }
    }

    /* Remove from frame list. */
    for (int i = 0; i < 1024; i++) g_frame_list[i] = 1;
    return ok;
}

/* ── USB CONTROL transfer (setup + optional data + status) ──
 * Тогглы по спецификации USB: SETUP=DATA0 → дата=DATA1…→ статус=DATA1
 * (бит 19 — в слове TOKEN, см. комментарий у TD_TOKEN). */
static bool usb_control(uint8_t bmReqType, uint8_t bReq, uint16_t wVal,
                         uint16_t wIdx, uint16_t wLen, void *data) {
    memset(g_tds, 0, sizeof(g_tds));
    memset(g_qhs, 0, sizeof(g_qhs));

    /* «ЭТАП 5»: защита от рассинхрона аргументов — дата-стадия без буфера
     * не строится (все текущие вызовы с буфером, защита на будущее). */
    if (wLen > 512) {
        serial_puts("USBMSD: usb_control: wLen>512 - отклонено\n");
        return false;
    }
    if (wLen > 0 && data == NULL) {
        serial_puts("USBMSD: usb_control: wLen>0 без буфера - отклонено\n");
        return false;
    }

    /* Build SETUP packet in buffer. */
    uint8_t *setup = g_buf + 512; /* use tail of g_buf */
    setup[0] = bmReqType; setup[1] = bReq;
    setup[2] = (uint8_t)wVal;       setup[3] = (uint8_t)(wVal >> 8);
    setup[4] = (uint8_t)wIdx;       setup[5] = (uint8_t)(wIdx >> 8);
    setup[6] = (uint8_t)wLen;       setup[7] = (uint8_t)(wLen >> 8);

    uint32_t ls = g_ls ? TD_CTRL_LS : 0u;
    int td = 0;

    /* SETUP TD — DATA0 */
    g_tds[td].link     = (uint32_t)&g_tds[td+1]; /* depth-first, no Vf */
    g_tds[td].ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS | ls;
    g_tds[td].token    = TD_TOKEN(PID_SETUP, g_devaddr, 0, 0, 8);
    g_tds[td].buffer   = (uint32_t)setup;
    td++;

    /* DATA stage (optional).
     * «ЭТАП F1»: буфер — ВСЕГДА g_buf (выровнен на 4096, TD не пересекает
     * страницу): раньше OUT-стадия указывала на буфер вызывающего (стек!
     * с произвольным выравниванием) — на реальном железе ошибка DMA.
     * Чанк — EP0 MPS устройства (не 64): у LS/старых устройств EP0=8,
     * большие чанки рвутся коротким пакетом. */
    uint8_t toggle = 1; /* DATA1 for first data TD */
    if (wLen > 0 && data) {
        uint8_t pid = (bmReqType & 0x80) ? PID_IN : PID_OUT;
        if (pid == PID_OUT) memcpy(g_buf, data, wLen);
        else memset(g_buf, 0, wLen);   /* «ЭТАП 5»: детерминированный хвост при short packet (паттерн usb_mouse v1.5) */
        uint32_t rem = wLen;
        uint8_t *ptr = g_buf;
        uint8_t mps = g_ep0_mps;
        if (mps < 8)  mps = 8;
        if (mps > 64) mps = 64;
        while (rem > 0) {
            /* «ЭТАП F1»: защита пула TD — оставляем слот под STATUS */
            if (td >= TD_POOL_SIZE - 1) {
                serial_puts("USBMSD: TD pool overflow в DATA-стадии\n");
                return false;
            }
            uint32_t chunk = (rem > (uint32_t)mps) ? mps : rem;
            g_tds[td].link     = (uint32_t)&g_tds[td+1];
            g_tds[td].ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS | ls;
            g_tds[td].token    = TD_TOKEN(pid, g_devaddr, 0, toggle, chunk);
            g_tds[td].buffer   = (uint32_t)ptr;
            ptr += chunk; rem -= chunk; toggle ^= 1; td++;
        }
    }

    /* STATUS TD — opposite direction, DATA1 */
    uint8_t status_pid = (bmReqType & 0x80) ? PID_OUT : PID_IN;
    g_tds[td].link     = 1; /* Terminate */
    g_tds[td].ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS | TD_CTRL_IOC | ls;
    g_tds[td].token    = TD_TOKEN(status_pid, g_devaddr, 0, 1, 0); /* DATA1 */
    g_tds[td].buffer   = 0;
    int ntd = td + 1;

    /* Link TD chain depth-first: td[0]→td[1]→...→td[ntd-1] */
    for (int i = 0; i < ntd - 1; i++)
        g_tds[i].link = (uint32_t)&g_tds[i+1]; /* depth-first, breadth bit clear */
    g_tds[ntd-1].link = 1;

    /* «ЭТАП 5»: компиляторный барьер — поля TD здесь НЕ volatile, и без
     * барьера GCC -O2 мог поставить запись elem_link (ВООРУЖАЕТ цепочку
     * TD для HC) раньше токенов/буферов (паттерн um_barrier usb_mouse.c,
     * там это ловили на реальном железе). */
    __asm__ volatile("" ::: "memory");
    g_qhs[0].head_link = 1;
    g_qhs[0].elem_link = (uint32_t)&g_tds[0];

    bool ok = uhci_run_qh(&g_qhs[0], ntd);
    /* [#481] [LOGS] ЭТАП 11: копирование IN-данных — ПОСЛЕ передачи (было
     * ДО запуска: контрольная передача «успешно» возвращала НУЛИ, и
     * перечисление MSD на UHCI падало на первом GET_DESCRIPTOR). */
    if (ok && wLen > 0 && data && (bmReqType & 0x80u))
        memcpy(data, g_buf, wLen);
    return ok;
}

/* ── Bulk transfer (IN or OUT) ──
 * Тоггл — в TOKEN (бит 19); LS-бит — из скорости устройства.
 * [#482] [LOGS] ЭТАП 11: у фаз BOT СВОИ буферы. Раньше OUT-фаза копировала
 * буфер вызывающего в ОБЩИЙ g_buf — и CBW (31 байт) затирал ПЕРВЫЕ 31
 * байта данных сектора, подготовленных в том же g_buf: каждая запись
 * на флешку начиналась с мусора «USBC…» (двойная порча: FAT и данные).
 * Теперь: OUT — TDs смотрят НАПРЯМУЮ в буфер вызывающего (он обязан
 * быть DMA-безопасным: g_cbw/g_buf — выровнены), IN — стадия через
 * g_buf с копией вызывающему (стек-буферы SCSI-ответов безопасны). */
static bool usb_bulk(uint8_t ep, bool is_in, void *buf, uint32_t len, uint8_t *toggle) {
    memset(g_tds, 0, sizeof(g_tds));
    memset(g_qhs, 0, sizeof(g_qhs));

    uint8_t pid = is_in ? PID_IN : PID_OUT;

    /* «ЭТАП F1»: граница DMA-буфера 512 байт (защита от переполнения). */
    if (len == 0 || len > 512) {
        serial_puts("USBMSD: usb_bulk: некорректная длина - отклонено\n");
        return false;
    }
    uint8_t *dma;
    if (is_in) {
        dma = g_buf;                      /* стадия: TDs → g_buf → копия */
        memset(g_buf, 0, len);            /* детерминированный хвост */
    } else {
        dma = (uint8_t *)buf;             /* уже готовый буфер (см. #482) */
    }

    uint32_t ls = g_ls ? TD_CTRL_LS : 0u;
    uint32_t rem = len, offset = 0;
    int td = 0;
    while (rem > 0 && td < TD_POOL_SIZE - 1) {
        uint32_t chunk = (rem > 512) ? 512u : rem;
        g_tds[td].link     = 1;
        g_tds[td].ctrl_sts = TD_CTRL_ACTIVE | TD_CTRL_3ERRS | ls;
        g_tds[td].token    = TD_TOKEN(pid, g_devaddr, ep & 0x0Fu, *toggle, chunk);
        g_tds[td].buffer   = (uint32_t)(dma + offset);
        offset += chunk; rem -= chunk; *toggle ^= 1; td++;
    }
    for (int i = 0; i < td - 1; i++) g_tds[i].link = (uint32_t)&g_tds[i+1];
    g_tds[td-1].link = 1;

    __asm__ volatile("" ::: "memory");   /* «ЭТАП 5»: TD готовы до вооружения QH */
    g_qhs[0].head_link = 1;
    g_qhs[0].elem_link = (uint32_t)&g_tds[0];

    bool ok = uhci_run_qh(&g_qhs[0], td);
    if (is_in && ok) memcpy(buf, dma, len);
    return ok;
}

/* ── USB Mass Storage / BOT ── */
#define CBW_SIG 0x43425355u
#define CSW_SIG 0x53425355u

typedef struct __attribute__((packed)) {
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataTransferLength;
    uint8_t  bmFlags;           /* 0x80 = device-to-host */
    uint8_t  bLUN;
    uint8_t  bCBLength;
    uint8_t  CB[16];
} cbw_t;

typedef struct __attribute__((packed)) {
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataResidue;
    uint8_t  bStatus;
} csw_t;

/* [#482] СВОИ буферы фаз BOT: CBW — выровнен (TD не пересекает 4К);
 * SCSI-ответы (INQUIRY/CAPACITY) — тоже (стек-буферы убраны из DMA). */
static uint8_t g_cbw[64] __attribute__((aligned(64)));
static uint8_t g_csw[64] __attribute__((aligned(64)));
static uint8_t g_scsi_resp[64] __attribute__((aligned(64)));

static bool bot_command(const uint8_t *cdb, uint8_t cdb_len,
                         bool data_in, void *data, uint32_t data_len) {
    /* ── CBW ── */
    memset(g_cbw, 0, 31);
    {
        cbw_t *cbw = (cbw_t *)g_cbw;
        cbw->dSignature = CBW_SIG;
        cbw->dTag       = ++g_cbw_tag;
        cbw->dDataTransferLength = data_len;
        cbw->bmFlags    = data_in ? 0x80u : 0x00u;
        cbw->bLUN       = 0;
        cbw->bCBLength  = cdb_len;
        memcpy(cbw->CB, cdb, cdb_len);
    }

    if (!usb_bulk(g_ep_out, false, g_cbw, 31, &g_toggle_out))
        return false;

    /* ── Data phase ── */
    if (data_len > 0) {
        if (!usb_bulk(data_in ? g_ep_in : g_ep_out, data_in,
                      data, data_len, data_in ? &g_toggle_in : &g_toggle_out))
            return false;
    }

    /* ── CSW ── */
    memset(g_csw, 0, 13);
    if (!usb_bulk(g_ep_in, true, g_csw, 13, &g_toggle_in))
        return false;
    {
        csw_t *csw = (csw_t *)g_csw;
        if (csw->dSignature != CSW_SIG || csw->dTag != g_cbw_tag) return false;
        return (csw->bStatus == 0);
    }
}

/* SCSI INQUIRY */
static bool scsi_inquiry(void) {
    uint8_t cdb[6] = {0x12, 0, 0, 0, 36, 0};
    memset(g_scsi_resp, 0, sizeof(g_scsi_resp));
    if (!bot_command(cdb, 6, true, g_scsi_resp, 36)) return false;

    /* Vendor (bytes 8-15) */
    for (int i = 0; i < 8; i++)
        usb_drive.vendor[i] = (char)(g_scsi_resp[8+i] >= 0x20 ? g_scsi_resp[8+i] : ' ');
    usb_drive.vendor[8] = 0;

    /* Product (bytes 16-31) */
    for (int i = 0; i < 16; i++)
        usb_drive.product[i] = (char)(g_scsi_resp[16+i] >= 0x20 ? g_scsi_resp[16+i] : ' ');
    usb_drive.product[16] = 0;

    return true;
}

/* SCSI READ CAPACITY(10) */
static bool scsi_read_capacity(void) {
    uint8_t cdb[10] = {0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    memset(g_scsi_resp, 0, sizeof(g_scsi_resp));
    if (!bot_command(cdb, 10, true, g_scsi_resp, 8)) return false;
    uint32_t last_lba = ((uint32_t)g_scsi_resp[0] << 24) | ((uint32_t)g_scsi_resp[1] << 16) |
                        ((uint32_t)g_scsi_resp[2] << 8)  |  (uint32_t)g_scsi_resp[3];
    usb_drive.sectors = last_lba + 1;
    return true;
}

bool usb_read_sector(uint32_t lba, void *buf512) {
    if (!usb_drive.present) return false;
    uint8_t cdb[10] = {
        0x28, 0,
        (uint8_t)(lba >> 24), (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),  (uint8_t)(lba),
        0, 0, 1, 0
    };
    return bot_command(cdb, 10, true, buf512, 512);
}

bool usb_write_sector(uint32_t lba, const void *buf512) {
    if (!usb_drive.present) return false;
    uint8_t cdb[10] = {
        0x2A, 0,
        (uint8_t)(lba >> 24), (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),  (uint8_t)(lba),
        0, 0, 1, 0
    };
    /* bot_command needs non-const; copy to g_buf first. */
    memcpy(g_buf, buf512, 512);
    return bot_command(cdb, 10, false, g_buf, 512);
}

/* ── Enumerate USB device on the given port ── */
static bool usb_enumerate(void) {
    g_devaddr   = 0;
    g_toggle_in = 0; g_toggle_out = 0;
    g_cbw_tag   = 0;
    g_ep0_mps   = 8;    /* «ЭТАП F1»: уточнится из bMaxPacketSize0 */

    /* GET_DESCRIPTOR (Device, length=8) at address 0 to learn max packet size.
     * «ЭТАП F1»: до 3 попыток с паузой 20 мс (как usb_mouse.c/uhci_ctl.c) —
     * реальное устройство после ресета может не ответить на самый первый
     * запрос (раньше одна неудача срывала перечисление). */
    uint8_t dev_desc[18];
    memset(dev_desc, 0, sizeof(dev_desc));
    bool got8 = false;
    for (int att = 0; att < 3 && !got8; att++) {
        /* «ЭТАП 5 / #295»: мусорный ответ (нулевой/чужой bDescriptorType)
         * считается НЕудачей — ретрай, а не парсинг случайных байтов */
        got8 = usb_control(0x80, USB_REQ_GET_DESCRIPTOR,
                           (USB_DT_DEVICE << 8), 0, 8, dev_desc) &&
               dev_desc[1] == USB_DT_DEVICE;
        if (!got8) usb_delay(20000);
    }
    if (!got8) {
        serial_puts("USBMSD: GET_DESCRIPTOR(8) на адресе 0 не удался\n");
        return false;
    }

    /* «ЭТАП 5 / #204/#294»: bMaxPacketSize0 клампится по скорости
     * устройства (uhci_mps_fix выше): LS -> строго 8, FS -> 8/16/32/64,
     * мусор (0,1,3,...) -> безопасные 8. */
    g_ep0_mps = uhci_mps_fix(dev_desc[7], g_ls);

    /* SET_ADDRESS to 1.
     * «ЭТАП 5 / #232/#233» (проверено): счётчика/инкремента адресов здесь
     * НЕТ и утечки нет — MSD-стек шлёт фиксированный адрес 1 и работает
     * одноразово (usb_init() при загрузке, hotplug-реэнumerации в этом
     * файле не существует).  Битмапа адресов с возвратом сделана там, где
     * реальный hotplug: usb_mouse.c (адрес 2 зарезервирован за uhci_ctl.c). */
    if (!usb_control(0x00, USB_REQ_SET_ADDRESS, 1, 0, 0, 0))
        return false;
    g_devaddr = 1;
    /* «ЭТАП F1»: recovery ≥2 мс после SET_ADDRESS, лучше 20 мс
     * (как usb_mouse.c): было 5 — устройство могло пропустить след. запрос. */
    usb_delay(20000);

    /* GET full Device Descriptor. */
    if (!usb_control(0x80, USB_REQ_GET_DESCRIPTOR,
                     (USB_DT_DEVICE << 8), 0, 18, dev_desc)) {
        return false;
    }
    /* «ЭТАП 5 / #295»: тип полного дескриптора устройства */
    if (dev_desc[1] != USB_DT_DEVICE) {
        serial_puts("USBMSD: dev18: неверный bDescriptorType\n");
        return false;
    }

    /* GET Configuration Descriptor (short first). */
    uint8_t cfg_hdr[9];
    if (!usb_control(0x80, USB_REQ_GET_DESCRIPTOR,
                     (USB_DT_CONFIG << 8), 0, 9, cfg_hdr)) {
        return false;
    }
    /* «ЭТАП 5 / #295»: тип конфиг-дескриптора обязан быть CONFIG */
    if (cfg_hdr[1] != USB_DT_CONFIG) {
        serial_puts("USBMSD: cfg9: неверный bDescriptorType\n");
        return false;
    }
    uint16_t total_len = (uint16_t)((uint16_t)cfg_hdr[2] | ((uint16_t)cfg_hdr[3] << 8));
    /* «ЭТАП 5 / #296»: границы wTotalLength; больше 255 Б не читаем
     * (стек-буфер cfg_full — 255): усечение ЛОГИРУЕТСЯ, парсинг остаётся
     * строго в пределах прочитанного. */
    if (total_len < 9) {
        serial_puts("USBMSD: wTotalLength<9 - мусор\n");
        return false;
    }
    if (total_len > 255) {
        serial_puts("USBMSD: wTotalLength>255 - усечён до 255\n");
        total_len = 255;
    }

    uint8_t cfg_full[255];
    if (!usb_control(0x80, USB_REQ_GET_DESCRIPTOR,
                     (USB_DT_CONFIG << 8), 0, total_len, cfg_full)) {
        return false;
    }

    /* Walk descriptors to find MSC interface + bulk endpoints. */
    bool msc_found = false;
    g_ep_in = 0; g_ep_out = 0;
    uint8_t config_val = cfg_hdr[5];
    int pos = 0;
    while (pos + 1 < (int)total_len) {   /* «ЭТАП 5 / #296b»: заголовок дескриптора целиком в данных */
        uint8_t dlen  = cfg_full[pos];
        uint8_t dtype = cfg_full[pos+1];
        if (dlen < 2) break;
        /* «ЭТАП 5 / #296b»: дескриптор ОБРЕЗАН границей прочитанного —
         * дальше только мусор/за пределами буфера (pos=248, dlen=9 читал
         * бы cfg_full[255..257] — ООВ стека). Как uhci_ctl.c: p+len>total. */
        if (pos + dlen > (int)total_len) break;

        if (dtype == 0x04 && dlen >= 9) { /* Interface descriptor — «ЭТАП 5 / #296b»: len>=9 — граница полей */
            /* Check for Mass Storage class (0x08), SCSI (0x06), BOT (0x50). */
            if (cfg_full[pos+5] == 0x08 &&
                cfg_full[pos+6] == 0x06 &&
                cfg_full[pos+7] == 0x50) {
                msc_found = true;
                usb_drive.iface = cfg_full[pos+2];
            }
        } else if (dtype == 0x05 && dlen >= 7 && msc_found) { /* Endpoint descriptor — «ЭТАП 5 / #296b»: len>=7 */
            uint8_t ep_addr = cfg_full[pos+2];
            uint8_t ep_type = cfg_full[pos+3] & 0x03u;
            if (ep_type == 0x02) { /* Bulk */
                if (ep_addr & 0x80u) g_ep_in  = ep_addr & 0x0Fu;
                else                  g_ep_out = ep_addr & 0x0Fu;
            }
        }
        pos += dlen;
    }

    if (!msc_found || g_ep_in == 0 || g_ep_out == 0) return false;

    /* SET_CONFIGURATION — «ЭТАП 5 / #297»: отказ ФАТАЛЕН (устройство не
     * сконфигурировано, bulk-точки не работают). */
    if (!usb_control(0x00, USB_REQ_SET_CONFIG, config_val, 0, 0, 0))
        return false;
    usb_delay(5000);

    /* SCSI: wait for device-ready with a few retries. */
    for (int retry = 0; retry < 5; retry++) {
        if (scsi_inquiry()) break;
        usb_delay(200000);
    }
    if (!scsi_read_capacity()) return false;

    usb_drive.present   = true;
    usb_drive.bulk_in   = g_ep_in;
    usb_drive.bulk_out  = g_ep_out;
    return true;
}

/* ── PCI scan for UHCI host controller ── */
void usb_init(void) {
    memset(&usb_drive, 0, sizeof(usb_drive));
    g_io = 0;

    /* Scan all PCI buses — «ЭТАП F1»: все 256 (было 0..7: контроллер
     * за PCI-мостом на шине 8+ не находился никогда). */
    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t dev = 0; dev < 32; dev++) {
            for (uint8_t fn = 0; fn < 8; fn++) {
                uint32_t id = pci_read32((uint8_t)bus, dev, fn, 0);
                if ((id & 0xFFFFu) == 0xFFFFu) {
                    if (fn == 0) goto next_dev;
                    continue;
                }
                uint8_t  class_code = pci_read8((uint8_t)bus, dev, fn, 0x0B);
                uint8_t  subclass   = pci_read8((uint8_t)bus, dev, fn, 0x0A);
                uint8_t  prog_if    = pci_read8((uint8_t)bus, dev, fn, 0x09);

                /* UHCI: class=0x0C, sub=0x03, prog-if=0x00 */
                if (class_code == 0x0C && subclass == 0x03 && prog_if == 0x00) {
                    /* [#453] контроллер ЗАНЯТ UHCI-мышью — не трогаем:
                     * uhci_reset() здесь убивал живую мышь
                     * (usb_mouse_init отработал раньше, kernel.c).
                     * [LOGS] ЭТАП 11: и контроллер ЖИВОЙ UHCI-клавиатуры
                     * (usb_uhci_kbd.c) — usb_init() вызывается в sticklog
                     * ПОСЛЕ неё; сброс глушил клавиатуру на весь сеанс. */
                    {
                        extern uint16_t usb_mouse_uhci_io(void);
                        extern uint16_t usb_mouse_kbd_controller(void);
                        uint32_t bar4_busy = pci_read32((uint8_t)bus, dev, fn, 0x20u);
                        if (bar4_busy & 1u) {
                            uint16_t bio = (uint16_t)(bar4_busy & 0xFFE0u);
                            if (usb_mouse_uhci_io() == bio) {
                                serial_puts("USB: UHCI занят мышью - скип (sticklog)\n");
                                continue;
                            }
                            if (usb_mouse_kbd_controller() == bio) {
                                serial_puts("USB: UHCI занят клавиатурой - скип (sticklog)\n");
                                continue;
                            }
                        }
                    }
                    /* BAR4 contains the UHCI I/O base (bit 0 = I/O space). */
                    uint32_t bar4 = pci_read32((uint8_t)bus, dev, fn, 0x20);
                    if (!(bar4 & 1u)) continue; /* not I/O bar */
                    g_io = (uint16_t)(bar4 & 0xFFE0u);
                    if (g_io == 0) continue;

                    /* Enable bus mastering + I/O. */
                    uint32_t cmd = pci_read32((uint8_t)bus, dev, fn, 0x04);
                    pci_write32((uint8_t)bus, dev, fn, 0x04, cmd | 0x05u);

                    /* Reset and start UHCI. */
                    uhci_reset();
                    uhci_init_framelist();

                    /* «ЭТАП F1»: порты — до первого несуществующего
                     * PORTSC (регистра числа портов у UHCI нет; Linux
                     * uhci-hcd делает так): у PIIX3 их 2, на некоторых
                     * платах 4.  Было жёстко 0/1 — порты 3-4 пропадали
                     * молча.
                     * Ghost-фикс: 0xFFFF ИЛИ reserved-биты 13-15 —
                     * порта нет (QEMU piix3 на несуществующих портах
                     * возвращает 0xFF7F с CCS=1 — фантомы). */
                    for (int portno = 0; portno < 4; portno++) {
                        uint16_t preg = (uint16_t)(UHCI_PORT0 + 2u * (uint32_t)portno);
                        uint16_t pst = uhci_read16(preg);
                        if (pst == 0xFFFFu || (pst & 0xE000u) != 0u)
                            break;              /* дальше портов нет (или фантом) */
                        if (uhci_port_reset(preg)) {
                            if (usb_enumerate()) {
                                /* [LOGS] ЭТАП 11: контроллер теперь
                                 * ВЛАДЕЕТСЯ MSD — rescan мыши (и её
                                 * mute/restore портов) больше не трогает
                                 * его: живой USB-адрес флешки священен. */
                                extern void usb_mouse_set_msd_controller(uint16_t);
                                usb_mouse_set_msd_controller(g_io);
                                serial_puts("USBMSD: контроллер закреплён за MSD (rescan мыши не трогает)\n");
                                return;
                            }
                        }
                    }
                    /* No device found on this controller; stop the HC. */
                    uhci_write16(UHCI_CMD, 0);
                    g_io = 0;
                }
                if (fn == 0 && !(pci_read8((uint8_t)bus, dev, fn, 0x0E) & 0x80u))
                    break; /* single-function device */
            }
            next_dev:;
        }
    }
}
