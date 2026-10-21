/*
 * mouse_hw.c — MOUSEHW: новый (с нуля) драйвер компьютерной мыши для
 * РЕАЛЬНОГО ЖЕЛЕЗА — дополнение к работающей связке UHCI(v1.5)+PS/2.
 *
 * ЗАЧЕМ: у современных ПК (Intel Haswell 2013+, AMD FM2+) USB-мышь сидит
 * на xHCI-контроллере (PCI class 0x0C / subclass 0x03 / prog-if 0x30).
 * Работающий драйвер usb_mouse.c — UHCI-only и такой контроллер не видит;
 * он сознательно НЕ ТРОНУТ.  Этот модуль добавляет недостающий транспорт.
 *
 * ПРИНЦИП ДОПОЛНЕНИЯ:
 *   • свой PCI-скан, свои кольца, свои контексты — общих данных нет;
 *   • если xHCI-контроллера нет (QEMU piix3-usb-uhci), модуль после
 *     скана немедленно выходит — поведение системы не меняется вовсе;
 *   • указатель отдаётся MOUSEHW только когда usb_mouse неактивен:
 *     ps2_poll() опрашивает usb_mouse → mouse_hw → PS/2 (в этом порядке);
 *   • при сбоях драйвер деактивируется, PS/2 возвращается автоматически.
 *
 * Все раскладки битов xHCI сверены с Linux drivers/usb/host/xhci.h (v6.6)
 * и с реализацией QEMU hw/usb/hcd-xhci.c:
 *   • slot ctx (struct xhci_slot_ctx Linux v6.6): dword0 = route [19:0] +
 *     speed [23:20] + MTT [25] + Hub [26] + context entries [31:27];
 * dword1 = RHPort [23:16] + Max Exit Latency [15:0] (двигал
 *     RHPort в [15:8] по неверной цитате «Linux p<<8» — номер порта
 *     уходил в MEL, RHPort=0, Address Device отвечал cc=0x11 Parameter);
 *   • EP ctx: dword0 = state/Mult/MaxPStreams/Interval [23:16]; dword1
 *     (ep_info2) = Max Packet [31:16] + Max Burst [15:8] + EP type [5:3]
 * + Error Count [2:1] (двигал CErr в [31:27] по неверной
 *     цитате «ERROR_COUNT(p)<<27» — мусор 3<<27 попал в MaxPacket и
 *     Address Device снова отвечал cc=0x11); dword2/3 = deq|DCS[0],
 *     dword4 = ESIT [31:16]/avg TRB [15:0] tx_info;
 *   • EP type INT_IN=7; TRB: NORMAL=1 SETUP=2 DATA=3 STATUS=4 LINK=6,
 *     ENABLE_SLOT=9 ADDR_DEV=11 CONFIG_EP=12 EVAL_CTX=13,
 *     события TRANSFER=32 CMD_COMPL=33 PORT_CHANGE=34;
 *   • Transfer Event: CC в dword2 [31:24], остаток [23:0], slot [31:24] /
 *     EP-id [23:16] в dword3; Command Completion: ptr=command TRB,
 *     CC в dword2 [31:24], slot [31:24] dword3;
 *   • EP indexation = индекс в контексте: EP0=1, EP1 OUT=2, EP1 IN=3 —
 *     он же номер doorbell (QEMU: eps[epid-1]);
 *   • PORTSC: CCS[0] PED[1] PR[4] PP[9] speed[13:10] (1=FULL 2=LOW
 *     3=HIGH 4=SUPER), change-биты W1C 17..23; stride набора портов 16;
 *   • адреса: op=BAR+CAPLENGTH, порты op+0x400, runtime=BAR+RTSOFF
 *     (IR0 = +0x20: IMAN/ERSTSZ/ERSTBA/ERDP), doorbells=BAR+DBOFF;
 *   • СМЕЩЕНИЯ ВНУТРИ op (критично!): USBCMD=+0x00 USBSTS=+0x04
 *     DNCTRL=+0x14 CRCR=+0x18 (64-бит) DCBAAP=+0x30 (64-бит)
 *     CONFIG=+0x38 — сверено со struct xhci_op_regs (Linux v6.6) и
 *     свитчами xhci_oper_read/write QEMU 2.0.  Неверные смещения здесь
 *     НЕ дают ошибки: записи молча уходят мимо (0x10 — reserved,
 *     CONFIG попадает в старшую половину CRCR, кольцо стартует с 0) —
 *     симптом ровно тот, что был: «enable slot failed cc=0»;
 *   • QEMU-особенности: ERDP обязан оставаться внутри event ring
 *     (заворачиваем), producer cycle state стартует с 1, дверной звонок
 *     игнорируется пока USBCMD.RS=0 → RS ставим до первого звонка;
 *   • ERST-запись = 16 байт: addr(0-7) | rsvd(8-11) | size(12-15) — размер
 *     НЕ по смещению 16 (иначе QEMU делает xhci_die и event ring не
 *     активируется — симптом: «enable slot failed cc=0»);
 *   • EP-контекст: DCS (deq[0]) обязан совпадать с cycle-битом первого
 *     TRB кольца (=1): QEMU epctx->ring.ccs = ctx[2] & 1 — иначе кольцо
 *     молчит («ctrl timeout»);
 *   • DCBAA[slot] обязан указывать на выходной контекст ДО Address
 *     Device (QEMU пишет/читает контексты по этому адресу); DCBAA[0] =
 *     адрес массива scratchpad-указателей.
 *
 * Протокол мыши: HID boot (SET_PROTOCOL(boot), SET_IDLE) — отчёты
 * [btn,dx,dy(,dz)]; авто-детект абсолютного планшета по report
 * descriptor (INPUT Data|Var|Absolute, 16 бит) с масштабированием
 * LOGICAL_MAX на весь framebuffer — как у рабочего UHCI-драйвера v1.5,
 * поэтому ощущение скорости курсора одинаковое.
 *
 * Ограничения (скоуп дополнения):
 *   • только первый найденный xHCI-контроллер и одна HID-мышь;
 *   • SuperSpeed (скорость 4) мыши пропускаются — их не существует
 *     практически (пишется в serial);
 *   • колесо и средняя кнопка игнорируются (mouse_state_t — 2 кнопки).
 *
 * Логи: префикс «MOUSEHW:» в COM1 (-serial stdio).
 */

#include "mouse_hw.h"
#include "ehci.h"      /* [#412] ehci_portmap() для полной карты портов */
#include "usbmap.h"
#include "usbdev.h"
#include "version.h"   /* [#473] номер СБОРКИ в заголовке КАРТЫ ГНЕЗД */
#include "io.h"
#include "string.h"
#include "timer.h"   /* F3: kdelay_us — калиброванная PIT-задержка */

/* [#415] карта UHCI: владельцы портов (драйверы UHCI живут в других файлах) */
extern uint16_t usb_mouse_uhci_io(void);    /* usb_mouse.c: I/O-база мыши */
extern int usb_mouse_uhci_port(void);       /* usb_mouse.c: 0-базированный порт мыши */
extern int uhci_kbd_bound_port(uint16_t io); /* uhci_kbd.c: порт клавиатуры */

/* COM1-лог реализован в kernel.c (QEMU -serial stdio). */
extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);

/* ── PCI config space (I/O CF8/CFC) ───────────────────────────────────────── */
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
/* [#430] 16-битная запись в конфиг-пространство PCI: USBLEGSUP (UHCI,
 * смещение 0xC0) — 16-битный регистр, dword-запись затронула бы 0xC2+. */
static void pci_write16(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off, uint16_t v) {
    uint32_t addr = (1u << 31) | ((uint32_t)bus << 16) |
                    ((uint32_t)dev << 11) | ((uint32_t)fn << 8) | (off & 0xFC);
    outl(PCI_ADDR, addr);
    outw((uint16_t)(PCI_DATA + (off & 2u)), v);
}
static uint8_t pci_read8(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off) {
    uint32_t d = pci_read32(bus, dev, fn, off & 0xFC);
    return (uint8_t)(d >> ((off & 3) * 8));
}

/* ── MMIO helpers (32-битное ядро, identity map, BAR ниже 4 ГБ) ───────────── */
static void mmw32(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static uint32_t mmr32(uint32_t a)         { return *(volatile uint32_t *)a; }

/* когерентность кэша CPU с DMA — гигиена DMA, остаётся (wbinvd
 * выталкивает записи в ОЗУ: контроллер обязан видеть выгруженную
 * память; в QEMU TCG кэша нет — там он ничего не меняет).
 * уточняет диагноз фото: отказ xHCI «Address Device cc=0x11
 * (Parameter)» был НЕ кэшем, а ДВУМЯ неверными полями контекстов
 * (SLOT_ROOT_PORT p<<8 вместо p<<16 и EP_CERR3 3<<27 вместо 3<<1) —
 * дампы показали их однозначно: slot1=0x00000300 (RHPort=0,
 * MEL=768) и ep0_1=0x18080020 (MaxPacket=0x1808 вместо 8). */
static void dma_sync(void) { __asm__ volatile("wbinvd" ::: "memory"); }
static void mmw64(uint32_t a, uint64_t v) {
    mmw32(a, (uint32_t)v);
    mmw32(a + 4, (uint32_t)(v >> 32));
}
static uint32_t vphys(const void *p) { return (uint32_t)p; }

/* ── Capability registers (BAR0-относительные) ────────────────────────────── */
#define XHC_CAP_HCSPARAMS1 0x04u
#define XHC_CAP_HCSPARAMS2 0x08u
#define XHC_CAP_HCCPARAMS1 0x10u
#define XHC_CAP_DBOFF      0x14u
#define XHC_CAP_RTSOFF     0x18u

/* ── Operational registers ─────────────────────────────────────────────────── */
#define XHC_OP_USBCMD   0x00u
#define XHC_OP_USBSTS   0x04u
#define XHC_OP_DNCTRL   0x14u
#define XHC_OP_CRCR     0x18u
#define XHC_OP_DCBAAP   0x30u
#define XHC_OP_CONFIG   0x38u
#define XHC_OP_PORT0    0x400u   /* PORTSC1; stride набора портов = 16 байт */

#define USBCMD_RS       (1u << 0)
#define USBCMD_HCRST    (1u << 1)
#define USBSTS_HCH      (1u << 0)
#define USBSTS_CNR      (1u << 11)

/* PORTSC (сверено с Linux v6.6 и QEMU) */
#define PORTSC_CCS      (1u << 0)
#define PORTSC_PED      (1u << 1)
#define PORTSC_PR       (1u << 4)
#define PORTSC_PP       (1u << 9)
#define PORTSC_CHG      (0x7Fu << 17)      /* CSC..CEC, запись 1 = очистить */
#define PORTSC_WPR      (1u << 31)         /* Warm Port Reset (USB3-линк) */
#define PORTSC_SPEED_SH 10
/* Код скорости в PORTSC по спецификации xHCI (7.4.4, биты 13:10):
 * 1 = Full-Speed (12M), 2 = Low-Speed (1.5M), 3 = High-Speed (480M),
 * 4 = SuperSpeed (5G). константы были перепутаны местами
 * (LOW=1/FULL=2) — реальная низкоскоростная мышь (фото: порт 3
 * speed=2 = Low) получала mps EP0 = 64 вместо 8.  В Slot Context Speed
 * значение из PORTSC уходит как есть (там было и остаётся верно). */
#define PORT_SPEED_FULL 1
#define PORT_SPEED_LOW  2
#define PORT_SPEED_HIGH 3
#define PORT_SPEED_SUPER 4

/* Interrupter 0 (runtime base + 0x20) */
#define XHC_IR0         0x20u
#define XHC_IR_IMAN     0x00u
#define XHC_IR_ERSTSZ   0x08u
#define XHC_IR_ERSTBA   0x10u
#define XHC_IR_ERDP     0x18u
#define ERDP_EHB        (1u << 3)

/* ── TRB (16 байт) ─────────────────────────────────────────────────────────── */
typedef struct {
    uint64_t param;
    uint32_t status;
    uint32_t control;
} mhw_trb_t;

#define TRB_CYCLE      (1u << 0)
#define TRB_TC         (1u << 1)   /* Link Toggle Cycle */
#define TRB_IOC        (1u << 5)
#define TRB_IDT        (1u << 6)   /* Immediate Data (SETUP) */
#define TRB_TYPE(t)    ((t) << 10)

#define TRB_TR_NORMAL  1
#define TRB_TR_SETUP   2
#define TRB_TR_DATA    3
#define TRB_TR_STATUS  4
#define TRB_TR_LINK    6

#define TRB_CR_ENABLE_SLOT  9
#define TRB_CR_ADDR_DEV     11
#define TRB_CR_EVAL_CTX     13
#define TRB_CR_CONFIG_EP    12

#define TRB_EV_TRANSFER     32
#define TRB_EV_CMD_COMPL    33
#define TRB_EV_PORT_CHANGE  34

#define DATA_DIR_IN    (1u << 16)  /* DIR у DATA/STATUS stage */
#define TRB_BSR        (1u << 9)   /* Address Device: Block SetAddress */

#define CC_SUCCESS        1
#define CC_SHORT_PACKET   13

/* ── Slot / EP context fields (сверены с Linux xhci.h) ────────────────────── */
#define SLOT_SPEED(s)      ((uint32_t)(s) << 20)
#define SLOT_LAST_CTX(n)   ((uint32_t)(n) << 27)
/* — ГЛАВНЫЙ ФИКС (фото: «ADDRESS DEVICE cc=Parameter x2 (CSZ
 * 32+64)», дамп slot1=0x00000300): Root Hub Port Number — биты 23:16
 * dword 1 Slot Context.  Linux v6.6 xhci.h (строка 656):
 * ROOT_HUB_PORT(p) = ((p) & 0xff) << 16; Max Exit Latency — биты [15:0].
 * «поправил» верный p<<16 на p<<8 (по неверной цитате «Linux:
 * p << 8»): номер порта 3 попадал в MEL (0x300 = 768), RHPort
 * оставался 0 — строгое железо (AMD FCH 1022:7814) отвечало на Address
 * Device Parameter Error. ЭТО И БЫЛ КОРЕНЬ ВСЕХ ОТКАЗОВ -. */
#define SLOT_ROOT_PORT(p)  ((uint32_t)(p) << 16)

#define EP_INTERVAL(n)     ((uint32_t)(n) << 16)
/* — ГЛАВНЫЙ ФИКС №2 (фото: дамп ep0_1=0x18080020 → контроллер
 * видел MaxPacket = 0x1808 = 6152 вместо 8): Error Count — биты 2:1
 * dword 1 (ep_info2) EP-контекста.  Linux v6.6 xhci.h (строка 759):
 * ERROR_COUNT(p) = ((p) & 0x3) << 1; там же EP Type [5:3], Max Burst
 * [15:8], Max Packet [31:16]. «поправил» верные биты 2:1 на 31:27
 * (по неверной цитате «ERROR_COUNT(p)<<27») — 3<<27 замусорил старшие
 * биты MaxPacket.  Бит 0 (Force Event) = 0. */
#define EP_CERR3           (3u << 1)
#define EP_TYPE_CTRL       (4u << 3)
#define EP_TYPE_INT_IN     (7u << 3)
#define EP_MAX_PACKET(m)   ((uint32_t)(m) << 16)

/* Input Control Context: ADD-флаги (EP index: slot=0, EP0=1, EP1 IN=3) */
#define ICC_SLOT           (1u << 0)
#define ICC_EP0            (1u << 1)
#define ICC_EP1_IN         (1u << 3)
#define ICC_EP(n)          (1u << (2u * (uint32_t)(n) + 1u))   /* [#424] EPn IN: контекст-индекс 2n+1 */

/* имена completion-кодов и скоростей — чтобы текстлог на фото
 * читался без таблицы под рукой. */
static const char *cc_name(uint32_t cc) {
    switch (cc) {
    case CC_SUCCESS:      return "Success";
    case 2:               return "DataBuffer";
    case 3:               return "Babble";
    case 4:               return "UsbTransaction";
    case 5:               return "Trb";
    case 6:               return "Stall";
    case 7:               return "Resource";
    case 8:               return "Bandwidth";
    case 9:               return "NoSlots";
    case 11:              return "SlotNotEnabled";
    case 12:              return "EpNotEnabled";
    case CC_SHORT_PACKET: return "ShortPacket";
    case 17:              return "Parameter";
    case 19:              return "ContextState";
    case 21:              return "EventRingFull";
    case 22:              return "IncompatibleDevice";
    case 23:              return "MissedService";
    default:              return "Unknown";
    }
}

static const char *spd_name(uint32_t sv) {
    switch (sv) {
    case PORT_SPEED_FULL:  return "Full";
    case PORT_SPEED_LOW:   return "Low";
    case PORT_SPEED_HIGH:  return "High";
    case PORT_SPEED_SUPER: return "Super";
    default:               return "?";
    }
}

/* ── USB requests ──────────────────────────────────────────────────────────── */
#define USB_REQ_GET_DESCRIPTOR 0x06u
#define USB_REQ_SET_CONFIG     0x09u
#define USB_HID_SET_REPORT     0x09u   /* LED-проба «ты клава?» */
#define USB_REQ_SET_ADDRESS    0x05u   /* уникальный адрес каждому */
#define USB_HID_SET_IDLE       0x0Au
#define USB_HID_SET_PROTOCOL   0x0Bu
#define USB_DT_DEVICE   0x01u
#define USB_DT_CONFIG   0x02u
#define USB_DT_REPORT   0x22u
#define DESC_INTERFACE  0x04u
#define DESC_ENDPOINT   0x05u

/* ── Static memory (BSS, физически ниже 4 ГБ — DMA совместимо) ────────────── */
#define XHCI_NINST 4   /* [#455] Г8: AMD/Intel платформы дают до 4 xHCI */
#define OHCI_NINST 4   /* [#415] AMD FCH: до 4-5 OHCI-компаньонов — порты гнёзд 6-7 невидимы */
static int g_xi;                    /* индекс ТЕКУЩЕГО xHCI-контроллера */
static int g_ms_xi = -1;            /* инстанс-владелец мыши (-1 = мыши нет) */
static uint8_t g_ms_xep = 1;         /* [#424] номер interrupt-IN EP мыши (композиты: EP2!) */
static int g_kb_xi = -1;            /* инстанс-владелец клавиатуры */

/* 0.3.0: у AMD FCH ДВА xHCI (0:10.0/0:10.1 — USB3/USB2-линии одних
 * гнёзд).  Раньше вся DMA-память была ОДНОЙ: зонд второго контроллера
 * затирал буферы первого («мышь+клава оба в 3.0 не работают», если
 * достались разным контроллерам).  Теперь у каждого инстанса СВОИ
 * структуры; движок не переписан — он работает через АЛИАСЫ по g_xi. */
static uint64_t hw_dcbaa_a[XHCI_NINST][512]  __attribute__((aligned(4096)));
static mhw_trb_t hw_cmd_a[XHCI_NINST][16]    __attribute__((aligned(64)));
static mhw_trb_t hw_evt_a[XHCI_NINST][16]    __attribute__((aligned(64)));
static mhw_trb_t hw_ep0_a[XHCI_NINST][8]     __attribute__((aligned(64)));
static mhw_trb_t hw_ep1_a[XHCI_NINST][8]     __attribute__((aligned(64)));
static uint64_t  hw_erst_a[XHCI_NINST][8]    __attribute__((aligned(64)));
/* F3: каждый scratchpad-буфер обязан быть выровнен на 4К (спека xHCI
 * §6.1 «Scratchpad Buffer»), а DCBAA — на 64 байта минимум; Linux для
 * обеих берёт dma_alloc_coherent (страничное выравнивание). Раньше
 * стоял aligned(64): база 64-байтная -> ВЕСЬ массив scratchpad-буферов
 * был некратен 4096, и ни один буфер не попадал на границу страницы —
 * строгий реальный кремний мог уходить в ошибку по TRB/scratchpad.
 * DCBAA расширен до 512 записей (строка = ровно 4096 байт): ОБА
 * инстанса xHCI получают по 4К-выровненному массиву. Адреса — 32-битные
 * (vphys), т.е. строго < 4 ГБ, старшие dword DCBAA-записей = 0. */
static uint8_t  hw_scratch_a[XHCI_NINST][32][4096] __attribute__((aligned(4096))); /* [#456] >16 скратчпадов не выкидывает контроллер из карты */
static uint64_t  hw_sparr_a[XHCI_NINST][32]  __attribute__((aligned(64)));

#define hw_dcbaa   (hw_dcbaa_a[g_xi])
#define hw_cmd     (hw_cmd_a[g_xi])
#define hw_evt     (hw_evt_a[g_xi])
#define hw_ep0     (hw_ep0_a[g_xi])
#define hw_ep1     (hw_ep1_a[g_xi])
#define hw_erst    (hw_erst_a[g_xi])
#define hw_scratch (hw_scratch_a[g_xi])
#define hw_sparr   (hw_sparr_a[g_xi])

/* общий скратч (в момент времени идёт ОДНА перечисляемая передача) */
static uint8_t  hw_devctx[512] __attribute__((aligned(64)));
static uint8_t  hw_inctx[384]  __attribute__((aligned(64)));
static uint8_t hw_setup[8] __attribute__((aligned(4)));
static uint8_t hw_pkt[64]  __attribute__((aligned(64)));   /* [#425] FS/HS мыши дают wMaxPacketSize 32/64: буфер 16 байт означал babble/переполнение */
/* «ЭТАП 3»: 192-байтный дескриптор отчёта читается ОДНИМ TD — буфер
 * не должен пересекать границу 4К-страницы (правило OHCI TD).
 * Выравнивание 256 при длине ≤192 гарантирует это. */
static uint8_t hw_rpt[192] __attribute__((aligned(256)));

/* подпись текущего инстанса для журнала диспетчера usbdev
 * [#415] с НОМЕРОМ инстанса (их теперь до 4): раньше «xHCI#2» печаталось
 * и для 3-го, и для 4-го. */
static const char *xhci_bus_label(void) {
    static char lb[12];
    int n = 0;
    const char *p = "xHCI#";
    while (*p && n < 6) lb[n++] = *p++;
    char nb[8];
    itoa((uint32_t)(g_xi + 1), nb, 10);
    for (int q = 0; nb[q] && n < 11; q++) lb[n++] = nb[q];
    lb[n] = 0;
    return lb;
}

/* [#448] версия порта для реестра/карты: Supported Protocol ЭТОГО порта
 * (USB3 → 3, USB2 → 2), при неизвестном протоколе — по скорости устройства
 * (SuperSpeed → 3, low/full/high → 2).  Раньше ЖЁСТКО «3» — и клавиатура
 * Low-speed на USB2-порту xHCI записывалась «драйвером клавиатуры 3.0»
 * (жалоба пользователя: «для него все порты 4 и 5 — версия 3.0»). */
static int xh_port_ver(uint32_t port, uint32_t speed);

/* раскладка осей из report descriptor: настоящие смещения/размеры
 * X и Y (8/12/16 бит, report ID).  У дешёвых 2.4G-приёмников X/Y 12-битные
 * и упакованы по битам - старый разбор «байт1=X байт2=Y» дёргал курсор
 * по вертикали (X работал, Y улетал вверх). */
static bool     g_rpt_ok  = false;   /* раскладка распарсена             */
static uint8_t  g_rpt_rid = 0;       /* report ID (0 = нет)              */
static uint16_t g_x_off, g_y_off;    /* битовые смещения осей            */
static uint8_t  g_x_bits, g_y_bits;  /* размер оси в битах               */

/* ── Driver state ──────────────────────────────────────────────────────────── */
typedef struct { mhw_trb_t *t; int n; int idx; int cyc; } mhw_ring_t;

/* [#465] ЭТАП 9: размер — XHCI_NINST (4).  ВСЮ ЖИЗНЬ массивы были [2]:
 * при g_xi=2/3 (3-й/4-й xHCI — платы с PCIe USB3-картой) алиасы
 * r_cmd/r_ep0/r_ep1/r_kbd_ep1 читали/писали ЗА ГРАНИЦАМИ — порча
 * соседних статиков ядра.  Хуже: цикл гигиены в mouse_hw_init()
 * писал r_kbd_ep1[2]/[3] мимо КАЖДУЮ загрузку (даже на 1-2
 * контроллерах) — вероятный источник «мусора 0x32 в OHCI-статиках»,
 * который ловил старый СНИМОК. */
static mhw_ring_t g_r_cmd[XHCI_NINST], g_r_ep0[XHCI_NINST],
                  g_r_ep1[XHCI_NINST], g_r_kbd_ep1[XHCI_NINST],
                  g_r_msd_o[XHCI_NINST], g_r_msd_i[XHCI_NINST];
#define r_cmd     (g_r_cmd[g_xi])
#define r_ep0     (g_r_ep0[g_xi])
#define r_ep1     (g_r_ep1[g_xi])
#define r_kbd_ep1 (g_r_kbd_ep1[g_xi])
#define r_msd_o   (g_r_msd_o[g_xi])
#define r_msd_i   (g_r_msd_i[g_xi])

/* ═══ [LOGS] ЭТАП 11: USB Mass Storage (BOT + SCSI) на xHCI ════════════════
 * Флешка на xHCI — как ОНСИ-вариант (ниже), но через кольца TRB:
 * bulk-пара EP (OUT+IN) в ОДНОМ Configure Endpoint, фазы BOT —
 * обычные NORMAL TRB с IOC и ожиданием Transfer Event по (слот,
 * DC-цель 2*EP+dir).  События мыши/клавы во время ожиданий съедаются —
 * после КАЖДОЙ BOT-команды звоним пере-arm (паттерн #456).  Проверено
 * в QEMU (qemu-xhci + usb-storage); на реальном железе НЕ проверено —
 * честная пометка в README (ЭТАП 11, «непроверенные сценарии»). */
static mhw_trb_t hw_msd_out_a[XHCI_NINST][8]   __attribute__((aligned(64)));
static mhw_trb_t hw_msd_in_a[XHCI_NINST][8]    __attribute__((aligned(64)));
/* вход/выход-контексты MSD-слота (свои — как у клавиатуры: Output-контекст
 * после Address Device копируется ЦЕЛИКОМ, #423); размер покрывает EP≤7
 * при CSZ=64: индекс 15 → (15+1)*64+8 = 1032 — берём 1088 */
static uint8_t  hw_msd_devctx_a[XHCI_NINST][1088] __attribute__((aligned(64)));
static uint8_t  hw_msd_inctx_a[XHCI_NINST][1088]  __attribute__((aligned(64)));
#define hw_msd_devctx (hw_msd_devctx_a[g_xi])
#define hw_msd_inctx  (hw_msd_inctx_a[g_xi])
/* CBW/CSW/данные: CBW и CSW обязаны быть в НЕПРЕРЫВНОЙ физической памяти
 * (TRB param), данные — не пересекать 64К-границу: выравнивание 4096 */
static uint8_t  hw_msd_cbw[32]  __attribute__((aligned(4096)));
static uint8_t  hw_msd_csw[16]  __attribute__((aligned(4096)));
static uint8_t  hw_msd_dat[512] __attribute__((aligned(4096)));
static int      g_xm_slot;          /* слот MSD (0 = нет)                     */
static int      g_xm_xi = -1;       /* инстанс-владелец                      */
static uint8_t  g_xm_port;          /* её порт                               */
static uint8_t  g_xm_ep_out;        /* bulk-OUT EP (номер)                   */
static uint8_t  g_xm_ep_in;         /* bulk-IN EP (номер)                    */
static uint32_t g_xm_mps;           /* MaxPacketSize bulk-пары               */
static uint32_t g_xm_sect;          /* объём в LBA512                        */
static char     g_xm_name[28];
static uint32_t g_xm_tag;

/* ── мини-«база драйверов» — известные VID:PID + подбор по классу ────── */
typedef struct { uint16_t vid, pid; const char *name; } usb_known_t;
static const usb_known_t g_usbdb[] = {
    { 0x046D, 0xC077, "Logitech B100 (HID boot mouse)" },
    { 0x046D, 0xC016, "Logitech Optical Wheel Mouse" },
    { 0x046D, 0xC018, "Logitech Optical Mouse (PS/2-USB)" },
    { 0x046D, 0xC05A, "Logitech M90/M100" },
    { 0x046D, 0xC05C, "Logitech M120" },
    { 0x093A, 0x2510, "PixArt optical mouse (Genius/Defender/OKLICK)" },
    { 0x093A, 0x2521, "PixArt wired optical mouse" },
    { 0x1BCF, 0x0007, "Notebook optical mouse (SNOWY)" },
    { 0x045E, 0x0040, "Microsoft Wheel Mouse Optical" },
    { 0x045E, 0x0083, "Microsoft Basic Optical Mouse" },
    { 0x1241, 0x1111, "A4Tech/Belkin optical mouse" },
    { 0x1A81, 0x0004, "Defender optical mouse" },
    { 0x04FC, 0x0175, "Sunplus/Chicony optical mouse" },
};
static const char *usb_db_lookup(uint16_t vid, uint16_t pid) {
    for (unsigned i = 0; i < sizeof(g_usbdb) / sizeof(g_usbdb[0]); i++)
        if (g_usbdb[i].vid == vid && g_usbdb[i].pid == pid)
            return g_usbdb[i].name;
    return 0;
}
static void usb_db_report(uint16_t vid, uint16_t pid) {
    const char *n = usb_db_lookup(vid, pid);
    serial_puts("MOUSEHW: \xd0\xb4\xd1\x80\xd0\xb0\xd0\xb9\xd0\xb2\xd0\xb5\xd1\x80 \xd0\xbf\xd0\xbe\xd0\xb4\xd0\xbe\xd0\xb1\xd1\x80\xd0\xb0\xd0\xbd: ");
    if (n) { serial_puts(n); serial_puts("\n"); return; }
    serial_puts("\xd0\xbd\xd0\xb5\xd1\x82 \xd0\xb2 \xd0\xb1\xd0\xb0\xd0\xb7\xd0\xb5 (vid=");
    serial_puthex(vid); serial_puts(" pid="); serial_puthex(pid);
    serial_puts(") — \xd1\x83\xd0\xbd\xd0\xb8\xd0\xb2\xd0\xb5\xd1\x80\xd1\x81\xd0\xb0\xd0\xbb\xd1\x8c\xd0\xbd\xd1\x8b\xd0\xb9 HID-\xd0\xb4\xd1\x80\xd0\xb0\xd0\xb9\xd0\xb2\xd0\xb5\xd1\x80\n");
}

/* ══ ТАБЛИЦА УСТРОЙСТВ USB — модель Windows/Linux (usb_device_t) ═══════
 * ОС ведёт реестр всего, что увидела на шине: ключ = контроллер+порт, в
 * записи «паспорт» устройства (idVendor/idProduct из device descriptor),
 * роль по bInterfaceProtocol интерфейса HID (1 = клавиатура, 2 = мышь),
 * слот/адрес и флаг активности.  Подключение = register, отключение =
 * unregister, каждое событие попадает в serial «USB-TAB:» и в кольцо
 * журнала (8 последних - печатается СНИМКами).  Обработчики (мышь/клава)
 * НЕ привязаны к физическому порту: порт теперь только место события,
 * личность устройства определяется по дескрипторам - ровно как usbcore
 * в Linux и usbhub/PnP в Windows. */
#define UD_EMPTY  0
#define UD_MOUSE  1
#define UD_KBD    2
#define UD_MARKED 3
typedef struct {
    int      used;      /* запись занята                            */
    int      bus;       /* 0 = xHCI, 1 = OHCI                       */
    int      port;      /* номер порта на контроллере               */
    uint16_t vid, pid;  /* «паспорт»: idVendor / idProduct          */
    int      role;      /* UD_*                                     */
    int      proto;     /* bInterfaceProtocol (1/2), -1 = нет HID   */
    int      slot;      /* xHCI slot или OHCI адрес                 */
    int      active;    /* 1 = привязано и отвечает                 */
} usb_dev_t;
static usb_dev_t g_usbtab[8];
static uint32_t  g_usbtab_ev = 0;     /* счётчик событий (для кольца)     */
static char      g_evlog[8][96];      /* журнал последних событий         */

static const char *usbdev_rolename(int role) {
    switch (role) {
    case UD_MOUSE:  return "МЫШЬ";
    case UD_KBD:    return "КЛАВА";
    case UD_MARKED: return "помечено";
    default:        return "пусто";
    }
}
static void usbdev_cat(char *d, int *p, int cap, const char *s) {
    while (*s && *p < cap - 1) d[(*p)++] = *s++;
}
static void usbdev_catnum(char *d, int *p, int cap, uint32_t v) {
    char tmp[12];
    int  k = 0;
    if (v == 0) tmp[k++] = '0';
    while (v && k < 11) { tmp[k++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (k && *p < cap - 1) d[(*p)++] = tmp[--k];
}
static void usbdev_cathex16(char *d, int *p, int cap, uint16_t v) {
    static const char hx[] = "0123456789ABCDEF";
    if (*p + 5 >= cap) return;
    d[(*p)++] = hx[(v >> 12) & 0xFu];
    d[(*p)++] = hx[(v >> 8) & 0xFu];
    d[(*p)++] = hx[(v >> 4) & 0xFu];
    d[(*p)++] = hx[v & 0xFu];
}
/* 4-значный hex как в паспорте USB (VID_046D&PID_C077 у Windows). */
static void usbdev_puthex16(uint16_t v) {
    static const char hx[] = "0123456789ABCDEF";
    char b[7];
    b[0] = '0'; b[1] = 'x';
    b[2] = hx[(v >> 12) & 0xFu];
    b[3] = hx[(v >> 8) & 0xFu];
    b[4] = hx[(v >> 4) & 0xFu];
    b[5] = hx[v & 0xFu];
    b[6] = 0;
    serial_puts(b);
}
/* Событие: строка в serial + копия в кольцо журнала (печатает СНИМОК). */
static void usbdev_event2(const char *what, const char *suffix, int bus,
                          int port, uint16_t vid, uint16_t pid, int role) {
    char *d = g_evlog[g_usbtab_ev & 7u];
    int  p  = 0;
    g_usbtab_ev++;
    usbdev_cat(d, &p, 96, what);
    usbdev_cat(d, &p, 96, " ");
    usbdev_cat(d, &p, 96, bus ? "OHCI" : "xHCI");
    usbdev_cat(d, &p, 96, " порт ");
    usbdev_catnum(d, &p, 96, (uint32_t)port);
    usbdev_cat(d, &p, 96, " vid=0x");
    usbdev_cathex16(d, &p, 96, vid);
    usbdev_cat(d, &p, 96, " pid=0x");
    usbdev_cathex16(d, &p, 96, pid);
    usbdev_cat(d, &p, 96, " ");
    usbdev_cat(d, &p, 96, usbdev_rolename(role));
    if (suffix && *suffix) {
        usbdev_cat(d, &p, 96, " (");
        usbdev_cat(d, &p, 96, suffix);
        usbdev_cat(d, &p, 96, ")");
    }
    d[p] = 0;
    serial_puts("USB-TAB: ");
    serial_puts(d);
    serial_puts("\n");
}
static void usbdev_event(const char *what, int bus, int port,
                         uint16_t vid, uint16_t pid, int role) {
    usbdev_event2(what, 0, bus, port, vid, pid, role);
}
/* Подключение: Windows-PnP добавил устройство в реестр. */
static void usbdev_register(int bus, int port, uint16_t vid, uint16_t pid,
                            int role, int proto, int slot, int active) {
    usb_dev_t *u = 0;
    for (int i = 0; i < 8; i++)
        if (g_usbtab[i].used && g_usbtab[i].bus == bus &&
            g_usbtab[i].port == port) { u = &g_usbtab[i]; break; }
    if (!u)
        for (int i = 0; i < 8; i++)
            if (!g_usbtab[i].used) { u = &g_usbtab[i]; break; }
    if (!u) {
        usbdev_event("ТАБЛИЦА ПОЛНА - запись пропущена", bus, port, vid, pid, role);
        return;
    }
    u->used = 1;  u->bus = bus;   u->port = port;
    u->vid = vid; u->pid = pid;   u->role = role;
    u->proto = proto; u->slot = slot; u->active = active;
    usbdev_event("подключено", bus, port, vid, pid, role);
}
/* Отключение: Windows - surprise removal, устройство ушло из порта. */
static void usbdev_unregister(int bus, int port, const char *why) {
    for (int i = 0; i < 8; i++)
        if (g_usbtab[i].used && g_usbtab[i].bus == bus &&
            g_usbtab[i].port == port) {
            usbdev_event2("отключено", why, bus, port,
                          g_usbtab[i].vid, g_usbtab[i].pid, g_usbtab[i].role);
            g_usbtab[i].used = 0;
            return;
        }
}
/* Печать таблицы + журнала (итог скана и СНИМКи - фото для диагностики). */
static void usbdev_dump(const char *when) {
    int  any = 0;
    char nb[12];
    serial_puts("USB-TAB: ТАБЛИЦА УСТРОЙСТВ USB (");
    serial_puts(when);
    serial_puts("):\n");
    for (int i = 0; i < 8; i++)
        if (g_usbtab[i].used) {
            usb_dev_t *u = &g_usbtab[i];
            /* изувеченная «смазом» запись гасится с честной строкой */
            if (u->bus < 0 || u->bus > 1 || u->port < 0 || u->port > 254 ||
                u->role < 0 || u->role > 3) {
                serial_puts("USB-TAB: [");
                serial_puthex((uint32_t)i);
                serial_puts("] СМАЗ - запись сброшена\n");
                g_usbtab[i].used = 0;
                continue;
            }
            serial_puts("USB-TAB: [");
            serial_puthex((uint32_t)i);
            serial_puts("] ");
            serial_puts(u->bus ? "OHCI" : "xHCI");
            serial_puts(" порт ");
            serial_puts(itoa((uint32_t)u->port, nb, 10));
            serial_puts(": ");
            serial_puts(usbdev_rolename(u->role));
            serial_puts(" vid=");
            usbdev_puthex16(u->vid);
            serial_puts(" pid=");
            usbdev_puthex16(u->pid);
            if (u->slot) {
                serial_puts(" слот ");
                serial_puts(itoa((uint32_t)u->slot, nb, 10));
            }
            serial_puts(u->active ? " активна\n" : " помечено\n");
            any = 1;
        }
    if (!any) serial_puts("USB-TAB: (устройств нет)\n");
    serial_puts("USB-TAB: журнал событий:\n");
    if (g_usbtab_ev == 0) {
        serial_puts("USB-TAB: (событий нет)\n");
    } else {
        uint32_t k = g_usbtab_ev > 8u ? g_usbtab_ev - 8u : 0u;
        for (; k < g_usbtab_ev; k++) {
            serial_puts("USB-TAB: * ");
            serial_puts(g_evlog[k & 7u]);
            serial_puts("\n");
        }
    }
}

/* ── OHCI (USB 1.x) + EHCI (USB 2.0) — поддержка НЕ только 3.0 ─────── */
static uint32_t g_oh_mmio_a[OHCI_NINST];
static uint8_t  g_oh_nports_a[OHCI_NINST];
/* [#419] книга PCI-координат OHCI: печатается в карте даже для
 * неподнявшегося контроллера (раньше он исчезал из карты бесследно) */
static uint8_t  g_oh_dead_a[OHCI_NINST];    /* 1 = найден по PCI, но не поднялся */
static uint8_t  g_oh_pci_a[OHCI_NINST][3];  /* b:d/f для карты */
static int      g_oi = 0;            /* индекс ТЕКУЩЕГО OHCI-контроллера */
/* [#415] подпись с НОМЕРОМ инстанса (их теперь до 4): раньше «OHCI#2»
 * печаталось и для 3-го, и для 4-го. */
static const char *ohci_bus_label(void) {
    static char lb[12];
    int n = 0;
    const char *p = "OHCI#";
    while (*p && n < 6) lb[n++] = *p++;
    char nb[8];
    itoa((uint32_t)(g_oi + 1), nb, 10);
    for (int q = 0; nb[q] && n < 11; q++) lb[n++] = nb[q];
    lb[n] = 0;
    return lb;
}
static int      g_oh_ms_oi = -1;     /* инстанс-владелец мыши OHCI */
/* [#461/#449] Честная метка ГНЕЗДА OHCI/UHCI-порта для реестра: сигналинг
 * линии — USB 1.x (устройство на этой шине — 1.x-устройство, как Linux
 * называет шину «usb1"). Максимум ГНЕЗДА (USB 2.0, если это компаньон
 * EHCI) печатает КАРТА ГНЕЗД [#459] с честной пометкой «соответствие
 * без ACPI неизвестно» — в реестре драйвер честно 1.x. */
static const char *ohci_sock_label(void) {
    (void)usbdev_companion_possible;
    return "1.x";
}
static int      g_oh_kb_oi = -1;     /* инстанс-владелец клавиатуры OHCI */
static int      g_oh_msd_oi = -1;    /* инстанс-владелец флешки (sticklog) */
#define g_oh_mmio   (g_oh_mmio_a[g_oi])
#define g_oh_nports (g_oh_nports_a[g_oi])
static int      g_oh_active;      /* мышь перечислена на OHCI                */
static int      g_force_usb;     /* 1 = cmdline «usbmouse» - USB берём
                                    даже без PS/2-клавиатуры (Клава -> USB) */
static uint8_t  g_oh_addr_a[OHCI_NINST];
#define g_oh_addr (g_oh_addr_a[g_oi])
static uint8_t  g_oh_next_addr_a[OHCI_NINST];
#define g_oh_next_addr (g_oh_next_addr_a[g_oi])
static uint8_t  g_oh_ms_usb_addr;  /* адрес мыши OHCI (0 = стоит на 0) */
static uint8_t  g_oh_kbd_usb_addr; /* адрес клавы OHCI (0 = стоит на 0)*/
static uint8_t  g_oh_ms_ls;        /* мышь OHCI - LowSpeed? */
static int      g_oh_ls_a[OHCI_NINST];
#define g_oh_ls (g_oh_ls_a[g_oi])
static int      g_xhci_live_a[XHCI_NINST];
#define g_xhci_live (g_xhci_live_a[g_xi])
static uint32_t g_hp_tick;        /* троттлинг hotplug-опроса                */
/* самовосстановление hotplug + честные СНИМКи */
static uint8_t  g_hp_cd_a[XHCI_NINST][16];
static int      g_hp_fails_a[XHCI_NINST];
#define g_hp_cd    (g_hp_cd_a[g_xi])
#define g_hp_fails (g_hp_fails_a[g_xi])
static int      g_oh_probed;      /* 1 = probe_ohci выполнялся (фото:
                                   * мусор 0x32 в OHCI-СНИМКе обманывал
                                   * фильтр «g_oh_mmio == 0»)                */
static uint8_t  g_xh_bus_a[XHCI_NINST], g_xh_dev_a[XHCI_NINST], g_xh_fn_a[XHCI_NINST];
static uint16_t g_xh_vid_a[XHCI_NINST], g_xh_did_a[XHCI_NINST];
static int      g_xh_have_a[XHCI_NINST];
#define g_xh_bus (g_xh_bus_a[g_xi])
#define g_xh_dev (g_xh_dev_a[g_xi])
#define g_xh_fn  (g_xh_fn_a[g_xi])
#define g_xh_vid (g_xh_vid_a[g_xi])
#define g_xh_did (g_xh_did_a[g_xi])
#define g_xh_have (g_xh_have_a[g_xi])

static uint8_t  oh_hcca_a[OHCI_NINST][256]   __attribute__((aligned(256)));
static uint32_t oh_ed_ctrl_a[OHCI_NINST][4]  __attribute__((aligned(16)));
static uint32_t oh_ed_intr_a[OHCI_NINST][4]  __attribute__((aligned(16)));
static uint32_t oh_td_a[OHCI_NINST][3][4]    __attribute__((aligned(16)));
static uint32_t oh_td_i_a[OHCI_NINST][4]     __attribute__((aligned(16)));
#define oh_hcca    (oh_hcca_a[g_oi])
#define oh_ed_ctrl (oh_ed_ctrl_a[g_oi])
#define oh_ed_intr (oh_ed_intr_a[g_oi])
#define oh_td      (oh_td_a[g_oi])
#define oh_td_i    (oh_td_i_a[g_oi])
static uint8_t  oh_setup[8]   __attribute__((aligned(16)));
static uint8_t  oh_pkt[64]    __attribute__((aligned(64)));  /* [#425] дескрипторные MPS мыши до 64 */
static uint32_t oh_td_tail_a[OHCI_NINST][4]   __attribute__((aligned(16)));
static uint32_t oh_td_tail_i_a[OHCI_NINST][4] __attribute__((aligned(16)));
#define oh_td_tail   (oh_td_tail_a[g_oi])
#define oh_td_tail_i (oh_td_tail_i_a[g_oi])
/* USB HID-КЛАВИАТУРА на OHCI (boot protocol): свой ED/TD/буфер.
 * Клавиатура и мышь - обычно РАЗНЫЕ устройства на разных портах, поэтому
 * у клавиатуры свой адрес/эндпоинт.  Отчёт [mods][rsv][k1..k6] (8 байт)
 * переводится в сканкоды набора 1 и вливается в очередь ps2.c. */
static uint32_t oh_ed_kbd_a[OHCI_NINST][4]  __attribute__((aligned(16)));
static uint32_t oh_td_k_a[OHCI_NINST][4]    __attribute__((aligned(16)));
#define oh_ed_kbd (oh_ed_kbd_a[g_oi])
#define oh_td_k   (oh_td_k_a[g_oi])
static uint8_t  oh_kpkt[16]   __attribute__((aligned(16)));    /* отчёт клавиатуры   */
/* [#418] дескрипторные буферы перечисления OHCI — СТАТИЧЕСКИЕ и
 * выровненные: стековые cfg[256]/dd[18]/cfg9[9]/dd8[8] могли пересекать
 * границу 4К-страницы, а буфер OHCI TD не имеет права её пересекать
 * (правило драйвера, строка выше).  Адрес стека детерминирован сборкой —
 * на машине пользователя это либо всегда падало («дескриптор из
 * нулей»), либо никогда. */
static uint8_t oh_dd8_buf[8]   __attribute__((aligned(16)));
static uint8_t oh_dd_buf[18]   __attribute__((aligned(32)));
static uint8_t oh_cfg9_buf[9]  __attribute__((aligned(16)));
static uint8_t oh_cfg_buf[256] __attribute__((aligned(256)));
static int      g_oh_kbd;        /* клавиатура перечислена на OHCI          */
static int      g_oh_kbd_port;   /* её порт (для детекта отключения)        */
static int      g_oh_ms_port;    /* порт мыши OHCI (детект отключения) */
static int      g_oh_kbd_ls;     /* Low-Speed клавиатура                    */
static uint8_t  g_oh_kbd_ep;     /* номер interrupt-IN эндпоинта            */
static uint8_t  g_oh_ms_ep = 1;  /* номер interrupt-IN эндпоинта мыши */
static uint8_t  g_oh_kbd_mps;    /* max packet size эндпоинта               */
static uint8_t  g_oh_kbd_iface;  /* номер bInterfaceNumber клавиатуры       */
static uint32_t g_kbd_pkts;      /* счётчик отчётов (первый - в журнал)     */
static int       er_idx_a[XHCI_NINST];
static int       er_cyc_a[XHCI_NINST] = {1, 1, 1, 1};
#define er_idx (er_idx_a[g_xi])
#define er_cyc (er_cyc_a[g_xi])
static uint8_t g_oh_ep0_mps_a[OHCI_NINST] = {8, 8, 8, 8};
#define g_oh_ep0_mps (g_oh_ep0_mps_a[g_oi])

/* ══ ЭП-ТАБЛИЦА - память портов («таблица видов эпидов») ══════════════
 * Как только устройство успешно опознано на порту, порт запоминает:
 * вид (МЫШЬ/КЛАВА), ПРОВЕРЕННУЮ скорость шины, interrupt-эндпоинт
 * (эпид), размер пакета, номер конфигурации, vid/pid.  На следующем
 * появлении того же vid/pid в этом же порту:
 *  - перечисление начинается с проверенной скорости (бит LSDA регистра
 *    порта на части контроллеров показывает неверно - устройство
 *    молчит на чужом сигналинге, «ctrl cc=5» вечно);
 *  - связывание идёт по запомненным эндпоинтам БЕЗ полного перебора
 *    дескрипторов.
 * Память переживает отключение - поэтому после первого опознания
 * устройство обнаруживается каждый раз, и у каждого порта СВОЯ
 * личность: клавиатура и мышь не сливаются в одно устройство. */
typedef struct {
    uint8_t  used;      /* порт уже видел устройство                 */
    uint8_t  role;      /* UD_MOUSE / UD_KBD                         */
    uint8_t  ls;        /* проверенная скорость: 1 = low-speed       */
    uint8_t  addr;      /* последний рабочий USB-адрес               */
    uint8_t  ep;        /* interrupt-IN эндпоинт (номер)             */
    uint8_t  iface;     /* bInterfaceNumber                          */
    uint8_t  mps;       /* максимальный пакет эндпоинта              */
    uint8_t  cfgv;      /* bConfigurationValue (0 = не запоминать)   */
    uint8_t  both;      /* составное: мышь+клава - быстрый путь не для него */
    uint16_t vid, pid;  /* паспорт устройства                        */
    uint32_t seen;      /* сколько раз опознано                      */
} oh_memo_t;
static oh_memo_t g_oh_memo_a[OHCI_NINST][16];
static uint8_t   g_oh_slow_a[OHCI_NINST][16];
#define g_oh_memo (g_oh_memo_a[g_oi])
#define g_oh_slow (g_oh_slow_a[g_oi])

/* Обеззараживание ЭП-памяти порта: запись обязана быть либо пустой, либо
 * целиком осмысленной.  Фото: что-то затирает statics ПОСЛЕ зануления BSS
 * («СМАЗ») - и без проверки водитель доверял бы мусорной скорости/виду
 * (это и есть «мышь = клавиатура / не находится»).  Возвращает 1, если
 * записью МОЖНО пользоваться; мусор молча гасится и печатается. */
static int oh_memo_sane(int port) {
    oh_memo_t *m = &g_oh_memo[port];
    if (!m->used) return 1;                      /* пусто - норм */
    if (m->role == UD_MOUSE || m->role == UD_KBD)
        return 1;                                /* осмысленная запись */
    char nb[8];
    serial_puts("MOUSEHW: ЭП-память порта ");
    serial_puts(itoa((uint32_t)port, nb, 10));
    serial_puts(" была мусором (СМАЗ) - запись сброшена\n");
    memset(m, 0, sizeof(oh_memo_t));
    return 0;
}

static void oh_memo_save(int port, int role, uint8_t ep, uint8_t iface,
                         uint8_t mps, uint8_t cfgv, uint16_t vid, uint16_t pid) {
    /* cfgv==0 - составное устройство с MSD: ему всегда нужен полный
     * перебор ради MSD-части, ЭП-память вести не стоит */
    if (port < 1 || port > 15 || cfgv == 0) return;
    oh_memo_t *m = &g_oh_memo[port];
    m->used = 1;  m->role = (uint8_t)role;
    m->ls   = (uint8_t)(g_oh_ls ? 1 : 0);
    m->addr = g_oh_addr;  m->ep = ep;  m->iface = iface;  m->mps = mps;
    m->cfgv = cfgv;  m->vid = vid;  m->pid = pid;  m->seen++;
    char nb[8];
    serial_puts("USB-TAB: ЭП-память: OHCI порт ");
    serial_puts(itoa((uint32_t)port, nb, 10));
    serial_puts(m->role == UD_MOUSE ? " запомнил МЫШЬ" : " запомнил КЛАВИАТУРУ");
    serial_puts(" vid=0x"); serial_puthex(vid);
    serial_puts(" pid=0x"); serial_puthex(pid);
    serial_puts(" эпид=0x"); serial_puthex((uint32_t)(0x80u | ep));
    serial_puts(m->ls ? " low-speed" : " full-speed");
    serial_puts("\n");
}
static void oh_memo_dump(void) {
    char nb[8];
    serial_puts("ЭП-ТАБЛИЦА (память портов OHCI):\n");
    int any = 0;
    for (int p = 1; p <= (int)g_oh_nports && p <= 15; p++) {
        oh_memo_t *m = &g_oh_memo[p];
        if (!m->used) continue;
        any = 1;
        serial_puts("ЭП-ТАБЛИЦА: порт ");
        serial_puts(itoa((uint32_t)p, nb, 10));
        serial_puts(": ");
        serial_puts(m->role == UD_MOUSE ? "МЫШЬ" : "КЛАВИАТУРА");
        serial_puts(" vid=0x"); serial_puthex(m->vid);
        serial_puts(" pid=0x"); serial_puthex(m->pid);
        serial_puts(" эпид=0x"); serial_puthex((uint32_t)(0x80u | m->ep));
        serial_puts(m->ls ? " low-speed" : " full-speed");
        serial_puts(" опознаний=");
        serial_puts(itoa(m->seen, nb, 10));
        serial_puts("\n");
    }
    if (!any) serial_puts("ЭП-ТАБЛИЦА: (пока пусто - устройства ещё не опознавались)\n");
}
static uint32_t g_xh_mmio_a[XHCI_NINST], g_xh_op_a[XHCI_NINST], g_xh_rts_a[XHCI_NINST], g_xh_db_a[XHCI_NINST];
static uint32_t g_xh_csz_a[XHCI_NINST];
#define g_mmio (g_xh_mmio_a[g_xi])
#define g_op   (g_xh_op_a[g_xi])
#define g_rts  (g_xh_rts_a[g_xi])
#define g_db   (g_xh_db_a[g_xi])
#define g_csz  (g_xh_csz_a[g_xi])
/* состояние последней попытки перечисления xHCI — для СНИМКА */
static uint8_t  g_xh_port;        /* порт последней попытки            */
static uint32_t g_xh_speed;       /* скорость (2 = Low-Speed мышь)     */
static uint32_t g_xh_cc;          /* CC последней команды Address Dev  */
static char     g_xh_diag[96];    /* последний шаг для xHCI-СНИМКА     */

static void xh_diag_set(const char *s) {
    uint32_t i = 0;
    while (s[i] && i < sizeof(g_xh_diag) - 1u) { g_xh_diag[i] = s[i]; i++; }
    g_xh_diag[i] = 0;
}
static void xh_diag_add(const char *s) {
    uint32_t i = 0, j = 0;
    while (g_xh_diag[i]) i++;
    while (s[j] && i < sizeof(g_xh_diag) - 1u) g_xh_diag[i++] = s[j++];
    g_xh_diag[i] = 0;
}
static void xh_diag_num(uint32_t v) {
    char nb[12];
    xh_diag_add(itoa(v, nb, 10));
}
static uint8_t  g_maxports_a[XHCI_NINST];
/* [#410] Протокол КАЖДОГО порта xHCI (2 = USB2, 3 = USB3, 0 = не
 * читался) — из Supported Protocol Capability в Extended Capabilities.
 * На AMD FCH у второго xHCI (0:10.1) ВСЕ порты USB2, и раньше low-speed
 * клавиатура «представлялась» с меткой «USB 3.0» (фото) — теперь метка
 * берётся из таблицы протоколов контроллера. */
#define XH_NPORT_MAX 255  /* [#460] MaxPorts — биты [31:24], ПОЛЕ 8-БИТНОЕ, до 255: прежний обрез 63 был дефектом полноты карты (контроллеры >63 портов — редкие PCIe-карты, но карта не должна молча терять гнёзда); Г9: карта не режет порты и на 32 */
static uint8_t  g_xh_pproto_a[XHCI_NINST][XH_NPORT_MAX + 1];
#define g_xh_pproto (g_xh_pproto_a[g_xi])

/* [#448] версия порта для реестра/карты (тело — здесь, где видны
 * g_xh_pproto/XH_NPORT_MAX): Supported Protocol ЭТОГО порта, при
 * неизвестном протоколе — по скорости устройства. */
static int xh_port_ver(uint32_t port, uint32_t speed) {
    if (port >= 1 && port <= XH_NPORT_MAX) {
        if (g_xh_pproto[port] == 3) return 3;
        if (g_xh_pproto[port] == 2) return 2;
    }
    return (speed >= PORT_SPEED_SUPER) ? 3 : 2;
}
#define g_maxports (g_maxports_a[g_xi])
static bool     g_scanned, g_present, g_active;
static bool     g_oh_scanned;   /* «этап 2»: отдельный флаг mouse_hw_init_ohci */

/* «этап 2»: список USB-контроллеров PCI — файловые статики: xHCI-часть
 * (mouse_hw_init) и OHCI-часть (mouse_hw_init_ohci) делят один скан. */
static uint8_t  g_usb_bus[16], g_usb_dev[16], g_usb_fn[16], g_usb_pg[16];
static int      g_usb_n;
static int      g_slot, g_port;
/* ══ ЗАЩИТА ОТ «СМАЗА» ОЗУ ═══════════════════════════════════════════
 * Фото порт мыши стал 0x32=50 -> фиктивное «устройство отключено
 * (xHCI порт 0x32)» -> мышь убита собственной ОС; таблица устройств
 * забита значением 50, журнал усечён до строк «2».  ТА ЖЕ подпись 0x32
 * была на фото/. Linux так не умирает: страничная защита ловит
 * дикие записи, а hub-модель ПЕРЕ-перечисляет по событию порта.  У нас
 * плоская память без защиты - поэтому:
 *  1) g_port/g_slot несут штамп (значение ^ магия): расхождение = СМАЗ,
 *     значение восстанавливается из тени, факт печатается в журнал;
 *  2) фиктивный detach ОТМЕНЯЕТСЯ, если порт вне 1..g_maxports;
 *  3) канарейки 0xCA5E5ACA - в СНИМКах видно, добрал ли смаз их, и по
 *     следующему фото определяются ГРАНИЦЫ смаз-диапазона;
 *  4) usbdev_dump гасит записи таблицы с бешеными полями. */
#define MHW_PORT_XOR   0x5A5A5A5Au
static int      g_port_shadow;
static uint32_t g_port_stamp;
static int      g_slot_shadow;
static uint32_t g_slot_stamp;
#define MHW_CAN_VAL 0xCA5E5ACAu
static volatile uint32_t g_can_top[16];
static volatile uint32_t g_can_bot[16];
static void mhw_port_set(int p) {
    g_port = p;  g_port_shadow = p;
    g_port_stamp = (uint32_t)p ^ MHW_PORT_XOR;
}
static void mhw_slot_set(int s) {
    g_slot = s;  g_slot_shadow = s;
    g_slot_stamp = (uint32_t)s ^ MHW_PORT_XOR;
}
static bool mhw_port_check(const char *where) {
    if ((uint32_t)g_port != (uint32_t)g_port_shadow ||
        g_port_stamp != ((uint32_t)g_port_shadow ^ MHW_PORT_XOR)) {
        serial_puts("MOUSEHW: СМАЗ: порт мыши ");
        serial_puthex((uint32_t)g_port);
        serial_puts(" - восстановлен в ");
        serial_puthex((uint32_t)g_port_shadow);
        serial_puts(" (");
        serial_puts(where);
        serial_puts(")\n");
        mhw_port_set(g_port_shadow);
        return true;
    }
    return false;
}
static bool mhw_slot_check(const char *where) {
    if ((uint32_t)g_slot != (uint32_t)g_slot_shadow ||
        g_slot_stamp != ((uint32_t)g_slot_shadow ^ MHW_PORT_XOR)) {
        serial_puts("MOUSEHW: СМАЗ: слот мыши ");
        serial_puthex((uint32_t)g_slot);
        serial_puts(" - восстановлен в ");
        serial_puthex((uint32_t)g_slot_shadow);
        serial_puts(" (");
        serial_puts(where);
        serial_puts(")\n");
        mhw_slot_set(g_slot_shadow);
        return true;
    }
    return false;
}
static void mhw_canary_init(void) {
    for (int i = 0; i < 16; i++) {
        g_can_top[i] = MHW_CAN_VAL;
        g_can_bot[i] = MHW_CAN_VAL;
    }
}
static int mhw_canary_check(const char *where) {
    static int last_bad;
    int bad = 0, first = -1;
    uint32_t fv = 0;
    for (int i = 0; i < 16; i++) {
        if (g_can_top[i] != MHW_CAN_VAL) {
            bad++;
            if (first < 0) { first = i; fv = g_can_top[i]; }
        }
        if (g_can_bot[i] != MHW_CAN_VAL) {
            bad++;
            if (first < 0) { first = i + 16; fv = g_can_bot[i]; }
        }
    }
    if (bad > 0 && bad != last_bad) {
        serial_puts("MOUSEHW: СМАЗ: канарейки битые x=");
        serial_puthex((uint32_t)bad);
        serial_puts(" первый @");
        serial_puthex((uint32_t)first);
        serial_puts(" =");
        serial_puthex(fv);
        serial_puts(" (");
        serial_puts(where);
        serial_puts(")\n");
    }
    last_bad = bad;
    return bad;
}
/* ── «диалог с устройствами» ────────────────────────────────────────────────
 * ОС опрашивает КАЖДЫЙ порт: «ты клава?» - привязать и искать мышь дальше;
 * «мышь?» - взять себе; «ввода нет?» - пометить и пропускать.  Клавиатура
 * больше НИКОГДА не занимает место мыши, мышь ищется по ВСЕМ портам,
 * подключённое позже устройство находится опросом портов. */
#define PR_EMPTY 0u   /* порт пуст / роль сброшена                          */
#define PR_MOUSE 1u   /* на порту МЫШЬ (перечислена, слот = g_slot)         */
#define PR_KBD   2u   /* КЛАВИАТУРА привязана («ты клава? - да!»)           */
#define PR_OTHER 3u   /* устройство без ввода - помечено и пропускается     */
static uint8_t  g_port_role_a[XHCI_NINST][XH_NPORT_MAX + 1];
#define g_port_role (g_port_role_a[g_xi])
static int      g_eslot_a[XHCI_NINST];      /* слот ТЕКУЩЕГО перечисления (per-instance) */
#define g_eslot (g_eslot_a[g_xi])
static int      g_kbd_slot;   /* слот клавиатуры xHCI (0 = клавиатуры нет)  */
static uint8_t  g_kbd_xport;  /* её порт (для детекта отключения)           */
static uint8_t  g_kbd_xmps;   /* mps её interrupt-IN EP (8..16)             */
static uint8_t  g_kbd_xiface; /* номер bInterfaceNumber клавиатуры          */
static uint8_t  g_kbd_xep;    /* её interrupt-IN EP (обычно 1; композиты 2) */
static uint8_t  hw_kbd_devctx_a[XHCI_NINST][512] __attribute__((aligned(64)));
static mhw_trb_t hw_kbd_ep1_a[XHCI_NINST][8]     __attribute__((aligned(64)));
static uint8_t  hw_kbd_inctx_a[XHCI_NINST][384]  __attribute__((aligned(64)));
static uint8_t  hw_mouse_devctx_a[XHCI_NINST][512] __attribute__((aligned(64)));
static uint8_t  hw_kbd_pkt[16] __attribute__((aligned(16)));
#define hw_kbd_devctx  (hw_kbd_devctx_a[g_xi])
#define hw_kbd_ep1     (hw_kbd_ep1_a[g_xi])
#define hw_kbd_inctx   (hw_kbd_inctx_a[g_xi])
#define hw_mouse_devctx (hw_mouse_devctx_a[g_xi])
static uint8_t  g_iface, g_mps;
static bool     g_boot_iface, g_absolute;
static uint8_t  g_hid_proto;    /* bInterfaceProtocol выбранного HID */
static bool     g_ctrl_quiet;   /* молчать в ctrl_xfer (STALL мыши -
                                 * нормальный ответ на LED-пробу)             */
/* непонятный HID (protocol 0) на OHCI - данные для LED-пробы */
static uint8_t  g_oh_amb_iface, g_oh_amb_ep, g_oh_amb_mps, g_oh_amb_sub;
static uint16_t g_log_max;
static uint32_t g_packets, g_errors;
static int      g_fb_w, g_fb_h;
static mouse_state_t g_mouse;
static bool     prev_left, prev_right;

/* ══ [#474] ЭТАП 10: СНИМКИ КОНФИГУРАЦИИ ЖИВЫХ СЛОТОВ ══════════════════
 *
 * ДЕФЕКТ (критический, «видны — но не работают» у второго устройства):
 * скрэтч-глобали перечисления (g_mps, g_ms_xep, g_kbd_xmps, g_rpt_ok,
 * g_absolute, g_log_max, g_oh_kbd_mps, ...) пишутся при КАЖДОМ
 * перечислении ЛЮБОГО устройства — даже когда реестр usbdev ОТКАЗАЛ
 * ему в привязке (слоты заняты). ЖИВАЯ мышь/клавиатура при этом:
 *   • фильтр событий (2*EP+1) и дверной звонок читали ЧУЖОЙ g_ms_xep/
 *     g_kbd_xep — отчёты живого устройства переставали совпадать с
 *     фильтром и НЕ обрабатывались (мышь/клава «замирали»);
 *   • перевзвод TRB/TD уходил с ЧУЖИМ MPS — неверная длина;
 *   • раскладка осей (g_rpt_ok/g_x_off/...) сбрасывалась в false при
 *     каждом чужом hw_detect_mode — мышь с 12-битными осями молча
 *     падала в boot-разбор (дефект «Y улетает» возвращался);
 *   • g_absolute/g_log_max подменялись планшетом соседа — курсор
 *     прыгал в угол.
 * Решение (как «личный контекст» hw_mouse_devctx, но для СКРЭТЧА):
 * снимок конфигурации берётся ОДИН раз при привязке (g_lm — мышь,
 * g_lk — клавиатура) и ВСЕ пути опроса/перевзвода/звонков читают
 * ТОЛЬКО его. Перечисление соседа больше не влияет на живое устройство.
 * Аксессоры с фолбэком на скрэтч — на случай пути привязки, где
 * снимок не сделан (совместимость со старым поведением). */
typedef struct {
    uint8_t  valid;
    uint8_t  mps;        /* interrupt-IN MPS живой мыши                  */
    uint8_t  xep;        /* номер EP (xHCI; у OHCI в TD уже зашит)       */
    uint8_t  absolute;   /* планшет (absolute-отчёты)?                   */
    uint8_t  rpt_ok;     /* раскладка осей распарсена?                   */
    uint8_t  rpt_rid;    /* report ID (0 = нет)                          */
    uint16_t x_off, y_off;
    uint8_t  x_bits, y_bits;
    uint16_t log_max;    /* LOGICAL_MAX планшета                         */
} mhw_live_ms_t;
typedef struct {
    uint8_t  valid;
    uint8_t  xmps;       /* xHCI: MPS EP клавиатуры                      */
    uint8_t  xep;        /* xHCI: номер EP                               */
    uint8_t  oh_mps;     /* OHCI: MPS (перевзвод TD)                     */
} mhw_live_kbd_t;
static mhw_live_ms_t  g_lm;
static mhw_live_kbd_t g_lk;
/* сохранение снимка ЖИВОЙ МЫШИ из скрэтча перечисления (вызывается
 * в конце привязки, ДО первого hw_arm/постановки TD). */
static void lm_save(void) {
    g_lm.valid    = 1;
    g_lm.mps      = g_mps;
    g_lm.xep      = g_ms_xep;
    g_lm.absolute = g_absolute ? 1 : 0;
    g_lm.rpt_ok   = g_rpt_ok ? 1 : 0;
    g_lm.rpt_rid  = g_rpt_rid;
    g_lm.x_off    = g_x_off;  g_lm.y_off = g_y_off;
    g_lm.x_bits   = g_x_bits; g_lm.y_bits = g_y_bits;
    g_lm.log_max  = g_log_max;
}
static void lm_drop(void) { g_lm.valid = 0; }
/* сохранение снимка ЖИВОЙ КЛАВИАТУРЫ (xHCI-часть). */
static void lk_save_x(void) {
    g_lk.valid = 1;
    g_lk.xmps  = g_kbd_xmps;
    g_lk.xep   = g_kbd_xep;
}
/* OHCI-часть снимка клавиатуры (перевзвод TD в ohci_poll_reports_inst). */
static void lk_save_oh(void) {
    g_lk.valid  = 1;
    g_lk.oh_mps = g_oh_kbd_mps;
}
static void lk_drop(void) { g_lk.valid = 0; }
/* аксессоры с фолбэком на скрэтч (снимок не сделан = старое поведение) */
static uint8_t lm_mps(void)      { return g_lm.valid ? g_lm.mps      : g_mps; }
static uint8_t lm_xep(void)      { return g_lm.valid ? g_lm.xep      : g_ms_xep; }
static int     lm_absolute(void) { return g_lm.valid ? g_lm.absolute : (int)g_absolute; }
static int     lm_rpt_ok(void)   { return g_lm.valid ? g_lm.rpt_ok   : (int)g_rpt_ok; }
static uint8_t lk_xmps(void)     { return g_lk.valid ? g_lk.xmps     : g_kbd_xmps; }
static uint8_t lk_xep(void)      { return g_lk.valid ? g_lk.xep      : g_kbd_xep; }
static uint8_t lk_oh_mps(void)   { return g_lk.valid ? g_lk.oh_mps   : g_oh_kbd_mps; }
/* [#475/#476] определены ниже (перед oh_kbd_process), а нужны и путям
 * привязки/отключения клавиатуры выше по файлу. */
static void zl_note(const char *what);
static void oh_kbd_state_reset(void);
/* [LOGS] ЭТАП 11: xHCI MSD — определены ниже (после OHCI-варианта),
 * а вызываются из hw_enumerate выше. */
static bool xm_dev_probe(const uint8_t *cfg, uint16_t total, uint32_t speed);
static bool xm_setup(uint8_t port, uint32_t speed, uint8_t cfg_val);

static void hw_delay(uint32_t us) {
    /* F3: тело заменено на калиброванную задержку из kernel/timer.c
     * (PIT ch2, как calibrate_delay() в Linux). Старый цикл inb(0x80)
     * длился по-разному на QEMU (~20 нс/ит) и реальном ПК (~1 мкс/ит):
     * USB-тайминги (TRSTRCY, дебаунс 100 мс, recovery 50 мс) уплывали
     * в разы — «QEMU зелёный, железо мёртвое». До вызова timer_init()
     * kdelay_us честно работает консервативным фолбэком (1 итер/мкс). */
    kdelay_us(us);
}

/* ── Кольца: постановка TRB с авто-LINK на границе сегмента ───────────────── */
static uint32_t ring_put(mhw_ring_t *r, uint64_t param, uint32_t status,
                         uint32_t control) {
    if (r->idx == r->n - 1) {                 /* последний слот — LINK TRB */
        mhw_trb_t *l = &r->t[r->idx];
        l->param   = (uint64_t)vphys(r->t);
        l->status  = 0;
        l->control = TRB_TYPE(TRB_TR_LINK) | TRB_TC | (uint32_t)r->cyc;
        r->cyc ^= 1;
        r->idx  = 0;
    }
    mhw_trb_t *t = &r->t[r->idx];
    t->param   = param;
    t->status  = status;
    /* БАГ-ИСТОРИЯ (Task 28): было «control | TRB_CYCLE | r->cyc» — при
     * r->cyc==0 бит cycle всё равно форсировался в 1 (TRB_CYCLE==1).
     * Первый ctrl-трансфер, пересекший границу 8-TRB кольца (DATA/STATUS
     * через LINK на TRB0/1), нёс cycle=1 при ожидаемых контроллером 0 →
     * кольцо молча стояло → «ctrl timeout».  Теперь cycle-бит = r->cyc. */
    t->control = (control & ~TRB_CYCLE) | (uint32_t)r->cyc;
    r->idx++;
    return vphys(t);
}

/* ── Event ring: взять событие (если есть) и сразу продвинуть ERDP ──────────
 * ERDP обязан оставаться ВНУТРИ кольца (QEMU проверяет границы):
 * указатель = адрес следующего необработанного TRB, при завороте — старт. */
static bool evt_get(uint64_t *param, uint32_t *status, uint32_t *control) {
    dma_sync();   /* кольцо событий пишет КОНТРОЛЛЕР - читаем свежее */
    mhw_trb_t *t = &hw_evt[er_idx];
    if ((t->control & TRB_CYCLE) != (uint32_t)er_cyc) return false;
    *param   = t->param;
    *status  = t->status;
    *control = t->control;
    er_idx++;
    if (er_idx == 16) { er_idx = 0; er_cyc ^= 1; }
    uint32_t addr = vphys(hw_evt) + 16u * (uint32_t)er_idx;
    mmw64(g_rts + XHC_IR0 + XHC_IR_ERDP, (uint64_t)addr | ERDP_EHB);
    return true;
}

/* ── Дверные звонки: db[0] = команды (target 0), db[slot] = EP-цель ───────── */
static void db_command(void)    { dma_sync(); mmw32(g_db, 0); }
/* ctrl-передачи звонят в слот ПЕРЕЧИСЛЕНИЯ (g_eslot): клавиатура
 * перечисляется в СВОЁМ слоте, мышь - в своём (раньше жёстко g_slot). */
static void db_ep0(void)        { dma_sync(); mmw32(g_db + 4u * (uint32_t)g_eslot, 1); }
static void db_ep1_in(void)     { if (g_ms_xi < 0) return; dma_sync(); mmw32(g_xh_db_a[g_ms_xi] + 4u * (uint32_t)g_slot, 2u * (uint32_t)lm_xep() + 1u); }   /* [#424] цель = 2*EP+1; [#474] EP — из СНИМКА живой мыши */
/* звонок в interrupt-IN клавиатуры (свой слот, EP1 IN = target 3) */
static void db_kbd_in(void)     { if (g_kb_xi < 0) return; dma_sync(); mmw32(g_xh_db_a[g_kb_xi] + 4u * (uint32_t)g_kbd_slot, 2u * (uint32_t)lk_xep() + 1u); }   /* [#474] EP — из СНИМКА живой клавы */
/* перед КАЖДЫМ звонком wbinvd - контроллер читает контексты/TRB
 * из ОЗУ, а не из нашего кэша.  В QEMU wbinvd безвреден (кэша нет). */

/* ── Ждать завершения КОМАНДЫ (Command Completion Event) ────────────────────
 * События-не-матчи (port status change и пр.) потребляются молча.
 * ccode=0 — таймаут. */
static bool wait_cmd(uint32_t trb_phys, uint32_t *ccode, uint32_t *slot_out) {
    int timeout = 3000;   /* ~3 c */
    for (;;) {
        uint64_t p; uint32_t st, ctl;
        while (evt_get(&p, &st, &ctl)) {
            uint32_t type = (ctl >> 10) & 0x3Fu;
            if (type == TRB_EV_CMD_COMPL && (uint32_t)p == trb_phys) {
                *ccode    = st >> 24;
                *slot_out = ctl >> 24;
                return (*ccode == CC_SUCCESS);
            }
            /* прочие события — пропустить */
        }
        if (timeout-- <= 0) { *ccode = 0; *slot_out = 0; return false; }
        hw_delay(1000);
    }
}

/* ── USB control transfer через EP0 (только для перечисления) ────────────────
 * SETUP(+DATA)+STATUS ставятся в кольцо одной пачкой, IOC — только у
 * STATUS stage: ровно одно Transfer Event на трансфер (CC=SUCCESS или
 * SHORT_PACKET — оба успеха).  STATUS-стейдж направлен противоположно
 * data/setup (DIR=1 для host-to-device запросов). */
static bool ctrl_xfer(uint8_t bm, uint8_t bReq, uint16_t wVal, uint16_t wIdx,
                      uint16_t wLen, void *data) {
    hw_setup[0] = bm;
    hw_setup[1] = bReq;
    hw_setup[2] = (uint8_t)wVal;  hw_setup[3] = (uint8_t)(wVal >> 8);
    hw_setup[4] = (uint8_t)wIdx;  hw_setup[5] = (uint8_t)(wIdx >> 8);
    hw_setup[6] = (uint8_t)wLen;  hw_setup[7] = (uint8_t)(wLen >> 8);

    uint64_t setup64;
    memcpy(&setup64, hw_setup, 8);
    ring_put(&r_ep0, setup64, 8, TRB_TYPE(TRB_TR_SETUP) | TRB_IDT);

    bool in = (bm & 0x80u) != 0;
    if (wLen > 0 && data) {
        if (in) memset(data, 0, wLen);
        ring_put(&r_ep0, vphys(data), wLen,
                 TRB_TYPE(TRB_TR_DATA) | (in ? DATA_DIR_IN : 0));
    }
    ring_put(&r_ep0, 0, 0,
             TRB_TYPE(TRB_TR_STATUS) | (in ? 0 : DATA_DIR_IN) | TRB_IOC);
    db_ep0();

    int timeout = 3000;   /* ~3 c */
    for (;;) {
        uint64_t p; uint32_t st, ctl;
        while (evt_get(&p, &st, &ctl)) {
            uint32_t type = (ctl >> 10) & 0x3Fu;
            if (type == TRB_EV_TRANSFER && (ctl >> 24) == (uint32_t)g_eslot) {
                uint32_t cc = st >> 24;
                if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) {
                    if (!g_ctrl_quiet) {   /* STALL от LED-пробы - не ошибка */
                        serial_puts("MOUSEHW: ctrl error cc=");
                        serial_puthex(cc);
                        serial_puts(" ("); serial_puts(cc_name(cc)); serial_puts(")\n");
                    }
                    return false;
                }
                return true;
            }
        }
        if (timeout-- <= 0) {
            serial_puts("MOUSEHW: ctrl timeout\n");
            return false;
        }
        hw_delay(1000);
    }
}

/* ── Context builders ─────────────────────────────────────────────────────── */
static void inctx_icc(uint32_t add) {
    uint32_t *icc = (uint32_t *)hw_inctx;
    icc[0] = 0;      /* drop flags */
    icc[1] = add;    /* add flags  */
}

static void inctx_slot(uint32_t speed, uint8_t port, uint32_t last) {
    uint32_t *d = (uint32_t *)(hw_inctx + g_csz);
    d[0] = SLOT_LAST_CTX(last) | SLOT_SPEED(speed);
    d[1] = SLOT_ROOT_PORT(port);
    d[2] = 0;
    d[3] = 0;
}

static void inctx_ep0(uint32_t mps, uint32_t interval) {
    uint32_t *d = (uint32_t *)(hw_inctx + 2u * g_csz);
    d[0] = EP_INTERVAL(interval);
    d[1] = EP_CERR3 | EP_TYPE_CTRL | EP_MAX_PACKET(mps);
    d[2] = vphys(hw_ep0) | 1u;                  /* DCS=1: первый TRB cycle=1 */
    d[3] = 0;                                   /* верхняя половина deq = 0 */
    d[4] = 8u;                                   /* avg TRB len = 8 — биты
                                                    [15:0]; ESIT (биты [31:16])
                                                    у control-EP не задаётся */
    d[5] = d[6] = d[7] = 0;
}

/* [#424] interrupt-IN EP мыши с ЛЮБЫМ номером: композитные донглы дают
 * клавиатуру на EP1, мышь на EP2.  Контекст-индекс EPn IN = 2n+1,
 * смещение в input-контексте = g_csz + (2n+1)*g_csz.  Дверной звонок —
 * тоже 2n+1.  Раньше всё было захардкожено под EP1 — мышь с EP≠1
 * опрашивалась по чужому эндпоинту → STALL → 8 ошибок → деактивация.
 * (Версия inctx_ep1_in только-под-EP1 удалена как dead code: с #424
 * единственный вызов ушёл на inctx_ep_in.) */
static void inctx_ep_in(uint32_t ep, uint32_t mps) {
    uint32_t *d = (uint32_t *)(hw_inctx + (2u * ep + 2u) * g_csz);
    d[0] = EP_INTERVAL(3);
    d[1] = EP_CERR3 | EP_TYPE_INT_IN | EP_MAX_PACKET(mps);
    d[2] = vphys(hw_ep1) | 1u;                  /* DCS=1 */
    d[3] = 0;
    d[4] = (mps << 16) | 8u;
    d[5] = d[6] = d[7] = 0;
}

/* ── Отчёты: та же семантика движения, что у usb_mouse.c v1.5 ─────────────── */
static void hw_move_to(int nx, int ny) {
    if (nx < 0) nx = 0;
    if (nx >= g_fb_w) nx = g_fb_w - 1;
    if (ny < 0) ny = 0;
    if (ny >= g_fb_h) ny = g_fb_h - 1;
    if (nx != g_mouse.x || ny != g_mouse.y) g_mouse.moved = true;
    g_mouse.x = nx;
    g_mouse.y = ny;
}

static void hw_apply_buttons(uint8_t btn) {
    bool nl = (btn & 1) != 0;
    bool nr = (btn & 2) != 0;
    if (nl && !prev_left)  g_mouse.clicked_left  = true;
    if (nr && !prev_right) g_mouse.clicked_right = true;
    prev_left  = nl;
    prev_right = nr;
    g_mouse.left  = nl;
    g_mouse.right = nr;
}

static void hw_boot_report(const uint8_t *p, int act) {
    if (act < 3) return;
    int dx = (int)(int8_t)p[1] * 2;      /* x2 — та же скорость, что в v1.5 */
    int dy = (int)(int8_t)p[2] * 2;
    if (dx >  48) dx =  48;
    if (dx < -48) dx = -48;
    if (dy >  48) dy =  48;
    if (dy < -48) dy = -48;
    hw_move_to(g_mouse.x + dx, g_mouse.y + dy);   /* USB: +dy = вниз по экрану */
    hw_apply_buttons(p[0]);
}

static void hw_tablet_report(const uint8_t *p, int act) {
    if (act < 5) return;
    /* g_log_max заполняется ТОЛЬКО при успешном парсе report descriptor;
     * при fallback-детекте «absolute по subclass» (дескриптор не читается
     * - мыши нередко STALL'ят запрос) он оставался 0 от прошлого устройства
     * -> деление на 1 -> курсор в правом-нижнем углу.  Guard как в
     * usb_mouse.c v1.5.  [#474] масштаб — из СНИМКА живой мыши (чужое
     * перечисление больше не подменяет его). */
    uint32_t lim = g_lm.valid ? ((uint32_t)g_lm.log_max + 1u)
                              : (g_log_max ? ((uint32_t)g_log_max + 1u) : 32768u);
    uint32_t x = (uint32_t)p[1] | ((uint32_t)p[2] << 8);
    uint32_t y = (uint32_t)p[3] | ((uint32_t)p[4] << 8);
    int nx = (int)((x * (uint32_t)g_fb_w) / lim);
    int ny = (int)((y * (uint32_t)g_fb_h) / lim);
    hw_move_to(nx, ny);
    hw_apply_buttons(p[0]);
}

/* ── битовые поля отчёта (X/Y любого размера из descriptor) ──────────── */
static int32_t hw_bits_get(const uint8_t *p, int bitoff, int bits) {
    int32_t v = 0;
    for (int i = 0; i < bits; i++) {
        int b = bitoff + i;
        v |= (int32_t)((p[b >> 3] >> (b & 7)) & 1u) << i;
    }
    if (v & (1 << (bits - 1))) v -= (1 << bits);    /* знакорасширение */
    return v;
}

/* ── парсер report descriptor: где лежат X (0x30) и Y (0x31) ────────────────
 * Короткие items; счёт бит ТОЛЬКО по INPUT (Output/Feature - свои поля).
 * Usage-список: X и Y могут идти ОДНОЙ INPUT (count=2) - usage[i] с добивкой
 * последним usage (спека HID).  Неудача парса - fallback на legacy-разбор. */
static void hw_parse_axes(const uint8_t *r, uint16_t n) {
    g_rpt_ok = false;                           /* парс начался заново */
    uint32_t page = 0, size = 0, cnt = 1;
    uint32_t usg[8];
    int      nusg = 0;
    uint8_t  rid = 0;
    int      pos = 0, bit = 0;
    bool     has_x = false, has_y = false;
    uint32_t x_off = 0, y_off = 0, x_bits = 0, y_bits = 0;

    while (pos < (int)n) {
        uint8_t item = r[pos++];
        if (item == 0xFE) {                     /* long item - пропустить   */
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

        switch (item & 0xFCu) {
        case 0x04u: page = val; break;          /* Usage Page (глоб.)       */
        case 0x08u:                             /* Usage (лок.)             */
            if (nusg < 8) usg[nusg++] = val;
            break;
        case 0x74u: size = val; break;          /* Report Size              */
        case 0x94u: cnt = val ? val : 1u; break;/* Report Count             */
        case 0x84u: rid = (uint8_t)val; break;  /* Report ID                */
        case 0x80u:                             /* INPUT                    */
            if ((val & 0x03u) == 0x02u) {       /* Data,Var                 */
                for (uint32_t f = 0; f < cnt; f++) {
                    uint32_t u = nusg ? usg[(f < (uint32_t)nusg)
                                            ? f : (uint32_t)nusg - 1u] : 0u;
                    if (page == 1u && u == 0x30u && !has_x) {
                        x_off = (uint32_t)bit; x_bits = size; has_x = true;
                    } else if (page == 1u && u == 0x31u && !has_y) {
                        y_off = (uint32_t)bit; y_bits = size; has_y = true;
                    }
                    bit += (int)size;
                }
            } else {                            /* Const/Arr - просто место */
                bit += (int)(size * cnt);
            }
            nusg = 0;
            break;
        /* Output/Feature/Collection биты INPUT-отчёта не занимают */
        default: break;
        }
        if (bit > 160) break;                   /* защита от мусора         */
    }

    if (has_x && has_y && x_bits >= 4u && x_bits <= 16u &&
        y_bits >= 4u && y_bits <= 16u &&
        (x_off + x_bits) <= 112u && (y_off + y_bits) <= 112u) {
        g_x_off  = (uint16_t)x_off;
        g_y_off  = (uint16_t)y_off;
        g_x_bits = (uint8_t)x_bits;
        g_y_bits = (uint8_t)y_bits;
        g_rpt_rid = rid;
        g_rpt_ok  = true;
        bool legacy = (rid == 0u && x_off == 8u && x_bits == 8u &&
                       y_off == 16u && y_bits == 8u);
        if (!legacy) {                          /* не спамим про обычные    */
            serial_puts("MOUSEHW: axes X off=");
            serial_puthex(x_off);
            serial_puts(" bits=");
            serial_puthex(x_bits);
            serial_puts(" Y off=");
            serial_puthex(y_off);
            serial_puts(" bits=");
            serial_puthex(y_bits);
            serial_puts(" rid=");
            serial_puthex((uint32_t)rid);
            serial_puts("\n");
        }
    }
}

/* ── отчёт по раскладке из descriptor (8/12/16 бит, report ID) ───────── */
static void hw_hid_report(const uint8_t *p, int act) {
    /* [#474] раскладка — из СНИМКА живой мыши: чужое перечисление (вторая
     * мышь, планшет-сосед, композит) сбрасывало g_rpt_ok — оси молча
     * уезжали в boot-разбор. Снимок не зависит от скрэтча. */
    uint8_t  rid    = g_lm.valid ? g_lm.rpt_rid  : g_rpt_rid;
    uint16_t x_off  = g_lm.valid ? g_lm.x_off   : g_x_off;
    uint16_t y_off  = g_lm.valid ? g_lm.y_off   : g_y_off;
    uint8_t  x_bits = g_lm.valid ? g_lm.x_bits  : g_x_bits;
    uint8_t  y_bits = g_lm.valid ? g_lm.y_bits  : g_y_bits;
    int base = rid ? 1 : 0;                     /* байт report ID           */
    int need = base + (int)(((uint32_t)y_off + (uint32_t)y_bits + 7u) >> 3);
    if (act < need) {
        if (!rid && act >= 3) hw_boot_report(p, act);
        return;                                 /* обрыв - молчим           */
    }
    int dx = (int)hw_bits_get(p, base * 8 + (int)x_off, (int)x_bits) * 2;
    int dy = (int)hw_bits_get(p, base * 8 + (int)y_off, (int)y_bits) * 2;
    if (dx >  48) dx =  48;
    if (dx < -48) dx = -48;
    if (dy >  48) dy =  48;
    if (dy < -48) dy = -48;
    hw_move_to(g_mouse.x + dx, g_mouse.y + dy); /* +dy = вниз, как и было   */
    hw_apply_buttons(p[base]);
}

/* ── Деактивация: указатель возвращается к PS/2 через ps2_poll ────────────── */
static void hw_deactivate(const char *why) {
    if (g_ms_xi >= 0) g_xi = g_ms_xi;   /* слот живёт на инстансе мыши */
    mhw_port_check("deactivate");   /* unregister по НАСТОЯЩЕМУ порту */
    serial_puts("MOUSEHW: deactivated (");
    serial_puts(why);
    serial_puts("), PS/2 resumes\n");
    /* мышь покинула порт - unregister в ТАБЛИЦЕ УСТРОЙСТВ (модель
     * Windows: disconnect -> surprise removal -> запись погашена). */
    usbdev_unregister(0, (int)g_port, why);
    g_active = false;
    /* слот мыши неактивен, но в железе ОСТАЁТСЯ включённым -
     * отдаём его g_eslot: следующая зачистка перед перечислением отключит. */
    if (g_slot != 0) { g_eslot = g_slot; mhw_slot_set(0); }  /* штамп */
    g_ms_xi = -1;                       /* 0.3.0: инстанс свободен */
    lm_drop();                           /* [#474] снимок мыши погашен — следующая привязка создаст новый */
    usbdev_release(USBDEV_ROLE_MOUSE, why);
}

/* ── Постановка interrupt-IN TRB (один outstanding) + дверной звонок ──────── */
static void hw_arm(void) {
    if (g_ms_xi >= 0) g_xi = g_ms_xi;   /* кольцо/звонок — на инстансе мыши */
    ring_put(&r_ep1, vphys(hw_pkt), lm_mps(), TRB_TYPE(TRB_TR_NORMAL) | TRB_IOC);   /* [#474] MPS — из СНИМКА живой мыши */
    db_ep1_in();
}

/* ── HID report descriptor: абсолютный планшет? ─────────────────────────────
 * Минимальный парсер short items: INPUT (0x81) c флагами Data|Var|Absolute
 * (val&7==0x02) и Report Size 16 → абсолютные оси; LOGICAL_MAX берётся
 * из глобальных item'ов (0x26/0x27).  Свежая реализация (не копия v1.5). */
static bool hw_parse_absolute(const uint8_t *r, uint16_t n, uint16_t *max) {
    uint32_t rpt_size = 0;
    uint32_t log_max  = 0;
    uint16_t abs_max  = 0;
    bool     abs16    = false;
    int pos = 0;

    while (pos < (int)n) {
        uint8_t item = r[pos++];
        if (item == 0xFE) {                     /* long item — пропустить   */
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

        if (item == 0x75) {                     /* REPORT_SIZE              */
            rpt_size = val;
        } else if (item == 0x26 || item == 0x27) {
            if (val <= 0xFFFFu) log_max = val;  /* LOGICAL_MAX (положительный) */
        } else if (item == 0x81) {              /* INPUT                    */
            if ((val & 7u) == 0x02u && rpt_size == 16u) {
                abs16   = true;
                abs_max = (uint16_t)log_max;
            }
        }
    }
    if (!abs16) return false;
    if (max) *max = abs_max ? abs_max : 32767u;  /* sane default (QEMU)     */
    return true;
}

/* ── Режим устройства: планшет (absolute) или мышь (relative boot) ────────── */
static bool hw_detect_mode(const uint8_t *cfg, uint16_t total) {
    g_rpt_ok = false;   /* ре-enum - старая раскладка недействительна */
    int pos = 0, rpt_len = 0;
    bool in_iface = false;

    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) {
            in_iface = (cfg[pos + 2] == g_iface);
        } else if (type == 0x21u && in_iface) {  /* HID descriptor */
            rpt_len = cfg[pos + 7] | ((int)cfg[pos + 8] << 8);
            break;
        }
        pos += len;
    }

    if (rpt_len <= 0) {
        serial_puts("MOUSEHW: no report desc length, mode by subclass\n");
        return !g_boot_iface;
    }
    if (rpt_len > (int)sizeof(hw_rpt)) rpt_len = (int)sizeof(hw_rpt);

    memset(hw_rpt, 0, sizeof(hw_rpt));
    if (!ctrl_xfer(0x81, USB_REQ_GET_DESCRIPTOR, (USB_DT_REPORT << 8),
                   g_iface, (uint16_t)rpt_len, hw_rpt)) {
        serial_puts("MOUSEHW: report desc read failed, mode by subclass\n");
        return !g_boot_iface;
    }
    {
        bool absolute = hw_parse_absolute(hw_rpt, (uint16_t)rpt_len, &g_log_max);
        hw_parse_axes(hw_rpt, (uint16_t)rpt_len);  /* раскладка осей X/Y */
        return absolute;
    }
}

/* первый «непонятный» HID (protocol 0) - кандидат на L2/L3/L4
 * цепочки классификации.  Мышью его больше не назначаем молча. */
static bool    g_xh_amb_ok;
static uint8_t g_xh_amb_iface;
static uint8_t g_xh_amb_sub;

/* ── Найти HID-интерфейс (boot-мышь в приоритете) + interrupt-IN EP ───────── */
static bool hw_find_hid(const uint8_t *cfg, uint16_t total) {
    /* «любая мышь» - ранжирование интерфейсов: 3 = boot-мышь
     * (subclass 1 + protocol 2), 2 = мышь по protocol 2 БЕЗ boot-подкласса
 * (беспроводные приёмники в composite-девайсе). - ГЛАВНЫЙ ФИКС
     * «ОС ДУМАЕТ, ЧТО КЛАВА ЭТО МЫШЬ»: клавиатурный интерфейс (protocol 1)
     * НЕ ВЫБИРАЕТСЯ НИКОГДА - его подбирает hw_find_kbd («ты клава? - да!»).
     * «Прочий HID» (protocol 0) мышью больше НЕ назначается даже рангом -
     * он запоминается отдельно (g_xh_amb_*) и проходит цепочку L2
     * usage-скан -> L3 LED-проба -> L4 (неоднозначно = клава).  Мышью его
     * делает только L2. */
    int best = -1;
    int best_rank = 0;
    int pos = 0;
    g_hid_proto = 0xFFu;                      /* «не HID» по умолчанию */
    g_xh_amb_ok = false;                      /* кандидат сброшен */
    g_xh_amb_sub = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE && cfg[pos + 5] == 0x03u) {   /* HID class */
            int rank = 0;
            if (cfg[pos + 7] == 0x02u)                            /* mouse   */
                rank = (cfg[pos + 6] == 0x01u) ? 3 : 2;           /* boot?   */
            else if (cfg[pos + 7] != 0x01u && !g_xh_amb_ok) {
                /* не мышь и не клава - первый такой в кандидаты */
                g_xh_amb_ok    = true;
                g_xh_amb_iface = cfg[pos + 2];
                g_xh_amb_sub   = cfg[pos + 6];       /* subclass (boot?) */
            }
            if (rank > best_rank) {
                best        = cfg[pos + 2];
                best_rank   = rank;
                g_hid_proto = cfg[pos + 7];   /* кто он по дескриптору */
            }
        }
        pos += len;
    }
    if (best < 0) return false;
    g_iface      = (uint8_t)best;
    g_boot_iface = (best_rank == 3);

    bool in_iface = false;
    pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) {
            in_iface = (cfg[pos + 2] == g_iface);
        } else if (type == DESC_ENDPOINT && in_iface) {
            uint8_t addr = cfg[pos + 2];
            uint8_t attr = cfg[pos + 3];
            if ((addr & 0x80u) && (attr & 0x03u) == 0x03u) {         /* INT IN */
                uint16_t mps = cfg[pos + 4] | ((uint16_t)cfg[pos + 5] << 8);
                g_ms_xep = (uint8_t)(addr & 0x0Fu);   /* [#424] НОМЕР EP (композиты: мышь на EP2!) */
                g_mps = (uint8_t)(mps > 64 ? 64 : mps);   /* [#425] кламп 64, НЕ 16 */
                if (g_mps < 4) g_mps = 4;
                return true;
            }
        }
        pos += len;
    }
    return false;
}

/* ── Best-effort Disable Slot (при неудачном перечислении) ──────────────────
 * работает со слотом ПЕРЕЧИСЛЕНИЯ (g_eslot). Слоты ЖИВЫХ устройств
 * (мышь g_slot, клавиатура g_kbd_slot) эта функция больше не глушит:
 * раньше «зачистка» смотрела на g_slot и hotplug при активной мыши
 * убивал её слот, если в другом порту начиналось перечисление. */
static void hw_disable_slot(void) {
    int s = g_eslot;
    if (s <= 0) return;
    uint32_t trb = ring_put(&r_cmd, 0, 0,
                            TRB_TYPE(10) | ((uint32_t)s << 24));
    (void)trb;
    db_command();
    uint32_t cc, sl;
    (void)wait_cmd(trb, &cc, &sl);   /* результат игнорируем */
    hw_dcbaa[s] = 0;                 /* освободить выходной контекст */
    g_eslot = 0;
}

/* отключить слот клавиатуры (устройство выдернули из порта). */
static void hw_kbd_disable_slot(void) {
    int s = g_kbd_slot;
    if (s <= 0) return;
    if (g_kb_xi >= 0) g_xi = g_kb_xi;   /* слот живёт на инстансе клавы */
    uint32_t trb = ring_put(&r_cmd, 0, 0,
                            TRB_TYPE(10) | ((uint32_t)s << 24));
    (void)trb;
    db_command();
    uint32_t cc, sl;
    (void)wait_cmd(trb, &cc, &sl);
    hw_dcbaa[s] = 0;
    g_kbd_slot = 0;
    g_kb_xi = -1;                       /* 0.3.0: инстанс свободен */
    lk_drop();                          /* [#474] снимок клавы погашен */
    oh_kbd_state_reset();               /* [#476] prev_k не даёт фантомов следующей клаве */
    usbdev_release(USBDEV_ROLE_KBD, "kbd detach");
}

/* пере-постановка TRB мыши. Пока драйвер перечисляет НОВОЕ устройство
 * (клавиатуру в другом порту), события мыши съедаются ожиданиями команд
 * (wait_cmd/ctrl_xfer пропускают «чужие» события) без пере-arm - без этого
 * мышь замолкала НАВСЕГДА после первого же подключения клавиатуры. */
static void hw_arm_mouse_again(void) {
    if (g_active && g_slot != 0) hw_arm();
}

/* [#456] пере-постановка TRB КЛАВИАТУРЫ: события клавиатуры съедаются
 * ожиданиями команд (wait_cmd/ctrl_xfer потребляют ВСЁ кольцо событий)
 * при перечислении соседнего устройства — TRB ретайрится контроллером,
 * а драйвер события не видит.  У мыши был hw_arm_mouse_again(); у
 * клавиатуры его не было ВООБЩЕ — клава замолкала НАВСЕГДА после подъёма
 * мыши/флешки на том же xHCI (фото пользователя: клава умирает сразу
 * после перечисления мыши). */
static void hw_kbd_arm(void);   /* определена ниже (bind-путь клавиатуры) */
static void hw_kbd_arm_again(void) {
    if (g_kbd_slot == 0 || g_kb_xi < 0) return;
    int sv = g_xi;
    hw_kbd_arm();
    g_xi = sv;   /* hw_kbd_arm переключает g_xi на инстанс клавы — вернём */
}

/* ── КЛАВИАТУРА на xHCI - «ты клава? - да!» ─────────────────────────────────
 * hw_find_hid БОЛЬШЕ НЕ отдаёт клавиатурные интерфейсы (protocol == 1),
 * поэтому отдельный поиск: HID + protocol 1 (keyboard), boot-subclass в
 * приоритете.  Привязка = SET_CONFIG + SET_IDLE/SET_PROTOCOL(boot) +
 * Configure Endpoint (свой слот/контекст/кольцо EP1) + первый TRB.
 * Отчёты декодирует ОБЩИЙ с OHCI-путём oh_kbd_process -> ps2_kbd_inject. */
static bool hw_find_kbd(const uint8_t *cfg, uint16_t total) {
    int     k_iface = -1;
    bool    k_boot  = false, k_epok = false;
    uint8_t k_mps = 8, k_ep = 1;
    int pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE && cfg[pos + 5] == 0x03u &&
            cfg[pos + 7] == 0x01u) {                       /* protocol = kbd */
            int  cur  = cfg[pos + 2];
            bool boot = (cfg[pos + 6] == 0x01u);           /* subclass = boot */
            if (k_iface < 0 || (boot && !k_boot)) { k_iface = cur; k_boot = boot; }
        }
        pos += len;
    }
    if (k_iface < 0) return false;
    bool in_k = false;
    pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) {
            in_k = (cfg[pos + 2] == (uint8_t)k_iface);
        } else if (type == DESC_ENDPOINT && in_k && (cfg[pos + 2] & 0x80u) &&
                   (cfg[pos + 3] & 0x03u) == 0x03u) {      /* INT IN        */
            uint16_t mps = cfg[pos + 4] | ((uint16_t)cfg[pos + 5] << 8);
            if (mps < 8)  mps = 8;
            if (mps > 16) mps = 16;
            k_ep   = (uint8_t)(cfg[pos + 2] & 0xFu);
            k_mps  = (uint8_t)mps;
            k_epok = true;
            break;
        }
        pos += len;
    }
    if (!k_epok) return false;
    g_kbd_xiface = (uint8_t)k_iface;
    g_kbd_xmps   = k_mps;
    g_kbd_xep    = k_ep;   /* -fix: индекс контекста/звонок = 2*EP+1 */
    return true;
}

/* постановка interrupt-IN TRB клавиатуры + звонок. */
static void hw_kbd_arm(void) {
    if (g_kb_xi >= 0) g_xi = g_kb_xi;   /* инстанс клавиатуры */
    ring_put(&r_kbd_ep1, vphys(hw_kbd_pkt), lk_xmps(),
             TRB_TYPE(TRB_TR_NORMAL) | TRB_IOC);   /* [#474] MPS — из СНИМКА живой клавы */
    db_kbd_in();
}

/* ── «ТЫ КЛАВА?» - LED-проба ────────────────────────────────────────────────
 * ОС просит ЛЮБОЙ непонятный HID-интерфейс зажечь NumLock|CapsLock
 * (SET_REPORT, Output).  У клавиатуры есть output-отчёт LED - она отвечает
 * ACK и огоньки правда горят.  У мыши output-отчёта НЕТ - она честно
 * отвечает STALL.  Подстраховка - скан report descriptor'а как
 * hid_scan_report() в Linux: если у top-level Collection стоит
 * Usage(Generic Desktop, Keyboard) и мыши там нет - тоже клава. */

/* длина report descriptor'а интерфейса из конфигурации (0 = нет) */
static int hw_rpt_len(const uint8_t *cfg, uint16_t total, uint8_t iface) {
    int pos = 0;
    bool in_iface = false;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) in_iface = (cfg[pos + 2] == iface);
        else if (type == 0x21u && in_iface)          /* HID descriptor      */
            return cfg[pos + 7] | ((int)cfg[pos + 8] << 8);
        pos += len;
    }
    return 0;
}

/* сам скан usage-ов (чистый разбор массива - общий для xHCI и OHCI) */
static void hw_usage_scan(const uint8_t *rpt, int n, bool *kbd, bool *mouse) {
    *kbd = false;
    *mouse = false;
    int depth = 0;
    uint32_t page = 0, usage = 0;
    int pos = 0;
    while (pos < n) {
        uint8_t it = rpt[pos++];
        uint32_t sz = (uint32_t)(it & 0x03u);
        if (sz == 3u) sz = 4u;
        if (pos + (int)sz > n) break;
        uint32_t val = 0;
        for (uint32_t i = 0; i < sz; i++)
            val |= ((uint32_t)rpt[pos + i]) << (8 * i);
        pos += (int)sz;
        switch (it & 0xFCu) {
            case 0x04u: page = val; break;      /* Usage Page (global)      */
            case 0x08u: usage = val; break;     /* Usage (local)            */
            case 0xA0u:                         /* Collection (main)        */
                if (depth == 0) {
                    if (page == 0x01u && usage == 0x06u) *kbd = true;
                    if (page == 0x01u && usage == 0x02u) *mouse = true;
                }
                depth++;
                usage = 0;
                break;
            case 0xC0u:                         /* End Collection           */
                if (depth > 0) depth--;
                break;
            default: break;
        }
    }
}

/* ── Классификация HID: уровни правды по очереди ────────────────────────────
 * VERD_KBD / VERD_MOUSE / VERD_UNKNOWN.  L1 Interface Descriptor
 * (protocol 1/2) -> L2 Report Descriptor (Usage Page/Usage - как evdev
 * capabilities) -> L3 LED-проба -> L4: неоднозначно = КЛАВИАТУРА.
 * Порт (2.0/3.0) на тип не влияет. */
#define VERD_KBD     0
#define VERD_MOUSE   1
#define VERD_UNKNOWN 2

static int hid_verdict(int proto, bool u_kbd, bool u_mouse, bool led_ack) {
    if (proto == 1) return VERD_KBD;              /* L1: protocol = keyboard  */
    if (proto == 2) return VERD_MOUSE;            /* L1: protocol = mouse     */
    if (u_kbd && !u_mouse) return VERD_KBD;       /* L2: Usage(Page1, 0x06)   */
    if (u_mouse && !u_kbd) return VERD_MOUSE;     /* L2: Usage(Page1, 0x02)   */
    if (led_ack)           return VERD_KBD;       /* L3: огоньки горели       */
    return VERD_UNKNOWN;                          /* L4: неоднозначно -> клава */
}

/* имя скорости для строки классификации.  Порт (2.0/3.0) влияет только
 * на скорость и на то, какой HC обслуживает, - НЕ на тип. */
static const char *hid_port_speed(uint32_t sp) {
    switch (sp) {
        case PORT_SPEED_SUPER: return "3.0";
        case PORT_SPEED_HIGH:  return "2.0";
        default:               return "1.x";
    }
}

/* Строка классификации: в журнале видно, ЧТО выбрано, ПО КАКОМУ
 * уровню и НА КАКОМ порту.  Формат:
 * HID: addr=3 port=2.0 proto=keyboard via=interface */
static void hid_report_line(const char *speed_str, int port, int addr,
                            const char *proto, const char *via, int iface) {
    char nb[12];
    serial_puts("HID: addr=");
    serial_puts(itoa((uint32_t)addr, nb, 10));
    serial_puts(" port=");
    serial_puts(itoa((uint32_t)port, nb, 10));
    serial_puts(" speed=");
    serial_puts(speed_str);
    serial_puts(" proto=");
    serial_puts(proto);
    serial_puts(" via=");
    serial_puts(via);
    serial_puts(" iface=");
    serial_puts(itoa((uint32_t)iface, nb, 10));
    serial_puts("\n");
}

/* мышиную личность (INT IN EP + mps) для ПРОИЗВОЛЬНОГО интерфейса -
 * «непонятный» HID, которого L2 опознал мышью (via=descriptor). */
static bool hw_grab_mouse_iface(const uint8_t *cfg, uint16_t total,
                                uint8_t iface) {
    bool in_i = false, epok = false;
    int pos = 0;
    uint16_t mps = 8;
    uint8_t  ep  = 1;      /* [#424] номер EP из дескриптора */
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) in_i = (cfg[pos + 2] == iface);
        else if (type == DESC_ENDPOINT && in_i && (cfg[pos + 2] & 0x80u) &&
                 (cfg[pos + 3] & 0x03u) == 0x03u) {                  /* INT IN */
            mps  = cfg[pos + 4] | ((uint16_t)cfg[pos + 5] << 8);
            ep   = (uint8_t)(cfg[pos + 2] & 0x0Fu);
            epok = true;
            break;
        }
        pos += len;
    }
    if (!epok) return false;
    g_iface      = iface;
    g_boot_iface = false;
    g_mps = (uint8_t)(mps > 64 ? 64 : mps);   /* [#425] кламп 64, НЕ 16 */
    if (g_mps < 4) g_mps = 4;
    g_ms_xep = (uint8_t)(ep & 0x0Fu);
    return true;
}

/* Вердикт по «непонятному» HID - цепочка уровней:
 * L2 usage-скан report descriptor'а (пассивная правда - как
 * hid_scan_report() в ядре Linux) -> L3 LED-проба «зажги
 * NumLock+CapsLock» (активная; таймаут ограничен самим ctrl_xfer -
 * «мёртвое» устройство перечисление не вешает) -> L4 VERD_UNKNOWN -
 * дальше решает вызывающий (неоднозначно = клава).
 * via - чем опознано: "descriptor" / "led" / "". */
static int hw_ask_verdict(uint8_t iface, const uint8_t *cfg, uint16_t total,
                          const char **via) {
    bool kbd = false, mouse = false;
    bool ok;
    int rl = hw_rpt_len(cfg, total, iface);
    if (rl > 0 && rl <= (int)sizeof(hw_rpt)) {
        memset(hw_rpt, 0, sizeof(hw_rpt));
        g_ctrl_quiet = true;
        ok = ctrl_xfer(0x81, USB_REQ_GET_DESCRIPTOR, (USB_DT_REPORT << 8),
                       iface, (uint16_t)rl, hw_rpt);
        g_ctrl_quiet = false;
        if (ok) {
            hw_usage_scan(hw_rpt, rl, &kbd, &mouse);
            int vd = hid_verdict(0, kbd, mouse, false);       /* L2 */
            if (vd != VERD_UNKNOWN) {
                *via = "descriptor";
                serial_puts(vd == VERD_KBD
                    ? "MOUSEHW: дескриптор говорит КЛАВИАТУРА (usage=0x06)\n"
                    : "MOUSEHW: дескриптор говорит МЫШЬ (usage=0x02)\n");
                return vd;
            }
        }
    }
    /* [#446] LED по ТЕКУЩЕМУ состоянию Num/Caps, а не «зажечь всё»:
     * раньше проба зажигала NumLock|CapsLock (0x03) навсегда. */
    uint8_t leds;
    { int ln = 0, lc = 0; ps2_kbd_led_state(&ln, &lc);
      leds = (uint8_t)((ln ? 1u : 0u) | (lc ? 2u : 0u)); }
    g_ctrl_quiet = true;                        /* STALL мыши - не ошибка   */
    ok = ctrl_xfer(0x21, USB_HID_SET_REPORT, 0x0200u, iface, 1, &leds);
    g_ctrl_quiet = false;
    if (ok) {
        serial_puts("MOUSEHW: LED-проба: SET_REPORT подтверждён - это КЛАВИАТУРА\n");
        *via = "led";
        return VERD_KBD;
    }
    serial_puts("MOUSEHW: LED-проба не ответила - неоднозначно (вид не доказан)\n");
    *via = "";
    return VERD_UNKNOWN;
}

/* взять интерфейс iface в клавиатурный путь (найти его INT IN EP).
 * Для клавы, которая стесняется назваться (bInterfaceProtocol = 0). */
static bool hw_kbd_grab_iface(const uint8_t *cfg, uint16_t total, uint8_t iface) {
    bool in_k = false, epok = false;
    int pos = 0;
    uint8_t  ep  = 1;
    uint16_t mps = 8;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) {
            in_k = (cfg[pos + 2] == iface);
        } else if (type == DESC_ENDPOINT && in_k && (cfg[pos + 2] & 0x80u) &&
                   (cfg[pos + 3] & 0x03u) == 0x03u) {      /* INT IN        */
            mps  = cfg[pos + 4] | ((uint16_t)cfg[pos + 5] << 8);
            ep   = (uint8_t)(cfg[pos + 2] & 0xFu);
            epok = true;
            break;
        }
        pos += len;
    }
    if (!epok) return false;
    if (mps < 8)  mps = 8;
    if (mps > 16) mps = 16;
    g_kbd_xiface = iface;
    g_kbd_xmps   = (uint8_t)mps;
    g_kbd_xep    = ep;
    return true;
}

/* привязать КЛАВИАТУРУ в слоте перечисления g_eslot (устройство уже
 * адресовано, дескрипторы прочитаны - мы пришли из hw_enumerate step 6).
 * Возвращает true, если клавиатура живёт (слот переходит ей навсегда). */
static bool hw_bind_kbd(uint8_t cfgval) {
    uint32_t speed = g_xh_speed;
    uint32_t port  = g_xh_port;
    /* 1. SET_CONFIGURATION */
    if (!ctrl_xfer(0x00, USB_REQ_SET_CONFIG, cfgval, 0, 0, 0)) {
        xh_diag_set("КЛАВИАТУРА: SET_CONFIG не прошёл");
        return false;
    }
    /* 2. boot-протокол + idle (STALL игнорируем - как у мыши ) */
    (void)ctrl_xfer(0x21, USB_HID_SET_PROTOCOL, 0, g_kbd_xiface, 0, 0);
    (void)ctrl_xfer(0x21, USB_HID_SET_IDLE, 0, g_kbd_xiface, 0, 0);
    /* 3. Свой выходной контекст: слот-контекст уже записан контроллером в
     * hw_devctx (Address Device) - переносим его в hw_kbd_devctx и
     * перенаправляем DCBAA, иначе Configure Endpoint мыши затёр бы клаву.
     * [#423] копируем КОНТЕКСТ ЦЕЛИКОМ (512 Б): раньше только g_csz байт
     * (слот-контекст), и EP0-контекст клавиатуры в DCBAA оставался НУЛЁВЫМ
     * (MPS=0, dequeue=0) — на строгом AMD при повторном fetch слот умирал.
     * Мышь уже копировала честные sizeof(hw_mouse_devctx) байт. */
    memset(hw_kbd_devctx, 0, sizeof(hw_kbd_devctx));
    memcpy(hw_kbd_devctx, hw_devctx, sizeof(hw_kbd_devctx));
    hw_dcbaa[g_eslot] = (uint64_t)vphys(hw_kbd_devctx);
    /* 4. Кольцо EP1 клавиатуры - с чистого листа (циклы после переподключ.) */
    for (int i = 0; i < 8; i++) hw_kbd_ep1[i].control = 0;
    r_kbd_ep1.idx = 0;
    r_kbd_ep1.cyc = 1;
    /* 5. Configure Endpoint: slot + interrupt-IN EP клавиатуры, своё кольцо */
    memset(hw_kbd_inctx, 0, sizeof(hw_kbd_inctx));
    uint32_t *icc = (uint32_t *)hw_kbd_inctx;
    icc[0] = 0;                                  /* drop flags              */
    /* -fix: EP клавиатуры любой (композиты дают EP2): индекс контекста
     * и звонок = 2*EP+1 (EP1 IN = 3 - как было). */
    icc[1] = ICC_SLOT | (1u << (2u * (uint32_t)g_kbd_xep + 1u));
    uint32_t *sd = (uint32_t *)(hw_kbd_inctx + g_csz);
    /* Context Entries = индекс последнего контекста (у композитных
     * клавиатур interrupt-EP бывает 2 -> индекс 5): зашитая «3» на
     * строгих контроллерах даёт Configure Endpoint cc=Parameter. */
    sd[0] = SLOT_LAST_CTX(2u * (uint32_t)g_kbd_xep + 1u) | SLOT_SPEED(speed);
    sd[1] = SLOT_ROOT_PORT(port);
    /* 0.3.0 FIX (баг 0.3.1+): EP-контекст в INPUT-контексте лежит по
     * смещению 32 + 32*epid (спека 6.2.5: input = control ctx + полная
     * структура device-контекста).  Раньше писали по 32*epid — QEMU/HS
     * читали там НОЛИ: ring->dequeue = 0, doorbell «работал», но отчёты
     * клавиатуры НЕ приходили ВООБЩЕ («клава видится, а клавиши нет»). */
    uint32_t *d = (uint32_t *)(hw_kbd_inctx + g_csz +
                               (2u * (uint32_t)g_kbd_xep + 1u) * g_csz);
    d[0] = EP_INTERVAL(3);                       /* 2^3 x 125us = 1 мс      */
    d[1] = EP_CERR3 | EP_TYPE_INT_IN | EP_MAX_PACKET(g_kbd_xmps);
    d[2] = vphys(hw_kbd_ep1) | 1u;               /* DCS=1                   */
    d[3] = 0;
    d[4] = ((uint32_t)g_kbd_xmps << 16) | 8u;    /* ESIT, avg TRB = 8       */
    d[5] = d[6] = d[7] = 0;
    uint32_t trb = ring_put(&r_cmd, vphys(hw_kbd_inctx), 0,
                            TRB_TYPE(TRB_CR_CONFIG_EP) |
                            ((uint32_t)g_eslot << 24));
    db_command();
    uint32_t cc = 0, sl = 0;
    if (!wait_cmd(trb, &cc, &sl)) {
        serial_puts("MOUSEHW: КЛАВИАТУРА: configure endpoint failed cc=");
        serial_puthex(cc);
        serial_puts(" ("); serial_puts(cc_name(cc)); serial_puts(")\n");
        return false;
    }
    /* 6. Слот переходит клавиатуре насовсем */
    g_kbd_slot = g_eslot;
    g_eslot    = 0;
    g_kb_xi    = g_xi;      /* 0.3.0: инстанс-владелец клавиатуры */
    oh_kbd_state_reset();   /* [#476] prev_k/«первый отчёт» — с чистого листа */
    lk_save_x();            /* [#474] СНИМОК живой клавы (xHCI) */
    hw_kbd_arm();
    serial_puts("MOUSEHW: USB-КЛАВИАТУРА найдена (xHCI порт ");
    serial_puthex((uint32_t)port);
    serial_puts(", слот ");
    serial_puthex((uint32_t)g_kbd_slot);
    serial_puts(", iface ");
    serial_puthex((uint32_t)g_kbd_xiface);
    serial_puts(") - сканкоды идут в ps2.c\n");
    usbmap_pick(0x30, "keyboard", (int)port,
                xh_port_ver(port, g_xh_speed) == 3 ? "3.0" : "2.0");   /* [#448] честная версия ПОРТА */
    xh_diag_set("КЛАВИАТУРА готова: порт ");
    xh_diag_num(port);
    xh_diag_add(", слот ");
    xh_diag_num((uint32_t)g_kbd_slot);
    return true;
}

/* ── Полное перечисление HID мыши на данном порту ─────────────────────────── */
static bool hw_enumerate(uint8_t port, uint32_t speed) {
    g_xh_port  = port;                    /* трекинг для СНИМКА */
    g_xh_speed = speed;
    g_ms_xep   = 1;                       /* [#424] до чтения дескриптора — классика EP1 */
    /* масштаб абсолютного планшета - с чистого листа: g_log_max от
     * ПРЕДЫДУЩЕГО устройства (hotplug/замена) иначе масштабировал бы
     * оси нового; дефолт 32767 перепишет hw_parse_absolute при успехе. */
    g_log_max = 32767;
    /* КАНОН Linux xhci-ring.c - enqueue-состояние кольца обязано
     * совпадать с fresh dequeue контроллера (Address Device ставит EP0
     * deque = hw_ep0[0], DCS=1).  Раньше r_ep0 продолжал с залежалой
     * позиции и залежалым cycle-битом, поэтому ЛЮБОЕ ВТОРОЕ перечисление
     * на контроллере (клава после мыши, hotplug) валилось по STALE TRB:
     * «клава в 3.0 не работает», «не работает! ничего». */
    for (int i = 0; i < 8; i++) hw_ep0[i].control = 0;
    r_ep0.idx = 0;
    r_ep0.cyc = 1;
    xh_diag_set("enable slot (порт ");
    xh_diag_num(port);
    xh_diag_add(")");
    /* 1. Enable Slot */
    uint32_t trb = ring_put(&r_cmd, 0, 0, TRB_TYPE(TRB_CR_ENABLE_SLOT));
    db_command();
    uint32_t cc = 0, slot = 0;
    if (!wait_cmd(trb, &cc, &slot)) {
        serial_puts("MOUSEHW: enable slot failed cc=");
        serial_puthex(cc);
        serial_puts(" ("); serial_puts(cc_name(cc)); serial_puts(")\n");
        g_xh_cc = cc;
        xh_diag_set("ENABLE SLOT cc=");
        xh_diag_add(cc_name(cc));
        g_eslot = 0;
        return false;
    }
    g_eslot = (int)slot;   /* слот перечисления; g_slot выставится при успехе */
    /* [#411] КОЛЛИЗИЯ СЛОТОВ (фото: мышь slot=1 И клавиатура slot=1 на
     * одном xHCI#1 — Enable Slot вернул уже занятый ID: слот живого
     * устройства осиротел, его TRB повисли и мышь «умерла» на
     * выдёргивании клавиатуры). Спека запрещает выдачу занятого слота;
     * если контроллер так ответил — перечисление дальше идти НЕ может:
     * грозная строка в журнал + отказ. Слот НЕ дизейблим — он
     * принадлежит живому устройству! */
    if (slot != 0 &&
        ((g_slot == (int)slot && g_active && g_ms_xi == g_xi) ||
         (g_kbd_slot == (int)slot && g_kb_xi == g_xi) ||
         (g_xm_slot == (int)slot && g_xm_xi == g_xi))) {
        serial_puts("MOUSEHW: СЛОТ-КОЛЛИЗИЯ: Enable Slot вернул ");
        serial_puthex(slot);
        serial_puts(" - он уже занят (мышь слот ");
        serial_puthex((uint32_t)g_slot);
        serial_puts(", клавиатура слот ");
        serial_puthex((uint32_t)g_kbd_slot);
        serial_puts(") - перечисление ОТМЕНЕНО, живое устройство под защитой\n");
        g_eslot = 0;
        return false;
    }
    /* Выходной контекст слота — ДОЛЖЕН быть в DCBAA до Address Device:
     * QEMU/контроллер пишут и читают контексты по этому адресу. */
    hw_dcbaa[g_eslot] = (uint64_t)vphys(hw_devctx);

    /* 2. Address Device (BSR=0): адрес устройства = slot id.
 * корень отказов - найден — неверные битовые поля в
     * контекстах (SLOT_ROOT_PORT p<<8 вместо p<<16; EP_CERR3 3<<27
     * вместо 3<<1): контроллер видел RHPort=0 и MaxPacket=0x1808 и
     * честно отвечал cc=0x11 (Parameter).  Повтор с 64-байтным stride
     * (AMD CSZ test) оставлен как страховка на случай, если CSZ-бит
     * какого-нибудь чипа врёт. */
    int addr_ok = 0;
    for (int att = 0; att < 2 && !addr_ok; att++) {
        memset(hw_inctx, 0, sizeof(hw_inctx));
        inctx_icc(ICC_SLOT | ICC_EP0);
        inctx_slot(speed, port, 1);
        inctx_ep0(speed == PORT_SPEED_LOW ? 8u :
                  (speed == PORT_SPEED_SUPER ? 512u : 64u), 0);
        {   /* что МЫ записали в input-контекст ДО doorbell. Сравнить
             * с дампом «inctx …» после отказа: расхождение = контроллер
 * читает не то, что мы писали (кэш/адрес/stride). с
             * верными полями обе строки совпадают байт в байт. */
            uint32_t *ew = (uint32_t *)(hw_inctx + 2u * g_csz);
            serial_puts("MOUSEHW: addr wr att=");
            serial_puthex((uint32_t)(att + 1));
            serial_puts(" csz=");
            serial_puthex(g_csz);
            serial_puts(" ep0_1=");
            serial_puthex(ew[1]);
            serial_puts("\n");
        }
        trb = ring_put(&r_cmd, vphys(hw_inctx), 0,
                       TRB_TYPE(TRB_CR_ADDR_DEV) | ((uint32_t)g_eslot << 24));
        db_command();
        if (wait_cmd(trb, &cc, &slot)) { addr_ok = 1; break; }
        serial_puts("MOUSEHW: address device failed cc=");
        serial_puthex(cc);
        serial_puts(" ("); serial_puts(cc_name(cc)); serial_puts(")\n");
        {   /* дамп входного контекста одной строкой (+ ep0_2 = deq ptr) */
            uint32_t *icc = (uint32_t *)hw_inctx;
            uint32_t *sd  = (uint32_t *)(hw_inctx + g_csz);
            uint32_t *ed  = (uint32_t *)(hw_inctx + 2u * g_csz);
            serial_puts("MOUSEHW: inctx add=");
            serial_puthex(icc[1]);
            serial_puts(" slot0=");
            serial_puthex(sd[0]);
            serial_puts(" slot1=");
            serial_puthex(sd[1]);
            serial_puts(" ep0_1=");
            serial_puthex(ed[1]);
            serial_puts(" ep0_2=");
            serial_puthex(ed[2]);
            serial_puts("\n");
        }
        if (att == 0 && cc == 17 && g_csz == 32u) {
            serial_puts("MOUSEHW: retry with 64-byte ctx stride (AMD CSZ test)\n");
            g_csz = 64u;
            continue;
        }
        break;
    }
    if (!addr_ok) {
        g_xh_cc = cc;
        xh_diag_set("ADDRESS DEVICE cc=");
        xh_diag_add(cc_name(cc));
        xh_diag_add(" x2 (CSZ 32+64)");
        hw_disable_slot();
        return false;
    }
    g_xh_cc = CC_SUCCESS;                /* СНИМОК показывает Success,
                                            а не мусор неинициализированной
                                            переменной (фото: 0x32) */
    xh_diag_set("адрес выдан (слот ");
    xh_diag_num((uint32_t)g_slot);
    xh_diag_add("), читаю дескрипторы");

    /* 3. GET_DESCRIPTOR(Device, 18) */
    uint8_t dd[18];
    if (!ctrl_xfer(0x80, USB_REQ_GET_DESCRIPTOR, (USB_DT_DEVICE << 8),
                   0, 18, dd)) {
        xh_diag_set("GET_DESCRIPTOR(18) не прошёл");
        hw_disable_slot();
        return false;
    }
    uint16_t vid = (uint16_t)(dd[8] | ((uint16_t)dd[9] << 8));
    uint16_t pid = (uint16_t)(dd[10] | ((uint16_t)dd[11] << 8));
    usb_db_report(vid, pid);   /* подбор драйвера в мини-базе */

    /* 4. Evaluate Context: точный EP0 mps из bMaxPacketSize0 */
    memset(hw_inctx, 0, sizeof(hw_inctx));
    inctx_icc(ICC_SLOT | ICC_EP0);
    inctx_slot(speed, port, 1);
    /* у SuperSpeed bMaxPacketSize0 кодируется как 9 (=2^9=512). */
    uint32_t mps0 = dd[7];
    if (mps0 == 9u) mps0 = 512u;
    if (mps0 == 0u) mps0 = (speed == PORT_SPEED_SUPER) ? 512u : 8u;
    inctx_ep0(mps0, 0);
    trb = ring_put(&r_cmd, vphys(hw_inctx), 0,
                   TRB_TYPE(TRB_CR_EVAL_CTX) | ((uint32_t)g_eslot << 24));
    db_command();
    if (!wait_cmd(trb, &cc, &slot)) {
        serial_puts("MOUSEHW: evaluate context failed cc=");
        serial_puthex(cc);
        serial_puts(" ("); serial_puts(cc_name(cc)); serial_puts(")\n");
        g_xh_cc = cc;
        xh_diag_set("EVALUATE CONTEXT cc=");
        xh_diag_add(cc_name(cc));
        hw_disable_slot();
        return false;
    }

    /* 5. GET_DESCRIPTOR(Configuration): 9 байт → полная длина */
    uint8_t cfg9[9];
    if (!ctrl_xfer(0x80, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIG << 8),
                   0, 9, cfg9)) {
        xh_diag_set("GET_DESCRIPTOR(CFG,9) не прошёл");
        hw_disable_slot();
        return false;
    }
    uint16_t total = (uint16_t)(cfg9[2] | ((uint16_t)cfg9[3] << 8));
    if (total > 255) total = 255;
    uint8_t cfg[256];
    if (!ctrl_xfer(0x80, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIG << 8),
                   0, total, cfg)) {
        xh_diag_set("GET_DESCRIPTOR(CFG) не прошёл");
        hw_disable_slot();
        return false;
    }

    /* 6. «КТО ТЫ?» - классификация HID:
     * L1 Interface Descriptor (protocol 1=клава, 2=мышь) -> L2 Report
     * Descriptor (usage-скан) -> L3 LED-проба -> L4: неоднозначно
     * = КЛАВА (клавиатура всегда приоритетнее - без неё не ввести ни одной
     * команды).  Устройство не теряется никогда, порт в классификации
     * не участвует, составное устройство получает
 * ОБА драйвера (composite). настройки ЖИВОЙ мыши сохраняем -
     * если это вторая мышь («мышь уже работает»), их вернуть как были. */
    /* 6. «КТО ТЫ?» (0.3.0): ПОРТ уже представился диспетчеру (в
     * xhci_try_port: xHCI = «USB 3.0»).  Диспетчер usbdev подбирает
     * драйвер из ШЕСТИ: «драйвер клавиатуры 3.0» пробуется ПЕРВЫМ
     * (реестр!), «драйвер мыши 3.0» — если слот клавиатуры занят или
     * устройство не клавиатура.  Не подошёл ни один — НЕИЗВЕСТНОЕ
     * УСТРОЙСТВО.  Классификация L1/L2/L3 осталась: она определяет,
     * ЧТО устройство умеет (kbd_ok / mouse_ok). */
    bool cfg_done = false;
    {
        bool mouse_ok = hw_find_hid(cfg, total);   /* + g_iface/g_mps/g_ep */
        bool kbd_ok   = hw_find_kbd(cfg, total);   /* + g_kbd_xiface/xmps/xep */
        const char *via = "interface";

        /* непонятный HID (protocol 0): L2 usage-скан + L3 LED-проба */
        if (!kbd_ok && !mouse_ok && g_xh_amb_ok) {
            if (!ctrl_xfer(0x00, USB_REQ_SET_CONFIG, cfg9[5], 0, 0, 0)) {
                xh_diag_set("SET_CONFIGURATION не прошёл");
                hw_disable_slot();
                return false;
            }
            cfg_done = true;
            int vd = hw_ask_verdict(g_xh_amb_iface, cfg, total, &via);
            if (vd == VERD_UNKNOWN && g_xh_amb_sub == 1u) {
                vd = VERD_KBD;  via = "boot-subclass";
            }
            if (vd == VERD_KBD &&
                hw_kbd_grab_iface(cfg, total, g_xh_amb_iface)) {
                kbd_ok = true;
            } else if (vd == VERD_MOUSE &&
                       hw_grab_mouse_iface(cfg, total, g_xh_amb_iface)) {
                mouse_ok = true;
            }
        } else if (mouse_ok && !g_boot_iface) {
            /* не-boot мышь: L2 usage-скан решает ДО привязки (как раньше) */
            if (!ctrl_xfer(0x00, USB_REQ_SET_CONFIG, cfg9[5], 0, 0, 0)) {
                xh_diag_set("SET_CONFIGURATION не прошёл");
                hw_disable_slot();
                return false;
            }
            cfg_done = true;
            int vd = hw_ask_verdict(g_iface, cfg, total, &via);
            if (vd == VERD_KBD &&
                hw_kbd_grab_iface(cfg, total, g_iface)) {
                kbd_ok   = true;
                mouse_ok = false;
            }
            /* VERD_MOUSE/UNKNOWN: остаётся мышью — общий путь ниже */
        }

        int pver = xh_port_ver(port, speed);   /* [#448] Supported Protocol, не жёсткое «3» */
        int role = usbdev_try_bind(pver, kbd_ok, mouse_ok,
                                   xhci_bus_label(), (int)port, vid, pid);
        if (role == USBDEV_ROLE_KBD) {
            if (hw_bind_kbd(cfg9[5])) {
                hid_report_line(hid_port_speed(speed), port,
                                (int)g_kbd_slot, "keyboard", via,
                                (int)g_kbd_xiface);
                g_port_role[port] = PR_KBD;   /* [#460]: массив 256 = весь uint8_t */
                g_kbd_xport = port;
                usbdev_register(0, (int)port, vid, pid, UD_KBD, 1,
                                (int)g_kbd_slot, 1);
                hw_arm_mouse_again();
                return false;
            }
            usbdev_release(USBDEV_ROLE_KBD, "привязка не удалась");
            g_port_role[port] = PR_OTHER;   /* [#460] */

            usbdev_register(0, (int)port, vid, pid, UD_MARKED, 1, 0, 0);
            hw_disable_slot();
            return false;
        }
        if (role != USBDEV_ROLE_MOUSE) {
            /* [LOGS] ЭТАП 11: не HID? — а вдруг это ФЛЕШКА (MSC/BOT):
             * перечисляем накопитель на ЭТОМ слоте (bulk-пара EP) и
             * SCSI-старт — как OHCI-путь делает всегда.  Слот при успехе
             * переходит MSD насовсем (не disable!). */
            if (!kbd_ok && !mouse_ok &&
                xm_dev_probe(cfg, total, speed)) {
                if (xm_setup(port, speed, cfg9[5])) {
                    g_port_role[port] = PR_OTHER;
                    usbdev_register(0, (int)port, vid, pid, UD_MARKED, -1, 0, 0);
                    usbmap_pick(0x30, "mass-storage", (int)port,
                                xh_port_ver(port, speed) == 3 ? "3.0" : "2.0");
                    serial_puts("MOUSEHW: OHCI-порты не нужны: накопитель на xHCI готов\n");
                    return false;
                }
                /* SCSI не взлетел — слот чист как при любой неудаче */
                hw_disable_slot();
                return false;
            }
            /* ни клавиатура, ни мышь — или оба слота реестра заняты */
            g_port_role[port] = PR_OTHER;   /* [#460] */

            /* [#474] Регрессионный маркер: перечисление ВТОРОГО устройства
             * при живых слотах раньше ПОРТИЛО их конфигурацию скрэтчем
             * (g_mps/g_ms_xep/g_rpt_ok/g_absolute — «мышь видна, но не
             * работает»). Снимки g_lm/g_lk защищают живые устройства:
             * опрос/перевзвод/звонки читают ТОЛЬКО снимок. Строка видна в
             * журнале e2e (T15) и на реальном ПК. */
            if (g_lm.valid || g_lk.valid)
                serial_puts("MOUSEHW: [#474] второе устройство отклонено - "
                            "конфигурация ЖИВЫХ мыши/клавы НЕ тронута (снимок)\n");
            usbdev_register(0, (int)port, vid, pid, UD_MARKED, -1, 0, 0);
            hw_disable_slot();
            return false;
        }
        /* role == MOUSE: шаги 7-11 ниже доведут привязку мыши */
    }

    /* 7. SET_CONFIGURATION (если уже сделали ради LED-пробы - не повторяем) */
    if (!cfg_done) {
        if (!ctrl_xfer(0x00, USB_REQ_SET_CONFIG, cfg9[5], 0, 0, 0)) {
            xh_diag_set("SET_CONFIGURATION не прошёл");
            usbdev_release(USBDEV_ROLE_MOUSE, "SET_CONFIG не прошёл");
            hw_disable_slot();
            return false;
        }
    }

    /* 8. Режим отчётов (tablet/мышь) ДО конфигурации EP1.
     *    hw_detect_mode() читает report descriptor и заполняет g_log_max. */
    g_absolute = hw_detect_mode(cfg, total);

    /* 9. HID class requests (не фатальны при отказе). boot-протокол
     * просим у ЛЮБОЙ мыши, не только boot-subclass (мыши в report-протоколе
     * шлют отчёты не в boot-формате).  Отказ (STALL) игнорируем; планшетам
     * (g_absolute) boot-протокол только навредит - их не трогаем. */
    if (!g_absolute)
        (void)ctrl_xfer(0x21, USB_HID_SET_PROTOCOL, 0, g_iface, 0, 0);
    (void)ctrl_xfer(0x21, USB_HID_SET_IDLE, 0, g_iface, 0, 0);

    /* 10. Configure Endpoint: slot + interrupt-IN EP мыши (ADD = 0b1001)
     * [#424] НОМЕР EP — из дескриптора (g_ms_xep): композитные донглы дают
     * клаву на EP1, мышь на EP2. Раньше EP1 был захардкожен в ICC/контексте/
     * doorbell/фильтре событий — мышь с EP≠1 опрашивалась по чужому EP. */
    if (g_ms_xep == 0 || g_ms_xep > 15u) g_ms_xep = 1;   /* защита от мусора */
    memset(hw_inctx, 0, sizeof(hw_inctx));
    inctx_icc(ICC_SLOT | ICC_EP(g_ms_xep));
    inctx_slot(speed, port, 2u * (uint32_t)g_ms_xep + 1u);
    inctx_ep_in(g_ms_xep, g_mps);
    trb = ring_put(&r_cmd, vphys(hw_inctx), 0,
                   TRB_TYPE(TRB_CR_CONFIG_EP) | ((uint32_t)g_eslot << 24));
    db_command();
    if (!wait_cmd(trb, &cc, &slot)) {
        serial_puts("MOUSEHW: configure endpoint failed cc=");
        serial_puthex(cc);
        serial_puts(" ("); serial_puts(cc_name(cc)); serial_puts(")\n");
        g_xh_cc = cc;
        xh_diag_set("CONFIG EP cc=");
        xh_diag_add(cc_name(cc));
        usbdev_release(USBDEV_ROLE_MOUSE, "CONFIG EP не прошёл");
        hw_disable_slot();
        return false;
    }

    /* 11. Готово: ставим первый interrupt-IN TRB.
 * Configure Endpoint поставил контроллеру EP1 deque =
     * hw_ep1[0], DCS=1 - enqueue-состояние отчётного кольца обязано
     * совпасть (канон xhci-ring.c).  Баг «переподключил - мыши нет»
     * жил ровно здесь: hw_arm клал TRB на залежалую позицию r_ep1 с
     * залежалым cycle-битом, контроллер не видел НИ ОДНОГО отчёта. */
    for (int i = 0; i < 8; i++) hw_ep1[i].control = 0;
    r_ep1.idx = 0;
    r_ep1.cyc = 1;
    g_errors  = 0;
    g_packets = 0;
    mhw_slot_set(g_eslot); /* +слот ДО hw_arm, под штампом */
    g_present = true;
    g_active  = true;
    g_ms_xi   = g_xi;      /* 0.3.0: инстанс-владелец мыши */
    lm_save();             /* [#474] СНИМОК живой мыши: скрэтч перечисления
                            * больше не влияет на опрос/перевзвод/звонки */
    /* ЛИЧНЫЙ контекст мыши: копируем готовый выходной контекст из
     * перечислительного scratch (hw_devctx) в hw_mouse_devctx и
     * перенаправляем DCBAA.  hw_devctx снова свободен - следующее
     * перечисление (клава, hotplug) не портит живую мышь. */
    memset(hw_mouse_devctx, 0, sizeof(hw_mouse_devctx));
    memcpy(hw_mouse_devctx, hw_devctx, sizeof(hw_mouse_devctx));
    hw_dcbaa[g_slot] = (uint64_t)vphys(hw_mouse_devctx);
    /* мышь - запись в ТАБЛИЦЕ УСТРОЙСТВ */
    usbdev_register(0, (int)port, vid, pid, UD_MOUSE, 2, (int)g_slot, 1);
    usbmap_pick(0x30, "mouse", (int)port,
                xh_port_ver(port, speed) == 3 ? "3.0" : "2.0");   /* [#448] честная версия ПОРТА */
    hw_arm();

    serial_puts("MOUSEHW: ready vid=");
    serial_puthex(vid);
    serial_puts(" pid=");
    serial_puthex(pid);
    serial_puts(" port=");
    serial_puthex(port);
    serial_puts(" slot=");
    serial_puthex((uint32_t)g_slot);
    serial_puts(" iface=");
    serial_puthex(g_iface);
    serial_puts(" mps=");
    serial_puthex(g_mps);
    serial_puts(" mode=");
    serial_puts(g_absolute ? "absolute (tablet)" : "relative (boot mouse)");
    serial_puts("\n");
    xh_diag_set("МЫШЬ ГОТОВА: порт ");
    xh_diag_num(port);
    xh_diag_add(", слот ");
    xh_diag_num((uint32_t)g_slot);
    /* Составное устройство - каждый интерфейс получает
     * свой драйвер.  Мышь привязана; если в этом же корпусе есть
     * клавиатурный интерфейс - привязываем и его (мышь не страдает). */
    /* составное устройство: если слот клавиатуры ещё свободен —
     * подбираем «драйвер клавиатуры 3.0» и второму интерфейсу (реестр) */
    if (!usbdev_kbd_bound() && g_kbd_slot == 0 && hw_find_kbd(cfg, total)) {
        if (usbdev_try_bind(xh_port_ver(port, speed), 1, 0, xhci_bus_label(), (int)port, vid, pid)
                == USBDEV_ROLE_KBD) {
            if (hw_bind_kbd(cfg9[5])) {
                g_kbd_xport = port;
                usbdev_register(0, (int)port, vid, pid, UD_KBD, 1,
                                (int)g_kbd_slot, 1);
                hid_report_line(hid_port_speed(speed), port, (int)g_kbd_slot,
                                "keyboard", "composite", (int)g_kbd_xiface);
                serial_puts("MOUSEHW: составное устройство: клавиатура тоже привязана\n");
                hw_arm_mouse_again();
            } else {
                usbdev_release(USBDEV_ROLE_KBD, "composite: привязка не удалась");
            }
        }
    }
    return true;
}

/* ── BIOS/SMI hand-off через USB Legacy xECP (id 1); best-effort ──────────── */
static void hw_bios_handoff(void) {
    uint32_t hcc = mmr32(g_mmio + XHC_CAP_HCCPARAMS1);
    uint32_t ptr = (hcc >> 16) << 2;
    int guard = 32;
    while (ptr != 0 && guard-- > 0) {
        uint32_t hdr = mmr32(g_mmio + ptr);
        if ((hdr & 0xFFu) == 1u) {
            uint32_t base = g_mmio + ptr;
            serial_puts("MOUSEHW: USB Legacy xECP found, handoff\n");
            mmw32(base, hdr | (1u << 24));          /* OS Owned Semaphore */
            int t = 1500;
            while ((mmr32(base) & (1u << 16)) && t-- > 0) hw_delay(1000);
            if (t > 0) serial_puts("MOUSEHW: BIOS handoff done\n");
            else       serial_puts("MOUSEHW: BIOS handoff timeout, continue\n");
            /* F3 (спека xHCI §4.14.2): взяв владение (в т.ч. «силой»,
             * по таймауту), ОС обязана погасить SMI-источники BIOS в
             * USBLEGCTLSTS (xECP+4, второй dword капабилити): иначе SMM-
             * обработчик продолжает стрелять по контроллеру уже ПОСЛЕ
             * нашего HCRST (тот же класс бага, что был у OHCI — «СМАЗ»
             * чужих записей).  Старшие биты регистра — RW1C-статусы:
             * запись 0 по ним безвредна, снимаются только SMI-Enable биты. */
            mmw32(base + 4u, 0u);
            return;
        }
        ptr = ((hdr >> 8) & 0xFFu) << 2;   /* [#421] Next = dword0[15:8] (дворды→байты), НЕ [31:16]! */
    }
    serial_puts("MOUSEHW: no BIOS legacy xECP (clean)\n");
}

/* ── Останов и сброс контроллера: halt → HCRST → wait CNR ─────────────────── */
static bool hw_reset_hc(void) {
    if (!(mmr32(g_op + XHC_OP_USBSTS) & USBSTS_HCH)) {
        mmw32(g_op + XHC_OP_USBCMD,
              mmr32(g_op + XHC_OP_USBCMD) & ~USBCMD_RS);
        int t = 2000;
        while (!(mmr32(g_op + XHC_OP_USBSTS) & USBSTS_HCH) && t-- > 0)
            hw_delay(1000);
        if (!(mmr32(g_op + XHC_OP_USBSTS) & USBSTS_HCH)) return false;
    }
    mmw32(g_op + XHC_OP_USBCMD,
          mmr32(g_op + XHC_OP_USBCMD) | USBCMD_HCRST);
    int t = 5000;
    while (t-- > 0) {
        uint32_t c = mmr32(g_op + XHC_OP_USBCMD);
        uint32_t s = mmr32(g_op + XHC_OP_USBSTS);
        if (!(c & USBCMD_HCRST) && !(s & USBSTS_CNR)) return true;
        hw_delay(1000);
    }
    return false;
}

/* ── останов контроллера БЕЗ сброса: RS=0 → ждём HCH ────────────────────────
 * При неудаче перечисления контроллер ОБЯЗАН быть остановлен: работающий
 * xHCI продолжает DMA (кольца, scratchpad, контексты слотов), и на реальном
 * ПК бродячий DMA затирал кольцо журнала ядра (симптом «COREINFO: 1: 2»). */
static void xhci_halt_now(void) {
    mmw32(g_op + XHC_OP_USBCMD,
          mmr32(g_op + XHC_OP_USBCMD) & ~USBCMD_RS);
    int t = 5000;
    while (!(mmr32(g_op + XHC_OP_USBSTS) & USBSTS_HCH) && t-- > 0)
        hw_delay(1000);
    serial_puts("MOUSEHW: controller halted (HCH=");
    serial_puthex((mmr32(g_op + XHC_OP_USBSTS) & USBSTS_HCH) ? 1u : 0u);
    serial_puts(")\n");
}

/* ── PORTSC helpers ───────────────────────────────────────────────────────── */
static uint32_t port_addr(int port) {
    return g_op + XHC_OP_PORT0 + 16u * (uint32_t)(port - 1);
}

/* ── живая трассировка сброса порта (текстлог «test on errors») ─────────────
 * Данные с реального ПК (фото-прогона, мышь в синем гнезде):
 * порт 3 = raw=0x2E1 → CCS=1 (устройство ЕСТЬ), PED=0, PLS=7 (Polling —
 * USB3-линк тренируется и не завершается); порты 1/2/4 = 0x2A0 → пусто
 * (PLS=5 Rx.Detect).  Старый код молча ждал PED и уходил: между portsc-дампом
 * и «controller halted» на экране не было НИЧЕГО.  Теперь каждое изменение
 * raw печатается (до 16 строк), сброс двустадийный: PR (hot) → WPR (warm,
 * полный LFPS-пере-тренинг) → PR повторно. */
static void port_trace(int port, uint32_t raw, int *lines) {
    if (*lines >= 16) return;
    (*lines)++;
    serial_puts("MOUSEHW: p");
    serial_puthex((uint32_t)port);
    serial_puts(" raw=");
    serial_puthex(raw);
    serial_puts(" pls=");
    serial_puthex((raw >> 5) & 0xFu);
    serial_puts(" ccs=");
    serial_puthex(raw & 1u);
    serial_puts(" ped=");
    serial_puthex((raw >> 1) & 1u);
    serial_puts("\n");
}

/* Записать бит сброса (PR или WPR) и ждать PED, печатая изменения raw.
 * ms ≈ условные мс (hw_delay).  Возврат: raw с PED=1 либо 0 при таймауте. */
static uint32_t port_reset_trace(int port, uint32_t pa, bool warm, int ms,
                                 int *lines) {
    uint32_t v   = mmr32(pa);
    uint32_t bit = warm ? PORTSC_WPR : PORTSC_PR;
    serial_puts("MOUSEHW: p");
    serial_puthex((uint32_t)port);
    serial_puts(warm ? ": WARM reset (WPR), raw=" : ": port reset (PR), raw=");
    serial_puthex(v);
    serial_puts("\n");
    mmw32(pa, (v & 0xFFFFu) | bit | PORTSC_CHG);
    uint32_t last = mmr32(pa);
    port_trace(port, last, lines);
    while (ms-- > 0) {
        uint32_t cur = mmr32(pa);
        if (cur != last) {
            last = cur;
            port_trace(port, cur, lines);
        }
        if (cur & PORTSC_PED) return cur;
        hw_delay(1000);
    }
    return (last & PORTSC_PED) ? last : 0;
}

/* ── v1.9: экранная сводка PCI-скана (видна БЕЗ журнала и COM1) ────────────
 * Строка вида «USB PCI: EHCI 0:1d.0, xHCI 0:14.0» показывается в
 * приветственном диалоге (desktop.c, 5-я строка) сразу после загрузки.
 * На реальном ПК без COM1 это самый быстрый ответ на вопрос «почему мышь
 * не нашлась»: в списке только EHCI (0x20) → мышь сидит на EHCI и нужен
 * EHCI-драйвер; «USB PCI: контроллеров нет» → BIOS скрыл USB либо скан
 * не видит шину; есть xHCI → смотреть журнал (F12), на какой cc-стадии
 * остановилось перечисление. */
static char g_usb_diag[72];

static const char *usb_if_name(uint8_t pg) {
    switch (pg) {
        case 0x00u: return "UHCI";
        case 0x10u: return "OHCI";
        case 0x20u: return "EHCI";
        case 0x30u: case 0x31u: return "xHCI";
        default:    return "USB";
    }
}

/* Дописать b:d.f компактно (ведущие нули отбрасываются): "0:1d.0". */
static void diag_hex8(char *dst, int *p, uint8_t v) {
    const char *h = "0123456789ABCDEF";
    if ((v >> 4) != 0) dst[(*p)++] = h[v >> 4];
    dst[(*p)++] = h[v & 0xFu];
}

static void usb_diag_build(const uint8_t *bus, const uint8_t *dev,
                           const uint8_t *fn, const uint8_t *pg, int n) {
    int p = 0;
    const char *s = "USB PCI: ";
    while (*s) g_usb_diag[p++] = *s++;
    if (n == 0) {
        s = "контроллеров нет";
        while (*s) g_usb_diag[p++] = *s++;
    }
    for (int i = 0; i < n; i++) {
        if (i == 2) {                       /* показываем первые два + счётчик */
            const char *more = " +";
            while (*more) g_usb_diag[p++] = *more++;
            diag_hex8(g_usb_diag, &p, (uint8_t)(n - 2));
            break;
        }
        if (i) { g_usb_diag[p++] = ','; g_usb_diag[p++] = ' '; }
        const char *nm = usb_if_name(pg[i]);
        while (*nm) g_usb_diag[p++] = *nm++;
        if (p < (int)sizeof(g_usb_diag) - 12) {
            g_usb_diag[p++] = ' ';
            diag_hex8(g_usb_diag, &p, bus[i]);
            g_usb_diag[p++] = ':';
            diag_hex8(g_usb_diag, &p, dev[i]);
            g_usb_diag[p++] = '.';
            diag_hex8(g_usb_diag, &p, fn[i]);
        }
    }
    g_usb_diag[p] = 0;
}

const char *mouse_hw_usb_diag(void) { return g_usb_diag; }

/* ── forward — зондирование одного xHCI (определение ниже) ──────────── */
static int xhci_try_port(int p);   /* hotplug использует */
static int probe_xhci(int xi, uint8_t xbus, uint8_t xdev, uint8_t xfn,
                      uint16_t xvid, uint16_t xdid);

/* ── Инициализация: PCI → MMIO → reset → структуры → порты → перечисление ── */
/* ═══════════════════════════════════════════════════════════════════════════
 * OHCI — USB 1.x порты (AMD FCH 0:12.0 / 0:13.0). Полный мини-драйвер:
 * сброс контроллера, root hub, control-передачи через ControlList,
 * interrupt-IN ED в периодическом списке HCCA, hotplug.
 * Completion БЕЗ прерываний: у финального TD DI=0 → контроллер пишет
 * DoneHead в HCCA (событие WDH), мы поллим ПАМЯТЬ. IRQ к PIC не подключены.
 *
 * Схема очередей: ED.HeadP = первый ожидающий TD, ED.TailP = 0 (конец);
 * после выхода TD контроллер сам пишет HeadP = TD.NextTD = 0 → ED пуст.
 * Повторная постановка = переписать HeadP (carry-бит сохранить).
 * ═══════════════════════════════════════════════════════════════════════ */
#define OH_CTRL       0x04u
#define OH_CTRL_CUR   0x24u   /* HcControlCurrentED */
#define OH_CMDSTS     0x08u
#define OH_INTSTS     0x0Cu
#define OH_INTEN      0x10u
#define OH_HCCA       0x18u
#define OH_CTRL_HEAD  0x20u
#define OH_FMINT      0x34u
#define OH_PERSTART   0x40u
#define OH_LSTH       0x44u               /* HcLSThreshold - HCR НЕ сбрасывает! */
#define OH_RHDESCA    0x48u
#define OH_RHSTATUS   0x50u
#define OH_FMNUM      0x3Cu               /* HcFmNumber */
#define OH_DONEHEAD   0x30u               /* HcDoneHead */
#define OH_RHPORT     0x54u               /* + 4*(n-1) */

#define OH_CTRL_PLE   (1u << 2)
#define OH_CTRL_CLE   (1u << 4)
#define OH_CTRL_BLE   (1u << 5)
#define OH_CTRL_OPER  (2u << 6)
#define OH_CMD_HCR    (1u << 0)
#define OH_CMD_CLF    (1u << 1)
#define OH_INT_WDH    (1u << 1)
#define OH_INT_MIE    (1u << 31)   /* Master Interrupt Enable = бит 31 */
#define OH_RH_SETGLOB (1u << 16)

/* ED (16 байт) */
#define ED_FA(f)      ((uint32_t)(f) & 0x7Fu)
#define ED_EN(e)      (((uint32_t)(e) & 0xFu) << 7)
#define ED_IN         (2u << 11)
#define ED_LS         (1u << 13)
#define ED_SKIP       (1u << 14)
#define ED_MPS(m)     (((uint32_t)(m) & 0x7Fu) << 16)
/* TD (16 байт, свёрено с Linux ohci.h) */
#define TD_DP_SETUP   (0u << 19)
#define TD_DP_OUT     (1u << 19)
#define TD_DP_IN      (2u << 19)
#define TD_DI_NOINT   (7u << 21)
#define TD_DI_NOW     (0u << 21)
/* по спеке OHCI T[25:24]: 00=carry от ED, 01=DATA0, 10=DATA1,
 * 11=reserved (Linux ohci.h: TD_T_DATA0=0x01000000, TD_T_DATA1=0x02000000).
 * Было 10/11 - SETUP уходил с DATA1, реальное устройство молчало
 * (cc=5 DeviceNotResponding, фото/); QEMU прощал. ИСПРАВЛЕНО. */
#define TD_T_DATA0    (1u << 24)
#define TD_T_DATA1    (2u << 24)
#define TD_T_CARRY    (0u << 24)
#define TD_ROUND      (1u << 18)
#define TD_CC_GET(v)  (((v) >> 28) & 0xFu)
#define TD_CC_OK      0x0u

static void oh_w32(uint32_t off, uint32_t v) { mmw32(g_oh_mmio + off, v); }
static uint32_t oh_r32(uint32_t off)         { return mmr32(g_oh_mmio + off); }
static uint32_t oh_donehead(void) { return *(volatile uint32_t *)(oh_hcca + 0x84); }
static void oh_ack_wdh(void)      { oh_w32(OH_INTSTS, OH_INT_WDH); }
static uint32_t oh_port(int p)    { return OH_RHPORT + 4u * (uint32_t)(p - 1); }
static bool oh_port_claimed(int q);   /* forward - нужна СНИМКу */

/* чтение памяти, которую правит HC через DMA (CC/CBP/HeadP/буферы).
 * Обычное чтение компилятор кэширует/поднимает из цикла ожидания (inb
 * без memory-clobber) - CC «замирал» 0xF и УСПЕШНАЯ передача читалась
 * как таймаут (QEMU-лог ctrl cc=0xF при status_td с CC=0). */
static uint32_t oh_dma32(const uint32_t *p) {
    return *(const volatile uint32_t *)p;
}

/* ═══ имена CC-кодов OHCI (TD word0[31:28]) для текстлога ════════════ */
static const char *oh_cc_name(uint32_t cc) {
    switch (cc) {
        case 0x0: return "NoError";
        case 0x1: return "CRC";
        case 0x2: return "BitStuff";
        case 0x3: return "DataToggle";
        case 0x4: return "Stall";
        case 0x5: return "DeviceNotResponding";
        case 0x6: return "PIDCheckFailure";
        case 0x7: return "UnexpectedPID";
        case 0x8: return "DataOverrun";
        case 0x9: return "DataUnderrun";
        default:  return "Reserved";
    }
}

/* ═══ USB Mass Storage (BOT + SCSI) на OHCI — sticklog ════════════════
 * Флешка (раздел LOGS) — выгрузка журнала ядра на реальном ПК.  Класс
 * 0x08/0x06/0x50, bulk-пара EP.  Data-toggle ведёт КОНТРОЛЛЕР: TD с
 * T=CARRY берёт состояние из ED (HC сам обновляет C-бит при writeback),
 * свежая пара после SET_CONFIG начинает с DATA0 — как того ждёт BOT.
 * Фазы CBW -> DATA -> CSW ставятся ПО ОЧЕРЕДИ на ED своего направления,
 * между фазами ED уходит в Skip (схема ожидания та же, что у oh_ctrl:
 * WDH ИЛИ смена CC, interrupt-ED на время в Skip). */
#define OH_BULK_HEAD  0x28u          /* HcBulkHeadED                        */
#define OH_CMD_BLF    (1u << 2)      /* HcCommandStatus.BulkListFilled      */
#define ED_OUT        (1u << 11)     /* направление ED: OUT                 */

static uint32_t oh_td_blk_a[OHCI_NINST][9][4] __attribute__((aligned(16)));
static uint32_t oh_ed_blk_o_a[OHCI_NINST][4] __attribute__((aligned(16)));
static uint32_t oh_ed_blk_i_a[OHCI_NINST][4] __attribute__((aligned(16)));
#define oh_td_blk  (oh_td_blk_a[g_oi])
#define oh_ed_blk_o (oh_ed_blk_o_a[g_oi])
#define oh_ed_blk_i (oh_ed_blk_i_a[g_oi])
static uint8_t  oh_cbw[32]      __attribute__((aligned(4)));  /* 31 байт    */
static uint8_t  oh_csw[16]      __attribute__((aligned(4)));  /* 13 байт    */
static uint8_t  oh_blkbuf[512]  __attribute__((aligned(512)));  /* данные (TD не пересекает 4К) */
static int      g_oh_msd;        /* флешка перечислена и готова           */
static uint8_t  g_oh_msd_addr;   /* ЕЁ USB-адрес (g_oh_addr уплывает)     */
static uint8_t  g_oh_msd_in;     /* номер bulk-IN EP                      */
static uint8_t  g_oh_msd_out;    /* номер bulk-OUT EP                     */
static uint8_t  g_oh_msd_mps;    /* 64 (full-speed bulk)                  */
static uint32_t g_oh_msd_sect;   /* объём в LBA512                        */
static char     g_oh_msd_name[28];
static uint32_t g_oh_msd_tag;
static int      g_oh_msd_ctl;    /* 1 = oh_ctrl адресует ФЛЕШКУ           */
static int      g_oh_msd_port;   /* ЕЁ порт (не тушить, пока жива) */

/* последний значимый шаг OHCI-поиска флешки. Печатается в ИТОГЕ
 * sticklog (kernel.c), чтобы по экрану сразу видеть, ГДЕ именно стоп:
 * контроллер не найден / порты пусты / set address не прошёл / дескрипторы
 * нулевые / INQUIRY не прошла (ядро вообще не дошло до записи -
 * спотыкается где-то до WRITE(10)). */
static char g_oh_diag[128] = "скан ещё не выполнялся";
static void oh_diag_set(const char *s) {
    int i = 0;
    for (; s[i] && i < (int)sizeof(g_oh_diag) - 1; i++) g_oh_diag[i] = s[i];
    g_oh_diag[i] = 0;
}
static void oh_diag_set2(const char *a, const char *b) {
    oh_diag_set(a);
    int i = 0;
    while (g_oh_diag[i]) i++;
    for (int j = 0; b[j] && i < (int)sizeof(g_oh_diag) - 1; j++)
        g_oh_diag[i++] = b[j];
    g_oh_diag[i] = 0;
}

/* Control-передача через control-список. Возвращает CC финального TD
 * (0 = успех, 0xFF = таймаут).
 * -фиксы против реального железа (фото: set address failed, затем
 * vid=0 pid=0 на хотплаге):
 *  1) HccaDoneHead никогда не обнулялся → ВСЕ передачи после первой
 *     «проходили» мгновенно с фейковым CC=0 (дескриптор из нулей).
 *     Теперь: ack WDH + обнуление DoneHead ДО постановки, ожидание по биту
 *     WDH в HcInterruptStatus, очистка после; ED уходит в Skip сразу. */
static int oh_ctrl(uint8_t bm, uint8_t req, uint16_t wv, uint16_t wi,
                   uint16_t len, uint8_t *data) {
    if (g_oh_mmio == 0) return 0xFF;
    oh_w32(OH_CTRL_CUR, 0);   /* сброс CurrentED: HC не застрянет на старом ED */
    oh_setup[0] = bm; oh_setup[1] = req;
    oh_setup[2] = (uint8_t)wv;  oh_setup[3] = (uint8_t)(wv >> 8);
    oh_setup[4] = (uint8_t)wi;  oh_setup[5] = (uint8_t)(wi >> 8);
    oh_setup[6] = (uint8_t)len; oh_setup[7] = (uint8_t)(len >> 8);

    /* Живые interrupt-ED (мышь И клавиатура, ) на время control-
     * передачи ставим в Skip: их ретайр мог бы дать ложный WDH. */
    int unskip_m = 0, unskip_k = 0;
    if (g_oh_active && !(oh_ed_intr[0] & ED_SKIP)) {
        oh_ed_intr[0] |= ED_SKIP;
        unskip_m = 1;
    }
    if (g_oh_kbd && !(oh_ed_kbd[0] & ED_SKIP)) {
        oh_ed_kbd[0] |= ED_SKIP;
        unskip_k = 1;
    }
    dma_sync();                           /* SKIP дойдёт до HC */
    if (unskip_m || unskip_k) hw_delay(2000);

    oh_ack_wdh();
    *(volatile uint32_t *)(oh_hcca + 0x84) = 0;
    dma_sync();                           /* чистый DoneHead в ОЗУ */

    /* SETUP: DATA0 */
    oh_td[0][0] = (0xFu << 28) | TD_DP_SETUP | TD_DI_NOINT | TD_T_DATA0; /* CC=NotAccessed */
    oh_td[0][1] = vphys(oh_setup);
    /* без data-стадии (SET_ADDRESS и т.п.) SETUP идёт СРАЗУ на
     * STATUS - раньше в цепи оставался TD[1] от ПРОШЛОЙ передачи
     * (мусор с чужим CC/buffer) и HC исполнял его как лишнюю стадию. */
    oh_td[0][2] = (len && data) ? vphys(oh_td[1]) : vphys(oh_td[2]);
    oh_td[0][3] = vphys(oh_setup) + 7;
    /* DATA (если есть): DATA1, короткий пакет разрешён */
    if (len && data) {
        int dir = (bm & 0x80u) ? TD_DP_IN : TD_DP_OUT;
        if (dir == TD_DP_IN) memset(data, 0, len);
        oh_td[1][0] = (0xFu << 28) | dir | TD_DI_NOINT | TD_T_DATA1 | TD_ROUND; /* CC */
        oh_td[1][1] = vphys(data);
        oh_td[1][2] = vphys(oh_td[2]);
        oh_td[1][3] = vphys(data) + (uint32_t)len - 1u;
    }
    /* STATUS: направление противоположное data, DI=0 → writeback */
    int sdir = (bm & 0x80u) ? TD_DP_OUT : TD_DP_IN;
    oh_td[2][0] = (0xFu << 28) | sdir | TD_DI_NOW | TD_T_DATA1; /* CC=NotAccessed */
    oh_td[2][1] = 0;
    oh_td[2][2] = vphys(oh_td_tail);      /* NextTD -> dummy-хвост */
    oh_td[2][3] = 0;

        oh_ed_ctrl[0] = ED_FA(g_oh_msd_ctl ? g_oh_msd_addr : g_oh_addr) | (g_oh_ls ? ED_LS : 0u) | ED_MPS(g_oh_ep0_mps);
    oh_ed_ctrl[1] = vphys(oh_td_tail);    /* TailP = dummy-TD (спека) */
    oh_ed_ctrl[2] = vphys(oh_td[0]);      /* HeadP = первый TD (H=0)       */
    oh_ed_ctrl[3] = 0;
    oh_ed_ctrl[0] &= ~ED_SKIP;
    /* пред-полётная проверка control-ED: слово обязано быть ровно
 * тем, что написано строкой выше (FA|LS|MPS). Фото EDc word0 =
     * 0x1000 (FS, D=IN, MPS=0) - НЕ то, что пишет эта функция.  Расхождение
     * = кто-то пишет в нашу память МЕЖДУ строками. */
    if (g_oh_ls && !(oh_ed_ctrl[0] & ED_LS))
        serial_puts("MOUSEHW: СМАЗ: control ED word0 потерял LS-бит!\n");

    dma_sync();   /* ED/TD/SETUP-буфер — в ОЗУ ДО включения списка */
    oh_w32(OH_CTRL_HEAD, vphys(oh_ed_ctrl));
    oh_w32(OH_CMDSTS, OH_CMD_CLF);        /* список control заполнен       */

    /* ожидание ПО СОСТОЯНИЮ TD свежими чтениями (oh_dma32), а не по
     * WDH: WDH простреливает раньше writeback, а кэш компилятора держал
     * CC=0xF - успешная передача читалась как сбой.  Выходы: 1) STATUS-CC
     * изменился (передача закрыта, вердикт по нему); 2) SETUP/DATA
 * ошиблись; 3) ED Halted = БИТ 0 HeadP (бит 1 - это Carry, 
     * проверял его и «спотыкался» на удачном SETUP); 4) HeadP дошёл до
     * dummy-хвоста.  Затем settle и ФИНАЛЬНЫЙ вердикт по свежим CC. */
    int timeout = 300;
    int cc = 0xFF;
    while (timeout-- > 0) {
        dma_sync();   /* CC пишет HC (DMA) — инвалидируем кэш */
        uint32_t cc0 = TD_CC_GET(oh_dma32(&oh_td[0][0]));
        uint32_t cc2 = TD_CC_GET(oh_dma32(&oh_td[2][0]));
        if (cc2 != 0xF) { cc = (int)cc2; break; }
        if (cc0 != 0xF && cc0 != TD_CC_OK) { cc = (int)cc0; break; }
        if (oh_dma32(&oh_ed_ctrl[2]) & 1u) break;      /* Halted - бит 0 */
        if ((oh_dma32(&oh_ed_ctrl[2]) & ~3u) == vphys(oh_td_tail)) break;
        hw_delay(1000);
    }
    oh_ed_ctrl[0] |= ED_SKIP;             /* снять очередь немедленно       */
    hw_delay(2000);                       /* writeback поспевает */

    dma_sync();   /* writeback HC уже в ОЗУ — кэш сбросить */
    {
        int cc0f = (int)TD_CC_GET(oh_dma32(&oh_td[0][0]));
        int cc1f = (len && data) ? (int)TD_CC_GET(oh_dma32(&oh_td[1][0])) : 0xF;
        int cc2f = (int)TD_CC_GET(oh_dma32(&oh_td[2][0]));
        if (cc2f != 0xF) cc = cc2f;                 /* STATUS исполнен       */
        else if (cc0f != 0xF && cc0f != TD_CC_OK) cc = cc0f;
        else if (cc1f != 0xF && cc1f != TD_CC_OK) cc = cc1f;
        else if (cc == 0xFF && (oh_r32(OH_INTSTS) & OH_INT_WDH)) cc = cc2f;
    }
    if (cc == 0xFF) {
        /* таймаут - печатаем ВСЁ состояние HC, чтобы по текстлогу
         * сразу видеть: крутится ли кадровый движок, взят ли список,
         * дошли ли хотя бы до первого TD (cc0 != 0xF). */
        serial_puts("MOUSEHW: OHCI ctrl TIMEOUT dh=");
        serial_puthex(oh_donehead() & ~1u);
        serial_puts(" ed_halted=");
        serial_puthex(oh_dma32(&oh_ed_ctrl[2]) & 1u);   /* бит 0! */
        serial_puts(" ctrl=");
        serial_puthex(oh_r32(OH_CTRL));
        serial_puts(" cmdsts=");
        serial_puthex(oh_r32(OH_CMDSTS));
        serial_puts(" ist=");
        serial_puthex(oh_r32(OH_INTSTS));
        serial_puts(" headp=");
        serial_puthex(oh_dma32(&oh_ed_ctrl[2]));
        serial_puts(" cc0=");
        serial_puthex(TD_CC_GET(oh_dma32(&oh_td[0][0])));
        serial_puts(" cc1=");
        serial_puthex(TD_CC_GET(oh_dma32(&oh_td[1][0])));
        serial_puts(" cc2=");
        serial_puthex(TD_CC_GET(oh_dma32(&oh_td[2][0])));
        serial_puts("\n");
    }
    oh_ack_wdh();                                   /* чистый выход */
    *(volatile uint32_t *)(oh_hcca + 0x84) = 0;     /* DoneHead = 0 */

    if (cc != TD_CC_OK && cc != 0xFF) {
        serial_puts("MOUSEHW: OHCI ctrl cc=");
        serial_puthex((uint32_t)cc);
        serial_puts(" (");
        serial_puts(oh_cc_name((uint32_t)cc));
        serial_puts(") status_td=");
        serial_puthex(oh_td[2][0]);
        serial_puts("\n");
    }

    if (unskip_m) oh_ed_intr[0] &= ~ED_SKIP;   /* вернуть прерыватели     */
    if (unskip_k) oh_ed_kbd[0]  &= ~ED_SKIP;
    dma_sync();                           /* ED снова в ОЗУ */
    return cc;
}
/* ═══ BOT+SCSI поверх bulk-ED (только флешка sticklog) ════════════════ */

/* Сбросить HALT на обоих bulk-EP и тогглы в DATA0 (после Stall/сбоя). */
static void oh_msd_recover(void) {
    (void)oh_ctrl(0x02, 0x01, 0, g_oh_msd_out, 0, 0);
    (void)oh_ctrl(0x02, 0x01, 0, (uint16_t)(0x80u | g_oh_msd_in), 0, 0);
    oh_ed_blk_o[2] = vphys(oh_td_blk[0]) & ~3u;   /* H=0, C=0 (DATA0)      */
    oh_ed_blk_i[2] = vphys(oh_td_blk[0]) & ~3u;
}

/* Поставить цепочку TD на bulk-ED и дождаться конца.  buf — ОДИН
 * непрерывный static-буфер (<=512), in — направление данных.
 * Возвращает CC последнего TD (0 = успех, 0xFF = таймаут). */
static int oh_bulk_run(uint32_t *ed, uint8_t ep, int dir_in,
                       uint8_t *buf, uint32_t len, int timeout_ms) {
    if (g_oh_mmio == 0 || len == 0 || len > 512) return 0xFF;
    uint32_t mps = g_oh_msd_mps ? g_oh_msd_mps : 64u;
    int ntd = (int)((len + mps - 1u) / mps);
    if (ntd > 8) return 0xFF;

    /* interrupt-ED (мышь/клава) на время фазы в Skip — как в oh_ctrl */
    int unskip_m = 0, unskip_k = 0;
    if (g_oh_active && !(oh_ed_intr[0] & ED_SKIP)) {
        oh_ed_intr[0] |= ED_SKIP;
        unskip_m = 1;
    }
    if (g_oh_kbd && !(oh_ed_kbd[0] & ED_SKIP)) {
        oh_ed_kbd[0] |= ED_SKIP;
        unskip_k = 1;
    }
    dma_sync();                           /* SKIP дойдёт до HC */
    if (unskip_m || unskip_k) hw_delay(2000);

    oh_ack_wdh();
    *(volatile uint32_t *)(oh_hcca + 0x84) = 0;
    dma_sync();                           /* чистый DoneHead в ОЗУ */

    for (int i = 0; i < ntd; i++) {
        uint32_t off   = (uint32_t)i * mps;
        uint32_t chunk = len - off;
        if (chunk > mps) chunk = mps;
        int last = (i == ntd - 1);
        oh_td_blk[i][0] = (0xFu << 28)                       /* CC=NotAccessed */
            | (dir_in ? TD_DP_IN : TD_DP_OUT)
            | (last ? TD_DI_NOW : TD_DI_NOINT)
            | TD_T_CARRY                                     /* тоггл от HC    */
            | (dir_in ? TD_ROUND : 0u);
        oh_td_blk[i][1] = vphys(buf) + off;
        oh_td_blk[i][2] = last ? vphys(oh_td_tail) : vphys(oh_td_blk[i + 1]);
        oh_td_blk[i][3] = vphys(buf) + off + chunk - 1u;
    }

    ed[0] = ED_FA(g_oh_msd_addr) | (dir_in ? ED_IN : ED_OUT)
          | ED_EN(ep) | ED_MPS((int)mps);
    ed[1] = vphys(oh_td_tail);             /* TailP = dummy (спека, ) */
    ed[2] = vphys(oh_td_blk[0]) | (ed[2] & 2u);   /* Carry храним, Halted снимаем */
    ed[3] = 0;
    ed[0] &= ~ED_SKIP;

    dma_sync();   /* ED/TD/CBW-буфер — в ОЗУ ДО включения списка */
    oh_w32(OH_BULK_HEAD, vphys(ed));
    oh_w32(OH_CMDSTS, oh_r32(OH_CMDSTS) | OH_CMD_BLF);

    /* как в oh_ctrl - состояние TD свежими чтениями, Halted = бит 0
 * (проверял бит 1 = Carry: на УСПЕШНОМ CBW ED.C переворачивался,
     * цикл выходил «по halted» до writeback -> «bulk TIMEOUT cc_last=0xF»). */
    int cc = 0xFF;
    while (timeout_ms-- > 0) {
        dma_sync();   /* CC пишет HC (DMA) — инвалидируем кэш */
        uint32_t ccl = TD_CC_GET(oh_dma32(&oh_td_blk[ntd - 1][0]));
        if (ccl != 0xF) { cc = (int)ccl; break; }
        if (oh_dma32(&ed[2]) & 1u) break;            /* Halted - бит 0 */
        hw_delay(1000);
    }
    ed[0] |= ED_SKIP;                      /* снять очередь немедленно       */
    hw_delay(2000);                        /* writeback поспевает */
    dma_sync();   /* финальные CC читаем из ОЗУ, не из кэша */
    cc = (int)TD_CC_GET(oh_dma32(&oh_td_blk[ntd - 1][0]));
    if (cc == 0xF) {                       /* последний TD не дошёл:         */
        cc = 0xFF;                         /* ищем первую ошибку в цепи      */
        for (int i = 0; i < ntd; i++) {
            uint32_t c = TD_CC_GET(oh_dma32(&oh_td_blk[i][0]));
            if (c != 0xF && c != TD_CC_OK) { cc = (int)c; break; }
        }
    }
    if (cc == 0xFF) {
        serial_puts("MOUSEHW: OHCI bulk TIMEOUT len=");
        serial_puthex(len);
        serial_puts(" cc_last=");
        serial_puthex(TD_CC_GET(oh_dma32(&oh_td_blk[ntd - 1][0])));
        serial_puts(" halted=");
        serial_puthex(oh_dma32(&ed[2]) & 1u);
        serial_puts(" ist=");
        serial_puthex(oh_r32(OH_INTSTS));
        serial_puts("\n");
    }
    if (unskip_m) oh_ed_intr[0] &= ~ED_SKIP;
    if (unskip_k) oh_ed_kbd[0]  &= ~ED_SKIP;
    dma_sync();                           /* ED снова в ОЗУ */
    oh_ack_wdh();
    *(volatile uint32_t *)(oh_hcca + 0x84) = 0;

    if (cc != TD_CC_OK && cc != 0xFF) {
        serial_puts("MOUSEHW: OHCI bulk cc=");
        serial_puthex((uint32_t)cc);
        serial_puts(" (");
        serial_puts(oh_cc_name((uint32_t)cc));
        serial_puts(")\n");
    }
    return cc;
}

typedef struct __attribute__((packed)) {
    uint32_t sig, tag, dlen;
    uint8_t  flags, lun, cbl;
    uint8_t  cb[16];
} oh_cbw_t;

typedef struct __attribute__((packed)) {
    uint32_t sig, tag, resid;
    uint8_t  status;
} oh_csw_t;

#define OH_CSW_SIG 0x53425355u

static bool oh_msd_bot(const uint8_t *cdb, int cbl, int data_in,
                       uint8_t *data, uint32_t dlen) {
    oh_cbw_t *c = (oh_cbw_t *)oh_cbw;
    memset(oh_cbw, 0, sizeof(oh_cbw));
    c->sig   = 0x43425355u;
    c->tag   = ++g_oh_msd_tag;
    c->dlen  = dlen;
    c->flags = data_in ? 0x80u : 0x00u;
    c->lun   = 0;
    c->cbl   = (uint8_t)cbl;
    memcpy(c->cb, cdb, (uint32_t)cbl);

    /* CBW 1000 мс (было 500) - на реальной флешке после SET_CONFIG
     * первая команда может подлагивать. */
    if (oh_bulk_run(oh_ed_blk_o, g_oh_msd_out, 0, oh_cbw, 31, 1000)
        != TD_CC_OK) {
        oh_msd_recover();
        return false;
    }
    if (dlen) {
        /* DATA 10000 мс (было 5000) - WRITE(10) на медленной флешке. */
        int cc = oh_bulk_run(data_in ? oh_ed_blk_i : oh_ed_blk_o,
                             data_in ? g_oh_msd_in : g_oh_msd_out,
                             data_in, data, dlen, 10000);
        if (cc != TD_CC_OK) {
            oh_msd_recover();
            return false;
        }
    }
    memset(oh_csw, 0, sizeof(oh_csw));
    /* CSW 2000 мс (было 500) - флешка думает после DATA-фазы. */
    if (oh_bulk_run(oh_ed_blk_i, g_oh_msd_in, 1, oh_csw, 13, 2000)
        != TD_CC_OK) {
        oh_msd_recover();
        return false;
    }
        oh_csw_t *s = (oh_csw_t *)oh_csw;
        /* 1) Транспортная ошибка: сигнатура/тег не совпали → полный BOT-reset */
        if (s->sig != OH_CSW_SIG || s->tag != g_oh_msd_tag) {
                serial_puts("MOUSEHW: OHCI MSD CSW bad: sig=");
                serial_puthex(s->sig);
                serial_puts(" tag=");
                serial_puthex(s->tag);
                serial_puts(" st=");
                serial_puthex(s->status);
                serial_puts("\n");
                oh_msd_recover();
                return false;
        }
        /* 2) status=1 = "команда отклонена устройством" (например, TUR на
         *    ещё-не-готовой флешке после сброса).  Это НЕ транспортный сбой,
         *    CLEAR_FEATURE не нужен — просто сообщаем вызывающему, что комманда
         *    не прошла, и цикл retry в oh_msd_init попробует ещё раз. */
        if (s->status != 0) {
                return false;
        }
        return true;
}

static void oh_msd_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static bool oh_msd_inquiry(void) {
    uint8_t cdb[6] = { 0x12, 0, 0, 0, 36, 0 };
    memset(oh_blkbuf, 0, sizeof(oh_blkbuf));
    if (!oh_msd_bot(cdb, 6, 1, oh_blkbuf, 36)) return false;
    for (int i = 0; i < 8; i++)
        g_oh_msd_name[i] =
            (char)(oh_blkbuf[8 + i] >= 0x20 ? oh_blkbuf[8 + i] : ' ');
    g_oh_msd_name[8] = ' ';
    for (int i = 0; i < 16; i++)
        g_oh_msd_name[9 + i] =
            (char)(oh_blkbuf[16 + i] >= 0x20 ? oh_blkbuf[16 + i] : ' ');
    g_oh_msd_name[25] = 0;
    return true;
}

static bool oh_msd_ready(void) {           /* TEST UNIT READY                */
    uint8_t cdb[6] = { 0, 0, 0, 0, 0, 0 };
    return oh_msd_bot(cdb, 6, 0, 0, 0);
}

static bool oh_msd_capacity(void) {        /* READ CAPACITY(10)              */
    uint8_t cdb[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    memset(oh_blkbuf, 0, sizeof(oh_blkbuf));
    if (!oh_msd_bot(cdb, 10, 1, oh_blkbuf, 8)) return false;
    g_oh_msd_sect = ((uint32_t)oh_blkbuf[0] << 24)
                  | ((uint32_t)oh_blkbuf[1] << 16)
                  | ((uint32_t)oh_blkbuf[2] << 8)
                  |  (uint32_t)oh_blkbuf[3];
    g_oh_msd_sect += 1;
    return true;
}

bool mouse_hw_msd_read(uint32_t lba, void *buf512) {
    if (!g_oh_msd) return false;
    /* «ЭТАП 3»: g_oi после init-циклов застревает на ПОСЛЕДНЕМ инстансе —
     * флешка на OHCI#1 при g_oi=1 читала регистры ЧУЖОГО контроллера. */
    if (g_oh_msd_oi >= 0 && g_oh_msd_oi <= 1) g_oi = g_oh_msd_oi;
    uint8_t cdb[10] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0 };
    oh_msd_be32(cdb + 2, lba);
    g_oh_msd_ctl = 1;
    bool ok = oh_msd_bot(cdb, 10, 1, (uint8_t *)buf512, 512);
    g_oh_msd_ctl = 0;
    return ok;
}

bool mouse_hw_msd_write(uint32_t lba, const void *buf512) {
    if (!g_oh_msd) return false;
    /* «ЭТАП 3»: см. mouse_hw_msd_read — селектор на инстанс флешки. */
    if (g_oh_msd_oi >= 0 && g_oh_msd_oi <= 1) g_oi = g_oh_msd_oi;
    uint8_t cdb[10] = { 0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0 };
    oh_msd_be32(cdb + 2, lba);
    g_oh_msd_ctl = 1;
    bool ok = oh_msd_bot(cdb, 10, 0, (uint8_t *)buf512, 512);
    g_oh_msd_ctl = 0;
    return ok;
}

/* однострочная OHCI-сводка для sticklog-ИТОГА (kernel.c). */
void mouse_hw_ohci_diag(char *out, int outsz) {
    if (!out || outsz <= 0) return;
    int i = 0;
    for (; g_oh_diag[i] && i < outsz - 1; i++) out[i] = g_oh_diag[i];
    out[i] = 0;
}

/* ═══ «OHCI СНИМОК» - конечный блок диагностики для ФОТО ══════════════
 * kernel.c зовёт его ПОСЛЕ строки «ИТОГ: НЕ ЗАПИСАНО» - блок оказывается
 * в самом низу экрана sticklog.  Одно фото = вся картина сразу:
 *   HC   - живые HcControl/HcCommandStatus/HcInterruptStatus + расшифровка
 *          (HCFS-состояние, CLE/BLE/PLE, CLF/BLF, WDH/SF);
 *   FM   - тикает ли кадровый движок (два чтения HcFmNumber через 2 мс;
 *          не тикает = HC стоит, control-список не обслуживается ВООБЩЕ);
 *   REG  - HcHCCA/HcControlHeadED против ОЖИДАЕМЫХ vphys (несовпадение =
 *          контроллер смотрит не на наши ED); HcBulkHeadED, HcDoneHead
 *          (регистр) и HccaDoneHead (память) - если DoneHead != 0, HC
 *          ИСПОЛНЯЛ TD и проблема в разборе done-списка, а не в списках;
 *   EDc  - сырые 4 слова control-ED (Halted=бит0 HeadP, Carry=бит1,
 *          выравнивание адресов видно сразу);
 *   cc   - CC всех трёх TD control-цепи: 0xF = контроллер TD НЕ ТРОНУЛ
 *          (значит список не взят/ED не найден), 5 = устройство молчит,
 *          4 = Stall и т.д. + сырой word0 STATUS-TD (видны T-биты);
 *   blk  - bulk-ED флешки и CC его TD (для сбоев на MBR/BPB/записи);
 *   ПОРТЫ-СОСТ - сырые HcRhPortStatus и текущее состояние драйвера. */
/* «ЭТАП 3»: снимок печатается для КАЖДОГО живого OHCI-инстанса — на
 * AMD FCH их два, и g_oi после init-циклов застревает на последнем. */
/* [#469] ЭТАП 9: ВСЕ OHCI_NINST инстанса (было oi < 2 — СНИМОК 3-го/4-го
 * OHCI не печатался вовсе: диагностика была СЛЕПОЙ именно на тех
 * контроллерах, где у пользователя не работает ввод). */
static void ohci_snapshot_one(void);
void mouse_hw_ohci_snapshot(void) {
    int printed = 0;
    for (int oi = 0; oi < OHCI_NINST; oi++) {
        if (g_oh_mmio_a[oi] == 0) continue;
        g_oi = oi;
        if (printed++ > 0) {
            serial_puts("\n---- (OHCI-инстанс №");
            portmap_pm_n(oi + 1);
            serial_puts(") ----\n");
        }
        ohci_snapshot_one();
    }
    g_oi = 0;
    if (!printed) {
        /* ни одного прощупанного OHCI — как раньше, честное «смысла нет» */
        g_oi = 0;
        ohci_snapshot_one();
    }
}

static void ohci_snapshot_one(void) {
    char nb[12];
    serial_puts("\n==== OHCI СНИМОК (сфотографируй ЭТОТ блок целиком) ====\n");
    serial_puts("шаг: ");
    serial_puts(g_oh_diag);
    serial_puts("\n");
    if (!g_oh_probed) {
        /* probe_ohci НЕ выполнялся. ФОТО раньше СНИМОК печатал
         * МУСОР (0x32 во всех полях) - кто-то затёр statics ПОСЛЕ зануления
         * BSS, а фильтр «g_oh_mmio == 0» обманулся мусорным ненулевым mmio.
         * Теперь без реального скана печатаем честное «смысла не несёт». */
        serial_puts("HC: скан OHCI НЕ выполнялся (мышь уже на xHCI либо\n");
        serial_puts("    скан прерван раньше) - полей ниже нет по определению\n");
        serial_puts("==== КОНЕЦ СНИМКА ====\n");
        return;
    }
    if (g_oh_mmio == 0) {
        serial_puts("HC: OHCI-контроллер НЕ НАЙДЕН (mmio=0)\n");
        serial_puts("==== КОНЕЦ СНИМКА ====\n");
        return;
    }
    dma_sync();   /* EDc/cc/DoneQ пишет HC — СНИМОК из ОЗУ */
    uint32_t ctl = oh_r32(OH_CTRL);
    uint32_t csa = oh_r32(OH_CMDSTS);
    uint32_t ist = oh_r32(OH_INTSTS);
    serial_puts("HC: ctrl=0x");
    serial_puthex(ctl);
    serial_puts(" cmd=0x");
    serial_puthex(csa);
    serial_puts(" ist=0x");
    serial_puthex(ist);
    serial_puts("\n");
    {
        uint32_t st = (ctl >> 6) & 3u;
        serial_puts("    сост: ");
        serial_puts(st == 2 ? "Operational(работает)"
                  : st == 3 ? "USBSuspend(СПИТ!)"
                  : st == 1 ? "USBResume" : "USBReset");
        serial_puts(" CLE");
        serial_puts((ctl & OH_CTRL_CLE) ? "+" : "-");
        serial_puts(" BLE");
        serial_puts((ctl & OH_CTRL_BLE) ? "+" : "-");
        serial_puts(" PLE");
        serial_puts((ctl & OH_CTRL_PLE) ? "+" : "-");
        serial_puts(" | CLF");
        serial_puts((csa & OH_CMD_CLF) ? "+" : "-");
        serial_puts(" BLF");
        serial_puts((csa & OH_CMD_BLF) ? "+" : "-");
        serial_puts(" | WDH");
        serial_puts((ist & OH_INT_WDH) ? "+" : "-");
        serial_puts(" SF");
        serial_puts((ist & (1u << 2)) ? "+" : "-");
        serial_puts("\n");
    }
    uint32_t fm1 = oh_r32(OH_FMNUM) & 0xFFFFu;
    hw_delay(2000);
    uint32_t fm2 = oh_r32(OH_FMNUM) & 0xFFFFu;
    serial_puts("FM: 0x");
    serial_puthex(fm1);
    serial_puts(" -> 0x");
    serial_puthex(fm2);
    serial_puts(fm1 == fm2 ? " (НЕ ТИКАЕТ - HC стоит)" : " (тикает)");
    serial_puts("\n");
    serial_puts("REG: HCCA=0x");
    serial_puthex(oh_r32(OH_HCCA));
    serial_puts("/0x");
    serial_puthex(vphys(oh_hcca));
    serial_puts(" CtlHead=0x");
    serial_puthex(oh_r32(OH_CTRL_HEAD));
    serial_puts("/0x");
    serial_puthex(vphys(oh_ed_ctrl));
    serial_puts("\n     BulkHead=0x");
    serial_puthex(oh_r32(OH_BULK_HEAD));
    serial_puts(" DoneReg=0x");
    serial_puthex(oh_r32(OH_DONEHEAD));
    serial_puts(" DoneQ=0x");
    serial_puthex(oh_donehead());
    serial_puts("\n");
    serial_puts("EDc: ");
    for (int i = 0; i < 4; i++) {
        serial_puthex(oh_dma32(&oh_ed_ctrl[i]));
        serial_puts(" ");
    }
    serial_puts("\n");
    serial_puts("cc: setup=0x");
    serial_puthex(TD_CC_GET(oh_dma32(&oh_td[0][0])));
    serial_puts(" data=0x");
    serial_puthex(TD_CC_GET(oh_dma32(&oh_td[1][0])));
    serial_puts(" status=0x");
    serial_puthex(TD_CC_GET(oh_dma32(&oh_td[2][0])));
    serial_puts(" stTD=0x");
    serial_puthex(oh_dma32(&oh_td[2][0]));
    serial_puts("\n");
    serial_puts("blk: EDo2=0x");
    serial_puthex(oh_dma32(&oh_ed_blk_o[2]));
    serial_puts(" EDi2=0x");
    serial_puthex(oh_dma32(&oh_ed_blk_i[2]));
    serial_puts(" ccB0=0x");
    serial_puthex(TD_CC_GET(oh_dma32(&oh_td_blk[0][0])));
    serial_puts(" ccB8=0x");
    serial_puthex(TD_CC_GET(oh_dma32(&oh_td_blk[8][0])));
    serial_puts("\n");
    serial_puts("ПОРТЫ:");
    for (int p = 1; p <= (int)g_oh_nports && p <= 15; p++) {
        serial_puts(" p");
        serial_puts(itoa((uint32_t)p, nb, 10));
        serial_puts("=0x");
        serial_puthex(oh_r32(oh_port(p)));
        if ((p & 3u) == 0 && p != (int)g_oh_nports) serial_puts("\n      ");
    }
    serial_puts("\n");
    serial_puts("СОСТ: dev=");
    serial_puts(itoa((uint32_t)g_oh_addr, nb, 10));
    serial_puts(" ls=");
    serial_puts(itoa((uint32_t)g_oh_ls, nb, 10));
    serial_puts(" act=");
    serial_puts(itoa((uint32_t)g_oh_active, nb, 10));
    serial_puts(" kbd=");
    serial_puts(itoa((uint32_t)g_oh_kbd, nb, 10));
    serial_puts(" msd=");
    serial_puts(itoa((uint32_t)g_oh_msd, nb, 10));
    serial_puts(" ctl=");
    serial_puts(itoa((uint32_t)g_oh_msd_ctl, nb, 10));
    serial_puts(" nport=");
    serial_puts(itoa((uint32_t)g_oh_nports, nb, 10));
    {   /* сколько портов «устройство есть, но не перечислено» */
        int ocz = 0;
        for (int p = 1; p <= (int)g_oh_nports; p++) {
            uint32_t ps = oh_r32(oh_port(p));
            if ((ps & 1u) && !oh_port_claimed(p)) ocz++;
        }
        serial_puts(" ож=");
        serial_puts(itoa((uint32_t)ocz, nb, 10));
    }
    serial_puts("\n");
    /* регистр LS-порога + состояние канареек (СМАЗ-детектор) */
    serial_puts("LSTH=0x");
    serial_puthex(oh_r32(OH_LSTH));
    serial_puts(" канарейки=");
    serial_puts(mhw_canary_check("OHCI СНИМОК") ? "БИТЫ!" : "OK");
    serial_puts("\n");
    usbdev_dump("СНИМОК OHCI");   /* таблица устройств на экране */
    oh_memo_dump();               /* ЭП-таблица: память портов */
    serial_puts("==== КОНЕЦ СНИМКА (дальше лог не идёт) ====\n");
}

/* ═══ «xHCI СНИМОК» — диагностика мыши для финального блока kernel.c ═══
 * Печатается рядом с OHCI-СНИМОКом в самом конце экрана (sticklog убран).
 * шаг  - последний шаг перечисления (g_xh_diag);
 * HC   - mmio/live/act/slot/csz;
 * П    - сырые PORTSC всех портов;
 * OUT  - выходной контекст (что КОНТРОЛЛЕР видит/вернул): слот + EP0 +
 * EP1 IN (верные индексы контекстов: EP0 = 1*csz, EP1 IN = 3*csz);
 * IN   - входной EP0 (что МЫ писали).  Расшифровка OUT ep0_1 — по Linux
 * v6.6: MPS [31:16], EP type [5:3], CErr [2:1] (-расшифровка
 *      «CErr [31:27]/MPS [26:16]» была неверной и путала диагноз);
 * cc   - CC последней команды Address Device + порт/скорость. */
void mouse_hw_xhci_snapshot(void) {
    char nb[12];
    serial_puts("\n==== xHCI СНИМОК (мышь) ====\n");
    if (g_mmio == 0) {
        serial_puts("HC: xHCI-контроллер НЕ НАЙДЕН (mmio=0)\n");
        serial_puts("==== КОНЕЦ xHCI СНИМКА ====\n");
        return;
    }
    dma_sync();                           /* свежая DMA-память */
    serial_puts("шаг: ");
    serial_puts(g_xh_diag);
    serial_puts("\n");
    serial_puts("HC: mmio=0x");
    serial_puthex(g_mmio);
    serial_puts(" live=");
    serial_puts(itoa((uint32_t)g_xhci_live, nb, 10));
    serial_puts(" act=");
    serial_puts(itoa((uint32_t)g_active, nb, 10));
    serial_puts(" slot=");
    serial_puts(itoa((uint32_t)g_slot, nb, 10));
    serial_puts(" csz=");
    serial_puts(itoa((uint32_t)g_csz, nb, 10));
    serial_puts("\n");
    serial_puts("П:");
    for (int p = 1; p <= (int)g_maxports && p <= 8; p++) {
        serial_puts(" p");
        serial_puts(itoa((uint32_t)p, nb, 10));
        serial_puts("=0x");
        serial_puthex(mmr32(port_addr(p)));
    }
    serial_puts("\n");
    {
        /* ФОТО выявило неверные индексы чтения device context:
         * EP0 = контекст #1 (1*csz), EP1 IN = контекст #3 (3*csz).  Раньше
         * читали 2*csz = EP1 OUT (у мыши его НЕТ -> всегда нули), поэтому
 * «OUT: ep0_1=0» при живой мыши путал диагнозы. */
        uint32_t *od = (uint32_t *)hw_devctx;                 /* OUT: слот    */
        uint32_t *oe = (uint32_t *)(hw_devctx + 1u * g_csz);  /* OUT: EP0     */
        uint32_t *oi = (uint32_t *)(hw_devctx + 3u * g_csz);  /* OUT: EP1 IN  */
        uint32_t *ie = (uint32_t *)(hw_inctx + 2u * g_csz);   /* IN:  EP0     */
        serial_puts("OUT: slot0=0x");
        serial_puthex(od[0]);
        serial_puts(" slot1=0x");
        serial_puthex(od[1]);
        serial_puts(" ep0_0=0x");
        serial_puthex(oe[0]);
        serial_puts("\n");
        serial_puts("OUT: ep0_1=0x");
        serial_puthex(oe[1]);
        serial_puts(" (MPS=");
        serial_puthex((oe[1] >> 16) & 0xFFFFu);
        serial_puts(" EP=");
        serial_puthex((oe[1] >> 3) & 7u);
        serial_puts(" CErr=");
        serial_puthex((oe[1] >> 1) & 3u);
        serial_puts(")\n");
        serial_puts("OUT: ep1_1=0x");      /* настоящий EP мыши (INT IN) */
        serial_puthex(oi[1]);
        serial_puts(" (MPS=");
        serial_puthex((oi[1] >> 16) & 0xFFFFu);
        serial_puts(" EP=");
        serial_puthex((oi[1] >> 3) & 7u);
        serial_puts(" CErr=");
        serial_puthex((oi[1] >> 1) & 3u);
        serial_puts(") IN: ep0_1=0x");
        serial_puthex(ie[1]);
        serial_puts("\n");
    }
    serial_puts("cc: посл.ADR_DEV=0x");
    serial_puthex(g_xh_cc);
    serial_puts(" (порт ");
    serial_puts(itoa((uint32_t)g_xh_port, nb, 10));
    serial_puts(", скорость ");
    serial_puts(itoa(g_xh_speed, nb, 10));
    serial_puts(")\n");
    /* КАРТА ПОРТОВ - итог «диалога с устройствами» прямо на экране */
    serial_puts("ПОРТЫ:");
    for (int pr = 1; pr <= (int)g_maxports && pr <= 8; pr++) {
        const char *rn = "пусто";
        if (g_active && pr == g_port)                 rn = "МЫШЬ";
        else if (g_kbd_slot != 0 && pr == g_kbd_xport) rn = "клава";
        else if (pr < (int)sizeof(g_port_role)) {
            if (g_port_role[pr] == PR_MOUSE)          rn = "МЫШЬ";
            else if (g_port_role[pr] == PR_KBD)       rn = "клава";
            else if (g_port_role[pr] == PR_OTHER)     rn = "помечено";
        }
        serial_puts(" p");
        serial_puts(itoa((uint32_t)pr, nb, 10));
        serial_puts("=");
        serial_puts(rn);
    }
    serial_puts("\n");
    if (g_kbd_slot != 0) {
        serial_puts("КЛАВИАТУРА: xHCI порт ");
        serial_puts(itoa((uint32_t)g_kbd_xport, nb, 10));
        serial_puts(", слот ");
        serial_puts(itoa((uint32_t)g_kbd_slot, nb, 10));
        serial_puts(" (сканкоды -> ps2.c)\n");
    }
    serial_puts("канарейки=");
    serial_puts(mhw_canary_check("xHCI СНИМОК") ? "БИТЫ!" : "OK");
    serial_puts("\n");
    usbdev_dump("СНИМОК xHCI");   /* таблица устройств на экране */
    serial_puts("==== КОНЕЦ xHCI СНИМКА ====\n");
}

bool mouse_hw_msd_present(void) { return g_oh_msd != 0; }
uint32_t mouse_hw_msd_sectors(void) { return g_oh_msd_sect; }
const char *mouse_hw_msd_name(void) { return g_oh_msd_name; }

/* Перечислить флешку (вызывается из ohci_enumerate ПОСЛЕ SET_CONFIG).
 * INQUIRY (x3) -> TEST UNIT READY (x5) -> READ CAPACITY(10). */
static void oh_msd_init(void) {
    g_oh_msd = 0;
    g_oh_msd_tag = 0;
    g_oh_msd_ctl = 1;
    g_oh_msd_addr = g_oh_addr;             /* ЕЁ адрес — на всю жизнь ED     */
    g_oh_msd_mps = 64;                     /* full-speed bulk                */
    oh_ed_blk_o[0] = ED_SKIP;
    oh_ed_blk_o[1] = vphys(oh_td_tail);
    oh_ed_blk_o[2] = 0;
    oh_ed_blk_o[3] = 0;
    oh_ed_blk_i[0] = ED_SKIP;
    oh_ed_blk_i[1] = vphys(oh_td_tail);
    oh_ed_blk_i[2] = 0;
    oh_ed_blk_i[3] = 0;
    int ok = 0;
    for (int a = 0; a < 5 && !ok; a++) {  /* 5 попыток (медленные флешки) */
        ok = oh_msd_inquiry();
        if (!ok) hw_delay(250000);
    }
    if (!ok) {
        oh_diag_set("флешка: INQUIRY не прошла (BOT/CSW)");
        serial_puts("MOUSEHW: OHCI MSD: INQUIRY не прошла\n");
        g_oh_msd_ctl = 0;
        return;
    }
    oh_diag_set("флешка: INQUIRY ok '");
    oh_diag_set2(g_oh_diag, g_oh_msd_name);
    oh_diag_set2(g_oh_diag, "'");
    serial_puts("MOUSEHW: OHCI MSD: '");
    serial_puts(g_oh_msd_name);
    serial_puts("'\n");
    ok = 0;
    for (int a = 0; a < 5 && !ok; a++) {
        ok = oh_msd_ready();
        if (!ok) hw_delay(250000);
    }
    if (!ok) {
        oh_diag_set("флешка: TEST UNIT READY не прошла");
        serial_puts("MOUSEHW: OHCI MSD: TEST UNIT READY не прошла\n");
        g_oh_msd_ctl = 0;
        return;
    }
    if (!oh_msd_capacity()) {
        oh_diag_set("флешка: READ CAPACITY не прошла");
        serial_puts("MOUSEHW: OHCI MSD: READ CAPACITY не прошла\n");
        g_oh_msd_ctl = 0;
        return;
    }
    g_oh_msd = 1;
    g_oh_msd_ctl = 0;
    oh_diag_set("флешка ГОТОВА (BOT+SCSI работает)");
    serial_puts("MOUSEHW: USB-НАКОПИТЕЛЬ готов (OHCI): LBA ");
    serial_puthex(g_oh_msd_sect);
    serial_puts(" (");
    serial_puthex(g_oh_msd_sect >> 1);
    serial_puts(" КБ), BOT+SCSI работает\n");
}

/* ═══ [LOGS] ЭТАП 11: USB Mass Storage (BOT + SCSI) на xHCI ══════════════
 * Транспорт для флешки на xHCI-контроллере (реальные ПК 2013+).  Всё как
 * у OHCI-варианта выше, но фазы BOT — NORMAL TRB на кольцах bulk-EP:
 *   CBW  → bulk-OUT кольцо (31 байт)  → doorbell 2*ep_out
 *   DATA → кольцо направления          → doorbell 2*ep(+dir)
 *   CSW  → bulk-IN кольцо (13 байт)   → doorbell 2*ep_in+1
 * Ожидание — Transfer Event с совпадением слота И DC-цели (события
 * соседей по кольцу событий съедаются — после команды пере-arm мыши и
 * клавиатуры, паттерн #456).  Тайм-ауты ограничены (2-5 с на фазу). */

/* Найти MSC/BOT-интерфейс в конфиг-дескрипторе: класс 0x08/0x06/0x50 +
 * ПАРА bulk-EP.  Заполняет g_xm_ep_out/g_xm_ep_in/g_xm_mps. */
static bool xm_dev_probe(const uint8_t *cfg, uint16_t total, uint32_t speed) {
    int pos = 0;
    int msc_iface = -1;
    uint8_t ep_out = 0, ep_in = 0, mps = 64;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (pos + len > (int)total) break;
        if (type == DESC_INTERFACE && len >= 9 &&
            cfg[pos + 5] == 0x08 && cfg[pos + 6] == 0x06 &&
            cfg[pos + 7] == 0x50) {
            msc_iface = cfg[pos + 2];
        } else if (type == DESC_ENDPOINT && len >= 7 && msc_iface >= 0) {
            uint8_t ep_addr = cfg[pos + 2];
            uint8_t ep_type = cfg[pos + 3] & 0x03u;
            if (ep_type == 0x02) {              /* bulk */
                uint16_t w = (uint16_t)(cfg[pos + 4] |
                                        ((uint16_t)cfg[pos + 5] << 8));
                if (ep_addr & 0x80u) { ep_in = ep_addr & 0x0Fu; mps = (uint8_t)(w & 0x7FFu); }
                else                 { ep_out = ep_addr & 0x0Fu; mps = (uint8_t)(w & 0x7FFu); }
            }
        }
        pos += len;
    }
    if (msc_iface < 0 || ep_out == 0 || ep_in == 0) return false;
    if (ep_out > 7 || ep_in > 7) {   /* контексты EP≤7 (размер буфера) */
        serial_puts("MOUSEHW: xHCI MSD: bulk-EP выше 7 не поддержан (out=");
        serial_puthex(ep_out);
        serial_puts(" in=");
        serial_puthex(ep_in);
        serial_puts(")\n");
        return false;
    }
    g_xm_ep_out = ep_out;
    g_xm_ep_in  = ep_in;
    /* MPS по скорости: FS ≤ 64, HS ≤ 512, SS = 1024 (wMaxPacketSize у
     * SuperSpeed-дескриптора 1024; QEMU ставит честно) */
    if (speed == PORT_SPEED_SUPER) g_xm_mps = 1024u;
    else {
        g_xm_mps = mps;
        if (g_xm_mps == 0 || g_xm_mps > 512u) g_xm_mps =
            (speed == PORT_SPEED_HIGH) ? 512u : 64u;
        if (speed != PORT_SPEED_HIGH && g_xm_mps > 64u) g_xm_mps = 64u;
    }
    return true;
}

/* bulk-передача (одна фаза BOT): len ≤ 512, buf — непрерывный буфер. */
static bool xm_bulk(int dir_in, void *buf, uint32_t len) {
    if (g_xm_slot <= 0 || len == 0 || len > 512) return false;
    uint32_t target = dir_in
        ? (2u * (uint32_t)g_xm_ep_in + 1u)
        : (2u * (uint32_t)g_xm_ep_out);
    mhw_ring_t *r = dir_in ? &r_msd_i : &r_msd_o;
    ring_put(r, vphys(buf), len, TRB_TYPE(TRB_TR_NORMAL) | TRB_IOC);
    dma_sync();
    mmw32(g_db + 4u * (uint32_t)g_xm_slot, target);

    int timeout = 5000;                    /* 5 с на фазу — bounded */
    for (;;) {
        uint64_t p; uint32_t st, ctl;
        while (evt_get(&p, &st, &ctl)) {
            uint32_t type = (ctl >> 10) & 0x3Fu;
            if (type == TRB_EV_TRANSFER &&
                (ctl >> 24) == (uint32_t)g_xm_slot &&
                ((ctl >> 16) & 0x1Fu) == target) {
                uint32_t cc = st >> 24;
                if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) {
                    serial_puts("MOUSEHW: xHCI MSD bulk cc=");
                    serial_puthex(cc);
                    serial_puts(" (");
                    serial_puts(cc_name(cc));
                    serial_puts(") len=");
                    serial_puthex(len);
                    serial_puts("\n");
                    return false;
                }
                return true;
            }
        }
        if (timeout-- <= 0) {
            serial_puts("MOUSEHW: xHCI MSD bulk TIMEOUT (слот ");
            serial_puthex((uint32_t)g_xm_slot);
            serial_puts(", цель ");
            serial_puthex(target);
            serial_puts(", len=");
            serial_puthex(len);
            serial_puts(")\n");
            return false;
        }
        hw_delay(1000);
    }
}

/* BOT-команда (CBW → DATA → CSW) на xHCI.  Как oh_msd_bot. */
static bool xm_bot(const uint8_t *cdb, int cbl, int data_in,
                   uint8_t *data, uint32_t dlen) {
    memset(hw_msd_cbw, 0, 31);
    hw_msd_cbw[0] = 0x55; hw_msd_cbw[1] = 0x53;   /* 'US' (LE 0x43425355) */
    hw_msd_cbw[2] = 0x42; hw_msd_cbw[3] = 0x43;
    g_xm_tag++;
    hw_msd_cbw[4] = (uint8_t)g_xm_tag;
    hw_msd_cbw[5] = (uint8_t)(g_xm_tag >> 8);
    hw_msd_cbw[6] = (uint8_t)(g_xm_tag >> 16);
    hw_msd_cbw[7] = (uint8_t)(g_xm_tag >> 24);
    hw_msd_cbw[8] = (uint8_t)dlen;
    hw_msd_cbw[9] = (uint8_t)(dlen >> 8);
    hw_msd_cbw[10] = (uint8_t)(dlen >> 16);
    hw_msd_cbw[11] = (uint8_t)(dlen >> 24);
    hw_msd_cbw[12] = data_in ? 0x80u : 0x00u;
    hw_msd_cbw[13] = 0;                     /* LUN */
    hw_msd_cbw[14] = (uint8_t)cbl;
    for (int i = 0; i < cbl && i < 16; i++) hw_msd_cbw[15 + i] = cdb[i];

    /* события мыши/клавы ждут пере-arm после каждой фазы (съедаются
     * ожиданием) — как #456 в перечислении.
     * Буфер данных — НАПРЯМУЮ (TRB param): физически непрерывен
     * (identity-модель памяти); 512-байтные окна не пересекают 64К при
     * выравнивании буфера вызывающего (logstick: 4096). */
    if (!xm_bulk(0, hw_msd_cbw, 31)) { hw_arm_mouse_again(); hw_kbd_arm_again(); return false; }
    if (dlen) {
        if (!xm_bulk(data_in, data, dlen)) { hw_arm_mouse_again(); hw_kbd_arm_again(); return false; }
    }
    memset(hw_msd_csw, 0, 13);
    if (!xm_bulk(1, hw_msd_csw, 13)) { hw_arm_mouse_again(); hw_kbd_arm_again(); return false; }
    hw_arm_mouse_again();
    hw_kbd_arm_again();
    /* CSW: сигнатура 'USBS' (0x53425355: 55 53 42 53!) + тег + status.
     * (байт 3 у CSW = 'S', у CBW = 'C' — классическая ловушка) */
    if (hw_msd_csw[0] != 0x55 || hw_msd_csw[1] != 0x53 ||
        hw_msd_csw[2] != 0x42 || hw_msd_csw[3] != 0x53 ||
        hw_msd_csw[4] != (uint8_t)g_xm_tag || hw_msd_csw[5] != (uint8_t)(g_xm_tag >> 8) ||
        hw_msd_csw[6] != (uint8_t)(g_xm_tag >> 16) || hw_msd_csw[7] != (uint8_t)(g_xm_tag >> 24)) {
        serial_puts("MOUSEHW: xHCI MSD CSW bad: sig=");
        serial_puthex((uint32_t)hw_msd_csw[0] | ((uint32_t)hw_msd_csw[1] << 8) |
                      ((uint32_t)hw_msd_csw[2] << 16) | ((uint32_t)hw_msd_csw[3] << 24));
        serial_puts(" st=");
        serial_puthex(hw_msd_csw[12]);
        serial_puts("\n");
        return false;
    }
    if (hw_msd_csw[12] != 0) return false;  /* команда отклонена (TUR и т.п.) */
    return true;
}

static void xm_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static bool xm_inquiry(void) {
    uint8_t cdb[6] = { 0x12, 0, 0, 0, 36, 0 };
    memset(hw_msd_dat, 0, sizeof(hw_msd_dat));
    if (!xm_bot(cdb, 6, 1, hw_msd_dat, 36)) return false;
    for (int i = 0; i < 8; i++)
        g_xm_name[i] = (char)(hw_msd_dat[8 + i] >= 0x20 ? hw_msd_dat[8 + i] : ' ');
    g_xm_name[8] = ' ';
    for (int i = 0; i < 16; i++)
        g_xm_name[9 + i] = (char)(hw_msd_dat[16 + i] >= 0x20 ? hw_msd_dat[16 + i] : ' ');
    g_xm_name[25] = 0;
    return true;
}

static bool xm_ready(void) {
    uint8_t cdb[6] = { 0, 0, 0, 0, 0, 0 };
    return xm_bot(cdb, 6, 0, 0, 0);
}

static bool xm_capacity(void) {
    uint8_t cdb[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    memset(hw_msd_dat, 0, sizeof(hw_msd_dat));
    if (!xm_bot(cdb, 10, 1, hw_msd_dat, 8)) return false;
    g_xm_sect = ((uint32_t)hw_msd_dat[0] << 24) | ((uint32_t)hw_msd_dat[1] << 16) |
                ((uint32_t)hw_msd_dat[2] << 8) | (uint32_t)hw_msd_dat[3];
    g_xm_sect += 1;
    return true;
}

/* Перечислить MSD на СЛОТЕ ПЕРЕЧИСЛЕНИЯ (вызывается из hw_enumerate,
 * когда устройство — не HID): SET_CONFIG + Configure Endpoint (bulk-пара)
 * + INQUIRY/TUR/CAPACITY.  При успехе слот переходит MSD насовсем. */
static bool xm_setup(uint8_t port, uint32_t speed, uint8_t cfg_val) {
    /* 1. SET_CONFIGURATION */
    if (!ctrl_xfer(0x00, USB_REQ_SET_CONFIG, cfg_val, 0, 0, 0)) {
        serial_puts("MOUSEHW: xHCI MSD: SET_CONFIG не прошёл\n");
        return false;
    }
    /* 2. свой output-контекст (копия после Address Device, #423) + DCBAA */
    memset(hw_msd_devctx, 0, sizeof(hw_msd_devctx));
    memcpy(hw_msd_devctx, hw_devctx, 512);
    hw_dcbaa[g_eslot] = (uint64_t)vphys(hw_msd_devctx);
    /* 3. кольца bulk — с чистого листа (fresh dequeue, DCS=1) */
    for (int i = 0; i < 8; i++) {
        hw_msd_out_a[g_xi][i].control = 0;
        hw_msd_in_a[g_xi][i].control = 0;
    }
    r_msd_o.idx = 0; r_msd_o.cyc = 1;
    r_msd_i.idx = 0; r_msd_i.cyc = 1;
    /* 4. Configure Endpoint: слот + bulk-OUT + bulk-IN.
     * Контекст-индексы: EPn OUT = 2n, EPm IN = 2m+1; ICC-биты те же;
     * смещение EP-контекста в input-контексте = (индекс+1)*g_csz. */
    uint32_t idx_o = 2u * (uint32_t)g_xm_ep_out;
    uint32_t idx_i = 2u * (uint32_t)g_xm_ep_in + 1u;
    uint32_t last  = (idx_o > idx_i) ? idx_o : idx_i;
    memset(hw_msd_inctx, 0, sizeof(hw_msd_inctx));
    {
        uint32_t *icc = (uint32_t *)hw_msd_inctx;
        icc[0] = 0;                     /* drop */
        icc[1] = ICC_SLOT | (1u << idx_o) | (1u << idx_i);
        uint32_t *sd = (uint32_t *)(hw_msd_inctx + g_csz);
        sd[0] = SLOT_LAST_CTX(last) | SLOT_SPEED(speed);
        sd[1] = SLOT_ROOT_PORT(port);
        sd[2] = 0; sd[3] = 0;
        /* bulk-OUT контекст (EP type 2) */
        uint32_t *d = (uint32_t *)(hw_msd_inctx + (idx_o + 1u) * g_csz);
        d[0] = 0;
        d[1] = EP_CERR3 | (2u << 3) | EP_MAX_PACKET(g_xm_mps);
        d[2] = vphys(hw_msd_out_a[g_xi]) | 1u;    /* DCS=1 */
        d[3] = 0;
        d[4] = 512u;
        d[5] = d[6] = d[7] = 0;
        /* bulk-IN контекст (EP type 6) */
        d = (uint32_t *)(hw_msd_inctx + (idx_i + 1u) * g_csz);
        d[0] = 0;
        d[1] = EP_CERR3 | (6u << 3) | EP_MAX_PACKET(g_xm_mps);
        d[2] = vphys(hw_msd_in_a[g_xi]) | 1u;     /* DCS=1 */
        d[3] = 0;
        d[4] = 512u;
        d[5] = d[6] = d[7] = 0;
    }
    uint32_t trb = ring_put(&r_cmd, vphys(hw_msd_inctx), 0,
                            TRB_TYPE(TRB_CR_CONFIG_EP) | ((uint32_t)g_eslot << 24));
    db_command();
    uint32_t cc = 0, sl = 0;
    if (!wait_cmd(trb, &cc, &sl)) {
        serial_puts("MOUSEHW: xHCI MSD: configure endpoint failed cc=");
        serial_puthex(cc);
        serial_puts(" (");
        serial_puts(cc_name(cc));
        serial_puts(")\n");
        return false;
    }
    /* 5. слот переходит MSD насовсем (владение как у мыши/клавы) */
    g_xm_slot = g_eslot;
    g_eslot   = 0;
    g_xm_xi   = g_xi;
    g_xm_port = port;
    /* 6. SCSI: INQUIRY (x5, медленные флешки) → TUR (x5) → CAPACITY */
    int ok = 0;
    for (int a = 0; a < 5 && !ok; a++) {
        ok = xm_inquiry();
        if (!ok) hw_delay(250000);
    }
    if (!ok) {
        serial_puts("MOUSEHW: xHCI MSD: INQUIRY не прошла\n");
        g_xm_slot = 0; g_xm_xi = -1;
        return false;
    }
    serial_puts("MOUSEHW: xHCI MSD: '");
    serial_puts(g_xm_name);
    serial_puts("'\n");
    ok = 0;
    for (int a = 0; a < 5 && !ok; a++) {
        ok = xm_ready();
        if (!ok) hw_delay(250000);
    }
    if (!ok) {
        serial_puts("MOUSEHW: xHCI MSD: TEST UNIT READY не прошла\n");
        g_xm_slot = 0; g_xm_xi = -1;
        return false;
    }
    if (!xm_capacity()) {
        serial_puts("MOUSEHW: xHCI MSD: READ CAPACITY не прошла\n");
        g_xm_slot = 0; g_xm_xi = -1;
        return false;
    }
    serial_puts("MOUSEHW: USB-НАКОПИТЕЛЬ готов (xHCI): LBA ");
    serial_puthex(g_xm_sect);
    serial_puts(" (");
    serial_puthex(g_xm_sect >> 1);
    serial_puts(" КБ), BOT+SCSI работает\n");
    return true;
}

/* ── экспорт для logstick.c (единый блочный интерфейс) ── */
bool mouse_hw_xhci_msd_present(void) { return g_xm_slot != 0; }
uint32_t mouse_hw_xhci_msd_sectors(void) { return g_xm_sect; }
const char *mouse_hw_xhci_msd_name(void) { return g_xm_name; }

bool mouse_hw_xhci_msd_read(uint32_t lba, void *buf512) {
    if (!g_xm_slot) return false;
    int sv = g_xi;
    g_xi = g_xm_xi;                    /* регистры/кольца — инстанса MSD */
    uint8_t cdb[10] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0 };
    xm_be32(cdb + 2, lba);
    bool ok = xm_bot(cdb, 10, 1, (uint8_t *)buf512, 512);
    g_xi = sv;
    return ok;
}

bool mouse_hw_xhci_msd_write(uint32_t lba, const void *buf512) {
    if (!g_xm_slot) return false;
    int sv = g_xi;
    g_xi = g_xm_xi;
    uint8_t cdb[10] = { 0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0 };
    xm_be32(cdb + 2, lba);
    /* xm_bot копирует данные в hw_msd_dat (непрерывный буфер) */
    bool ok = xm_bot(cdb, 10, 0, (uint8_t *)buf512, 512);
    g_xi = sv;
    return ok;
}

/* OHCI-вариант детекта «планшет или мышь»: report descriptor читается
 * через oh_ctrl.  Раньше OHCI-путь вызывал xHCI ctrl_xfer (контроллер не
 * инициализирован) - latent-баг; boot-мыши он не мешал, но планшет на OHCI
 * определился бы неверно. */
static bool oh_detect_mode(const uint8_t *cfg, uint16_t total) {
    g_rpt_ok = false;   /* ре-enum - старая раскладка недействительна */
    int pos = 0, rpt_len = 0;
    bool in_iface = false;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) in_iface = (cfg[pos + 2] == g_iface);
        else if (type == 0x21u && in_iface) {
            rpt_len = cfg[pos + 7] | ((int)cfg[pos + 8] << 8);
            break;
        }
        pos += len;
    }
    if (rpt_len <= 0) return !g_boot_iface;
    if (rpt_len > (int)sizeof(hw_rpt)) rpt_len = (int)sizeof(hw_rpt);
    memset(hw_rpt, 0, sizeof(hw_rpt));
    if (oh_ctrl(0x81, 0x06, (0x22 << 8), g_iface, (uint16_t)rpt_len, hw_rpt)
        != TD_CC_OK) {
        serial_puts("MOUSEHW: OHCI: report desc не читается - режим по subclass\n");
        return !g_boot_iface;
    }
    {
        bool absolute = hw_parse_absolute(hw_rpt, (uint16_t)rpt_len, &g_log_max);
        hw_parse_axes(hw_rpt, (uint16_t)rpt_len);  /* раскладка осей X/Y */
        return absolute;
    }
}

/* Вердикт по «непонятному» HID на OHCI - та же цепочка уровней,
 * что на xHCI: L2 usage-скан report descriptor'а (пассивная правда,
 * hid_scan_report как в Linux) -> L3 LED-проба «зажги NumLock+CapsLock»
 * (таймаут ~300 мс внутри oh_ctrl - «мёртвое» устройство перечисление
 * не вешает) -> дальше решает вызывающий (неоднозначно = клава). */
static int oh_ask_verdict(uint8_t iface, const uint8_t *cfg, uint16_t total,
                          const char **via) {
    bool kbd = false, mouse = false;
    int rl = hw_rpt_len(cfg, total, iface);
    if (rl > 0 && rl <= (int)sizeof(hw_rpt)) {
        memset(hw_rpt, 0, sizeof(hw_rpt));
        if (oh_ctrl(0x81, 0x06, (USB_DT_REPORT << 8), iface, (uint16_t)rl, hw_rpt)
            == TD_CC_OK) {
            hw_usage_scan(hw_rpt, rl, &kbd, &mouse);
            int vd = hid_verdict(0, kbd, mouse, false);       /* L2 */
            if (vd != VERD_UNKNOWN) {
                *via = "descriptor";
                serial_puts(vd == VERD_KBD
                    ? "MOUSEHW: OHCI: дескриптор говорит КЛАВИАТУРА (usage=0x06)\n"
                    : "MOUSEHW: OHCI: дескриптор говорит МЫШЬ (usage=0x02)\n");
                return vd;
            }
        }
    }
    /* [#446] LED по текущему состоянию Num/Caps (не «зажечь всё») */
    uint8_t leds;
    { int ln = 0, lc = 0; ps2_kbd_led_state(&ln, &lc);
      leds = (uint8_t)((ln ? 1u : 0u) | (lc ? 2u : 0u)); }
    int cc = oh_ctrl(0x21, USB_HID_SET_REPORT, 0x0200u, iface, 1, &leds);
    if (cc == TD_CC_OK) {
        serial_puts("MOUSEHW: OHCI LED-проба: SET_REPORT подтверждён - это КЛАВИАТУРА\n");
        *via = "led";
        return VERD_KBD;
    }
    serial_puts("MOUSEHW: OHCI: LED-проба не ответила - неоднозначно (вид не доказан)\n");
    *via = "";
    return VERD_UNKNOWN;
}

/* честный разбор конфигурации: HID-мышь (protocol 2) и/или
 * HID-клавиатура (protocol 1) + их interrupt-IN эндпоинты.  Раньше
 * hw_find_hid хватал ЛЮБОЙ HID-интерфейс - клавиатура начинала кормить
 * курсор клавиатурными отчётами.  g_iface/g_mps - мышь, k_статики - клава.
 * protocol 0 («непонятно») собирается отдельно и идёт на LED-пробу. */
static bool oh_parse_device(const uint8_t *cfg, uint16_t total,
                            bool *has_mouse, bool *has_kbd,
                            bool *has_msd, bool *has_ambig) {
    *has_mouse = false;
    *has_kbd   = false;
    *has_msd   = false;
    *has_ambig = false;
    int     m_iface = -1, k_iface = -1;
    int     b_iface = -1;                       /* MSC/BOT интерфейс */
    int     a_iface = -1;                       /* protocol 0 - «непонятно» */
    bool    m_boot = false, k_boot = false;
    bool    m_epok = false, k_epok = false, a_epok = false;
    uint8_t m_mps = 8, k_mps = 8, m_ep = 1, k_ep = 1;
    uint8_t a_mps = 8, a_ep = 1, a_sub = 0;
    bool    b_inok = false, b_outok = false;
    uint8_t b_in = 0, b_out = 0;
    int pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE && cfg[pos + 5] == 0x03u) {
            int  cur  = cfg[pos + 2];
            bool boot = (cfg[pos + 6] == 0x01u);        /* subclass = boot  */
            if (cfg[pos + 7] == 0x02u) {                /* protocol = mouse */
                if (m_iface < 0 || (boot && !m_boot)) { m_iface = cur; m_boot = boot; }
            } else if (cfg[pos + 7] == 0x01u) {         /* protocol = kbd   */
                if (k_iface < 0 || (boot && !k_boot)) { k_iface = cur; k_boot = boot; }
            } else if (a_iface < 0) {                   /* protocol 0 */
                a_iface = cur;                          /* - на LED-пробу   */
                a_sub   = boot;                         /* subclass: boot?  */
            }
        }
        if (type == DESC_INTERFACE && cfg[pos + 5] == 0x08u &&
            cfg[pos + 6] == 0x06u && cfg[pos + 7] == 0x50u && b_iface < 0)
            b_iface = cfg[pos + 2];                 /* MSC / BOT */
        pos += len;
    }
    bool in_m = false, in_k = false, in_b = false, in_a = false;
    pos = 0;
    while (pos + 1 < (int)total) {
        uint8_t len  = cfg[pos];
        uint8_t type = cfg[pos + 1];
        if (len < 2) break;
        if (type == DESC_INTERFACE) {
            in_m = (m_iface >= 0 && cfg[pos + 2] == m_iface);
            in_k = (k_iface >= 0 && cfg[pos + 2] == k_iface);
            in_b = (b_iface >= 0 && cfg[pos + 2] == b_iface);
            in_a = (a_iface >= 0 && cfg[pos + 2] == a_iface);
        } else if (type == DESC_ENDPOINT) {
            uint8_t addr = cfg[pos + 2];
            uint8_t attr = cfg[pos + 3];
            if ((addr & 0x80u) && (attr & 0x03u) == 0x03u) {      /* INT IN */
                uint16_t mps = cfg[pos + 4] | ((uint16_t)cfg[pos + 5] << 8);
                if (mps < 4)  mps = 4;
                if (mps > 16) mps = 16;    /* реальная мышь может слать
                                              до 8+ байт - буферы 16 байт      */
                if (in_m) { m_ep = (uint8_t)(addr & 0xFu); m_mps = (uint8_t)mps; m_epok = true; }
                if (in_k) { k_ep = (uint8_t)(addr & 0xFu); k_mps = (uint8_t)mps; k_epok = true; }
                if (in_a) { a_ep = (uint8_t)(addr & 0xFu); a_mps = (uint8_t)mps; a_epok = true; }
            }
            if (in_b && (attr & 0x03u) == 0x02u) {                /* BULK */
                if (addr & 0x80u) { b_in  = (uint8_t)(addr & 0xFu); b_inok  = true; }
                else              { b_out = (uint8_t)(addr & 0xFu); b_outok = true; }
            }
        }
        pos += len;
    }
    if (m_iface >= 0 && m_epok) {
        g_iface      = (uint8_t)m_iface;
        g_boot_iface = m_boot;
        g_mps        = m_mps;
        g_oh_ms_ep   = m_ep;
        *has_mouse = true;
    }
    if (k_iface >= 0 && k_epok) {
        g_oh_kbd_ep    = k_ep;
        g_oh_kbd_mps   = k_mps;
        g_oh_kbd_iface = (uint8_t)k_iface;
        *has_kbd = true;
    }
    if (b_iface >= 0 && b_inok && b_outok) {
        g_oh_msd_in  = b_in;
        g_oh_msd_out = b_out;
        *has_msd = true;
    }
    if (a_iface >= 0 && a_epok) {
        g_oh_amb_iface = (uint8_t)a_iface;
        g_oh_amb_ep    = a_ep;
        g_oh_amb_mps   = a_mps;
        g_oh_amb_sub   = a_sub;
        *has_ambig = true;
    }
    return *has_mouse || *has_kbd || *has_msd || *has_ambig;
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
static void oh_inject_sc(uint8_t sc, int ext, int is_break) {
    if (ext) ps2_kbd_inject(0xE0);
    ps2_kbd_inject(is_break ? (uint8_t)(0x80u | sc) : sc);
}

/* разобрать boot-отчёт клавиатуры [mods][rsv][k1..k6].
 * модификаторы больше НЕ гоняются синтетическим make/break-потоком -
 * потерянный при передаче break навсегда включал Ctrl, и нажатие «о»
 * (физическая «J») срабатывало как Ctrl+J: режим курсора щёлкал, буква
 * пропадала.  Boot-протокол кладёт ПОЛНЫЙ байт модификаторов в КАЖДЫЙ
 * отчёт - отдаём его целиком (ps2_kbd_mods), залипать нечему.  Клавиши -
 * по-прежнему диффом массива usage; буфер за пределами act обнулён. */
/* [#475] Диагностика Ситуации B (длина = 0): троттлинг — первое
 * сообщение + раз в 1024-е, чтобы непрерывный поток нулевых длин
 * не тонил serial-журнал (как printk_once/ratelimit в Linux). */
/* [#476] prev_k вынесен из oh_kbd_process на уровень файла: сброс
 * oh_kbd_state_reset() доступен и путям привязки/отключения. */
static uint8_t prev_k[6];
static void zl_note(const char *what) {
    static uint32_t zl_cnt;
    zl_cnt++;
    if (zl_cnt == 1u || (zl_cnt & 1023u) == 0u) {
        serial_puts("MOUSEHW: ");
        serial_puts(what);
        { char nb[12]; serial_puts(" ["); serial_puts(itoa(zl_cnt, nb, 10)); serial_puts(" раз]\n"); }
    }
}

/* [#476] Сброс состояния РАЗБОРА клавиатуры (prev-массив + счётчик
 * «первого отчёта»): при привязке НОВОЙ клавы (переезд с другой шины —
 * OHCI→xHCI, переподключение) и при detach. Без сброса: «первый отчёт»
 * не печатался (g_kbd_pkts уже >0), а prev_k от СТАРОЙ клавы давал
 * фантомные отпускания на первом отчёте НОВОЙ. */
static void oh_kbd_state_reset(void) {
    memset(prev_k, 0, sizeof(prev_k));
    g_kbd_pkts = 0;
}
static void oh_kbd_process(const uint8_t *r, int act, const char *bus) {
    uint8_t cur_k[6];
    if (act < 1) {
        /* [#475] Ситуация B: длина 0 — НЕ считаем живостью (прежде
         * счётчик стоял ДО проверки — нулевые «отчёты» доказывали
         * «ДАННЫЕ ИДУТ» в карте гнёзд). */
        zl_note("клавиатура: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)");
        return;
    }
    if (g_kbd_pkts == 0) {
        serial_puts("MOUSEHW: первый отчёт КЛАВИАТУРЫ (");
        serial_puts(bus);
        serial_puts(", ");
        serial_puthex((uint32_t)act);
        serial_puts(" байт):");
        for (int i = 0; i < act && i < 8; i++) {
            serial_puts(" ");
            serial_puthex(r[i]);
        }
        serial_puts("\n");
    }
    g_kbd_pkts++;
    usbdev_note_report(USBDEV_ROLE_KBD);   /* [#462] живость для КАРТЫ ГНЕЗД */
    /* Часть клавиатур приписывает к boot-отчёту Report ID, хотя не
     * обязана.  Решение по ДЛИНЕ (9 байт = [ID][boot-8]): эвристика
     * «r[1]!=0» на пустом отчёте [ID][00][00...] принимала ID за
     * модификаторы - фантомный Ctrl/Shift (как в usb_kbd.c). */
    if (act >= 9) { r++; act--; }
    if (act < 1) return;
    ps2_kbd_mods(r[0]);   /* Ctrl/Shift/Alt - СОСТОЯНИЕМ, а не потоком */
    for (int i = 0; i < 6; i++) {
        cur_k[i] = (2 + i < act) ? r[2 + i] : 0;
        if (cur_k[i] <= 3) cur_k[i] = 0;      /* 0/rollover/POST-ошибки     */
    }
    /* клавиши: отпускания (были в prev, нет в cur) */
    for (int i = 0; i < 6; i++) {
        uint8_t u = prev_k[i];
        if (!u) continue;
        bool gone = true;
        for (int j = 0; j < 6; j++) if (cur_k[j] == u) { gone = false; break; }
        if (gone) {
            int ext = 0;
            uint8_t sc = hid_to_set1(u, &ext);
            if (sc) oh_inject_sc(sc, ext, 1);
        }
    }
    /* нажатия (есть в cur, не было в prev) */
    for (int i = 0; i < 6; i++) {
        uint8_t u = cur_k[i];
        if (!u) continue;
        bool fresh = true;
        for (int j = 0; j < 6; j++) if (prev_k[j] == u) { fresh = false; break; }
        if (fresh) {
            int ext = 0;
            uint8_t sc = hid_to_set1(u, &ext);
            if (sc) oh_inject_sc(sc, ext, 0);
        }
    }
    for (int i = 0; i < 6; i++) prev_k[i] = cur_k[i];
}

/* перестроить периодическую таблицу HCCA БЕЗ опоры на обход Skip-ED:
 * первый активный ED - во все 32 слота, второй - в цепочку за ним
 * (NextED).  Вызывается при любом изменении состава устройств. */
static void oh_hcca_rebuild(void) {
    uint32_t first = 0, second = 0;
    if (g_oh_active) {
        first  = vphys(oh_ed_intr);
        second = g_oh_kbd ? vphys(oh_ed_kbd) : 0;
    } else if (g_oh_kbd) {
        first = vphys(oh_ed_kbd);
    }
    oh_ed_intr[3] = 0;
    oh_ed_kbd[3]  = 0;
    if (first == 0) return;
    if (first == vphys(oh_ed_intr) && second) oh_ed_intr[3] = second;
    for (int i = 0; i < 32; i++) ((uint32_t *)oh_hcca)[i] = first;
    dma_sync();                           /* HCCA — в ОЗУ */
}

/* Связывание мыши OHCI - общий код полного перебора и быстрого пути
 * ЭП-таблицы.  Вызывающий заполняет g_iface/g_mps/g_oh_ms_ep и
 * g_absolute/g_boot_iface (после oh_detect_mode).  fast=1 - устройство
 * опознано по ЭП-таблице; cfgv=0 - ЭП-память не писать (составное
 * устройство с MSD-частью). */
static void oh_bind_mouse(int port, uint16_t vid, uint16_t pid,
                          uint8_t cfgv, int fast) {
    if (!g_absolute) (void)oh_ctrl(0x21, 0x0B, 0, g_iface, 0, 0);
    (void)oh_ctrl(0x21, 0x0A, 0, g_iface, 0, 0);
    if (g_mps < 4) g_mps = 4;
    oh_td_i[0] = TD_DP_IN | TD_DI_NOW | TD_T_CARRY | TD_ROUND;
    oh_td_i[1] = vphys(oh_pkt);
    oh_td_i[2] = vphys(oh_td_tail_i);
    oh_td_i[3] = vphys(oh_pkt) + (uint32_t)g_mps - 1u;
    oh_ed_intr[0] = ED_FA(g_oh_addr) | ED_EN(g_oh_ms_ep) | ED_IN |
                    (g_oh_ls ? ED_LS : 0u) | ED_MPS(g_mps);
    oh_ed_intr[1] = vphys(oh_td_tail_i);
    oh_ed_intr[2] = vphys(oh_td_i);
    oh_ed_intr[3] = 0;
    oh_ed_intr[0] &= ~ED_SKIP;
    g_errors    = 0;
    g_packets   = 0;
    g_oh_ms_port = port;   /* запомнить СВОЙ порт */
    g_oh_ms_oi   = g_oi;   /* 0.3.0: инстанс-владелец мыши */
    g_oh_ms_usb_addr = g_oh_addr;   /* честная книга адресов */
    g_oh_ms_ls   = (uint8_t)(g_oh_ls ? 1 : 0);   /* для SET_ADDRESS */
    g_oh_active = 1;
    g_present   = true;
    g_active    = true;
    lm_save();               /* [#474] СНИМОК живой мыши (OHCI-путь) */
    oh_hcca_rebuild();
    oh_memo_save(port, UD_MOUSE, g_oh_ms_ep, g_iface, (uint8_t)g_mps,
                 cfgv, vid, pid);
    /* мышь OHCI - запись в ТАБЛИЦЕ УСТРОЙСТВ */
    usbdev_register(1, (int)port, vid, pid, UD_MOUSE, 2, (int)g_oh_addr, 1);
    hid_report_line(g_oh_ls ? "1.x" : "2.0", port, (int)g_oh_addr,
                    "mouse", fast ? "ep-table" :
                    (g_boot_iface ? "interface" : "descriptor"),
                    (int)g_iface);
    serial_puts("MOUSEHW: ready (OHCI) vid=");
    serial_puthex(vid);
    serial_puts(" pid=");
    serial_puthex(pid);
    serial_puts(" порт USB 1.x, мышь добавлена\n");
    usbmap_pick(0x10, "mouse", (int)port, ohci_sock_label());   /* [#449] честная версия ГНЕЗДА */
}

/* Связывание клавиатуры OHCI - общий код обоих путей.  Вызывающий
 * заполняет g_oh_kbd_iface/g_oh_kbd_mps/g_oh_kbd_ep. */
static void oh_bind_kbd(int port, uint16_t vid, uint16_t pid,
                        uint8_t cfgv, int fast) {
    (void)fast;
    (void)oh_ctrl(0x21, 0x0B, 0, g_oh_kbd_iface, 0, 0);
    (void)oh_ctrl(0x21, 0x0A, 0, g_oh_kbd_iface, 0, 0);
    if (g_oh_kbd_mps < 8)  g_oh_kbd_mps = 8;
    if (g_oh_kbd_mps > 16) g_oh_kbd_mps = 16;
    oh_kbd_state_reset();   /* [#476] prev_k/«первый отчёт» — с чистого листа */
    oh_td_k[0] = TD_DP_IN | TD_DI_NOW | TD_T_CARRY | TD_ROUND;
    oh_td_k[1] = vphys(oh_kpkt);
    oh_td_k[2] = vphys(oh_td_tail_i);
    oh_td_k[3] = vphys(oh_kpkt) + (uint32_t)g_oh_kbd_mps - 1u;
    oh_ed_kbd[0] = ED_FA(g_oh_addr) | ED_EN(g_oh_kbd_ep) | ED_IN |
                   (g_oh_ls ? ED_LS : 0u) | ED_MPS(g_oh_kbd_mps);
    oh_ed_kbd[1] = vphys(oh_td_tail_i);
    oh_ed_kbd[2] = vphys(oh_td_k);
    oh_ed_kbd[3] = 0;
    oh_ed_kbd[0] &= ~ED_SKIP;
    g_oh_kbd      = 1;
    g_oh_kbd_port = port;
    g_oh_kb_oi    = g_oi;   /* 0.3.0: инстанс-владелец клавиатуры */
    g_oh_kbd_usb_addr = g_oh_addr;   /* честная книга адресов */
    g_oh_kbd_ls   = g_oh_ls;
    lk_save_oh();           /* [#474] СНИМОК живой клавы (OHCI-часть) */
    oh_hcca_rebuild();
    oh_memo_save(port, UD_KBD, g_oh_kbd_ep, g_oh_kbd_iface, g_oh_kbd_mps,
                 cfgv, vid, pid);
    /* клавиатура OHCI - запись в ТАБЛИЦЕ УСТРОЙСТВ */
    usbdev_register(1, (int)port, vid, pid, UD_KBD, 1, (int)g_oh_addr, 1);
    hid_report_line(g_oh_ls ? "1.x" : "2.0", port, (int)g_oh_addr,
                    "keyboard", "interface", (int)g_oh_kbd_iface);
    serial_puts("MOUSEHW: USB-КЛАВИАТУРА найдена (OHCI порт ");
    serial_puthex((uint32_t)port);
    serial_puts(")\n");
    usbmap_pick(0x10, "keyboard", (int)port, ohci_sock_label());   /* [#449] честная версия ГНЕЗДА */
}

/* Одна проба GET_DESCRIPTOR(8) с ПЕЧАТЬЮ исхода (скорость, cc, фейк-OK).
 * Возвращает cc передачи; *fake=1 - «cc=0, но байты НЕ дескриптор»
 * (устройство ОТВЕЧАЕТ, но не дескриптором: BIOS успело его
 * адресовать/настроить - лечится ресетом порта/слепым SET_ADDRESS,
 * см. круги B/D в ohci_enumerate).  Раньше часть исходов проходила
 * МОЛЧА (фейк-OK: cc=0, а в буфере мусор; попытки на второй скорости) -
 * по фото было видно только «3x cc=5 и failed», и невозможно было
 * понять, работает ли перебор скоростей вообще.  Теперь ВИДНО каждую
 * попытку И её байты (нули = не дал данных, 0x32 = СМАЗ, отчёт = BIOS). */
static int oh_dd8_probe(uint8_t *dd8, int port, int att, int *fake) {
    int cc8 = oh_ctrl(0x80, 0x06, (0x01 << 8), 0, 8, dd8);
    char nb8[8];
    *fake = 0;
    serial_puts("MOUSEHW: OHCI порт ");
    serial_puts(itoa((uint32_t)port, nb8, 10));
    serial_puts(": проба GET_DESCRIPTOR(8) №");
    serial_puts(itoa((uint32_t)(att + 1), nb8, 10));
    serial_puts(g_oh_ls ? " low-speed cc=" : " full-speed cc=");
    serial_puthex((uint32_t)cc8);
    if (cc8 == TD_CC_OK && !(dd8[0] == 0x12 && dd8[1] == 0x01)) {
        *fake = 1;
        serial_puts(" ДАННЫЕ НЕ ДЕСКРИПТОР (фейк-OK, считаю неудачей) байты:");
        for (int i = 0; i < 8; i++) { serial_puts(" "); serial_puthex(dd8[i]); }
    }
    serial_puts("\n");
    return cc8;
}

/* Свежий ресет порта ВНУТРИ перечисления (oh_wedge_reset): устройство
 * возвращается в Default-состояние - адрес 0, конфигурация 0, настройки
 * BIOS забыты.  Нужно, когда устройство ОТВЕЧАЕТ на addr-0
 * GET_DESCRIPTOR, но НЕ дескриптором (фейк-OK): в режиме sticklog BIOS
 * успевал адресовать/настроить клавиатуру до передачи управления ОС
 * («BIOS-эмуляция Клавы»), а настроенное устройство отвечает на
 * addr-0 GET_DESCRIPTOR(8) своим interrupt-ОТЧЁТОМ (тоже 8 байт!), а не
 * дескриптором - «ДАННЫЕ НЕ ДЕСКРИПТОР» на живом железе при исправном
 * коде передачи.  Ресет снимает это состояние. */
static void oh_wedge_reset(int p) {
    oh_w32(oh_port(p), 0x10u);              /* SetPortReset                */
    int t = 600;
    while ((oh_r32(oh_port(p)) & 0x10u) && t-- > 0) hw_delay(1000);
    oh_w32(oh_port(p), 0x100000u);          /* очистить PRSC               */
    uint32_t v = oh_r32(oh_port(p));
    if (!(v & 1u)) return;                  /* отвалился при сбросе        */
    if (!(v & 2u)) oh_w32(oh_port(p), 2u);  /* SetPortEnable, если не сам  */
    hw_delay(100000u);                      /* 100 мс recovery (LS-клава!) */
}

/* Перечисление HID-устройств на OHCI (устройство уже после сброса порта).
 * Возвращает: 0 - ничего, 1 - мышь (g_oh_active=1), 2 - только клавиатура.
 * адрес из счётчика (мышь и клавиатура - РАЗНЫЕ устройства),
 * GET_DESCRIPTOR с повторами (реальное устройство может не ответить
 * сразу после ресета - QEMU прощал, железо нет). */
static int ohci_enumerate(int ls, int port) {
    /* ЭП-таблица: порт уже знает это устройство - начинаем с
     * ПРОВЕРЕННОЙ скорости, а не с бита LSDA регистра.  Мусорную запись
     * (СМАЗ затёр statics) сначала обеззараживаем - иначе доверимся
     * чужой скорости и снова получим вечный cc=5. */
    if (port >= 1 && port <= 15 && g_oh_memo[port].used && oh_memo_sane(port)) {
        ls = g_oh_memo[port].ls;
        char nb[8];
        serial_puts("MOUSEHW: OHCI порт ");
        serial_puts(itoa((uint32_t)port, nb, 10));
        serial_puts(": скорость по ЭП-таблице (");
        serial_puts(ls ? "low-speed" : "full-speed");
        serial_puts(")\n");
    }
    g_oh_ls   = ls;
    /* «ЭТАП 3»: EP0 max packet — с 8 (низкоскоростной/безопасный дефолт),
     * после первого дескриптора уточняется из bMaxPacketSize0 (dd8[7]).
     * Раньше всегда 8: работало, но медленно для FS-устройств с 64. */
    g_oh_ep0_mps = 8;
    /* масштаб планшета - с чистого листа (см. hw_enumerate) */
    g_log_max = 32767;
    /* старт на адресе 0; если контроллер УЖЕ занят (мышь/клава
     * живут) - выдаём новичку уникальный адрес СРАЗУ, пока он один на
     * нуле (жители ушли с нуля в oh_relocate_residents ещё ДО ресета
     * новичка).  Два устройства на адресе 0 слышали бы чужие control-
     * передачи - «каша» из Симптома 2.  Одиночное устройство живёт на 0
 * как в - проверенный путь не тронут. */
    g_oh_addr = 0;
    if (g_oh_active || g_oh_kbd) {
        uint8_t na = g_oh_next_addr;
        g_oh_next_addr = (uint8_t)(na >= 127 ? 1 : na + 1);
        if (oh_ctrl(0x00, USB_REQ_SET_ADDRESS, na, 0, 0, 0) == TD_CC_OK) {
            g_oh_addr = na;
            char nb[8];
            serial_puts("MOUSEHW: HID: новичку выдан адрес ");
            serial_puts(itoa((uint32_t)na, nb, 10));
            serial_puts("\n");
        } else {
            serial_puts("MOUSEHW: SET_ADDRESS не вышел - работаем на адресе 0\n");
        }
    }

    /* Step 0: GET_DESCRIPTOR(8) — проверяем, что девайс вообще отвечает.
     * Если на скорости с порта ответа нет (3 попытки) - повтор на
     * ПРОТИВОПОЛОЖНОЙ: бит LSDA части контроллеров показывает неверно,
     * и устройство глухо на чужом сигналинге («ctrl cc=5» вечно).
     * Успех на чужой скорости - g_oh_ls остаётся перевёрнутым, при
     * связывании проверенная скорость уйдёт в ЭП-таблицу. */
    uint8_t *dd8 = oh_dd8_buf;    /* [#418] статический DMA-буфер (не стек!) */
    memset(dd8, 0, 8);
    int ok8 = 0, ls_was = ls, fake8 = 0;
    /* Круг A: скорость порта (LSDA/ЭП-таблица) - как раньше. */
    for (int att = 0; att < 3 && !ok8; att++) {
        int cc8 = oh_dd8_probe(dd8, port, att, &fake8);
        if (cc8 == TD_CC_OK && !fake8) ok8 = 1;
        if (!ok8) hw_delay(g_oh_ls ? 100000u : 50000u);  /* LS 100 мс */
    }
    /* Круг B (НОВЫЙ, фото порт 4): устройство ОТВЕЧАЕТ, но не
     * дескриптором (фейк-OK) - скорость ПРАВИЛЬНАЯ, состояние
     * устройства НЕТ.  Свежий ресет возвращает его в Default
     * (адрес 0): BIOS в sticklog-режиме успевал адресовать/настроить
     * клавиатуру до передачи управления ОС, и настроенное устройство
     * отвечает на addr-0 GET_DESCRIPTOR(8) interrupt-ОТЧЁТОМ (тоже
     * 8 байт), а не дескриптором.  После ресета g_oh_addr=0. */
    if (!ok8 && fake8) {
        char nb[8];
        serial_puts("MOUSEHW: OHCI порт ");
        serial_puts(itoa((uint32_t)port, nb, 10));
        serial_puts(": устройство отвечает НЕ дескриптором - свежий ресет порта (BIOS мог успеть его настроить)\n");
        oh_wedge_reset(port);
        g_oh_addr = 0;                    /* ресет вернул устройству адрес 0 */
        if (!(oh_r32(oh_port(port)) & 1u)) {
            oh_diag_set("порт ");
            oh_diag_set2(g_oh_diag, itoa((uint32_t)port, nb, 10));
            oh_diag_set2(g_oh_diag, ": устройство отвалилось при ресете");
            serial_puts("MOUSEHW: OHCI get device desc (8) failed\n");
            return 0;
        }
        for (int att = 0; att < 3 && !ok8; att++) {
            int cc8 = oh_dd8_probe(dd8, port, att, &fake8);
            if (cc8 == TD_CC_OK && !fake8) ok8 = 1;
            if (!ok8) hw_delay(g_oh_ls ? 100000u : 50000u);
        }
        if (ok8)
            serial_puts("MOUSEHW: после ресета устройство отвечает дескриптором - настройка BIOS снята\n");
    }
    if (!ok8) {
        int alt = ls_was ? 0 : 1;
        g_oh_ls = alt;
        serial_puts("MOUSEHW: OHCI: скорость не отвечает - пробую противоположную\n");
        for (int att = 0; att < 3 && !ok8; att++) {
            int cc8 = oh_dd8_probe(dd8, port, att, &fake8);
            if (cc8 == TD_CC_OK && !fake8) ok8 = 1;
            if (!ok8) hw_delay(50000u);
        }
        if (ok8) {
            char nb[8];
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puts(itoa((uint32_t)port, nb, 10));
            serial_puts(": устройство ответило на ");
            serial_puts(alt ? "low-speed" : "full-speed");
            serial_puts(" сигналинге - бит скорости порта показывал неверно, запоминаю\n");
        } else {
            g_oh_ls = ls_was;
        }
    }
    /* Круг D (НОВЫЙ, фото порт 4): SET_ADDRESS вслепую.  addr-0 не дал
     * дескриптора не только из-за скорости: BIOS оставлял устройство
     * АДРЕСОВАННЫМ/НАСТРОЕННЫМ (эмуляция клавы), и addr-0 трафик такое
     * устройство слышит криво (отвечает отчётом/мусором).  SET_ADDRESS
     * переводит его в НАШ адрес, GET_DESCRIPTOR по этому адресу
     * отвечает чисто.  На мёртвом устройстве SET_ADDRESS сам не пройдёт
     * (cc=5) - лишних попыток не добавляется. */
    if (!ok8) {
        uint8_t na = g_oh_next_addr;
        uint8_t sv_addr = g_oh_addr;
        char nb[8];
        g_oh_ls = ls_was;
        serial_puts("MOUSEHW: OHCI порт ");
        serial_puts(itoa((uint32_t)port, nb, 10));
        serial_puts(": addr-0 не дал дескриптора - SET_ADDRESS вслепую (");
        serial_puts(itoa((uint32_t)na, nb, 10));
        serial_puts("), затем GET_DESCRIPTOR по адресу\n");
        if (oh_ctrl(0x00, USB_REQ_SET_ADDRESS, na, 0, 0, 0) == TD_CC_OK) {
            g_oh_addr = na;
            for (int att = 0; att < 3 && !ok8; att++) {
                int cc8 = oh_dd8_probe(dd8, port, att, &fake8);
                if (cc8 == TD_CC_OK && !fake8) ok8 = 1;
                if (!ok8) hw_delay(g_oh_ls ? 100000u : 50000u);
            }
            if (ok8) {
                g_oh_next_addr = (uint8_t)(na >= 127 ? 1 : na + 1);
                serial_puts("MOUSEHW: устройство ответило ПОСЛЕ слепого SET_ADDRESS - BIOS оставляло его адресованным/настроенным\n");
            } else {
                g_oh_addr = sv_addr;
            }
        }
    }
    if (!ok8) {
        oh_diag_set("GET_DESCRIPTOR(8) не прошёл (addr 0)");
        serial_puts("MOUSEHW: OHCI get device desc (8) failed\n");
        return 0;
    }
    /* [#413] «тень адреса 0»: адрес выдаётся КАЖДОМУ новому устройству,
     * не только при живых мыши/клаве.  Раньше первое устройство и всё
     * «помеченное» (вебкамеры, картридеры, НЕИЗВЕСТНЫЕ) жили на адресе 0
     * ВЕЧНО: соседний addr-0 порт слышал их перечисление и отвечал на
     * ЧУЖИЕ токены («ДАННЫЕ НЕ ДЕСКРИПТОР»/cc=5 с фото пользователя по
     * портам 4/5), а после oh_unquiesce_restore оба снова сидели на нуле.
     * Теперь на шине не остаётся НИ ОДНОГО устройства с адресом 0. */
    if (g_oh_addr == 0) {
        uint8_t na = g_oh_next_addr;
        if (oh_ctrl(0x00, USB_REQ_SET_ADDRESS, na, 0, 0, 0) == TD_CC_OK) {
            g_oh_next_addr = (uint8_t)(na >= 127 ? 1 : na + 1);
            g_oh_addr = na;
            char nba[8];
            serial_puts("MOUSEHW: HID: устройству выдан адрес ");
            serial_puts(itoa((uint32_t)na, nba, 10));
            serial_puts(" (тень адреса 0 закрыта)\n");
        } else {
            serial_puts("MOUSEHW: SET_ADDRESS не вышел - работаю на адресе 0\n");
        }
    }
    serial_puts("MOUSEHW: dd8 =");
    for (int i = 0; i < 8; i++) { serial_puts(" "); serial_puthex(dd8[i]); }
    serial_puts("\n");
    /* «ЭТАП 3»: bMaxPacketSize0 (байт 7) — честный размер EP0.  Допустимы
     * только 8/16/32/64; мусор игнорируем (дефолт 8 уже стоит). */
    if (dd8[7] == 8 || dd8[7] == 16 || dd8[7] == 32 || dd8[7] == 64) {
        g_oh_ep0_mps = dd8[7];
        serial_puts("MOUSEHW: OHCI: EP0 MPS=");
        char nbm[8];
        serial_puts(itoa((uint32_t)dd8[7], nbm, 10));
        serial_puts(" (из bMaxPacketSize0)\n");
    }

    /* Step 1: GET_DESCRIPTOR(Device, 18) на том же адресе 0 */
    uint8_t *dd = oh_dd_buf;      /* [#418] статический DMA-буфер */
    int ok = 0;
    for (int att = 0; att < 3 && !ok; att++) {
        if (oh_ctrl(0x80, 0x06, (0x01 << 8), 0, 18, dd) == TD_CC_OK) ok = 1;
        else hw_delay(g_oh_ls ? 100000u : 20000u);  /* LS 100 мс */
    }
    if (ok && !(dd[0] == 0x12 && dd[1] == 0x01)) {
        serial_puts("MOUSEHW: OHCI get device desc: cc=0, но данные НЕ дескриптор (фейк-OK) - байты:");
        for (int i = 0; i < 18; i++) { serial_puts(" "); serial_puthex(dd[i]); }
        serial_puts("\n");
        ok = 0;
    }
    if (!ok) {
        char nb[8];
        oh_diag_set("GET_DESCRIPTOR(18) не прошёл (порт ");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)port, nb, 10));
        oh_diag_set2(g_oh_diag, ")");
        serial_puts("MOUSEHW: OHCI get device desc failed\n");
        return 0;
    }
    uint16_t vid = (uint16_t)(dd[8] | ((uint16_t)dd[9] << 8));
    uint16_t pid = (uint16_t)(dd[10] | ((uint16_t)dd[11] << 8));
    usb_db_report(vid, pid);

    /* Быстрый путь ЭП-таблицы: в ЭТОМ порту уже жил тот же vid/pid -
     * вид устройства и его эндпоинты известны, полный перебор
     * дескрипторов не нужен: сразу конфигурация и связывание. */
    if (port >= 1 && port <= 15 && g_oh_memo[port].used &&
        !g_oh_memo[port].both &&
        (g_oh_memo[port].role == UD_MOUSE || g_oh_memo[port].role == UD_KBD) &&
        g_oh_memo[port].vid == vid && g_oh_memo[port].pid == pid) {
        oh_memo_t *mm = &g_oh_memo[port];
        if (oh_ctrl(0x00, 0x09, mm->cfgv, 0, 0, 0) == TD_CC_OK) {
            oh_diag_set("OHCI: устройство опознано по ЭП-таблице");
            serial_puts("MOUSEHW: устройство опознано по ЭП-таблице (тот же vid/pid на этом порту) - быстрый путь\n");
            /* 0.3.0: даже по ЭП-памяти решение принимает диспетчер */
            int ver = usbdev_port_version(0x10);
            int role = usbdev_try_bind(ver, mm->role == UD_KBD,
                                       mm->role == UD_MOUSE,
                                       ohci_bus_label(), (int)port, vid, pid);
            if (role == USBDEV_ROLE_MOUSE) {
                g_iface      = mm->iface;
                g_mps        = mm->mps;
                g_oh_ms_ep   = mm->ep;
                g_boot_iface = true;
                g_absolute   = false;
                g_rpt_ok     = false;
                oh_bind_mouse(port, vid, pid, mm->cfgv, 1);
                return 1;
            }
            if (role == USBDEV_ROLE_KBD) {
                g_oh_kbd_iface = mm->iface;
                g_oh_kbd_mps   = mm->mps;
                g_oh_kbd_ep    = mm->ep;
                oh_bind_kbd(port, vid, pid, mm->cfgv, 1);
                return 2;
            }
            return 0;   /* слоты заняты: НЕИЗВЕСТНОЕ УСТРОЙСТВО */
        }
        serial_puts("MOUSEHW: SET_CONFIG по ЭП-таблице не вышел - полный перебор\n");
    }

    /* Step 2: GET_DESCRIPTOR(Configuration) — 9 байт */
    uint8_t *cfg9 = oh_cfg9_buf;  /* [#418] статический DMA-буфер */
    ok = 0;
    for (int att = 0; att < 2 && !ok; att++) {
        if (oh_ctrl(0x80, 0x06, (0x02 << 8), 0, 9, cfg9) == TD_CC_OK) ok = 1;
        else hw_delay(g_oh_ls ? 100000u : 20000u);  /* LS 100 мс */
    }
    if (ok && !(cfg9[0] == 9 && cfg9[1] == 0x02u)) {
        serial_puts("MOUSEHW: OHCI get cfg9: cc=0, но заголовок НЕ конфигурации (фейк-OK)\n");
        ok = 0;
    }
    if (!ok) {
        char nb[8];
        oh_diag_set("GET_DESCRIPTOR(конфиг 9Б) не прошёл (порт ");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)port, nb, 10));
        oh_diag_set2(g_oh_diag, ")");
        serial_puts("MOUSEHW: OHCI get cfg9 failed\n");
        return 0;
    }

    /* Step 3: GET_DESCRIPTOR(Configuration) — полный */
    uint16_t total = (uint16_t)(cfg9[2] | ((uint16_t)cfg9[3] << 8));
    if (total > 255) total = 255;
    uint8_t *cfg = oh_cfg_buf;    /* [#418] статический DMA-буфер, не стек */
    ok = 0;
    for (int att = 0; att < 2 && !ok; att++) {
        if (oh_ctrl(0x80, 0x06, (0x02 << 8), 0, total, cfg) == TD_CC_OK) ok = 1;
        else hw_delay(g_oh_ls ? 100000u : 20000u);  /* LS 100 мс */
    }
    if (ok && !(cfg[0] == 9 && cfg[1] == 0x02u)) {
        serial_puts("MOUSEHW: OHCI get config: cc=0, но заголовок НЕ конфигурации (фейк-OK) - байты:");
        for (int i = 0; i < 8; i++) { serial_puts(" "); serial_puthex(cfg[i]); }
        serial_puts("\n");
        ok = 0;
    }
    if (!ok) {
        char nb[8];
        oh_diag_set("GET_DESCRIPTOR(конфиг) не прошёл (порт ");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)port, nb, 10));
        oh_diag_set2(g_oh_diag, ")");
        serial_puts("MOUSEHW: OHCI get config failed\n");
        return 0;
    }

    /* Step 4: разбор интерфейсов */
    bool has_mouse = false, has_kbd = false, has_msd = false, has_ambig = false;
    if (!oh_parse_device(cfg, total, &has_mouse, &has_kbd, &has_msd,
                         &has_ambig)) {
        char nb[8];
        oh_diag_set("интерфейсов HID/MSD нет (порт ");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)port, nb, 10));
        oh_diag_set2(g_oh_diag, ", vid=");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)vid, nb, 16));
        oh_diag_set2(g_oh_diag, " pid=");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)pid, nb, 16));
        oh_diag_set2(g_oh_diag, ")");
        serial_puts("MOUSEHW: OHCI: HID/MSD-интерфейсов нет\n");
        return 0;
    }

    /* Step 5: SET_CONFIGURATION */
    if (oh_ctrl(0x00, 0x09, cfg9[5], 0, 0, 0) != TD_CC_OK) {
        char nb[8];
        oh_diag_set("SET CONFIG не прошёл (порт ");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)port, nb, 10));
        oh_diag_set2(g_oh_diag, ")");
        serial_puts("MOUSEHW: OHCI set config failed\n");
        return 0;
    }

    /* Step 5b: непонятный HID (protocol 0) - цепочка уровней:
     * L2 usage-скан -> L3 LED-проба -> L4 (неоднозначно = КЛАВА).
     * Составное устройство (мышь по protocol 2 уже найдена) даёт ОБА
     * драйвера (composite).  Мышью «непонятного» назначает только L2. */
    if (has_ambig && !has_kbd) {
        const char *via = "";
        int vd = oh_ask_verdict(g_oh_amb_iface, cfg, total, &via);
        /* «неоднозначно» больше НЕ клавиатура (фото: фантомные «клавы»
         * на 2/3/5 портах - вебкам/картридер с HID protocol 0 не
         * проходили LED-пробу и связывались клавой).  Клавиатурой
         * неоднозначное становится ТОЛЬКО с boot-subclass
         * (bInterfaceSubClass=1: устройство само называет себя
         * загрузочным HID).  Остальное - честное «помечено». */
        if (vd == VERD_UNKNOWN && g_oh_amb_sub == 1u) {
            vd = VERD_KBD;  via = "boot-subclass";
        }
        if (vd == VERD_UNKNOWN) {
            char nb[8];
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puts(itoa((uint32_t)port, nb, 10));
            serial_puts(": HID не доказал вид (LED-проба молчит, subclass=");
            serial_puts(itoa((uint32_t)g_oh_amb_sub, nb, 10));
            serial_puts(") - помечено, НЕ клавиатура\n");
            usbdev_register(1, (int)port, vid, pid, UD_MARKED, -1, 0, 0);
        } else if (vd == VERD_KBD) {
            g_oh_kbd_iface = g_oh_amb_iface;
            g_oh_kbd_ep    = g_oh_amb_ep;
            g_oh_kbd_mps   = g_oh_amb_mps;
            has_kbd = true;
            hid_report_line(g_oh_ls ? "1.x" : "2.0", port, (int)g_oh_addr,
                            "keyboard", via, (int)g_oh_amb_iface);
        } else if (!has_mouse) {
            g_iface      = g_oh_amb_iface;
            g_oh_ms_ep   = g_oh_amb_ep;
            g_mps        = g_oh_amb_mps;
            g_boot_iface = false;
            has_mouse = true;
            hid_report_line(g_oh_ls ? "1.x" : "2.0", port, (int)g_oh_addr,
                            "mouse", via, (int)g_oh_amb_iface);
        }
    }

    /* Step 6 (0.3.0): «порт, кто ты?» + подбор драйвера из ШЕСТИ.
     * OHCI-порты отвечают «USB 2.0», если на машине есть EHCI (порт
     * отдан компаньону), иначе «USB 1.x».  Клавиатура пробуется ПЕРВОЙ,
     * реестр ведёт usbdev. */
    {
        int ver  = usbdev_port_version(0x10);
        int role = usbdev_try_bind(ver, has_kbd, has_mouse,
                                   ohci_bus_label(), (int)port, vid, pid);

        /* флешка (MSD/BOT) — независимо от роли ввода */
        if (has_msd) {
            g_oh_msd_port = port;
            g_oh_msd_oi   = g_oi;
            oh_msd_init();
        }

        if (role == USBDEV_ROLE_MOUSE) {
            g_absolute = oh_detect_mode(cfg, total);
            oh_bind_mouse(port, vid, pid, has_msd ? 0 : cfg9[5], 0);
            /* составное: клавиатурный интерфейс того же корпуса —
             * второй драйвер, если слот клавиатуры ещё свободен */
            if (has_kbd && !usbdev_kbd_bound() &&
                usbdev_try_bind(ver, 1, 0, ohci_bus_label(),
                                (int)port, vid, pid) == USBDEV_ROLE_KBD) {
                oh_bind_kbd(port, vid, pid, has_msd ? 0 : cfg9[5], 0);
                if (port >= 1 && port <= 15) g_oh_memo[port].both = 1;
            }
            return 1;
        }
        if (role == USBDEV_ROLE_KBD) {
            oh_bind_kbd(port, vid, pid, has_msd ? 0 : cfg9[5], 0);
            if (has_mouse && !usbdev_mouse_bound() &&
                usbdev_try_bind(ver, 0, 1, ohci_bus_label(),
                                (int)port, vid, pid) == USBDEV_ROLE_MOUSE) {
                g_absolute = oh_detect_mode(cfg, total);
                oh_bind_mouse(port, vid, pid, has_msd ? 0 : cfg9[5], 0);
                if (port >= 1 && port <= 15) g_oh_memo[port].both = 1;
            }
            return 2;
        }
        /* ни один из шести драйверов не подошёл / оба слота заняты */
        serial_puts("MOUSEHW: OHCI порт ");
        {
            char nb[8];
            serial_puts(itoa((uint32_t)port, nb, 10));
        }
        serial_puts(": НЕИЗВЕСТНОЕ УСТРОЙСТВО (см. usbdev выше)\n");
        usbdev_register(1, (int)port, vid, pid, UD_MARKED, -1, 0, 0);
        return 0;
    }
}

/* SET_ADDRESS устройству, которое СЕЙЧАС стоит на адресе 0.
 * oh_ctrl адресует через ED_FA(g_oh_addr) - временно переключаем глобалы
 * на «жертву» и возвращаем. */
static bool oh_addr0_to(uint8_t new_addr, int dev_ls) {
    uint8_t sv_a = g_oh_addr;
    int     sv_l = g_oh_ls;
    bool    ok;
    g_oh_addr = 0;
    g_oh_ls   = dev_ls;
    ok = oh_ctrl(0x00, USB_REQ_SET_ADDRESS, new_addr, 0, 0, 0) == TD_CC_OK;
    g_oh_addr = sv_a;
    g_oh_ls   = sv_l;
    return ok;
}

/* уводим жителей с адреса 0 ПЕРЕД ресетом новичка (без ресета
 * устройство ещё молчит - адрес 0 свободен на время переезда).  Два
 * устройства на нуле слышали бы чужие control-передачи - «каша» из
 * Симптома 2 инструкции. */
static void oh_relocate_residents(void) {
    char nb[8];
    if (g_oh_active && g_oh_ms_usb_addr == 0) {
        uint8_t a = g_oh_next_addr;
        g_oh_next_addr = (uint8_t)(a >= 127 ? 1 : a + 1);
        if (oh_addr0_to(a, g_oh_ms_ls)) {
            g_oh_ms_usb_addr = a;
            oh_ed_intr[0] = (oh_ed_intr[0] & ~0x7Fu) | ED_FA(a);
            dma_sync();
            serial_puts("MOUSEHW: HID: мышь ушла с адреса 0 на ");
            serial_puts(itoa((uint32_t)a, nb, 10));
            serial_puts("\n");
        }
    }
    if (g_oh_kbd && g_oh_kbd_usb_addr == 0) {
        uint8_t a = g_oh_next_addr;
        g_oh_next_addr = (uint8_t)(a >= 127 ? 1 : a + 1);
        if (oh_addr0_to(a, g_oh_kbd_ls)) {
            g_oh_kbd_usb_addr = a;
            oh_ed_kbd[0] = (oh_ed_kbd[0] & ~0x7Fu) | ED_FA(a);
            dma_sync();
            serial_puts("MOUSEHW: HID: клавиатура ушла с адреса 0 на ");
            serial_puts(itoa((uint32_t)a, nb, 10));
            serial_puts("\n");
        }
    }
}

/* ── Сериализация addr-0 и вечный рескан ────────────────────────────────────
 * Случай с фото: p4=0x303 p5=0x303 (ОБА устройства подключены и включены
 * BIOS'ом), cc: setup=5 - они ОБА сидят на адресе 0 и глушат ответы
 * друг друга; после первой неудачи CSC больше не приходит, и скан не
 * повторяется НИКОГДА.  Решение - классическая сериализация шины:
 *  1) на время addr-0 перечисления чужие неперечисленные порты
 *     притушаются (ClearPortEnable);
 *  2) порты с устройством, но без заявки, перечисляются заново БЕЗ CSC
 *     (ohci_rescan_unclaimed); после 3 неудач порт отстаёт до
 *     переподключения.
 * Заявленные порты (мышь/клава/флешка) не трогаются НИКОГДА - путь
 * мыши и логика не меняются. */

/* порт занят перечисленным устройством?  [#416] Сверка ИНСТАНСА: порты
 * нумеруются ВНУТРИ контроллера — «порт 2 OHCI#2» не имеет отношения к
 * мыши на «порту 2 OHCI#1».  Раньше совпадение одного номера порта
 * объявляло порт соседнего контроллера «занятым» — он вечно скипался
 * сканом и не притушивался при addr-0 перечислении соседа. */
static bool oh_port_claimed(int q) {
    if (g_oh_active && g_oh_ms_oi == g_oi && q == (int)g_oh_ms_port)  return true;
    if (g_oh_kbd    && g_oh_kb_oi == g_oi && q == (int)g_oh_kbd_port) return true;
    if (g_oh_msd    && g_oh_msd_oi == g_oi && q == (int)g_oh_msd_port) return true;
    return false;
}

/* [#409] битовка портов, притушенных oh_quiesce_others текущего
 * перечисления (возврат — oh_unquiesce_restore) */
static uint16_t g_oh_quench_a[OHCI_NINST];
#define g_oh_quench (g_oh_quench_a[g_oi])

/* притушить чужие неперечисленные порты - addr-0 трафик обязан
 * слышать РОВНО одно устройство (два на адресе 0 отвечают в один
 * провод - «каша» Симптома 2, фото: setup=5). ClearPortEnable у
 * OHCI = запись 1 в бит CCS (сверено с Linux ohci-hub.c:
 * ClearPortFeature(ENABLE) пишет RH_PS_CCS, и с QEMU
 * ohci_port_set_status: val & CCS -> ctrl &= ~PES).  Обратное включение
 * - ресетом при перечислении самого порта (ресет ставит PES сам). */
static void oh_quiesce_others(int keep) {
    g_oh_quench = 0;                       /* [#409] список притушённых  */
    for (int q = 1; q <= (int)g_oh_nports; q++) {
        if (q == keep || oh_port_claimed(q)) continue;
        uint32_t ps = oh_r32(oh_port(q));
        if ((ps & 1u) && (ps & 2u)) {
            oh_w32(oh_port(q), 0x1u);      /* ClearPortEnable            */
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puthex((uint32_t)q);
            serial_puts(": притушён (на время addr-0 перечисления)\n");
            if (q <= 15) g_oh_quench |= (uint16_t)(1u << q);
        }
    }
}

/* [#409/#234-restore] ВЕРНУТЬ портам жизнь после addr-0 перечисления:
 * SetPortEnable (запись 2 в RH_PS). Раньше притушенный сосед оставался
 * выключенным НАВСЕГДА: его устройство сидит на CCS=1/PES=0, нового CSC
 * не приходит, рескан считает «не заявлено» и бесконечно перечисляет —
 * фото №2: «порт 4 притушён» → вечные «Данные НЕ ДЕСКРИПТОР»/cc=5 по
 * соседу, перечисляемому СЛЕДОМ. Возврат без ресета устройства. */
static void oh_unquiesce_restore(void) {
    if (!g_oh_quench) return;
    for (int q = 1; q <= 15; q++) {
        if (!(g_oh_quench & (uint16_t)(1u << q))) continue;
        uint32_t ps = oh_r32(oh_port(q));
        if ((ps & 1u) && !(ps & 2u)) {
            oh_w32(oh_port(q), 2u);          /* SetPortEnable            */
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puthex((uint32_t)q);
            serial_puts(": возвращен из притушения\n");
        }
    }
    g_oh_quench = 0;
    hw_delay(2000);
}

/* счёт неудачных попыток перечисления по портам (BSS = нули) */
static uint8_t g_oh_fail_a[OHCI_NINST][16];
#define g_oh_fail (g_oh_fail_a[g_oi])

/* Смена состояния порта OHCI: подключение/отключение/сброс/перечисление.
 * force=1 — обработать CCS даже без CSC (первичный скан после сброса HC). */
static void ohci_probe_port(int p, int force) {
    /* [#471] ЭТАП 9: счётчики портов (g_oh_fail/g_oh_slow/g_oh_memo)
     * размером [16] — порт вне 1..15 означал запись ЗА ГРАНИЦУ на редких
     * OHCI с >15 портами (карта и так печатает только 15). */
    if (p < 1 || p > 15) return;
    uint32_t ps = oh_r32(oh_port(p));
    if ((ps & 0x10000u) == 0 && !force) return;      /* нет CSC            */
    if (ps & 0x10000u) {                             /* W1C: очистить CSC  */
        oh_w32(oh_port(p), 0x10000u);
        g_oh_fail[p] = 0;   /* событие на порту - снова даём попытки */
        g_oh_slow[p] = 0;   /* и фоновый счётчик ЭП-таблицы */
    }
    ps = oh_r32(oh_port(p));
    /* «ЭТАП 3»: дебаунс подключений (Linux hub.c: ~100 мс стабильности).
     * Контакты при втыкании дребезгат: ресет по первому CSC ловил
     * «полуподключение» и перечисление срывалось на пустом месте.
     * Ждём 100 мс и перечитываем — дребезг отсеивается, стабильное
     * подключение остаётся.  На отключения не распространяется. */
    if (ps & 1u) {
        hw_delay(100000u);                            /* 100 мс            */
        uint32_t ps2 = oh_r32(oh_port(p));
        if (!(ps2 & 1u)) {
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puthex((uint32_t)p);
            serial_puts(": дребезг контактов (CCS пропал за 100 мс) - игнорирую\n");
            return;
        }
        ps = ps2;
    }
    if (!(ps & 1u)) {                                /* CCS=0 — отключение */
        /* у мыши СВОЙ порт (g_oh_ms_port): выдёргивание флешки из
         * соседнего порта больше не убивает работающую мышь. */
        if (g_oh_active && g_oh_ms_oi == g_oi && p == g_oh_ms_port) {
            serial_puts("MOUSEHW: USB: устройство отключено (OHCI порт ");
            serial_puthex((uint32_t)p);
            serial_puts(")\n");
            oh_ed_intr[0] |= ED_SKIP;
            g_oh_active = 0;
            g_present   = false;
            g_active    = false;
            g_oh_ms_usb_addr = 0;   /* адрес свободен для следующего */
            g_oh_ms_oi = -1;         /* 0.3.0: инстанс свободен */
            lm_drop();               /* [#474] снимок мыши погашен */
            usbdev_release(USBDEV_ROLE_MOUSE, "detach");
            usbdev_unregister(1, p, "detach");
        }
        if (g_oh_kbd && g_oh_kb_oi == g_oi && p == g_oh_kbd_port) {
            serial_puts("MOUSEHW: USB-клавиатура отключена (OHCI порт ");
            serial_puthex((uint32_t)p);
            serial_puts(")\n");
            oh_ed_kbd[0] |= ED_SKIP;
            g_oh_kbd = 0;
            g_oh_kbd_usb_addr = 0;  /* адрес свободен для следующего */
            g_oh_kb_oi = -1;         /* 0.3.0: инстанс свободен */
            lk_drop();               /* [#474] снимок клавы погашен */
            oh_kbd_state_reset();    /* [#476] prev_k чист для следующей клавы */
            usbdev_release(USBDEV_ROLE_KBD, "kbd detach");
            usbdev_unregister(1, p, "kbd detach");
            ps2_kbd_mods(0);      /* модификаторы не висят до след. подкл. */
            ps2_kbd_detached();   /* её зажатый Enter не залипает навсегда */
        }
        oh_hcca_rebuild();                   /* состав устройств изменился */
        return;
    }
    serial_puts("MOUSEHW: обнаружено новое устройство! (OHCI порт ");
    serial_puthex((uint32_t)p);
    serial_puts(")\n");

    /* сначала притушаем чужие неперечисленные порты - addr-0
     * трафик (SET_ADDRESS жителям ниже, затем новичку) обязан слышать
 * РОВНО одно устройство (фото: два на адресе 0 = setup=5). */
    oh_quiesce_others(p);

    /* уводим жителей с адреса 0 - новичок ещё не прошёл ресет и
     * молчит, адрес свободен для переезда. */
    oh_relocate_residents();

    oh_w32(oh_port(p), 0x10u);            /* SetPortReset                  */
    int t = 600;
    while ((oh_r32(oh_port(p)) & 0x10u) && t-- > 0) hw_delay(1000);
    oh_w32(oh_port(p), 0x100000u);        /* очистить PRSC                 */
    uint32_t v = oh_r32(oh_port(p));
    serial_puts("MOUSEHW: OHCI порт ");
    serial_puthex((uint32_t)p);
    serial_puts(": после сброса raw=");
    serial_puthex(v);
    serial_puts("\n");
    if (!(v & 1u)) {
        char nb[8];
        oh_diag_set("порт ");
        oh_diag_set2(g_oh_diag, itoa((uint32_t)p, nb, 10));
        oh_diag_set2(g_oh_diag, ": устройство отвалилось при сбросе");
        oh_unquiesce_restore();            /* [#409] соседи — назад в строй */
        return;                           /* отвалился во время сброса     */
    }
    if (!(v & 2u)) oh_w32(oh_port(p), 2u);/* SetPortEnable, если не сам    */
    /* recovery после ресета. LS-устройствам (клава/мышь) QEMU-ских
     * 10 мс мало: реальный LS девайс ещё не готов слушать (Linux ждёт
     * 100+ мс).  LS = LSDA БИТ 9 (RH_PS_LSDA=0x200, сверено с
     * drivers/usb/host/ohci.h ядра Linux: PPS=0x100, LSDA=0x200).
     * Раньше читали бит 6 (0x40) - его НИКОГДА нет в регистре, поэтому
     * low-speed мышь начинали на full-speed сигналинге и она молчала
     * («ctrl cc=5» вечно).  FS тоже даём 50 мс - часть устройств не
     * готова отвечать через 10 мс после ресета.  Для порта из
     * ЭП-таблицы скорость известна точно. */
    {
        int memo_ls = 0;
        if (p >= 1 && p <= 15 && g_oh_memo[p].used) {
            oh_memo_sane(p);
            memo_ls = g_oh_memo[p].used;   /* sane мог погасить мусор */
        }
        int lsg = memo_ls ? g_oh_memo[p].ls : (int)((v & 0x200u) ? 1 : 0);
        hw_delay(lsg ? 100000u : 50000u);
        if (!memo_ls && (v & 0x200u)) {
            char nb2[8];
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puts(itoa((uint32_t)p, nb2, 10));
            serial_puts(": устройство low-speed (LSDA бит 9)\n");
        }
    }
    ohci_enumerate((v & 0x200u) ? 1 : 0, p); /* LSDA = БИТ 9 (0x200);
                                              * ЭП-таблица поправит скорость */
    oh_unquiesce_restore();                /* [#409] соседи — назад в строй */
}

/* вечный рескан - «пусть сканирует даже если мышь не нашёл».
 * Порты, где устройство ЕСТЬ (CCS=1), но оно никем не заявлено,
 * перечисляются заново БЕЗ всякого CSC: после первой неудачи CSC не
 * приходит (фото показало dev=0 навсегда при живых p4/p5). 3
 * неудачи - порт отстаёт до переподключения, чтобы не сбрасывать
 * живое устройство бесконечно.  Порт с записью ЭП-таблицы (здесь уже
 * жило опознанное устройство) не отстаёт совсем: фоновая попытка раз
 * в ~4 секунды.  Вызывается из mouse_hw_poll раз в ~64 тика
 * (примерно раз в секунду опроса). */
static void ohci_rescan_unclaimed(void) {
    for (int p = 1; p <= (int)g_oh_nports; p++) {
        if (oh_port_claimed(p)) continue;
        uint32_t ps = oh_r32(oh_port(p));
        if (!(ps & 1u)) { g_oh_fail[p] = 0; continue; }   /* порт пуст   */
        if (g_oh_fail[p] >= 3) {
            /* Порт с записью ЭП-таблицы (здесь уже жило опознанное
             * устройство) не бросается: фоновая попытка раз в ~4
             * секунды, до 12 счёта - беспроводная мышь просыпается
             * сама, без перетыкания.  Порты без памяти отстают сразу. */
            if (!g_oh_memo[p].used || g_oh_fail[p] >= 12) continue;
            g_oh_slow[p]++;
            if ((g_oh_slow[p] & 3u) != 1u) continue;
            serial_puts("MOUSEHW: OHCI порт ");
            serial_puthex((uint32_t)p);
            serial_puts(": устройство из ЭП-таблицы молчит - фоновая попытка\n");
        }
        serial_puts("MOUSEHW: OHCI порт ");
        serial_puthex((uint32_t)p);
        serial_puts(": устройство на порту не заявлено - перечисляю снова\n");
        ohci_probe_port(p, 1);
        if (oh_port_claimed(p)) {
            oh_diag_set("OHCI: устройство перечислено ресканом");
            g_oh_fail[p] = 0;
            g_oh_slow[p] = 0;
        } else if (++g_oh_fail[p] >= 3) {
            if (g_oh_fail[p] == 3) {   /* для ЭП-портов дальше идут фоновые
                                        * попытки - спамить текстлог нельзя */
                char nb[8];
                serial_puts("MOUSEHW: OHCI порт ");
                serial_puthex((uint32_t)p);
                serial_puts(": не перечислился за 3 попытки - жду переподключения\n");
                oh_diag_set2("OHCI: порт ", itoa((uint32_t)p, nb, 10));
                oh_diag_set2(g_oh_diag, " не отвечает (рескан)");
            }
        }
    }
}

static int probe_ohci(int oi, uint8_t bus, uint8_t dev, uint8_t fn) {
    if (oi < 0 || oi >= OHCI_NINST) return 0;   /* [#415] инстансов теперь 4 */
    g_oi = oi;   /* все алиасы OHCI — на ЭТОТ контроллер */
    g_oh_probed = 1;   /* СНИМОК OHCI печатается только после реального
 * скана - иначе мусор (фото) */
    serial_puts("MOUSEHW: OHCI at ");
    serial_puthex(bus); serial_puts(":");
    serial_puthex(dev); serial_puts(".");
    serial_puthex(fn); serial_puts("\n");

    uint32_t bar = pci_read32(bus, dev, fn, 0x10);
    if (bar & 1u) { serial_puts("MOUSEHW: OHCI BAR0 is I/O, skip\n"); return 0; }
    g_oh_mmio = bar & ~0xFu;
    if (g_oh_mmio < 0x1000u) {
        g_oh_mmio = 0;
        oh_diag_set("OHCI: плохой BAR0 - пропущен");
        serial_puts("MOUSEHW: OHCI bad BAR0\n");
        return 0;
    }
    uint32_t cmd = pci_read32(bus, dev, fn, 0x04);
    pci_write32(bus, dev, fn, 0x04, cmd | 0x6u);

    /* «ЭТАП 3»: валидация HcRevision (спека OHCI 1.0 = 0x10) — MMIO
     * может оказаться чужим устройством с совпавшим prog-if. */
    {
        uint32_t rev = oh_r32(0x00u);
        if (((rev >> 16) & 0xFFu) != 0x10u && (rev & 0xFFu) != 0x10u) {
            char nb[8];
            serial_puts("MOUSEHW: OHCI: странный HcRevision=");
            serial_puthex(rev);
            serial_puts(" - всё равно пробую ( rev_hi=");
            serial_puts(itoa((rev >> 16) & 0xFFu, nb, 10));
            serial_puts(")\n");
        }
    }

    /* ── 0.3.0 (фото с реального железа): ВЗЯТИЕ ВЛАДЕНИЯ у SMM/BIOS ──
     * HCR-сброс НЕ останавливает SMM-обработчик «USB Legacy Support»:
     * BIOS продолжает поллить OHCI (эмуляция клавиатуры) ПОСЛЕ нашего
     * ресета, пишет поверх наших ED/TD («СМАЗ: control ED word0 =
     * 0x1000, MPS=0» — чужая запись), и КАЖДЫЙ SET_ADDRESS получает
     * cc=5 DeviceNotResponding, а low-speed устройство «отвечает»
     * нулями (фейк-OK). QEMU этого не эмулирует — там код работал.
     * Правильная процедура (OHCI 5.1.1.3, как в Linux ohci-hcd):
     *   1) HcControl.IR=1 → SMM владеет: включаем прерывание
     *      OwnershipChange, ставим HcCommandStatus.OCR и ждём до 2 с,
     *      пока SMM отдаст владение (IR→0);
     *   2) после (или если IR=0) глушим ВСЕ источники прерываний HC
     *      (HcInterruptDisable) — SMI от OHCI больше не генерятся —
     *      и только потом HCR. */
    {
        /* ФИКС «ЭТАП 3» (корень «порты 2.0 мертвы на реальном железе"):
         * до сих пор проверялся бит 2 (PLE) вместо бита 8 (IR=0x100),
         * а прерывание OwnershipChange включалось битом 3 (RD) вместо
         * бита 7 (OC=0x80).  На реальном AMD FCH с BIOS USB Legacy
         * (IR=1, PLE=0) проверка ЛОЖНО говорила «SMM не активен» —
         * handshake ПРОПУСКАЛСЯ, SMM оставался владельцем и затирал
         * наши ED/TD после HCR (фото: cc=5 DeviceNotResponding,
         * «control ED word0 = 0x1000»).  QEMU стартует с HcControl=0,
         * поэтому эмулятор эти баги не ловил.  Биты сверены с Linux
         * drivers/usb/host/ohci.h: OHCI_CTRL_IR = 1<<8, OHCI_INTR_OC = 1<<7. */
        uint32_t hcctl = oh_r32(OH_CTRL);
        serial_puts("MOUSEHW: OHCI: HcControl=");
        serial_puthex(hcctl);
        serial_puts("\n");
        if (hcctl & (1u << 8)) {              /* IR=1 (0x100): SMM владеет */
            serial_puts("MOUSEHW: OHCI: SMM владеет контроллером - запрашиваю передачу\n");
            oh_w32(OH_INTEN, (1u << 7) | OH_INT_MIE);   /* OC interrupt (0x80) */
            oh_w32(OH_CMDSTS, 1u << 3);                 /* OCR */
            int tw = 2000;
            while ((oh_r32(OH_CTRL) & (1u << 8)) && tw-- > 0)
                hw_delay(1000);
            if (oh_r32(OH_CTRL) & (1u << 8)) {
                /* «ЭТАП 2»: SMM не ответил на OCR — забираем ПРИНУДИТЕЛЬНО:
                 * HcControl.IR=0 перенаправляет события с SMI на обычные
                 * прерывания (которые мы всё равно держим замаскированными),
                 * SMM-обработчик BIOS больше не стреляет по контроллеру.
                 * HCR ниже сбрасывает поле в 0 окончательно. */
                serial_puts("MOUSEHW: OHCI: SMM не отдал владение за 2 с - забираю силой (IR=0)\n");
                uint32_t c = oh_r32(OH_CTRL);
                oh_w32(OH_CTRL, c & ~(1u << 8));
                hw_delay(20000);
                serial_puts("MOUSEHW: OHCI: после force-take HcControl=");
                serial_puthex(oh_r32(OH_CTRL));
                serial_puts("\n");
            } else {
                serial_puts("MOUSEHW: OHCI: SMM отдал владение ОС (handshake по спеке 5.1.1.3)\n");
            }
        } else {
            serial_puts("MOUSEHW: OHCI: SMM не активен (HcControl.IR=0)\n");
        }
        /* глушим прерывания/SMI-источники HC перед ресетом */
        oh_w32(0x14u /* HcInterruptDisable */, 0xFFFFFFFFu);
    }

    /* Сброс контроллера (владение уже наше - SMM больше не мешает) */
    oh_w32(OH_CMDSTS, OH_CMD_HCR);
    int t = 10000;
    while ((oh_r32(OH_CMDSTS) & OH_CMD_HCR) && t-- > 0) hw_delay(1);
    if (t <= 0) {
        oh_diag_set("OHCI: сброс контроллера не прошёл (timeout)");
        serial_puts("MOUSEHW: OHCI reset timeout, skip\n");
        g_oh_mmio = 0;
        return 0;
    }

    /* (ГЛАВНЫЙ фикс OHCI на реальном железе): после HCR-ресета
     * HcFmInterval на AMD FCH = 0 - кадровый движок не тикает и
     * control-список ВООБЩЕ не обрабатывается (в текстлоге это было
     * «OHCI ctrl TIMEOUT dh=0 ed_halted=0»).  QEMU прощал: в его
     * ohci_reset fi=0x2edf зашито.  u-boot пишет FmInterval явно -
     * делаем так же: FI=0x2EDF (1 мс @12МГц), FSMPS=0x2778. */
    oh_w32(OH_FMINT, (0x2778u << 16) | 0x2EDFu);
    /* HcLSThreshold - регистр, который HCR НЕ СБРАСЫВАЕТ (спека OHCI
 * 7.1). писала FmInterval, но забыла LSThreshold: BIOS мог
     * оставить 0, и HC не выпускает НИ ОДНОЙ low-speed транзакции - LS
     * клава/мышь (2.0-порты) молчат, SETUP получает cc=5
 * DeviceNotResponding (фото: p4/p5=0x303 - устройство ВКЛЮЧЕНО,
     * но не отвечает).  U-Boot/Linux пишут 0x628 (дефолт спеки) явно. */
    oh_w32(OH_LSTH, 0x0628u);

    /* [#408] NDP — ПОЛНЫЕ 8 бит RhDescriptorA[7:0] (спека OHCI 7.4.1,
 * Linux ohci-hub.c: num_ports = roothub_a & 0xFF... и клампится к 15
 * — больше 15 портов у рут-хаба не бывает по USB-спеке). Раньше
 * маска 0xF + кламп 8 прятали порты 9-15 (жалоба: «портов 12, ОС
 * видит 5»). Массивы ЭП-таблицы расширены до 16 (порты 1..15). */
    g_oh_nports = (uint8_t)(oh_r32(OH_RHDESCA) & 0xFFu);
    if (g_oh_nports == 0 || g_oh_nports > 15) g_oh_nports = 4;
    serial_puts("MOUSEHW: OHCI ports=");
    serial_puthex(g_oh_nports);
    serial_puts("\n");
    {
        char nb[8];
        oh_diag_set2("OHCI: контроллер найден, портов ",
                     itoa((uint32_t)g_oh_nports, nb, 10));
    }

    /* HCCA: все 32 записи периодической таблицы → interrupt ED (пока Skip) */
    memset(oh_hcca, 0, sizeof(oh_hcca));
    oh_ed_intr[0] = ED_SKIP;
    oh_ed_intr[1] = vphys(oh_td_tail_i);  /* TailP = dummy */
    oh_ed_intr[2] = 0;
    oh_ed_intr[3] = 0;
    for (int i = 0; i < 32; i++) ((uint32_t *)oh_hcca)[i] = vphys(oh_ed_intr);
    oh_w32(OH_HCCA, vphys(oh_hcca));
    oh_ed_ctrl[0] = ED_SKIP;
    oh_ed_ctrl[1] = vphys(oh_td_tail);    /* TailP = dummy */
    oh_ed_ctrl[2] = 0;
    oh_ed_ctrl[3] = 0;
    oh_w32(OH_CTRL_HEAD, vphys(oh_ed_ctrl));
    oh_ed_kbd[0]  = ED_SKIP;              /* клавиатуры пока нет */
    oh_ed_kbd[1]  = vphys(oh_td_tail_i);
    oh_ed_kbd[2]  = 0;
    oh_ed_kbd[3]  = 0;
    if (g_oh_kb_oi == oi) g_oh_kbd = 0;   /* живая клава ДРУГОГО инстанса
                                           * не трогаем (0.3.0) */
    g_oh_next_addr = 1;                   /* первый USB-адрес (инстанса) */
    g_kbd_pkts    = 0;

    /* Кадр 1 мс: PeriodicStart = 90% интервала (стандартная практика) */
    uint32_t fmi = oh_r32(OH_FMINT) & 0x3FFFu;
    oh_w32(OH_PERSTART, (fmi * 9u) / 10u);

    /* Прерывания к CPU не идут (PIC замаскирован), но WDH writeback
     * включить ОБЯЗАТЕЛЬНО — по нему контроллер пишет HccaDoneHead. */
    oh_w32(OH_INTEN, OH_INT_WDH | OH_INT_MIE);

    /* Питание портов.  «ЭТАП 3»: раньше ставился ТОЛЬКО SetGlobalPower —
     * на компаньоне с per-port power switching (PSM=1 в HcRhDescriptorA)
     * порты НЕ получали питание вообще (CCS не появлялся никогда —
     * Linux ohci-hub.c делает per-port SetPortPower при PSM=1).
     * Флаги NPS (питание всегда включено)/PSM/NOCP печатаем в журнал. */
    {
        uint32_t da = oh_r32(OH_RHDESCA);
        int nps = (da & (1u << 9)) != 0;
        int psm = (da & (1u << 8)) != 0;
        serial_puts("MOUSEHW: OHCI RhDescriptorA: NPS=");
        serial_puts(nps ? "1" : "0");
        serial_puts(" PSM=");
        serial_puts(psm ? "1" : "0");
        serial_puts(" NOCP=");
        serial_puts((da & (1u << 12)) ? "1" : "0");
        serial_puts("\n");
        oh_w32(OH_RHSTATUS, OH_RH_SETGLOB);      /* SetGlobalPower */
        uint32_t pot = (da >> 24) & 0xFFu;       /* POTPGT, x2 мс        */
        if (pot < 5u) pot = 5u;
        hw_delay(pot * 2000u);
        if (psm && !nps) {
            /* per-port switching: включаем каждый порт отдельно */
            for (int dp = 1; dp <= (int)g_oh_nports; dp++)
                oh_w32(oh_port(dp), 0x100u);     /* SetPortPower (PPS) */
            hw_delay(pot * 2000u);
            serial_puts("MOUSEHW: OHCI: per-port power включён (PSM=1)\n");
        }
    }
    /* [#417] кэш-когерентность включения HC: HCCA/ED/HcControlHeadED
     * только что записаны в кэшируемую память — без wbinvd контроллер
     * на WB-кэше (AMD FCH, без MTRR) читает из ОЗУ МУСОРНЫЕ указатели
     * периодической таблицы → дикий DMA.  Передачи #331 закрыл давно,
     * ЭТОТ момент был пропущен. */
    dma_sync();
    oh_w32(OH_CTRL, OH_CTRL_PLE | OH_CTRL_CLE | OH_CTRL_BLE | OH_CTRL_OPER);
    hw_delay(20000);              /* 2 кадра: HC стабилизируется ПОСЛЕ
                                  * перехода в USBOPERATIONAL, до первого
                                  * ресета порта (строгий реальный кремний) */

    serial_puts("MOUSEHW: OHCI portsc:");
    for (int dp = 1; dp <= (int)g_oh_nports; dp++) {
        serial_puts(" p");
        serial_puthex((uint32_t)dp);
        serial_puts("=");
        serial_puthex(oh_r32(oh_port(dp)));
    }
    serial_puts("\n");

    for (int p = 1; p <= (int)g_oh_nports; p++)
        ohci_probe_port(p, 1);            /* force: CSC мог быть сброшен   */
    if (g_oh_active) return 1;
    if (g_oh_msd) {
        serial_puts("MOUSEHW: OHCI: MSD перечислен, мыши нет (контроллер работает)\n");
    } else {
        oh_diag_set("OHCI: порты опрошены, HID/MSD не найдено");
        serial_puts("MOUSEHW: no OHCI mouse found (left running for hotplug)\n");
    }
    return 0;
}

/* Обслуживание interrupt-IN: разобрать done-список, пере-поставить TD */
/* 0.3.0: отчёты ВСЕХ живых OHCI-инстансов — мышь и клавиатура могут
 * жить на РАЗНЫХ компаньонах (AMD FCH: 0:12.0 и 0:13.0). */
static void ohci_poll_reports_inst(void);

static void ohci_poll_reports(void) {
    /* [#466] ЭТАП 9: ВСЕ OHCI_NINST инстанса (было oi < 2 — на 3-м/4-м
     * OHCI, куда AMD FCH и PCIe USB2-карты сажают порты задней панели,
     * устройства ПЕРЕЧИСЛЯЛИСЬ и попадали в КАРТУ, но отчёты НИКОГДА не
     * читались: мышь/клава «видны, но не работают» — главный симптом
     * пользователя на доп. USB-карте). */
    for (int oi = 0; oi < OHCI_NINST; oi++) {
        if (g_oh_mmio_a[oi] == 0) continue;
        g_oi = oi;
        ohci_poll_reports_inst();
    }
}

static void ohci_poll_reports_inst(void) {
    /* обработка done-списка ОДИН раз на кадр (HcFmNumber). Между
     * границами кадров DoneHead не меняется, а прежний код переигрывал
     * один и тот же отчёт тысячи раз за кадр (дёрганье курсора). */
    static uint32_t last_fm_a[OHCI_NINST];
    uint32_t fm = oh_r32(OH_FMNUM) & 0xFFFFu;
    if (fm == last_fm_a[g_oi]) return;
    dma_sync();   /* done-очередь/отчёты пишет HC — читаем свежие */
    uint32_t dh = oh_donehead();
    last_fm_a[g_oi] = fm;
    if (!dh) return;
    oh_ack_wdh();
    uint32_t t = dh & ~1u;
    int guard = 8;
    while (t && guard-- > 0) {
        uint32_t nxt = *(volatile uint32_t *)(t + 8);
        if (t == vphys(oh_td_i)) {
            uint32_t cc  = TD_CC_GET(oh_dma32(&oh_td_i[0]));
            uint32_t cbp = oh_dma32(&oh_td_i[1]); /* write-back: за последним байтом */
            if (cc == TD_CC_OK) {
                int act = (int)lm_mps();               /* [#474] MPS — из СНИМКА живой мыши */
                if (cbp) act = (int)(oh_dma32(&oh_td_i[3]) + 1u - cbp);
                if (act < 0) act = 0;
                if (act > (int)sizeof(oh_pkt)) act = (int)sizeof(oh_pkt);
                if (act < 1) {
                    /* [#475] Ситуация B: ДЛИНА 0 — это НЕ «живые данные»:
                     * не считаем отчёт и не трогаем курсор, перевзводим TD. */
                    zl_note("OHCI мышь: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)");
                } else {
                    g_packets++;
                    usbdev_note_report(USBDEV_ROLE_MOUSE);   /* [#462] живость для КАРТЫ ГНЕЗД */
                    uint8_t rp[16];           /* свежая копия отчёта (DMA) */
                    for (int i = 0; i < act; i++)
                        rp[i] = *(volatile uint8_t *)&oh_pkt[i];
                    if (g_packets == 1) {
                        serial_puts("MOUSEHW: first report (OHCI, ");
                        serial_puthex((uint32_t)act);
                        serial_puts(" bytes):");
                        for (int i = 0; i < act; i++) {
                            serial_puts(" ");
                            serial_puthex(rp[i]);
                        }
                        serial_puts("\n");
                    }
                    if (lm_absolute())    hw_tablet_report(rp, act);   /* [#474] маршрутизация — по СНИМКУ */
                    else if (lm_rpt_ok()) hw_hid_report(rp, act);
                    else                  hw_boot_report(rp, act);
                }
            } else {
                serial_puts("MOUSEHW: OHCI transfer error cc=");
                serial_puthex(cc);
                serial_puts("\n");
                if (++g_errors >= 8) {
                    oh_ed_intr[0] |= ED_SKIP;
                    g_oh_active = 0;
                    g_active    = false;
                    /* [#414] полный detach, как у выдёргивания: раньше здесь
                     * не сбрасывались g_present/реестр/таблица — слот мыши
                     * оставался «занят» НАВСЕГДА, и переподключённая мышь
                     * получала «драйвер уже занят» → НЕИЗВЕСТНОЕ до reboot. */
                    g_present = false;
                    g_oh_ms_usb_addr = 0;
                    g_oh_ms_oi = -1;
                    lm_drop();           /* [#474] снимок мыши погашен */
                    usbdev_release(USBDEV_ROLE_MOUSE, "OHCI transfer errors");
                    usbdev_unregister(1, (int)g_oh_ms_port, "OHCI transfer errors");
                    mouse_hw_portmap_mark();   /* [#451] карта устарела */
                    serial_puts("MOUSEHW: deactivated (OHCI transfer errors), PS/2 resumes\n");
                    return;
                }
            }
            /* пере-постановка TD (carry-бит ED сохранить, halted снять) */
            oh_td_i[0] = TD_DP_IN | TD_DI_NOW | TD_T_CARRY | TD_ROUND;
            oh_td_i[1] = vphys(oh_pkt);
            oh_td_i[2] = vphys(oh_td_tail_i);   /* NextTD -> dummy */
            oh_td_i[3] = vphys(oh_pkt) + (uint32_t)lm_mps() - 1u;   /* [#474] MPS — из СНИМКА */
            oh_ed_intr[2] = vphys(oh_td_i) | (oh_ed_intr[2] & 2u); /* C храним, H снимаем */
        }
        else if (t == vphys(oh_td_k)) {         /* отчёт клавиатуры */
            uint32_t cc  = TD_CC_GET(oh_dma32(&oh_td_k[0]));
            uint32_t cbp = oh_dma32(&oh_td_k[1]);
            if (cc == TD_CC_OK) {
                int act = (int)lk_oh_mps();          /* [#474] MPS — из СНИМКА живой клавы */
                if (cbp) act = (int)(oh_dma32(&oh_td_k[3]) + 1u - cbp);
                if (act < 0) act = 0;
                if (act > (int)sizeof(oh_kpkt)) act = (int)sizeof(oh_kpkt);
                if (act >= 1) {                 /* [#475] нулевая длина — НЕ живые данные */
                    uint8_t kp[16];           /* свежая копия отчёта (DMA) */
                    for (int i = 0; i < act; i++)
                        kp[i] = *(volatile uint8_t *)&oh_kpkt[i];
                    oh_kbd_process(kp, act, "OHCI");
                } else {
                    zl_note("OHCI клава: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)");
                }
            }   /* ошибки клавиатуры молча: клава не должна сыпать лог */
            memset(oh_kpkt, 0, sizeof(oh_kpkt));  /* без phantom-нажатий */
            oh_td_k[0] = TD_DP_IN | TD_DI_NOW | TD_T_CARRY | TD_ROUND;
            oh_td_k[1] = vphys(oh_kpkt);
            oh_td_k[2] = vphys(oh_td_tail_i);
            oh_td_k[3] = vphys(oh_kpkt) + (uint32_t)lk_oh_mps() - 1u;   /* [#474] MPS — из СНИМКА */
            oh_ed_kbd[2] = vphys(oh_td_k) | (oh_ed_kbd[2] & 2u); /* C храним, H снимаем */
        }
        t = nxt;
    }
    dma_sync();   /* пере-поставленные TD дойдут до HC */
}

/* ═══ «ЭТАП 2»: бывшая функция ehci_handoff() УДАЛЕНА — EHCI целиком
 * обслуживает kernel/ehci.c (handoff USBLEGSUP + глушение USBLEGCTLSTS,
 * HCR-сброс, расписания, CONFIGFLAG, перечисление HS, раздача LS/FS
 * компаньонам).  Старый handoff из mouse_hw.c ставил CONFIGFLAG=0 и
 * останавливал контроллер ДО ehci_init — теперь порядок обратный
 * (kernel.c: mouse_hw_init → ehci_init → mouse_hw_init_ohci), и «машание»
 * CONFIGFLAG туда-сюда под работающими устройствами больше не случается. */

/* ═══ 0.3.2: xHCI — снятие BIOS-владения (USBLEGSUP через xECP) ═══════
 * Современные платы (Intel 8-series+, AMD) держат «порты 2.0» на xHCI.
 * EECP-handoff раньше выполнялся только для EHCI — xHCI работал «как
 * повезёт»: пока BIOS (SMM) не отдал контроллер, MMIO-обращения могли
 * перехватываться и портиться. Здесь тот же семафор OS-owned по
 * спецификации xHCI: Extended Capability ID 1 (USB Legacy Support),
 * USBLEGSUP биты 16 (BIOS owned) / 24 (OS owned). */
static void xhci_handoff(uint8_t bus, uint8_t dev, uint8_t fn)
{
    uint32_t bar = pci_read32(bus, dev, fn, 0x10);
    uint32_t mm, off;
    int guard = 0;

    if (bar & 1u)
        return;                                /* I/O BAR — не xHCI */
    /* F3: 64-битный MEM BAR (тип 2) — старшие биты адреса в BAR1
     * (0x14); MMIO выше 4 ГБ 32-битному ядру недоступен (identity
     * map), такой контроллер пропускаем, не трогая чужие регистры. */
    if (((bar >> 1) & 3u) == 2u && pci_read32(bus, dev, fn, 0x14) != 0u)
        return;
    mm = bar & ~0xFu;
    if (mm < 0x1000u)
        return;

    {
        uint32_t cmd = pci_read32(bus, dev, fn, 0x04);
        pci_write32(bus, dev, fn, 0x04, cmd | 0x6u);   /* mem+busmaster */
    }

    /* HCCPARAMS1 (0x10), биты 31:16 = xECP ПЕРВОЙ капабилити (в двордах).
     * [#421/#422] Дальше идём по Next-указателю dword0[15:8] (как Linux
     * xhci-ext-caps.h), а НЕ по [31:16] — там лежат семафоры/версия.
     * Семафоры BIOS/OS-Owned — биты 16/24 dw0 (заголовка), USBLEGCTLSTS
     * — на +4 (второй dword).  Раньше «USBLEGSUP» читался/писался в dw1
     * (+4) и SMI глушился на +8: на AMD, где Legacy-капабилити не первая
     * в списке, handoff не выполнялся ВООБЩЕ — SMM продолжал владеть
     * контроллером («работает как повезёт»/«СМАЗ»). */
    off = (mmr32(mm + 0x10u) >> 16) & 0xFFFFu;
    while (off != 0u && guard++ < 16) {
        uint32_t hdr = mmr32(mm + off * 4u);
        if ((hdr & 0xFFu) == 1u) {             /* USB Legacy Support */
            if (hdr & 0x010000u) {             /* BIOS Owned Semaphore: dw0 бит 16 */
                serial_puts("MOUSEHW: xHCI at ");
                serial_puthex(bus); serial_puts(":");
                serial_puthex(dev); serial_puts(".");
                serial_puthex(fn); serial_puts(": BIOS handoff ");
                mmw32(mm + off * 4u, hdr | 0x01000000u);   /* OS Owned: dw0 бит 24 */
                int t = 1000;
                while ((mmr32(mm + off * 4u) & 0x010000u) && t-- > 0)
                    hw_delay(1000);
                serial_puts(t > 0 ? "done" : "timeout");
                serial_puts("\n");
                /* F3 (спека xHCI §4.14.2): глушим SMI-источники BIOS в
                 * USBLEGCTLSTS (xECP+4) — запись 0 снимает только
                 * SMI-Enable биты, RW1C-статусы не задевает. */
                mmw32(mm + off * 4u + 4u, 0u);
            }
            break;
        }
        off = (hdr >> 8) & 0xFFu;              /* [#421] Next xECP: dword0[15:8] */
    }
}


/* ═══ hotplug xHCI — «обнаружено новое устройство!» на живом контроллере
 * «диалог с устройствами» работает ВСЕГДА, даже при активной мыши:
 *   - свой порт мыши - отключение; свой порт клавиатуры - отключение;
 *   - прочие порты - только по CSC, с ролевой памятью: порт с клавиатурой
 *     («пропускаем - там клава») и помеченные («ввода нет») не
 *     перечисляются повторно - раньше клавиатурный порт без мыши гонял
 *     полный цикл перечисления каждые 16 тиков и после 3 «неудач»
 *     перезапускал контроллер каждые ~0.13 с (BIOS-эмуляция Клавы умирала). */
static void xhci_hotplug_inst(void) {
    static bool phantom_once;   /* «фиктивный порт» - раз за эпизод */
    if (!g_xhci_live || g_mmio == 0) return;
    /* 1) свой порт мыши: следим за отключением (и НЕ выходим - ловит
     *    клавиатуру и новые устройства и при активной мыши) */
    if (g_active && g_ms_xi == g_xi) {
        mhw_slot_check("hotplug");   /* сверка штампов */
        mhw_port_check("hotplug");
        if (g_port < 1 || g_port > (int)g_maxports) {
            /* подпись СМАЗа - порт мыши вне диапазона. Linux не
             * отцепляет устройство по испорченному полю: сначала факт
             * (PORTSC своего порта), потом вывод.  Мышь остаётся жить.
 * журнал - только про события (как printk_once в ядре
             * Linux): одно сообщение за эпизод, текстлог не тонет. */
            if (!phantom_once) {
                serial_puts("MOUSEHW: фиктивный порт мыши - отключение ОТМЕНЕНО (дальше молчу)\n");
                phantom_once = true;
            }
            return;
        }
        phantom_once = false;      /* порт снова в норме - эпизод закрыт */
        uint32_t pa = port_addr(g_port);
        uint32_t v  = mmr32(pa);
        if ((v & PORTSC_CCS) == 0) {
            serial_puts("MOUSEHW: USB: устройство отключено (xHCI порт ");
            serial_puthex((uint32_t)g_port);
            serial_puts(")\n");
            hw_deactivate("device detached");
        }
    }
    /* 2) свой порт клавиатуры */
    if (g_kbd_slot != 0 && g_kb_xi == g_xi) {
        uint32_t v = mmr32(port_addr(g_kbd_xport));
        if ((v & PORTSC_CCS) == 0) {
            serial_puts("MOUSEHW: USB-клавиатура отключена (xHCI порт ");
            serial_puthex((uint32_t)g_kbd_xport);
            serial_puts(")\n");
            hw_kbd_disable_slot();
            /* клавиатура покинула порт - unregister в таблице */
            usbdev_unregister(0, (int)g_kbd_xport, "kbd detach");
            ps2_kbd_detached();   /* её зажатый Enter не залипает навсегда */
            g_port_role[g_kbd_xport] = PR_EMPTY;   /* [#460] */
            g_kbd_xport = 0;
        }
    }
    /* 3) прочие порты: по CSC, уважая роль порта */
    for (int p = 1; p <= (int)g_maxports; p++) {
        uint32_t v;
        if (g_active && g_ms_xi == g_xi && p == g_port) continue;
        if (g_kbd_slot != 0 && g_kb_xi == g_xi && p == g_kbd_xport) continue;
        if (p < (int)sizeof(g_hp_cd) && g_hp_cd[p]) { g_hp_cd[p]--; continue; }
        v = mmr32(port_addr(p));
        if (v & (1u << 17)) {                     /* CSC                 */
            if (!(v & PORTSC_CCS)) {
                mmw32(port_addr(p), (v & 0xFFFFu) | (1u << 17));
                if (p < (int)sizeof(g_port_role))
                    g_port_role[p] = PR_EMPTY;    /* отключение - роль вон */
                continue;
            }
            /* «порт 2 пропускаем - там клава», «пометили - пропускаем» */
            if (p < (int)sizeof(g_port_role) &&
                (g_port_role[p] == PR_KBD || g_port_role[p] == PR_OTHER)) {
                mmw32(port_addr(p), (v & 0xFFFFu) | (1u << 17));
                continue;
            }
            if (xhci_try_port(p)) { g_hp_fails = 0; return; }
            /* РАНЬШЕ здесь контроллер останавливался НАВСЕГДА
             * (xhci_halt_now + g_xhci_live=0): одной неудачной попытки
             * (глюк мыши, флешка в порту) хватало, чтобы мышь больше
             * НИКОГДА не находилась до перезагрузки.  Теперь: Disable
             * Slot + кулдаун порта, после 3 неудач подряд - полный
             * ре-пробинг контроллера (сброс + чистое состояние).
 * ре-пробинг ТОЛЬКО когда мышь не активна - сброс общих
             * буферов убил бы работающую мышь; при активной мыши неудачи
             * перечисления (флешка в порту) просто кулдаунятся. */
            hw_disable_slot();
            g_hp_fails++;
            if (p < (int)sizeof(g_hp_cd)) g_hp_cd[p] = 16;
            if (g_hp_fails >= 3 && g_xh_have &&
                !(g_active && g_ms_xi == g_xi) &&
                !(g_kbd_slot != 0 && g_kb_xi == g_xi)) {   /* [#426] живая клава — не сбрасывать */
                g_hp_fails = 0;
                serial_puts("MOUSEHW: hotplug: 3 неудачи - полный ре-пробинг xHCI\n");
                xhci_halt_now();
                probe_xhci(g_xi, g_xh_bus, g_xh_dev, g_xh_fn, g_xh_vid, g_xh_did);
            } else {
                serial_puts("MOUSEHW: hotplug enumeration failed - повторю позже\n");
            }
            return;
        }
    }
}

/* ═══ [#412] USB-КАРТА ПОРТОВ — ПОЛНЫЙ список всех портов всех ПК ══════
 * Запрос пользователя: «программа ищет все порты и пишет в логе, какой
 * это порт и какой» (жалоба: «портов 12, ОС видит 5», «неправильно
 * определяет, что это за порт»). Карта печатается один раз на загрузке
 * (kernel.c, после mouse_hw_init_ohci) и покрывает:
 *   • xHCI×2  — порты 1..MAX_PORTS (HCSPARAMS1), протокол КАЖДОГО порта
 *               из Supported Protocol (USB2/USB3), CCS/PED/скорость,
 *               занятость (мышь/клавиатура/прочее) и слот;
 *   • EHCI×4  — порты из HCSPARAMS (блок печатает ehci_portmap);
 *   • OHCI×2  — порты из RhDescriptorA.NDP (теперь честные 8 бит);
 *   • UHCI    — PCI-скан (prog-if 0x00): по 2 порта на контроллер,
 *               статус из PORTSC через I/O.
 * Итоговая строка считает ВСЕ порты и рабочие устройства — сразу видно
 * расхождение «12 гнёзд на плате против 23 портов в карте» (гнёзда
 * дублируются линиями USB2+USB3) или «5 видимых». */
static uint8_t g_portmap_dirty;   /* [#451] флаг «карта устарела» (ставится привязкой/отключением) */
/* ═══ [#459] USB-КАРТА: ЕДИНАЯ КАРТА ФИЗИЧЕСКИХ ГНЕЗД — модель Linux «peer» ═══
 * Диагноз пользователя ПОДТВЕРДИЛСЯ сверкой с исходниками Linux (архив
 * drivers/: drivers/usb/core/port.c find_and_link_peer/link_peers,
 * drivers/usb/host/xhci-mem.c xhci_setup_port_arrays/xhci_add_in_port):
 *   • одно физическое гнездо USB 3.x держит ДВЕ логические линии —
 *     SuperSpeed и USB 2.0 (D+/D-); обычная мышь/клава в синем гнезде
 *     ВСЕГДА живёт на линии USB 2.0 — это НОРМА USB, а не ошибка;
 *   • Linux НЕ угадывает гнёзда: xHCI делит порты по Supported Protocol
 *     на usb2-/usb3-хабы, нумерует каждый ЗАНОВО (1..N) и связывает
 *     peer «root-порт N ↔ root-порт N» по индексу; точное положение
 *     разъёма на корпусе даёт только ACPI _PLD (его нет) — Linux тоже
 *     честно пишет «неизвестно»;
 *   • EHCI↔компаньоны (UHCI/OHCI) Linux вообще НЕ сопоставляет: FS/LS
 *     маршрутизирует сам чипсет (PORTSC OWNER), ОС видит устройство
 *     уже на порту компаньона — соответствие портов не знает НИКТО.
 * Прежняя карта печатала раздельные списки логических портов, и «USB3:
 * пусто, мышь на порту 6 (USB2)» выглядело как путаница номеров. Теперь:
 *   • xHCI: ГНЕЗДА = peer-пары (линия USB3=…, линия USB2=…);
 *   • EHCI: порт = гнездо USB 2.0 (+ FS/LS через компаньона) [#463];
 *   • OHCI/UHCI: линия 1.x, максимум гнезда — «USB 2.0, если компаньон»
 *     с честной пометкой «соответствие без ACPI неизвестно»;
 *   • под каждым гнездом — технические сведения (контроллер, исходные
 *     номера логических портов, скорость, перечисление, драйвер) и
 *     счётчик живых HID-отчётов [#462] — «ЗАНЯТ» доказан потоком данных;
 *   • порты с необъявленным протоколом не теряются (Г9/#460: до 255).
 * Печатается при загрузке (kernel.c) и после hotplug (#451). */

/* [#462] маленькая печать без uint32_t */
void portmap_pm_n(int v) {
    char nb[12];
    serial_puts(itoa((uint32_t)v, nb, 10));
}

/* [#462] строка живости владельца: «HID-отчётов: N (ДАННЫЕ ИДУТ)».
 * «ЗАНЯТ» в карте доказывается реальным потоком HID-отчётов, а не
 * только фактом привязки слота (диагноз пользователя: «статус ЗАНЯТ
 * не доказывает, что устройство передаёт данные»). */
void portmap_pm_alive(int role) {
    uint32_t n = usbdev_report_count(role);
    serial_puts(", HID-отчётов: ");
    portmap_pm_n((int)(n & 0x7FFFFFFFu));
    if (n > 0) serial_puts(" (ДАННЫЕ ИДУТ)");
    else       serial_puts(" (данных нет — проверь драйвер/устройство)");
}

/* [#459] состояние ОДНОЙ линии xHCI-гнезда. Возвращает: 0 = пусто,
 * 1 = устройство с драйвером (посчитано как devs_bound), 2 = устройство
 * есть, но без драйвера. Печатает строку «линия ...: ...». */
static int pm_xh_side(int i, int p, const char *side, int *devs_bound) {
    uint32_t v   = mmr32(port_addr(p));
    uint32_t spd = (v >> PORTSC_SPEED_SH) & 0xFu;
    if (!(v & PORTSC_CCS)) return 0;           /* линия свободна */
    serial_puts("USB-КАРТА:   линия ");
    serial_puts(side);
    serial_puts(": УСТРОЙСТВО (");
    serial_puts(spd_name(spd));
    if (!(v & PORTSC_PED)) serial_puts(", перечисление НЕ прошло");
    serial_puts(")");
    if (g_active && g_ms_xi == i && p == g_port) {
        serial_puts(" — МЫШЬ (слот ");
        portmap_pm_n((int)g_slot);
        portmap_pm_alive(USBDEV_ROLE_MOUSE);
        serial_puts(")");
        if (devs_bound) (*devs_bound)++;
        serial_puts("\n");
        return 1;
    }
    if (g_kbd_slot != 0 && g_kb_xi == i && p == g_kbd_xport) {
        serial_puts(" — КЛАВИАТУРА (слот ");
        portmap_pm_n((int)g_kbd_slot);
        portmap_pm_alive(USBDEV_ROLE_KBD);
        serial_puts(")");
        if (devs_bound) (*devs_bound)++;
        serial_puts("\n");
        return 1;
    }
    if (p < (int)sizeof(g_port_role) && g_port_role[p] == PR_OTHER)
        serial_puts(" (прочее — не клава/не мышь)");
    serial_puts("\n");
    return 2;
}

void mouse_hw_portmap(void)
{
    int ctrls = 0, lports = 0, devs_bound = 0;
    int xh_sock = 0, eh_ports = 0, ou_ports = 0;
    int sock_no = 0;        /* [#472] СКВОЗНОЙ номер гнезда — уникален
                             * по ВСЕЙ карте (раньше нумерация начиналась
                             * заново на каждом контроллере: «порт 4»
                             * материнки и «порт 4» доп-карты выглядели
                             * ОДНИМ гнездом — жалоба пользователя) */
    g_portmap_dirty = 0;   /* [#451] карта напечатана — она свежая */

    serial_puts("USB-КАРТА: ═══ ЕДИНАЯ КАРТА ФИЗИЧЕСКИХ ГНЕЗД — Сборка ");
    serial_puts(EGOROV_OS_BUILD_S);
    serial_puts(" ═══ [#473]\n");
    serial_puts("USB-КАРТА: ГНЕЗДА НУМЕРУЮТСЯ СКВОЗНО (№1..№N) ПО ВСЕЙ карте\n");
    serial_puts("USB-КАРТА: [#472]: «порт 4» одного контроллера и «порт 4» другого\n");
    serial_puts("USB-КАРТА: — это РАЗНЫЕ гнёзда; различай их по PCI-адресу в\n");
    serial_puts("USB-КАРТА: скобках (контроллер = шина:устройство.функция).\n");
    serial_puts("USB-КАРТА: гнездо USB 3.x держит ДВЕ линии (SuperSpeed + USB 2.0):\n");
    serial_puts("USB-КАРТА: обычная мышь/клава в синем гнезде живёт на линии USB 2.0 —\n");
    serial_puts("USB-КАРТА: это НОРМА USB, а не ошибка драйвера. Номер гнезда = пара\n");
    serial_puts("USB-КАРТА: линий (Linux find_and_link_peer: root-порт N usb2-хаба\n");
    serial_puts("USB-КАРТА: ↔ root-порт N usb3-хаба). Точное положение гнезда на корпусе\n");
    serial_puts("USB-КАРТА: без ACPI _PLD не знает и Linux — «неизвестно» пишем честно.\n");

    /* ── xHCI: ГНЕЗДА из пар протоколов (peer, Linux-модель) ── */
    for (int i = 0; i < XHCI_NINST; i++) {
        if (g_xh_mmio_a[i] == 0) continue;          /* не зондировался */
        ctrls++;
        int saved_xi = g_xi;
        g_xi = i;
        serial_puts("USB-КАРТА: ── xHCI#");
        portmap_pm_n(i + 1);
        serial_puts(" (PCI ");
        portmap_pm_n((int)g_xh_bus_a[i]);
        serial_puts(":");
        portmap_pm_n((int)g_xh_dev_a[i]);
        serial_puts(".");
        portmap_pm_n((int)g_xh_fn_a[i]);
        serial_puts("): ");
        if (!g_xhci_live_a[i]) {
            serial_puts("найден, но не работает (остановлен) - порты НЕ видны\n");
            g_xi = saved_xi;
            continue;
        }
        int nmax = (int)g_maxports_a[i];
        if (nmax > XH_NPORT_MAX) nmax = XH_NPORT_MAX;

        /* [#459] peer-группировка (как find_and_link_peer у Linux):
         * логические порты каждого протокола собираются в СВОЙ список
         * и связываются ПО ИНДЕКСУ — ровно «root-порт N ↔ root-порт N»
         * после раздельной нумерации usb2-/usb3-хабов. Сверено с
         * drivers/usb/host/xhci-mem.c: xhci_add_in_port()/xhci_create_
         * rhub_port_array() и drivers/usb/core/port.c: link_peers(). */
        uint8_t u3[XH_NPORT_MAX + 1]; int n3 = 0;   /* SuperSpeed-линии */
        uint8_t u2[XH_NPORT_MAX + 1]; int n2 = 0;   /* USB 2.0-линии    */
        int nunk = 0;
        for (int p = 1; p <= nmax; p++) {
            lports++;
            if      (g_xh_pproto_a[i][p] == 3) u3[n3++] = (uint8_t)p;
            else if (g_xh_pproto_a[i][p] == 2) u2[n2++] = (uint8_t)p;
            else nunk++;
        }
        int npair = (n3 > n2) ? n3 : n2;
        xh_sock += npair + nunk;
        serial_puts("портов ");
        portmap_pm_n(nmax);
        serial_puts(" (USB3: ");
        portmap_pm_n(n3);
        serial_puts(", USB2: ");
        portmap_pm_n(n2);
        if (nunk) {
            serial_puts(", протокол не объявлен: ");
            portmap_pm_n(nunk);
        }
        serial_puts(") → гнёзд peer: ");
        portmap_pm_n(npair + nunk);
        serial_puts("\n");

        for (int k = 0; k < npair; k++) {
            int p3 = (k < n3) ? (int)u3[k] : 0;
            int p2 = (k < n2) ? (int)u2[k] : 0;
            serial_puts("USB-КАРТА: ГНЕЗДО №");
            portmap_pm_n(++sock_no);
            serial_puts(" [xHCI#");
            portmap_pm_n(i + 1);
            serial_puts(" PCI ");
            portmap_pm_n((int)g_xh_bus_a[i]);
            serial_puts(":");
            portmap_pm_n((int)g_xh_dev_a[i]);
            serial_puts(".");
            portmap_pm_n((int)g_xh_fn_a[i]);
            serial_puts("]: макс ");
            if (p3 && p2) serial_puts("USB 3.x + USB 2.0");
            else if (p3)  serial_puts("USB 3.x (линия USB2 не объявлена)");
            else          serial_puts("USB 2.0 (SuperSpeed не объявлен)");
            serial_puts(" | лог.порты:");
            if (p3) { serial_puts(" USB3="); portmap_pm_n(p3); }
            if (p2) { serial_puts(" USB2="); portmap_pm_n(p2); }
            serial_puts("\n");
            int busy2 = 0, busy3 = 0;
            if (p3) busy3 = pm_xh_side(i, p3, "SuperSpeed", &devs_bound);
            if (p2) busy2 = pm_xh_side(i, p2, "USB 2.0", &devs_bound);
            if (!busy3 && !busy2)
                serial_puts("USB-КАРТА:   состояние: пусто (обе линии свободны)\n");
            else if (busy2 == 1 && p3)
                serial_puts("USB-КАРТА:   (устройство 2.0 на линии 2.0 гнезда 3.x — норма USB)\n");
        }
        /* порты с необъявленным протоколом — гнёзда с честной пометкой */
        for (int p = 1; p <= nmax; p++) {
            if (g_xh_pproto_a[i][p] == 2 || g_xh_pproto_a[i][p] == 3) continue;
            serial_puts("USB-КАРТА: ГНЕЗДО №");
            portmap_pm_n(++sock_no);
            serial_puts(" [xHCI#");
            portmap_pm_n(i + 1);
            serial_puts(" PCI ");
            portmap_pm_n((int)g_xh_bus_a[i]);
            serial_puts(":");
            portmap_pm_n((int)g_xh_dev_a[i]);
            serial_puts(".");
            portmap_pm_n((int)g_xh_fn_a[i]);
            serial_puts(" порт ");
            portmap_pm_n(p);
            serial_puts("]: протокол не объявлен (Supported Protocol не найден)\n");
            if (!pm_xh_side(i, p, "???", &devs_bound))
                serial_puts("USB-КАРТА:   состояние: пусто\n");
        }
        g_xi = saved_xi;
    }

    /* ── EHCI (до 4 инстансов) — блок печатает сам ehci.c [#463];
     * [#472] передаём СКВОЗНОЙ счётчик гнёзд — нумерация ЕДИНАЯ ── */
    ehci_portmap(&ctrls, &lports, &devs_bound, &eh_ports, &sock_no);

    /* ── OHCI (до OHCI_NINST инстансов) [#415/#419/#459] ── */
    for (int i = 0; i < OHCI_NINST; i++) {
        if (g_oh_mmio_a[i] == 0) {
            /* [#419] контроллер найден по PCI, но НЕ поднялся (reset
             * timeout/SMM): раньше он ИСЧЕЗАЛ из карты бесследно — счёт
             * гнёзд занижался. Печатаем честно, как xHCI. */
            if (g_oh_dead_a[i]) {
                ctrls++;
                serial_puts("USB-КАРТА: ── OHCI#");
                portmap_pm_n(i + 1);
                serial_puts(" (PCI ");
                portmap_pm_n((int)g_oh_pci_a[i][0]);
                serial_puts(":");
                portmap_pm_n((int)g_oh_pci_a[i][1]);
                serial_puts(".");
                portmap_pm_n((int)g_oh_pci_a[i][2]);
                serial_puts("): найден, но не поднялся - порты НЕ видны\n");
            }
            continue;
        }
        ctrls++;
        ou_ports += (int)g_oh_nports_a[i];
        int saved_oi = g_oi;
        g_oi = i;
        serial_puts("USB-КАРТА: ── OHCI#");
        portmap_pm_n(i + 1);
        serial_puts(" (PCI ");
        portmap_pm_n((int)g_oh_pci_a[i][0]);
        serial_puts(":");
        portmap_pm_n((int)g_oh_pci_a[i][1]);
        serial_puts(".");
        portmap_pm_n((int)g_oh_pci_a[i][2]);
        serial_puts("): портов ");
        portmap_pm_n((int)g_oh_nports_a[i]);
        serial_puts("\n");
        {   /* [#459/#463] честный тип гнезда: линия OHCI — сигналинг 1.x;
             * макс гнезда — USB 2.0, ТОЛЬКО если это гнездо-компаньон
             * EHCI; точное соответствие портов компаньонов без ACPI
             * неизвестно — как в Linux (маршрутизацию FS/LS делает сам
             * чипсет, ОС видит устройство уже на порту компаньона). */
            const char *sock2 = usbdev_companion_possible() ?
                 "USB 2.0, если это гнездо-компаньон EHCI (на плате EHCI\n"
                 "USB-КАРТА:     есть — вероятно компаньон; соответствие без ACPI неизвестно)"
                 : "USB 1.x (EHCI на плате нет — честные 1.x-гнёзда)";
            for (int p = 1; p <= (int)g_oh_nports_a[i] && p <= 15; p++) {
                uint32_t ps = oh_r32(oh_port(p));
                lports++;
                serial_puts("USB-КАРТА: ГНЕЗДО №");
                portmap_pm_n(++sock_no);
                serial_puts(" [OHCI#");
                portmap_pm_n(i + 1);
                serial_puts(" PCI ");
                portmap_pm_n((int)g_oh_pci_a[i][0]);
                serial_puts(":");
                portmap_pm_n((int)g_oh_pci_a[i][1]);
                serial_puts(".");
                portmap_pm_n((int)g_oh_pci_a[i][2]);
                serial_puts(" порт ");
                portmap_pm_n(p);
                serial_puts("]: линия USB 1.x (");
                serial_puts((ps & 0x200u) ? "LS" : "FS");
                serial_puts(") | макс гнезда: ");
                serial_puts(sock2);
                serial_puts("\n");
                if (!(ps & 1u)) {
                    serial_puts("USB-КАРТА:   состояние: пусто\n");
                    continue;
                }
                serial_puts("USB-КАРТА:   состояние: ");
                serial_puts((ps & 0x200u) ? "low-speed 1.5 Мбит/с" : "full-speed 12 Мбит/с");
                if (g_oh_active && g_oh_ms_oi == i && p == g_oh_ms_port) {
                    serial_puts(" — МЫШЬ (адрес ");
                    portmap_pm_n((int)g_oh_ms_usb_addr);
                    portmap_pm_alive(USBDEV_ROLE_MOUSE);
                    serial_puts(")");
                    devs_bound++;
                    serial_puts("\n");
                    continue;
                }
                if (g_oh_kbd && g_oh_kb_oi == i && p == g_oh_kbd_port) {
                    serial_puts(" — КЛАВИАТУРА (адрес ");
                    portmap_pm_n((int)g_oh_kbd_usb_addr);
                    portmap_pm_alive(USBDEV_ROLE_KBD);
                    serial_puts(")");
                    devs_bound++;
                    serial_puts("\n");
                    continue;
                }
                if (g_oh_msd && g_oh_msd_oi == i && p == g_oh_msd_port) {
                    serial_puts(" — ФЛЕШКА (sticklog)\n");
                    devs_bound++;
                    continue;
                }
                serial_puts(", устройство ЕСТЬ, перечисление ");
                if (g_oh_fail_a[i][p] >= 3) {
                    serial_puts("НЕ удалось (попыток ");
                    portmap_pm_n((int)g_oh_fail_a[i][p]);
                    serial_puts(") - жду переподключения");
                } else {
                    serial_puts("идёт...");
                }
                serial_puts("\n");
            }
        }
        g_oi = saved_oi;
    }

    /* ── UHCI: из СПИСКА PCI-скана (все шины! [#415]), до 4 портов на
     * контроллер (ghost-детект), владельцы видны [#415/Г4/Г5/#459] ── */
    for (int i = 0; i < g_usb_n; i++) {
        if (g_usb_pg[i] != 0x00u) continue;          /* не UHCI */
        uint8_t ub = g_usb_bus[i], ud = g_usb_dev[i], uf = g_usb_fn[i];
        uint32_t bar = pci_read32(ub, ud, uf, 0x20u);
        if (!(bar & 1u)) {                            /* [#434] fallback BAR0 */
            uint32_t bar0 = pci_read32(ub, ud, uf, 0x10u);
            if (bar0 & 1u) bar = bar0;
        }
        uint32_t ibase = (bar & 1u) ? (bar & ~0xFu) : 0u;
        if (ibase == 0 || (ibase & ~0xFFFFu)) continue;   /* легальный 16-битный I/O */
        uint16_t ubase = (uint16_t)ibase;
        uint16_t umio  = usb_mouse_uhci_io();
        int umport = (umio == ubase) ? usb_mouse_uhci_port() : -1;
        int ukport = uhci_kbd_bound_port(ubase);
        ctrls++;
        serial_puts("USB-КАРТА: ── UHCI (PCI ");
        portmap_pm_n((int)ub);
        serial_puts(":");
        portmap_pm_n((int)ud);
        serial_puts(".");
        portmap_pm_n((int)uf);
        serial_puts("): ");
        {   /* [#415] настоящие порты UHCI: 0xFFFF/reserved-биты = конца */
            int uports = 0;
            uint16_t psc[4];
            for (int p = 0; p < 4; p++) {
                psc[p] = inw((uint16_t)(ubase + 0x10u + 2u * (uint32_t)p));
                if (psc[p] == 0xFFFFu || (psc[p] & 0xE000u) != 0u) break;
                uports++;
            }
            if (uports == 0) uports = 2;   /* PORTSC не читается — каноничное 2 */
            ou_ports += uports;
            serial_puts("портов ");
            portmap_pm_n(uports);
            serial_puts("\n");
            const char *usock2 = usbdev_companion_possible() ?
                 "USB 2.0, если это гнездо-компаньон EHCI (соответствие\n"
                 "USB-КАРТА:     без ACPI неизвестно — как Linux)"
                 : "USB 1.x (EHCI на плате нет)";
            for (int p = 0; p < uports; p++) {
                lports++;
                serial_puts("USB-КАРТА: ГНЕЗДО №");
                portmap_pm_n(++sock_no);
                serial_puts(" [UHCI PCI ");
                portmap_pm_n((int)ub);
                serial_puts(":");
                portmap_pm_n((int)ud);
                serial_puts(".");
                portmap_pm_n((int)uf);
                serial_puts(" порт ");
                portmap_pm_n(p + 1);
                serial_puts("]: линия USB 1.x | макс гнезда: ");
                serial_puts(usock2);
                serial_puts("\n");
                if (!(psc[p] & 1u)) {
                    serial_puts("USB-КАРТА:   состояние: пусто\n");
                    continue;
                }
                serial_puts("USB-КАРТА:   состояние: ");
                serial_puts((psc[p] & 0x100u) ? "low-speed 1.5 Мбит/с" : "full-speed 12 Мбит/с");
                if (umport == p) {
                    serial_puts(" — МЫШЬ");
                    portmap_pm_alive(USBDEV_ROLE_MOUSE);
                    devs_bound++;
                    serial_puts("\n");
                    continue;
                }
                if (ukport == p) {
                    serial_puts(" — КЛАВИАТУРА");
                    portmap_pm_alive(USBDEV_ROLE_KBD);
                    devs_bound++;
                    serial_puts("\n");
                    continue;
                }
                serial_puts(", устройство ЕСТЬ (драйвер не привязан)\n");
            }
        }
    }

    /* ── ИТОГО [#415/#459] + подсказка ── */
    serial_puts("USB-КАРТА: ИТОГО: контроллеров ");
    portmap_pm_n(ctrls);
    serial_puts(" | логических портов ");
    portmap_pm_n(lports);
    serial_puts(" (линии USB2+USB3\n");
    serial_puts("USB-КАРТА: одного гнезда считаются порознь) | гнёзд: peer-гнёзд xHCI ");
    portmap_pm_n(xh_sock);
    serial_puts(" + порты EHCI ");
    portmap_pm_n(eh_ports);
    serial_puts(" + порты\n");
    serial_puts("USB-КАРТА: OHCI/UHCI ");
    portmap_pm_n(ou_ports);
    serial_puts(" (компаньоны EHCI и OHCI/UHCI-порты могут быть ОДНИМИ гнёздами —\n");
    serial_puts("USB-КАРТА: без ACPI точное число разъёмов корпуса не знает и Linux)\n");
    serial_puts("USB-КАРТА: СКВОЗНЫХ НОМЕРОВ гнёзд в этой карте: ");
    portmap_pm_n(sock_no);
    serial_puts(" (№1..№N\n");
    serial_puts("USB-КАРТА: уникальны [#472]: одинаковый локальный «порт N» у разных\n");
    serial_puts("USB-КАРТА: контроллеров = РАЗНЫЕ гнёзда — смотри PCI-адрес в скобках) |\n");
    serial_puts("USB-КАРТА: устройств с драйвером: ");
    portmap_pm_n(devs_bound);
    {   /* [#462] «живых» = привязанных и шлющих HID-отчёты */
        int alive = 0;
        if (usbdev_bound(USBDEV_ROLE_KBD) && usbdev_report_count(USBDEV_ROLE_KBD) > 0) alive++;
        if (usbdev_bound(USBDEV_ROLE_MOUSE) && usbdev_report_count(USBDEV_ROLE_MOUSE) > 0) alive++;
        serial_puts(", из них с живыми HID-данными: ");
        portmap_pm_n(alive);
    }
    serial_puts("\n");
    serial_puts("USB-КАРТА: подсказка: воткни устройство в гнездо — после hotplug карта\n");
    serial_puts("USB-КАРТА: печатается заново (#451) и покажет его СКВОЗНОЙ № гнезда\n");
    serial_puts("USB-КАРТА: (#472) и PCI-адрес контроллера. Калибровка гнёзд корпуса:\n");
    serial_puts("USB-КАРТА: перетыканием (ACPI _PLD в bare-metal ОС нет).\n");
}

/* ═══ [#451] HOTPLUG-ОБНОВЛЕНИЕ КАРТЫ (Г12): карта больше НЕ «один раз на
 * загрузке» — привязка/отключение ставят флаг, секундный тик usbdev
 * печатает СВЕЖУЮ карту.  Пользователь видит, в какой логический порт
 * попало устройство, ПО мере перетыкания (его «калибровка гнёзд»). ═══ */
void mouse_hw_portmap_mark(void) { g_portmap_dirty = 1; }

void mouse_hw_portmap_flush(void) {
    if (!g_portmap_dirty) return;
    g_portmap_dirty = 0;
    serial_puts("USB-КАРТА: ─── изменение после hotplug — СВЕЖАЯ карта портов ───\n");
    mouse_hw_portmap();
}

/* [#446] LED USB-клавиатур xHCI/OHCI: SET_REPORT(output) по текущему
 * состоянию NumLock/CapsLock.  Вызывается из usb_kbd_leds_all() при
 * каждом тумблере (ps2.c kbd_set_leds).  Раньше индикаторы USB-клавиатур
 * EHCI/OHCI/xHCI не реагировали на тумблеры ВООБЩЕ (синхронизировал
 * только UHCI-путь). */
void mouse_hw_kbd_set_leds(uint8_t hid_leds) {
    /* xHCI-клавиатура: ctrl_xfer адресует слот g_eslot — временно
     * переключаем на слот живой клавиатуры; события её EP за время
     * SET_REPORT вытесняются, пере-arm делает hotplug-цикл [#456]. */
    if (g_kbd_slot != 0 && g_kb_xi >= 0 && g_xhci_live_a[g_kb_xi]) {
        int sv_xi = g_xi, sv_eslot = g_eslot;
        g_xi = g_kb_xi;
        g_eslot = g_kbd_slot;
        g_ctrl_quiet = 1;
        (void)ctrl_xfer(0x21, USB_HID_SET_REPORT, 0x0200u, g_kbd_xiface, 1, &hid_leds);
        g_ctrl_quiet = 0;
        g_eslot = sv_eslot;
        g_xi = sv_xi;
    }
    /* OHCI-клавиатура: FA в oh_ed_ctrl берётся из g_oh_addr — подменяем
     * глобалы на владельца и возвращаем. */
    if (g_oh_kbd && g_oh_kb_oi >= 0) {
        int sv_oi = g_oi;
        uint8_t sv_addr = g_oh_addr;
        int sv_ls = g_oh_ls;
        uint8_t sv_mps = g_oh_ep0_mps;
        g_oi = g_oh_kb_oi;
        g_oh_addr = g_oh_kbd_usb_addr;
        g_oh_ls = g_oh_kbd_ls ? 1 : 0;
        g_oh_ep0_mps = 8;                       /* безопасный EP0 MPS */
        (void)oh_ctrl(0x21, USB_HID_SET_REPORT, 0x0200u, g_kbd_xiface, 1, &hid_leds);
        g_oh_ep0_mps = sv_mps;
        g_oh_ls = sv_ls;
        g_oh_addr = sv_addr;
        g_oi = sv_oi;
    }
}

/* ── 0.3.0: полный обход портов — каждую секунду из usbdev_tick() ─────
 * ВСЕ инстансы xHCI + ВСЕ инстансы OHCI, независимо от обстоятельств:
 * переподключения, выдёргивания и новые устройства ловятся опросом. */
void mouse_hw_hotplug_sweep(void) {
    for (int i = 0; i < XHCI_NINST; i++) {
        if (!g_xhci_live_a[i]) continue;
        g_xi = i;
        xhci_hotplug_inst();
    }
    for (int oi = 0; oi < OHCI_NINST; oi++) {   /* [#415] все 4 инстанса */
        if (g_oh_mmio_a[oi] == 0) continue;
        g_oi = oi;
        for (int p2 = 1; p2 <= (int)g_oh_nports_a[oi]; p2++)
            ohci_probe_port(p2, 0);
        if ((g_hp_tick & 63u) == 0u) ohci_rescan_unclaimed();
    }
}

/* cmdline «usbmouse» - взять USB под контроль даже без PS/2-клавы.
 * Вызывается из kernel.c ДО mouse_hw_init. */
void mouse_hw_force_usb(int force) { g_force_usb = force; }

void mouse_hw_init(int fb_w, int fb_h) {
    if (g_scanned) return;
    g_scanned = true;

    /* ГИГИЕНА СОСТОЯНИЯ OHCI (фото: в OHCI-СНИМКе МУСОР 0x32 во
     * всех полях и «регистрах» - statics OHCI-блока кто-то затёр ПОСЛЕ
     * зануления BSS, а СНИМОК поверил мусорному g_oh_mmio != 0 и напечатал
     * ерунду как «состояние контроллера»).  Явно приводим всё OHCI-
     * хозяйство в исходное состояние ДО любых сканов; сам СНИМОК теперь
     * печатается только если probe_ohci реально выполнялся (g_oh_probed). */
    /* 0.3.0→ЭТАП 9 [#468]: гигиена ПО ВСЕМ инстансам (xHCI×4 + OHCI×4:
     * было oi < 2 — statics 3-го/4-го OHCI оставались с мусором BSS
     * после чужих записей, и карта/СНИМОК показывали ерунду). */
    for (int oi = 0; oi < OHCI_NINST; oi++) {
        g_oi = oi;
        memset(oh_hcca, 0, sizeof(oh_hcca));
        memset(oh_ed_ctrl, 0, sizeof(oh_ed_ctrl));
        memset(oh_ed_intr, 0, sizeof(oh_ed_intr));
        memset(oh_ed_kbd, 0, sizeof(oh_ed_kbd));
        memset(oh_ed_blk_o, 0, sizeof(oh_ed_blk_o));
        memset(oh_ed_blk_i, 0, sizeof(oh_ed_blk_i));
        memset(oh_td, 0, sizeof(oh_td));
        memset(oh_td_i, 0, sizeof(oh_td_i));
        memset(oh_td_k, 0, sizeof(oh_td_k));
        memset(oh_td_tail, 0, sizeof(oh_td_tail));
        memset(oh_td_tail_i, 0, sizeof(oh_td_tail_i));
        memset(oh_td_blk, 0, sizeof(oh_td_blk));
    }
    g_oi = 0;
    g_oh_active = 0;   g_oh_probed = 0;   g_oh_msd = 0;
    g_oh_msd_sect = 0;  g_oh_msd_addr = 0;  g_oh_msd_ctl = 0;  g_oh_ms_ep = 1;
    g_oh_kbd = 0;       g_oh_kbd_port = 0;  g_oh_kbd_ls = 0;   g_oh_kbd_ep = 0;
    g_oh_ms_usb_addr = 0;  g_oh_kbd_usb_addr = 0;  g_oh_ms_ls = 0;
    g_oh_ms_port = 0;   g_oh_ms_oi = -1;   g_oh_kb_oi = -1;   g_oh_msd_oi = -1;
    g_oh_kbd_mps = 0;   g_oh_kbd_iface = 0; g_kbd_pkts = 0;
    /* клавиатура xHCI/роли портов - в исходное состояние */
    g_kbd_slot = 0;     g_kbd_xport = 0;    g_kbd_xiface = 0;  g_kbd_xmps = 0;
    g_kbd_xep = 1;      g_kb_xi = -1;       g_ms_xi = -1;
    for (int xi = 0; xi < XHCI_NINST; xi++) {
        g_xi = xi;
        g_eslot = 0;
        memset(g_port_role, 0, sizeof(g_port_role));
        for (int i = 0; i < 8; i++) hw_kbd_ep1[i].control = 0;
        r_kbd_ep1.idx = 0;
        r_kbd_ep1.cyc = 1;
    }
    g_xi = 0;
    memset(oh_setup, 0, sizeof(oh_setup));
    memset(oh_pkt, 0, sizeof(oh_pkt));
    memset(oh_kpkt, 0, sizeof(oh_kpkt));
    g_oh_msd_name[0] = 0;
    oh_diag_set("скан ещё не выполнялся");
    /* штампы и канарейки - в исходное состояние */
    mhw_canary_init();
    mhw_port_set(0);
    mhw_slot_set(0);

    /* (пересмотрено в ). История: если PS/2-клавиатура НЕ ответила
     * на сброс (ACK нет), клавиатура, скорее всего, внутренняя USB,
     * которую эмулирует BIOS, и старые агрессивные handoff убивали эту
     * эмуляцию («Клава отваливалась»).  Агрессивные handoff вычищены, и
     * захват xHCI клавиатуру больше не роняет: печать в Блокноте работает
     * при активной xHCI-мыши.  Скан выполняется ВСЕГДА, в любом пункте
     * GRUB (1 LIVE / 2 ДИАГНОСТИКА / 3 INSTALL). */
    if (!ps2_kbd_alive() && !g_force_usb) {
        serial_puts("MOUSEHW: PS/2-клавиатуры нет - сканирую USB всё равно\n");
        serial_puts("MOUSEHW: мышь должна работать и в режиме 1 LIVE.\n");
        oh_diag_set("PS/2-клавиатуры нет; USB-скан выполняется");
    }
    if (g_force_usb && !ps2_kbd_alive()) {
        serial_puts("MOUSEHW: режим usbmouse: USB беру под контроль, Клава\n");
        serial_puts("MOUSEHW: теперь ищется по USB (HID boot) - см. ниже.\n");
    }


    g_fb_w = fb_w;
    g_fb_h = fb_h;
    g_mouse.x = fb_w / 2;
    g_mouse.y = fb_h / 2;
    g_mouse.left = g_mouse.right = false;
    g_mouse.moved = false;
    g_mouse.clicked_left = false;
    g_mouse.clicked_right = false;
    prev_left = prev_right = false;

    serial_puts("MOUSEHW: scanning PCI for xHCI\n");

    /* ── PCI-скан xHCI (class 0x0C / sub 0x03 / prog-if 0x30) ──
     * Скан ВСЕХ 256 шин: бортовой xHCI почти всегда на шине 0, но
     * бывают платы с контроллером за мостом (шина >7 — раньше они
     * пропадали).  Чтение конфигурации несуществующих устройств
     * безопасно возвращает 0xFFFF.
 * скан НЕ прерывается на первом xHCI — раньше ранний выход
     * обрывал список «USB controllers on PCI:» на самом xHCI, и
     * EHCI/OHCI за ним (типичный AMD FCH: xHCI 0:10, OHCI/EHCI
     * 0:12.x/0:13.x) оставались невидимыми. */
    /* собираем ВСЕ xHCI-контроллеры: на реальных ПК их обычно два
     * (AMD FCH: 0:10.0 — USB3-линии гнёзд, 0:10.1 — USB2-линии ТЕХ ЖЕ
     * гнёзд).  Пробуем каждый по очереди. */
    uint8_t  xh_bus[4], xh_dev[4], xh_fn[4];
    uint16_t xh_vid[4], xh_did[4];
    int      xh_n = 0;
    /* Диагностика для Журнала ядра (F12): ВСЕ USB-контроллеры
     * (class 0x0C/03, любой prog-if: 0x00 UHCI, 0x10 OHCI, 0x20 EHCI,
     * 0x30/0x31 xHCI).  На реальном ПК без COM1 эта строка сразу
     * показывает, чем владеет мышь: если в списке только EHCI (0x20) —
     * мышь сидит на EHCI и ни один из наших USB-драйверов её не видит.
     * «Этап 2»: список — файловые статики g_usb_* (его же использует
     * mouse_hw_init_ohci после ehci_init). */
    uint8_t usb_bus[16], usb_dev[16], usb_fn[16], usb_pg[16];
    int     usb_n = 0;
    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t dev = 0; dev < 32; dev++) {
            for (uint8_t fn = 0; fn < 8; fn++) {
                uint32_t id = pci_read32((uint8_t)bus, dev, fn, 0x00);
                if ((id & 0xFFFFu) == 0xFFFFu) {
                    if (fn == 0) break;
                    continue;
                }
                uint32_t cc = pci_read32((uint8_t)bus, dev, fn, 0x08);
                if ((cc >> 24) == 0x0Cu && ((cc >> 16) & 0xFFu) == 0x03u) {
                    if (usb_n < 16) {
                        usb_bus[usb_n] = (uint8_t)bus;
                        usb_dev[usb_n] = dev;
                        usb_fn[usb_n]  = fn;
                        usb_pg[usb_n]  = (uint8_t)((cc >> 8) & 0xFFu);
                        usb_n++;
                    }
                    if (((cc >> 8) & 0xFFu) == 0x30u && xh_n < 4) {
                        xh_bus[xh_n] = (uint8_t)bus;
                        xh_dev[xh_n] = dev;
                        xh_fn[xh_n]  = fn;
                        xh_vid[xh_n] = (uint16_t)(id & 0xFFFFu);
                        xh_did[xh_n] = (uint16_t)(id >> 16);
                        xh_n++;
                    }
                }
                if (fn == 0 && !(pci_read8((uint8_t)bus, dev, fn, 0x0E) & 0x80u))
                    break;   /* однофункциональное устройство */
            }
        }
    }

    /* [#430] UHCI legacy/SMI-takeover: PCI 0xC0 (USBLEGSUP).  Канон Linux
     * uhci-hcd (uhci_reset_hc): запись 0x8F00 гасит SMI-источники BIOS и
     * RW1C-статусы — ДО первого HCRESET наших UHCI-драйверов (все они
     * запускаются позже: usb_mouse_init/usb_uhci_kbd_init/usb_init).
     * Иначе SMM продолжал владеть контроллером: эмуляция клавиатуры BIOS
     * перехватывала транзакции (QEMU прощает, реальный BIOS — нет). */
    for (int i = 0; i < usb_n; i++) {
        if (usb_pg[i] != 0x00u) continue;          /* только UHCI */
        uint32_t leg = pci_read32(usb_bus[i], usb_dev[i], usb_fn[i], 0xC0u);
        if (leg & 0xCF00u) {
            serial_puts("MOUSEHW: UHCI ");
            serial_puthex((uint32_t)usb_bus[i]); serial_puts(":");
            serial_puthex((uint32_t)usb_dev[i]); serial_puts(".");
            serial_puthex((uint32_t)usb_fn[i]);
            serial_puts(": BIOS legacy support активен - снимаю (USBLEGSUP)\n");
        }
        pci_write16(usb_bus[i], usb_dev[i], usb_fn[i], 0xC0u, 0x8F00u);
    }

    /* Экранная сводка (приветственный диалог, desktop.c) — строится
     * из тех же данных, что и serial-строка ниже. */
    usb_diag_build(usb_bus, usb_dev, usb_fn, usb_pg, usb_n);

    /* Диагностическая строка — видна в Журнале ядра (F12) без COM1. */
    serial_puts("MOUSEHW: USB controllers on PCI:");
    if (usb_n == 0) serial_puts(" none");
    for (int i = 0; i < usb_n; i++) {
        serial_puts(" ");
        serial_puthex((uint32_t)usb_bus[i]);
        serial_puts(":");
        serial_puthex((uint32_t)usb_dev[i]);
        serial_puts(".");
        serial_puthex((uint32_t)usb_fn[i]);
        serial_puts("(");
        serial_puthex((uint32_t)usb_pg[i]);
        serial_puts(")");
    }
    if (usb_n == 16) serial_puts(" ...");   /* [#415] список полон — дальше могло быть ещё */
    serial_puts("\n");

    if (xh_n == 0) {
        /* Раньше здесь был ранний return без force_usb: на классических
         * чипсетах БЕЗ xHCI (Intel UHCI+EHCI, старый AMD OHCI+EHCI)
         * EHCI-handoff и probe_ohci не вызывались ВООБЩЕ - USB-мышь и
         * OHCI-клавиатура не распознавались никогда.  QEMU piix3 это не
         * задевает (там нет ни EHCI, ни OHCI - циклы ниже молчат). */
        serial_puts("MOUSEHW: no xHCI controller - ищу EHCI/OHCI (клава/мышь 2.0/1.x)\n");
        oh_diag_set("xHCI нет - скан продолжается на EHCI/OHCI");
    }
    serial_puts("MOUSEHW: xHCI candidates: ");
    serial_puthex((uint32_t)xh_n);
    serial_puts("\n");
    /* 0.3.2: xHCI-BIOS-handoff ДО пробинга: без снятия SMM-владения
     * MMIO-операции на реальном железе работают «как повезёт» — а
     * «порты 2.0» на современных платах часто висят именно на xHCI. */
    for (int xi = 0; xi < xh_n; xi++)
        xhci_handoff(xh_bus[xi], xh_dev[xi], xh_fn[xi]);

    for (int xi = 0; xi < xh_n && xi < XHCI_NINST; xi++) {
        /* 0.3.0: зондируем ВСЕ xHCI — у AMD их два (0:10.0/0:10.1):
         * мышь и клавиатура могут достаться РАЗНЫМ контроллерам; ранний
         * выход на первой найденной мыши ломал «оба в порту 3.0». */
        (void)probe_xhci(xi, xh_bus[xi], xh_dev[xi], xh_fn[xi],
                         xh_vid[xi], xh_did[xi]);
    }
    serial_puts("MOUSEHW: xHCI scan done (все контроллеры; опрос каждую секунду)\n");

    /* «ЭТАП 2» (порядок — решало всё): EHCI-handoff и зонд OHCI-компаньонов
     * РАНЬШЕ выполнялись здесь, ДО ehci_init() из kernel.c.  Полноценный
     * EHCI-драйвер потом ставил CONFIGFLAG=1 и ЗАБИРАЛ порты обратно у
     * компаньонов — уже перечисленная мышь срывалась, а повторное
     * обнаружение зависело от воли CSC.  Теперь mouse_hw_init() — только
     * xHCI, а OHCI-компаньоны зондит mouse_hw_init_ohci() СТРОГО ПОСЛЕ
     * ehci_init() (kernel.c): EHCI уже раздал LS/FS-порты компаньонам,
     * HS-устройства перечислил сам — компаньоны зондятся в финальном
     * состоянии, их никто больше не дёргает. */
    for (int i = 0; i < usb_n && i < 16; i++) {
        g_usb_bus[i] = usb_bus[i];
        g_usb_dev[i] = usb_dev[i];
        g_usb_fn[i]  = usb_fn[i];
        g_usb_pg[i]  = usb_pg[i];
    }
    g_usb_n = usb_n;
    serial_puts("MOUSEHW: xHCI-часть готова; OHCI-компаньоны - после ehci_init()\n");
}

/* ═══ «ЭТАП 2»: зонд OHCI-компаньонов — вызывается из kernel.c СТРОГО ═════
 * ПОСЛЕ ehci_init(): EHCI уже взял порты 2.0, раздал LS/FS компаньонам
 * (Port Owner) и перечислил HS-устройства сам.  Компаньоны зондятся в
 * финальном состоянии — CONFIGFLAG больше не «машет» портами туда-
 * сюда под уже работающими устройствами.  Порядок в kernel.c:
 *   mouse_hw_init() → ehci_init() → mouse_hw_init_ohci() → UHCI. */
void mouse_hw_init_ohci(void) {
    if (g_oh_scanned) return;
    g_oh_scanned = true;

    if (g_usb_n == 0) {
        serial_puts("MOUSEHW: OHCI: скан PCI не выполнялся (xHCI-часть не звали?)\n");
        return;
    }
    {
        int oh_n = 0;
        for (int i = 0; i < g_usb_n && oh_n < OHCI_NINST; i++)
            if (g_usb_pg[i] == 0x10u) {
                /* 0.3.0: ОБА OHCI-компаньона живут ОДНОВРЕМЕННО (AMD FCH:
                 * 0:12.0 и 0:13.0 — клавиатура и мышь из портов 2.0 часто
                 * распределяются между ними).  Раньше второй зонд затирал
                 * состояние первого.
                 * [#415] инстансов теперь 4 (до 4-5 OHCI на AMD FCH —
                 * гнёзда 6-7 раньше были невидимы).
                 * [#419] PCI-координаты ДО проба: не поднялся — честная
                 * строка в карте вместо тихого исчезновения. */
                g_oh_pci_a[oh_n][0] = g_usb_bus[i];
                g_oh_pci_a[oh_n][1] = g_usb_dev[i];
                g_oh_pci_a[oh_n][2] = g_usb_fn[i];
                (void)probe_ohci(oh_n, g_usb_bus[i], g_usb_dev[i], g_usb_fn[i]);
                if (g_oh_mmio_a[oh_n] == 0) {
                    g_oh_dead_a[oh_n] = 1;
                    serial_puts("MOUSEHW: OHCI#");
                    { char nb[8]; serial_puts(itoa((uint32_t)(oh_n + 1), nb, 10)); }
                    serial_puts(" (PCI ");
                    { char nb[8]; serial_puts(itoa((uint32_t)g_usb_bus[i], nb, 10)); }
                    serial_puts(":");
                    { char nb[8]; serial_puts(itoa((uint32_t)g_usb_dev[i], nb, 10)); }
                    serial_puts(".");
                    { char nb[8]; serial_puts(itoa((uint32_t)g_usb_fn[i], nb, 10)); }
                    serial_puts(") не поднялся - попадёт в карту как мёртвый\n");
                }
                oh_n++;
            }
        if (oh_n == 0)
            serial_puts("MOUSEHW: OHCI-компаньонов нет (машина без 2.0-компаньонов)\n");
    }
    serial_puts("MOUSEHW: USB scan done (xHCI + EHCI + OHCI)\n");
    usbdev_dump("итог скана");
}

/* ── обработка ОДНОГО порта xHCI (скан при загрузке + hotplug) ──────────────
 * «Обнаружено новое устройство!» печатается на каждое подключение.
 * Возвращает 1, если мышь на этом порту перечислена. */
static int xhci_try_port(int p) {
    uint32_t pa = port_addr(p);
    uint32_t v = mmr32(pa);
    if (!(v & PORTSC_CCS)) return 0;
    serial_puts("MOUSEHW: обнаружено новое устройство! (xHCI порт ");
    serial_puthex((uint32_t)p);
    serial_puts(")\n");
    /* change-биты ЗДЕСЬ не чистим — лишняя запись PORTSC роняет линк
     * AMD; сброс сам поднимет порт. */

    if (!(v & PORTSC_PED)) {
        int      lines = 0;
        uint32_t r = port_reset_trace(p, pa, false, 600, &lines);
        if (!r) r = port_reset_trace(p, pa, true, 1000, &lines);
        if (!r) r = port_reset_trace(p, pa, false, 600, &lines);
        /* при УСПЕШНОМ сбросе (r содержит PED) НЕ перечитываем и
         * НЕ пишем PORTSC ВООБЩЕ (AMD применяет PLS-биты без LWS). */
        if (r) {
            v = r;
        } else {
            uint32_t v2 = mmr32(pa);
            mmw32(pa, (v2 & 0xFFFFu) | PORTSC_CHG);  /* очистить change */
            v = mmr32(pa);
        }
        serial_puts("MOUSEHW: p");
        serial_puthex((uint32_t)p);
        serial_puts(": reset finished, raw=");
        serial_puthex(v);
        serial_puts("\n");
    }
    if (!(v & PORTSC_CCS) || !(v & PORTSC_PED)) return 0;

    uint32_t speed = (v >> PORTSC_SPEED_SH) & 0xFu;
    if (speed < PORT_SPEED_LOW || speed > PORT_SPEED_SUPER) return 0;

    /* 0.3.0: ПОРТ представляется диспетчеру — [#410] с ВЕРНЫМ типом:
     * low/full/high-скорость физически бывает только на USB2-порту
     * xHCI (SuperSpeed — на USB3). Раньше все порты xHCI звались
     * «USB 3.0» — фото: клавиатура speed=Low представилась как 3.0. */
    usbdev_ask_port(xhci_bus_label(),
                    speed >= PORT_SPEED_SUPER ? 3 : 2, (int)p);

    serial_puts("MOUSEHW: port ");
    serial_puthex((uint32_t)p);
    serial_puts(" speed=");
    serial_puthex(speed);
    serial_puts(" ("); serial_puts(spd_name(speed)); serial_puts(")\n");

    /* порт ЖИВОЙ мыши защищён: перечисление чужого порта временно
     * занимает g_port, при неудаче/не-мыши порт возвращается. */
    int sv_port = g_port;
    mhw_port_set(p);   /* под штампом */
    /* зачистка зависшего слота: после deactivate по ошибкам
     * передачи слот ОСТАЁТСЯ включённым - Enable Slot тогда падает
 * (мест нет) и мышь больше не находится. зачищаем слот
     * ПЕРЕЧИСЛЕНИЯ (g_eslot) - живую мышь (g_slot) и клавиатуру
     * (g_kbd_slot) hotplug больше не глушит. */
    if (g_eslot != 0) hw_disable_slot();
    int ok = hw_enumerate((uint8_t)p, speed);
    if (!ok) mhw_port_set(sv_port); /* +вернуть порт под штампом */
    if (ok && p < (int)sizeof(g_port_role)) g_port_role[p] = PR_MOUSE;
    /* [#426] привязка КЛАВИАТУРЫ — тоже УСПЕХ порта: раньше kbd-биндинг
     * считался «неудачей» (g_hp_fails++), и после 3 «неудач» полный
     * ре-пробинг контроллера СБРАСЫВАЛ живую клавиатуру этого xHCI. */
    if (!ok && g_kbd_slot != 0 && g_kb_xi == g_xi && g_kbd_xport == p)
        ok = 1;
    hw_kbd_arm_again();   /* [#456] события клавы съели ожидания команд */
    hw_arm_mouse_again();   /* события мыши съели ожидания - пере-arm */
    return ok;
}

/* ── зондирование ОДНОГО xHCI-контроллера ──────────────────────────────────
 * На реальных ПК xHCI обычно два (AMD FCH: 0:10.0 — USB3-линии гнёзд,
 * 0:10.1 — USB2-линии ТЕХ ЖЕ гнёзд).  Мышь почти всегда USB2 — её D+/D-
 * могут сидеть на ВТОРОМ контроллере, который раньше не зондировался.
 * Возвращает 1, если мышь найдена и перечислена; иначе останавливает
 * контроллер (не оставляет DMA) и возвращает 0. */
static int probe_xhci(int xi, uint8_t xbus, uint8_t xdev, uint8_t xfn,
                      uint16_t xvid, uint16_t xdid) {
    if (xi < 0 || xi >= XHCI_NINST) return 0;
    g_xi = xi;   /* все алиасы движка — на ЭТОТ контроллер */
    /* запоминаем, КАКОЙ контроллер зондировали - самовосстановление
     * hotplug делает полный ре-пробинг по этим координатам */
    g_xh_bus = xbus; g_xh_dev = xdev; g_xh_fn = xfn;
    g_xh_vid = xvid; g_xh_did = xdid; g_xh_have = 1;
    /* F3: ре-пробинг (hotplug: 3 неудачи -> xhci_halt_now + probe)
    * стартует с чистого листа. Раньше g_xhci_live оставался «1» от
     * прошлой жизни, если ниже контроллер не сбросился/не стартовал
     * ("reset failed"/"did not start") — poll/hotplug продолжали
     * ходить по мёртвому MMIO и читать мусор 0xFFFFFFFF как PORTSC. */
    g_xhci_live = 0;
    /* 0.3.0: буферы У КАЖДОГО контроллера свои — зонд второго больше
     * НЕ сносит клавиатуру первого («оба в 3.0»).  Гашим только
     * устройства, живущие на ЭТОМ инстансе. */
    if (g_kb_xi == xi && g_kbd_slot != 0) {
        serial_puts("MOUSEHW: клавиатура ЭТОГО xHCI сброшена (ре-пробинг)\n");
        g_kbd_slot = 0;
        g_kbd_xport = 0;
        g_kb_xi = -1;
        usbdev_release(USBDEV_ROLE_KBD, "xhci re-probe");
    }
    if (g_ms_xi == xi && g_slot != 0)
        hw_deactivate("xhci re-probe");
    /* [LOGS] ЭТАП 11: MSD этого инстанса тоже гасим — полный ре-пробинг
     * сбрасывает контроллер, слот и кольца больше недействительны */
    if (g_xm_xi == xi) {
        g_xm_slot = 0;
        g_xm_xi   = -1;
        serial_puts("MOUSEHW: накопитель ЭТОГО xHCI сброшен (ре-пробинг)\n");
    }
    serial_puts("MOUSEHW: xHCI at ");
    serial_puthex(xbus); serial_puts(":");
    serial_puthex(xdev); serial_puts(".");
    serial_puthex(xfn); serial_puts(" (");
    serial_puthex(xvid); serial_puts(":");
    serial_puthex(xdid); serial_puts(")\n");

    /* ── BAR0 (MMIO): 32-битный адрес строго ниже 4 ГБ ── */
    uint32_t bar_lo = pci_read32(xbus, xdev, xfn, 0x10);
    uint32_t bar_hi = pci_read32(xbus, xdev, xfn, 0x14);
    if (bar_lo & 1u) {
        serial_puts("MOUSEHW: BAR0 is I/O space, skip\n");
        return 0;
    }
    uint32_t type   = (bar_lo >> 1) & 0x3u;
    if (type == 0x1u || (type == 0x2u && bar_hi != 0)) {
        serial_puts("MOUSEHW: unsupported BAR0 (I/O or >4GB), skip\n");
        return 0;
    }
    g_mmio = bar_lo & ~0xFu;
    if (g_mmio < 0x1000u) {
        serial_puts("MOUSEHW: bad BAR0, skip\n");
        return 0;
    }

    /* Memory space + bus master */
    uint32_t cmd = pci_read32(xbus, xdev, xfn, 0x04);
    pci_write32(xbus, xdev, xfn, 0x04, cmd | 0x6u);

    /* ── Разбор капабилити ── */
    uint32_t caplen = mmr32(g_mmio) & 0xFFu;
    if (caplen < 0x20u || caplen > 0x400u) caplen = 0x40u;
    g_op  = g_mmio + caplen;
    g_rts = g_mmio + (mmr32(g_mmio + XHC_CAP_RTSOFF) & ~0x1Fu);
    g_db  = g_mmio + (mmr32(g_mmio + XHC_CAP_DBOFF) & ~0x3u);
    uint32_t hcs1 = mmr32(g_mmio + XHC_CAP_HCSPARAMS1);
    uint32_t hcs2 = mmr32(g_mmio + XHC_CAP_HCSPARAMS2);
    uint32_t hcc1 = mmr32(g_mmio + XHC_CAP_HCCPARAMS1);
    g_maxports = (uint8_t)((hcs1 >> 24) & 0xFFu);   /* [#458] биты [31:24] целиком */
    g_csz      = (hcc1 & (1u << 2)) ? 64u : 32u;
    uint32_t scratch = ((hcs2 >> 16) & 0x3E0u) | ((hcs2 >> 27) & 0x1Fu);

    serial_puts("MOUSEHW: mmio=");
    serial_puthex(g_mmio);
    serial_puts(" ports=");
    serial_puthex(g_maxports);
    serial_puts(" csz=");
    serial_puthex(g_csz);
    serial_puts(" scratch=");
    serial_puthex(scratch);
    serial_puts("\n");

    /* сырые капабилити — сразу видно раскладку HCSPARAMS/HCCPARAMS:
     * ports печатается из битов 30:24 hcs1, и на живом чипе это стоит
     * проверить по полному значению (в QEMU раскладка стандартная). */
    serial_puts("MOUSEHW: caps caplen=");
    serial_puthex(caplen);
    serial_puts(" hcs1=");
    serial_puthex(hcs1);
    serial_puts(" hcs2=");
    serial_puthex(hcs2);
    serial_puts(" hcc1=");
    serial_puthex(hcc1);
    serial_puts("\n");

    if (g_maxports == 0 || scratch > 32) {
        serial_puts("MOUSEHW: bad controller params, skip\n");
        return 0;
    }

    /* [#410] Протокол КАЖДОГО порта — Supported Protocol Capability
     * (Extended Capabilities, ID 2): major=2 → USB2-порт (LS/FS/HS),
     * major=3 → USB3-порт (SuperSpeed). xECP — биты [31:16] HCCPARAMS1,
     * в ДВОРДАХ от базовой капабилити (сверено дамном QEMU: HCCPARAMS1
     * 0x00087001 → xECP=8 → кэпы на +0x20: ID=2 major=2 poff=5 pcnt=4
     * «USB2: 5-8», следом major=3 «USB3: 1-4»). Next в dword0[15:8]
     * тоже в двордах. Полевая сверка: Linux xhci-ext-caps.h. */
    memset(g_xh_pproto_a[xi], 0, sizeof(g_xh_pproto_a[xi]));
    {
        uint32_t xecp = (hcc1 >> 16) & 0xFFFFu;     /* дворды от базы */
        if (xecp) xecp <<= 2;                       /* → байты          */
        if (xecp && xecp < 0x10000u) {
            uint32_t off = xecp;
            for (int guard = 0; guard < 64 && off && off < 0x10000u; guard++) {
                uint32_t w0  = mmr32(g_mmio + off);
                uint32_t id  = w0 & 0xFFu;
                uint32_t nxt = (w0 >> 8) & 0xFFu;
                if (id == 2u) {                    /* Supported Protocol */
                    uint32_t w2    = mmr32(g_mmio + off + 8u);
                    uint32_t major = w0 >> 24;
                    uint32_t poff  = w2 & 0xFFu;        /* первый порт  */
                    uint32_t pcnt  = (w2 >> 8) & 0xFFu; /* сколько портов*/
                    if (poff >= 1 && pcnt >= 1 &&
                        poff <= XH_NPORT_MAX && pcnt <= XH_NPORT_MAX &&
                        poff + pcnt - 1u <= XH_NPORT_MAX) {
                        for (uint32_t q = poff; q < poff + pcnt; q++)
                            g_xh_pproto_a[xi][q] =
                                (uint8_t)(major >= 3u ? 3u : 2u);
                    }
                }
                if (nxt == 0u) break;
                off += nxt << 2;
            }
        }
        /* компактная строка таблицы протоколов — сразу в журнал */
        serial_puts("MOUSEHW: протоколы портов xHCI#");
        { char nbi[8]; serial_puts(itoa((uint32_t)(xi + 1), nbi, 10)); }
        serial_puts(": ");
        int printed = 0;
        for (int proto = 3; proto >= 2; proto--) {
            int first = -1, last = -1;
            for (int q = 1; q <= XH_NPORT_MAX; q++) {
                if (g_xh_pproto_a[xi][q] == proto) {
                    if (first < 0) first = q;
                    last = q;
                }
            }
            if (first < 0) {
                serial_puts(proto == 3 ? "USB3: нет" : "; USB2: нет");
            } else {
                if (printed) serial_puts("; ");
                serial_puts(proto == 3 ? "USB3: " : "USB2: ");
                { char nbi[8]; serial_puts(itoa((uint32_t)first, nbi, 10)); }
                if (last != first) {
                    serial_puts("-");
                    { char nbi[8]; serial_puts(itoa((uint32_t)last, nbi, 10)); }
                }
            }
            printed = 1;
        }
        serial_puts("\n");
    }

    /* ── BIOS hand-off, затем halt → HCRST ── */
    hw_bios_handoff();
    if (!hw_reset_hc()) {
        serial_puts("MOUSEHW: controller reset failed, skip\n");
        return 0;
    }

    /* ── Структуры: DCBAA (+scratchpad), CRCR, ERST, ERDP ── */
    memset(hw_dcbaa, 0, sizeof(hw_dcbaa));
    memset(hw_devctx, 0, sizeof(hw_devctx));
    memset(hw_erst, 0, sizeof(hw_erst));
    for (int i = 0; i < 16; i++) { hw_cmd[i].control = 0; hw_evt[i].control = 0; }
    for (int i = 0; i < 8; i++)  { hw_ep0[i].control = 0; hw_ep1[i].control = 0; }
    for (int i = 0; i < 8; i++) hw_kbd_ep1[i].control = 0;
    /* 0.3.0: кольца указывают на память ЭТОГО инстанса (без этого
     * .t остаётся мусором и TRB уезжают по случайному адресу — QEMU
     * читал пустой command ring, «enable slot failed cc=0») */
    g_r_cmd[xi].t = hw_cmd_a[xi];      g_r_cmd[xi].n = 16;
    g_r_cmd[xi].idx = 0;               g_r_cmd[xi].cyc = 1;   /* !cycle */
    g_r_ep0[xi].t = hw_ep0_a[xi];      g_r_ep0[xi].n = 8;
    g_r_ep1[xi].t = hw_ep1_a[xi];      g_r_ep1[xi].n = 8;
    g_r_kbd_ep1[xi].t = hw_kbd_ep1_a[xi]; g_r_kbd_ep1[xi].n = 8;
    g_r_msd_o[xi].t = hw_msd_out_a[xi];  g_r_msd_o[xi].n = 8;
    g_r_msd_i[xi].t = hw_msd_in_a[xi];   g_r_msd_i[xi].n = 8;
    r_msd_o.idx = 0;  r_msd_o.cyc = 1;
    r_msd_i.idx = 0;  r_msd_i.cyc = 1;
    r_kbd_ep1.idx = 0;
    r_kbd_ep1.cyc = 1;
    r_ep0.idx = 0;                     /* enqueue = fresh dequeue */
    r_ep0.cyc = 1;
    r_ep1.idx = 0;
    r_ep1.cyc = 1;
    memset(g_port_role, 0, sizeof(g_port_role));
    er_idx = 0;
    er_cyc = 1;

    uint32_t maxslots = hcs1 & 0xFFu;
    memset(hw_sparr, 0, sizeof(hw_sparr));
    for (uint32_t i = 0; i < scratch && i < 32; i++)
        hw_sparr[i] = (uint64_t)vphys(hw_scratch[i]);
    /* Спецификация §6.1: DCBAA[0] = адрес МАССИВА scratchpad-указателей */
    hw_dcbaa[0] = scratch ? (uint64_t)vphys(hw_sparr) : 0;
    for (uint32_t i = 1; i <= maxslots && i < 256; i++) hw_dcbaa[i] = 0;

    mmw64(g_op + XHC_OP_DCBAAP, (uint64_t)vphys(hw_dcbaa));
    mmw32(g_op + XHC_OP_CONFIG, maxslots);   /* MaxSlotsEn */

    mmw64(g_op + XHC_OP_CRCR, (uint64_t)vphys(hw_cmd) | 1u);  /* RCS=1 */

    /* Контрольная считка: QEMU возвращает записанное (crcr = адрес|1,
     * dcbaap = адрес DCBAA).  Если смещения неверны — записи уходят мимо,
     * здесь будет 0, и любая команда «виснет» (enable slot cc=0). */
    serial_puts("MOUSEHW: regs crcr=");
    serial_puthex(mmr32(g_op + XHC_OP_CRCR));
    serial_puts(" dcbaap=");
    serial_puthex(mmr32(g_op + XHC_OP_DCBAAP));
    serial_puts("\n");
    /* физические адреса буферов контекстов. Если output-writeback
     * контроллера попадает в НАШ input-контекст (в дампе после отказа
     * ep0_1 отличается от «addr wr»), смотрим сюда: пересечение
     * физ адресов inctx/devctx = ошибка трансляции адресов. */
    serial_puts("MOUSEHW: phys inctx=");
    serial_puthex(vphys(hw_inctx));
    serial_puts(" devctx=");
    serial_puthex(vphys(hw_devctx));
    serial_puts(" dcbaa=");
    serial_puthex(vphys(hw_dcbaa));
    serial_puts("\n");

    /* ERST-запись — ровно 16 байт: addr(0-7), rsvd(8-11), size(12-15).
     * Размер на байтах 12-15, НЕ по смещению 16 (QEMU при size<16
     * вызывает xhci_die — event ring мёртв, «enable slot failed cc=0»). */
    hw_erst[0] = (uint64_t)vphys(hw_evt);
    hw_erst[1] = 16;
    hw_erst[2] = 0;
    hw_erst[3] = 0;
    mmw32(g_rts + XHC_IR0 + XHC_IR_ERSTSZ, 1);
    mmw64(g_rts + XHC_IR0 + XHC_IR_ERSTBA, (uint64_t)vphys(hw_erst));
    mmw64(g_rts + XHC_IR0 + XHC_IR_ERDP,
          (uint64_t)vphys(hw_evt) | ERDP_EHB);
    mmw32(g_rts + XHC_IR0 + XHC_IR_IMAN, 0);   /* прерывания не используем */


    /* ── Запуск контроллера (до любого doorbell!) ──
     * [#420] dma_sync ПЕРЕД RS: DCBAA/скратчпады/CRCR/ERST писались в
     * кэшируемую память; на WB-кэше без MTRR (AMD FCH) контроллер читал
     * из ОЗУ НУЛИ BSS → «перечисление работает через раз». QEMU кэша
     * не имеет — T6/T7/T8 молчали. */
    dma_sync();
    mmw32(g_op + XHC_OP_USBCMD, USBCMD_RS);
    int t = 2000;
    while ((mmr32(g_op + XHC_OP_USBSTS) & USBSTS_HCH) && t-- > 0)
        hw_delay(1000);
    if (t <= 0) {
        serial_puts("MOUSEHW: controller did not start, skip\n");
        return 0;
    }

    /* ── Питание портов + очистка change-битов ── */
    for (int p = 1; p <= (int)g_maxports; p++) {
        uint32_t pa = port_addr(p);
        uint32_t v = mmr32(pa);
        mmw32(pa, (v & 0xFFFFu) | PORTSC_PP | PORTSC_CHG);
    }

    /* сырые PORTSC всех портов ОДНОЙ строкой (на текстлог-экране
     * важен каждый ряд при переборе нескольких контроллеров): если мышь
     * сидит на порту этого контроллера, CCS/PED/PP и скорость видны сразу. */
    serial_puts("MOUSEHW: portsc:");
    for (int dp = 1; dp <= (int)g_maxports; dp++) {
        serial_puts(" p");
        serial_puthex((uint32_t)dp);
        serial_puts("=");
        serial_puthex(mmr32(port_addr(dp)));
    }
    serial_puts("\n");

    /* USB3-линк может завершить тренировку с задержкой — даём до
     * ~1.5с на появление CCS на любом порту (если сразу пусто). */
    {
        int any0 = 0;
        for (int dp = 1; dp <= (int)g_maxports; dp++)
            if (mmr32(port_addr(dp)) & PORTSC_CCS) any0 = 1;
        if (!any0) {
            int w = 1500;
            while (w-- > 0) {
                int any = 0;
                for (int dp = 1; dp <= (int)g_maxports; dp++)
                    if (mmr32(port_addr(dp)) & PORTSC_CCS) any = 1;
                if (any) {
                    serial_puts("MOUSEHW: attach detected after wait\n");
                    break;
                }
                hw_delay(1000);
            }
        }
    }

    /* ── Опрос портов (через xhci_try_port). Контроллер ПОСЛЕ скана
     * НЕ останавливаем: он продолжает работать, hotplug ловит подключение
     * мыши позже (простаивающий xHCI без активных слотов ничего не DMA-ит) */
    g_xhci_live = 1;
    /* Windows-модель - опрашиваем ВСЕ порты контроллера. Раньше
     * скан обрывался на первой мыши (return 1): клавиатура на порту с
     * бОльшим номером не находилась («клава в 3.0 не работает»).  Мышь
 * перечисляется В СВОЙ ХОД (путь не изменён), скан продолжается:
     * клавиатура на любом порту привяжется после мыши - это безопасно,
     * у мыши личный контекст (hw_mouse_devctx), кольца сброшены. */
    {
        int mouse_found = 0;
        for (int p = 1; p <= (int)g_maxports; p++)
            if (xhci_try_port(p)) mouse_found = 1;
        if (mouse_found) return 1;
    }
    serial_puts("MOUSEHW: no xHCI mouse found (left running for hotplug)\n");
    return 0;
}

/* ── Public: poll event ring (никогда не блокирует) ───────────────────────── */
void mouse_hw_poll(mouse_state_t *out) {
    g_mouse.clicked_left  = false;
    g_mouse.clicked_right = false;
    g_mouse.moved         = false;

    /* 0.3.0: события ВСЕХ живых xHCI-инстансов — мышь и клавиатура могут
     * жить на РАЗНЫХ контроллерах (AMD FCH: 0:10.0/0:10.1). */
    for (int i = 0; i < XHCI_NINST; i++) {
        if (!g_xhci_live_a[i]) continue;
        g_xi = i;
        if (g_active && g_slot != 0 && g_ms_xi == i) {
            mhw_port_check("poll");
            mhw_slot_check("poll");
        }
        if ((g_active && g_slot != 0 && g_ms_xi == i) ||
            (g_kbd_slot != 0 && g_kb_xi == i)) {
            uint64_t p; uint32_t st, ctl;
            while (evt_get(&p, &st, &ctl)) {
                uint32_t type = (ctl >> 10) & 0x3Fu;
                if (type == TRB_EV_TRANSFER &&
                    g_kbd_slot != 0 && g_kb_xi == i &&
                    (ctl >> 24) == (uint32_t)g_kbd_slot &&
                    ((ctl >> 16) & 0x1Fu) == (2u * (uint32_t)lk_xep() + 1u)) {   /* [#474] EP — из СНИМКА живой клавы */
                    uint32_t cc  = st >> 24;
                    uint32_t rem = st & 0xFFFFFFu;
                    if (cc == CC_SUCCESS || cc == CC_SHORT_PACKET) {
                        int act = (int)lk_xmps() - (int)rem;   /* [#474] MPS — из СНИМКА */
                        if (act < 0) act = 0;
                        if (act > (int)sizeof(hw_kbd_pkt))
                            act = (int)sizeof(hw_kbd_pkt);
                        if (act >= 1)              /* [#475] нулевая длина — НЕ живые данные */
                            oh_kbd_process(hw_kbd_pkt, act, "xHCI");
                        else
                            zl_note("xHCI клава: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)");
                    }   /* ошибки клавиатуры молча: клава не должна сыпать лог */
                    hw_kbd_arm();   /* пере-постановка TRB (и после ошибки) */
                } else if (type == TRB_EV_TRANSFER &&
                    g_active && g_slot != 0 && g_ms_xi == i &&
                    (ctl >> 24) == (uint32_t)g_slot &&
                    ((ctl >> 16) & 0x1Fu) == (2u * (uint32_t)lm_xep() + 1u)) {   /* [#424] EP мыши — любой; [#474] — из СНИМКА живой мыши */
                    uint32_t cc  = st >> 24;
                    uint32_t rem = st & 0xFFFFFFu;
                    if (cc == CC_SUCCESS || cc == CC_SHORT_PACKET) {
                        int act = (int)lm_mps() - (int)rem;    /* [#474] MPS — из СНИМКА (TRB вооружён им) */
                        if (act < 0) act = 0;
                        if (act > (int)sizeof(hw_pkt)) act = (int)sizeof(hw_pkt);
                        if (act < 1) {
                            /* [#475] Ситуация B: длина 0 — не считаем живостью */
                            zl_note("xHCI мышь: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)");
                        } else {
                        g_packets++;
                        usbdev_note_report(USBDEV_ROLE_MOUSE);   /* [#462] живость для КАРТЫ ГНЕЗД */
                        if (g_packets == 1) {
                            serial_puts("MOUSEHW: first report (");
                            serial_puthex((uint32_t)act);
                            serial_puts(" bytes):");
                            for (int q = 0; q < act; q++) {
                                serial_puts(" ");
                                serial_puthex(hw_pkt[q]);
                            }
                            serial_puts("\n");
                        }
                        if (lm_absolute())    hw_tablet_report(hw_pkt, act);   /* [#474] маршрутизация — по СНИМКУ */
                        else if (lm_rpt_ok()) hw_hid_report(hw_pkt, act);
                        else                  hw_boot_report(hw_pkt, act);
                        }
                    } else {
                        serial_puts("MOUSEHW: transfer error cc=");
                        serial_puthex(cc);
                        serial_puts("\n");
                        if (++g_errors >= 8) {
                            hw_deactivate("transfer errors");
                            break;
                        }
                    }
                    if (g_active) hw_arm();   /* пере-постановка TRB */
                } else if (type == TRB_EV_PORT_CHANGE) {
                    uint32_t port = (uint32_t)(p >> 24) & 0xFFu;
                    if (g_active && g_ms_xi == i && port == (uint32_t)g_port) {
                        uint32_t ps = mmr32(port_addr((int)port));
                        if (ps & (1u << 17)) {                 /* именно CSC */
                            if (ps & PORTSC_CCS) {
                                serial_puts("MOUSEHW: мышь переподключена (xHCI порт ");
                                serial_puthex(port);
                                serial_puts(") - пере-перечисляю\n");
                                hw_deactivate("device replugged");
                            } else {
                                serial_puts("MOUSEHW: USB: устройство отключено (xHCI порт ");
                                serial_puthex(port);
                                serial_puts(")\n");
                                hw_deactivate("device detached");
                            }
                        } else {
                            mmw32(port_addr((int)port),
                                  (ps & 0xFFFFu) | PORTSC_CHG);
                        }
                    }
                }
                /* Command Completion во время работы: игнорируем */
            }
        } else {
            /* F3: живой контроллер БЕЗ наших устройств (мышь на другой
             * шине/драйвере) всё равно шлёт события — Port Status
             * Change от подключений. Раньше они копились нечитанными:
             * ERDP стоял на месте, после 16 событий event ring
             * переполнялся (Event Ring Full) и контроллер замолчал
             * для будущих перечислений («гонка CSC»). Дренаж двигает
             * ERDP; сами подключения ловит hotplug-обход PORTSC. */
            uint64_t p; uint32_t st, ctl;
            while (evt_get(&p, &st, &ctl)) { /* потребить и забыть */ }
        }
    }

    if (g_oh_active || g_oh_kbd) ohci_poll_reports();   /* и клавиатура */

    /* быстрые проверки портов (троттлинг кадров); ГЛАВНЫЙ секундный
     * обход — mouse_hw_hotplug_sweep() из usbdev_tick(). */
    g_hp_tick++;
    if ((g_hp_tick & 7u) == 0u)
        mouse_hw_hotplug_sweep();

    if (out) *out = g_mouse;
}

/* ── Public: опрос ТОЛЬКО клавиатурной части (никогда не блокирует) ────────
 * Вызывается из ps2_poll_key(): когда активна мышь ДРУГОГО драйвера
 * (UHCI), mouse_hw_poll не вызывается ни из ps2_poll (ранний return),
 * ни из desktop (условие) - клавиатуры OHCI/xHCI замерзали, qTD/TRB
 * копились.  Здесь опрашиваем только клаву; когда мышь mouse_hw жива,
 * всё делает mouse_hw_poll (двойной обработки нет - event ring один,
 * evt_get двигает указатель). */
void mouse_hw_kbd_poll(void) {
    /* [#467] ЭТАП 9: живость проверяем по ВСЕМ инстансам (было только
     * [0]/[1]: если подняты лишь 3-й/4-й контроллер, функция выходила
     * сразу — клавиатура на них ЗАМИРАЛА, qTD/TRB копились). */
    bool any_usb = false;
    for (int i = 0; i < XHCI_NINST && !any_usb; i++)
        if (g_xhci_live_a[i]) any_usb = true;
    for (int oi = 0; oi < OHCI_NINST && !any_usb; oi++)
        if (g_oh_mmio_a[oi] != 0) any_usb = true;
    if (!any_usb)
        return;
    if (g_active && g_slot != 0) return;      /* мышь xHCI жива: её poll сам
                                                 * разбирает и клавиатуру */
    if (g_oh_active) return;                  /* мышь OHCI жива: аналогично */

    /* 0.3.0: клавиатура может жить на ЛЮБОМ xHCI-инстансе */
    for (int i = 0; i < XHCI_NINST; i++) {
        if (!g_xhci_live_a[i]) continue;
        g_xi = i;
        if (g_kbd_slot == 0 || g_kb_xi != i) {
            /* F3: дренаж event ring на инстансах БЕЗ нашей клавиатуры
             * (см. mouse_hw_poll): ERDP обязан двигаться на каждом
             * живом контроллере, иначе накопившиеся CSC-события
             * переполняют event ring (Event Ring Full). */
            uint64_t p; uint32_t st, ctl;
            while (evt_get(&p, &st, &ctl)) { /* потребить и забыть */ }
            continue;
        }
        uint64_t p; uint32_t st, ctl;
        while (evt_get(&p, &st, &ctl)) {
            uint32_t type = (ctl >> 10) & 0x3Fu;
            if (type == TRB_EV_TRANSFER &&
                (ctl >> 24) == (uint32_t)g_kbd_slot &&
                ((ctl >> 16) & 0x1Fu) == (2u * (uint32_t)lk_xep() + 1u)) {   /* [#474] EP — из СНИМКА живой клавы */
                uint32_t cc  = st >> 24;
                uint32_t rem = st & 0xFFFFFFu;
                if (cc == CC_SUCCESS || cc == CC_SHORT_PACKET) {
                    int act = (int)lk_xmps() - (int)rem;   /* [#474] MPS — из СНИМКА */
                    if (act < 0) act = 0;
                    if (act > (int)sizeof(hw_kbd_pkt))
                        act = (int)sizeof(hw_kbd_pkt);
                    if (act >= 1)              /* [#475] нулевая длина — НЕ живые данные */
                        oh_kbd_process(hw_kbd_pkt, act, "xHCI");
                    else
                        zl_note("xHCI клава: отчёт НУЛЕВОЙ ДЛИНЫ (данных нет)");
                }
                hw_kbd_arm();   /* пере-постановка TRB (и после ошибки) */
            }
            /* прочие события без активной мыши - молча (hotplug ниже) */
        }
    }

    if (g_oh_kbd) ohci_poll_reports();       /* готовые отчёты OHCI-клавы */

    /* hotplug НОВЫХ устройств (клава/мышь), пока основная мышь
     * обслуживается другим драйвером (UHCI): секундный обход делает
     * usbdev_tick(), здесь — быстрый троттлинг кадров. */
    g_hp_tick++;
    if ((g_hp_tick & 7u) == 0u)
        mouse_hw_hotplug_sweep();
}

bool mouse_hw_present(void) { return g_present; }
bool mouse_hw_active(void)  { return g_present && g_active; }

/* USB-клавиатура OHCI (g_oh_kbd) или xHCI (g_kbd_slot) жива? */
bool mouse_hw_kbd_present(void) { return g_oh_kbd != 0 || g_kbd_slot != 0; }
