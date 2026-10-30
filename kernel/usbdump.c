/* usbdump.c — [#494] МЕГА-ЛОГ: PCI-дамп с именами, расшифровка USB-
 * дескрипторов (включая СТРОКИ), побитовые расшифровки регистров портов,
 * метки времени.  Всё через serial_puts/serial_puthex (зеркало в klog →
 * EGOROV.LOG).  Ни одной передачи на шину отсюда НЕ отправляется: только
 * чтение PCI config space и разбор УЖЕ прочитанных буферов — модуль не
 * может сломать перечисление.  Ответ «почему Linux работает везде» —
 * в шапке usbdump.h (коротко: списки не при чём; Linux повторяет сброс
 * порта до включения, не шлёт в остановленный HC и не верит висячим
 * битам — фиксы #492/#493 делают то же самое). */

#include "types.h"

#include "usbdump.h"
#include "io.h"

/* serial API живёт в kernel.c (не в общем заголовке — декларируем тут,
 * как делает timer.c). */
void serial_puts(const char *s);
void serial_puthex(uint32_t v);

/* ── маленькие помощники ─────────────────────────────────────────────── */

static void ud_pc(char c) {
    char s[2];
    s[0] = c; s[1] = 0;
    serial_puts(s);
}

/* Ротация из 8 статических буферов: itoa_ можно вызывать до 8 раз в
 * одном выражении serial_puts(a)...(b)... без порчи. */
static const char *itoa_(uint32_t v) {
    static char b[8 * 12];
    static int slot;
    char *p = b + (slot = (slot + 1) & 7) * 12;
    char t[10];
    int i = 0, j = 0;
    if (v == 0) { p[0] = '0'; p[1] = 0; return p; }
    while (v > 0 && i < 9) { t[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i > 0) p[j++] = t[--i];
    p[j] = 0;
    return p;
}

/* 2 шестнадцатеричные ЦИФРЫ без префикса (serial_puthex печатает «0x»+8 —
 * для байтов дескрипторов нужен ровно 2-значный формат). */
static void ud_hex2(uint8_t v) {
    const char *h = "0123456789ABCDEF";
    char s[3];
    s[0] = h[(v >> 4) & 0xF];
    s[1] = h[v & 0xF];
    s[2] = 0;
    serial_puts(s);
}

static void ud_hex8(const uint8_t *b, int n) {
    for (int i = 0; i < n; i++) {
        if (i) serial_puts(" ");
        ud_hex2(b[i]);
    }
}

/* ── Таблицы имён (компактная выжимка pci.ids/usb.ids) ───────────────── */

typedef struct { uint16_t vid; const char *name; } ud_vid_t;
typedef struct { uint16_t vid, pid; const char *name; } ud_did_t;

static const ud_vid_t ud_pci_vendors[] = {
    {0x1022, "AMD"},       {0x1002, "AMD/ATI"},   {0x8086, "Intel"},
    {0x1106, "VIA"},       {0x10DE, "NVIDIA"},    {0x1B21, "ASMedia"},
    {0x1033, "NEC"},       {0x10EC, "Realtek"},   {0x1102, "Creative"},
    {0x1043, "ASUS"},      {0x1458, "Gigabyte"},  {0x1462, "MSI"},
    {0x1849, "ASRock"},    {0x1B36, "Red Hat/QEMU"}, {0x8087, "Intel"},
    {0x1234, "QEMU-видео"}, {0x15AD, "VMware"},   {0x1AF4, "virtio"},
    {0x80EE, "VirtualBox"}, {0x106B, "Apple"},    {0x17EF, "Lenovo"},
    {0x413C, "Dell"},      {0x1912, "Renesas"},
};

/* USB-контроллеры и частые мосты (то, что реально встречается в ПК
 * пользователя — AMD FCH 7807/7808/7814 подтверждены EGOROV.LOG). */
static const ud_did_t ud_pci_devices[] = {
    /* AMD FCH / Hudson / Bolton / Promontory / 400-500-серии */
    {0x1022, 0x7807, "AMD FCH OHCI USB 1.1 (компаньон)"},
    {0x1022, 0x7808, "AMD FCH EHCI USB 2.0"},
    {0x1022, 0x7809, "AMD FCH xHCI USB 3.0"},
    {0x1022, 0x7812, "AMD FCH xHCI USB 3.0 (Hudson)"},
    {0x1022, 0x7814, "AMD FCH xHCI USB 3.0 (Kimpton)"},
    {0x1022, 0x7907, "AMD FCH OHCI (Promontory)"},
    {0x1022, 0x7908, "AMD FCH EHCI (Promontory)"},
    {0x1022, 0x7914, "AMD FCH xHCI (Promontory)"},
    {0x1022, 0x43B9, "AMD 400-серии xHCI"},
    {0x1022, 0x43BB, "AMD 400-серии xHCI"},
    {0x1022, 0x43C8, "AMD xHCI (400-серии)"},
    {0x1022, 0x1487, "AMD xHCI (500-серии)"},
    {0x1022, 0x1577, "AMD Ryzen Host-мост"},
    {0x1022, 0x1453, "AMD Family 17h IOMMU"},
    {0x1022, 0x1483, "AMD Starship IOMMU"},
    /* Intel: PIIX → ICH → PCH */
    {0x8086, 0x7000, "Intel PIIX3 ISA-мост"},
    {0x8086, 0x7010, "Intel PIIX3 IDE"},
    {0x8086, 0x7020, "Intel PIIX3 UHCI USB"},
    {0x8086, 0x7110, "Intel PIIX4 ISA"},
    {0x8086, 0x7112, "Intel PIIX4 UHCI"},
    {0x8086, 0x2442, "Intel ICH2 UHCI"},
    {0x8086, 0x2444, "Intel ICH2 EHCI"},
    {0x8086, 0x2482, "Intel ICH4 UHCI"},
    {0x8086, 0x2487, "Intel ICH4 EHCI"},
    {0x8086, 0x2658, "Intel ICH6 UHCI"},
    {0x8086, 0x265C, "Intel ICH6 EHCI"},
    {0x8086, 0x27C8, "Intel ICH7 UHCI"},
    {0x8086, 0x27CC, "Intel ICH7 EHCI"},
    {0x8086, 0x2830, "Intel ICH8 UHCI"},
    {0x8086, 0x2836, "Intel ICH8 EHCI #1"},
    {0x8086, 0x283A, "Intel ICH8 EHCI #2"},
    {0x8086, 0x2934, "Intel ICH9 UHCI"},
    {0x8086, 0x293A, "Intel ICH9 EHCI #1"},
    {0x8086, 0x293C, "Intel ICH9 EHCI #2"},
    {0x8086, 0x3A34, "Intel ICH10 UHCI"},
    {0x8086, 0x3A3A, "Intel ICH10 EHCI"},
    {0x8086, 0x1C26, "Intel 6-серии EHCI"},
    {0x8086, 0x1C2D, "Intel 6-серии EHCI #2"},
    {0x8086, 0x1D26, "Intel C600 EHCI"},
    {0x8086, 0x8C26, "Intel 8-серии EHCI"},
    {0x8086, 0x8C2D, "Intel 8-серии EHCI #2"},
    {0x8086, 0x8D26, "Intel C610 EHCI"},
    {0x8086, 0x8C31, "Intel 8-серии xHCI"},
    {0x8086, 0x9C26, "Intel LP EHCI"},
    {0x8086, 0x9C31, "Intel LP xHCI"},
    {0x8086, 0xA12F, "Intel 100/200-серии xHCI"},
    {0x8086, 0xA2AF, "Intel 200-серии xHCI"},
    /* VIA / ASMedia / NEC / Renesas (платы PCI-E→USB) */
    {0x1106, 0x3034, "VIA VT82xx UHCI"},
    {0x1106, 0x3104, "VIA USB 2.0 EHCI"},
    {0x1106, 0x3483, "VIA VL805 xHCI (плата)"},
    {0x1B21, 0x1042, "ASMedia ASM1042 xHCI"},
    {0x1B21, 0x1142, "ASMedia ASM1142 xHCI"},
    {0x1B21, 0x2142, "ASMedia EHCI (плата)"},
    {0x1B21, 0x3142, "ASMedia xHCI (плата)"},
    {0x1033, 0x0035, "NEC uPD720101 EHCI"},
    {0x1033, 0x0194, "NEC uPD720200 xHCI"},
    {0x1912, 0x0015, "Renesas uPD720202 xHCI (плата)"},
    /* QEMU (для сравнения логов железо/VM) */
    {0x1B36, 0x000D, "QEMU Q35 MCH"},
    {0x8086, 0x1237, "Intel 440FX (QEMU)"},
    {0x1B36, 0x0001, "QEMU IVSHMEM"},
};

/* USB-вендоры (мыши/клавы/флешки/хабы — то, что видно в наших таблицах) */
static const ud_vid_t ud_usb_vendors[] = {
    {0x046D, "Logitech"},       {0x045E, "Microsoft"},
    {0x093A, "Pixart (A4Tech и др. мыши)"},
    {0x25A7, "Areson (мыши)"},  {0x1BCF, "Delux и др. (мыши/клавы)"},
    {0x0C45, "SONiX (камеры/клавы)"}, {0x04F2, "Chicony"},
    {0x17EF, "Lenovo"},         {0x1532, "Razer"},
    {0x1B1C, "Corsair"},        {0x1038, "SteelSeries"},
    {0x0B05, "ASUS"},           {0x04B3, "IBM/Lenovo"},
    {0x05AC, "Apple"},          {0x046A, "Cherry"},
    {0x2A7A, "небрендовая клавиатура (2A7A)"},
    {0x346D, "небрендовая флешка (VendorCo)"},
    {0x0781, "SanDisk"},        {0x0951, "Kingston"},
    {0x13FE, "Phison (флешки)"},{0x090C, "Silicon Motion (флешки)"},
    {0x058F, "Alcor Micro (флешки/хабы)"},
    {0x0BDA, "Realtek (хабы/флешки)"},
    {0x2109, "VIA Labs (хабы)"},{0x1A40, "Terminus (хабы)"},
    {0x05E3, "Genesys (хабы)"}, {0x0D8C, "C-Media (звук)"},
    {0x0C76, "JMTek (флешки)"}, {0x0DD8, "Kingmax (флешки)"},
    {0x0416, "Winbond (флешки)"},{0x1EA7, "TP-Link"},
    {0x2357, "TP-Link (Wi-Fi)"},
};

const char *ud_pci_vendor(uint16_t vid) {
    for (unsigned i = 0; i < sizeof(ud_pci_vendors) / sizeof(ud_pci_vendors[0]); i++)
        if (ud_pci_vendors[i].vid == vid) return ud_pci_vendors[i].name;
    return 0;
}

const char *ud_pci_device(uint16_t vid, uint16_t pid) {
    for (unsigned i = 0; i < sizeof(ud_pci_devices) / sizeof(ud_pci_devices[0]); i++)
        if (ud_pci_devices[i].vid == vid && ud_pci_devices[i].pid == pid)
            return ud_pci_devices[i].name;
    return 0;
}

const char *ud_usb_vendor(uint16_t vid) {
    for (unsigned i = 0; i < sizeof(ud_usb_vendors) / sizeof(ud_usb_vendors[0]); i++)
        if (ud_usb_vendors[i].vid == vid) return ud_usb_vendors[i].name;
    return 0;
}

/* ── PCI config space (локальные помощники — как в kernel.c) ─────────── */

static uint32_t ud_pci_r32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg) {
    uint32_t addr = 0x80000000u | ((uint32_t)bus << 16) |
                    ((uint32_t)dev << 11) | ((uint32_t)fn << 8) | (reg & 0xFCu);
    outl(0xCF8, addr);
    return inl(0xCFC);
}

static uint8_t ud_pci_r8(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg) {
    uint32_t addr = 0x80000000u | ((uint32_t)bus << 16) |
                    ((uint32_t)dev << 11) | ((uint32_t)fn << 8) | (reg & 0xFCu);
    outl(0xCF8, addr);
    return (uint8_t)(inl(0xCFC) >> (8u * (reg & 3)));
}

static void ud_pci_class_name(uint32_t cc, char *out, int n) {
    const char *s = "?";
    uint8_t cls = (uint8_t)(cc >> 24), sub = (uint8_t)(cc >> 16);
    uint8_t prog = (uint8_t)(cc >> 8);
    int i = 0;
    switch (cls) {
    case 0x00: s = "Unclassified"; break;
    case 0x01: s = (sub == 0x06) ? "SATA" : (sub == 0x01 ? "IDE" :
                  (sub == 0x08 ? "NVMe" : "Storage")); break;
    case 0x02: s = "Network"; break;
    case 0x03: s = (sub == 0x00) ? "VGA-видео" : "Display"; break;
    case 0x04: s = "Multimedia/звук"; break;
    case 0x05: s = "Memory"; break;
    case 0x06: s = (sub == 0x00) ? "Host-мост" :
                  (sub == 0x01 ? "ISA-мост" :
                  (sub == 0x04 ? "PCI-PCI мост" :
                  (sub == 0x09 ? "CardBus" : "Мост"))); break;
    case 0x07: s = "COM-порт"; break;
    case 0x08: s = "System peripheral"; break;
    case 0x09: s = "Input"; break;
    case 0x0B: s = "Процессор"; break;
    case 0x0C:
        if (sub == 0x03) {
            s = (prog == 0x00) ? "UHCI USB 1.1" :
                (prog == 0x10) ? "OHCI USB 1.1" :
                (prog == 0x20) ? "EHCI USB 2.0" :
                (prog == 0x30) ? "xHCI USB 3.x" : "USB device";
        } else if (sub == 0x05) s = "SMBus";
        else if (sub == 0x00) s = "FireWire";
        else s = "Serial bus";
        break;
    case 0xFF: s = "Unassigned/vendor"; break;
    default:   s = "Прочее"; break;
    }
    while (i < n - 1 && s[i]) { out[i] = s[i]; i++; }
    out[i] = 0;
}

/* [#494] ПОЛНЫЙ PCI-ДАМП: все устройства, имена, BAR0, прерывание.
 * Linux-эквивалент: lspci. Печатаем до 48 устройств + итог. */
void ud_pci_dump_all(void) {
    serial_puts("PCI-DUMP: ═══ ВСЕ УСТРОЙСТВА PCI (256 шин × 32 × 8, [#494]) ═══\n");
    serial_puts("PCI-DUMP: имена — встроенная выжимка pci.ids; Linux-эквивалент: lspci\n");
    int shown = 0, total = 0, usb_hc = 0;
    for (uint16_t bus = 0; bus < 256; bus++) {
        for (uint8_t dev = 0; dev < 32; dev++) {
            for (uint8_t fn = 0; fn < 8; fn++) {
                uint32_t id = ud_pci_r32((uint8_t)bus, dev, fn, 0x00);
                if ((id & 0xFFFFu) == 0xFFFFu) {
                    if (fn == 0) break;      /* нет девайса — нет функций */
                    continue;
                }
                total++;
                uint32_t cc  = ud_pci_r32((uint8_t)bus, dev, fn, 0x08);
                uint16_t vid = (uint16_t)(id & 0xFFFFu);
                uint16_t pid = (uint16_t)(id >> 16);
                int is_usb = ((cc >> 24) == 0x0Cu && ((cc >> 16) & 0xFFu) == 0x03u);
                if (is_usb) usb_hc++;
                if (shown >= 48) {
                    /* всё равно уважаем multifunction-правило обхода */
                } else {
                    shown++;
                    serial_puts("PCI-DUMP: ");
                    serial_puts(itoa_((uint32_t)bus)); serial_puts(":");
                    serial_puts(itoa_((uint32_t)dev)); serial_puts(".");
                    serial_puts(itoa_((uint32_t)fn));
                    serial_puts(" [");
                    ud_hex2((uint8_t)(vid >> 8)); ud_hex2((uint8_t)vid);
                    serial_puts(":");
                    ud_hex2((uint8_t)(pid >> 8)); ud_hex2((uint8_t)pid);
                    serial_puts("] класс ");
                    ud_hex2((uint8_t)(cc >> 24));
                    serial_puts("/"); ud_hex2((uint8_t)(cc >> 16));
                    serial_puts("/"); ud_hex2((uint8_t)(cc >> 8));
                    serial_puts(" (");
                    { char cn[40]; ud_pci_class_name(cc, cn, sizeof(cn)); serial_puts(cn); }
                    serial_puts(")");
                    const char *vn = ud_pci_vendor(vid);
                    const char *dn = ud_pci_device(vid, pid);
                    if (vn) {
                        serial_puts(" «"); serial_puts(vn);
                        if (dn) { serial_puts(" — "); serial_puts(dn); }
                        serial_puts("»");
                    } else if (dn) {
                        serial_puts(" «"); serial_puts(dn); serial_puts("»");
                    } else {
                        serial_puts(" (не в базе — норма)");
                    }
                    uint32_t bar0 = ud_pci_r32((uint8_t)bus, dev, fn, 0x10);
                    if (bar0 & 1u) {
                        serial_puts(" BAR0=I/O 0x");
                        serial_puthex(bar0 & ~0x3u);
                    } else if (bar0 & ~0xFu) {
                        serial_puts(" BAR0=MMIO 0x");
                        serial_puthex(bar0 & ~0xFu);
                    }
                    uint8_t ipin = ud_pci_r8((uint8_t)bus, dev, fn, 0x3D);
                    uint8_t iline = ud_pci_r8((uint8_t)bus, dev, fn, 0x3C);
                    if (ipin) {
                        serial_puts(" IRQ pin "); serial_puts(itoa_((uint32_t)ipin));
                        serial_puts(" → line "); serial_puts(itoa_((uint32_t)iline));
                    }
                    serial_puts("\n");
                }
                if (fn == 0 &&
                    !(ud_pci_r8((uint8_t)bus, dev, fn, 0x0E) & 0x80u))
                    break;   /* не multifunction */
            }
        }
    }
    serial_puts("PCI-DUMP: ИТОГО устройств: ");
    serial_puts(itoa_((uint32_t)total));
    if (shown < total) {
        serial_puts(" (показано 48, ещё ");
        serial_puts(itoa_((uint32_t)(total - shown)));
        serial_puts(" не печатаю)");
    }
    serial_puts("; USB-контроллеров: ");
    serial_puts(itoa_((uint32_t)usb_hc));
    serial_puts("\nPCI-DUMP: «список» (pci.ids) нужен и Linux только для ИМЁН (lspci);");
    serial_puts(" драйверы ищут по КЛАССУ 0C03xx — ровно как этот USB-стек\n");
}

/* ── Метки времени (RTC CMOS, секунды от старта ядра) ────────────────── */

static uint32_t ud_rtc_hms(void) {
    outb(0x70, 0x04); uint8_t h = inb(0x71);
    outb(0x70, 0x02); uint8_t m = inb(0x71);
    outb(0x70, 0x00); uint8_t s = inb(0x71);
    uint32_t hh = (uint32_t)((h >> 4) * 10 + (h & 0xF));
    uint32_t mm = (uint32_t)((m >> 4) * 10 + (m & 0xF));
    uint32_t ss = (uint32_t)((s >> 4) * 10 + (s & 0xF));
    return hh * 3600u + mm * 60u + ss;
}

static uint32_t ud_boot;

void ud_boot_mark(void) { ud_boot = ud_rtc_hms(); }

uint32_t ud_seconds(void) {
    uint32_t now = ud_rtc_hms();
    return (now + 86400u - (ud_boot % 86400u)) % 86400u;
}

void ud_ts(const char *label) {
    serial_puts("TS=");
    serial_puts(itoa_(ud_seconds()));
    serial_puts("с ");
    if (label) serial_puts(label);
    serial_puts("\n");
}

/* ── Дескрипторы ─────────────────────────────────────────────────────── */

void ud_dev_desc(const char *where, const uint8_t *dd, int len) {
    if (!dd || len < 18) {
        serial_puts("USB-DSC: [");
        if (where) serial_puts(where);
        serial_puts("] дескриптор короче 18 байт — не печатаю\n");
        return;
    }
    serial_puts("USB-DSC: [");
    if (where) serial_puts(where);
    serial_puts("] ДЕВАЙС-ДЕСКРИПТОР (18 байт):");
    ud_hex8(dd, 18);
    serial_puts("\nUSB-DSC:   bcdUSB=0x");
    ud_hex2(dd[3]); ud_hex2(dd[2]);
    serial_puts(" класс=0x"); ud_hex2(dd[4]);
    serial_puts(dd[4] == 0 ? " (по интерфейсам)" : " (по девайсу)");
    serial_puts(" подкл=0x"); ud_hex2(dd[5]);
    serial_puts(" прото=0x"); ud_hex2(dd[6]);
    serial_puts(" MPS0=");
    serial_puts(itoa_((uint32_t)(dd[7] == 9 ? 512 : dd[7])));
    if (dd[7] == 9) serial_puts(" (код 9 = SuperSpeed)");
    serial_puts(" VID=0x"); ud_hex2(dd[9]); ud_hex2(dd[8]);
    serial_puts(" PID=0x"); ud_hex2(dd[11]); ud_hex2(dd[10]);
    serial_puts(" bcdDev=0x");
    ud_hex2(dd[13]); ud_hex2(dd[12]);
    serial_puts("\nUSB-DSC:   iManuf=");
    serial_puts(itoa_((uint32_t)dd[14]));
    serial_puts(" iProd=");
    serial_puts(itoa_((uint32_t)dd[15]));
    serial_puts(" iSer=");
    serial_puts(itoa_((uint32_t)dd[16]));
    serial_puts(" конфигураций=");
    serial_puts(itoa_((uint32_t)dd[17]));
    {
        uint16_t vid = (uint16_t)(dd[8] | ((uint16_t)dd[9] << 8));
        const char *vn = ud_usb_vendor(vid);
        if (vn) { serial_puts(" вендор: «"); serial_puts(vn); serial_puts("»"); }
        else    { serial_puts(" вендор не в базе (норма)"); }
    }
    serial_puts("\n");
}

void ud_cfg_desc(const char *where, const uint8_t *cfg, int total) {
    if (!cfg || total < 9 || cfg[0] < 9 || cfg[1] != 0x02) {
        serial_puts("USB-DSC: [");
        if (where) serial_puts(where);
        serial_puts("] конфиг-дескриптор битый/короткий — не разбираю\n");
        return;
    }
    serial_puts("USB-DSC: [");
    if (where) serial_puts(where);
    serial_puts("] КОНФИГ: всего ");
    serial_puts(itoa_((uint32_t)(cfg[2] | ((uint16_t)cfg[3] << 8))));
    serial_puts(" байт, интерфейсов ");
    serial_puts(itoa_((uint32_t)cfg[4]));
    serial_puts(", bConfigValue=");
    serial_puts(itoa_((uint32_t)cfg[5]));
    serial_puts(", MaxPower=");
    serial_puts(itoa_((uint32_t)cfg[8] * 2));
    serial_puts(" мА\n");
    /* обход упаковки: interface(9) → HID(9) → endpoint(7), по bLength/bType */
    int i = (int)cfg[0] < total ? (int)cfg[0] : 9;
    int iface = -1, eps = 0;
    while (i + 2 <= total) {
        int l = cfg[i], t = cfg[i + 1];
        if (l < 2 || i + l > total) break;
        if (t == 0x04 && l >= 9) {            /* interface descriptor */
            iface = cfg[i + 2];
            eps = 0;
            serial_puts("USB-DSC:   iface #");
            serial_puts(itoa_((uint32_t)iface));
            serial_puts(": класс=0x"); ud_hex2(cfg[i + 5]);
            serial_puts(cfg[i + 5] == 0x03 ? " (HID)" :
                        cfg[i + 5] == 0x08 ? " (Mass Storage)" :
                        cfg[i + 5] == 0x09 ? " (HUB)" :
                        cfg[i + 5] == 0x00 ? " (по endpoint'ам)" : "");
            serial_puts(" подкл=0x"); ud_hex2(cfg[i + 6]);
            serial_puts(" прото=");
            serial_puts(itoa_((uint32_t)cfg[i + 7]));
            serial_puts(cfg[i + 7] == 1 ? " (клавиатура)" :
                        cfg[i + 7] == 2 ? " (мышь)" : "");
            serial_puts(" альт=");
            serial_puts(itoa_((uint32_t)cfg[i + 3]));
            serial_puts("\n");
        } else if (t == 0x21 && l >= 9 && iface >= 0) {  /* HID descriptor */
            serial_puts("USB-DSC:     HID: report-дескриптор ");
            serial_puts(itoa_((uint32_t)(cfg[i + 7] | ((uint16_t)cfg[i + 8] << 8))));
            serial_puts(" байт, версия 0x");
            ud_hex2(cfg[i + 3]); ud_hex2(cfg[i + 2]);
            serial_puts("\n");
        } else if (t == 0x05 && l >= 7 && iface >= 0) {  /* endpoint */
            eps++;
            uint8_t addr = cfg[i + 2];
            serial_puts("USB-DSC:     EP 0x");
            ud_hex2(addr);
            serial_puts((addr & 0x80) ? " IN " : " OUT ");
            serial_puts(itoa_((uint32_t)cfg[i + 4] | ((uint16_t)cfg[i + 5] << 8)));
            serial_puts(" байт, каждые ");
            serial_puts(itoa_((uint32_t)cfg[i + 6]));
            serial_puts(" мс (iface #");
            serial_puts(itoa_((uint32_t)iface));
            serial_puts(", EP #");
            serial_puts(itoa_((uint32_t)eps));
            serial_puts(")\n");
        }
        i += l;
    }
}

/* Печатает «...» (UTF-16LE) — БЕЗ префикса: контекст «USB-STR: [где]»
 * печатает вызывающий. */
void ud_str_desc(const uint8_t *sd, int len) {
    if (!sd || len < 2 || sd[1] != 0x03) {
        serial_puts("(нет строки: дескриптор не строковый)\n");
        return;
    }
    int n = sd[0] < len ? sd[0] : len;
    serial_puts("«");
    for (int i = 2; i + 1 < n; i += 2) {
        char c = (char)sd[i];
        ud_pc((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    serial_puts("» (UTF-16LE, ");
    serial_puts(itoa_((uint32_t)((n - 2) / 2)));
    serial_puts(" симв.)\n");
}

/* ── Побитовые расшифровки регистров ─────────────────────────────────── */

static void ud_bit(uint32_t v, uint32_t m) {
    serial_puts((v & m) ? "1" : "0");
}

void ud_ohci_port_bits(uint32_t v) {
    serial_puts(" биты:[CCS=");
    ud_bit(v, 1u);
    serial_puts(" PES=");
    ud_bit(v, 2u);
    serial_puts(" PRS=");
    ud_bit(v, 0x10u);
    serial_puts(" PPS=");
    ud_bit(v, 0x100u);
    serial_puts(" LSDA=");
    ud_bit(v, 0x200u);
    serial_puts(" CSC=");
    ud_bit(v, 0x10000u);
    serial_puts(" PRSC=");
    ud_bit(v, 0x100000u);
    if (v & ~0x110313u) {
        serial_puts(" прочие=0x");
        serial_puthex(v & ~0x110313u);
    }
    serial_puts("]");
}

void ud_ehci_port_bits(uint32_t v) {
    serial_puts(" биты:[CCS=");
    ud_bit(v, 1u);
    serial_puts(" CSC=");
    ud_bit(v, 2u);
    serial_puts(" PED=");
    ud_bit(v, 4u);
    serial_puts(" PEC=");
    ud_bit(v, 8u);
    serial_puts(" OCC=");
    ud_bit(v, 0x20u);
    serial_puts(" PR=");
    ud_bit(v, 0x100u);
    serial_puts(" линия=");
    {
        uint32_t ls = (v >> 10) & 3u;
        if (ls == 1) serial_puts("K(low-speed)");
        else if (ls == 2) serial_puts("J(high/full)");
        else if (ls == 0) serial_puts("SE0");
        else { serial_puts("0x"); serial_puthex(ls); }
    }
    serial_puts(" PP=");
    ud_bit(v, 0x1000u);
    serial_puts(" PO=");
    ud_bit(v, 0x2000u);
    if (v & ~0x33FFu) {
        serial_puts(" прочие=0x");
        serial_puthex(v & ~0x33FFu);
    }
    serial_puts("]");
}

void ud_xhci_port_bits(uint32_t v) {
    serial_puts(" биты:[CCS=");
    ud_bit(v, 1u);
    serial_puts(" PED=");
    ud_bit(v, 2u);
    serial_puts(" OCA=");
    ud_bit(v, 4u);
    serial_puts(" PR=");
    ud_bit(v, 8u);
    serial_puts(" PP=");
    ud_bit(v, 0x10u);
    serial_puts(" PLS=");
    serial_puts(itoa_((v >> 5) & 0xFu));
    serial_puts(" скорость=");
    serial_puts(itoa_((v >> 9) & 0xFu));
    if (v & 0xFFFF0000u) {
        serial_puts(" change=0x");
        serial_puthex(v >> 16);
    }
    serial_puts("]");
}

void ud_ehci_status_bits(uint32_t cmd, uint32_t sts) {
    serial_puts(" USBCMD:[RS=");
    ud_bit(cmd, 1u);
    serial_puts(" PSE=");
    ud_bit(cmd, 0x10u);
    serial_puts(" ASE=");
    ud_bit(cmd, 0x20u);
    serial_puts(" IAAD=");
    ud_bit(cmd, 0x40u);
    serial_puts("] USBSTS:[IAA=");
    ud_bit(sts, 0x20u);      /* бит 5: Interrupt on Async Advance      */
    serial_puts(" HALT=");
    ud_bit(sts, 0x1000u);    /* бит 12: HCHalted (был 0x8000 — врал!)  */
    serial_puts(" RECL=");
    ud_bit(sts, 0x2000u);    /* бит 13: Reclamation                    */
    serial_puts(" PSS=");
    ud_bit(sts, 0x4000u);    /* бит 14: Periodic Schedule Status       */
    serial_puts(" ASS=");
    ud_bit(sts, 0x8000u);    /* бит 15: Async Schedule Status          */
    serial_puts(" PCD=");
    ud_bit(sts, 4u);
    serial_puts(" FLR=");
    ud_bit(sts, 8u);
    serial_puts("]");
}
