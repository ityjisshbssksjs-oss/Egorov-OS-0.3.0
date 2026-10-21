/*
 * ACPI power management for Egorov OS — v1.8
 *
 * WHY (real-machine report: "выключить и перезагрузить не смог"):
 *   The previous shutdown wrote only to the hard-coded QEMU/Bochs/
 *   VirtualBox ACPI ports (0x604/0xB004/0x4004) — on real hardware those
 *   writes go nowhere.  The last-resort `outb(0x64, 0xFE)` pulses the
 *   i8042 RESET line — that REBOOTS, and only when a legacy keyboard
 *   controller exists at all (many UEFI boards have none).  Reboot was
 *   not reachable from the UI in the first place.
 *
 * v1.8 (QEMU 2.0.0 report: "\_S5_ not found in DSDT" although Linux
 *       powers off fine, plus a scary "power-off FAILED" banner):
 *   - ROOT CAUSE: QEMU >= 1.7 declares \_S5_ inside an SSDT (ssdt-misc),
 *     NOT in the DSDT.  \_S5_ is now searched in the DSDT AND in every
 *     SSDT listed in the RSDT/XSDT — the same way ACPICA resolves the
 *     whole namespace.  Real firmware usually declares it in the DSDT.
 *   - SCI_EN was read from PM1a_EVT (bit 0 = TMROF, a random timer
 *     flag!); it actually lives in bit 0 of PM1a_CNT.  Fixed — on real
 *     HW the SMI ACPI-enable step could be skipped silently.
 *   - ACPI 2.0+ X_PM1a_CNT_BLK/X_PM1b_CNT_BLK offsets were wrong
 *     (164/172 → 172/184); all X_ reads are now guarded by FADT length.
 *   - The VM honours the S5 request asynchronously, so "power-off
 *     FAILED" used to be printed even on success — now we wait ~3 s
 *     after each power-off write before declaring failure.
 *   - AML package parse: the NumElements byte was not skipped, so the
 *     \_S5_ package (QEMU: 12 06 04 00 00 00 00) never decoded — found
 *     against a physical RAM dump of QEMU 2.0.0 (SSDT @+0x96).
 *
 * WHAT THIS MODULE DOES
 *   1. Locates ACPI RSDP (EBDA 0x40E→seg<<4, then 0xE0000..0xFFFFF),
 *      validates the checksum, picks XSDT (ACPI 2.0+) or RSDT and finds
 *      the FADT ("FACP").
 *   2. Shutdown = ACPI S5:
 *        - if SCI_EN (PM1a_STS bit 0) is clear and SMI_CMD/ACPI_ENABLE
 *          are provided, ACPI mode is enabled first;
 *        - the \_S5_ package is extracted from the DSDT (scan for
 *          "_S5_" + PackageOp, PkgLength skip, two TermArg elements
 *          → SLP_TYPa/b);
 *        - writes (SLP_TYP << 10) | SLP_EN into PM1a_CNT (and PM1b),
 *          honouring the X_PM1a_CNT_BLK GAS (SystemIO or SystemMemory).
 *   3. Reboot = FADT RESET_REG (flag RESET_REG_SUPPORTED) → port 0xCF9
 *      (0x02 / 0x04 / 0x0E sequence) → i8042 0xFE pulse → triple fault
 *      (LIDT 0 + INT3).  Four independent mechanisms.
 *
 * Notes:
 *   - The kernel runs flat (no paging), so MMIO GAS addresses below
 *     4 GiB are accessed directly by pointer.
 *   - The VM convenience ports remain as a shutdown fallback for
 *     QEMU/Bochs/VirtualBox configurations without ACPI tables.
 *   - power_shutdown() NEVER reboots as a fallback: on failure it halts
 *     and logs that the power button must be held.  A surprise reset
 *     from a "shutdown" click is what confused the real-machine run.
 */

#include "types.h"
#include "io.h"
#include "power.h"

/* Serial logging lives in kernel.c. */
extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);

/* ── Tiny helpers ──────────────────────────────────────────────────────── */

/* Port-0x80 delay: ~1 µs per iteration with the driver's calibration. */
static void pwr_delay(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 4; i++) inb(0x80);
}

static bool sig4(const uint8_t *p, const char *s) {
    for (int i = 0; i < 4; i++)
        if (p[i] != (uint8_t)s[i]) return false;
    return true;
}

static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint64_t rd64(const uint8_t *p) {
    return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32);
}

/* ── ACPI structures (only the fields we need) ─────────────────────────── */

typedef struct __attribute__((packed)) {
    uint8_t  space_id;     /* 0 = SystemMemory, 1 = SystemIO            */
    uint8_t  bit_width;
    uint8_t  bit_offset;
    uint8_t  access_size;
    uint64_t address;
} acpi_gas_t;

#define RSDP_SIG "RSD PTR "
#define FADT_FLAGS_RESET_REG_SUPPORTED (1u << 10)

#define FACP_SMI_CMD        48
#define FACP_ACPI_ENABLE    52
#define FACP_ACPI_DISABLE   53
#define FACP_PM1A_EVT_BLK   56
#define FACP_PM1A_CNT_BLK   64
#define FACP_PM1B_CNT_BLK   68
#define FACP_DSDT           40
#define FACP_FLAGS         112
#define FACP_RESET_REG     116
#define FACP_RESET_VALUE   128
#define FACP_X_DSDT        140
#define FACP_X_PM1A_CNT    172   /* v1.8: were 164 — that hit X_PM1b_EVT */
#define FACP_X_PM1B_CNT    184   /* v1.8: were 172 */
#define FACP_X_PM1A_END    (FACP_X_PM1A_CNT + 12) /* min FADT length */
#define FACP_X_PM1B_END    (FACP_X_PM1B_CNT + 12)

/* ── RSDP ──────────────────────────────────────────────────────────────── */

static const uint8_t *rsdp_checksum_ok(const uint8_t *p) {
    uint8_t sum = 0;
    for (int i = 0; i < 20; i++) sum = (uint8_t)(sum + p[i]);
    if (sum) return 0;
    if (p[15] >= 2) {                     /* extended checksum (v2+) */
        sum = 0;
        for (int i = 0; i < 36; i++) sum = (uint8_t)(sum + p[i]);
        if (sum) return 0;
    }
    return p;
}

static const uint8_t *rsdp_scan(uint32_t from, uint32_t to) {
    for (uint32_t a = from; a + 36 <= to; a += 16) {
        const uint8_t *p = (const uint8_t *)a;
        int i;
        for (i = 0; i < 8; i++)
            if (p[i] != (uint8_t)RSDP_SIG[i]) break;
        if (i == 8 && rsdp_checksum_ok(p)) return p;
    }
    return 0;
}

static const uint8_t *acpi_find_rsdp(void) {
    /* EBDA segment lives at 0x40E.  Read via asm: GCC 14 flags a plain
     * dereference of the integer constant as an array-bounds violation. */
    uint16_t seg;
    __asm__ volatile("movw 0x40E, %0" : "=r"(seg));
    uint32_t base = ((uint32_t)seg) << 4;
    if (base >= 0x80000 && base < 0x100000) {
        const uint8_t *p = rsdp_scan(base, base + 1024);
        if (p) return p;
    }
    return rsdp_scan(0xE0000, 0x100000);
}

/* ── Table lookup via RSDT (4-byte entries) / XSDT (8-byte entries) ────── */

static const uint8_t *sdtable_find(uint32_t base, int wide, const char *sig) {
    if (!base || base > 0xFFFFFFF0u) return 0;
    const uint8_t *t = (const uint8_t *)base;
    uint32_t len = rd32(t + 4);
    if (len < 40 || len > 0x100000u) return 0;
    uint32_t n = (len - 36) / (wide ? 8 : 4);
    const uint8_t *ent = t + 36;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t p = wide ? (uint32_t)rd64(ent + i * 8) : rd32(ent + i * 4);
        if (!p || p > 0xFFFFFFF0u) continue;
        const uint8_t *h = (const uint8_t *)p;
        if (sig4(h, sig) && rd32(h + 4) >= 4) return h;
    }
    return 0;
}

/* RSDP of the last acpi_find_fadt() — reused to walk the root table
 * for SSDTs (v1.8). */
static const uint8_t *g_rsdp;

static const uint8_t *acpi_find_fadt(void) {
    const uint8_t *rsdp = acpi_find_rsdp();
    g_rsdp = rsdp;
    if (!rsdp) return 0;
    serial_puts("POWER: RSDP found, rev=");
    serial_puthex(rsdp[15]);
    serial_puts("\n");
    /* Глубокий аудит #225: ACPI 1.0 (RSDP rev < 2) XSDT-указателя не
     * имеет — только RSDT с 4-байтными записями.  Все три случая —
     * rev < 2, XSDT == 0 и «XSDT есть, а FADT в нём не нашёлся» —
     * скатываются на RSDT-путь ниже (строка sdtable_find(rd32(rsdp+16),
     * 0, "FACP")); acpi_root_table() для SSDT-обхода устроен так же. */
    if (rsdp[15] >= 2) {
        uint64_t xsdt = rd64(rsdp + 24);
        if (xsdt && xsdt < 0x100000000ull) {
            const uint8_t *f = sdtable_find((uint32_t)xsdt, 1, "FACP");
            if (f) return f;
        }
    }
    return sdtable_find(rd32(rsdp + 16), 0, "FACP");
}

/* ── GAS write helpers (flat memory model: MMIO below 4 GiB is direct) ─── */

static bool gas_io_ok(const acpi_gas_t *g) {
    return g->address != 0 && g->space_id == 1 && g->address <= 0xFFFFu;
}
static bool gas_mem_ok(const acpi_gas_t *g) {
    return g->address != 0 && g->space_id == 0 && g->address < 0x100000000ull;
}

/* X_ GAS fields exist only in FADT revisions long enough to hold them;
 * on a rev-1 FADT (116 bytes, e.g. QEMU) they point past the table. */
static const acpi_gas_t *x_gas(const uint8_t *fadt, uint32_t len,
                               uint32_t off, uint32_t need) {
    return (len >= need) ? (const acpi_gas_t *)(fadt + off) : 0;
}

/* Resolve a PM1a/b_CNT register: X_ GAS preferred, legacy u32 fallback.
 * Returns the SystemIO port to outw() to, or 0 when none.  (MMIO
 * PM1a_CNT is not handled — it does not occur on real x86 boards.) */
static uint16_t pm1_cnt_io(const acpi_gas_t *xg, uint32_t legacy) {
    if (xg && gas_io_ok(xg)) return (uint16_t)xg->address;
    if (legacy && legacy <= 0xFFFFu) return (uint16_t)legacy;
    return 0;
}

/* ── DSDT: extract the \_S5_ sleep package ─────────────────────────────── */

/* Decode one TermArg byte value: ZeroOp/OneOp/BytePrefix/WordPrefix/
 * DWordPrefix.  Returns bytes consumed or 0 on unsupported encoding. */
static int acpi_termarg(const uint8_t *d, uint32_t n, uint32_t i, uint8_t *out) {
    if (i >= n) return 0;
    uint8_t op = d[i];
    if (op == 0x00) { *out = 0; return 1; }                       /* ZeroOp  */
    if (op == 0x01) { *out = 1; return 1; }                       /* OneOp   */
    if (op == 0x0A) { if (i + 1 >= n) return 0; *out = d[i + 1]; return 2; }
    if (op == 0x0B) { if (i + 2 >= n) return 0; *out = d[i + 1]; return 3; }
    if (op == 0x0C) { if (i + 4 >= n) return 0; *out = d[i + 1]; return 5; }
    return 0;
}

static bool acpi_find_s5(const uint8_t *d, uint32_t n,
                         uint8_t *slp_a, uint8_t *slp_b) {
    if (!d || n < 8) return false;
    for (uint32_t i = 0; i + 5 < n; i++) {
        if (d[i] != 0x5F || d[i + 1] != 0x53 ||
            d[i + 2] != 0x35 || d[i + 3] != 0x5F) continue;   /* "_S5_" */
        uint32_t j = i + 4;
        if (d[j] != 0x12) continue;                            /* PackageOp */
        j++;
        uint8_t lead = (uint8_t)((d[j] >> 6) & 3u);            /* PkgLength */
        uint32_t extra = lead;                                 /* extra len bytes */
        if (j + extra + 1 >= n) continue;
        j += 1 + extra;                 /* skip PkgLength -> NumElements      */
        j += 1;                         /* skip NumElements! (QEMU ssdt-misc:
                                         * 12 06 04 00 00 00 00 — без этого
                                         * парсер съедает 0x04 как элемент)  */
        int r = acpi_termarg(d, n, j, slp_a);
        if (!r) continue;
        j += (uint32_t)r;
        if (!acpi_termarg(d, n, j, slp_b)) *slp_b = *slp_a;
        return true;
    }
    return false;
}

static const uint8_t *acpi_dsdt(const uint8_t *fadt) {
    uint8_t rev = fadt[8];
    if (rev >= 2) {
        uint64_t x = rd64(fadt + FACP_X_DSDT);
        if (x && x < 0x100000000ull) return (const uint8_t *)(uint32_t)x;
    }
    uint32_t d = rd32(fadt + FACP_DSDT);
    return (d && d < 0xFFFFFFF0u) ? (const uint8_t *)d : 0;
}

/* Root table (RSDT or XSDT) of the last acpi_find_fadt() — v1.8. */
static const uint8_t *acpi_root_table(void) {
    if (!g_rsdp) return 0;
    if (g_rsdp[15] >= 2) {
        uint64_t x = rd64(g_rsdp + 24);
        if (x && x < 0x100000000ull) {
            const uint8_t *t = (const uint8_t *)(uint32_t)x;
            if (sig4(t, "XSDT") && rd32(t + 4) >= 40 && rd32(t + 4) < 0x100000u)
                return t;
        }
    }
    uint32_t r = rd32(g_rsdp + 16);
    if (r && r < 0xFFFFFFF0u) {
        const uint8_t *t = (const uint8_t *)r;
        if (sig4(t, "RSDT") && rd32(t + 4) >= 40 && rd32(t + 4) < 0x100000u)
            return t;
    }
    return 0;
}

/* Search \_S5_ in the DSDT first, then in every SSDT reachable from the
 * root table.  QEMU >= 1.7 declares \_S5_ only in an SSDT (ssdt-misc);
 * most real firmware declares it in the DSDT.  ACPICA resolves \_S5_
 * from the whole namespace, so must we.  *scanned counts the AML
 * tables looked at (serial diagnostics). */
static bool acpi_find_s5_ns(const uint8_t *fadt, uint8_t *a, uint8_t *b,
                            int *scanned) {
    *scanned = 0;
    const uint8_t *dsdt = acpi_dsdt(fadt);
    if (dsdt && sig4(dsdt, "DSDT") && rd32(dsdt + 4) < 0x200000u) {
        (*scanned)++;
        if (acpi_find_s5(dsdt, rd32(dsdt + 4), a, b)) return true;
    }
    const uint8_t *root = acpi_root_table();
    if (root) {
        int wide = sig4(root, "XSDT");
        uint32_t n = (rd32(root + 4) - 36) / (wide ? 8u : 4u);
        const uint8_t *ent = root + 36;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t p = wide ? (uint32_t)rd64(ent + i * 8) : rd32(ent + i * 4);
            if (!p || p > 0xFFFFFFF0u) continue;
            const uint8_t *h = (const uint8_t *)p;
            if (!sig4(h, "SSDT")) continue;
            uint32_t l = rd32(h + 4);
            if (l < 40 || l > 0x200000u) continue;
            (*scanned)++;
            if (acpi_find_s5(h, l, a, b)) return true;
        }
    }
    return false;
}

/* Enable ACPI mode if the firmware left it disabled.
 * v1.8: SCI_EN is bit 0 of PM1a_CNT — the old code read PM1a_EVT, whose
 * bit 0 is the timer status TMROF (a random flag!). */
static void acpi_enable_if_needed(const uint8_t *fadt, uint16_t pm1a) {
    if (!pm1a) return;
    if (inw(pm1a) & 1u) return;                           /* SCI_EN set */
    uint32_t smi = rd32(fadt + FACP_SMI_CMD);
    uint8_t  en  = fadt[FACP_ACPI_ENABLE];
    if (!smi || smi > 0xFFFFu || !en) {
        serial_puts("POWER: SCI_EN clear, no SMI_CMD — trying anyway\n");
        return;
    }
    serial_puts("POWER: enabling ACPI via SMI\n");
    /* Глубокий аудит #226: read-back самого SMI_CMD-порта НЕнадёжен —
     * на реальных чипсетах этот порт часто write-only (чтение даёт 0xFF
     * или мусор), так что прямая сверка дала бы ложный warn почти всегда.
     * Вместо неё — дешёвая КОСВЕННАЯ проверка результата: после записи и
     * задержки перечитываем SCI_EN (бит 0 PM1a_CNT); если режим так и не
     * поднялся — честное предупреждение в журнал (SMI мог не сработать). */
    outb((uint16_t)smi, en);
    pwr_delay(100000);                                    /* ~100 ms    */
    if (!(inw(pm1a) & 1u))
        serial_puts("POWER: WARN: SCI_EN не поднялся после SMI_CMD — запись\n"
                    "POWER: в SMI_CMD не сработала (порт write-only или не наш)\n");
}

/* ── Public: power off (ACPI S5) ───────────────────────────────────────── */

/* The VM (and real firmware) processes the S5 write asynchronously while
 * our vCPU keeps executing — a successful power-off looks like a sudden
 * power cut.  Wait this long BEFORE declaring failure. */
#define POWER_SETTLE_US 3000000u                                /* ~3 s */

void power_shutdown(void) {
    cli();
    serial_puts("POWER: shutdown requested (ACPI S5)\n");

    const uint8_t *fadt = acpi_find_fadt();
    if (fadt) {
        uint32_t fadt_len = rd32(fadt + 4);
        serial_puts("POWER: FADT found, len=");
        serial_puthex(fadt_len);
        serial_puts("\n");

        uint16_t pm1a = pm1_cnt_io(x_gas(fadt, fadt_len, FACP_X_PM1A_CNT,
                                         FACP_X_PM1A_END),
                                   (fadt_len >= 68 ?
                                    rd32(fadt + FACP_PM1A_CNT_BLK) : 0));
        uint16_t pm1b = pm1_cnt_io(x_gas(fadt, fadt_len, FACP_X_PM1B_CNT,
                                         FACP_X_PM1B_END),
                                   (fadt_len >= 72 ?
                                    rd32(fadt + FACP_PM1B_CNT_BLK) : 0));
        if (!pm1a && !pm1b) {
            serial_puts("POWER: PM1a_CNT not usable (MMIO GAS?)\n");
        }

        acpi_enable_if_needed(fadt, pm1a);

        uint8_t a = 0, b = 0;
        int scanned = 0;
        if (acpi_find_s5_ns(fadt, &a, &b, &scanned)) {
            serial_puts("POWER: \\_S5_ slp_typa=");
            serial_puthex(a);
            serial_puts(" PM1a_CNT=");
            serial_puthex(pm1a);
            serial_puts(" (scanned ");
            serial_puthex((uint32_t)scanned);
            serial_puts(" AML tables)\n");
            if (pm1a || pm1b) {
                if (pm1a) outw(pm1a, (uint16_t)(((uint16_t)a << 10) | (1u << 13)));
                if (pm1b) outw(pm1b, (uint16_t)(((uint16_t)b << 10) | (1u << 13)));
                pwr_delay(POWER_SETTLE_US);
                serial_puts("POWER: still running after S5 write — falling back\n");
            }
        } else {
            serial_puts("POWER: \\_S5_ not found (scanned ");
            serial_puthex((uint32_t)scanned);
            serial_puts(" AML tables)\n");
        }
    } else {
        serial_puts("POWER: no FACP (ACPI not available)\n");
    }

    /* Fallback: VM convenience ports (QEMU / Bochs / VirtualBox). */
    serial_puts("POWER: trying VM ACPI ports\n");
    outw(0x604,  0x2000);   /* QEMU/KVM: SLP_TYP=0 + SLP_EN  */
    outw(0xB004, 0x2000);   /* Bochs                         */
    outw(0x4004, 0x3400);   /* VirtualBox                    */
    pwr_delay(POWER_SETTLE_US);

    /* Nothing worked: HALT (never a surprise reset from a shutdown click).
     * The user must hold the power button — serial log explains it. */
    serial_puts("POWER: power-off FAILED — system halted, "
                "press and hold the power button\n");
    for (;;) __asm__ volatile("hlt");
}

/* ── Public: reset the machine ─────────────────────────────────────────── */

void power_reboot(void) {
    cli();
    serial_puts("POWER: reboot requested\n");

    /* 1. FADT RESET_REG — the canonical way on ACPI 2.0+ hardware. */
    const uint8_t *fadt = acpi_find_fadt();
    if (fadt && rd32(fadt + 4) >= 129 &&          /* RESET_REG fits */
        (rd32(fadt + FACP_FLAGS) & FADT_FLAGS_RESET_REG_SUPPORTED)) {
        const acpi_gas_t *g = (const acpi_gas_t *)(fadt + FACP_RESET_REG);
        uint8_t v = fadt[FACP_RESET_VALUE];
        serial_puts("POWER: FADT RESET_REG space=");
        serial_puthex(g->space_id);
        serial_puts(" addr=");
        serial_puthex((uint32_t)g->address);
        serial_puts(" val=");
        serial_puthex(v);
        serial_puts("\n");
        if (g->space_id == 1 && g->address <= 0xFFFFu)
            outb((uint16_t)g->address, v);
        else if (g->space_id == 0 && g->address < 0x100000000ull)
            *(volatile uint8_t *)(uint32_t)g->address = v;
        pwr_delay(200000);                                /* ~200 ms    */
    }

    /* 2. Port 0xCF9 — reset control on most PC chipsets. */
    serial_puts("POWER: trying port 0xCF9\n");
    outb(0xCF9, 0x02);   /* RST_CPU */
    pwr_delay(50000);
    outb(0xCF9, 0x04);   /* RST_SYS */
    pwr_delay(50000);
    outb(0xCF9, 0x0E);   /* full reset */
    pwr_delay(100000);

    /* 3. i8042 keyboard-controller pulse (legacy boards). */
    serial_puts("POWER: trying i8042 pulse\n");
    for (int i = 0; i < 100; i++) {
        if (!(inb(0x64) & 2)) break;
        pwr_delay(1000);
    }
    outb(0x64, 0xFE);
    pwr_delay(100000);

    /* 4. Triple fault: empty IDT + INT3 → #GP → #DF → shutdown → reset. */
    serial_puts("POWER: trying triple fault\n");
    {
        static struct { uint16_t limit; uint32_t base; } __attribute__((packed))
            null_idtr = { 0, 0 };
        __asm__ volatile("lidt %0" :: "m"(null_idtr) : "memory");
        __asm__ volatile("int3");
    }

    serial_puts("POWER: reboot FAILED — halted\n");
    for (;;) __asm__ volatile("hlt");
}
