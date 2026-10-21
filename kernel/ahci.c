/* AHCI SATA driver for Egorov OS — v1.0
 *
 * Detects the first AHCI controller via PCI config-space scan (bus 0),
 * initialises the first attached ATA port, and provides sector read/write
 * through DMA.  On success fills ata_primary_master and sets ata_use_ahci
 * so that the existing ATA API transparently routes here.
 *
 * Limitations: single port (first found), no NCQ, BSS-static aligned
 * buffers; LBA48 (READ/WRITE DMA EXT) — при поддержке диском.
 */
#include "ahci.h"
#include "ata.h"
#include "io.h"
#include "string.h"
#include "timer.h"    /* kdelay_ms/kdelay_us — реальные таймауты вместо итерационных циклов */

/* ── PCI config-space (I/O CF8/CFC) ── */
#define PCI_ADDR 0xCF8u
#define PCI_DATA 0xCFCu

static uint32_t pci_r32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off) {
    uint32_t a = (1u<<31)|((uint32_t)bus<<16)|((uint32_t)dev<<11)|
                 ((uint32_t)fn<<8)|(off&0xFCu);
    outl(PCI_ADDR, a); return inl(PCI_DATA);
}
static void pci_w32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off, uint32_t v) {
    uint32_t a = (1u<<31)|((uint32_t)bus<<16)|((uint32_t)dev<<11)|
                 ((uint32_t)fn<<8)|(off&0xFCu);
    outl(PCI_ADDR, a); outl(PCI_DATA, v);
}
static uint8_t pci_r8(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off) {
    return (uint8_t)(pci_r32(bus,dev,fn,off&0xFCu)>>((off&3u)*8u));
}

/* ── AHCI register layout (relative to ABAR) ── */
#define HBA_CAP      0x00u
#define HBA_GHC      0x04u
#define HBA_PI       0x0Cu

#define PORT_OFF(p)  (0x100u + (uint32_t)(p)*0x80u)
#define P_CLB(p)     (PORT_OFF(p)+0x00u)
#define P_CLBU(p)    (PORT_OFF(p)+0x04u)
#define P_FB(p)      (PORT_OFF(p)+0x08u)
#define P_FBU(p)     (PORT_OFF(p)+0x0Cu)
#define P_IS(p)      (PORT_OFF(p)+0x10u)
#define P_CMD(p)     (PORT_OFF(p)+0x18u)
#define P_TFD(p)     (PORT_OFF(p)+0x20u)
#define P_SIG(p)     (PORT_OFF(p)+0x24u)
#define P_SSTS(p)    (PORT_OFF(p)+0x28u)
#define P_SCTL(p)    (PORT_OFF(p)+0x2Cu)
#define P_SERR(p)    (PORT_OFF(p)+0x30u)
#define P_CI(p)      (PORT_OFF(p)+0x38u)

#define GHC_AE   (1u<<31)
#define GHC_HR   (1u<<0)   /* HBA reset — контроллер снимает бит сам */
#define CMD_ST   (1u<<0)
#define CMD_FRE  (1u<<4)
#define CMD_FR   (1u<<14)
#define CMD_CR   (1u<<15)
#define TFD_BSY  (1u<<7)
#define TFD_DRQ  (1u<<3)
#define TFD_ERR  (1u<<0)
#define IS_TFES  (1u<<30)  /* task file error status */

/* ── ATA8-ACS / AHCI 1.3 command structures ── */

typedef struct {
    uint16_t         flags;   /* CFL[4:0] | ATAPI | W | P | R | B | C | reserved */
    uint16_t         prdtl;
    volatile uint32_t prdbc;
    uint32_t         ctba;
    uint32_t         ctbau;
    uint32_t         rsv[4];
} __attribute__((packed)) cmd_hdr_t;

typedef struct {
    uint32_t dba;
    uint32_t dbau;
    uint32_t rsv;
    uint32_t dbc;   /* byte count - 1; bit 31 = interrupt on completion */
} __attribute__((packed)) prdt_t;

typedef struct {
    uint8_t  cfis[64];   /* Command FIS */
    uint8_t  acmd[16];   /* ATAPI command (unused) */
    uint8_t  rsv[48];
    prdt_t   prdt[1];    /* one PRDT entry is enough for a single sector */
} __attribute__((packed)) cmd_tbl_t;

/* H2D Register FIS */
typedef struct {
    uint8_t  type;       /* 0x27 */
    uint8_t  pmport_c;   /* PM port [3:0] | C [7] */
    uint8_t  cmd;
    uint8_t  feat;
    uint8_t  lba0, lba1, lba2, dev;
    uint8_t  lba3, lba4, lba5, feah;
    uint8_t  cntl, cnth, icc, ctl;
    uint8_t  rsv[4];
} __attribute__((packed)) h2d_fis_t;

/* ── BSS-allocated DMA buffers (properly aligned) ──
 * g_databuf выровнен на 4096: PRDT-запись описывает непрерывный
 * физический диапазон, и 512-байтный буфер, пересекающий границу
 * страницы при выравнивании 4, описать одной записью нельзя —
 * контроллер молча портит соседнюю страницу (F5, аудит п.1). */
static cmd_hdr_t  g_cmdlist[32]  __attribute__((aligned(1024)));
static uint8_t    g_fisbuf[256]  __attribute__((aligned(256)));
static cmd_tbl_t  g_cmdtbl       __attribute__((aligned(128)));
static uint8_t    g_databuf[512] __attribute__((aligned(4096)));

/* ── Driver state ── */
ahci_drive_t ahci_drive;
extern bool          ata_use_ahci;         /* defined in ata.c */
extern ata_drive_t   ata_primary_master;   /* defined in ata.c */

static uint32_t g_abar = 0;
static int      g_port = -1;
static bool     g_lba48 = false;  /* IDENTIFY word 83 бит 10 — диски >128 ГБ */

/* ── MMIO accessors (volatile pointer dereference, not I/O port) ── */
static uint32_t hba_r(uint32_t off) {
    return *(volatile uint32_t*)(g_abar + off);
}
static void hba_w(uint32_t off, uint32_t v) {
    *(volatile uint32_t*)(g_abar + off) = v;
}

/* ── Port start/stop ── */
static void port_stop(int p) {
    uint32_t c = hba_r(P_CMD(p));
    c &= ~CMD_ST;  hba_w(P_CMD(p), c);
    /* CR снимается до 500 мс РЕАЛЬНОГО времени (kdelay, а не «голые»
     * итерации, длительность которых зависит от частоты CPU). */
    for (uint32_t t = 0; t < 500u; t++) {
        if (!(hba_r(P_CMD(p)) & CMD_CR)) break;
        kdelay_ms(1);
    }
    c  = hba_r(P_CMD(p));
    c &= ~CMD_FRE; hba_w(P_CMD(p), c);
    /* FR снимается до 500 мс, если FIS-приёмник не занят */
    for (uint32_t t = 0; t < 500u; t++) {
        if (!(hba_r(P_CMD(p)) & CMD_FR))  break;
        kdelay_ms(1);
    }
}

static void port_start(int p) {
    /* Перед ST устройство должно снять BSY/DRQ - до 1 с реального
     * времени (аудит F5 п.22: итерационные циклы = мс на 4 ГГц). */
    for (uint32_t t = 0; t < 1000u; t++) {
        if (!(hba_r(P_TFD(p)) & (TFD_BSY|TFD_DRQ))) break;
        kdelay_ms(1);
    }
    uint32_t c = hba_r(P_CMD(p));
    c |= CMD_FRE; hba_w(P_CMD(p), c);
    c |= CMD_ST; hba_w(P_CMD(p), c);
}

/* Дождаться полного снятия CI (слот 0) с таймаутом: следующая
 * команда не должна затирать ещё не завершённую (аудит F5 п.2). */
static bool port_wait_ci(int p, uint32_t timeout_ms) {
    for (uint32_t t = 0; t < timeout_ms; t++) {
        if (!(hba_r(P_CI(p)) & 1u)) return true;
        kdelay_ms(1);
    }
    return (hba_r(P_CI(p)) & 1u) == 0u;
}

/* Программный сброс порта при ошибке задачи (аудит F5 п.3):
 * PxSCTL.SRST=1 -> ~1 мс -> SRST=0 -> дождаться DET=3 ->
 * очистить SERR/IS -> восстановить CLB/FB (сброс их обнуляет) -> запуск. */
static void port_reset(int p) {
    port_stop(p);
    hba_w(P_SCTL(p), (hba_r(P_SCTL(p)) & ~0x0Fu) | 0x01u);  /* SRST=1 */
    kdelay_ms(1);
    hba_w(P_SCTL(p), hba_r(P_SCTL(p)) & ~0x01u);            /* SRST=0 */
    /* Phy-связь восстанавливается до ~1 с; DET=3 = device present + comm */
    for (uint32_t t = 0; t < 2000u; t++) {
        if ((hba_r(P_SSTS(p)) & 0x0Fu) == 3u) break;
        kdelay_ms(1);
    }
    hba_w(P_SERR(p), 0xFFFFFFFFu);   /* очистка накопленных ошибок */
    hba_w(P_IS(p),   0xFFFFFFFFu);
    hba_w(P_CLB(p),  (uint32_t)g_cmdlist); hba_w(P_CLBU(p), 0u);
    hba_w(P_FB(p),   (uint32_t)g_fisbuf);  hba_w(P_FBU(p),  0u);
    port_start(p);
}

/* Issue command in slot 0 and poll for completion. */
static bool port_exec(int p) {
    hba_w(P_IS(p), 0xFFFFFFFFu);
    hba_w(P_CI(p), 1u);
    /* Таймаут ~10 с реального времени: 100000 x kdelay_us(100) */
    for (uint32_t t = 0; t < 100000u; t++) {
        uint32_t ci = hba_r(P_CI(p));
        uint32_t is = hba_r(P_IS(p));
        if (is & IS_TFES) {
            /* Ошибка задачи: прежде чем вернуть failure - дождаться
             * снятия CI и сделать программный сброс порта, иначе
             * следующая команда затрёт незавершённую и контроллер
             * зависнет в ошибках (аудит F5 п.2/п.3). */
            (void)port_wait_ci(p, 1000u);
            port_reset(p);
            return false;
        }
        if (!(ci & 1u)) {
            /* CI обнулён контроллером - команда завершена */
            return !(hba_r(P_TFD(p)) & TFD_ERR);
        }
        kdelay_us(100);
    }
    /* Таймаут: не оставляем «висящую» команду в слоте 0 */
    (void)port_wait_ci(p, 1000u);
    return false;
}

/* Build LBA28/LBA48 DMA read or write command in slot 0. */
static void build_cmd(uint32_t lba, bool write) {
    memset(g_cmdlist, 0, sizeof(g_cmdlist));
    memset(&g_cmdtbl,  0, sizeof(g_cmdtbl));

    cmd_hdr_t *h = &g_cmdlist[0];
    h->flags = (write ? (1u<<6) : 0u) | 5u;  /* W | CFL=5 dwords */
    h->prdtl = 1;
    h->ctba  = (uint32_t)&g_cmdtbl;
    h->ctbau = 0;

    h2d_fis_t *f = (h2d_fis_t*)g_cmdtbl.cfis;
    f->type     = 0x27;
    f->pmport_c = 0x80;                              /* C=1: command */
    if (g_lba48) {
        /* LBA48: READ DMA EXT 0x25 / WRITE DMA EXT 0x35 - иначе диски
         * >128 ГБ видят только младшие 28 бит LBA и запись уходит
         * «не в тот сектор» (аудит F5 п.7). */
        f->cmd      = write ? 0x35u : 0x25u;
        f->dev      = 0x40u;                         /* бит LBA, без LBA[27:24] */
        f->lba0     = (uint8_t)(lba);
        f->lba1     = (uint8_t)(lba >>  8);
        f->lba2     = (uint8_t)(lba >> 16);
        f->lba3     = (uint8_t)(lba >> 24);          /* LBA[31:24] */
        f->lba4     = 0;                             /* LBA[39:32] (диски <2 ТБ) */
        f->lba5     = 0;                             /* LBA[47:40] */
        f->cntl     = 1;   /* 1 сектор (старший байт счётчика = 0) */
    } else {
        f->cmd      = write ? 0xCAu : 0xC8u;         /* WRITE/READ DMA */
        f->dev      = 0xE0u | ((lba >> 24) & 0x0Fu); /* LBA mode | LBA[27:24] */
        f->lba0     = (uint8_t)(lba);
        f->lba1     = (uint8_t)(lba >>  8);
        f->lba2     = (uint8_t)(lba >> 16);
        f->cntl     = 1;   /* 1 sector */
    }

    prdt_t *pr = &g_cmdtbl.prdt[0];
    pr->dba  = (uint32_t)g_databuf;
    pr->dbau = 0;
    pr->dbc  = 511;    /* 512 bytes - 1 */
}

/* ── Public API ── */

void ahci_init(void) {
    ahci_drive.present = false;
    g_abar = 0; g_port = -1; g_lba48 = false;

    /* ATA PIO already found a drive — no AHCI needed. */
    if (ata_primary_master.present) return;

    /* Scan PCI buses 0-255, all 32 devices, all 8 functions for AHCI.
       Mass-storage class=0x01, SATA subclass=0x06.
       Prog-if values for AHCI:
         0x01 = AHCI 1.0          (most controllers, VirtualBox ICH6/ICH8)
         0x80 = vendor-specific   (some real AHCI chips)
       We also accept 0x00 (non-specific) because some BIOS/firmware versions
       report prog-if=0x00 for the AHCI controller on emulated hardware.
       Раньше сканировались шины 0-7: контроллер за PCI-PCI мостом
       на шине >7 не находился вообще (аудит F5 п.8). Полный обход
       256 шин стоит копейки - пустые шины отсеются по vendor 0xFFFF. */
    for (uint32_t bus = 0; bus < 256; bus++) {
    for (uint32_t dev = 0; dev < 32; dev++) {
    for (uint32_t fn  = 0; fn  < 8;  fn++) {
        /* Absent device: vendor ID = 0xFFFF. */
        if ((pci_r32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0) & 0xFFFFu) == 0xFFFFu) continue;
        if (pci_r8((uint8_t)bus,(uint8_t)dev,(uint8_t)fn, 0x0B) != 0x01u) continue; /* storage */
        if (pci_r8((uint8_t)bus,(uint8_t)dev,(uint8_t)fn, 0x0A) != 0x06u) continue; /* SATA    */
        uint8_t pi = pci_r8((uint8_t)bus,(uint8_t)dev,(uint8_t)fn, 0x09);
        /* Accept prog-if 0x01 (AHCI 1.0), 0x80 (vendor-specific AHCI),
           and 0x00 (unspecified — some BIOSes/VMs report this). */
        if (pi != 0x01u && pi != 0x80u && pi != 0x00u) continue;

        /* Enable bus master + memory access. */
        uint32_t cmd = pci_r32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0x04);
        cmd |= 0x06u;
        pci_w32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0x04, cmd);

        /* ABAR = BAR5.  Memory BARs: bits[3:0] are flags, not address.
           Mask them out: bit0=mem, bit1=type_lo, bit2=type_hi, bit3=prefetch.
           Using 0xFFFFFFF0 (not 0xFFFFF000) to support BARs not page-aligned. */
        uint32_t abar = pci_r32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0x24) & 0xFFFFFFF0u;
        if (!abar || abar == 0xFFFFFFF0u) continue;
        g_abar = abar;

        /* Включить AHCI-режим ДО аппаратного сброса: если BIOS оставил
         * контроллер в legacy-IDE режиме (GHC.AE=0), бит GHC.HR
         * игнорируется и HBA reset «не даёт ничего» (аудит F5 п.4). */
        hba_w(HBA_GHC, hba_r(HBA_GHC) | GHC_AE);

        /* Аппаратный сброс HBA: HR=1, контроллер снимает бит сам.
         * Портовые структуры мы конфигурируем с нуля (CLB/FB ниже),
         * так что сброс безопасен. Таймаут 1 с - на случай «залипшего»
         * HR (плохие VM-эмуляции), тогда просто продолжаем без reset. */
        hba_w(HBA_GHC, GHC_AE | GHC_HR);
        for (uint32_t t = 0; t < 1000u; t++) {
            if (!(hba_r(HBA_GHC) & GHC_HR)) break;
            kdelay_ms(1);
        }
        /* После reset AE у ряда контроллеров сбрасывается в 0 - включаем
         * заново (в legacy-режиме MMIO портных регистров недоступны). */
        hba_w(HBA_GHC, hba_r(HBA_GHC) | GHC_AE);

        /* Перебираем только реализованные порты: маска PI + верхняя
         * граница CAP.NP+1 (аудит F5 п.5). */
        uint32_t pi_mask = hba_r(HBA_PI);
        int nports = (int)((hba_r(HBA_CAP) & 0x1Fu)) + 1;
        if (nports > 32) nports = 32;
        for (int p = 0; p < nports; p++) {
            if (!(pi_mask & (1u << p))) continue;

            /* SATA status: DET=3 (device present + phy comm), IPM=1 (active). */
            uint32_t ssts = hba_r(P_SSTS(p));
            if ((ssts & 0x0Fu) != 3u)       continue;
            if (((ssts >> 8) & 0x0Fu) != 1u) continue;

            /* Signature 0x00000101 = ATA device.
               Some SATA controllers report 0 or different values initially —
               we already confirmed the device is present via SSTS DET=3,
               so accept any non-ATAPI signature.  Skip only obvious ATAPI. */
            {
                uint32_t sig = hba_r(P_SIG(p));
                /* ATAPI signature is 0xEB140101 — skip it.
                   Accept 0x00000101 (ATA), 0 (not yet initialized), and others. */
                if (sig == 0xEB140101u) continue;
            }

            /* Initialise port. */
            port_stop(p);
            hba_w(P_CLB(p),  (uint32_t)g_cmdlist); hba_w(P_CLBU(p), 0u);
            hba_w(P_FB(p),   (uint32_t)g_fisbuf);  hba_w(P_FBU(p),  0u);
            hba_w(P_SERR(p), 0xFFFFFFFFu);
            hba_w(P_IS(p),   0xFFFFFFFFu);
            port_start(p);

            /* IDENTIFY DEVICE (0xEC). */
            memset(g_cmdlist, 0, sizeof(g_cmdlist));
            memset(&g_cmdtbl,  0, sizeof(g_cmdtbl));
            memset(g_databuf,  0, 512);

            cmd_hdr_t *h  = &g_cmdlist[0];
            h->flags = 5u;   /* read, CFL=5 */
            h->prdtl = 1;
            h->ctba  = (uint32_t)&g_cmdtbl;

            h2d_fis_t *f  = (h2d_fis_t*)g_cmdtbl.cfis;
            f->type     = 0x27; f->pmport_c = 0x80;
            f->cmd      = 0xEC; /* IDENTIFY */

            g_cmdtbl.prdt[0].dba = (uint32_t)g_databuf;
            g_cmdtbl.prdt[0].dbc = 511;

            if (!port_exec(p)) { port_stop(p); continue; }

            /* Parse IDENTIFY response. */
            uint16_t *id = (uint16_t*)g_databuf;

            /* LBA48 (word 83, бит 10): EXT-команды + полный размер.
             * Слова 60-61 у >128 ГБ дисков содержат обрезанный
             * 28-битный максимум 0x0FFFFFFF; настоящий размер -
             * в словах 100-101 (младшие 32 бита 48-битного поля). */
            g_lba48 = (id[83] & 0x0400u) != 0u;
            if (g_lba48) {
                uint32_t sec48 = ((uint32_t)id[101] << 16) | id[100];
                ahci_drive.sectors = sec48
                                  ? sec48
                                  : (((uint32_t)id[61] << 16) | id[60]);
            } else {
                ahci_drive.sectors  = ((uint32_t)id[61] << 16) | id[60];
            }

            g_port = p;
            ahci_drive.present  = true;
            ahci_drive.io_base  = abar;
            ahci_drive.port_num = p;

            for (int i = 0; i < 20; i++) {
                ahci_drive.model[i*2]   = (char)(id[27+i] >> 8);
                ahci_drive.model[i*2+1] = (char)(id[27+i] & 0xFF);
            }
            ahci_drive.model[40] = 0;
            for (int i = 39; i >= 0; i--) {
                if (ahci_drive.model[i] == ' ' || ahci_drive.model[i] == 0)
                    ahci_drive.model[i] = 0;
                else break;
            }

            /* Expose via ata_primary_master AND ata_drives[] so all existing
               code works — including ata_read/write_sector_drive(). */
            ata_primary_master.present   = true;
            ata_primary_master.io_base   = 0;    /* 0 signals "AHCI, not PIO" */
            ata_primary_master.ctl_base  = 0;
            ata_primary_master.drive_sel = 0;
            ata_primary_master.sectors   = ahci_drive.sectors;
            for (int i = 0; i <= 40; i++)
                ata_primary_master.model[i] = ahci_drive.model[i];

            /* Also populate ata_drives[0] so ata_read/write_sector_drive() work */
            if (ata_drive_count == 0) {
                ata_drives[0] = ata_primary_master;
                ata_drive_count = 1;
            }

            ata_use_ahci = true;
            return;
        }
    }           /* fn loop  */
    }           /* dev loop */
    }           /* bus loop */
}

bool ahci_read_sector(uint32_t lba, void *buf512) {
    /* Границы: 28-битный лимит актуален только без LBA48; размер
     * диска сверяем всегда (см. ata_pio_read). */
    if (g_port < 0) return false;
    if (!g_lba48 && lba >= 0x0FFFFFFFu) return false;
    if (ahci_drive.sectors && lba >= ahci_drive.sectors) return false;
    memset(g_databuf, 0, 512);
    build_cmd(lba, false);
    if (!port_exec(g_port)) return false;
    memcpy(buf512, g_databuf, 512);
    return true;
}

bool ahci_write_sector(uint32_t lba, const void *buf512) {
    /* Границы: 28-битный лимит актуален только без LBA48; размер
     * диска сверяем всегда (см. ata_pio_read). */
    if (g_port < 0) return false;
    if (!g_lba48 && lba >= 0x0FFFFFFFu) return false;
    if (ahci_drive.sectors && lba >= ahci_drive.sectors) return false;
    memcpy(g_databuf, buf512, 512);
    build_cmd(lba, true);
    return port_exec(g_port);
}

/* FLUSH CACHE (0xE7) через обычный путь команды: сброс write-cache
 * диска на носитель. Вызывается инсталлятором после записи
 * MBR/ядра, чтобы данные пережили перезагрузку (аудит F5 п.14). */
bool ahci_flush(void) {
    if (g_port < 0) return false;

    memset(g_cmdlist, 0, sizeof(g_cmdlist));
    memset(&g_cmdtbl,  0, sizeof(g_cmdtbl));

    cmd_hdr_t *h = &g_cmdlist[0];
    h->flags = 5u;          /* CFL=5, без передачи данных */
    h->prdtl = 0;           /* команда без PRDT */
    h->ctba  = (uint32_t)&g_cmdtbl;
    h->ctbau = 0;

    h2d_fis_t *f = (h2d_fis_t*)g_cmdtbl.cfis;
    f->type     = 0x27;
    f->pmport_c = 0x80;     /* C=1: команда */
    f->cmd      = 0xE7;     /* FLUSH CACHE */
    f->dev      = 0xE0u;    /* LBA-режим */

    return port_exec(g_port);
}
