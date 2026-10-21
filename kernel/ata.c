#include "ata.h"
#include "io.h"
#include "string.h"
#include "timer.h"    /* kdelay_us — реальные таймауты вместо итерационных циклов */

ata_drive_t ata_drives[ATA_MAX_DRIVES];
int         ata_drive_count = 0;
ata_drive_t ata_primary_master;

/* Set to true by ahci_init() when an AHCI drive is used instead of ATA PIO. */
bool ata_use_ahci = false;

/* Forward declarations for AHCI fallback (defined in ahci.c). */
extern bool ahci_read_sector(uint32_t lba, void *buf512);
extern bool ahci_write_sector(uint32_t lba, const void *buf512);
extern bool ahci_flush(void);

static void ata_400ns_p(uint16_t ctl) {
    for (int i = 0; i < 4; i++) (void)inb(ctl);
}

/* Таймауты в РЕАЛЬНОМ времени (kdelay), а не «голые» итерации:
 * на 4 ГГц миллион итераций inb — это единицы миллисекунд, и медленный
 * диск/пустой канал отваливался раньше времени (аудит F5 п.9). */
static bool ata_wait_ready_p(uint16_t io) {
    /* DRDY/BSY: до 3 с, опрос раз в 1 мс (регистр меняется медленно) */
    for (uint32_t t = 0; t < 3000u; t++) {
        uint8_t s = inb(io + 7);
        if (s & 0x01) return false;
        if (!(s & 0x80) && (s & 0x40)) return true;
        kdelay_us(1000);
    }
    return false;
}

static bool ata_wait_drq_p(uint16_t io) {
    /* DRQ появляется за ~мс: до 1 с, опрос раз в 100 мкс */
    for (uint32_t t = 0; t < 10000u; t++) {
        uint8_t s = inb(io + 7);
        if (s & 0x01) return false;
        if (!(s & 0x80) && (s & 0x08)) return true;
        kdelay_us(100);
    }
    return false;
}

/* Probe one ATA position; returns true and fills *drv on success. */
static bool ata_probe(uint16_t io, uint16_t ctl, uint8_t sel, ata_drive_t *drv) {
    memset(drv, 0, sizeof(*drv));

    /* Floating bus: STATUS == 0xFF means nothing attached. */
    outb(io + 6, sel);
    ata_400ns_p(ctl);
    uint8_t st = inb(io + 7);
    if (st == 0xFF) return false;

    /* Send IDENTIFY. */
    outb(io + 2, 0); outb(io + 3, 0); outb(io + 4, 0); outb(io + 5, 0);
    outb(io + 7, 0xEC);
    ata_400ns_p(ctl);

    st = inb(io + 7);
    if (st == 0) return false;

    /* Wait BSY=0 — до 3 с реального времени. */
    for (uint32_t t = 0; t < 3000u; t++) {
        st = inb(io + 7);
        if (!(st & 0x80)) break;
        kdelay_us(1000);
    }
    /* Явная сигнатура ATAPI (cyl low=0x14, cyl high=0xEB): пакетным
     * приводам нужен IDENTIFY PACKET DEVICE (0xA1), а не 0xEC —
     * пропускаем (аудит F5 п.11). Прочие ненулевые cylinder-регистры
     * тоже признак не-ATA устройства. */
    uint8_t cyl_lo = inb(io + 4), cyl_hi = inb(io + 5);
    if (cyl_lo == 0x14 && cyl_hi == 0xEB) return false;   /* ATAPI */
    if (cyl_lo || cyl_hi) return false;                   /* не ATA */
    if (!ata_wait_drq_p(io)) return false;

    uint16_t buf[256];
    for (int i = 0; i < 256; i++) buf[i] = inw(io);
    /* Раньше ERR после PIO-передачи IDENTIFY не проверялся: битые
     * данные попадали в model/sectors (аудит F5, своя находка). */
    if (inb(io + 7) & 0x01) return false;

    drv->present   = true;
    drv->io_base   = io;
    drv->ctl_base  = ctl;
    drv->drive_sel = sel;

    /* Model: words 27-46, big-endian pairs. */
    for (int i = 0; i < 20; i++) {
        drv->model[i*2]   = (char)(buf[27+i] >> 8);
        drv->model[i*2+1] = (char)(buf[27+i] & 0xFF);
    }
    drv->model[40] = 0;
    for (int i = 39; i >= 0; i--) {
        if (drv->model[i] == ' ' || drv->model[i] == 0) drv->model[i] = 0;
        else break;
    }
    drv->sectors = ((uint32_t)buf[61] << 16) | buf[60];
    /* LBA48 (word 83, бит 10): у >128 ГБ дисков слова 60-61 содержат
     * обрезанный 28-битный максимум 0x0FFFFFFF; настоящий размер —
     * в словах 100-101 (младшие 32 бита 48-битного поля). */
    drv->lba48 = (buf[83] & 0x0400u) != 0u;
    if (drv->lba48) {
        uint32_t sec48 = ((uint32_t)buf[101] << 16) | buf[100];
        if (sec48) drv->sectors = sec48;
    }
    return true;
}

void ata_init(void) {
    memset(&ata_primary_master, 0, sizeof(ata_primary_master));
    memset(ata_drives, 0, sizeof(ata_drives));
    ata_drive_count = 0;
    ata_use_ahci = false;

    /* Probe all four ATA positions. */
    typedef struct { uint16_t io, ctl; uint8_t sel; } probe_t;
    static const probe_t p[4] = {
        { 0x1F0, 0x3F6, 0xA0 },
        { 0x1F0, 0x3F6, 0xB0 },
        { 0x170, 0x376, 0xA0 },
        { 0x170, 0x376, 0xB0 },
    };
    for (int i = 0; i < 4 && ata_drive_count < ATA_MAX_DRIVES; i++) {
        if (ata_probe(p[i].io, p[i].ctl, p[i].sel, &ata_drives[ata_drive_count])) {
            ata_drive_count++;
        }
    }

    /* Keep ata_primary_master as first found drive for backward compat. */
    if (ata_drive_count > 0) {
        ata_primary_master = ata_drives[0];
    }
}

/* --- PIO read/write for a specific drive struct --- */
static bool ata_pio_read(ata_drive_t *d, uint32_t lba, void *buf512) {
    uint16_t io  = d->io_base;
    uint16_t ctl = d->ctl_base;
    uint8_t  drv = d->drive_sel;

    /* LBA28: биты выше 27-го молча усекались -> запись «не в тот сектор»
     * на дисках >128 ГБ; заодно сверяем с размером диска.
     * С поддержкой LBA48 используется READ SECTORS EXT (аудит F5 п.10). */
    if (!d->lba48 && lba >= 0x0FFFFFFFu) return false;
    if (d->sectors && lba >= d->sectors) return false;

    /* ATA spec: select the drive FIRST, wait 400ns, THEN wait DRDY.
       Without this, after probing an ATAPI slave, the bus may still have
       the slave selected and its ERR/BSY state confuses the master read. */
    outb(io+6, (drv & 0xF0) | 0x40);
    ata_400ns_p(ctl);
    if (!ata_wait_ready_p(io)) return false;

    if (d->lba48 && lba >= 0x0FFFFFFFu) {
        /* LBA48: двойной набор регистров + READ SECTORS EXT (0x24).
         * LBA — 32-битный, старшие биты [47:32] всегда 0, но считаем
         * через uint64_t — без UB при сдвиге и готово к расширению. */
        uint64_t l64 = (uint64_t)lba;
        outb(io+6, 0x40);                      /* dev: только бит LBA */
        ata_400ns_p(ctl);
        outb(io+1, 0);                         /* features (и EXP-копия) */
        outb(io+2, 0);                         /* счётчик секторов, старший байт */
        outb(io+3, (uint8_t)(l64 >> 24));      /* LBA[31:24] */
        outb(io+4, (uint8_t)(l64 >> 32));      /* LBA[39:32] */
        outb(io+5, (uint8_t)(l64 >> 40));      /* LBA[47:40] */
        outb(io+2, 1);                         /* счётчик секторов */
        outb(io+3, (uint8_t)(l64));            /* LBA[7:0] */
        outb(io+4, (uint8_t)(l64 >> 8));
        outb(io+5, (uint8_t)(l64 >> 16));
        outb(io+7, 0x24);                      /* READ SECTORS EXT */
    } else {
        outb(io+6, (drv & 0xF0) | 0x40 | ((lba >> 24) & 0x0F));
        ata_400ns_p(ctl);
        outb(io+1, 0);
        outb(io+2, 1);
        outb(io+3,  lba        & 0xFF);
        outb(io+4, (lba >>  8) & 0xFF);
        outb(io+5, (lba >> 16) & 0xFF);
        outb(io+7, 0x20);
    }
    if (!ata_wait_drq_p(io)) return false;
    insw(io, buf512, 256);
    return true;
}

static bool ata_pio_write(ata_drive_t *d, uint32_t lba, const void *buf512) {
    uint16_t io  = d->io_base;
    uint16_t ctl = d->ctl_base;
    uint8_t  drv = d->drive_sel;

    /* LBA28-границы (см. ata_pio_read); с поддержкой LBA48 —
     * WRITE SECTORS EXT (аудит F5 п.10). */
    if (!d->lba48 && lba >= 0x0FFFFFFFu) return false;
    if (d->sectors && lba >= d->sectors) return false;

    /* Select drive first, then wait DRDY (same ATA spec fix as ata_pio_read). */
    outb(io+6, (drv & 0xF0) | 0x40);
    ata_400ns_p(ctl);
    if (!ata_wait_ready_p(io)) return false;

    if (d->lba48 && lba >= 0x0FFFFFFFu) {
        /* LBA48: двойной набор регистров + WRITE SECTORS EXT (0x34) */
        uint64_t l64 = (uint64_t)lba;
        outb(io+6, 0x40);                      /* dev: только бит LBA */
        ata_400ns_p(ctl);
        outb(io+1, 0);                         /* features (и EXP-копия) */
        outb(io+2, 0);                         /* счётчик секторов, старший байт */
        outb(io+3, (uint8_t)(l64 >> 24));      /* LBA[31:24] */
        outb(io+4, (uint8_t)(l64 >> 32));      /* LBA[39:32] */
        outb(io+5, (uint8_t)(l64 >> 40));      /* LBA[47:40] */
        outb(io+2, 1);                         /* счётчик секторов */
        outb(io+3, (uint8_t)(l64));            /* LBA[7:0] */
        outb(io+4, (uint8_t)(l64 >> 8));
        outb(io+5, (uint8_t)(l64 >> 16));
        outb(io+7, 0x34);                      /* WRITE SECTORS EXT */
    } else {
        outb(io+6, (drv & 0xF0) | 0x40 | ((lba >> 24) & 0x0F));
        ata_400ns_p(ctl);
        outb(io+1, 0);
        outb(io+2, 1);
        outb(io+3,  lba        & 0xFF);
        outb(io+4, (lba >>  8) & 0xFF);
        outb(io+5, (lba >> 16) & 0xFF);
        outb(io+7, 0x30);
    }
    if (!ata_wait_drq_p(io)) return false;
    const uint16_t *src = (const uint16_t *)buf512;
    for (int i = 0; i < 256; i++) outw(io, src[i]);
    if (!ata_wait_ready_p(io)) return false;
    uint8_t st = inb(io+7);
    if (st & 0x01) return false;
    /* FLUSH CACHE: EXT-вариант (0xEA) для LBA48-дисков */
    outb(io+7, d->lba48 ? 0xEA : 0xE7);
    if (!ata_wait_ready_p(io)) return false;
    st = inb(io+7);
    if (st & 0x01) return false;
    return true;
}

bool ata_read_sector(uint32_t lba, void *buf512) {
    if (ata_use_ahci) return ahci_read_sector(lba, buf512);
    if (!ata_primary_master.present) return false;
    return ata_pio_read(&ata_primary_master, lba, buf512);
}

bool ata_write_sector(uint32_t lba, const void *buf512) {
    if (ata_use_ahci) return ahci_write_sector(lba, buf512);
    if (!ata_primary_master.present) return false;
    return ata_pio_write(&ata_primary_master, lba, buf512);
}

bool ata_read_sector_drive(int drive_idx, uint32_t lba, void *buf512) {
    if (drive_idx < 0 || drive_idx >= ata_drive_count) return false;
    if (!ata_drives[drive_idx].present) return false;
    /* On AHCI systems drive_idx 0 is the AHCI drive — use AHCI path */
    if (ata_use_ahci && drive_idx == 0) return ahci_read_sector(lba, buf512);
    return ata_pio_read(&ata_drives[drive_idx], lba, buf512);
}

bool ata_write_sector_drive(int drive_idx, uint32_t lba, const void *buf512) {
    if (drive_idx < 0 || drive_idx >= ata_drive_count) return false;
    if (!ata_drives[drive_idx].present) return false;
    /* On AHCI systems drive_idx 0 is the AHCI drive — use AHCI path */
    if (ata_use_ahci && drive_idx == 0) return ahci_write_sector(lba, buf512);
    return ata_pio_write(&ata_drives[drive_idx], lba, buf512);
}

/* Явный сброс write-cache диска на носитель (аудит F5 п.14):
 * AHCI — своя команда FLUSH CACHE через cmd-слот, ATA PIO —
 * FLUSH CACHE (0xE7) / FLUSH CACHE EXT (0xEA) по портам.
 * PIO-путь и так флэшит после каждой записи — здесь дожимаем
 * для инсталлятора финальный сброс после больших циклов записи. */
bool ata_flush_cache(void) {
    if (ata_use_ahci) return ahci_flush();
    if (!ata_primary_master.present) return false;

    uint16_t io  = ata_primary_master.io_base;
    uint16_t ctl = ata_primary_master.ctl_base;
    uint8_t  drv = ata_primary_master.drive_sel;

    outb(io+6, (drv & 0xF0) | 0x40);
    ata_400ns_p(ctl);
    if (!ata_wait_ready_p(io)) return false;
    outb(io+7, ata_primary_master.lba48 ? 0xEAu : 0xE7u);
    if (!ata_wait_ready_p(io)) return false;
    return (inb(io+7) & 0x01) == 0;
}
