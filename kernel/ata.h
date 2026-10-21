#ifndef EGOROV_ATA_H
#define EGOROV_ATA_H

#include "types.h"

#define ATA_MAX_DRIVES 4

typedef struct {
    bool     present;
    char     model[41];
    uint32_t sectors;      /* user-addressable sector count */
    uint16_t io_base;      /* I/O base port (0x1F0 or 0x170) */
    uint16_t ctl_base;     /* Control base (0x3F6 or 0x376) */
    uint8_t  drive_sel;    /* 0xA0 = master, 0xB0 = slave */
    bool     lba48;        /* IDENTIFY word 83 bit 10 — EXT-команды */
} ata_drive_t;

/* All detected drives (up to 4). */
extern ata_drive_t ata_drives[ATA_MAX_DRIVES];
extern int         ata_drive_count;

/* First successfully detected drive — kept for backward compatibility. */
extern ata_drive_t ata_primary_master;

/* Set to true by ahci_init() when an AHCI drive is used instead of ATA PIO. */
extern bool ata_use_ahci;

/* Probe all four ATA positions; fills ata_drives[] and ata_primary_master. */
void ata_init(void);

/* Read/write using ata_primary_master (or AHCI if ata_use_ahci). */
bool ata_read_sector(uint32_t lba, void *buf512);
bool ata_write_sector(uint32_t lba, const void *buf512);

/* Read/write using a specific drive index (0-based into ata_drives[]). */
bool ata_read_sector_drive(int drive_idx, uint32_t lba, void *buf512);
bool ata_write_sector_drive(int drive_idx, uint32_t lba, const void *buf512);

/* FLUSH CACHE — сброс write-cache диска на носитель (для инсталлятора). */
bool ata_flush_cache(void);

#endif
