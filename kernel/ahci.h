#ifndef EGOROV_AHCI_H
#define EGOROV_AHCI_H

#include "types.h"

typedef struct {
    bool     present;
    char     model[41];
    uint32_t sectors;
    uint32_t io_base;   /* ABAR physical address (0 if unused) */
    int      port_num;
} ahci_drive_t;

extern ahci_drive_t ahci_drive;

/* Scan PCI for an AHCI controller.  If a drive is found, fills
 * ata_primary_master with model/sectors and sets ata_use_ahci = true
 * so that ata_read_sector / ata_write_sector transparently dispatch here.
 * Called after ata_init() — is a no-op if ATA already found a disk. */
void ahci_init(void);

bool ahci_read_sector(uint32_t lba, void *buf512);
bool ahci_write_sector(uint32_t lba, const void *buf512);

/* FLUSH CACHE (0xE7) — сброс write-cache диска на носитель.
 * Используется инсталлятором после записи MBR/ядра. */
bool ahci_flush(void);

#endif
