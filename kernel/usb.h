#ifndef EGOROV_USB_H
#define EGOROV_USB_H

#include "types.h"

/* ── USB Mass Storage result codes ── */
#define USB_OK          0
#define USB_ERR_NODEV   1   /* no USB host controller found */
#define USB_ERR_TIMEOUT 2   /* device not responding */
#define USB_ERR_IO      3   /* SCSI / bulk transfer error */

/* ── Detected USB mass-storage device ── */
typedef struct {
    bool     present;
    uint32_t sectors;    /* total sector count (512-byte LBA) */
    char     vendor[9];  /* SCSI INQUIRY vendor string (8 chars + NUL) */
    char     product[17];/* SCSI INQUIRY product string (16 chars + NUL) */
    /* Internal — do not use directly. */
    uint32_t mmio_base;  /* EHCI: BAR MMIO; UHCI: 0 (uses _io_base) */
    uint16_t io_base;    /* UHCI: I/O base port */
    uint8_t  bulk_in;    /* endpoint address */
    uint8_t  bulk_out;   /* endpoint address */
    uint8_t  iface;      /* interface number */
    uint8_t  ctrl_type;  /* 0 = UHCI, 1 = EHCI */
} usb_drive_t;

/* Globally available detected USB mass-storage drive (if any). */
extern usb_drive_t usb_drive;

/* Scan PCI for USB host controllers, enumerate devices, find mass storage.
   Call once at boot (after pci is accessible). */
void usb_init(void);

/* Read one 512-byte sector from the USB drive (LBA28).
   Returns true on success. */
bool usb_read_sector(uint32_t lba, void *buf512);

/* Write one 512-byte sector to the USB drive (LBA28).
   Returns true on success. */
bool usb_write_sector(uint32_t lba, const void *buf512);

#endif
