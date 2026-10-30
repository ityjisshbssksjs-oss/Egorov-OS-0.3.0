#include "blockdev.h"
#include "ata.h"
#include "usb.h"
#include "mouse_hw.h"
#include "string.h"

static blockdev_info_t g_devs[BLOCKDEV_MAX];
static int g_count;

static void copy_name(char *dst, int cap, const char *src, const char *fallback) {
    int i = 0;
    const char *s = (src && src[0]) ? src : fallback;
    if (!s) s = "Disk";
    while (s[i] && i < cap - 1) { dst[i] = s[i]; i++; }
    dst[i] = 0;
}

static void add_dev(blockdev_kind_t kind, int index, uint32_t sectors,
                    const char *name, const char *fallback) {
    if (g_count >= BLOCKDEV_MAX || sectors == 0) return;
    blockdev_info_t *d = &g_devs[g_count++];
    memset(d, 0, sizeof(*d));
    d->present = true;
    d->kind = kind;
    d->index = index;
    d->sectors = sectors;
    copy_name(d->name, BLOCKDEV_NAME_MAX, name, fallback);
}

void blockdev_refresh(void) {
    g_count = 0;
    memset(g_devs, 0, sizeof(g_devs));

    /* ATA PIO and AHCI both use the ATA public API. ahci_init() exposes its
     * selected disk as ata_drives[0], so callers don't need an AHCI special case. */
    for (int i = 0; i < ata_drive_count && i < ATA_MAX_DRIVES; i++) {
        if (!ata_drives[i].present || !ata_drives[i].sectors) continue;
        add_dev(BLOCKDEV_ATA, i, ata_drives[i].sectors,
                ata_drives[i].model, "ATA/SATA disk");
    }

    /* Keep old UHCI/EHCI USB MSD support where it has already been initialized. */
    if (usb_drive.present && usb_drive.sectors)
        add_dev(BLOCKDEV_USB_LEGACY, 0, usb_drive.sectors, usb_drive.product,
                "USB Mass Storage");

    /* Build 9's mouse_hw layer already enumerates OHCI/xHCI mass-storage
     * devices. Reuse its read-only block accessors rather than starting a
     * second USB enumeration from the Explorer. */
    if (mouse_hw_msd_present() && mouse_hw_msd_sectors())
        add_dev(BLOCKDEV_USB_OHCI, 0, mouse_hw_msd_sectors(),
                mouse_hw_msd_name(), "OHCI USB storage");
    if (mouse_hw_xhci_msd_present() && mouse_hw_xhci_msd_sectors())
        add_dev(BLOCKDEV_USB_XHCI, 0, mouse_hw_xhci_msd_sectors(),
                mouse_hw_xhci_msd_name(), "xHCI USB storage");
}

int blockdev_count(void) { return g_count; }

bool blockdev_get(int index, blockdev_info_t *out) {
    if (!out || index < 0 || index >= g_count) return false;
    *out = g_devs[index];
    return true;
}

bool blockdev_read_sector(int index, uint32_t lba, void *sector512) {
    if (!sector512 || index < 0 || index >= g_count) return false;
    blockdev_info_t *d = &g_devs[index];
    if (!d->present || lba >= d->sectors) return false;
    switch (d->kind) {
        case BLOCKDEV_ATA:
            return ata_read_sector_drive(d->index, lba, sector512);
        case BLOCKDEV_USB_LEGACY:
            return usb_read_sector(lba, sector512);
        case BLOCKDEV_USB_OHCI:
            return mouse_hw_msd_read(lba, sector512);
        case BLOCKDEV_USB_XHCI:
            return mouse_hw_xhci_msd_read(lba, sector512);
        default:
            return false;
    }
}
