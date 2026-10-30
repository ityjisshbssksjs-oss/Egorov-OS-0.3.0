#ifndef EGOROV_BLOCKDEV_H
#define EGOROV_BLOCKDEV_H

#include "types.h"

#define BLOCKDEV_MAX 12
#define BLOCKDEV_NAME_MAX 48

typedef enum {
    BLOCKDEV_ATA = 1,
    BLOCKDEV_USB_LEGACY = 2,
    BLOCKDEV_USB_OHCI = 3,
    BLOCKDEV_USB_XHCI = 4
} blockdev_kind_t;

typedef struct {
    bool present;
    blockdev_kind_t kind;
    int index;                 /* ATA drive index; ignored for singleton USB transports */
    uint32_t sectors;          /* 512-byte sectors */
    char name[BLOCKDEV_NAME_MAX];
} blockdev_info_t;

/* Refresh the inventory from already initialized drivers. Does not initialize
 * controllers itself and does not write to any device. */
void blockdev_refresh(void);
int blockdev_count(void);
bool blockdev_get(int index, blockdev_info_t *out);
bool blockdev_read_sector(int index, uint32_t lba, void *sector512);

#endif
