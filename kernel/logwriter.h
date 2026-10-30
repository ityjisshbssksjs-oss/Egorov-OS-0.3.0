#ifndef EGOROV_LOGWRITER_H
#define EGOROV_LOGWRITER_H

#include "types.h"
#include "logstick.h"

#define LOGWRITER_MAX_DEVICES 3
#define LOGWRITER_NAME_SIZE   48
#define LOGWRITER_RESULT_SIZE 144
#define LOGWRITER_WIN_W       580
#define LOGWRITER_WIN_H       340

#define LOGWRITER_EXPORT_FAIL         0
#define LOGWRITER_EXPORT_OK           1
#define LOGWRITER_EXPORT_RAW_REQUIRED 2

typedef struct {
    int transport;                 /* LS_TR_*; one currently exposed MSD per transport */
    uint32_t sectors;
    char name[LOGWRITER_NAME_SIZE];
} logwriter_device_t;

/* Enumerate currently initialized USB Mass Storage transports. This does not
 * imply that every physical drive behind one controller is visible. */
int logwriter_list_devices(logwriter_device_t *out, int max_devices);

/* Write EGOROV.LOG into a validated, non-boot FAT32 data partition of any
 * volume label; read back and compare every byte. Returns RAW_REQUIRED only
 * when the device can be read but no safe FAT32 data partition is recognized. */
int logwriter_export(int transport, uint32_t expected_sectors,
                      const char *expected_name, uint32_t *bytes_written,
                      char *result, uint32_t result_capacity);
/* Destructive raw format (EGORAW1 at LBA 0, payload starting at LBA 1).
 * Caller MUST have presented a clear warning and obtained confirmation. */
int logwriter_export_raw(int transport, uint32_t expected_sectors,
                         const char *expected_name, uint32_t *bytes_written,
                         char *result, uint32_t result_capacity);

/* UI entry points. */
void logwriter_ui_init(void);
void logwriter_ui_draw(int wx, int wy, int w, int h);
bool logwriter_ui_click(int wx, int wy, int mx, int my); /* true = close requested */
bool logwriter_ui_key(uint32_t key);                 /* true = close requested */

#endif
