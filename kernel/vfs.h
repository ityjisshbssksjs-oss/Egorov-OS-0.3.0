#ifndef EGOROV_VFS_H
#define EGOROV_VFS_H

#include "types.h"

#define VFS_MAX_VOLUMES 16
#define VFS_MAX_ENTRIES 128
#define VFS_NAME_MAX 96
#define VFS_LABEL_MAX 16

/* Read-only FAT32 volume. Partition and cluster offsets are validated against
 * the underlying block-device size before use. */
typedef struct {
    bool present;
    int blockdev_index;
    uint32_t partition_start;
    uint32_t partition_sectors;
    uint32_t total_sectors;
    uint32_t fat_start;
    uint32_t fat_sectors;
    uint32_t data_start;
    uint32_t cluster_count;
    uint32_t root_cluster;
    uint32_t sectors_per_cluster;
    uint32_t bytes_per_cluster;
    uint32_t serial;
    char label[VFS_LABEL_MAX];
    char device_name[48];
} vfs_volume_t;

typedef struct {
    char name[VFS_NAME_MAX];
    bool is_dir;
    uint8_t attributes;
    uint32_t size;
    uint32_t first_cluster;
} vfs_dirent_t;

typedef struct {
    bool opened;
    bool is_dir;
    int volume_index;
    uint32_t first_cluster;
    uint32_t current_cluster;
    uint32_t current_cluster_index;
    uint32_t size;
    uint32_t position;
} vfs_file_t;

void vfs_refresh(void);
int vfs_volume_count(void);
const vfs_volume_t *vfs_volume_get(int index);
/* cluster 0 means the volume root. Returns entry count, or -1 on I/O/FS error. */
int vfs_readdir(int volume_index, uint32_t cluster,
                vfs_dirent_t *out, int capacity);
bool vfs_last_list_truncated(void);
bool vfs_stat(int volume_index, uint32_t directory_cluster,
              const char *name, vfs_dirent_t *out);
bool vfs_open(int volume_index, uint32_t directory_cluster,
              const char *name, vfs_file_t *out);
/* Read-only sequential file read: byte count read, 0 at EOF, -1 on error. */
int vfs_read(vfs_file_t *file, void *dst, uint32_t bytes);
void vfs_close(vfs_file_t *file);

#endif
