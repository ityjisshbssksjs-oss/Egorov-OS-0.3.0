#include "vfs.h"
#include "blockdev.h"
#include "string.h"

#define SECTOR_SIZE 512u
#define FAT32_EOC_MIN 0x0FFFFFF8u
#define FAT32_BAD     0x0FFFFFF7u
#define FAT32_MASK    0x0FFFFFFFu
#define FAT32_MAX_SPC 128u
#define DIR_ATTR_LFN  0x0Fu
#define DIR_ATTR_DIR  0x10u
#define DIR_ATTR_VOL  0x08u
#define LFN_UNITS     260

static vfs_volume_t g_volumes[VFS_MAX_VOLUMES];
static int g_volume_count;
static bool g_list_truncated;
static uint8_t g_sector[SECTOR_SIZE];
static uint8_t g_sector2[SECTOR_SIZE];
static vfs_dirent_t g_lookup[VFS_MAX_ENTRIES];
static uint16_t g_lfn[LFN_UNITS];

static uint16_t rd16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static bool pow2_u32(uint32_t n) { return n && !(n & (n - 1u)); }
static bool add_u32_ok(uint32_t a, uint32_t b, uint32_t *out) {
    uint64_t x = (uint64_t)a + (uint64_t)b;
    if (x > 0xFFFFFFFFull) return false;
    *out = (uint32_t)x;
    return true;
}
static bool read_dev(int dev, uint32_t lba, void *out) {
    return blockdev_read_sector(dev, lba, out);
}
static void copy_fixed_trim(char *dst, int cap, const uint8_t *src, int n) {
    int start = 0, end = n;
    while (start < n && src[start] == ' ') start++;
    while (end > start && (src[end-1] == ' ' || src[end-1] == 0)) end--;
    int j = 0;
    while (start < end && j < cap - 1) dst[j++] = (char)src[start++];
    dst[j] = 0;
}
static void name_fallback(char *dst, int cap, const char *name) {
    int i = 0;
    while (name[i] && i < cap - 1) { dst[i] = name[i]; i++; }
    dst[i] = 0;
}
static bool name_equal_fat(const char *a, const char *b) {
    /* FAT long/short names are case-insensitive for ASCII. Full Unicode
     * case-folding is intentionally not claimed by this small read-only VFS. */
    if (!a || !b) return false;
    while (*a && *b) {
        char ca = *a++, cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb) return false;
    }
    return *a == 0 && *b == 0;
}

/* Validates a FAT32 BPB against both the partition and physical disk. */
static bool parse_fat32(int dev, uint32_t start, uint32_t available,
                        const uint8_t *boot, vfs_volume_t *out) {
    if (!out || !boot || boot[510] != 0x55 || boot[511] != 0xAA) return false;
    uint32_t bps = rd16(boot + 11);
    uint32_t spc = boot[13];
    uint32_t reserved = rd16(boot + 14);
    uint32_t nfats = boot[16];
    uint32_t root_entries = rd16(boot + 17);
    uint32_t total16 = rd16(boot + 19);
    uint32_t fatsz16 = rd16(boot + 22);
    uint32_t total32 = rd32(boot + 32);
    uint32_t fatsz32 = rd32(boot + 36);
    uint32_t root_cluster = rd32(boot + 44) & FAT32_MASK;
    uint32_t fsinfo = rd16(boot + 48);
    uint32_t total = total16 ? total16 : total32;
    if (bps != SECTOR_SIZE || !pow2_u32(spc) || spc > FAT32_MAX_SPC ||
        reserved == 0 || nfats == 0 || nfats > 2 || root_entries != 0 ||
        fatsz16 != 0 || fatsz32 == 0 || total == 0 || total > available ||
        root_cluster < 2 || (fsinfo != 0xFFFFu && fsinfo >= reserved)) return false;
    uint64_t fats_total = (uint64_t)nfats * fatsz32;
    uint64_t overhead = (uint64_t)reserved + fats_total;
    if (overhead >= total) return false;
    uint32_t data_sectors = total - (uint32_t)overhead;
    uint32_t clusters = data_sectors / spc;
    if (clusters < 1 || root_cluster >= clusters + 2u) return false;
    /* Every addressable cluster must fit in the FAT. */
    if (((uint64_t)clusters + 2ull) * 4ull > (uint64_t)fatsz32 * SECTOR_SIZE) return false;
    uint32_t end;
    if (!add_u32_ok(start, total, &end)) return false;
    (void)end;

    memset(out, 0, sizeof(*out));
    out->present = true;
    out->blockdev_index = dev;
    out->partition_start = start;
    out->partition_sectors = available;
    out->total_sectors = total;
    out->fat_start = start + reserved;
    out->fat_sectors = fatsz32;
    out->data_start = start + (uint32_t)overhead;
    out->cluster_count = clusters;
    out->root_cluster = root_cluster;
    out->sectors_per_cluster = spc;
    out->bytes_per_cluster = spc * SECTOR_SIZE;
    out->serial = rd32(boot + 67);
    copy_fixed_trim(out->label, VFS_LABEL_MAX, boot + 71, 11);
    if (!out->label[0]) name_fallback(out->label, VFS_LABEL_MAX, "FAT32 volume");
    return true;
}

static void add_volume(const vfs_volume_t *v, const char *devname) {
    if (g_volume_count >= VFS_MAX_VOLUMES || !v || !v->present) return;
    g_volumes[g_volume_count] = *v;
    name_fallback(g_volumes[g_volume_count].device_name,
                  sizeof(g_volumes[g_volume_count].device_name), devname);
    g_volume_count++;
}

void vfs_refresh(void) {
    g_volume_count = 0;
    g_list_truncated = false;
    memset(g_volumes, 0, sizeof(g_volumes));
    blockdev_refresh();
    int devices = blockdev_count();
    for (int di = 0; di < devices; di++) {
        blockdev_info_t dev;
        if (!blockdev_get(di, &dev) || !dev.present || !dev.sectors) continue;
        if (!read_dev(di, 0, g_sector)) continue;

        /* First try a superfloppy FAT32 volume. A valid BPB takes precedence
         * over interpreting the same 0x55AA signature as an MBR. */
        vfs_volume_t vol;
        if (parse_fat32(di, 0, dev.sectors, g_sector, &vol)) {
            add_volume(&vol, dev.name);
            continue;
        }

        if (g_sector[510] != 0x55 || g_sector[511] != 0xAA) continue;
        /* MBR: exactly four primary entries in v0.3.1. Extended partitions and
         * GPT are deliberately not followed until their parsers are implemented. */
        for (int pi = 0; pi < 4; pi++) {
            const uint8_t *pe = g_sector + 446 + pi * 16;
            uint8_t ptype = pe[4];
            uint32_t pstart = rd32(pe + 8);
            uint32_t psize = rd32(pe + 12);
            if (ptype == 0 || pstart == 0 || psize == 0) continue;
            uint64_t pend = (uint64_t)pstart + psize;
            if (pend > dev.sectors || pend > 0x100000000ull) continue;
            if (!read_dev(di, pstart, g_sector2)) continue;
            if (parse_fat32(di, pstart, psize, g_sector2, &vol)) {
                add_volume(&vol, dev.name);
                if (g_volume_count >= VFS_MAX_VOLUMES) {
                    g_list_truncated = true;
                    break;
                }
            }
        }
    }
}

int vfs_volume_count(void) { return g_volume_count; }
const vfs_volume_t *vfs_volume_get(int index) {
    if (index < 0 || index >= g_volume_count) return NULL;
    return &g_volumes[index];
}
bool vfs_last_list_truncated(void) { return g_list_truncated; }

static bool valid_cluster(const vfs_volume_t *v, uint32_t c) {
    return v && c >= 2u && c < v->cluster_count + 2u;
}

static bool fat_next_cluster(const vfs_volume_t *v, uint32_t cluster, uint32_t *next) {
    if (!v || !next || !valid_cluster(v, cluster)) return false;
    uint64_t byte_off = (uint64_t)cluster * 4ull;
    uint64_t lba64 = (uint64_t)v->fat_start + byte_off / SECTOR_SIZE;
    if (lba64 > 0xFFFFFFFFull || lba64 >= (uint64_t)v->partition_start + v->total_sectors) return false;
    if (!read_dev(v->blockdev_index, (uint32_t)lba64, g_sector)) return false;
    uint32_t value = rd32(g_sector + (uint32_t)(byte_off % SECTOR_SIZE)) & FAT32_MASK;
    if (value >= FAT32_EOC_MIN) { *next = 0; return true; }
    if (value == FAT32_BAD || value < 2u || !valid_cluster(v, value)) return false;
    *next = value;
    return true;
}

static bool cluster_lba(const vfs_volume_t *v, uint32_t cluster, uint32_t sector_in_cluster,
                        uint32_t *lba) {
    if (!lba || !valid_cluster(v, cluster) || sector_in_cluster >= v->sectors_per_cluster) return false;
    uint64_t x = (uint64_t)v->data_start +
                 (uint64_t)(cluster - 2u) * v->sectors_per_cluster + sector_in_cluster;
    uint64_t end = (uint64_t)v->partition_start + v->total_sectors;
    if (x >= end || x > 0xFFFFFFFFull) return false;
    *lba = (uint32_t)x;
    return true;
}

static uint8_t lfn_checksum(const uint8_t *sfn) {
    uint8_t sum = 0;
    for (int i = 0; i < 11; i++) sum = (uint8_t)(((sum & 1u) ? 0x80u : 0u) + (sum >> 1) + sfn[i]);
    return sum;
}
static void lfn_reset(void) {
    for (int i = 0; i < LFN_UNITS; i++) g_lfn[i] = 0xFFFFu;
}
static void lfn_copy_piece(const uint8_t *e, uint16_t *dst) {
    static const uint8_t offs[13] = {1,3,5,7,9,14,16,18,20,22,24,28,30};
    uint32_t ordinal = (uint32_t)(e[0] & 0x1Fu);
    /* A valid VFAT LFN ordinal is 1..20 (255 UTF-16 code units). */
    if (ordinal == 0 || ordinal > 20) return;
    uint32_t base = (ordinal - 1u) * 13u;
    if (base >= LFN_UNITS) return;
    for (int i = 0; i < 13 && base + (uint32_t)i < LFN_UNITS; i++)
        dst[base + (uint32_t)i] = rd16(e + offs[i]);
}
static void utf16_to_utf8(char *dst, int cap, const uint16_t *src) {
    int p = 0;
    for (int i = 0; i < LFN_UNITS && p < cap - 1; i++) {
        uint32_t c = src[i];
        if (c == 0 || c == 0xFFFFu) break;
        if (c >= 0xD800u && c <= 0xDBFFu) {
            if (i + 1 < LFN_UNITS && src[i+1] >= 0xDC00u && src[i+1] <= 0xDFFFu) {
                c = 0x10000u + (((c - 0xD800u) << 10) | (src[++i] - 0xDC00u));
            } else c = 0xFFFDu;
        } else if (c >= 0xDC00u && c <= 0xDFFFu) c = 0xFFFDu;
        if (c < 0x80u) {
            if (p + 1 >= cap) break;
            dst[p++] = (char)c;
        } else if (c < 0x800u) {
            if (p + 2 >= cap) break;
            dst[p++] = (char)(0xC0u | (c >> 6));
            dst[p++] = (char)(0x80u | (c & 0x3Fu));
        } else if (c < 0x10000u) {
            if (p + 3 >= cap) break;
            dst[p++] = (char)(0xE0u | (c >> 12));
            dst[p++] = (char)(0x80u | ((c >> 6) & 0x3Fu));
            dst[p++] = (char)(0x80u | (c & 0x3Fu));
        } else {
            if (p + 4 >= cap) break;
            dst[p++] = (char)(0xF0u | (c >> 18));
            dst[p++] = (char)(0x80u | ((c >> 12) & 0x3Fu));
            dst[p++] = (char)(0x80u | ((c >> 6) & 0x3Fu));
            dst[p++] = (char)(0x80u | (c & 0x3Fu));
        }
    }
    dst[p] = 0;
}
static void short_name_to_utf8(char *dst, int cap, const uint8_t *e) {
    int p = 0;
    /* Short names are stored in the OEM codepage; preserve ASCII directly.
     * Non-ASCII short names are shown as '?' until a codepage table is added. */
    for (int i = 0; i < 8 && e[i] != ' '; i++) {
        if (p >= cap - 1) break;
        uint8_t c = e[i] == 0x05 ? 0xE5 : e[i];
        dst[p++] = c < 0x80 ? (char)c : '?';
    }
    int ext = 0;
    while (ext < 3 && e[8 + ext] != ' ') ext++;
    if (ext && p < cap - 1) dst[p++] = '.';
    for (int i = 0; i < ext && p < cap - 1; i++)
        dst[p++] = e[8+i] < 0x80 ? (char)e[8+i] : '?';
    dst[p] = 0;
}

int vfs_readdir(int volume_index, uint32_t directory_cluster,
                vfs_dirent_t *out, int capacity) {
    if (volume_index < 0 || volume_index >= g_volume_count || !out || capacity <= 0) return -1;
    const vfs_volume_t *v = &g_volumes[volume_index];
    uint32_t cluster = directory_cluster ? directory_cluster : v->root_cluster;
    if (!valid_cluster(v, cluster)) return -1;
    int count = 0;
    uint32_t visited = 0;
    bool ended = false;
    bool chain_end = false;
    g_list_truncated = false;
    bool have_lfn = false;
    uint8_t pending_lfn_sum = 0;
    lfn_reset();

    while (valid_cluster(v, cluster) && visited < v->cluster_count && !ended) {
        visited++;
        for (uint32_t si = 0; si < v->sectors_per_cluster && !ended; si++) {
            uint32_t lba;
            if (!cluster_lba(v, cluster, si, &lba) || !read_dev(v->blockdev_index, lba, g_sector)) return -1;
            for (uint32_t off = 0; off < SECTOR_SIZE; off += 32u) {
                const uint8_t *e = g_sector + off;
                if (e[0] == 0x00u) { ended = true; break; }
                if (e[0] == 0xE5u) { have_lfn = false; lfn_reset(); continue; }
                if (e[11] == DIR_ATTR_LFN) {
                    if (e[0] & 0x40u) { lfn_reset(); have_lfn = true; pending_lfn_sum = e[13]; }
                    if (have_lfn && e[13] == pending_lfn_sum) lfn_copy_piece(e, g_lfn);
                    else have_lfn = false;
                    continue;
                }
                if (e[11] & DIR_ATTR_VOL) { have_lfn = false; lfn_reset(); continue; }
                if (e[0] == '.' && (e[1] == ' ' || e[1] == '.')) { have_lfn = false; lfn_reset(); continue; }
                if (count >= capacity) { g_list_truncated = true; ended = true; break; }
                vfs_dirent_t *d = &out[count];
                memset(d, 0, sizeof(*d));
                d->attributes = e[11];
                d->is_dir = (e[11] & DIR_ATTR_DIR) != 0;
                d->first_cluster = ((uint32_t)rd16(e + 20) << 16) | rd16(e + 26);
                d->size = rd32(e + 28);
                if (have_lfn && pending_lfn_sum == lfn_checksum(e)) utf16_to_utf8(d->name, sizeof(d->name), g_lfn);
                else short_name_to_utf8(d->name, sizeof(d->name), e);
                if (!d->name[0]) short_name_to_utf8(d->name, sizeof(d->name), e);
                count++;
                have_lfn = false;
                lfn_reset();
            }
        }
        if (ended) break;
        uint32_t next;
        if (!fat_next_cluster(v, cluster, &next)) return -1;
        if (next == 0) { chain_end = true; break; }
        cluster = next;
    }
    /* A directory must terminate with a 0x00 entry or a valid FAT EOC.
     * Reaching the cluster-count limit otherwise indicates a loop/corrupt chain. */
    if (!ended && !chain_end) return -1;
    return count;
}

bool vfs_stat(int volume_index, uint32_t directory_cluster,
              const char *name, vfs_dirent_t *out) {
    if (!name || !out) return false;
    g_list_truncated = false;
    int n = vfs_readdir(volume_index, directory_cluster, g_lookup, VFS_MAX_ENTRIES);
    if (n < 0) return false;
    for (int i = 0; i < n; i++) {
        if (name_equal_fat(g_lookup[i].name, name)) { *out = g_lookup[i]; return true; }
    }
    return false;
}

bool vfs_open(int volume_index, uint32_t directory_cluster,
              const char *name, vfs_file_t *out) {
    if (!out) return false;
    vfs_dirent_t ent;
    if (!vfs_stat(volume_index, directory_cluster, name, &ent)) return false;
    memset(out, 0, sizeof(*out));
    out->opened = true;
    out->is_dir = ent.is_dir;
    out->volume_index = volume_index;
    out->first_cluster = ent.first_cluster;
    out->current_cluster = ent.first_cluster;
    out->current_cluster_index = 0;
    out->size = ent.size;
    out->position = 0;
    if (!ent.is_dir && ent.size) {
        const vfs_volume_t *v = vfs_volume_get(volume_index);
        if (!valid_cluster(v, ent.first_cluster) ||
            (uint64_t)ent.size > (uint64_t)v->cluster_count * v->bytes_per_cluster) {
            memset(out, 0, sizeof(*out));
            return false;
        }
    }
    return true;
}

int vfs_read(vfs_file_t *file, void *dst, uint32_t bytes) {
    if (!file || !file->opened || file->is_dir || (!dst && bytes)) return -1;
    if (file->position >= file->size || bytes == 0) return 0;
    const vfs_volume_t *v = vfs_volume_get(file->volume_index);
    if (!v) return -1;
    uint32_t remain = file->size - file->position;
    if ((uint64_t)file->size > (uint64_t)v->cluster_count * v->bytes_per_cluster)
        return -1;
    if (bytes > remain) bytes = remain;
    uint32_t done = 0;
    while (done < bytes) {
        uint32_t wanted_cluster_index = file->position / v->bytes_per_cluster;
        if (wanted_cluster_index < file->current_cluster_index) {
            file->current_cluster = file->first_cluster;
            file->current_cluster_index = 0;
        }
        if (wanted_cluster_index >= v->cluster_count)
            return done ? (int)done : -1;
        while (file->current_cluster_index < wanted_cluster_index) {
            uint32_t next;
            if (file->current_cluster_index >= v->cluster_count - 1u ||
                !fat_next_cluster(v, file->current_cluster, &next) || next == 0)
                return done ? (int)done : -1;
            file->current_cluster = next;
            file->current_cluster_index++;
        }
        uint32_t in_cluster = file->position % v->bytes_per_cluster;
        uint32_t sector_in_cluster = in_cluster / SECTOR_SIZE;
        uint32_t byte_in_sector = in_cluster % SECTOR_SIZE;
        uint32_t lba;
        if (!cluster_lba(v, file->current_cluster, sector_in_cluster, &lba) ||
            !read_dev(v->blockdev_index, lba, g_sector)) return done ? (int)done : -1;
        uint32_t chunk = SECTOR_SIZE - byte_in_sector;
        if (chunk > bytes - done) chunk = bytes - done;
        memcpy((uint8_t *)dst + done, g_sector + byte_in_sector, chunk);
        file->position += chunk;
        done += chunk;
    }
    return (int)done;
}

void vfs_close(vfs_file_t *file) {
    if (file) memset(file, 0, sizeof(*file));
}
