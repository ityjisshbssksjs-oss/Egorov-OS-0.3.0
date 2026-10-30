/* Host-side regression test for Egorov OS's read-only FAT32 VFS.
 * Builds a tiny synthetic MBR/FAT32 disk in RAM; never touches host disks. */
#include "vfs.h"
#include "blockdev.h"
#include "string.h"

extern int printf(const char *fmt, ...);

#define TEST_SECTORS 1000u
static uint8_t g_disk[TEST_SECTORS * 512u];
static blockdev_info_t g_dev;

static void wr16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void wr32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static uint8_t sfn_sum(const uint8_t *sfn) {
    uint8_t sum=0;
    for (int i=0;i<11;i++) sum=(uint8_t)(((sum&1u)?0x80u:0u)+(sum>>1)+sfn[i]);
    return sum;
}
static void write_lfn(uint8_t *e, uint8_t ord, uint8_t sum, const uint16_t *name,
                      uint32_t base) {
    static const uint8_t offs[13]={1,3,5,7,9,14,16,18,20,22,24,28,30};
    memset(e, 0xFF, 32);
    e[0]=ord; e[11]=0x0F; e[12]=0; e[13]=sum; wr16(e+26,0);
    for (uint32_t i=0;i<13;i++) {
        uint32_t n=base+i;
        uint16_t ch = (n < 18u) ? name[n] : (n==18u ? 0u : 0xFFFFu);
        wr16(e+offs[i], ch);
    }
}
static void make_image(void) {
    memset(g_disk, 0, sizeof(g_disk));
    /* MBR: one FAT32-LBA primary partition, LBA 1..999. */
    uint8_t *mbr=g_disk;
    uint8_t *pe=mbr+446;
    pe[4]=0x0C; wr32(pe+8,1); wr32(pe+12,999);
    mbr[510]=0x55; mbr[511]=0xAA;

    /* FAT32 boot sector, deliberately small enough for a unit test. */
    uint8_t *b=g_disk+512;
    wr16(b+11,512); b[13]=1; wr16(b+14,1); b[16]=1;
    wr16(b+17,0); wr16(b+19,0); wr16(b+22,0); wr32(b+32,999);
    wr32(b+36,8); wr32(b+44,2); wr16(b+48,0xFFFF);
    wr32(b+67,0x1234ABCDu);
    memcpy(b+71,"TESTVOL    ",11); b[510]=0x55; b[511]=0xAA;

    /* FAT at disk LBA 2. Cluster 2=root, 3=long-name file, 4=second file. */
    uint8_t *fat=g_disk+2u*512u;
    wr32(fat+2u*4u,0x0FFFFFFFu);
    wr32(fat+3u*4u,0x0FFFFFFFu);
    wr32(fat+4u*4u,0x0FFFFFFFu);

    /* Root cluster at LBA 10. LFN "Long File Name.txt" + alias LONGFI~1.TXT. */
    uint8_t *root=g_disk+10u*512u;
    static const uint16_t longname[18]={'L','o','n','g',' ','F','i','l','e',' ','N','a','m','e','.','t','x','t'};
    uint8_t sfn[11]={'L','O','N','G','F','I','~','1','T','X','T'};
    uint8_t sum=sfn_sum(sfn);
    write_lfn(root,0x42u,sum,longname,13u);
    write_lfn(root+32,0x01u,sum,longname,0u);
    memcpy(root+64,sfn,11); root[64+11]=0x20;
    wr16(root+64+20,0); wr16(root+64+26,3); wr32(root+64+28,11);

    /* Second entry: SECOND.TXT, 3 bytes, cluster 4. */
    uint8_t *e=root+96;
    memcpy(e,"SECOND  TXT",11); e[11]=0x20; wr16(e+20,0); wr16(e+26,4); wr32(e+28,3);
    root[128]=0; /* End of directory */
    memcpy(g_disk+11u*512u,"hello world",11);
    memcpy(g_disk+12u*512u,"xyz",3);

    memset(&g_dev,0,sizeof(g_dev)); g_dev.present=true; g_dev.kind=BLOCKDEV_ATA;
    g_dev.index=0; g_dev.sectors=TEST_SECTORS; memcpy(g_dev.name,"Synthetic FAT32",16);
}

/* Fake block-device API consumed by vfs.c; no hardware writes exist here. */
void blockdev_refresh(void) {}
int blockdev_count(void) { return 1; }
bool blockdev_get(int index, blockdev_info_t *out) {
    if (index != 0 || !out) return false;
    *out = g_dev;
    return true;
}
bool blockdev_read_sector(int index, uint32_t lba, void *out) {
    if (index!=0 || !out || lba>=TEST_SECTORS) return false;
    memcpy(out,g_disk+lba*512u,512u); return true;
}

#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); return 1; } else printf("PASS: %s\n", msg); } while (0)
int main(void) {
    make_image();
    vfs_refresh();
    CHECK(vfs_volume_count()==1, "mount primary MBR FAT32 partition");
    const vfs_volume_t *v=vfs_volume_get(0);
    CHECK(v && v->partition_start==1 && v->root_cluster==2, "partition geometry and root cluster");
    CHECK(strcmp(v->label,"TESTVOL")==0, "read volume label");

    vfs_dirent_t entries[8];
    int n=vfs_readdir(0,0,entries,8);
    CHECK(n==2, "enumerate two real directory entries");
    CHECK(strcmp(entries[0].name,"Long File Name.txt")==0, "decode VFAT long filename");
    CHECK(strcmp(entries[1].name,"SECOND.TXT")==0, "decode 8.3 short filename");

    vfs_dirent_t one[1];
    CHECK(vfs_readdir(0,0,one,1)==1 && vfs_last_list_truncated(), "report directory listing truncation");
    CHECK(vfs_readdir(0,0,entries,8)==2 && !vfs_last_list_truncated(), "clear stale truncation flag on next scan");

    vfs_file_t file;
    CHECK(vfs_open(0,0,"Long File Name.txt",&file), "open file read-only through VFS");
    vfs_close(&file);
    CHECK(vfs_open(0,0,"long file NAME.TXT",&file), "ASCII path lookup is FAT case-insensitive");
    char buf[12]; memset(buf,0,sizeof(buf));
    int a=vfs_read(&file,buf,5); int c=vfs_read(&file,buf+5,6);
    CHECK(a==5 && c==6 && memcmp(buf,"hello world",11)==0, "read file content across sequential calls");
    CHECK(vfs_read(&file,buf,1)==0, "return EOF at file end");
    vfs_close(&file);

    /* Invalid MBR extent must not mount a partition beyond device capacity. */
    wr32(g_disk+446+12,TEST_SECTORS+1u);
    vfs_refresh();
    CHECK(vfs_volume_count()==0, "reject partition extending beyond disk");
    printf("All synthetic FAT32 VFS tests passed.\n");
    return 0;
}
