/* installer.c — Windows-style disk/partition wizard for Egorov OS
   Fixes v2.1:
   - ALL return → do_reboot() (fixes freeze on Cancel/X/<)
   - Clicking rows while creating does NOT cancel create mode
   - g_creating only cancelled by ESC/Apply/Cancel button
   - hov updated every frame (not just on move/key)
   - mbr_ensure() called before every partition write
   - Serial debug for every ATA write operation
   - Далее always enabled (can install to disk 0 even without partition)
   - Numpad digit keys supported
   - Better drive display in table
*/

#include "fb.h"
#include "text.h"
#include "gui.h"
#include "ps2.h"
#include "usb_mouse.h"
#include "mouse_hw.h"
#include "ata.h"
#include "ahci.h"
#include "string.h"
#include "io.h"
#include "power.h"
#include "timer.h"    /* kdelay_ms — кадровые паузы реального времени */

/* ── Serial debug ─────────────────────────────────────────── */
#define COM1 0x3F8
static void iputc(char c){while(!(inb(COM1+5)&0x20)){}outb(COM1,(uint8_t)c);}
static void iputs(const char *s){for(;*s;s++){if(*s=='\n')iputc('\r');iputc(*s);}}
static void iputu(uint32_t v){
    char b[12];int p=11;b[p]=0;
    if(!v){iputs("0");return;}
    while(v){b[--p]='0'+(char)(v%10);v/=10;}iputs(b+p);
}
static void iputx(uint32_t v){
    iputs("0x");
    char b[9];int p=8;b[p]=0;
    for(int i=0;i<8;i++){uint8_t n=(uint8_t)(v&0xF);b[--p]=(char)(n<10?'0'+n:'A'+n-10);v>>=4;}
    iputs(b+p);
}

/* ── External kernel/boot symbols ───────────────────────────── */
extern uint8_t _kernel_image_start[];
extern uint8_t _kernel_image_end[];
extern uint8_t _bootloader_start[];
extern uint8_t _bootloader_end[];

/* ── Delay ──────────────────────────────────────────────────── */
/* bdelay() заменён на kdelay_ms() из kernel/timer.h: «голый» цикл
 * volatile+inb на 4 ГГц длится в разы меньше расчётного, пауза
 * кадра «съезжала» (аудит F5 п.22). */

/* ═══════════════════════ MBR structures ════════════════════════ */
#define PART_EMPTY  0x00
#define PART_FAT32  0x0B
#define PART_EXT2   0x83
#define PART_SWAP   0x82
#define PART_NTFS   0x07
#define PART_EOS    0xEE

typedef struct {
    uint8_t  status;
    uint8_t  chs_start[3];
    uint8_t  type;
    uint8_t  chs_end[3];
    uint32_t lba_start;
    uint32_t lba_size;
} __attribute__((packed)) mbr_part_t;

typedef struct {
    uint8_t    bootstrap[446];
    mbr_part_t parts[4];
    uint8_t    sig[2];
} __attribute__((packed)) mbr_t;

static mbr_t g_mbr;
static bool  g_mbr_ok   = false;
static int   g_drv      = 0;    /* currently selected drive */

/* ── Slider state for partition create ───────────────────────── */
static uint32_t g_slider_mb   = 100;  /* selected partition size in MB */
static bool     g_slider_drag = false;

/* ── MBR helpers ─────────────────────────────────────────────── */
static void lba_to_chs(uint32_t lba, uint8_t *c){
    uint32_t cy=lba/(255*63),rm=lba%(255*63);
    uint8_t h=(uint8_t)(rm/63),s=(uint8_t)(rm%63)+1;
    if(cy>1023){cy=1023;h=254;s=63;}
    c[0]=h; c[1]=(uint8_t)((s&0x3F)|((cy>>8)<<6)); c[2]=(uint8_t)(cy&0xFF);
}
static uint32_t sec2mb(uint32_t s){return s/2048;}
static uint32_t mb2sec(uint32_t m){return m*2048;}
static uint32_t parse_uint(const char *s){
    uint32_t v=0;
    for(int i=0;s[i]>='0'&&s[i]<='9';i++) v=v*10+(uint32_t)(s[i]-'0');
    return v;
}
static void app_str(char *d,int *p,const char *s){
    for(int i=0;s[i]&&*p<79;i++) d[(*p)++]=s[i];
    d[*p]=0;
}
static const char *ptype(uint8_t t){
    switch(t){
        case PART_FAT32: return "FAT32";
        case PART_EXT2:  return "Linux";
        case PART_SWAP:  return "Swap";
        case PART_EOS:   return "EgorovOS";
        case PART_NTFS:  return "NTFS";
        case 0x05:       return "Extended";
        case 0x0C:       return "FAT32X";
        default:         return "";
    }
}

static bool mbr_load(void){
    uint8_t b[512];
    iputs("mbr_load drv="); iputu((uint32_t)g_drv); iputs("\n");
    if(!ata_read_sector_drive(g_drv,0,b)){
        iputs("mbr_load: read fail\n"); return false;
    }
    iputs("mbr_load: sig=");
    iputx(((uint32_t)b[510]<<8)|b[511]); iputs("\n");
    /* BUG FIX: if signature is invalid, treat disk as blank (no partitions).
       Without this check, garbage bytes on a new disk look like valid partitions. */
    if(b[510]!=0x55||b[511]!=0xAA){
        iputs("mbr_load: invalid sig, blank MBR\n");
        memset(&g_mbr,0,sizeof(g_mbr));
        g_mbr.sig[0]=0x55; g_mbr.sig[1]=0xAA;
        g_mbr_ok=true;
        return true;
    }
    memcpy(&g_mbr,b,512); g_mbr_ok=true;
    return true;
}
static bool mbr_save(void){
    uint8_t b[512]; memcpy(b,&g_mbr,512);
    b[510]=0x55; b[511]=0xAA;
    iputs("mbr_save drv="); iputu((uint32_t)g_drv); iputs("\n");
    bool r=ata_write_sector_drive(g_drv,0,b);
    iputs("mbr_save result="); iputs(r?"OK":"FAIL"); iputs("\n");
    return r;
}
/* Ensure g_mbr is valid (call before any write if mbr_load may have failed) */
static void mbr_ensure(void){
    if(!g_mbr_ok){
        memset(&g_mbr,0,sizeof(g_mbr));
        g_mbr.sig[0]=0x55; g_mbr.sig[1]=0xAA; g_mbr_ok=true;
        iputs("mbr_ensure: initialized blank MBR\n");
    }
}

/* Find first empty MBR slot */
static int mbr_free(void){
    for(int i=0;i<4;i++)
        if(g_mbr.parts[i].type==PART_EMPTY&&g_mbr.parts[i].lba_size==0) return i;
    return -1;
}
/* LBA of first byte after all used partitions.
 * Резерв: 2048..2815 - ядро (HDD-загрузчик F4 читает 768 секторов
 * = 384 КБ с LBA 2048; Makefile следит за лимитом).  Раньше
 * резерв был 512 секторов (256 КБ): на расширенном ядре новый
 * раздел создавался прямо ПОВЕРХ его хвоста и форматирование
 * уничтожало установленную систему (координация F4/F5). */
#define KERNEL_LBA     2048u
#define KERNEL_SECS    768u
static uint32_t mbr_next_lba(void){
    uint32_t end=KERNEL_LBA+KERNEL_SECS;
    for(int i=0;i<4;i++){
        if(g_mbr.parts[i].type!=PART_EMPTY){
            uint32_t e=g_mbr.parts[i].lba_start+g_mbr.parts[i].lba_size;
            if(e>end) end=e;
        }
    }
    return end;
}

/* ═══════════════════════ Table entries ══════════════════════════ */
#define MAX_ENT 12
typedef struct {
    int      drv;
    int      slot;         /* MBR slot 0-3, or -1 = free space */
    bool     is_part;
    uint32_t total_mb;
    uint32_t free_mb;
    char     name[64];
    char     type_s[16];
} ent_t;

static ent_t  g_ent[MAX_ENT];
static int    g_ent_n = 0;

static void build_entries(void){
    g_ent_n=0;
    for(int d=0;d<ata_drive_count&&d<4;d++){
        uint8_t b[512]; mbr_t m;
        bool has_mbr=false;
        if(ata_read_sector_drive(d,0,b)){
            memcpy(&m,b,512);
            has_mbr=(m.sig[0]==0x55&&m.sig[1]==0xAA);
        }
        uint32_t disk_mb=sec2mb(ata_drives[d].sectors);
        uint32_t used_mb=0;
        if(has_mbr){
            for(int i=0;i<4&&g_ent_n<MAX_ENT;i++){
                if(m.parts[i].type!=PART_EMPTY&&m.parts[i].lba_size>0){
                    ent_t *e=&g_ent[g_ent_n++];
                    e->drv=d; e->slot=i; e->is_part=true;
                    e->total_mb=sec2mb(m.parts[i].lba_size);
                    e->free_mb=0;
                    int p=0; char ds[4],ps[4];
                    itoa((uint32_t)d,ds,10); itoa((uint32_t)(i+1),ps,10);
                    /* "  Диск N Раздел M" */
                    app_str(e->name,&p,"  \xd0\x94\xd0\xb8\xd1\x81\xd0\xba ");
                    app_str(e->name,&p,ds);
                    app_str(e->name,&p," \xd0\xa0\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb ");
                    app_str(e->name,&p,ps);
                    const char *tn=ptype(m.parts[i].type);
                    int tp=0; while(tn[tp]&&tp<15){e->type_s[tp]=tn[tp];tp++;} e->type_s[tp]=0;
                    used_mb+=e->total_mb;
                }
            }
        }
        /* Free space entry */
        if(disk_mb>used_mb+1&&g_ent_n<MAX_ENT){
            ent_t *e=&g_ent[g_ent_n++];
            e->drv=d; e->slot=-1; e->is_part=false;
            e->total_mb=disk_mb-used_mb; e->free_mb=disk_mb-used_mb;
            int p=0; char ds[4]; itoa((uint32_t)d,ds,10);
            /* "Незанятое пространство на диске N" */
            app_str(e->name,&p,"\xd0\x9d\xd0\xb5\xd0\xb7\xd0\xb0\xd0\xbd\xd1\x8f\xd1\x82\xd0\xbe\xd0\xb5 \xd0\xbf\xd1\x80\xd0\xbe\xd1\x81\xd1\x82\xd1\x80\xd0\xb0\xd0\xbd\xd1\x81\xd1\x82\xd0\xb2\xd0\xbe \xd0\xbd\xd0\xb0 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba\xd0\xb5 ");
            app_str(e->name,&p,ds);
            e->type_s[0]=0;
        }
    }
    iputs("build_entries: n="); iputu((uint32_t)g_ent_n); iputs("\n");
}

/* ═══════════════════════ Drawing helpers ════════════════════════ */
#define WIN_W 620
#define WIN_H 460
static int WX,WY;

static void init_coords(void){
    WX=((int)fb.width -WIN_W)/2;
    WY=((int)fb.height-WIN_H)/2;
}

static bool hit(int mx,int my,int x,int y,int w,int h){
    return mx>=x&&mx<x+w&&my>=y&&my<y+h;
}

static void fmt_size(uint32_t mb, char *buf){
    int p=0;
    if(mb>=1024){
        uint32_t gi=mb/1024, gf=(mb%1024)*10/1024;
        char tmp[12]; itoa(gi,tmp,10); app_str(buf,&p,tmp);
        buf[p++]='.'; tmp[0]=(char)('0'+gf); tmp[1]=0; app_str(buf,&p,tmp);
        app_str(buf,&p," \xd0\x93\xd0\x91");
    } else {
        char tmp[12]; itoa(mb,tmp,10); app_str(buf,&p,tmp);
        app_str(buf,&p," \xd0\x9c\xd0\x91");
    }
    buf[p]=0;
}

/* ── Draw window frame ─────────────────────────────────────────── */
static void draw_frame(void){
    fb_grad_v(0,0,(int)fb.width,(int)fb.height,
              rgb(0x2E,0x4F,0x9C),rgb(0x0B,0x21,0x55));
    fb_rect(WX+5,WY+5,WIN_W,WIN_H,rgb(0,0,0));
    fb_rect(WX,WY,WIN_W,WIN_H,rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(WX,WY,WIN_W,WIN_H,rgb(0x7A,0x9A,0xC8));
    fb_grad_v(WX,WY,WIN_W,34,rgb(0x4A,0x7C,0xC8),rgb(0x2C,0x5A,0xA8));
    fb_hline(WX,WY+34,WIN_W,rgb(0xA0,0xC0,0xE8));
    fb_hline(WX,WY+35,WIN_W,rgb(0xC8,0xD8,0xEE));
    /* Shield icon */
    fb_rect(WX+10,WY+9,12,16,rgb(0x20,0x60,0xC0));
    fb_rect_outline(WX+10,WY+9,12,16,rgb(0x80,0xB0,0xFF));
    fb_rect(WX+13,WY+12,6,10,rgb(0xFF,0xFF,0xFF));
    /* Title */
    text_draw_str_shadow(WX+30,WY+10,
        "\xd0\xa3\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xba\xd0\xb0 Egorov OS",
        COL_WHITE,rgb(0x10,0x30,0x80));
    /* Back button (<) */
    fb_grad_v(WX+WIN_W-56,WY+6,22,22,rgb(0x60,0x90,0xD0),rgb(0x30,0x60,0xB0));
    fb_rect_outline(WX+WIN_W-56,WY+6,22,22,rgb(0x18,0x40,0x90));
    text_draw_str(WX+WIN_W-50,WY+12,"<",COL_WHITE);
    /* Close button (X) */
    fb_grad_v(WX+WIN_W-30,WY+6,22,22,rgb(0xD0,0x50,0x50),rgb(0xA0,0x20,0x20));
    fb_rect_outline(WX+WIN_W-30,WY+6,22,22,rgb(0x80,0x10,0x10));
    text_draw_str(WX+WIN_W-24,WY+12,"x",COL_WHITE);
}

static void draw_bottom_bar(void){
    int by=WY+WIN_H-38;
    fb_rect(WX,by,WIN_W,38,rgb(0xF0,0xF0,0xF0));
    fb_hline(WX,by,WIN_W,rgb(0xC0,0xC0,0xC0));
}

static void tool_btn(int x,int y,int w,int h,
                     const char *lbl,bool hover,bool en){
    uint32_t t,b,brd;
    if(!en){t=b=rgb(0xEE,0xEE,0xEE);brd=rgb(0xC8,0xC8,0xC8);}
    else if(hover){t=rgb(0xE0,0xEE,0xFF);b=rgb(0xB8,0xD0,0xF0);brd=rgb(0x40,0x80,0xC0);}
    else{t=rgb(0xF5,0xF5,0xF5);b=rgb(0xE0,0xE0,0xE0);brd=rgb(0xA0,0xA0,0xA0);}
    fb_grad_v(x,y,w,h,t,b);
    fb_rect_outline(x,y,w,h,brd);
    uint32_t tc=en?COL_TEXT:rgb(0xA0,0xA0,0xA0);
    int tw=text_width(lbl);
    text_draw_str(x+(w-tw)/2,y+(h-10)/2,lbl,tc);
}

/* ═════════════════════════ WELCOME SCREEN ══════════════════════════ */
#define H_NONE    -1
#define H_ROW(i)  (100+(i))
#define H_REFRESH 0
#define H_DELETE  1
#define H_FORMAT  2
#define H_CREATE  3
#define H_LOAD    4
#define H_EXTEND  5
#define H_APPLY   6
#define H_CANCEL  7
#define H_NEXT    8
#define H_BACK    9
#define H_CLOSE   10
#define H_SPUP   11
#define H_SPDN   12
#define H_INST   13

static void draw_welcome(int hov){
    draw_frame();
    draw_bottom_bar();
    int cx=WX+24,cy=WY+44;
    /* "Где вы хотите установить Egorov OS?" */
    text_draw_str_scaled(cx,cy,
        "\xd0\x93\xd0\xb4\xd0\xb5 \xd0\xb2\xd1\x8b \xd1\x85\xd0\xbe\xd1\x82\xd0\xb8\xd1\x82\xd0\xb5 \xd1\x83\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xb8\xd1\x82\xd1\x8c Egorov OS?",
        rgb(0x00,0x38,0x84),1);
    cy+=22;
    fb_hline(WX+16,cy,WIN_W-32,rgb(0xD0,0xD0,0xD0)); cy+=10;

    if(ata_drive_count==0){
        fb_rect(cx,cy,400,44,rgb(0xFF,0xF0,0xF0));
        fb_rect_outline(cx,cy,400,44,rgb(0xC0,0x40,0x40));
        /* "ОШИБКА: жёсткий диск не обнаружен!" */
        text_draw_str(cx+8,cy+8,
            "\xd0\x9e\xd0\xa8\xd0\x98\xd0\x91\xd0\x9a\xd0\x90: \xd0\xb6\xd1\x91\xd1\x81\xd1\x82\xd0\xba\xd0\xb8\xd0\xb9 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba \xd0\xbd\xd0\xb5 \xd0\xbe\xd0\xb1\xd0\xbd\xd0\xb0\xd1\x80\xd1\x83\xd0\xb6\xd0\xb5\xd0\xbd!",
            rgb(0xC0,0x10,0x10));
        /* "Проверьте подключение диска." */
        text_draw_str(cx+8,cy+26,
            "\xd0\x9f\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb5\xd1\x80\xd1\x8c\xd1\x82\xd0\xb5 \xd0\xbf\xd0\xbe\xd0\xb4\xd0\xba\xd0\xbb\xd1\x8e\xd1\x87\xd0\xb5\xd0\xbd\xd0\xb8\xd0\xb5 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba\xd0\xb0.",
            rgb(0x80,0x00,0x00));
    } else {
        char ln[80]; int p;
        /* "Обнаружено дисков: N" */
        p=0; app_str(ln,&p,"\xd0\x9e\xd0\xb1\xd0\xbd\xd0\xb0\xd1\x80\xd1\x83\xd0\xb6\xd0\xb5\xd0\xbd\xd0\xbe \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba\xd0\xbe\xd0\xb2: ");
        char ns[4]; itoa((uint32_t)ata_drive_count,ns,10); app_str(ln,&p,ns);
        text_draw_str(cx,cy,ln,rgb(0x10,0x50,0xA0)); cy+=18;
        for(int i=0;i<ata_drive_count&&i<4;i++){
            p=0; char di[4]; itoa((uint32_t)(i+1),di,10);
            /* "  Диск N: <model> (NNN MB)" */
            app_str(ln,&p,"  \xd0\x94\xd0\xb8\xd1\x81\xd0\xba "); app_str(ln,&p,di); app_str(ln,&p,": ");
            for(int j=0;ata_drives[i].model[j]&&p<52;j++) ln[p++]=ata_drives[i].model[j];
            ln[p++]=' '; ln[p++]='(';
            char sz[12]; itoa(sec2mb(ata_drives[i].sectors),sz,10);
            app_str(ln,&p,sz); app_str(ln,&p," MB)");
            ln[p]=0;
            text_draw_str(cx,cy,ln,COL_TEXT); cy+=16;
        }
        cy+=6;
        fb_hline(WX+16,cy,WIN_W-32,rgb(0xD0,0xD0,0xD0)); cy+=10;
        /* "Нажмите Далее, чтобы выбрать диск и разделы." */
        text_draw_str(cx,cy,
            "\xd0\x9d\xd0\xb0\xd0\xb6\xd0\xbc\xd0\xb8\xd1\x82\xd0\xb5 \xd0\x94\xd0\xb0\xd0\xbb\xd0\xb5\xd0\xb5, \xd1\x87\xd1\x82\xd0\xbe\xd0\xb1\xd1\x8b \xd0\xb2\xd1\x8b\xd0\xb1\xd1\x80\xd0\xb0\xd1\x82\xd1\x8c \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba \xd0\xb8 \xd1\x80\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb\xd1\x8b.",
            COL_TEXT); cy+=16;
        /* "ВНИМАНИЕ: данные на выбранном разделе могут быть уничтожены!" */
        text_draw_str(cx,cy,
            "\xd0\x92\xd0\x9d\xd0\x98\xd0\x9c\xd0\x90\xd0\x9d\xd0\x98\xd0\x95: \xd0\xb4\xd0\xb0\xd0\xbd\xd0\xbd\xd1\x8b\xd0\xb5 \xd0\xbd\xd0\xb0 \xd0\xb2\xd1\x8b\xd0\xb1\xd1\x80\xd0\xb0\xd0\xbd\xd0\xbd\xd0\xbe\xd0\xbc \xd1\x80\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb\xd0\xb5 \xd0\xbc\xd0\xbe\xd0\xb3\xd1\x83\xd1\x82 \xd0\xb1\xd1\x8b\xd1\x82\xd1\x8c \xd1\x83\xd0\xbd\xd0\xb8\xd1\x87\xd1\x82\xd0\xbe\xd0\xb6\xd0\xb5\xd0\xbd\xd1\x8b!",
            rgb(0xC0,0x10,0x10));
    }
    int bby=WY+WIN_H-38;
    bool can=ata_drive_count>0;
    /* Далее */
    if(can) gui_button(WX+WIN_W-200,bby+7,90,24,
        "\xd0\x94\xd0\xb0\xd0\xbb\xd0\xb5\xd0\xb5",hov==H_NEXT,false);
    else {
        fb_grad_v(WX+WIN_W-200,bby+7,90,24,rgb(0xE8,0xE8,0xE8),rgb(0xD8,0xD8,0xD8));
        fb_rect_outline(WX+WIN_W-200,bby+7,90,24,rgb(0xC8,0xC8,0xC8));
        text_draw_str(WX+WIN_W-165,bby+16,
            "\xd0\x94\xd0\xb0\xd0\xbb\xd0\xb5\xd0\xb5",rgb(0xA0,0xA0,0xA0));
    }
    /* Отмена */
    gui_button(WX+WIN_W-100,bby+7,80,24,
        "\xd0\x9e\xd1\x82\xd0\xbc\xd0\xb5\xd0\xbd\xd0\xb0",hov==H_CANCEL,false);
}

/* ═════════════════════ PARTITION MANAGER ═══════════════════════ */
#define TBL_X  (WX+16)
#define TBL_W  (WIN_W-32)
#define TBL_HDR_Y (WY+72)
#define TBL_HDR_H 22
#define ROW_H  26
#define C0W  (TBL_W-210)
#define C1W  80
#define C2W  80
#define C3W  (TBL_W-C0W-C1W-C2W)
#define CX0  (TBL_X+2)
#define CX1  (TBL_X+C0W)
#define CX2  (TBL_X+C0W+C1W)
#define CX3  (TBL_X+C0W+C1W+C2W)

/* When creating: two toolbar rows (26+26) + gap (30) + slider panel (52) +
   bottom gap = 150px total.
   When NOT creating: two rows (26+26) + gap = 52 (compact). */
#define TB1_Y(cre) (WY+WIN_H-38-(cre?150:52))
#define TB2_Y(cre) (TB1_Y(cre)+26)
#define INP_Y(cre) (TB2_Y(cre)+30)
#define SLD_H      52    /* slider panel total height */

/* PM state */
static int   g_sel     = 0;
static bool  g_creating= false;
static char  g_szb[12]="100";
static int   g_szl     = 3;
static char  g_stat[80];
static uint32_t g_stcol;

static void draw_partman(int hov){
    draw_frame();
    draw_bottom_bar();

    /* "Где вы хотите установить Egorov OS?" */
    text_draw_str_scaled(WX+24,WY+42,
        "\xd0\x93\xd0\xb4\xd0\xb5 \xd0\xb2\xd1\x8b \xd1\x85\xd0\xbe\xd1\x82\xd0\xb8\xd1\x82\xd0\xb5 \xd1\x83\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xb8\xd1\x82\xd1\x8c Egorov OS?",
        rgb(0x00,0x38,0x84),1);

    /* Table header */
    fb_grad_v(TBL_X,TBL_HDR_Y,TBL_W,TBL_HDR_H,rgb(0xEB,0xEB,0xEB),rgb(0xD8,0xD8,0xD8));
    fb_rect_outline(TBL_X,TBL_HDR_Y,TBL_W,TBL_HDR_H,rgb(0xA0,0xA0,0xA0));
    /* "Файл" */
    text_draw_str(CX0,TBL_HDR_Y+6,"\xd0\xa4\xd0\xb0\xd0\xb9\xd0\xbb",rgb(0x30,0x30,0x30));
    /* "Общий раз..." */
    text_draw_str(CX1+4,TBL_HDR_Y+6,"\xd0\x9e\xd0\xb1\xd1\x89\xd0\xb8\xd0\xb9 \xd1\x80\xd0\xb0\xd0\xb7...",rgb(0x30,0x30,0x30));
    /* "Свободно" */
    text_draw_str(CX2+4,TBL_HDR_Y+6,"\xd0\xa1\xd0\xb2\xd0\xbe\xd0\xb1\xd0\xbe\xd0\xb4\xd0\xbd\xd0\xbe",rgb(0x30,0x30,0x30));
    /* "Тип" */
    text_draw_str(CX3+4,TBL_HDR_Y+6,"\xd0\xa2\xd0\xb8\xd0\xbf",rgb(0x30,0x30,0x30));
    fb_vline(CX1,TBL_HDR_Y,TBL_HDR_H,rgb(0xA0,0xA0,0xA0));
    fb_vline(CX2,TBL_HDR_Y,TBL_HDR_H,rgb(0xA0,0xA0,0xA0));
    fb_vline(CX3,TBL_HDR_Y,TBL_HDR_H,rgb(0xA0,0xA0,0xA0));

    /* Rows */
    bool cre=g_creating;
    int max_rows=cre?4:7;
    int tbl_body_h=0;
    int ry=TBL_HDR_Y+TBL_HDR_H;
    for(int i=0;i<g_ent_n&&i<max_rows;i++){
        ent_t *e=&g_ent[i];
        bool sel=(i==g_sel);
        bool rh=(hov==H_ROW(i));
        uint32_t bg,brd;
        if(sel){bg=rgb(0x33,0x99,0xFF);brd=rgb(0x00,0x66,0xCC);}
        else if(rh){bg=rgb(0xE0,0xF0,0xFF);brd=rgb(0x80,0xC0,0xFF);}
        else{bg=rgb(0xFF,0xFF,0xFF);brd=rgb(0xD8,0xD8,0xD8);}
        fb_rect(TBL_X,ry,TBL_W,ROW_H,bg);
        fb_rect_outline(TBL_X,ry,TBL_W,ROW_H,brd);
        uint32_t tc=sel?COL_WHITE:COL_TEXT;
        uint32_t dc=sel?rgb(0xC8,0xE4,0xFF):rgb(0x60,0x60,0x60);

        uint32_t ic=e->is_part?rgb(0x40,0x80,0xC0):rgb(0x70,0xA0,0xD0);
        fb_rect(CX0,ry+5,13,16,ic);
        fb_rect_outline(CX0,ry+5,13,16,sel?COL_WHITE:rgb(0x20,0x50,0x90));
        fb_hline(CX0+2,ry+8,9,sel?rgb(0xA0,0xC8,0xFF):COL_WHITE);
        fb_hline(CX0+2,ry+11,9,sel?rgb(0xA0,0xC8,0xFF):COL_WHITE);

        text_draw_str(CX0+17,ry+8,e->name,tc);

        char sb[24]; sb[0]=0;
        fmt_size(e->total_mb,sb); text_draw_str(CX1+4,ry+8,sb,tc);
        if(e->free_mb>0&&!e->is_part){
            sb[0]=0; fmt_size(e->free_mb,sb); text_draw_str(CX2+4,ry+8,sb,tc);
        }
        if(e->type_s[0]) text_draw_str(CX3+4,ry+8,e->type_s,dc);

        fb_vline(CX1,ry,ROW_H,rgb(0xC0,0xC0,0xC0));
        fb_vline(CX2,ry,ROW_H,rgb(0xC0,0xC0,0xC0));
        fb_vline(CX3,ry,ROW_H,rgb(0xC0,0xC0,0xC0));
        tbl_body_h+=ROW_H; ry+=ROW_H;
    }
    fb_rect_outline(TBL_X,TBL_HDR_Y,TBL_W,TBL_HDR_H+tbl_body_h,rgb(0x80,0x80,0x80));

    /* ── Toolbar ── */
    ent_t *se=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
    bool can_del = se&&se->is_part;
    bool can_fmt = se&&se->is_part;
    bool can_cre = se&&!se->is_part&&!g_creating;

    int tb1=TB1_Y(cre), tb2=TB2_Y(cre);
    int bh=22;
    int bx=TBL_X;

    /* Row 1: Обновить | Удалить | Форматировать | Создать */
    tool_btn(bx,tb1,80,bh,
        "\xe2\x86\xba \xd0\x9e\xd0\xb1\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xb8\xd1\x82\xd1\x8c",
        hov==H_REFRESH,true); bx+=84;
    tool_btn(bx,tb1,80,bh,
        "\xd0\xa3\xd0\xb4\xd0\xb0\xd0\xbb\xd0\xb8\xd1\x82\xd1\x8c",
        hov==H_DELETE,can_del); bx+=84;
    tool_btn(bx,tb1,110,bh,
        "\xd0\xa4\xd0\xbe\xd1\x80\xd0\xbc\xd0\xb0\xd1\x82\xd0\xb8\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb0\xd1\x82\xd1\x8c",
        hov==H_FORMAT,can_fmt); bx+=114;
    tool_btn(bx,tb1,80,bh,
        "\xd0\xa1\xd0\xbe\xd0\xb7\xd0\xb4\xd0\xb0\xd1\x82\xd1\x8c",
        hov==H_CREATE,can_cre);

    /* Row 2: Загрузить | Расширить */
    bx=TBL_X;
    tool_btn(bx,tb2,90,bh,
        "\xd0\x97\xd0\xb0\xd0\xb3\xd1\x80\xd1\x83\xd0\xb7\xd0\xb8\xd1\x82\xd1\x8c",
        hov==H_LOAD,false); bx+=94;
    tool_btn(bx,tb2,100,bh,
        "\xd0\xa0\xd0\xb0\xd1\x81\xd1\x88\xd0\xb8\xd1\x80\xd0\xb8\xd1\x82\xd1\x8c",
        hov==H_EXTEND,false);

    /* ── Визуальный слайдер создания раздела ── */
    if(g_creating){
        int iy=INP_Y(cre);
        /* Panel border */
        fb_rect_outline(TBL_X-2,iy-2,TBL_W+4,SLD_H+4,rgb(0x18,0x5A,0xCC));
        fb_rect_outline(TBL_X-3,iy-3,TBL_W+6,SLD_H+6,rgb(0x80,0xB4,0xFF));
        fb_rect(TBL_X-1,iy-1,TBL_W+2,SLD_H+2,rgb(0xF0,0xF6,0xFF));

        /* ── Row 1: label + slider track ── */
        int sly=iy+8;
        /* "Размер раздела:" */
        text_draw_str(TBL_X+4,sly,
            "\xd0\xa0\xd0\xb0\xd0\xb7\xd0\xbc\xd0\xb5\xd1\x80 \xd1\x80\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb\xd0\xb0:",COL_TEXT);
        int slx=TBL_X+114;
        int slw=TBL_W-254;  /* track width, leaves room for size label */
        int slth=14;        /* track height */

        /* Compute available space from the selected free-space entry */
        ent_t *fse=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
        uint32_t avail_mb=fse&&!fse->is_part?fse->free_mb:0;
        if(avail_mb<100)avail_mb=100;
        if(g_slider_mb>avail_mb)g_slider_mb=avail_mb;
        if(g_slider_mb<100)g_slider_mb=100;

        /* Track background (trough) */
        fb_grad_v(slx,sly,slw,slth,rgb(0xC8,0xD4,0xE8),rgb(0xA8,0xB8,0xD0));
        fb_rect_outline(slx,sly,slw,slth,rgb(0x60,0x80,0xB0));

        /* Filled portion — use 32-bit only (slw<=400, slider_mb<=avail, product fits) */
        int filled_w=0;
        if(avail_mb>0){
            uint32_t ratio=((uint32_t)g_slider_mb*(uint32_t)(slw-2))/avail_mb;
            filled_w=(int)ratio; if(filled_w>slw-2)filled_w=slw-2;
        }
        if(filled_w>0)
            fb_grad_h(slx+1,sly+1,filled_w,slth-2,rgb(0x28,0x84,0xE4),rgb(0x10,0x5A,0xC4));

        /* Slider handle */
        int hndx=slx+1+filled_w-5; if(hndx<slx)hndx=slx; if(hndx>slx+slw-11)hndx=slx+slw-11;
        fb_grad_v(hndx,sly,10,slth,rgb(0xA0,0xCC,0xFF),rgb(0x50,0x90,0xD8));
        fb_rect_outline(hndx,sly,10,slth,rgb(0x10,0x4A,0xA8));
        fb_vline(hndx+4,sly+2,slth-4,rgb(0xFF,0xFF,0xFF));
        fb_vline(hndx+5,sly+2,slth-4,rgb(0xFF,0xFF,0xFF));

        /* Size label to the right of track: "NNN МБ / NNN ГБ" */
        char sizelbl[24]; sizelbl[0]=0; fmt_size(g_slider_mb,sizelbl);
        int lx=slx+slw+6;
        text_draw_str(lx,sly+2,sizelbl,rgb(0x00,0x38,0xA4));

        /* Tick marks: quarters */
        for(int q=1;q<4;q++){
            int tx=slx+(slw*q)/4;
            fb_vline(tx,sly+slth,4,rgb(0x80,0x80,0x80));
        }
        /* Min / max labels */
        text_draw_str(slx,sly+slth+6,"100 \xd0\x9c\xd0\x91",rgb(0x80,0x80,0x80));
        char maxlbl[24]; maxlbl[0]=0; fmt_size(avail_mb,maxlbl);
        int mxw=text_width(maxlbl);
        text_draw_str(slx+slw-mxw,sly+slth+6,maxlbl,rgb(0x80,0x80,0x80));

        /* ── Row 2: buttons + hint ── */
        int aby=iy+SLD_H-22;
        /* Применить */
        gui_button(TBL_X+4,aby,86,20,
            "\xd0\x9f\xd1\x80\xd0\xb8\xd0\xbc\xd0\xb5\xd0\xbd\xd0\xb8\xd1\x82\xd1\x8c",
            hov==H_APPLY,false);
        /* Отмена */
        gui_button(TBL_X+94,aby,72,20,
            "\xd0\x9e\xd1\x82\xd0\xbc\xd0\xb5\xd0\xbd\xd0\xb0",
            hov==H_CANCEL,false);
        /* Hint "← → для точной настройки" */
        text_draw_str(TBL_X+174,aby+4,
            "\xe2\x86\x90 \xe2\x86\x92 \xd0\xb4\xd0\xbb\xd1\x8f \xd1\x82\xd0\xbe\xd1\x87\xd0\xbd\xd0\xbe\xd0\xb9 \xd0\xbd\xd0\xb0\xd1\x81\xd1\x82\xd1\x80\xd0\xbe\xd0\xb9\xd0\xba\xd0\xb8",
            rgb(0x60,0x60,0x60));
    }

    /* Status message */
    if(g_stat[0]){
        int sy=WY+WIN_H-38-14;
        fb_rect(TBL_X,sy,TBL_W,14,rgb(0xF8,0xF8,0xF8));
        text_draw_str(TBL_X+4,sy+2,g_stat,g_stcol);
    }

    /* Далее button — always enabled */
    int bby=WY+WIN_H-38;
    gui_button(WX+WIN_W-100,bby+7,80,24,
        "\xd0\x94\xd0\xb0\xd0\xbb\xd0\xb5\xd0\xb5",hov==H_NEXT,false);
}

/* ═════════════════════ CONFIRM SCREEN ════════════════════════ */
static void draw_confirm(int hov){
    draw_frame();
    draw_bottom_bar();
    int cx=WX+24,cy=WY+44;
    /* "Подтвердите параметры установки:" */
    text_draw_str(cx,cy,
        "\xd0\x9f\xd0\xbe\xd0\xb4\xd1\x82\xd0\xb2\xd0\xb5\xd1\x80\xd0\xb4\xd0\xb8\xd1\x82\xd0\xb5 \xd0\xbf\xd0\xb0\xd1\x80\xd0\xb0\xd0\xbc\xd0\xb5\xd1\x82\xd1\x80\xd1\x8b \xd1\x83\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xb2\xd0\xba\xd0\xb8:",COL_TEXT);
    cy+=22;
    char ln[80]; int p;
    /* "Диск: <model>" */
    p=0; app_str(ln,&p,"\xd0\x94\xd0\xb8\xd1\x81\xd0\xba: ");
    for(int i=0;ata_drives[g_drv].model[i]&&p<60;i++) ln[p++]=ata_drives[g_drv].model[i];
    ln[p]=0;
    text_draw_str(cx,cy,ln,rgb(0x10,0x50,0xA0)); cy+=18;
    /* "Загрузчик → MBR (сектор 0)" */
    text_draw_str(cx,cy,"\xd0\x97\xd0\xb0\xd0\xb3\xd1\x80\xd1\x83\xd0\xb7\xd1\x87\xd0\xb8\xd0\xba \xe2\x86\x92 MBR (\xd1\x81\xd0\xb5\xd0\xba\xd1\x82\xd0\xbe\xd1\x80 0)",COL_TEXT); cy+=14;
    /* "Ядро → LBA 2048+" */
    text_draw_str(cx,cy,"\xd0\xaf\xd0\xb4\xd1\x80\xd0\xbe \xe2\x86\x92 LBA 2048+",COL_TEXT); cy+=20;
    /* "Таблица разделов:" */
    text_draw_str(cx,cy,"\xd0\xa2\xd0\xb0\xd0\xb1\xd0\xbb\xd0\xb8\xd1\x86\xd0\xb0 \xd1\x80\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb\xd0\xbe\xd0\xb2:",COL_TEXT); cy+=16;
    bool any=false;
    for(int i=0;i<4;i++){
        mbr_part_t *pt=&g_mbr.parts[i];
        p=0; char sl[4]; itoa((uint32_t)(i+1),sl,10);
        /* "  Раздел N: " */
        app_str(ln,&p,"  \xd0\xa0\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb ");
        app_str(ln,&p,sl); app_str(ln,&p,": ");
        if(pt->type==PART_EMPTY&&pt->lba_size==0){
            /* "(пусто)" */
            app_str(ln,&p,"(\xd0\xbf\xd1\x83\xd1\x81\xd1\x82\xd0\xbe)");
        } else {
            app_str(ln,&p,ptype(pt->type)); app_str(ln,&p," ");
            char mb[12]; itoa(sec2mb(pt->lba_size),mb,10);
            app_str(ln,&p,mb); app_str(ln,&p," MB");
            any=true;
        }
        ln[p]=0;
        text_draw_str(cx,cy,ln,COL_TEXT); cy+=14;
    }
    if(!any){
        /* "  (разделы не созданы — установка без таблицы)" */
        text_draw_str(cx,cy,
            "  (\xd1\x80\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb\xd1\x8b \xd0\xbd\xd0\xb5 \xd1\x81\xd0\xbe\xd0\xb7\xd0\xb4\xd0\xb0\xd0\xbd\xd1\x8b \xe2\x80\x94 \xd1\x83\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xba\xd0\xb0 \xd0\xb1\xd0\xb5\xd0\xb7 \xd1\x82\xd0\xb0\xd0\xb1\xd0\xbb\xd0\xb8\xd1\x86\xd1\x8b)",
            rgb(0x70,0x70,0x70)); cy+=14;
    }
    cy+=8;
    fb_hline(WX+16,cy,WIN_W-32,rgb(0xD0,0xD0,0xD0)); cy+=10;
    /* "ВНИМАНИЕ: Загрузчик и ядро будут записаны на диск!" */
    text_draw_str(cx,cy,
        "\xd0\x92\xd0\x9d\xd0\x98\xd0\x9c\xd0\x90\xd0\x9d\xd0\x98\xd0\x95: \xd0\x97\xd0\xb0\xd0\xb3\xd1\x80\xd1\x83\xd0\xb7\xd1\x87\xd0\xb8\xd0\xba \xd0\xb8 \xd1\x8f\xd0\xb4\xd1\x80\xd0\xbe \xd0\xb1\xd1\x83\xd0\xb4\xd1\x83\xd1\x82 \xd0\xb7\xd0\xb0\xd0\xbf\xd0\xb8\xd1\x81\xd0\xb0\xd0\xbd\xd1\x8b \xd0\xbd\xd0\xb0 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba!",
        rgb(0xC0,0x10,0x10));
    int bby=WY+WIN_H-38;
    /* Установить */
    gui_button(WX+WIN_W-210,bby+7,100,24,
        "\xd0\xa3\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xb8\xd1\x82\xd1\x8c",hov==H_INST,false);
    /* Назад */
    gui_button(WX+WIN_W-100,bby+7,80,24,
        "\xd0\x9d\xd0\xb0\xd0\xb7\xd0\xb0\xd0\xb4",hov==H_BACK,false);
}

/* ═════════════════════ PROGRESS SCREEN ═══════════════════════ */
static void draw_progress(int pct,const char *msg){
    draw_frame();
    int cx=WX+24,cy=WY+60;
    /* "Идёт копирование файлов..." */
    text_draw_str(cx,cy,
        "\xd0\x98\xd0\xb4\xd1\x91\xd1\x82 \xd0\xba\xd0\xbe\xd0\xbf\xd0\xb8\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb0\xd0\xbd\xd0\xb8\xd0\xb5 \xd1\x84\xd0\xb0\xd0\xb9\xd0\xbb\xd0\xbe\xd0\xb2...",COL_TEXT);
    text_draw_str(cx,cy+18,msg,COL_TEXT);
    gui_progress(cx,cy+44,WIN_W-48,24,pct);
    char ps[8]; int pl=0;
    char nt[8]; itoa((uint32_t)pct,nt,10);
    app_str(ps,&pl,nt); ps[pl++]='%'; ps[pl]=0;
    text_draw_str(cx+(WIN_W-48)/2,cy+74,ps,COL_TEXT);
    /* "Подождите, не выключайте компьютер." */
    text_draw_str(cx,cy+100,
        "\xd0\x9f\xd0\xbe\xd0\xb4\xd0\xbe\xd0\xb6\xd0\xb4\xd0\xb8\xd1\x82\xd0\xb5, \xd0\xbd\xd0\xb5 \xd0\xb2\xd1\x8b\xd0\xba\xd0\xbb\xd1\x8e\xd1\x87\xd0\xb0\xd0\xb9\xd1\x82\xd0\xb5 \xd0\xba\xd0\xbe\xd0\xbc\xd0\xbf\xd1\x8c\xd1\x8e\xd1\x82\xd0\xb5\xd1\x80.",COL_TEXT);
    fb_present();
}

/* ════════════════════════ DONE SCREEN ════════════════════════ */
static void draw_done(int hov){
    draw_frame();
    draw_bottom_bar();
    int cx=WX+24,cy=WY+60;
    fb_rect(cx,cy,28,28,rgb(0x10,0x90,0x10));
    fb_rect_outline(cx,cy,28,28,rgb(0x08,0x60,0x08));
    text_draw_str_shadow(cx+7,cy+8,"OK",COL_WHITE,COL_BLACK);
    /* "Установка успешно завершена!" */
    text_draw_str(cx+40,cy+8,
        "\xd0\xa3\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xba\xd0\xb0 \xd1\x83\xd1\x81\xd0\xbf\xd0\xb5\xd1\x88\xd0\xbd\xd0\xbe \xd0\xb7\xd0\xb0\xd0\xb2\xd0\xb5\xd1\x80\xd1\x88\xd0\xb5\xd0\xbd\xd0\xb0!",
        rgb(0x10,0x80,0x10));
    cy+=44;
    /* "Egorov OS записана на жёсткий диск." */
    text_draw_str(cx,cy,"Egorov OS \xd0\xb7\xd0\xb0\xd0\xbf\xd0\xb8\xd1\x81\xd0\xb0\xd0\xbd\xd0\xb0 \xd0\xbd\xd0\xb0 \xd0\xb6\xd1\x91\xd1\x81\xd1\x82\xd0\xba\xd0\xb8\xd0\xb9 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba.",COL_TEXT); cy+=18;
    /* "Извлеките установочный носитель (CD/USB)" */
    text_draw_str(cx,cy,
        "\xd0\x98\xd0\xb7\xd0\xb2\xd0\xbb\xd0\xb5\xd0\xba\xd0\xb8\xd1\x82\xd0\xb5 \xd1\x83\xd1\x81\xd1\x82\xd0\xb0\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xbe\xd1\x87\xd0\xbd\xd1\x8b\xd0\xb9 \xd0\xbd\xd0\xbe\xd1\x81\xd0\xb8\xd1\x82\xd0\xb5\xd0\xbb\xd1\x8c (CD/USB)",COL_TEXT); cy+=14;
    /* "и нажмите Перезагрузить." */
    text_draw_str(cx,cy,
        "\xd0\xb8 \xd0\xbd\xd0\xb0\xd0\xb6\xd0\xbc\xd0\xb8\xd1\x82\xd0\xb5 \xd0\x9f\xd0\xb5\xd1\x80\xd0\xb5\xd0\xb7\xd0\xb0\xd0\xb3\xd1\x80\xd1\x83\xd0\xb7\xd0\xb8\xd1\x82\xd1\x8c.",COL_TEXT);
    int bby=WY+WIN_H-38;
    /* Перезагрузить */
    gui_button(WX+WIN_W-130,bby+7,110,24,
        "\xd0\x9f\xd0\xb5\xd1\x80\xd0\xb5\xd0\xb7\xd0\xb0\xd0\xb3\xd1\x80\xd1\x83\xd0\xb7\xd0\xb8\xd1\x82\xd1\x8c",hov==0,false);
}

/* ══════════════════════ INSTALLATION ════════════════════════ */
static void do_reboot(void){
    iputs("REBOOT\n");
    /* v1.7: полный ACPI-путь (RESET_REG → 0xCF9 → i8042 → triple fault)
       вместо одиночного 0x64←0xFE, который не работает на UEFI-платах
       без легаси-контроллера клавиатуры. */
    power_reboot();
}

static bool do_install(void){
    iputs("install: start, drv="); iputu((uint32_t)g_drv); iputs("\n");
    iputs("install: bl_start="); iputx((uint32_t)_bootloader_start); iputs("\n");
    iputs("install: bl_end=");   iputx((uint32_t)_bootloader_end);   iputs("\n");
    iputs("install: kn_start="); iputx((uint32_t)_kernel_image_start); iputs("\n");
    iputs("install: kn_end=");   iputx((uint32_t)_kernel_image_end);   iputs("\n");

    uint32_t blsz=(uint32_t)(_bootloader_end-_bootloader_start);
    uint32_t blsec=(blsz+511)/512;
    iputs("install: blsec="); iputu(blsec); iputs("\n");
    /* stage1 HDD-загрузчика читает только 63 сектора stage2: больше
     * писать бессмысленно - система не загрузится */
    if(blsec>63){ iputs("install: bootloader too big (>63 sectors)\n"); return false; }

    uint8_t buf[512];

    /* Write bootloader (MBR + stage2), preserving partition table in sector 0 */
    for(uint32_t s=0;s<blsec;s++){
        memset(buf,0,512);
        uint32_t rem=blsz-s*512, cp=rem<512?rem:512;
        memcpy(buf,_bootloader_start+s*512,cp);
        if(s==0){
            /* Preserve partition table from g_mbr */
            memcpy(buf+446,g_mbr.parts,64);
            buf[510]=0x55; buf[511]=0xAA;
        }
        if(!ata_write_sector_drive(g_drv,(uint32_t)s,buf)){
            iputs("bl write fail s="); iputu(s); iputs("\n");
            return false;
        }
    }
    iputs("install: bootloader written\n");

    /* Write kernel flat binary at LBA 2048 */
    uint32_t ksz=(uint32_t)(_kernel_image_end-_kernel_image_start);
    uint32_t ksec=(ksz+511)/512;
    iputs("install: ksec="); iputu(ksec); iputs("\n");
    /* ksec считается из РЕАЛЬНОГО размера embed-секции egorov.bin
     * (_kernel_image_start/_end из boot/embed.S) - цикл записи
     * покрывает всё ядро целиком, ничего не захардкожено. */

    /* Снапшоты первого/последнего сектора ядра для read-back verify */
    static uint8_t snap0[512], snapN[512];
    uint32_t last_sec = ksec ? ksec-1 : 0;

    for(uint32_t s=0;s<ksec;s++){
        memset(buf,0,512);
        uint32_t rem=ksz-s*512, cp=rem<512?rem:512;
        memcpy(buf,_kernel_image_start+s*512,cp);
        if(s==0)          memcpy(snap0,buf,512);
        if(s==last_sec)   memcpy(snapN,buf,512);
        if(!ata_write_sector_drive(g_drv,2048+s,buf)){
            iputs("kernel write fail s="); iputu(s); iputs("\n");
            return false;
        }
        if((s&0x1F)==0){
            uint32_t denom=ksec?ksec:1;
            uint32_t pct;
            if(ksec<47000000U) pct=10+(s*85U)/denom;
            else { uint32_t ss=s>>10,dd=denom>>10; if(!dd)dd=1; pct=10+(ss*85U)/dd; }
            if(pct>98)pct=98;
            char pmsg[48]; int pp=0;
            /* "Копирование сектора N/M" */
            app_str(pmsg,&pp,"\xd0\x9a\xd0\xbe\xd0\xbf\xd0\xb8\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb0\xd0\xbd\xd0\xb8\xd0\xb5 \xd1\x81\xd0\xb5\xd0\xba\xd1\x82\xd0\xbe\xd1\x80\xd0\xb0 ");
            char sn[12]; itoa(s,sn,10); app_str(pmsg,&pp,sn);
            app_str(pmsg,&pp,"/"); char st[12]; itoa(ksec,st,10); app_str(pmsg,&pp,st);
            pmsg[pp]=0;
            draw_progress((int)pct,pmsg);
        }
    }

    /* FLUSH CACHE (аудит F5 п.14): write-cache диска мог удержать
     * хвост записи - сбрасываем на носитель ДО проверки и перезагрузки.
     * ATA-путь флэшит после каждого сектора, AHCI - этой командой. */
    if(!ata_flush_cache()){
        iputs("install: FLUSH CACHE failed\n");
        return false;
    }
    iputs("install: flush cache OK\n");

    /* READ-BACK verify (аудит F5 п.13): перечитываем MBR и первый/
     * последний сектор ядра и сравниваем побайтово - ловим тихую
     * порчу данных (кабель, кэш, эмуляция). */
    {
        uint8_t rb[512];
        bool vok=true;
        if(!ata_read_sector_drive(g_drv,2048,rb)||memcmp(rb,snap0,512)!=0){
            iputs("verify: kernel FIRST sector mismatch\n"); vok=false;
        }
        if(ksec>1&&(!ata_read_sector_drive(g_drv,2048+last_sec,rb)
                    ||memcmp(rb,snapN,512)!=0)){
            iputs("verify: kernel LAST sector mismatch\n"); vok=false;
        }
        if(!ata_read_sector_drive(g_drv,0,rb)||rb[510]!=0x55||rb[511]!=0xAA){
            iputs("verify: MBR signature mismatch\n"); vok=false;
        } else if(memcmp(rb+446,g_mbr.parts,64)!=0){
            iputs("verify: MBR partition table mismatch\n"); vok=false;
        }
        if(!vok){ iputs("install: verify FAILED\n"); return false; }
        iputs("install: verify OK\n");
    }
    /* "Готово!" */
    draw_progress(100,"\xd0\x93\xd0\xbe\xd1\x82\xd0\xbe\xd0\xb2\xd0\xbe!");
    kdelay_ms(500);
    iputs("install: done\n");
    return true;
}

/* ══════════════════════ ACTION HELPERS ═══════════════════════ */
static void set_status(const char *s, uint32_t col){
    int p=0; g_stat[0]=0; app_str(g_stat,&p,s); g_stcol=col; g_stat[p]=0;
}
static void pm_refresh(void){
    mbr_load(); build_entries();
    if(g_sel>=g_ent_n) g_sel=g_ent_n>0?g_ent_n-1:0;
}
static void pm_delete(void){
    ent_t *e=&g_ent[g_sel];
    if(!e->is_part) return;
    mbr_ensure();
    memset(&g_mbr.parts[e->slot],0,sizeof(mbr_part_t));
    /* "Раздел удалён." */
    if(mbr_save()) set_status("\xd0\xa0\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb \xd1\x83\xd0\xb4\xd0\xb0\xd0\xbb\xd1\x91\xd0\xbd.",rgb(0x10,0x60,0x10));
    /* "Ошибка записи MBR." */
    else set_status("\xd0\x9e\xd1\x88\xd0\xb8\xd0\xb1\xd0\xba\xd0\xb0 \xd0\xb7\xd0\xb0\xd0\xbf\xd0\xb8\xd1\x81\xd0\xb8 MBR.",rgb(0xC0,0x10,0x10));
    mbr_load(); build_entries();
    if(g_sel>=g_ent_n) g_sel=g_ent_n>0?g_ent_n-1:0;
}
static void pm_format(void){
    ent_t *e=&g_ent[g_sel];
    if(!e->is_part) return;
    mbr_ensure();
    mbr_part_t *p=&g_mbr.parts[e->slot];
    uint8_t z[512]; memset(z,0,512);
    uint32_t clr=p->lba_size<8?p->lba_size:8;
    bool ok=true;
    for(uint32_t s=0;s<clr;s++) if(!ata_write_sector_drive(g_drv,p->lba_start+s,z)){ok=false;break;}
    /* "Раздел отформатирован." / "Ошибка форматирования." */
    if(ok) set_status("\xd0\xa0\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb \xd0\xbe\xd1\x82\xd1\x84\xd0\xbe\xd1\x80\xd0\xbc\xd0\xb0\xd1\x82\xd0\xb8\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb0\xd0\xbd.",rgb(0x10,0x60,0x10));
    else set_status("\xd0\x9e\xd1\x88\xd0\xb8\xd0\xb1\xd0\xba\xd0\xb0 \xd1\x84\xd0\xbe\xd1\x80\xd0\xbc\xd0\xb0\xd1\x82\xd0\xb8\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb0\xd0\xbd\xd0\xb8\xd1\x8f.",rgb(0xC0,0x10,0x10));
}

static void pm_apply_create(void){
    /* Use slider value directly */
    uint32_t mb=g_slider_mb;
    iputs("pm_apply_create: mb="); iputu(mb); iputs("\n");
    /* "Введите размер > 0 MB." */
    if(mb<1){set_status("\xd0\x92\xd0\xb2\xd0\xb5\xd0\xb4\xd0\xb8\xd1\x82\xd0\xb5 \xd1\x80\xd0\xb0\xd0\xb7\xd0\xbc\xd0\xb5\xd1\x80 > 0 MB.",rgb(0xC0,0x10,0x10));return;}
    mbr_ensure();
    int slot=mbr_free();
    iputs("pm_apply_create: slot="); iputu((uint32_t)(slot<0?99:slot)); iputs("\n");
    /* "Нет свободных слотов MBR (max 4)." */
    if(slot<0){set_status("\xd0\x9d\xd0\xb5\xd1\x82 \xd1\x81\xd0\xb2\xd0\xbe\xd0\xb1\xd0\xbe\xd0\xb4\xd0\xbd\xd1\x8b\xd1\x85 \xd1\x81\xd0\xbb\xd0\xbe\xd1\x82\xd0\xb2 MBR (max 4).",rgb(0xC0,0x10,0x10));return;}
    uint32_t lba_s=mbr_next_lba();
    uint32_t lba_z=mb2sec(mb);
    uint32_t disk_end=ata_drives[g_drv].sectors;
    if(lba_s+lba_z>disk_end) lba_z=disk_end>lba_s?disk_end-lba_s:0;
    iputs("pm_apply_create: lba_s="); iputu(lba_s);
    iputs(" lba_z="); iputu(lba_z); iputs("\n");
    /* "Недостаточно места (мин. 1 MB)." */
    if(lba_z<2048){set_status("\xd0\x9d\xd0\xb5\xd0\xb4\xd0\xbe\xd1\x81\xd1\x82\xd0\xb0\xd1\x82\xd0\xbe\xd1\x87\xd0\xbd\xd0\xbe \xd0\xbc\xd0\xb5\xd1\x81\xd1\x82\xd0\xb0 (\xd0\xbc\xd0\xb8\xd0\xbd. 1 MB).",rgb(0xC0,0x10,0x10));return;}
    mbr_part_t *np=&g_mbr.parts[slot];
    np->status=0; np->type=PART_EOS;
    np->lba_start=lba_s; np->lba_size=lba_z;
    lba_to_chs(lba_s,np->chs_start);
    lba_to_chs(lba_s+lba_z-1,np->chs_end);
    g_mbr.sig[0]=0x55; g_mbr.sig[1]=0xAA;
    if(mbr_save()){
        char msg[48]; int mp=0;
        /* "Раздел создан! NNN MB." */
        app_str(msg,&mp,"\xd0\xa0\xd0\xb0\xd0\xb7\xd0\xb4\xd0\xb5\xd0\xbb \xd1\x81\xd0\xbe\xd0\xb7\xd0\xb4\xd0\xb0\xd0\xbd! ");
        char mbs[12]; itoa(mb,mbs,10); app_str(msg,&mp,mbs); app_str(msg,&mp," MB.");
        msg[mp]=0;
        set_status(msg,rgb(0x10,0x60,0x10));
    }
    /* "Ошибка записи MBR." */
    else set_status("\xd0\x9e\xd1\x88\xd0\xb8\xd0\xb1\xd0\xba\xd0\xb0 \xd0\xb7\xd0\xb0\xd0\xbf\xd0\xb8\xd1\x81\xd0\xb8 MBR.",rgb(0xC0,0x10,0x10));
    mbr_load(); build_entries(); g_creating=false; g_sel=0;
}

/* slider_step: adjust g_slider_mb by delta MB, clamped to [100, avail_mb] */
static void slider_step(int delta){
    ent_t *fse=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
    uint32_t avail_mb=fse&&!fse->is_part?fse->free_mb:100;
    if(avail_mb<100)avail_mb=100;
    if(delta>0){
        g_slider_mb+=(uint32_t)delta;
        if(g_slider_mb>avail_mb)g_slider_mb=avail_mb;
    } else {
        uint32_t d=(uint32_t)(-delta);
        g_slider_mb=(g_slider_mb>100+d)?g_slider_mb-d:100;
    }
}

/* Compute hover ID for partition manager */
static int pm_hover(int mx,int my){
    /* Title buttons */
    if(hit(mx,my,WX+WIN_W-56,WY+6,22,22)) return H_BACK;
    if(hit(mx,my,WX+WIN_W-30,WY+6,22,22)) return H_CLOSE;

    /* Table rows — selectable regardless of g_creating */
    bool cre=g_creating;
    int max_rows=cre?4:7;
    int ry=TBL_HDR_Y+TBL_HDR_H;
    for(int i=0;i<g_ent_n&&i<max_rows;i++){
        if(hit(mx,my,TBL_X,ry,TBL_W,ROW_H)) return H_ROW(i);
        ry+=ROW_H;
    }

    /* Toolbar */
    int tb1=TB1_Y(cre),tb2=TB2_Y(cre),bh=22;
    ent_t *se=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
    bool can_del=se&&se->is_part, can_fmt=se&&se->is_part;
    bool can_cre=se&&!se->is_part&&!g_creating;
    int bx=TBL_X;
    if(hit(mx,my,bx,tb1,80,bh)) return H_REFRESH;
    bx+=84;
    if(can_del&&hit(mx,my,bx,tb1,80,bh)) return H_DELETE;
    bx+=84;
    if(can_fmt&&hit(mx,my,bx,tb1,110,bh)) return H_FORMAT;
    bx+=114;
    if(can_cre&&hit(mx,my,bx,tb1,80,bh)) return H_CREATE;
    bx=TBL_X;
    if(hit(mx,my,bx,tb2,90,bh)) return H_LOAD;
    bx+=94;
    if(hit(mx,my,bx,tb2,100,bh)) return H_EXTEND;

    /* Slider panel buttons */
    if(g_creating){
        int iy=INP_Y(cre);
        int aby=iy+SLD_H-22;
        if(hit(mx,my,TBL_X+4,aby,86,20)) return H_APPLY;
        if(hit(mx,my,TBL_X+94,aby,72,20)) return H_CANCEL;
    }
    /* Next button */
    int bby=WY+WIN_H-38;
    if(hit(mx,my,WX+WIN_W-100,bby+7,80,24)) return H_NEXT;
    return H_NONE;
}

/* ═══════════════════════ MAIN LOOP ═══════════════════════════ */
void installer_run(void){
    iputs("\n[installer_run]\n");
    /* kernel_main уже вызвал ata_init()+ahci_init().  Повторный
     * ata_init() СТИРАЛ найденные диски (memset ata_drives +
     * ata_use_ahci=false) - на машинах только с AHCI/SATA установщик
     * видел «жёсткого диска нет» и установка была невозможна. */
    if(ata_drive_count==0 && !ata_use_ahci){
        ata_init();
        if(ata_drive_count==0) ahci_init();
    }
    iputs("drives="); iputu((uint32_t)ata_drive_count); iputs("\n");
    for(int i=0;i<ata_drive_count;i++){
        iputs("  drv["); iputu((uint32_t)i); iputs("]: ");
        iputs(ata_drives[i].model);
        iputs(" secs="); iputu(ata_drives[i].sectors); iputs("\n");
    }
    ps2_init((int)fb.width,(int)fb.height);
    init_coords();

    if(ata_drive_count>0){
        g_drv=0;
        if(!mbr_load()){
            memset(&g_mbr,0,sizeof(g_mbr));
            g_mbr.sig[0]=0x55; g_mbr.sig[1]=0xAA; g_mbr_ok=true;
            iputs("mbr: initialized blank\n");
        }
        build_entries();
    }
    g_sel=0; g_creating=false; g_stat[0]=0; g_stcol=COL_TEXT;

    typedef enum {SCR_WELCOME,SCR_PARTMAN,SCR_CONFIRM,SCR_DONE,SCR_ERR} scr_t;
    scr_t scr=SCR_WELCOME;
    int hov=H_NONE;

    for(;;){
        /* локальная копия состояния: алиасинг на внутреннюю структуру
         * PS/2-драйвера (ps2_mouse()) при активной USB-мыши затирал её. */
        mouse_state_t ms;
        ps2_poll(&ms);
        /* пока рабочей мыши нет (или она на UHCI), mouse_hw следит за
         * портами xHCI/OHCI: hotplug мыши + перечисление USB-клавы. */
        if(!usb_mouse_active() && !mouse_hw_active()) mouse_hw_poll(0);
        /* разбор ВСЕХ накопившихся клавиш кадра (до 32): одна клавиша
         * за кадр теряла символы при быстром наборе (каждый кадр -
         * полная перерисовка 3 МБ) - как уже сделано в desktop.c. */
        uint32_t key=0; int kg=32;
        while(kg-- > 0 && ps2_key_pending()){
            uint32_t k2=ps2_poll_key();
            if(k2){ key=k2; break; }
        }
        int mx=ms.x,my=ms.y;
        bool clicked=ms.clicked_left;

        /* ─── WELCOME ─────────────────────────────────── */
        if(scr==SCR_WELCOME){
            bool can=ata_drive_count>0;
            int bby=WY+WIN_H-38;
            if(hit(mx,my,WX+WIN_W-56,WY+6,22,22)) hov=H_BACK;
            else if(hit(mx,my,WX+WIN_W-30,WY+6,22,22)) hov=H_CLOSE;
            else if(can&&hit(mx,my,WX+WIN_W-200,bby+7,90,24)) hov=H_NEXT;
            else if(hit(mx,my,WX+WIN_W-100,bby+7,80,24)) hov=H_CANCEL;
            else hov=H_NONE;

            if(clicked){
                if(hov==H_NEXT&&can){ scr=SCR_PARTMAN; g_stat[0]=0; }
                /* FIX: Cancel/Close/Back → reboot, NOT return (return causes triple fault) */
                else if(hov==H_CANCEL||hov==H_CLOSE||hov==H_BACK){ do_reboot(); }
            }
            if(key==KEY_ENTER&&can){ scr=SCR_PARTMAN; g_stat[0]=0; }
            if(key==KEY_ESC){ do_reboot(); }  /* ESC → reboot instead of return */

            fb_fill(rgb(0,0,50));
            draw_welcome(hov);
            gui_cursor(mx,my); fb_present();
        }

        /* ─── PARTITION MANAGER ──────────────────────── */
        else if(scr==SCR_PARTMAN){
            /* ALWAYS update hov every frame */
            hov=pm_hover(mx,my);

            /* Keyboard nav */
            if(!g_creating){
                if(key==KEY_UP&&g_sel>0){ g_sel--; g_stat[0]=0; }
                if(key==KEY_DOWN&&g_sel<g_ent_n-1){ g_sel++; g_stat[0]=0; }
            }

            /* Slider keyboard + drag when creating */
            if(g_creating){
                if(key==KEY_ESC){ g_creating=false; g_stat[0]=0; g_slider_drag=false; }
                else if(key==KEY_ENTER){ pm_apply_create(); }
                else if(key==KEY_LEFT){ slider_step(-100); }
                else if(key==KEY_RIGHT){ slider_step(100); }

                /* Slider drag via mouse — read track geometry */
                {
                    bool cre2=true;
                    int iy2=INP_Y(cre2);
                    int slx2=TBL_X+114;
                    int slw2=TBL_W-254;
                    int sly2=iy2+8;
                    int slth2=14;
                    /* Start drag on mouse-down inside track area */
                    if(ms.left&&hit(mx,my,slx2-8,sly2,slw2+16,slth2)){
                        g_slider_drag=true;
                    }
                    if(g_slider_drag){
                        if(!ms.left){ g_slider_drag=false; }
                        else {
                            ent_t *fse2=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
                            uint32_t avail2=fse2&&!fse2->is_part?fse2->free_mb:100;
                            if(avail2<100)avail2=100;
                            int rel=mx-slx2; if(rel<0)rel=0; if(rel>slw2)rel=slw2;
                            /* 32-bit only: rel<=slw2~330, avail2<=4M, product<4GB */
                            uint32_t newmb=((uint32_t)rel*avail2)/(uint32_t)slw2;
                            if(newmb<100)newmb=100;
                            if(newmb>avail2)newmb=avail2;
                            g_slider_mb=newmb;
                        }
                    }
                }
            } else {
                /* Keyboard shortcuts (only when NOT creating) */
                if(key=='C'||key=='c'){
                    ent_t *se2=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
                    if(se2&&!se2->is_part){
                        /* Init slider to 50% of available space */
                        uint32_t av=se2->free_mb;
                        g_slider_mb=av/2; if(g_slider_mb<100)g_slider_mb=100;
                        if(g_slider_mb>av)g_slider_mb=av;
                        g_creating=true; g_stat[0]=0; g_slider_drag=false;
                        iputs("create mode: opened\n");
                    }
                }
                if(key=='D'||key=='d'){
                    ent_t *se2=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
                    if(se2&&se2->is_part) pm_delete();
                }
                if(key=='F'||key=='f'){
                    ent_t *se2=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
                    if(se2&&se2->is_part) pm_format();
                }
                if(key==KEY_ENTER){ scr=SCR_CONFIRM; }
                if(key==KEY_ESC){ scr=SCR_WELCOME; }
            }

            /* Mouse clicks */
            if(clicked){
                if(hov==H_BACK){ scr=SCR_WELCOME; g_creating=false; }
                else if(hov==H_CLOSE){ do_reboot(); }
                else if(hov>=H_ROW(0)&&hov<H_ROW(MAX_ENT)){
                    int idx=hov-100;
                    if(idx>=0&&idx<g_ent_n){
                        g_sel=idx;
                        /* FIX: clicking a row does NOT cancel create mode */
                        /* User must press ESC or click Cancel to exit create mode */
                        g_stat[0]=0;
                    }
                }
                else if(hov==H_REFRESH){
                    pm_refresh();
                    /* "Обновлено." */
                    set_status("\xd0\x9e\xd0\xb1\xd0\xbd\xd0\xbe\xd0\xb2\xd0\xbb\xd0\xb5\xd0\xbd\xd0\xbe.",rgb(0x10,0x60,0x10));
                }
                else if(hov==H_DELETE){ pm_delete(); }
                else if(hov==H_FORMAT){ pm_format(); }
                else if(hov==H_CREATE){
                    ent_t *sce=(g_sel<g_ent_n)?&g_ent[g_sel]:0;
                    uint32_t av=sce&&!sce->is_part?sce->free_mb:1000;
                    g_slider_mb=av/2; if(g_slider_mb<100)g_slider_mb=100;
                    if(g_slider_mb>av)g_slider_mb=av;
                    g_creating=true; g_stat[0]=0; g_slider_drag=false;
                    iputs("CREATE button clicked\n");
                }
                else if(hov==H_APPLY){ pm_apply_create(); }
                else if(hov==H_CANCEL){ g_creating=false; g_stat[0]=0; g_slider_drag=false; }
                else if(hov==H_NEXT){ scr=SCR_CONFIRM; g_creating=false; }
            }

            fb_fill(rgb(0,0,50));
            draw_partman(hov);
            gui_cursor(mx,my); fb_present();
        }

        /* ─── CONFIRM ────────────────────────────────── */
        else if(scr==SCR_CONFIRM){
            int bby=WY+WIN_H-38;
            if(hit(mx,my,WX+WIN_W-56,WY+6,22,22)) hov=H_BACK;
            else if(hit(mx,my,WX+WIN_W-30,WY+6,22,22)) hov=H_CLOSE;
            else if(hit(mx,my,WX+WIN_W-210,bby+7,100,24)) hov=H_INST;
            else if(hit(mx,my,WX+WIN_W-100,bby+7,80,24)) hov=H_BACK;
            else hov=H_NONE;

            if(clicked){
                if(hov==H_INST){
                    iputs("INSTALL clicked\n");
                    /* "Подготовка..." */
                    draw_progress(0,"\xd0\x9f\xd0\xbe\xd0\xb4\xd0\xb3\xd0\xbe\xd1\x82\xd0\xbe\xd0\xb2\xd0\xba\xd0\xb0...");
                    bool ok=do_install();
                    scr=ok?SCR_DONE:SCR_ERR;
                } else if(hov==H_BACK){ scr=SCR_PARTMAN; }
                else if(hov==H_CLOSE){ do_reboot(); }
            }
            if(key==KEY_ENTER){
                iputs("INSTALL key\n");
                /* "Подготовка..." */
                draw_progress(0,"\xd0\x9f\xd0\xbe\xd0\xb4\xd0\xb3\xd0\xbe\xd1\x82\xd0\xbe\xd0\xb2\xd0\xba\xd0\xb0...");
                bool ok=do_install(); scr=ok?SCR_DONE:SCR_ERR;
            }
            if(key==KEY_ESC) scr=SCR_PARTMAN;

            fb_fill(rgb(0,0,50));
            draw_confirm(hov);
            gui_cursor(mx,my); fb_present();
        }

        /* ─── DONE ───────────────────────────────────── */
        else if(scr==SCR_DONE){
            int bby=WY+WIN_H-38;
            hov=(hit(mx,my,WX+WIN_W-130,bby+7,110,24))?0:H_NONE;
            /* Also check title buttons */
            if(hit(mx,my,WX+WIN_W-30,WY+6,22,22)) hov=H_CLOSE;
            if(clicked){
                if(hov==0||hov==H_CLOSE) do_reboot();
            }
            if(key==KEY_ENTER||key==KEY_ESC) do_reboot();

            fb_fill(rgb(0,0,50));
            draw_done(hov);
            gui_cursor(mx,my); fb_present();
        }

        /* ─── ERROR ──────────────────────────────────── */
        else if(scr==SCR_ERR){
            draw_frame();
            int cx=WX+24,cy=WY+80;
            /* Error box */
            fb_rect(cx,cy,WIN_W-48,60,rgb(0xFF,0xF0,0xF0));
            fb_rect_outline(cx,cy,WIN_W-48,60,rgb(0xC0,0x40,0x40));
            /* "ОШИБКА: не удалось записать данные на диск." */
            text_draw_str(cx+8,cy+10,
                "\xd0\x9e\xd0\xa8\xd0\x98\xd0\x91\xd0\x9a\xd0\x90: \xd0\xbd\xd0\xb5 \xd1\x83\xd0\xb4\xd0\xb0\xd0\xbb\xd0\xbe\xd1\x81\xd1\x8c \xd0\xb7\xd0\xb0\xd0\xbf\xd0\xb8\xd1\x81\xd0\xb0\xd1\x82\xd1\x8c \xd0\xb4\xd0\xb0\xd0\xbd\xd0\xbd\xd1\x8b\xd0\xb5 \xd0\xbd\xd0\xb0 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba.",
                rgb(0xC0,0x10,0x10));
            /* "Проверьте подключение диска." */
            text_draw_str(cx+8,cy+28,
                "\xd0\x9f\xd1\x80\xd0\xbe\xd0\xb2\xd0\xb5\xd1\x80\xd1\x8c\xd1\x82\xd0\xb5 \xd0\xbf\xd0\xbe\xd0\xb4\xd0\xba\xd0\xbb\xd1\x8e\xd1\x87\xd0\xb5\xd0\xbd\xd0\xb8\xd0\xb5 \xd0\xb4\xd0\xb8\xd1\x81\xd0\xba\xd0\xb0.",
                COL_TEXT);
            /* "Смотрите serial-лог для деталей." */
            text_draw_str(cx+8,cy+44,
                "\xd0\xa1\xd0\xbc\xd0\xbe\xd1\x82\xd1\x80\xd0\xb8\xd1\x82\xd0\xb5 serial-\xd0\xbb\xd0\xbe\xd0\xb3 \xd0\xb4\xd0\xbb\xd1\x8f \xd0\xb4\xd0\xb5\xd1\x82\xd0\xb0\xd0\xbb\xd0\xb5\xd0\xb9.",
                rgb(0x70,0x70,0x70));
            draw_bottom_bar();
            int bby=WY+WIN_H-38;
            /* Перезагрузить */
            hov=(hit(mx,my,WX+WIN_W-130,bby+7,110,24))?0:H_NONE;
            gui_button(WX+WIN_W-130,bby+7,110,24,
                "\xd0\x9f\xd0\xb5\xd1\x80\xd0\xb5\xd0\xb7\xd0\xb0\xd0\xb3\xd1\x80\xd1\x83\xd0\xb7\xd0\xb8\xd1\x82\xd1\x8c",hov==0,false);
            if(clicked&&hov==0) do_reboot();
            if(key==KEY_ENTER||key==KEY_ESC) do_reboot();
            gui_cursor(mx,my); fb_present();
        }

        kdelay_ms(5);
    }
}
