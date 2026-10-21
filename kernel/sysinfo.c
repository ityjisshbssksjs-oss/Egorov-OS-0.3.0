/* System Information app for Egorov OS.
   Shows hardware/software details detected at runtime. */
#include "sysinfo.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "ata.h"
#include "usb.h"
#include "rtc.h"
#include "string.h"

/* Helper: append string to buf at pos p, return new p. */
static int scat(char *buf, int p, const char *s) {
    while (*s) buf[p++] = *s++;
    buf[p] = 0;
    return p;
}

static int scat_num(char *buf, int p, uint32_t v) {
    char tmp[12];
    itoa(v, tmp, 10);
    return scat(buf, p, tmp);
}

static void draw_row(int x, int y, const char *label, const char *value, uint32_t vcol) {
    text_draw_str(x,       y, label, rgb(0x40, 0x40, 0x60));
    text_draw_str(x + 210, y, value, vcol);
}

void sysinfo_draw(int wx, int wy) {
    int bx = wx + 8;
    int by = wy + 32;  /* below title bar */
    int row = 18;
    int cw  = SYSINFO_WIN_W - 16;

    /* Header */
    fb_rect(bx, by, cw, 22, rgb(0x32, 0x6B, 0xCE));
    text_draw_str_shadow(bx + 6, by + 4, "Сведения о системе Egorov OS", COL_WHITE, COL_BLACK);
    by += 28;

    /* Separator */
    fb_hline(bx, by, cw, rgb(0xA0, 0xA0, 0x90));
    by += 6;

    /* OS */
    draw_row(bx, by, "Операционная система:", "Egorov OS 0.3.0", rgb(0x10, 0x70, 0x10));
    by += row;
    draw_row(bx, by, "Версия ядра:", "0.3.0 (монолитный i386)", COL_TEXT);
    by += row;
    draw_row(bx, by, "Загрузчик:", "GRUB Multiboot 1", COL_TEXT);
    by += row;
    draw_row(bx, by, "Архитектура:", "x86 32-bit (i386)", COL_TEXT);
    by += row + 4;

    fb_hline(bx, by, cw, rgb(0xC0, 0xC0, 0xC0));
    by += 6;

    /* Display */
    {
        char buf[64]; int p = 0;
        p = scat_num(buf, p, fb.width);  buf[p++] = 'x';
        p = scat_num(buf, p, fb.height); p = scat(buf, p, " @ ");
        p = scat_num(buf, p, fb.bpp);    p = scat(buf, p, " bpp (VBE)");
        draw_row(bx, by, "Видеорежим:", buf, COL_TEXT);
    }
    by += row;
    draw_row(bx, by, "Видеопамять (примерно):", "8 МБ (VBE, без драйверов)", COL_TEXT);
    by += row + 4;

    fb_hline(bx, by, cw, rgb(0xC0, 0xC0, 0xC0));
    by += 6;

    /* Disk (ATA) */
    if (ata_primary_master.present) {
        char buf[64]; int p = 0;
        for (int i = 0; ata_primary_master.model[i] && p < 40; i++)
            buf[p++] = ata_primary_master.model[i];
        buf[p] = 0;
        draw_row(bx, by, "Жёсткий диск:", buf, COL_TEXT);
        by += row;

        {
            char sz[32]; int q = 0;
            uint32_t mb = ata_primary_master.sectors / 2048;
            if (mb >= 1024) {
                q = scat_num(sz, q, mb / 1024); q = scat(sz, q, " ГБ (");
                q = scat_num(sz, q, mb);        q = scat(sz, q, " МБ)");
            } else {
                q = scat_num(sz, q, mb); q = scat(sz, q, " МБ");
            }
            sz[q] = 0;
            draw_row(bx, by, "Размер диска:", sz, COL_TEXT);
            by += row;
        }

        /* ATA channel info */
        {
            char ch[32]; int q = 0;
            const char *chan = (ata_primary_master.io_base == 0x1F0) ? "Первичный" : "Вторичный";
            const char *pos  = (ata_primary_master.drive_sel == 0xA0) ? "Master" : "Slave";
            q = scat(ch, q, chan); ch[q++] = ' '; q = scat(ch, q, pos);
            draw_row(bx, by, "Канал диска:", ch, COL_TEXT);
            by += row;
        }
    } else {
        draw_row(bx, by, "Жёсткий диск:", "Не обнаружен", rgb(0xC0, 0x10, 0x10));
        by += row;
    }
    by += 4;

    fb_hline(bx, by, cw, rgb(0xC0, 0xC0, 0xC0));
    by += 6;

    /* USB Flash Drive */
    if (usb_drive.present) {
        char usb_info[48]; int p = 0;
        for (int i = 0; usb_drive.vendor[i] && i < 8; i++)   usb_info[p++] = usb_drive.vendor[i];
        usb_info[p++] = ' ';
        for (int i = 0; usb_drive.product[i] && i < 16; i++) usb_info[p++] = usb_drive.product[i];
        usb_info[p] = 0;
        draw_row(bx, by, "USB Flash Drive:", usb_info, rgb(0x10, 0x50, 0xA0));
        by += row;

        char sz[24]; int q = 0;
        uint32_t mb = usb_drive.sectors / 2048;
        q = scat_num(sz, q, mb); q = scat(sz, q, " МБ"); sz[q] = 0;
        draw_row(bx, by, "Размер USB:", sz, COL_TEXT);
        by += row;
    } else {
        draw_row(bx, by, "USB Flash Drive:", "Не подключена", rgb(0x80, 0x80, 0x80));
        by += row;
    }
    by += 4;

    fb_hline(bx, by, cw, rgb(0xC0, 0xC0, 0xC0));
    by += 6;

    /* Clock */
    {
        rtc_time_t t;
        rtc_read(&t);
        char buf[32]; int p = 0;
        char tmp[8];
        itoa(t.hour, tmp, 10);
        if (t.hour < 10) buf[p++] = '0';
        p = scat(buf, p, tmp); buf[p++] = ':';
        itoa(t.min,  tmp, 10);
        if (t.min  < 10) buf[p++] = '0';
        p = scat(buf, p, tmp); buf[p++] = ':';
        itoa(t.sec,  tmp, 10);
        if (t.sec  < 10) buf[p++] = '0';
        p = scat(buf, p, tmp); buf[p] = 0;
        draw_row(bx, by, "Системное время:", buf, COL_TEXT);
        by += row;

        char date[32]; int d = 0;
        itoa(t.day,   tmp, 10); if (t.day   < 10) date[d++] = '0'; d = scat(date, d, tmp); date[d++] = '.';
        itoa(t.month, tmp, 10); if (t.month < 10) date[d++] = '0'; d = scat(date, d, tmp); date[d++] = '.';
        itoa(t.year,  tmp, 10); d = scat(date, d, tmp); date[d] = 0;
        draw_row(bx, by, "Дата:", date, COL_TEXT);
        by += row;
    }
    by += 4;

    fb_hline(bx, by, cw, rgb(0xC0, 0xC0, 0xC0));
    by += 8;

    /* Input */
    draw_row(bx, by, "Клавиатура:", "PS/2 (опрос)", COL_TEXT);
    by += row;
    draw_row(bx, by, "Мышь:", "PS/2 (опрос)", COL_TEXT);
    by += row + 4;

    /* Footer note */
    text_draw_str(bx, by,
        "Данные получены напрямую из аппаратуры через порты ввода-вывода.",
        rgb(0x80, 0x80, 0x80));
}
