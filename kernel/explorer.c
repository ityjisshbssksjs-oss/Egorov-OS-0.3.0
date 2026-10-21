/* File Explorer for Egorov OS.
 * Only functional when booted from HDD (g_hdd=true).
 * Shows a virtual filesystem with folders and EXE files.
 * USB flash drive is shown when usb_drive.present is true.
 */
#include "explorer.h"
#include "exe_runner.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "string.h"
#include "usb.h"

#define TREE_W      150
#define TOOLBAR_H   26
#define STATUSBAR_H 18
#define TITLE_H     28
#define BODY_Y_OFF  (TITLE_H + TOOLBAR_H)
#define BODY_H(wh)  ((wh) - TITLE_H - TOOLBAR_H - STATUSBAR_H)

/* EXE launch signals for desktop.c */
int  explorer_launched_exe = -1;
char explorer_exe_name[36] = {0};

/* ── Virtual filesystem ── */
typedef struct {
    const char *name;
    bool is_dir;
    uint32_t size_kb;
} vfs_entry_t;

/* Root: Мой компьютер */
static const vfs_entry_t dir_root[] = {
    { "Локальный диск (C:)", true,  0 },
    { "Корзина",             true,  0 },
    { "USB Flash Drive (D:)",true,  0 },  /* shown only when usb_drive.present */
};
#define DIR_ROOT_BASE_N 2

/* C:\ */
static const vfs_entry_t dir_c[] = {
    { "EgorovOS",      true,  0   },
    { "Программы",     true,  0   },
    { "Пользователи",  true,  0   },
    { "Документы",     true,  0   },
};
#define DIR_C_N 4

/* C:\EgorovOS */
static const vfs_entry_t dir_egos[] = {
    { "kernel.elf",   false, 156 },
    { "boot.bin",     false, 4   },
    { "grub.cfg",     false, 1   },
    { "README.txt",   false, 8   },
    { "drivers",      true,  0   },
    { "system",       true,  0   },
};
#define DIR_EGOS_N 6

/* C:\Программы — folders first, then EXE shortcuts.
 * EXE indices:  4=Калькулятор, 5=Часы, 6=Рисование, 7=Блокнот, 8=Змейка
 * These map to EXE_TARGET_CALC..EXE_TARGET_SNAKE (exe_runner.h, values 0-4). */
static const vfs_entry_t dir_prog[] = {
    { "Калькулятор",     true,  0  },   /* idx 0 */
    { "Часы",            true,  0  },   /* idx 1 */
    { "Рисование",       true,  0  },   /* idx 2 */
    { "Блокнот",         true,  0  },   /* idx 3 */
    { "Калькулятор.exe", false, 38 },   /* idx 4 → EXE_TARGET_CALC    */
    { "Часы.exe",        false, 22 },   /* idx 5 → EXE_TARGET_CLOCK   */
    { "Рисование.exe",   false, 54 },   /* idx 6 → EXE_TARGET_PAINT   */
    { "Блокнот.exe",     false, 31 },   /* idx 7 → EXE_TARGET_NOTEPAD */
    { "Змейка.exe",      false, 19 },   /* idx 8 → EXE_TARGET_SNAKE   */
};
#define DIR_PROG_N 9
#define DIR_PROG_EXE_START 4   /* first .exe entry index in dir_prog */

/* C:\Пользователи */
static const vfs_entry_t dir_users[] = {
    { "Пользователь",  true, 0 },
    { "Гость",         true, 0 },
};
#define DIR_USERS_N 2

/* C:\Документы */
static const vfs_entry_t dir_docs[] = {
    { "Добро пожаловать.txt", false, 2 },
    { "Справка.txt",          false, 5 },
};
#define DIR_DOCS_N 2

/* Корзина */
static const vfs_entry_t dir_trash[] = { /* empty */ };
#define DIR_TRASH_N 0

/* USB Flash Drive — virtual entries (drive is raw, no FS driver) */
static const vfs_entry_t dir_usb[] = { /* shown via draw_usb_panel */ };
#define DIR_USB_N 0

typedef enum {
    LOC_ROOT = 0,
    LOC_C,
    LOC_EGOS,
    LOC_PROG,
    LOC_USERS,
    LOC_DOCS,
    LOC_TRASH,
    LOC_USB,
    LOC_COUNT
} loc_t;

static const char *loc_names[LOC_COUNT] = {
    "Мой компьютер",
    "Локальный диск (C:)",
    "C:\\EgorovOS",
    "C:\\Программы",
    "C:\\Пользователи",
    "C:\\Документы",
    "Корзина",
    "USB Flash Drive (D:)",
};

static const vfs_entry_t *loc_entries[LOC_COUNT] = {
    dir_root, dir_c, dir_egos, dir_prog, dir_users, dir_docs, dir_trash, dir_usb,
};
static const int loc_base_counts[LOC_COUNT] = {
    DIR_ROOT_BASE_N, DIR_C_N, DIR_EGOS_N, DIR_PROG_N,
    DIR_USERS_N, DIR_DOCS_N, DIR_TRASH_N, DIR_USB_N,
};

static int root_count(void) {
    return usb_drive.present ? DIR_ROOT_BASE_N + 1 : DIR_ROOT_BASE_N;
}
static int loc_count(loc_t l) {
    if (l == LOC_ROOT) return root_count();
    return loc_base_counts[l];
}

/* ── State ── */
static bool   g_hdd = false;
static loc_t  g_loc = LOC_ROOT;
static loc_t  g_history[8];
static int    g_hist_pos = 0;
static int    g_scroll   = 0;
static int    g_selected = -1;

static bool tree_c_expanded = false;

static void nav_to(loc_t loc) {
    if (g_hist_pos < 7) { g_history[g_hist_pos++] = g_loc; }
    g_loc = loc; g_scroll = 0; g_selected = -1;
}

void explorer_init(bool booted_from_hdd) {
    g_hdd = booted_from_hdd;
    g_loc = LOC_ROOT; g_scroll = 0; g_selected = -1;
    g_hist_pos = 0;
    explorer_launched_exe = -1;
    explorer_exe_name[0] = 0;
}

void explorer_open_trash(void) {
    g_loc = LOC_TRASH; g_scroll = 0; g_selected = -1;
}

/* ── Icons ── */

static void draw_folder_icon(int x, int y, uint32_t col) {
    fb_rect(x, y+2, 6, 2, col);
    fb_rect(x, y+4, 10, 6, col);
}

static void draw_file_icon(int x, int y) {
    fb_rect(x, y, 8, 10, rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(x, y, 8, 10, rgb(0x80,0x80,0x80));
    fb_pixel(x+5, y,   rgb(0x80,0x80,0x80));
    fb_pixel(x+6, y+1, rgb(0x80,0x80,0x80));
    fb_pixel(x+7, y+2, rgb(0x80,0x80,0x80));
}

/* EXE icon: file with a blue application badge */
static void draw_exe_icon(int x, int y) {
    fb_rect(x, y, 8, 10, rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(x, y, 8, 10, rgb(0x60,0x60,0xA0));
    fb_pixel(x+5, y,   rgb(0x60,0x60,0xA0));
    fb_pixel(x+6, y+1, rgb(0x60,0x60,0xA0));
    fb_pixel(x+7, y+2, rgb(0x60,0x60,0xA0));
    /* Blue badge bottom-right */
    fb_rect(x+4, y+6, 4, 4, rgb(0x10,0x70,0xD0));
}

static void draw_usb_icon(int x, int y, uint32_t col) {
    fb_rect(x, y+2, 10, 8, col);
    fb_rect_outline(x, y+2, 10, 8, rgb(0x40,0x40,0x40));
    fb_rect(x+2, y, 6, 3, rgb(0xC0,0xC0,0xC0));
    fb_pixel(x+3, y+5, rgb(0xFF,0xFF,0xFF));
    fb_pixel(x+6, y+5, rgb(0xFF,0xFF,0xFF));
}

/* Check whether a vfs_entry has a .exe extension */
static bool is_exe(const vfs_entry_t *e) {
    if (e->is_dir) return false;
    const char *n = e->name;
    int len = 0;
    while (n[len]) len++;
    if (len < 4) return false;
    return (n[len-4]=='.' && (n[len-3]=='e'||n[len-3]=='E')
                          && (n[len-2]=='x'||n[len-2]=='X')
                          && (n[len-1]=='e'||n[len-1]=='E'));
}

/* Draw one row in the file list panel */
static void draw_entry_row(int lx, int ry, int lw, int idx,
                           const vfs_entry_t *e, bool selected)
{
    if (selected) fb_rect(lx, ry, lw, 16, rgb(0x32,0x6B,0xCE));

    if (e->is_dir) {
        draw_folder_icon(lx+2, ry+2, selected ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
    } else if (is_exe(e)) {
        draw_exe_icon(lx+2, ry+3);
    } else {
        draw_file_icon(lx+2, ry+2);
    }

    uint32_t tc = selected ? COL_WHITE : COL_TEXT;
    text_draw_str(lx+16, ry+2, e->name, tc);

    if (!e->is_dir) {
        char sz[20]; char num[12]; int p = 0;
        itoa(e->size_kb, num, 10);
        for (int i = 0; num[i]; i++) sz[p++] = num[i];
        const char *s = " КБ"; for (int i = 0; s[i]; i++) sz[p++] = s[i];
        sz[p] = 0;
        text_draw_str(lx + lw - 60, ry+2, sz, tc);
    } else {
        text_draw_str(lx + lw - 60, ry+2, "Папка", tc);
    }
    (void)idx;
}

/* Draw tree navigation panel */
static void draw_tree(int wx, int wy, int ww, int wh) {
    int tx = wx+2;
    int ty = wy + BODY_Y_OFF;
    int th = BODY_H(wh);
    fb_rect(tx, ty, TREE_W, th, rgb(0xF0,0xF0,0xF0));
    fb_rect_outline(tx, ty, TREE_W, th, rgb(0xC0,0xC0,0xC0));
    (void)ww;

    int cy = ty + 4;

    bool root_sel = (g_loc == LOC_ROOT);
    if (root_sel) fb_rect(tx+2, cy-1, TREE_W-4, 16, rgb(0x32,0x6B,0xCE));
    draw_folder_icon(tx+4, cy, root_sel ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
    text_draw_str(tx+18, cy, "Мой компьютер", root_sel ? COL_WHITE : COL_TEXT);
    cy += 18;

    bool c_sel = (g_loc == LOC_C || g_loc == LOC_EGOS || g_loc == LOC_PROG ||
                  g_loc == LOC_USERS || g_loc == LOC_DOCS);
    if (c_sel) fb_rect(tx+12, cy-1, TREE_W-14, 16, rgb(0x32,0x6B,0xCE));
    text_draw_str(tx+14, cy, tree_c_expanded ? "[-]" : "[+]", rgb(0x60,0x60,0x60));
    draw_folder_icon(tx+30, cy, c_sel ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
    text_draw_str(tx+44, cy, "Диск C:", c_sel ? COL_WHITE : COL_TEXT);
    cy += 18;

    if (tree_c_expanded) {
        const char *subdirs[] = { "EgorovOS", "Программы", "Пользователи", "Документы" };
        loc_t sublocs[] = { LOC_EGOS, LOC_PROG, LOC_USERS, LOC_DOCS };
        for (int i = 0; i < 4; i++) {
            bool ss = (g_loc == sublocs[i]);
            if (ss) fb_rect(tx+24, cy-1, TREE_W-26, 16, rgb(0x32,0x6B,0xCE));
            draw_folder_icon(tx+26, cy, ss ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
            text_draw_str(tx+40, cy, subdirs[i], ss ? COL_WHITE : COL_TEXT);
            cy += 16;
        }
    }

    bool tr_sel = (g_loc == LOC_TRASH);
    if (tr_sel) fb_rect(tx+2, cy-1, TREE_W-4, 16, rgb(0x32,0x6B,0xCE));
    draw_folder_icon(tx+4, cy, tr_sel ? rgb(0xFF,0xD0,0x50) : rgb(0xC0,0xC0,0xC8));
    text_draw_str(tx+18, cy, "Корзина", tr_sel ? COL_WHITE : COL_TEXT);
    cy += 18;

    if (usb_drive.present) {
        bool usb_sel = (g_loc == LOC_USB);
        if (usb_sel) fb_rect(tx+2, cy-1, TREE_W-4, 16, rgb(0x32,0x6B,0xCE));
        draw_usb_icon(tx+4, cy, usb_sel ? rgb(0x80,0xD0,0xFF) : rgb(0x40,0xA0,0xE0));
        text_draw_str(tx+18, cy, "Диск D: (USB)", usb_sel ? COL_WHITE : COL_TEXT);
    }
}

static void draw_toolbar(int wx, int wy, int ww) {
    int tx = wx+2;
    int ty = wy + TITLE_H;
    fb_grad_v(tx, ty, ww-4, TOOLBAR_H, rgb(0xF0,0xEE,0xE0), rgb(0xD8,0xD4,0xC4));
    fb_hline(tx, ty+TOOLBAR_H-1, ww-4, rgb(0xA0,0xA0,0x90));

    gui_button(tx+2, ty+2, 36, TOOLBAR_H-4, "< Назад", g_hist_pos > 0, false);

    int abx = tx + 42;
    int abw = ww - 50;
    fb_rect(abx, ty+4, abw, TOOLBAR_H-8, rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(abx, ty+4, abw, TOOLBAR_H-8, rgb(0x80,0x80,0x80));
    const char *addr = loc_names[g_loc];
    text_draw_str(abx+4, ty+6, addr, COL_TEXT);
}

static void draw_statusbar(int wx, int wy, int ww, int wh) {
    int ty = wy + wh - STATUSBAR_H;
    fb_rect(wx+2, ty, ww-4, STATUSBAR_H, rgb(0xD4,0xD0,0xC8));
    fb_hline(wx+2, ty, ww-4, rgb(0x80,0x80,0x80));

    char buf[80];
    int p = 0;

    if (g_loc == LOC_USB) {
        const char *s = "USB: ";
        for (int i = 0; s[i]; i++) buf[p++] = s[i];
        char nm[12]; itoa(usb_drive.sectors / 2048, nm, 10);
        for (int i = 0; nm[i]; i++) buf[p++] = nm[i];
        s = " MB  Vendor: "; for (int i = 0; s[i]; i++) buf[p++] = s[i];
        for (int i = 0; usb_drive.vendor[i] && i < 8; i++) buf[p++] = usb_drive.vendor[i];
        s = " "; buf[p++] = s[0];
        for (int i = 0; usb_drive.product[i] && i < 16; i++) buf[p++] = usb_drive.product[i];
    } else {
        int n = loc_count(g_loc);
        if (n == 0) {
            const char *s = "Папка пуста";
            for (int i = 0; s[i]; i++) buf[p++] = s[i];
        } else {
            const char *s = "Объектов: "; for (int i = 0; s[i]; i++) buf[p++] = s[i];
            char nm[8]; itoa(n, nm, 10);
            for (int i = 0; nm[i]; i++) buf[p++] = nm[i];

            /* Hint for EXE files */
            if (g_loc == LOC_PROG) {
                s = "   (дважды щёлкните .exe для запуска)";
                for (int i = 0; s[i]; i++) buf[p++] = s[i];
            }
        }
    }
    buf[p] = 0;
    text_draw_str(wx+6, ty+2, buf, COL_TEXT);
}

static void draw_usb_panel(int lx, int ly, int lw, int lh) {
    fb_rect(lx, ly, lw, lh, rgb(0xFF,0xFF,0xFF));

    if (!usb_drive.present) {
        text_draw_str(lx+10, ly+20, "USB Flash Drive не подключена.", rgb(0xC0,0x10,0x10));
        return;
    }

    fb_rect(lx+10, ly+10, 30, 20, rgb(0x40,0xA0,0xE0));
    fb_rect_outline(lx+10, ly+10, 30, 20, rgb(0x20,0x70,0xB0));
    fb_rect(lx+16, ly+7, 18, 6, rgb(0xC0,0xC0,0xC0));
    fb_rect_outline(lx+16, ly+7, 18, 6, rgb(0x80,0x80,0x80));
    text_draw_str_shadow(lx+15, ly+14, "USB", rgb(0xFF,0xFF,0xFF), rgb(0,0,0));

    text_draw_str(lx+50, ly+12, "USB Flash Drive (D:)", rgb(0x10,0x10,0x80));

    char info[64]; int p = 0;
    for (int i = 0; usb_drive.vendor[i] && i < 8; i++)  info[p++] = usb_drive.vendor[i];
    info[p++] = ' ';
    for (int i = 0; usb_drive.product[i] && i < 16; i++) info[p++] = usb_drive.product[i];
    info[p] = 0;
    text_draw_str(lx+50, ly+28, info, COL_TEXT);

    char cap[40]; int q = 0;
    uint32_t mb = usb_drive.sectors / 2048;
    char nm[12]; itoa(mb, nm, 10);
    for (int i = 0; nm[i]; i++) cap[q++] = nm[i];
    const char *s = " MB  ("; for (int i = 0; s[i]; i++) cap[q++] = s[i];
    itoa(usb_drive.sectors, nm, 10);
    for (int i = 0; nm[i]; i++) cap[q++] = nm[i];
    s = " секторов)"; for (int i = 0; s[i]; i++) cap[q++] = s[i];
    cap[q] = 0;
    text_draw_str(lx+50, ly+44, cap, COL_TEXT);

    fb_hline(lx+4, ly+64, lw-8, rgb(0xC0,0xC0,0xC0));
    text_draw_str(lx+10, ly+72, "Файловая система: RAW (нет драйвера FAT)", rgb(0x80,0x40,0x10));
    text_draw_str(lx+10, ly+90, "Флешка определена через USB UHCI (PCI).", COL_TEXT);
    text_draw_str(lx+10, ly+108,"Для работы с файлами требуется FAT-драйвер.", COL_TEXT);

    (void)lh;
}

void explorer_draw(int wx, int wy, int mx, int my, bool left_down) {
    (void)mx; (void)my; (void)left_down;

    int ww = EXPLORER_WIN_W;
    int wh = EXPLORER_WIN_H;

    fb_rect(wx+2, wy+TITLE_H+TOOLBAR_H, ww-4, BODY_H(wh), rgb(0xFF,0xFF,0xFF));

    if (!g_hdd && g_loc != LOC_TRASH && g_loc != LOC_ROOT && g_loc != LOC_USB) {
        text_draw_str(wx+60, wy+BODY_Y_OFF+40,
            "Проводник недоступен в Live-режиме.", rgb(0xC0,0x10,0x10));
        text_draw_str(wx+60, wy+BODY_Y_OFF+60,
            "Установите Egorov OS на жёсткий диск.", COL_TEXT);
        draw_toolbar(wx, wy, ww);
        draw_statusbar(wx, wy, ww, wh);
        return;
    }

    draw_toolbar(wx, wy, ww);
    draw_tree(wx, wy, ww, wh);
    draw_statusbar(wx, wy, ww, wh);

    int lx = wx + 2 + TREE_W + 2;
    int ly = wy + BODY_Y_OFF;
    int lw = ww - 4 - TREE_W - 2;
    int lh = BODY_H(wh);

    if (g_loc == LOC_USB) {
        draw_usb_panel(lx, ly, lw, lh);
        return;
    }

    /* Column headers */
    fb_rect(lx, ly, lw, 16, rgb(0xD8,0xD4,0xC4));
    fb_hline(lx, ly+15, lw, rgb(0x80,0x80,0x80));
    text_draw_str(lx+2,   ly+2, "Имя",    COL_TEXT);
    text_draw_str(lx+lw-60, ly+2, "Размер", COL_TEXT);

    int cnt = loc_count(g_loc);
    const vfs_entry_t *entries = loc_entries[g_loc];
    int max_visible = (lh - 16) / 16;
    int start = g_scroll;
    if (start > cnt - max_visible) start = cnt - max_visible;
    if (start < 0) start = 0;

    if (cnt == 0) {
        text_draw_str(lx+10, ly+30, "Эта папка пуста.", rgb(0x80,0x80,0x80));
    }

    for (int i = start; i < cnt && (i - start) < max_visible; i++) {
        int ry = ly + 16 + (i - start) * 16;
        draw_entry_row(lx, ry, lw, i, &entries[i], g_selected == i);
    }
}

/* Hit tests */
static bool hit_back(int wx, int wy, int mx, int my) {
    int tx = wx+2, ty = wy + TITLE_H;
    return mx >= tx+2 && mx < tx+2+36 && my >= ty+2 && my < ty+2+TOOLBAR_H-4;
}

static int hit_tree(int wx, int wy, int wh, int mx, int my) {
    int tx = wx+2, ty = wy + BODY_Y_OFF;
    if (mx < tx || mx >= tx+TREE_W) return -1;
    if (my < ty || my >= ty + BODY_H(wh)) return -1;
    int cy = ty + 4;

    if (my >= cy-1 && my < cy+15) return LOC_ROOT;
    cy += 18;

    if (my >= cy-1 && my < cy+15) {
        if (mx >= tx+14 && mx < tx+28) return -2;
        return LOC_C;
    }
    cy += 18;

    if (tree_c_expanded) {
        loc_t sublocs[] = { LOC_EGOS, LOC_PROG, LOC_USERS, LOC_DOCS };
        for (int i = 0; i < 4; i++) {
            if (my >= cy-1 && my < cy+15) return (int)sublocs[i];
            cy += 16;
        }
    }

    if (my >= cy-1 && my < cy+15) return LOC_TRASH;
    cy += 18;

    if (usb_drive.present && my >= cy-1 && my < cy+15) return LOC_USB;

    return -1;
}

static int hit_entry(int wx, int wy, int wh, int mx, int my) {
    int lx = wx + 2 + TREE_W + 2;
    int ly = wy + BODY_Y_OFF;
    int lw = EXPLORER_WIN_W - 4 - TREE_W - 2;
    int lh = BODY_H(wh);
    if (mx < lx || mx >= lx+lw) return -1;
    if (my < ly+16 || my >= ly+lh) return -1;
    int idx = (my - ly - 16) / 16 + g_scroll;
    if (idx >= 0 && idx < loc_count(g_loc)) return idx;
    return -1;
}

void explorer_click(int wx, int wy, int mx, int my) {
    int wh = EXPLORER_WIN_H;

    /* Reset launch signal */
    explorer_launched_exe = -1;

    /* Back button */
    if (hit_back(wx, wy, mx, my)) {
        if (g_hist_pos > 0) {
            g_loc = g_history[--g_hist_pos];
            g_scroll = 0; g_selected = -1;
        }
        return;
    }

    /* Tree click */
    int tree_hit = hit_tree(wx, wy, wh, mx, my);
    if (tree_hit == -2) {
        tree_c_expanded = !tree_c_expanded;
        return;
    }
    if (tree_hit >= 0) {
        if (!g_hdd && tree_hit != LOC_ROOT && tree_hit != LOC_TRASH && tree_hit != LOC_USB) return;
        nav_to((loc_t)tree_hit);
        if (tree_hit == LOC_C || tree_hit == LOC_EGOS ||
            tree_hit == LOC_PROG || tree_hit == LOC_USERS || tree_hit == LOC_DOCS)
            tree_c_expanded = true;
        return;
    }

    if (g_loc == LOC_USB) return;

    /* File list click */
    int entry_idx = hit_entry(wx, wy, wh, mx, my);
    if (entry_idx < 0) {
        g_selected = -1;
        return;
    }

    if (g_selected == entry_idx) {
        /* Double-click */
        const vfs_entry_t *e = &loc_entries[g_loc][entry_idx];

        if (e->is_dir) {
            /* Navigate into directory */
            if (g_loc == LOC_ROOT) {
                if (entry_idx == 0) { if (g_hdd) nav_to(LOC_C); }
                else if (entry_idx == 1) nav_to(LOC_TRASH);
                else if (entry_idx == 2 && usb_drive.present) nav_to(LOC_USB);
            } else if (g_loc == LOC_C) {
                loc_t subs[] = { LOC_EGOS, LOC_PROG, LOC_USERS, LOC_DOCS };
                if (entry_idx < 4) nav_to(subs[entry_idx]);
            }
        } else if (g_loc == LOC_PROG && is_exe(e)) {
            /* Launch EXE: map index to exe_target_t */
            int exe_idx = entry_idx - DIR_PROG_EXE_START;
            if (exe_idx >= 0 && exe_idx <= 4) {
                explorer_launched_exe = exe_idx;
                int i = 0;
                const char *n = e->name;
                while (n[i] && i < 35) { explorer_exe_name[i] = n[i]; i++; }
                explorer_exe_name[i] = 0;
            }
            g_selected = -1;
        }
        /* Other file types: no action */
    } else {
        g_selected = entry_idx;
    }
}
