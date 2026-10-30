/* Read-only Explorer for Egorov OS 0.3.1.
 * Lists actual MBR primary partitions with valid FAT32 BPBs and scans real
 * directories through the VFS. No synthetic file/directory entries are used. */
#include "explorer.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "string.h"
#include "vfs.h"

#define TREE_W      150
#define TOOLBAR_H   26
#define STATUSBAR_H 18
#define TITLE_H     28
#define BODY_Y_OFF  (TITLE_H + TOOLBAR_H)
#define BODY_H(wh)  ((wh) - TITLE_H - TOOLBAR_H - STATUSBAR_H)
#define EXPLORER_MAX_VISIBLE_ENTRIES VFS_MAX_ENTRIES
#define EXPLORER_HISTORY 16
#define TREE_ROOT_HIT (-10)
#define TREE_VOLUME_BASE 100
#define TREE_TRASH_HIT (-11)

/* Existing desktop launch handshake retained for ABI compatibility. Real PE32
 * loading is a later roadmap milestone; a file named .exe is not faked as a
 * successfully launched process by this read-only Explorer. */
int  explorer_launched_exe = -1;
char explorer_exe_name[36] = {0};

typedef struct {
    char name[VFS_NAME_MAX];
    bool is_dir;
    bool is_volume;
    bool is_trash;
    uint32_t size;
    uint32_t volume_size_mb;
    uint32_t first_cluster;
    int volume_index;
} explorer_entry_t;

typedef struct {
    int volume_index;       /* -1 = My Computer */
    uint32_t cluster;       /* 0 = volume root */
    bool trash;
    char path[192];
} nav_state_t;

static explorer_entry_t g_entries[EXPLORER_MAX_VISIBLE_ENTRIES];
static vfs_dirent_t g_fs_entries[VFS_MAX_ENTRIES];
static int g_entry_count;
static int g_volume_index = -1;
static uint32_t g_cluster;
static bool g_trash;
static char g_path[192] = "Мой компьютер";
static char g_status[128] = "Проводник: только чтение; MBR + FAT32";
static bool g_scan_error;
static bool g_truncated;
static nav_state_t g_history[EXPLORER_HISTORY];
static int g_history_count;
static int g_scroll;
static int g_selected = -1;

static void str_copy(char *dst, int cap, const char *src) {
    int i = 0;
    if (!dst || cap <= 0) return;
    if (!src) src = "";
    while (src[i] && i < cap - 1) { dst[i] = src[i]; i++; }
    dst[i] = 0;
}
static void str_append(char *dst, int cap, const char *src) {
    int p = 0;
    if (!dst || cap <= 0) return;
    while (dst[p] && p < cap - 1) p++;
    int i = 0;
    while (src && src[i] && p < cap - 1) dst[p++] = src[i++];
    dst[p] = 0;
}
static void set_status(const char *s) { str_copy(g_status, sizeof(g_status), s); }

static void nav_save(void) {
    if (g_history_count >= EXPLORER_HISTORY) {
        for (int i = 1; i < EXPLORER_HISTORY; i++) g_history[i-1] = g_history[i];
        g_history_count = EXPLORER_HISTORY - 1;
    }
    nav_state_t *s = &g_history[g_history_count++];
    s->volume_index = g_volume_index;
    s->cluster = g_cluster;
    s->trash = g_trash;
    str_copy(s->path, sizeof(s->path), g_path);
}

static void fill_volume_entry(int idx, explorer_entry_t *e) {
    const vfs_volume_t *v = vfs_volume_get(idx);
    memset(e, 0, sizeof(*e));
    if (!v) return;
    e->is_dir = true;
    e->is_volume = true;
    e->volume_index = idx;
    str_copy(e->name, sizeof(e->name), v->label[0] ? v->label : "FAT32 volume");
    str_append(e->name, sizeof(e->name), " (FAT32, ");
    str_append(e->name, sizeof(e->name), v->device_name);
    str_append(e->name, sizeof(e->name), ")");
    e->volume_size_mb = v->total_sectors / 2048u;
}

static void refresh_current(void) {
    g_entry_count = 0;
    g_scan_error = false;
    g_truncated = false;
    g_scroll = 0;
    g_selected = -1;
    if (g_trash) {
        set_status("Корзина пока не реализована; данные на диске не изменены.");
        return;
    }
    if (g_volume_index < 0) {
        int n = vfs_volume_count();
        for (int i = 0; i < n && g_entry_count < EXPLORER_MAX_VISIBLE_ENTRIES - 1; i++)
            fill_volume_entry(i, &g_entries[g_entry_count++]);
        if (g_entry_count < EXPLORER_MAX_VISIBLE_ENTRIES) {
            explorer_entry_t *e = &g_entries[g_entry_count++];
            memset(e, 0, sizeof(*e));
            str_copy(e->name, sizeof(e->name), "Корзина");
            e->is_dir = true;
            e->is_trash = true;
            e->volume_index = -1;
        }
        if (n == 0) set_status("Не найдено поддерживаемых FAT32-томов (MBR). GPT/NTFS/exFAT пока не поддерживаются.");
        else if (vfs_last_list_truncated()) set_status("Список томов ограничен лимитом VFS.");
        else set_status("Обнаружены реальные FAT32-тома; двойной щелчок открывает том.");
        return;
    }

    int n = vfs_readdir(g_volume_index, g_cluster, g_fs_entries, VFS_MAX_ENTRIES);
    if (n < 0) {
        g_scan_error = true;
        set_status("Ошибка чтения каталога FAT32. Проверьте диск и журнал.");
        return;
    }
    for (int i = 0; i < n && i < EXPLORER_MAX_VISIBLE_ENTRIES; i++) {
        explorer_entry_t *e = &g_entries[g_entry_count++];
        memset(e, 0, sizeof(*e));
        str_copy(e->name, sizeof(e->name), g_fs_entries[i].name);
        e->is_dir = g_fs_entries[i].is_dir;
        e->size = g_fs_entries[i].size;
        e->first_cluster = g_fs_entries[i].first_cluster;
        e->volume_index = g_volume_index;
    }
    g_truncated = vfs_last_list_truncated();
    if (g_truncated) set_status("Каталог показан частично: достигнут предел записей проводника.");
    else if (n == 0) set_status("Каталог пуст.");
    else set_status("FAT32: только чтение. Файлы не изменяются и не запускаются.");
}

static void navigate_root(void) {
    g_volume_index = -1;
    g_cluster = 0;
    g_trash = false;
    str_copy(g_path, sizeof(g_path), "Мой компьютер");
    refresh_current();
}
static void navigate_volume(int volume_index) {
    const vfs_volume_t *v = vfs_volume_get(volume_index);
    if (!v) { set_status("Том исчез или больше недоступен."); return; }
    nav_save();
    g_volume_index = volume_index;
    g_cluster = 0;
    g_trash = false;
    str_copy(g_path, sizeof(g_path), v->label[0] ? v->label : "FAT32");
    str_append(g_path, sizeof(g_path), ":\\");
    refresh_current();
}
static void navigate_directory(const explorer_entry_t *e) {
    if (!e || !e->is_dir) return;
    if (e->is_trash) {
        nav_save();
        g_trash = true;
        g_volume_index = -1;
        g_cluster = 0;
        str_copy(g_path, sizeof(g_path), "Корзина");
        refresh_current();
        return;
    }
    if (e->is_volume) { navigate_volume(e->volume_index); return; }
    if (g_volume_index < 0 || e->first_cluster < 2u) {
        set_status("Некорректный кластер каталога; переход отменён.");
        return;
    }
    nav_save();
    g_cluster = e->first_cluster;
    str_append(g_path, sizeof(g_path), e->name);
    str_append(g_path, sizeof(g_path), "\\");
    refresh_current();
}

void explorer_init(bool booted_from_hdd) {
    (void)booted_from_hdd;
    explorer_launched_exe = -1;
    explorer_exe_name[0] = 0;
    g_history_count = 0;
    g_volume_index = -1;
    g_cluster = 0;
    g_trash = false;
    g_path[0] = 0;
    str_copy(g_path, sizeof(g_path), "Мой компьютер");
    /* Inventory is refreshed when Explorer opens, even in Live mode. */
    vfs_refresh();
    refresh_current();
}

void explorer_open_trash(void) {
    nav_save();
    g_trash = true;
    g_volume_index = -1;
    g_cluster = 0;
    str_copy(g_path, sizeof(g_path), "Корзина");
    refresh_current();
}

static void draw_folder_icon(int x, int y, uint32_t col) {
    fb_rect(x, y+2, 6, 2, col);
    fb_rect(x, y+4, 10, 6, col);
}
static void draw_file_icon(int x, int y) {
    fb_rect(x, y, 8, 10, rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(x, y, 8, 10, rgb(0x80,0x80,0x80));
    fb_pixel(x+5, y, rgb(0x80,0x80,0x80));
    fb_pixel(x+6, y+1, rgb(0x80,0x80,0x80));
    fb_pixel(x+7, y+2, rgb(0x80,0x80,0x80));
}
static bool name_is_exe(const explorer_entry_t *e) {
    int n = 0;
    if (!e || e->is_dir) return false;
    while (e->name[n]) n++;
    if (n < 4) return false;
    return e->name[n-4] == '.' &&
           (e->name[n-3] == 'e' || e->name[n-3] == 'E') &&
           (e->name[n-2] == 'x' || e->name[n-2] == 'X') &&
           (e->name[n-1] == 'e' || e->name[n-1] == 'E');
}
static void draw_entry_row(int lx, int ry, int lw, int idx,
                           const explorer_entry_t *e, bool selected) {
    if (selected) fb_rect(lx, ry, lw, 16, rgb(0x32,0x6B,0xCE));
    if (e->is_dir) draw_folder_icon(lx+2, ry+2,
        selected ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
    else draw_file_icon(lx+2, ry+2);
    uint32_t tc = selected ? COL_WHITE : COL_TEXT;
    text_draw_str(lx+16, ry+2, e->name, tc);
    char sz[24], num[16];
    if (e->is_volume) {
        itoa(e->volume_size_mb, num, 10);
        int p = 0;
        for (int i=0; num[i] && p<18; i++) sz[p++] = num[i];
        const char *s = " МБ"; while (*s && p<22) sz[p++] = *s++;
        sz[p] = 0;
        text_draw_str(lx+lw-54, ry+2, sz, tc);
    } else if (!e->is_dir) {
        uint32_t kb = e->size / 1024u + ((e->size % 1024u) ? 1u : 0u);
        itoa(kb, num, 10);
        int p = 0;
        for (int i=0; num[i] && p<18; i++) sz[p++] = num[i];
        const char *s = " КБ"; while (*s && p<22) sz[p++] = *s++;
        sz[p] = 0;
        text_draw_str(lx+lw-54, ry+2, sz, tc);
    } else text_draw_str(lx+lw-54, ry+2, "Папка", tc);
    (void)idx;
}

static void draw_tree(int wx, int wy, int wh) {
    int tx = wx + 2, ty = wy + BODY_Y_OFF, th = BODY_H(wh);
    fb_rect(tx, ty, TREE_W, th, rgb(0xF0,0xF0,0xF0));
    fb_rect_outline(tx, ty, TREE_W, th, rgb(0xC0,0xC0,0xC0));
    int cy = ty + 4;
    bool root_sel = g_volume_index < 0 && !g_trash;
    if (root_sel) fb_rect(tx+2, cy-1, TREE_W-4, 16, rgb(0x32,0x6B,0xCE));
    draw_folder_icon(tx+4, cy, root_sel ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
    text_draw_str(tx+18, cy, "Мой компьютер", root_sel ? COL_WHITE : COL_TEXT);
    cy += 20;
    int n = vfs_volume_count();
    int rows = th > 34 ? (th - 34) / 16 : 0;
    if (rows > n) rows = n;
    for (int i = 0; i < rows; i++) {
        bool sel = (g_volume_index == i && !g_trash);
        if (sel) fb_rect(tx+10, cy-1, TREE_W-12, 15, rgb(0x32,0x6B,0xCE));
        draw_folder_icon(tx+14, cy, sel ? rgb(0xFF,0xD0,0x50) : rgb(0xFF,0xC0,0x30));
        char label[20], num[8];
        str_copy(label, sizeof(label), "Том "); itoa((uint32_t)(i+1), num, 10); str_append(label, sizeof(label), num);
        text_draw_str(tx+28, cy, label, sel ? COL_WHITE : COL_TEXT);
        cy += 16;
    }
    cy = ty + th - 18;
    bool trash_sel = g_trash;
    if (trash_sel) fb_rect(tx+2, cy-1, TREE_W-4, 16, rgb(0x32,0x6B,0xCE));
    draw_folder_icon(tx+4, cy, trash_sel ? rgb(0xFF,0xD0,0x50) : rgb(0xC0,0xC0,0xC8));
    text_draw_str(tx+18, cy, "Корзина", trash_sel ? COL_WHITE : COL_TEXT);
}

static void draw_toolbar(int wx, int wy, int ww) {
    int tx = wx+2, ty = wy+TITLE_H;
    fb_grad_v(tx, ty, ww-4, TOOLBAR_H, rgb(0xF0,0xEE,0xE0), rgb(0xD8,0xD4,0xC4));
    fb_hline(tx, ty+TOOLBAR_H-1, ww-4, rgb(0xA0,0xA0,0x90));
    gui_button(tx+2, ty+2, 36, TOOLBAR_H-4, "< Назад", g_history_count > 0, false);
    gui_button(tx+42, ty+2, 62, TOOLBAR_H-4, "Обновить", true, false);
    int abx = tx + 108, abw = ww - 116;
    fb_rect(abx, ty+4, abw, TOOLBAR_H-8, rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(abx, ty+4, abw, TOOLBAR_H-8, rgb(0x80,0x80,0x80));
    text_draw_str(abx+4, ty+6, g_path, COL_TEXT);
}
static void draw_statusbar(int wx, int wy, int ww, int wh) {
    int ty = wy + wh - STATUSBAR_H;
    fb_rect(wx+2, ty, ww-4, STATUSBAR_H, rgb(0xD4,0xD0,0xC8));
    fb_hline(wx+2, ty, ww-4, rgb(0x80,0x80,0x80));
    text_draw_str(wx+6, ty+2, g_status, g_scan_error ? rgb(0xB0,0x10,0x10) : COL_TEXT);
}

void explorer_draw(int wx, int wy, int mx, int my, bool left_down) {
    (void)mx; (void)my; (void)left_down;
    int ww = EXPLORER_WIN_W, wh = EXPLORER_WIN_H;
    fb_rect(wx+2, wy+TITLE_H+TOOLBAR_H, ww-4, BODY_H(wh), rgb(0xFF,0xFF,0xFF));
    draw_toolbar(wx, wy, ww);
    draw_tree(wx, wy, wh);
    draw_statusbar(wx, wy, ww, wh);

    int lx = wx + 2 + TREE_W + 2;
    int ly = wy + BODY_Y_OFF;
    int lw = ww - 4 - TREE_W - 2;
    int lh = BODY_H(wh);
    fb_rect(lx, ly, lw, lh, rgb(0xFF,0xFF,0xFF));
    fb_rect(lx, ly, lw, 16, rgb(0xD8,0xD4,0xC4));
    fb_hline(lx, ly+15, lw, rgb(0x80,0x80,0x80));
    text_draw_str(lx+2, ly+2, "Имя", COL_TEXT);
    text_draw_str(lx+lw-54, ly+2, "Размер", COL_TEXT);
    int max_visible = (lh - 16) / 16;
    int start = g_scroll;
    if (start > g_entry_count - max_visible) start = g_entry_count - max_visible;
    if (start < 0) start = 0;
    if (g_scan_error) {
        text_draw_str(lx+10, ly+30, "Не удалось прочитать каталог.", rgb(0xC0,0x10,0x10));
        text_draw_str(lx+10, ly+48, "Файлы не изменялись.", COL_TEXT);
        return;
    }
    if (g_entry_count == 0) {
        if (g_volume_index < 0)
            text_draw_str(lx+10, ly+30, "Нет распознанных FAT32-томов.", rgb(0x80,0x80,0x80));
        else text_draw_str(lx+10, ly+30, "Каталог пуст.", rgb(0x80,0x80,0x80));
    }
    for (int i = start; i < g_entry_count && (i-start) < max_visible; i++) {
        int ry = ly + 16 + (i-start) * 16;
        draw_entry_row(lx, ry, lw, i, &g_entries[i], g_selected == i);
    }
}

static bool hit(int mx, int my, int x, int y, int w, int h) {
    return mx >= x && mx < x+w && my >= y && my < y+h;
}
static int hit_tree(int wx, int wy, int wh, int mx, int my) {
    int tx = wx+2, ty = wy+BODY_Y_OFF;
    if (mx < tx || mx >= tx+TREE_W || my < ty || my >= ty+BODY_H(wh)) return -1;
    int cy = ty + 4;
    if (my >= cy-1 && my < cy+15) return TREE_ROOT_HIT;
    cy += 20;
    int rows = BODY_H(wh) > 34 ? (BODY_H(wh)-34)/16 : 0;
    int n = vfs_volume_count(); if (rows > n) rows = n;
    for (int i = 0; i < rows; i++, cy += 16)
        if (my >= cy-1 && my < cy+14) return TREE_VOLUME_BASE + i;
    if (my >= ty+BODY_H(wh)-19 && my < ty+BODY_H(wh)-2) return TREE_TRASH_HIT;
    return -1;
}
static int hit_entry(int wx, int wy, int wh, int mx, int my) {
    int lx = wx + 2 + TREE_W + 2, ly = wy + BODY_Y_OFF;
    int lw = EXPLORER_WIN_W - 4 - TREE_W - 2, lh = BODY_H(wh);
    if (mx < lx || mx >= lx+lw || my < ly+16 || my >= ly+lh) return -1;
    int idx = (my - ly - 16) / 16 + g_scroll;
    return (idx >= 0 && idx < g_entry_count) ? idx : -1;
}

void explorer_click(int wx, int wy, int mx, int my) {
    const int wh = EXPLORER_WIN_H;
    explorer_launched_exe = -1;

    int tx = wx+2, ty = wy+TITLE_H;
    if (hit(mx, my, tx+2, ty+2, 36, TOOLBAR_H-4)) {
        if (g_history_count > 0) {
            nav_state_t *s = &g_history[--g_history_count];
            g_volume_index = s->volume_index;
            g_cluster = s->cluster;
            g_trash = s->trash;
            str_copy(g_path, sizeof(g_path), s->path);
            refresh_current();
        }
        return;
    }
    if (hit(mx, my, tx+42, ty+2, 62, TOOLBAR_H-4)) {
        vfs_refresh();
        /* Keep the current path if the same volume still exists. */
        if (g_volume_index >= vfs_volume_count()) navigate_root();
        else refresh_current();
        set_status("Сканирование дисков завершено (режим только чтение).");
        return;
    }

    int th = hit_tree(wx, wy, wh, mx, my);
    if (th == TREE_ROOT_HIT) { navigate_root(); g_history_count = 0; return; }
    if (th >= TREE_VOLUME_BASE && th < TREE_VOLUME_BASE + vfs_volume_count()) {
        navigate_volume(th - TREE_VOLUME_BASE); return;
    }
    if (th == TREE_TRASH_HIT) { explorer_open_trash(); return; }

    int idx = hit_entry(wx, wy, wh, mx, my);
    if (idx < 0) { g_selected = -1; return; }
    if (g_selected != idx) { g_selected = idx; return; }

    explorer_entry_t e = g_entries[idx];
    if (e.is_dir) {
        navigate_directory(&e);
        return;
    }
    /* 0.3.1 is intentionally read-only: list real files, but do not pretend
     * that arbitrary binaries are executable or that the filesystem is writable. */
    if (name_is_exe(&e)) set_status("EXE найден. Реальный PE32-запуск запланирован на 0.3.3.");
    else set_status("Файл найден на диске. Открытие и копирование появятся в следующих этапах.");
}
