/* Manual LOGWRITER window: choose a visible USB Mass Storage transport,
 * export a frozen journal snapshot, and require read-back verification. */
#include "logwriter.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "ps2.h"
#include "string.h"

#define LW_BG       rgb(0xEC, 0xE9, 0xD8)
#define LW_TEXT     rgb(0x18, 0x18, 0x18)
#define LW_SELECTED rgb(0xC8, 0xDE, 0xF9)
#define LW_OK       rgb(0x12, 0x75, 0x24)
#define LW_ERROR    rgb(0xB0, 0x20, 0x20)
#define LW_ROW_Y    90
#define LW_ROW_H    42

static logwriter_device_t g_devices[LOGWRITER_MAX_DEVICES];
static int g_count;
static int g_selected = -1;
static bool g_finished;
static bool g_success;
static bool g_raw_available; /* no valid FAT32 found; explicit RAW path may be offered */
static bool g_raw_armed;     /* second confirmation required before destructive RAW */
static uint32_t g_bytes;
static char g_status[LOGWRITER_RESULT_SIZE];

static void lw_set_status(const char *s) {
    uint32_t i = 0;
    if (!s) { g_status[0] = 0; return; }
    while (s[i]) {
        unsigned char c = (unsigned char)s[i];
        uint32_t n = (c < 0x80u) ? 1u :
                     ((c & 0xE0u) == 0xC0u) ? 2u :
                     ((c & 0xF0u) == 0xE0u) ? 3u :
                     ((c & 0xF8u) == 0xF0u) ? 4u : 1u;
        if (i + n >= sizeof(g_status)) break;
        bool complete = true;
        for (uint32_t j = 1; j < n; j++) if (!s[i + j]) complete = false;
        if (!complete) break;
        for (uint32_t j = 0; j < n; j++) g_status[i + j] = s[i + j];
        i += n;
    }
    g_status[i] = 0;
}

static const char *lw_transport_name(int t) {
    switch (t) {
        case LS_TR_OHCI: return "OHCI";
        case LS_TR_XHCI: return "xHCI";
        case LS_TR_UHCI: return "UHCI";
        default: return "USB";
    }
}

static void lw_num(char *out, uint32_t n) {
    itoa(n, out, 10);
}

static void lw_refresh(void) {
    memset(g_devices, 0, sizeof(g_devices));
    g_count = logwriter_list_devices(g_devices, LOGWRITER_MAX_DEVICES);
    if (g_count < 0) g_count = 0;
    if (g_count > LOGWRITER_MAX_DEVICES) g_count = LOGWRITER_MAX_DEVICES;
    g_selected = -1;
    g_finished = false;
    g_success = false;
    g_raw_available = false;
    g_raw_armed = false;
    g_bytes = 0;
    if (g_count == 0)
        lw_set_status("USB Mass Storage не обнаружены. Проверьте контроллер и повторите поиск.");
    else
        lw_set_status("Выберите флешку, затем нажмите «Записать и проверить».");
}

void logwriter_ui_init(void) {
    lw_refresh();
}

static void lw_draw_status(int x, int y, int maxw) {
    const char *s = g_status;
    char line[96];
    int row = 0;
    while (*s && row < 2) {
        int p = 0;
        const char *last_space = 0;
        int last_space_p = -1;
        while (*s && p < 90) {
            unsigned char c = (unsigned char)*s;
            int n = (c < 0x80) ? 1 : ((c & 0xE0) == 0xC0 ? 2 : ((c & 0xF0) == 0xE0 ? 3 : 4));
            if (p + n >= (int)sizeof(line)) break;
            if (c == ' ') { last_space = s; last_space_p = p; }
            for (int j = 0; j < n && s[j]; j++) line[p++] = s[j];
            line[p] = 0;
            if (text_width(line) > maxw) {
                if (last_space && last_space_p > 0) {
                    p = last_space_p;
                    s = last_space + 1;
                }
                break;
            }
            s += n;
        }
        if (p == 0 && *s) { /* one very wide glyph: make progress safely */
            unsigned char c = (unsigned char)*s;
            int n = (c < 0x80) ? 1 : ((c & 0xE0) == 0xC0 ? 2 : ((c & 0xF0) == 0xE0 ? 3 : 4));
            for (int j = 0; j < n && *s; j++) line[p++] = *s++;
        }
        line[p] = 0;
        text_draw_str(x, y + row * (FONT_GLYPH_H + 3), line,
                      g_raw_armed ? LW_ERROR : (g_finished ? (g_success ? LW_OK : LW_ERROR) : LW_TEXT));
        row++;
        while (*s == ' ') s++;
    }
}

void logwriter_ui_draw(int wx, int wy, int w, int h) {
    fb_rect(wx + 3, wy + 28, w - 6, h - 31, LW_BG);
    text_draw_str(wx + 14, wy + 38,
                  "Обычный режим: безопасный FAT32-раздел с любой меткой.", LW_TEXT);
    text_draw_str(wx + 14, wy + 56,
                  "Если FAT32 не распознан, доступен RAW-режим с подтверждением.", LW_TEXT);
    text_draw_str(wx + 14, wy + 69,
                  "ВНИМАНИЕ: RAW перезаписывает LBA 0 и ломает разметку диска.", rgb(0xB0,0x20,0x20));

    if (g_count == 0) {
        fb_rect_outline(wx + 14, wy + LW_ROW_Y, w - 28, 42, rgb(0xA0,0x90,0x80));
        text_draw_str(wx + 24, wy + LW_ROW_Y + 13, "Накопители не обнаружены.", LW_ERROR);
    } else {
        for (int i = 0; i < g_count; i++) {
            int ry = wy + LW_ROW_Y + i * LW_ROW_H;
            fb_rect(wx + 14, ry, w - 28, LW_ROW_H - 4,
                    (i == g_selected) ? LW_SELECTED : rgb(0xF8,0xF6,0xEE));
            fb_rect_outline(wx + 14, ry, w - 28, LW_ROW_H - 4,
                            (i == g_selected) ? rgb(0x25,0x70,0xC0) : rgb(0xB5,0xB0,0xA0));
            char num[12], cap[12], line[80];
            lw_num(num, (uint32_t)(i + 1));
            lw_num(cap, g_devices[i].sectors / 2048u);
            int p = 0;
            const char *prefix = (i == g_selected) ? "> " : "  ";
            while (*prefix && p < 78) line[p++] = *prefix++;
            for (int j = 0; num[j] && p < 78; j++) line[p++] = num[j];
            const char *dot = ". "; while (*dot && p < 78) line[p++] = *dot++;
            const char *tr = lw_transport_name(g_devices[i].transport);
            while (*tr && p < 78) line[p++] = *tr++;
            const char *sep = " | "; while (*sep && p < 78) line[p++] = *sep++;
            for (int j = 0; g_devices[i].name[j] && p < 78; j++) line[p++] = g_devices[i].name[j];
            line[p] = 0;
            text_draw_str(wx + 22, ry + 3, line, LW_TEXT);

            p = 0;
            const char *s1 = "Ёмкость примерно "; while (*s1 && p < 78) line[p++] = *s1++;
            for (int j = 0; cap[j] && p < 78; j++) line[p++] = cap[j];
            const char *s2 = " МиБ"; while (*s2 && p < 78) line[p++] = *s2++;
            line[p] = 0;
            text_draw_str(wx + 34, ry + 21, line, rgb(0x55,0x55,0x55));
        }
    }

    int status_y = wy + h - 89;
    lw_draw_status(wx + 16, status_y, w - 32);

    int by = wy + h - 42;
    gui_button(wx + 12, by, 112, 28, "Обновить", false, false);
    const char *write_label = g_raw_armed ? "ДА, ЗАПИСАТЬ RAW" :
                              (g_raw_available ? "Подготовить RAW" : "Записать и проверить");
    gui_button(wx + 132, by, 264, 28, write_label, false, false);
    gui_button(wx + w - 124, by, 112, 28, "Закрыть", false, false);
}

static void lw_do_export(void) {
    if (g_selected < 0 || g_selected >= g_count) {
        lw_set_status("Сначала выберите накопитель в списке.");
        g_finished = true;
        g_success = false;
        return;
    }

    /* The user must first see the destructive warning and click again. */
    if (g_raw_available && !g_raw_armed) {
        g_raw_armed = true;
        g_finished = false;
        g_success = false;
        lw_set_status("RAW перезапишет LBA 0 и разметку. Файлы могут стать недоступны. Только пустая флешка. Нажмите ещё раз.");
        return;
    }

    g_bytes = 0;
    char result[LOGWRITER_RESULT_SIZE];
    int status;
    if (g_raw_armed) {
        status = logwriter_export_raw(g_devices[g_selected].transport,
                                      g_devices[g_selected].sectors,
                                      g_devices[g_selected].name, &g_bytes,
                                      result, sizeof(result));
        g_raw_armed = false;
        g_raw_available = false;
    } else {
        status = logwriter_export(g_devices[g_selected].transport,
                                  g_devices[g_selected].sectors,
                                  g_devices[g_selected].name, &g_bytes,
                                  result, sizeof(result));
    }

    if (status == LOGWRITER_EXPORT_RAW_REQUIRED) {
        g_raw_available = true;
        g_raw_armed = false;
        g_finished = false;
        g_success = false;
        lw_set_status("FAT32-раздел не распознан. Нажмите «Подготовить RAW», затем прочитайте предупреждение и подтвердите. RAW делает накопитель немонтируемым как обычную флешку.");
        return;
    }

    g_finished = true;
    g_success = (status == LOGWRITER_EXPORT_OK);
    if (g_success) {
        char bytes[12];
        lw_num(bytes, g_bytes);
        lw_set_status(result);
        /* Append byte count when room remains. */
        uint32_t n = (uint32_t)strlen(g_status);
        const char *sep = " Байтов: ";
        for (uint32_t i = 0; sep[i] && n + 1u < sizeof(g_status); i++) g_status[n++] = sep[i];
        for (uint32_t i = 0; bytes[i] && n + 1u < sizeof(g_status); i++) g_status[n++] = bytes[i];
        g_status[n] = 0;
    } else {
        lw_set_status(result[0] ? result : "Запись не удалась; подробности в диагностическом журнале.");
    }
}

bool logwriter_ui_click(int wx, int wy, int mx, int my) {
    int lx = mx - wx, ly = my - wy;
    if (ly >= LW_ROW_Y && ly < LW_ROW_Y + LOGWRITER_MAX_DEVICES * LW_ROW_H) {
        int i = (ly - LW_ROW_Y) / LW_ROW_H;
        bool on_row = i >= 0 && i < g_count &&
                      ly < LW_ROW_Y + i * LW_ROW_H + (LW_ROW_H - 4);
        if (on_row) {
            g_selected = i;
            g_finished = false;
            g_success = false;
            g_raw_available = false;
            g_raw_armed = false;
            lw_set_status("Нажмите «Записать и проверить». При отсутствии FAT32 можно выбрать RAW.");
        }
        return false;
    }
    int by = LOGWRITER_WIN_H - 42;
    if (ly >= by && ly < by + 28) {
        if (lx >= 12 && lx < 124) {
            lw_refresh();
            return false;
        }
        if (lx >= 132 && lx < 396) {
            lw_do_export();
            return false;
        }
        if (lx >= LOGWRITER_WIN_W - 124 && lx < LOGWRITER_WIN_W - 12)
            return true;
    }
    (void)lx;
    return false;
}

bool logwriter_ui_key(uint32_t key) {
    if (key == KEY_ESC) return true;
    if (key == KEY_UP && g_count > 0) {
        if (g_selected < 0) g_selected = 0;
        else if (g_selected > 0) g_selected--;
        g_finished = false;
        return false;
    }
    if (key == KEY_DOWN && g_count > 0) {
        if (g_selected < 0) g_selected = 0;
        else if (g_selected + 1 < g_count) g_selected++;
        g_finished = false;
        return false;
    }
    if (key == KEY_ENTER) { lw_do_export(); return false; }
    return false;
}
