/* Notepad for Egorov OS — v1.6
 * Features: cursor movement, blue text selection, mouse-click cursor,
 * Russian/English layout (F12), save/open files on HDD. */
#include "notepad.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "string.h"
#include "ps2.h"
#include "ata.h"

#define MAX_CHARS    2048
#define LINE_H       18   /* глиф 16 px + интерлиньяж: раньше 14 и
                           * строки наезжали друг на друга */
#define PAD           6
#define TOOLBAR_H    22

/* геометрия кнопок тулбара — одна на отрисовку и хит-тест.
 * Ширины подобраны под подписи (8 px на символ) с полями, чтобы
 * слова не вылезали из кнопок: Очистить/Выделить 8 симв.,
 * Сохранить 9, Открыть 7. */
static const int np_btn_x[4] = { 4, 84, 164, 252 };
static const int np_btn_w[4] = { 76, 76, 84, 68 };
#define NP_BTN_H     20
#define NP_BTN_Y(wy) ((wy) + 29)

/* LBA on disk for the notepad save file (well past kernel area
 * 2048..2815 — 768 секторов после расширения загрузчика F4). */
#define NOTEPAD_LBA      8192
#define NOTEPAD_SECTORS  5      /* 5 × 512 = 2560 bytes > MAX_CHARS + header */
#define NOTEPAD_MAGIC0   0x4E   /* 'N' */
#define NOTEPAD_MAGIC1   0x50   /* 'P' */
#define NOTEPAD_MAGIC2   0x41   /* 'A' */
#define NOTEPAD_MAGIC3   0x44   /* 'D' */

static char g_buf[MAX_CHARS + 4];
static int  g_len    = 0;
static int  g_cursor = 0;
static int  g_blink  = 0;
static bool g_ru_mode = false;

/* Selection state */
static bool g_has_sel    = false;
static int  g_sel_anchor = 0;   /* byte offset where selection started */

/* HDD available for save/load */
static bool g_hdd_avail = false;

/* ── UTF-8 helpers ── */

static int utf8_encode(uint32_t cp, char out[4]) {
    if (cp < 0x80) {
        out[0] = (char)cp; return 1;
    } else if (cp < 0x800) {
        out[0] = (char)(0xC0 | (cp >> 6));
        out[1] = (char)(0x80 | (cp & 0x3F));
        return 2;
    } else {
        out[0] = (char)(0xE0 | (cp >> 12));
        out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[2] = (char)(0x80 | (cp & 0x3F));
        return 3;
    }
}

/* Длина UTF-8-последовательности от pos вперёд (в байтах).
 * Защита от мусорного/обрезанного UTF-8 (аудит F5 п.18):
 * - continuation-байт без лидера -> 1 (не 0 и не отрицательное);
 * - лид-байт, обрезанный концом буфера -> усечь до g_len-pos,
 *   иначе KEY_RIGHT выводил g_cursor ЗА g_len и курсор исчезал. */
static int utf8_fwd_len(int pos) {
    if (pos >= g_len) return 0;
    uint8_t b = (uint8_t)g_buf[pos];
    int n = 1;
    if ((b & 0xE0) == 0xC0)   n = 2;
    else if ((b & 0xF0) == 0xE0) n = 3;
    else if ((b & 0xF8) == 0xF0) n = 4;
    /* мусорный continuation-байт (0x80..0xBF) остаётся n=1 */
    if (pos + n > g_len) n = g_len - pos;   /* обрезанный лид-байт */
    return n < 1 ? 1 : n;
}

/* Длина UTF-8-последовательности, ЗАКАНЧИВАЮЩЕЙСЯ в pos, назад.
 * Защита от мусора: лид-байт обязан объявлять ровно n байтов;
 * иначе (сиротские continuation-байты, мусор) шаг = 1 — иначе
 * Backspace съедал до 3 байтов «разом» по мусору (F5 п.18). */
static int utf8_back_len(int pos) {
    if (pos <= 0) return 0;
    int n = 1;
    while (n < 4 && (pos - n) > 0 && ((uint8_t)g_buf[pos - n] & 0xC0) == 0x80)
        n++;
    uint8_t lead = (uint8_t)g_buf[pos - n];
    int expect = 1;
    if      ((lead & 0xE0) == 0xC0) expect = 2;
    else if ((lead & 0xF0) == 0xE0) expect = 3;
    else if ((lead & 0xF8) == 0xF0) expect = 4;
    if (expect != n) n = 1;   /* лид не согласован — мусор, шаг 1 */
    return n;
}

/* ── Russian JCUKEN keyboard layout ── */
static const uint32_t ru_lo[26] = {
    0x0444,0x0438,0x0441,0x0432,0x0443,
    0x0430,0x043F,0x0440,0x0448,0x043E,
    0x043B,0x0434,0x044C,0x0442,0x0449,
    0x0437,0x0439,0x043A,0x044B,0x0435,
    0x0433,0x043C,0x0446,0x0447,0x043D,
    0x044F
};
static const uint32_t ru_hi[26] = {
    0x0424,0x0418,0x0421,0x0412,0x0423,
    0x0410,0x041F,0x0420,0x0428,0x041E,
    0x041B,0x0414,0x042C,0x0422,0x0429,
    0x0417,0x0419,0x041A,0x042B,0x0415,
    0x0413,0x041C,0x0426,0x0427,0x041D,
    0x042F
};

/* ── Delete selection helper ── */
static void delete_selection(void) {
    if (!g_has_sel) return;
    int lo = g_sel_anchor < g_cursor ? g_sel_anchor : g_cursor;
    int hi = g_sel_anchor > g_cursor ? g_sel_anchor : g_cursor;
    int del = hi - lo;
    for (int i = lo; i < g_len - del; i++) g_buf[i] = g_buf[i + del];
    g_len   -= del;
    g_cursor = lo;
    g_buf[g_len] = 0;
    g_has_sel = false;
}

/* ── Init / Clear ── */
void notepad_init(void) {
    /* Preserve buffer if there is user content; just reset cursor. */
    if (g_len == 0) {
        static const char *dt =
            "\xd0\x94\xd0\xbe\xd0\xb1\xd1\x80\xd0\xbe "
            "\xd0\xbf\xd0\xbe\xd0\xb6\xd0\xb0\xd0\xbb\xd0\xbe\xd0\xb2\xd0\xb0\xd1\x82\xd1\x8c "
            "\xd0\xb2 Egorov OS!\n"
            "F12 \xe2\x86\x92 EN/RU.\n";
        const char *s = dt;
        while (*s && g_len < MAX_CHARS) g_buf[g_len++] = *s++;
        g_buf[g_len] = 0;
    }
    g_cursor  = g_len;
    g_has_sel = false;
    g_blink   = 0;
}

void notepad_clear(void) {
    g_len = 0; g_cursor = 0;
    g_buf[0] = 0;
    g_has_sel = false;
    g_blink   = 0;
}

void notepad_set_hdd(bool hdd) {
    g_hdd_avail = hdd;
}

/* ── Save / Open ── */
void notepad_save_hdd(void) {
    if (!g_hdd_avail || !ata_primary_master.present) return;
    uint8_t buf[512];

    /* Sector 0 of file: 8-byte header + up to 504 bytes of content */
    memset(buf, 0, 512);
    buf[0] = NOTEPAD_MAGIC0; buf[1] = NOTEPAD_MAGIC1;
    buf[2] = NOTEPAD_MAGIC2; buf[3] = NOTEPAD_MAGIC3;
    buf[4] = (uint8_t)(g_len & 0xFF);
    buf[5] = (uint8_t)((g_len >> 8) & 0xFF);
    buf[6] = 0; buf[7] = 0;

    int first_chunk = g_len < 504 ? g_len : 504;
    memcpy(buf + 8, g_buf, first_chunk);
    /* игнор возврата = тихая потеря сохранённого текста */
    if (!ata_write_sector(NOTEPAD_LBA, buf)) return;

    /* Remaining sectors */
    int offset    = first_chunk;
    int remaining = g_len - first_chunk;
    for (int s = 1; s < NOTEPAD_SECTORS && remaining > 0; s++) {
        memset(buf, 0, 512);
        int chunk = remaining < 512 ? remaining : 512;
        memcpy(buf, g_buf + offset, chunk);
        if (!ata_write_sector(NOTEPAD_LBA + s, buf)) return;
        offset    += chunk;
        remaining -= chunk;
    }
}

void notepad_open_hdd(void) {
    if (!g_hdd_avail || !ata_primary_master.present) return;
    uint8_t buf[512];
    if (!ata_read_sector(NOTEPAD_LBA, buf)) return;

    if (buf[0] != NOTEPAD_MAGIC0 || buf[1] != NOTEPAD_MAGIC1 ||
        buf[2] != NOTEPAD_MAGIC2 || buf[3] != NOTEPAD_MAGIC3) return;

    int len = (int)buf[4] | ((int)buf[5] << 8);
    if (len < 0 || len > MAX_CHARS) return;

    int first_chunk = len < 504 ? len : 504;
    g_len = 0;
    memcpy(g_buf, buf + 8, first_chunk);
    g_len = first_chunk;

    int offset    = first_chunk;
    int remaining = len - first_chunk;
    for (int s = 1; s < NOTEPAD_SECTORS && remaining > 0; s++) {
        if (!ata_read_sector(NOTEPAD_LBA + s, buf)) break;
        int chunk = remaining < 512 ? remaining : 512;
        memcpy(g_buf + offset, buf, chunk);
        g_len     += chunk;
        offset    += chunk;
        remaining -= chunk;
    }
    g_buf[g_len] = 0;
    g_cursor  = g_len;
    g_has_sel = false;
}

/* ── Hit test for toolbar buttons (та же таблица, что в отрисовке) ── */
int notepad_hit_btn(int wx, int wy, int mx, int my) {
    if (my < NP_BTN_Y(wy) || my >= NP_BTN_Y(wy) + NP_BTN_H) return -1;
    for (int i = 0; i < 4; i++) {
        if (mx >= wx + np_btn_x[i] && mx < wx + np_btn_x[i] + np_btn_w[i])
            return i;
    }
    return -1;
}

/* ── Select all ── */
void notepad_select_all(void) {
    if (g_len == 0) return;
    g_sel_anchor = 0;
    g_cursor     = g_len;
    g_has_sel    = true;
}

/* ── Mouse click: move cursor to clicked character position ── */
void notepad_mouse_click(int wx, int wy, int mx, int my) {
    int bx   = wx + 2;
    int by_t = wy + 28 + TOOLBAR_H;
    int bw   = NOTEPAD_WIN_W - 4;
    int bh   = NOTEPAD_WIN_H - 28 - TOOLBAR_H - 4;
    int max_x = bx + bw - PAD;
    int max_y = by_t + bh - PAD;

    /* Click must be inside text area */
    if (mx < bx || mx >= bx + bw || my < by_t || my >= by_t + bh) return;

    int cx = bx + PAD;
    int cy = by_t + PAD;

    int best_pos  = 0;
    int best_dist = 0x7FFFFFFF;

    int i = 0;
    while (i <= g_len) {
        int dcx = cx - mx;
        int dcy = cy - my;
        int dist = dcx * dcx + dcy * dcy;
        if (dist < best_dist) { best_dist = dist; best_pos = i; }
        if (i == g_len) break;

        char ch = g_buf[i];
        if (ch == '\r') { i++; continue; }
        if (ch == '\n') {
            cy += LINE_H; cx = bx + PAD;
            i++;
            if (cy + LINE_H > max_y) break;
            continue;
        }
        if (cx + FONT_GLYPH_W > max_x) {
            cy += LINE_H; cx = bx + PAD;
            if (cy + LINE_H > max_y) break;
        }
        int cl = utf8_fwd_len(i);
        cx += FONT_GLYPH_W;
        i  += cl;
    }

    g_cursor  = best_pos;
    g_has_sel = false;
}

/* ── Draw ── */
void notepad_draw(int wx, int wy, int hover_btn, int pressed_btn) {
    int ww = NOTEPAD_WIN_W;
    int wh = NOTEPAD_WIN_H;
    int bx = wx + 2;
    int by = wy + 28 + TOOLBAR_H;
    int bw = ww - 4;
    int bh = wh - 28 - TOOLBAR_H - 4;

    /* ── Toolbar ── */
    fb_grad_v(wx+2, wy+28, ww-4, TOOLBAR_H, rgb(0xF0,0xEE,0xE0), rgb(0xD8,0xD4,0xC4));
    fb_hline(wx+2, wy+28+TOOLBAR_H-1, ww-4, rgb(0xA0,0xA0,0x90));

    /* четыре кнопки из общей таблицы — подписи влезают с полями */
    {
        static const char *np_label[4] = {
            "\xd0\x9e\xd1\x87\xd0\xb8\xd1\x81\xd1\x82\xd0\xb8\xd1\x82\xd1\x8c",
            "\xd0\x92\xd1\x8b\xd0\xb4\xd0\xb5\xd0\xbb\xd0\xb8\xd1\x82\xd1\x8c",
            "\xd0\xa1\xd0\xbe\xd1\x85\xd1\x80\xd0\xb0\xd0\xbd\xd0\xb8\xd1\x82\xd1\x8c",
            "\xd0\x9e\xd1\x82\xd0\xba\xd1\x80\xd1\x8b\xd1\x82\xd1\x8c"
        };
        for (int i = 0; i < 4; i++)
            gui_button(wx + np_btn_x[i], NP_BTN_Y(wy), np_btn_w[i], NP_BTN_H,
                       np_label[i], hover_btn == i, pressed_btn == i);
    }

    /* Layout indicator */
    text_draw_str(wx+ww-68, wy+32, "F12:", rgb(0x80,0x80,0x80));
    text_draw_str(wx+ww-28, wy+32,
                  g_ru_mode ? "RU" : "EN",
                  g_ru_mode ? rgb(0x10,0x60,0xC0) : rgb(0x20,0x80,0x20));

    /* ── Text area ── */
    fb_rect(bx, by, bw, bh, rgb(0xFF,0xFF,0xFF));
    fb_rect_outline(bx, by, bw, bh, rgb(0x80,0x80,0x80));

    int cx    = bx + PAD;
    int cy    = by + PAD;
    int max_x = bx + bw - PAD;
    int max_y = by + bh - PAD;

    g_blink = (g_blink + 1) & 0x3F;

    /* Selection range */
    int sel_lo = 0, sel_hi = 0;
    if (g_has_sel) {
        sel_lo = g_sel_anchor < g_cursor ? g_sel_anchor : g_cursor;
        sel_hi = g_sel_anchor > g_cursor ? g_sel_anchor : g_cursor;
    }

    int i = 0;
    while (i <= g_len) {
        /* Blinking cursor */
        if (i == g_cursor && g_blink < 32) {
            fb_rect(cx, cy, 2, LINE_H, rgb(0x10,0x10,0x10));
        }
        if (i == g_len) break;

        char ch = g_buf[i];
        if (ch == '\r') { i++; continue; }
        if (ch == '\n') {
            /* Highlight newline character width if selected */
            if (g_has_sel && i >= sel_lo && i < sel_hi)
                fb_rect(cx, cy, FONT_GLYPH_W, LINE_H, rgb(0x40,0x80,0xFF));
            cy += LINE_H; cx = bx + PAD;
            i++;
            if (cy + LINE_H > max_y) break;
            continue;
        }
        if (cx + FONT_GLYPH_W > max_x) {
            cy += LINE_H; cx = bx + PAD;
            if (cy + LINE_H > max_y) break;
        }

        int cl = utf8_fwd_len(i);

        /* Blue highlight for selected range */
        if (g_has_sel && i >= sel_lo && i < sel_hi) {
            fb_rect(cx, cy, FONT_GLYPH_W, LINE_H, rgb(0x40,0x80,0xFF));
        }

        char tmp[5]; int j;
        for (j = 0; j < cl && j < 4; j++) tmp[j] = g_buf[i + j];
        tmp[j] = 0;
        uint32_t col = (g_has_sel && i >= sel_lo && i < sel_hi)
                       ? rgb(0xFF,0xFF,0xFF)
                       : (uint32_t)COL_TEXT;
        text_draw_str(cx, cy, tmp, col);
        cx += FONT_GLYPH_W;
        i  += cl;
    }
}

/* ── Key handler ── */
void notepad_key(uint32_t k) {
    /* F12 — toggle Russian/English layout */
    if (k == KEY_F12) { g_ru_mode = !g_ru_mode; return; }

    /* Shift+Left/Right — extend selection */
    if (k == KEY_SHIFT_LEFT) {
        if (!g_has_sel) { g_sel_anchor = g_cursor; g_has_sel = true; }
        int back = utf8_back_len(g_cursor);
        if (back > 0) g_cursor -= back;
        if (g_cursor == g_sel_anchor) g_has_sel = false;
        return;
    }
    if (k == KEY_SHIFT_RIGHT) {
        if (!g_has_sel) { g_sel_anchor = g_cursor; g_has_sel = true; }
        int fwd = utf8_fwd_len(g_cursor);
        if (fwd > 0) g_cursor += fwd;
        if (g_cursor == g_sel_anchor) g_has_sel = false;
        return;
    }

    /* Arrow keys — UTF-8-aware cursor movement, clears selection */
    if (k == KEY_LEFT) {
        g_has_sel = false;
        int back = utf8_back_len(g_cursor);
        if (back > 0) g_cursor -= back;
        return;
    }
    if (k == KEY_RIGHT) {
        g_has_sel = false;
        int fwd = utf8_fwd_len(g_cursor);
        if (fwd > 0) g_cursor += fwd;
        return;
    }
    if (k == KEY_UP || k == KEY_DOWN) { g_has_sel = false; return; }

    /* Backspace — delete selection or codepoint before cursor */
    if (k == KEY_BS) {
        if (g_has_sel) {
            delete_selection();
        } else {
            int back = utf8_back_len(g_cursor);
            if (back > 0) {
                int new_cur = g_cursor - back;
                for (int i = new_cur; i < g_len - back; i++)
                    g_buf[i] = g_buf[i + back];
                g_len   -= back;
                g_cursor = new_cur;
                g_buf[g_len] = 0;
            }
        }
        return;
    }

    /* Enter — insert newline */
    if (k == '\n' || k == '\r') {
        if (g_has_sel) delete_selection();
        if (g_len < MAX_CHARS) {
            for (int i = g_len; i > g_cursor; i--) g_buf[i] = g_buf[i-1];
            g_buf[g_cursor] = '\n';
            g_len++; g_cursor++;
            g_buf[g_len] = 0;
        }
        return;
    }

    /* Printable characters */
    uint32_t cp = 0;
    if (g_ru_mode) {
        if      (k >= 'a' && k <= 'z') cp = ru_lo[k - 'a'];
        else if (k >= 'A' && k <= 'Z') cp = ru_hi[k - 'A'];
        else if (k == '[')  cp = 0x0445;
        else if (k == ']')  cp = 0x044A;
        else if (k == '{')  cp = 0x0425;
        else if (k == '}')  cp = 0x042A;
        else if (k == ';')  cp = 0x0436;
        else if (k == ':')  cp = 0x0416;
        else if (k == '\'') cp = 0x044D;
        else if (k == '"')  cp = 0x042D;
        else if (k == ',')  cp = 0x0431;
        else if (k == '<')  cp = 0x0411;
        else if (k == '.')  cp = 0x044E;
        else if (k == '>')  cp = 0x042E;
        else if (k >= 0x20 && k < 0x100) cp = k;
    } else {
        if (k >= 0x20 && k < 0x100) cp = k;
    }
    if (!cp) return;

    /* If text is selected, delete it first */
    if (g_has_sel) delete_selection();

    char enc[4];
    int enc_len = utf8_encode(cp, enc);
    if (g_len + enc_len > MAX_CHARS) return;

    for (int i = g_len + enc_len - 1; i >= g_cursor + enc_len; i--)
        g_buf[i] = g_buf[i - enc_len];
    for (int i = 0; i < enc_len; i++) g_buf[g_cursor + i] = enc[i];
    g_len    += enc_len;
    g_cursor += enc_len;
    g_buf[g_len] = 0;
}
