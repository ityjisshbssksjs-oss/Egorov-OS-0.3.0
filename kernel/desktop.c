/* Рабочий стол Egorov OS: фон «bliss», панель задач с меню «Пуск»,
 * окна приложений, модальные диалоги, часы в трее.
 * Курсором управляют стрелки или мышь — Ctrl+J переключает режим,
 * движение мыши возвращает управление мыши; стартовый режим выбирается
 * по наличию живого указующего устройства. */
#include "fb.h"
#include "text.h"
#include "gui.h"
#include "ps2.h"
#include "usb_mouse.h"
#include "mouse_hw.h"
#include "string.h"
#include "rtc.h"
#include "calc.h"
#include "clock_app.h"
#include "paint.h"
#include "explorer.h"
#include "notepad.h"
#include "sysinfo.h"
#include "snake.h"
#include "exe_runner.h"
#include "terminal.h"
#include "power.h"
#include "version.h"   /* [#473] ЭТАП 9: «Сборка N» в окнах О системе */

/* «ЭТАП 3»: калиброванная задержка из UHCI-стека (usb_uhci_kbd.c) —
 * для темпинга кадров typematic-движения курсора. */
extern void hal_msleep(uint32_t ms);
extern void serial_puts(const char *s);

/* ── Статус мыши и режим курсора ─────────────────────────────────────── */

/* true — курсор водят стрелки (кнопки мыши работают в обоих режимах).
 * Ctrl+J переключает, а любое движение мыши возвращает режим мыши —
 * отдельного «обратного переключения» не требуется. */
static bool cursor_kb_mode = true;

/* ── «ЭТАП 3»: программный auto-repeat стрелок (typematic) ──────────────
 * РАНЬШЕ (v0.3.0 с PS/2-клавиатурой) стрелку можно было ЗАЖАТЬ и мышь
 * «ехала», пока не отпустишь: PS/2-контроллер сам шлёт повторы make.
 * У USB HID повтора НЕТ — отчёт просто держит usage зажатой клавиши,
 * и диф-разбор даёт ОДНО нажатие = один шаг 8 px.  Возвращаем
 * поведение «как раньше и как в Linux»: отслеживаем зажатые стрелки
 * (make/break из ps2_poll_key, KEY_BREAK) и двигаем курсор КАЖДЫЙ
 * кадр, пока стрелка удерживается: начальная задержка ~230 мс, потом
 * плавный разгон шага 4→16 px (кадр = 15 мс).  Работает с ЛЮБОЙ
 * клавиатурой: PS/2, UHCI, OHCI, xHCI, EHCI — событие одно и то же. */
static bool s_arr_l, s_arr_r, s_arr_u, s_arr_d;  /* стрелки зажаты     */
static uint32_t s_arr_hold;                       /* кадров удержания   */

static void arr_held_clear(void) {
    s_arr_l = s_arr_r = s_arr_u = s_arr_d = false;
    s_arr_hold = 0;
}

/* e2e-маркер typematic: старт движения логируется один раз на удержание. */
static bool s_typ_logged;
static void typ_logged_reset(void) { s_typ_logged = false; }

static char mstat_line[80];    /* «Мышь: …» для окна «Мой компьютер» */
static char mstat_tray[80];    /* короткая строка для панели задач   */
static bool mstat_missing;     /* мыши нет — статус в трее красный   */

static void sput(char *buf, int *p, const char *s) {
    while (*s) buf[(*p)++] = *s++;
}

static void mouse_status_refresh(void) {
    const char *line, *tray;
    mstat_missing = false;

    if (usb_mouse_present()) {
        line = "Мышь: USB (UHCI)";  tray = "USB (UHCI)";
    } else if (mouse_hw_present()) {
        line = "Мышь: USB (xHCI)";  tray = "USB (xHCI)";
    } else if (ps2_mouse_present()) {
        line = "Мышь: PS/2";        tray = "PS/2";
    } else {
        mstat_missing = true;
        /* USB-контроллеры на шине есть, а мышь не нашлась?  Первый
         * символ сводки после «USB PCI: » — буква контроллера
         * (U/O/E/x) или «к» («контроллеров нет»). */
        const char *diag = mouse_hw_usb_diag();
        bool usb_there = diag[9] == 'U' || diag[9] == 'O' ||
                         diag[9] == 'E' || diag[9] == 'x';
        line = usb_there ? "Мышь не найдена (USB есть — Пуск: Терминал)"
                         : "Мышь не найдена (нет USB-UHCI/xHCI и PS/2)";
        tray = "Мышь не найдена";
    }

    int p = 0;
    sput(mstat_line, &p, line);  mstat_line[p] = 0;
    p = 0;
    sput(mstat_tray, &p, tray);
    sput(mstat_tray, &p, cursor_kb_mode ? "  [клав]" : "  [мышь]");
    mstat_tray[p] = 0;
}

/* ── Окна приложений ─────────────────────────────────────────────────── */

typedef enum {
    APP_MYPC = 0,
    APP_CALC,
    APP_CLOCK,
    APP_PAINT,
    APP_EXPLORER,
    APP_NOTEPAD,
    APP_SYSINFO,
    APP_SNAKE,
    APP_EXE_RUNNER,
    APP_TERMINAL,
    APP_COUNT
} app_id_t;

typedef struct {
    int x, y, w, h;
    const char *title;
    bool open;
} dwin_t;

static dwin_t g_apps[APP_COUNT] = {
    [APP_MYPC]       = { 100,  70, 480, 244, "Мой компьютер - Egorov OS", false },
    [APP_CALC]       = { 360,  80, CALC_WIN_W,     CALC_WIN_H,     "Калькулятор",          false },
    [APP_CLOCK]      = { 340, 200, CLOCK_WIN_W,    CLOCK_WIN_H,    "Часы",                 false },
    [APP_PAINT]      = { 240,  60, PAINT_WIN_W,    PAINT_WIN_H,    "Рисование",            false },
    [APP_EXPLORER]   = {  40,  40, EXPLORER_WIN_W, EXPLORER_WIN_H, "Проводник",            false },
    [APP_NOTEPAD]    = { 200, 100, NOTEPAD_WIN_W,  NOTEPAD_WIN_H,  "Блокнот",              false },
    [APP_SYSINFO]    = { 160,  80, SYSINFO_WIN_W,  SYSINFO_WIN_H,  "Системная информация", false },
    [APP_SNAKE]      = { 180,  60, SNAKE_WIN_W,    SNAKE_WIN_H,    "Змейка",               false },
    [APP_EXE_RUNNER] = { 150, 120, EXE_WIN_W,      EXE_WIN_H,      "Запуск программы",     false },
    [APP_TERMINAL]   = { 120,  60, TERMINAL_WIN_W, TERMINAL_WIN_H, "Терминал",             false },
};

static app_id_t g_active = APP_MYPC;

/* Запуск программы из Проводника: номер цели совпадает с app_id первых
 * пяти приложений (см. exe_runner.h). */
static const app_id_t exe_to_app[] = {
    APP_CALC, APP_CLOCK, APP_PAINT, APP_NOTEPAD, APP_SNAKE
};

static void open_app(app_id_t id)  { g_active = id; g_apps[id].open = true; }
static void close_app(app_id_t id) { g_apps[id].open = false; }

/* ── Меню «Пуск» ─────────────────────────────────────────────────────── */

#define SM_WIDTH    236
#define SM_HEADER   52     /* синяя шапка с «пользователем» */
#define SM_ITEM_H   20
#define SM_PAD      4
#define SM_FOOTER   44     /* две нижние строки: Перезагрузка + Выключение */
#define SM_FOOT_ROW 22     /* высота строки футера */
#define SM_ITEMS    13     /* пункты меню; перезагрузка живёт в футере */
#define SM_REBOOT   13     /* хит-тест: строка «Перезагрузка»      */
#define SM_SHUTDOWN 14     /* хит-тест: строка «Завершение работы» */

static const char *sm_labels[SM_ITEMS] = {
    "Мой компьютер",
    "Проводник",
    "Калькулятор",
    "Часы",
    "Рисование",
    "Блокнот",
    "Системная информация",
    "Змейка",
    "Панель управления",
    "Справка и поддержка",
    "О системе",
    "Терминал",
    "Запустить EXE...",
};

static bool start_menu_open = false;
static bool g_hdd = false;

static int sm_height(void) {
    return SM_HEADER + 2 * SM_PAD + SM_ITEMS * SM_ITEM_H + SM_FOOTER;
}

/* Верх пункта i в экранных координатах; одна геометрия и для отрисовки,
 * и для хит-теста — системы координат не могут разъехаться. */
static int sm_item_y(int i) {
    int my = (int)fb.height - GUI_TASKBAR_H - sm_height();
    return my + SM_HEADER + SM_PAD + i * SM_ITEM_H;
}

static int sm_shutdown_y(void) {
    return (int)fb.height - GUI_TASKBAR_H - SM_FOOTER;
}

/* ── Диалоги ─────────────────────────────────────────────────────────── */

static const char *dlg_title;
static const char *dlg_lines[5];
static int         dlg_count;

static void open_dialog(const char *title, const char *l1, const char *l2,
                        const char *l3, const char *l4, const char *l5) {
    const char *ls[5] = { l1, l2, l3, l4, l5 };
    dlg_title = title;
    dlg_count = 0;
    for (int i = 0; i < 5; i++)
        if (ls[i]) dlg_lines[dlg_count++] = ls[i];
}

static void close_dialog(void) { dlg_title = 0; dlg_count = 0; }

static int dlg_w(void)     { return 420; }
static int dlg_h(void)     { return 180; }
static int dlg_x(void)     { return ((int)fb.width  - dlg_w()) / 2; }
static int dlg_y(void)     { return ((int)fb.height - dlg_h()) / 2; }
static int dlg_btn_x(void) { return dlg_x() + (dlg_w() - 100) / 2; }
static int dlg_btn_y(void) { return dlg_y() + dlg_h() - 38; }

/* ── Наведение и нажатие ─────────────────────────────────────────────── */

static struct {
    bool win_close, win_ok, win_prop, win_help;
    bool icon_mypc, icon_trash;
    bool start;
    int  menu_item;                /* 0..SM_ITEMS-1, SM_SHUTDOWN или -1 */
    bool dlg_ok;
    int  calc_btn, paint_tool, notepad_btn;
} hov;

static struct {
    bool win_close, win_ok, win_prop, win_help, dlg_ok;
    int  calc_btn, paint_tool, notepad_btn;
} prs;

static bool inside(int mx, int my, int x, int y, int w, int h) {
    return mx >= x && mx < x + w && my >= y && my < y + h;
}

static char tray_clock[12] = {0};
static uint8_t last_sec = 0xFF;

/* ── Фон: небо, солнце, облака, холмы ────────────────────────────────── */

/* Заполненный круг рядами горизонтальных линий; краевой dx ищется без
 * извлечения корня — от края к центру он только уменьшается. */
static void fill_circle(int cx, int cy, int r, uint32_t c) {
    int dx = r;
    for (int dy = 0; dy <= r; dy++) {
        while (dx > 0 && dx * dx + dy * dy > r * r) dx--;
        fb_hline(cx - dx, cy + dy, dx * 2 + 1, c);
        if (dy) fb_hline(cx - dx, cy - dy, dx * 2 + 1, c);
    }
}

/* Плоское облако: три круга и выравнивающая «подошва». */
static void draw_cloud(int x, int y) {
    uint32_t c = rgb(0xF6, 0xF9, 0xFC);
    fill_circle(x,        y,        12, c);
    fill_circle(x + 15,   y - 5,    17, c);
    fill_circle(x + 34,   y,        11, c);
    fb_rect(x - 10, y, 56, 10, c);
}

/* Высота холма в точке x: плавная кривая через опорные высоты pts[0..4]
 * (smoothstep 3t²-2t³ в фиксированной точке, в фиксированной точке).  Дробь 14 бит: при 16 битах t*t
 * переполняет int32 и холмы покрываются острыми зубцами. */
static int hill_height(const int *pts, int x, int w) {
    int seg = (w - 1) / 4;
    if (seg < 1) seg = 1;
    int i = x / seg;
    if (i > 3) i = 3;
    int t  = ((x - i * seg) << 14) / seg;     /* 0..16384 */
    int sq = (t * t) >> 14;
    int cu = (sq * t) >> 14;
    int s  = 3 * sq - 2 * cu;                 /* 0..16384 */
    return pts[i] + (((pts[i + 1] - pts[i]) * s) >> 14);
}

static void draw_background(void) {
    int w = (int)fb.width, h = (int)fb.height;
    int horizon = h * 2 / 3;
    int gh = h - horizon;                     /* высота «земли» */

    /* Небо: глубокая синева к дымке у горизонта. */
    fb_grad_v(0, 0, w, horizon, rgb(0x2F, 0x6F, 0xC4), rgb(0xB4, 0xD9, 0xF2));

    /* Солнце с мягким ореолом. */
    int sun_x = w * 3 / 4, sun_y = h / 8;
    fill_circle(sun_x, sun_y, gh / 6 + 14, rgb(0xD9, 0xEA, 0xF4));
    fill_circle(sun_x, sun_y, gh / 7 + 8,  rgb(0xF3, 0xF2, 0xDE));
    fill_circle(sun_x, sun_y, gh / 10 + 4, rgb(0xFB, 0xF0, 0xCE));
    fill_circle(sun_x, sun_y, gh / 16,     rgb(0xFF, 0xF8, 0xE2));

    draw_cloud(w / 9,     h / 7);
    draw_cloud(w * 2 / 5, h / 12);
    draw_cloud(w * 3 / 5, h / 16);

    /* Холмы: дальний ряд светлый, ближний — сочный.  Каждый рисуется
     * столбиками от своей кромки до низа экрана (по одному fb_vline
     * на столбец), поэтому кадр не зависит от числа пикселей «внутри». */
    static const int FAR[5]  = { 48, 24, 64, 30, 52 };
    static const int NEAR[5] = { 22, 66, 14, 78, 38 };
    int fpts[5], npts[5];
    for (int i = 0; i < 5; i++) {
        fpts[i] = FAR[i]  * gh / 256;
        npts[i] = NEAR[i] * gh / 256;
    }
    for (int x = 0; x < w; x++) {
        int f = hill_height(fpts, x, w);
        fb_vline(x, horizon - f, h - horizon + f, rgb(0x9C, 0xC0, 0x74));
    }
    for (int x = 0; x < w; x++) {
        int top = horizon - hill_height(npts, x, w);
        fb_pixel(x, top, rgb(0x4E, 0x86, 0x38));
        fb_vline(x, top + 1, h - top - 1, rgb(0x67, 0xA4, 0x48));
    }
}

/* ── Значки рабочего стола ───────────────────────────────────────────── */

/* Подпись, центрированная по средней линии значка. */
static void icon_label(int cx, int y, const char *text) {
    text_draw_str_shadow(cx - text_width(text) / 2, y, text, COL_WHITE, COL_BLACK);
}

static void draw_desktop_icons(void) {
    int cx = 46;                              /* центр колонки значков */

    /* «Мой компьютер»: монитор с синим экраном на подставке. */
    if (hov.icon_mypc) fb_rect_outline(8, 8, 76, 78, rgb(0xA0, 0xC0, 0xFF));
    fb_rect(26, 18, 36, 26, rgb(0xD0, 0xD0, 0xD0));
    fb_rect_outline(26, 18, 36, 26, COL_BLACK);
    fb_rect(29, 21, 30, 20, rgb(0x32, 0x6B, 0xCE));
    fb_rect(40, 46, 8, 4,  rgb(0x80, 0x80, 0x80));
    fb_rect(32, 50, 24, 3, rgb(0x60, 0x60, 0x60));
    icon_label(cx, 56, "Мой");
    icon_label(cx, 72, "компьютер");

    /* «Корзина»: бак с крышкой и рёбрами. */
    if (hov.icon_trash) fb_rect_outline(8, 94, 76, 72, rgb(0xA0, 0xC0, 0xFF));
    fb_rect(28, 106, 32, 28, rgb(0xC0, 0xC0, 0xC8));
    fb_rect_outline(28, 106, 32, 28, COL_BLACK);
    fb_rect(26, 100, 36, 7, rgb(0xA0, 0xA0, 0xA8));
    fb_rect_outline(26, 100, 36, 7, COL_BLACK);
    for (int i = 0; i < 3; i++) fb_vline(34 + i * 7, 109, 22, rgb(0x80, 0x80, 0x88));
    icon_label(cx, 138, "Корзина");
}

/* ── Окно приложения и его содержимое ────────────────────────────────── */

static void draw_close_btn(const dwin_t *w) {
    int bx = w->x + w->w - 24, by = w->y + 6;
    uint32_t c = prs.win_close ? rgb(0x90, 0x20, 0x20)
               : (hov.win_close ? rgb(0xE8, 0x68, 0x58) : rgb(0xC8, 0x40, 0x38));
    fb_rect(bx, by, 16, 14, c);
    fb_rect_outline(bx, by, 16, 14, COL_WHITE);
    for (int i = 0; i < 8; i++) {
        fb_pixel(bx + 4 + i,  by + 3 + i,     COL_WHITE);
        fb_pixel(bx + 11 - i, by + 3 + i,     COL_WHITE);
    }
}

static void draw_mypc_body(const dwin_t *w) {
    int bx = w->x + 8, by = w->y + 36;

    fb_rect(bx - 2, by, w->w - 12, 28, COL_TITLE_TOP);
    text_draw_str_shadow(bx + 4, by + 6, "Egorov OS " EGOROV_OS_FULL, COL_WHITE, COL_BLACK);
    by += 34;

    text_draw_str(bx, by, "Системные сведения:", COL_TEXT);

    char buf[80];
    int p = 0;
    sput(buf, &p, "  Разрешение: ");
    char num[16];
    itoa(fb.width, num, 10);  sput(buf, &p, num);
    sput(buf, &p, "x");
    itoa(fb.height, num, 10); sput(buf, &p, num);
    sput(buf, &p, " @ ");
    itoa(fb.bpp, num, 10);    sput(buf, &p, num);
    sput(buf, &p, " bpp");
    buf[p] = 0;
    text_draw_str(bx, by + 18, buf, COL_TEXT);

    text_draw_str(bx, by + 34, "  Архитектура: i386 (32-bit)", COL_TEXT);
    text_draw_str(bx, by + 50, "  Ядро: монолитное, собственное", COL_TEXT);
    text_draw_str(bx, by + 66, "  Загрузчик: GRUB Multiboot 1 / HDD", COL_TEXT);
    text_draw_str(bx, by + 82, "  EXE: PE32 i386 (встроенный загрузчик)", COL_TEXT);

    const char *mode_s = g_hdd ? "  Режим: установленная система (жёсткий диск)"
                               : "  Режим: Live (с носителя)";
    uint32_t mode_c = g_hdd ? rgb(0x10, 0x80, 0x10) : rgb(0xA0, 0x60, 0x10);
    text_draw_str(bx, by + 98, mode_s, mode_c);
    text_draw_str(bx, by + 114, mstat_line, COL_TEXT);

    fb_hline(bx - 2, by + 132, w->w - 12, rgb(0xA0, 0xA0, 0x90));
    gui_button(bx,       by + 140, 100, 24, "ОК",       hov.win_ok,   prs.win_ok);
    gui_button(bx + 110, by + 140, 100, 24, "Свойства", hov.win_prop, prs.win_prop);
    gui_button(bx + 220, by + 140, 100, 24, "Справка",  hov.win_help, prs.win_help);
}

static void draw_active_app(int mouse_x, int mouse_y) {
    dwin_t *w = &g_apps[g_active];
    if (!w->open) return;

    window_t ww = { w->x, w->y, w->w, w->h, w->title };
    gui_window(&ww);

    switch (g_active) {
        case APP_MYPC:       draw_mypc_body(w); break;
        case APP_CALC:       calc_draw(w->x, w->y, hov.calc_btn, prs.calc_btn); break;
        case APP_CLOCK:      clock_app_draw(w->x, w->y); break;
        case APP_PAINT:      paint_draw(w->x, w->y, hov.paint_tool, prs.paint_tool); break;
        case APP_EXPLORER:   explorer_draw(w->x, w->y, mouse_x, mouse_y, false); break;
        case APP_NOTEPAD:    notepad_draw(w->x, w->y, hov.notepad_btn, prs.notepad_btn); break;
        case APP_SYSINFO:    sysinfo_draw(w->x, w->y); break;
        case APP_SNAKE:      snake_draw(w->x, w->y); break;
        case APP_EXE_RUNNER: exe_runner_draw(w->x, w->y); break;
        case APP_TERMINAL:   terminal_draw(w->x, w->y); break;
        default: break;
    }

    draw_close_btn(w);
}

/* ── Меню «Пуск» и диалоги: отрисовка ────────────────────────────────── */

static void draw_start_menu(void) {
    int mw = SM_WIDTH, mh = sm_height();
    int mx = 4, my = (int)fb.height - GUI_TASKBAR_H - mh;

    fb_rect(mx + 4, my + 4, mw, mh, rgb(0, 0, 0));

    fb_grad_v(mx, my, mw, SM_HEADER, rgb(0x2E, 0x6A, 0xC8), rgb(0x07, 0x3D, 0x9F));
    fb_rect(mx + 8, my + 6, 40, 40, rgb(0xFF, 0xCC, 0x80));
    fb_rect_outline(mx + 8, my + 6, 40, 40, COL_WHITE);
    text_draw_str_shadow(mx + 56, my + 12, "Пользователь", COL_WHITE, COL_BLACK);
    text_draw_str_shadow(mx + 56, my + 28, "Egorov OS", rgb(0xC0, 0xD8, 0xFF), COL_BLACK);

    fb_rect(mx, my + SM_HEADER, mw, mh - SM_HEADER - SM_FOOTER, COL_WHITE);

    for (int i = 0; i < SM_ITEMS; i++) {
        int iy = sm_item_y(i);
        bool disabled = (i == 1 && !g_hdd);   /* Проводник только на HDD */
        if (hov.menu_item == i && !disabled) {
            fb_rect(mx + 4, iy, mw - 8, SM_ITEM_H, COL_TITLE_TOP);
            text_draw_str(mx + 14, iy + 2, sm_labels[i], COL_WHITE);
        } else {
            text_draw_str(mx + 14, iy + 2, sm_labels[i],
                          disabled ? rgb(0xA0, 0xA0, 0xA0) : COL_TEXT);
        }
    }

    int fy = my + mh - SM_FOOTER;
    fb_rect(mx, fy, mw, SM_FOOTER, rgb(0xD6, 0xE5, 0xF7));
    fb_hline(mx, fy, mw, rgb(0xB0, 0xC4, 0xDC));
    fb_hline(mx, fy + SM_FOOT_ROW, mw, rgb(0xB0, 0xC4, 0xDC));

    /* «Перезагрузка» — в футере, рядом с выключением:
     * тёплый квадрат с кольцом и стрелкой перезапуска. */
    bool rb_hover = (hov.menu_item == SM_REBOOT);
    if (rb_hover) fb_rect(mx + 4, fy + 3, mw - 8, SM_FOOT_ROW - 6, rgb(0xD8, 0x7A, 0x10));
    int ix = mx + 14, iy2 = fy + 3;
    fb_rect(ix, iy2, 16, 16, rgb(0xC8, 0x50, 0x10));
    fb_rect_outline(ix, iy2, 16, 16, rgb(0x80, 0x30, 0x08));
    fb_rect_outline(ix + 4, iy2 + 4, 8, 8, COL_WHITE);
    fb_hline(ix + 4, iy2 + 8, 10, COL_WHITE);
    fb_pixel(ix + 13, iy2 + 7, COL_WHITE);
    fb_pixel(ix + 13, iy2 + 9, COL_WHITE);
    text_draw_str(mx + 38, fy + 3, "Перезагрузка",
                  rb_hover ? COL_WHITE : COL_TEXT);

    /* «Завершение работы»: красный квадрат, белые кольцо и штрих. */
    bool sd_hover = (hov.menu_item == SM_SHUTDOWN);
    int sy = fy + SM_FOOT_ROW;
    if (sd_hover) fb_rect(mx + 4, sy + 3, mw - 8, SM_FOOT_ROW - 6, rgb(0xC0, 0x10, 0x10));
    fb_rect(ix, sy + 3, 16, 16, rgb(0xC0, 0x10, 0x10));
    fb_rect_outline(ix, sy + 3, 16, 16, rgb(0x80, 0x08, 0x08));
    fb_rect_outline(ix + 4, sy + 7, 8, 8, COL_WHITE);
    fb_vline(ix + 8, sy + 4, 5, COL_WHITE);
    text_draw_str(mx + 38, sy + 3, "Завершение работы",
                  sd_hover ? COL_WHITE : COL_TEXT);
}

static void draw_dialog(void) {
    int dw = dlg_w(), dh = dlg_h();
    int dx = dlg_x(), dy = dlg_y();

    fb_rect(dx + 4, dy + 4, dw, dh, rgb(0, 0, 0));
    fb_rect(dx, dy, dw, dh, COL_WINDOW_BG);
    fb_rect_outline(dx, dy, dw, dh, COL_WINDOW_BORDER);
    fb_rect_outline(dx + 1, dy + 1, dw - 2, dh - 2, COL_WINDOW_BORDER);
    fb_grad_v(dx + 2, dy + 2, dw - 4, 26, COL_TITLE_TOP, COL_TITLE_BOT);
    text_draw_str_shadow(dx + 10, dy + 6, dlg_title, COL_WHITE, COL_BLACK);

    fb_rect(dx + 16, dy + 44, 32, 32, COL_TITLE_TOP);
    fb_rect_outline(dx + 16, dy + 44, 32, 32, COL_WHITE);
    text_draw_str_shadow(dx + 28, dy + 52, "i", COL_WHITE, COL_BLACK);

    for (int i = 0; i < dlg_count; i++)
        text_draw_str(dx + 60, dy + 46 + i * 18, dlg_lines[i], COL_TEXT);

    gui_button(dlg_btn_x(), dlg_btn_y(), 100, 26, "ОК", hov.dlg_ok, prs.dlg_ok);
}

/* ── Полный кадр ─────────────────────────────────────────────────────── */

static void redraw_all(int mouse_x, int mouse_y) {
    mouse_status_refresh();
    draw_background();
    draw_desktop_icons();
    draw_active_app(mouse_x, mouse_y);
    gui_taskbar(fb.width, fb.height, hov.start, tray_clock,
                mstat_tray, mstat_missing);
    if (start_menu_open) draw_start_menu();
    if (dlg_title)       draw_dialog();
    gui_cursor(mouse_x, mouse_y);
}

/* ── Хит-тест ────────────────────────────────────────────────────────── */

static void compute_hover(int mx, int my) {
    hov.win_close = hov.win_ok = hov.win_prop = hov.win_help = false;
    hov.icon_mypc = hov.icon_trash = false;
    hov.start = false;
    hov.menu_item = -1;
    hov.dlg_ok = false;
    hov.calc_btn = hov.paint_tool = hov.notepad_btn = -1;

    if (dlg_title) {
        if (inside(mx, my, dlg_btn_x(), dlg_btn_y(), 100, 26)) hov.dlg_ok = true;
        return;
    }

    dwin_t *aw = &g_apps[g_active];
    if (aw->open) {
        if (inside(mx, my, aw->x + aw->w - 28, aw->y + 4, 24, 18))
            hov.win_close = true;

        if (g_active == APP_MYPC) {
            int bx = aw->x + 8, by = aw->y + 36 + 174;
            if (inside(mx, my, bx,       by, 100, 24)) hov.win_ok   = true;
            if (inside(mx, my, bx + 110, by, 100, 24)) hov.win_prop = true;
            if (inside(mx, my, bx + 220, by, 100, 24)) hov.win_help = true;
        } else if (g_active == APP_CALC) {
            hov.calc_btn = calc_hit_test(aw->x, aw->y, mx, my);
        } else if (g_active == APP_PAINT) {
            hov.paint_tool = paint_hit_tool(aw->x, aw->y, mx, my);
        } else if (g_active == APP_NOTEPAD) {
            hov.notepad_btn = notepad_hit_btn(aw->x, aw->y, mx, my);
        }
    }

    if (inside(mx, my, 8, 8, 76, 78))  hov.icon_mypc  = true;
    if (inside(mx, my, 8, 94, 76, 72)) hov.icon_trash = true;

    int ty = (int)fb.height - GUI_TASKBAR_H;
    if (inside(mx, my, 4, ty + 2, 100, GUI_TASKBAR_H - 4)) hov.start = true;

    if (start_menu_open) {
        for (int i = 0; i < SM_ITEMS; i++)
            if (inside(mx, my, 8, sm_item_y(i), SM_WIDTH - 8, SM_ITEM_H)) {
                hov.menu_item = i;
                return;
            }
        int fy = sm_shutdown_y();
        if (inside(mx, my, 8, fy, SM_WIDTH - 8, SM_FOOT_ROW))
            hov.menu_item = SM_REBOOT;
        else if (inside(mx, my, 8, fy + SM_FOOT_ROW, SM_WIDTH - 8, SM_FOOT_ROW))
            hov.menu_item = SM_SHUTDOWN;
    }
}

/* ── Обработка клика ─────────────────────────────────────────────────── */

static void handle_click(int mx, int my) {
    if (dlg_title) {
        if (hov.dlg_ok) close_dialog();
        return;
    }

    if (start_menu_open) {
        switch (hov.menu_item) {
            case 0:
                start_menu_open = false; open_app(APP_MYPC); return;
            case 1:  /* Проводник живёт только на установленной системе */
                start_menu_open = false;
                if (g_hdd) { explorer_init(g_hdd); open_app(APP_EXPLORER); }
                else open_dialog("Проводник",
                                 "Проводник доступен только",
                                 "после установки на жёсткий диск.",
                                 "Используйте установщик.", 0, 0);
                return;
            case 2:
                start_menu_open = false; calc_reset(); open_app(APP_CALC); return;
            case 3:
                start_menu_open = false; open_app(APP_CLOCK); return;
            case 4:
                start_menu_open = false; open_app(APP_PAINT); return;
            case 5:
                start_menu_open = false; notepad_init(); open_app(APP_NOTEPAD); return;
            case 6:
                start_menu_open = false; open_app(APP_SYSINFO); return;
            case 7:
                start_menu_open = false; snake_init(); open_app(APP_SNAKE); return;
            case 8:
                start_menu_open = false;
                open_dialog("Панель управления",
                            "Параметры системы:",
                            "  Разрешение: фиксированное (VBE)",
                            "  Звук: нет     Сеть: нет", 0, 0);
                return;
            case 9:
                start_menu_open = false;
                open_dialog("Справка и поддержка",
                            "Egorov OS — демонстрационная ОС.",
                            "Полная справка в README.md",
                            "из архива с исходным кодом.", 0, 0);
                return;
            case 10:
                start_menu_open = false;
                open_dialog("О системе",
                            "Egorov OS " EGOROV_OS_FULL " (2026)",
                            "EXE: встроенный PE32/i386 загрузчик",
                            "Проводник: C:\\Программы\\*.exe",
                            "Мышь: USB (UHCI/xHCI) или PS/2 (опрос).", 0);
                return;
            case 11:
                start_menu_open = false;
                terminal_init();
                open_app(APP_TERMINAL);
                return;
            case 12:
                start_menu_open = false;
                if (g_hdd) {
                    explorer_init(g_hdd);
                    open_app(APP_EXPLORER);
                    open_dialog("Запустить EXE",
                                "Откройте C:\\Программы в Проводнике",
                                "и дважды щёлкните нужный .exe файл.", 0, 0, 0);
                } else {
                    exe_runner_start("demo.exe", EXE_TARGET_UNKNOWN);
                    g_apps[APP_EXE_RUNNER].title = "Запуск программы";
                    open_app(APP_EXE_RUNNER);
                }
                return;
            case SM_REBOOT:
                start_menu_open = false;
                open_dialog("Перезагрузка",
                            "Выполняется перезагрузка...",
                            "ACPI RESET_REG / 0xCF9 / i8042.", 0, 0, 0);
                fb_present();
                power_reboot();
                return;
            case SM_SHUTDOWN:
                start_menu_open = false;
                open_dialog("Завершение работы",
                            "Выполняется завершение работы...",
                            "Компьютер будет выключен.", 0, 0, 0);
                fb_present();
                power_shutdown();
                return;
            default:
                /* Клик мимо пунктов закрывает меню, но не гасит «Пуск» —
                 * повторный клик по кнопке меню не должен его закрывать. */
                if (!hov.start) start_menu_open = false;
                return;
        }
    }

    if (hov.start) { start_menu_open = true; return; }

    dwin_t *aw = &g_apps[g_active];
    if (aw->open) {
        if (hov.win_close) {
            close_app(g_active);
            if (g_active == APP_EXE_RUNNER) explorer_launched_exe = -1;
            return;
        }

        switch (g_active) {
            case APP_MYPC:
                if (hov.win_ok) { close_app(APP_MYPC); return; }
                if (hov.win_prop) {
                    open_dialog("Свойства системы",
                                "Egorov OS " EGOROV_OS_FULL " (2026)",
                                "Архитектура: i386, 32-битное ядро",
                                "EXE: встроенный PE32 i386 загрузчик",
                                "Загрузчик: GRUB Multiboot 1 / HDD BL", 0);
                    return;
                }
                if (hov.win_help) {
                    open_dialog("Справка",
                                "Проводник: дважды щёлкните .exe",
                                "в C:\\Программы для запуска.",
                                "Пуск → Запустить EXE...", 0, 0);
                    return;
                }
                break;

            case APP_CALC:
                if (hov.calc_btn >= 0) { calc_click(hov.calc_btn); return; }
                break;

            case APP_PAINT:
                if (hov.paint_tool >= 0) { paint_tool_click(hov.paint_tool); return; }
                break;

            case APP_EXPLORER:
                explorer_click(aw->x, aw->y, mx, my);
                /* Проводник сообщил о запуске программы — открыть окно. */
                if (explorer_launched_exe >= 0 && explorer_launched_exe < 5) {
                    exe_runner_start(explorer_exe_name,
                                     (exe_target_t)explorer_launched_exe);
                    g_apps[APP_EXE_RUNNER].title = "Запуск программы";
                    open_app(APP_EXE_RUNNER);
                    explorer_launched_exe = -1;
                }
                return;

            case APP_NOTEPAD:
                switch (hov.notepad_btn) {
                    case 0: notepad_clear(); return;
                    case 1: notepad_select_all(); return;
                    case 2:
                        if (g_hdd) notepad_save_hdd();
                        else open_dialog("Сохранить",
                                         "Сохранение доступно только",
                                         "после установки на жёсткий диск.",
                                         "Установите ОС и перезапустите.", 0, 0);
                        return;
                    case 3:
                        if (g_hdd) notepad_open_hdd();
                        else open_dialog("Открыть",
                                         "Открытие файлов доступно только",
                                         "после установки на жёсткий диск.",
                                         "Установите ОС и перезапустите.", 0, 0);
                        return;
                    default:
                        notepad_mouse_click(aw->x, aw->y, mx, my);
                        return;
                }

            case APP_SNAKE:
                snake_click(aw->x, aw->y, mx, my);
                return;

            default:
                break;
        }
    }

    if (hov.icon_mypc) { open_app(APP_MYPC); return; }
    if (hov.icon_trash) {
        if (g_hdd) {
            explorer_init(g_hdd);
            explorer_open_trash();
            open_app(APP_EXPLORER);
        } else {
            open_dialog("Проводник",
                        "Проводник доступен только",
                        "после установки на жёсткий диск.",
                        "Используйте установщик из меню.", 0, 0);
        }
        return;
    }
}

/* ── Главный цикл рабочего стола ─────────────────────────────────────── */

void desktop_run(bool booted_from_hdd) {
    g_hdd = booted_from_hdd;
    mouse_state_t m;
    ps2_poll(&m);

    /* Позиция курсора живёт здесь, между кадрами.  У каждого драйвера
     * своя копия позиции, и без общего счётчика стрелки «откатывались»
     * следующим же кадром.  Драйвер перезаписывает позицию, только когда
     * сам сдвинул указатель (m.moved) и активен режим мыши. */
    int cur_x = m.x, cur_y = m.y;

    /* Если живое указующее устройство уже отвечает — стартуем сразу
     * в режиме мыши; машина вовсе без мыши начинает со стрелок. */
    cursor_kb_mode = !(usb_mouse_active() || mouse_hw_active() ||
                       ps2_mouse_present());

    int  last_x = -1, last_y = -1;
    bool need_redraw = true;
    bool paint_dragging = false;
    uint32_t exe_frame_cnt = 0;

    paint_init();
    calc_reset();
    notepad_set_hdd(g_hdd);
    notepad_init();
    explorer_init(g_hdd);
    snake_init();

    rtc_time_t t;
    rtc_read(&t);
    rtc_format_hms(&t, tray_clock);
    last_sec = t.sec;

    mouse_status_refresh();
    open_dialog("Добро пожаловать в Egorov OS",
                "Egorov OS успешно загружена!",
                g_hdd ? "Режим: система запущена с жёсткого диска."
                      : "Режим: Live (запуск с носителя).",
                mstat_line,
                mouse_hw_usb_diag(),
                0);

    for (;;) {
        ps2_poll(&m);

        /* Пока рабочей мыши нет, mouse_hw_poll следит за портами xHCI/OHCI:
         * мышь можно подключить без перезагрузки.  При активной мыши там же
         * ловится отключение/переподключение (PORT_CHANGE/CCS). */
        if (!usb_mouse_active() && !mouse_hw_active())
            mouse_hw_poll(0);

        /* Мышь дёрнулась — она тут же берёт курсор на себя, каким бы ни
         * был режим: чтобы вернуться к мыши, Ctrl+J не нужен.  Позиция у
         * каждого драйвера своя, поэтому курсор прыгает ровно туда, где
         * мышь реально оказалась. */
        if (m.moved) {
            if (cursor_kb_mode) { cursor_kb_mode = false; need_redraw = true; }
            cur_x = m.x;
            cur_y = m.y;
        }

        m.x = cur_x;
        m.y = cur_y;

        /* Разбираем ВСЕ накопившиеся клавиши кадра: одна клавиша за кадр
         * теряла символы при быстром наборе и позволяла очереди
         * сканкодов переполняться — вместе с байтами пропадали
         * break-коды модификаторов. */
        bool enter_used = false;   /* Enter уже ушёл в диалог/терминал */
        int  key_guard  = 32;
        uint32_t k;
        while (key_guard-- > 0 && ps2_key_pending()) {
            k = ps2_poll_key();
            if (k == 0) continue;      /* служебный байт потока */

            /* «ЭТАП 3»: ОТПУСКАНИЕ стрелки (KEY_BREAK) — только для
             * typematic-трекинга; приложения об этом событии не знают. */
            if (k & KEY_BREAK) {
                if (k == (KEY_UP | KEY_BREAK))    s_arr_u = false;
                else if (k == (KEY_DOWN | KEY_BREAK))  s_arr_d = false;
                else if (k == (KEY_LEFT | KEY_BREAK))  s_arr_l = false;
                else if (k == (KEY_RIGHT | KEY_BREAK)) s_arr_r = false;
                continue;
            }

            /* Стрелки в режиме клавиатуры двигают курсор с шагом 8 px.
             * «ЭТАП 3»: НАЖАТИЕ засчитывается только если стрелка НЕ
             * была зажата (иначе аппаратный PS/2-typematic двигал бы
             * курсор и событием, и кадром одновременно); зажатие
             * запоминается — дальше движение делает typematic-блок
             * ниже (для USB-клавиатур без собственного повтора). */
            bool kb_arrow = cursor_kb_mode &&
                (k == KEY_LEFT || k == KEY_RIGHT || k == KEY_UP || k == KEY_DOWN);
            if (kb_arrow) {
                bool fresh = true;
                if      (k == KEY_LEFT)  { fresh = !s_arr_l; s_arr_l = true; }
                else if (k == KEY_RIGHT) { fresh = !s_arr_r; s_arr_r = true; }
                else if (k == KEY_UP)    { fresh = !s_arr_u; s_arr_u = true; }
                else if (k == KEY_DOWN)  { fresh = !s_arr_d; s_arr_d = true; }
                if (fresh) {
                    int nx = cur_x, ny = cur_y;
                    if      (k == KEY_LEFT)  nx -= 8;
                    else if (k == KEY_RIGHT) nx += 8;
                    else if (k == KEY_UP)    ny -= 8;
                    else if (k == KEY_DOWN)  ny += 8;
                    if (nx < 0) nx = 0;
                    if (ny < 0) ny = 0;
                    if (nx > (int)fb.width  - 1) nx = (int)fb.width  - 1;
                    if (ny > (int)fb.height - 1) ny = (int)fb.height - 1;
                    cur_x = nx;
                    cur_y = ny;
                    need_redraw = true;
                }
                continue;
            }

            if (k == KEY_CTRL_J) {
                cursor_kb_mode = !cursor_kb_mode;
                arr_held_clear();     /* смена режима сбрасывает typematic */
                need_redraw = true;
            } else if (k == KEY_ESC) {
                if (dlg_title)                  close_dialog();
                else if (start_menu_open)       start_menu_open = false;
                else if (g_apps[g_active].open) close_app(g_active);
                need_redraw = true;
            } else if (k == KEY_ENTER) {
                if (dlg_title) {
                    close_dialog();
                    enter_used = true;     /* без синтетического клика */
                } else if (g_apps[g_active].open && g_active == APP_TERMINAL) {
                    terminal_key(k);
                    enter_used = true;
                } else if (g_apps[g_active].open && g_active == APP_NOTEPAD) {
                    notepad_key(k);        /* Enter = новая строка */
                    enter_used = true;
                }
                need_redraw = true;
            } else if (k == KEY_BS) {
                if (!dlg_title) {
                    if (g_apps[g_active].open && g_active == APP_NOTEPAD)
                        notepad_key(k);
                    else if (g_apps[g_active].open && g_active == APP_TERMINAL)
                        terminal_key(k);
                }
                need_redraw = true;
            } else if (!dlg_title) {
                bool open = g_apps[g_active].open;
                if (open && g_active == APP_CALC)          calc_key(k);
                else if (open && g_active == APP_NOTEPAD)  notepad_key(k);
                else if (open && g_active == APP_SNAKE)    snake_key(k);
                else if (open && g_active == APP_TERMINAL) terminal_key(k);
                need_redraw = true;
            }
        }

        /* Enter в режиме клавиатуры работает как зажатая левая кнопка:
         * нажатие — клик, удержание — кнопка остаётся нажатой (перо в
         * Paint не поднимается), отпускание — поднята.  Если Enter уже
         * ушёл в диалог/терминал/блокнот, синтетического клика нет. */
        {
            static bool enter_arm = true;
            bool enter_now = ps2_enter_down();
            if (!enter_now) enter_arm = true;
            if (enter_now && cursor_kb_mode && !enter_used) {
                if (enter_arm) { m.clicked_left = true; enter_arm = false; }
                m.left = true;
            }
        }

        /* ══ «ЭТАП 3»: typematic — зажатая стрелка ведёт курсор ═════════
         * Пока стрелка удерживается (и активен режим клавиатуры после
         * Ctrl+J), курсор движется КАЖДЫЙ кадр — как с PS/2-клавиатурой
         * в v0.3.0 («зажал — мышь едет, отпустил — остановилась»), но
         * теперь и с USB-клавиатурами: программный auto-repeat.  Шаг
         * Разгоняется от 4 до 16 px; кадр = 15 мс (темпинг в конце
         * цикла).  Начальная задержка 15 кадров ≈ 230 мс — точность
         * одиночного нажатия не страдает. */
        if (cursor_kb_mode && (s_arr_l || s_arr_r || s_arr_u || s_arr_d)) {
            s_arr_hold++;
            if (s_arr_hold > 15) {
                if (!s_typ_logged) {
                    s_typ_logged = true;
                    serial_puts("KBD-CURSOR: стрелка УДЕРЖАНА - typematic: курсор едет, пока не отпустишь\n");
                }
                int step = 4 + (int)((s_arr_hold - 15u) >> 4);
                if (step > 16) step = 16;
                int nx = cur_x, ny = cur_y;
                if (s_arr_l) nx -= step;
                if (s_arr_r) nx += step;
                if (s_arr_u) ny -= step;
                if (s_arr_d) ny += step;
                if (nx < 0) nx = 0;
                if (ny < 0) ny = 0;
                if (nx > (int)fb.width  - 1) nx = (int)fb.width  - 1;
                if (ny > (int)fb.height - 1) ny = (int)fb.height - 1;
                if (nx != cur_x || ny != cur_y) {
                    cur_x = nx;
                    cur_y = ny;
                    need_redraw = true;
                }
            }
        } else {
            /* стрелки отпущены (или не зажимались): если было движение —
             * ОДНОКРАТНЫЙ маркер остановки в serial-журнал (e2e-тест). */
            if (s_arr_hold > 15) {
                serial_puts("KBD-CURSOR: стрелка отпущена - курсор остановился\n");
                typ_logged_reset();
            }
            s_arr_hold = 0;
        }

        if (!dlg_title && g_apps[g_active].open && g_active == APP_SNAKE) {
            snake_tick();
            need_redraw = true;
        }

        if (g_apps[APP_EXE_RUNNER].open && g_active == APP_EXE_RUNNER) {
            exe_frame_cnt++;
            if ((exe_frame_cnt & 3) == 0) exe_runner_tick();
            need_redraw = true;

            if (exe_runner_done()) {
                exe_target_t tgt = exe_runner_get_target();
                close_app(APP_EXE_RUNNER);
                if (tgt < EXE_TARGET_UNKNOWN) {
                    app_id_t target_app = exe_to_app[(int)tgt];
                    if (target_app == APP_CALC)    calc_reset();
                    if (target_app == APP_NOTEPAD) notepad_init();
                    if (target_app == APP_SNAKE)   snake_init();
                    open_app(target_app);
                } else {
                    g_active = APP_MYPC;
                    g_apps[APP_MYPC].open = true;
                }
                need_redraw = true;
            }
        }

        compute_hover(m.x, m.y);

        prs.win_close   = m.left && hov.win_close;
        prs.win_ok      = m.left && hov.win_ok;
        prs.win_prop    = m.left && hov.win_prop;
        prs.win_help    = m.left && hov.win_help;
        prs.dlg_ok      = m.left && hov.dlg_ok;
        prs.calc_btn    = (m.left && hov.calc_btn   >= 0) ? hov.calc_btn   : -1;
        prs.paint_tool  = (m.left && hov.paint_tool >= 0) ? hov.paint_tool : -1;
        prs.notepad_btn = (m.left && hov.notepad_btn >= 0) ? hov.notepad_btn : -1;

        if (m.clicked_left) { handle_click(m.x, m.y); need_redraw = true; }

        /* Холст Paint: рисование при зажатой кнопке, линия Брезенхэмом. */
        if (!dlg_title && g_apps[g_active].open && g_active == APP_PAINT) {
            dwin_t *pw = &g_apps[APP_PAINT];
            if (m.left && paint_in_canvas(pw->x, pw->y, m.x, m.y)) {
                paint_paint(pw->x, pw->y, m.x, m.y, paint_dragging);
                paint_dragging = true;
                need_redraw = true;
            } else {
                if (paint_dragging) paint_release();
                paint_dragging = false;
            }
        }

        if (m.x != last_x || m.y != last_y) {
            need_redraw = true;
            last_x = m.x;
            last_y = m.y;
        }

        /* rtc_read ждёт конца UIP (до 100000 чтений порта) - раз в
         * секунду это до ~100 мс лага курсора на реальном железе.
         * Читаем раз в 64 кадра (~раз в секунду при 60 fps). */
        static uint32_t rtc_div = 0;
        if (!rtc_div) { rtc_div = 64; rtc_read(&t); }
        else if (--rtc_div == 0) { rtc_div = 64; rtc_read(&t); }
        if (t.sec != last_sec) {
            rtc_format_hms(&t, tray_clock);
            last_sec = t.sec;
            need_redraw = true;
        }

        if (need_redraw) {
            /* [#313]: полная перерисовка в backbuf ОСТАЁТСЯ (это кэш-
             * теплая ОЗУ, дёшево), а дорогая часть - запись в VRAM -
             * теперь минимальна: fb_present (shadow-present, fb.c)
             * копирует только изменившиеся строки. */
            redraw_all(m.x, m.y);
            fb_present();
            need_redraw = false;
        }

        /* «ЭТАП 3»: темпинг кадра при зажатых стрелках — цикл спит 15 мс,
         * typematic-движение получает равномерные ~66 кадров/с (и CPU не
         * жгётся впустую).
         * «ЭТАП 5» (G3): сон расширен на УДЕРЖАНИЕ ЛЮБОЙ USB-клавиши
         * (ps2_typematic_held) - софт-автоповтор из ps2.c отсчитывает
         * кадры по ps2_poll(), и без выравнивания его скорость зависела
         * бы от скорости пустого цикла (на реальном железе - тысячи
         * итераций/с, повторы слились бы в поток). PS/2-клавиатуры сюда
         * не попадают: у них автоповтор аппаратный, tm-состояние пусто,
         * цикл крутится как раньше. */
        if ((cursor_kb_mode && (s_arr_l || s_arr_r || s_arr_u || s_arr_d)) ||
            ps2_typematic_held())
            hal_msleep(15);

        /* [#313-смежное] лёгкая пауза конвейера на каждой итерации:
         * снижает энергопотребление и паразитную нагрузку на шину от
         * пустого спина (потоки чтения PORTSC/MMIO не вжаривают PCI
         * непрерывно). Именно pause, а НЕ hlt: IRQ в ОС замаскированы,
         * hlt повесил бы машину до сброса. */
        __asm__ __volatile__("pause");
    }
}
