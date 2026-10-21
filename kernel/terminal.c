/* Терминал для Egorov OS — с чёрным фоном и командами (регистронезависимый)
 *
 * Глубокий аудит #224 (G4): границы проверены — ввод ограничен
 * TERMINAL_MAX_INPUT-1; строки журнала (g_lines[1024][80]) режутся до
 * 79 байт + NUL в terminal_add_line; cmd_upper[80] копирует <= 79;
 * argv-буферов нет (команды разбираются strncmp по литералу «ECHO »);
 * COREINFO-вывод держится в out[80] через room = 79 - p; DATE-буфер
 * 32 байта при максимуме ~20.  Реальных out-of-bounds НЕ найдено. */
#include "terminal.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "ps2.h"
#include "string.h"
#include "rtc.h"
#include "klog.h"
#include "io.h"

#define TERMINAL_MAX_LINES 1024
#define TERMINAL_MAX_INPUT  256
#define TERMINAL_BG         rgb(0,0,0)
#define TERMINAL_FG         rgb(0xFF,0xFF,0xFF)
#define TERMINAL_PROMPT     rgb(0x00,0xFF,0x00)

static char g_lines[TERMINAL_MAX_LINES][80];
static int  g_line_count = 0;
static int  g_scroll = 0;          /* индекс первой отображаемой строки */
static bool g_auto_scroll = true;  /* автоматически листать вниз при новых строках */

static char g_input[TERMINAL_MAX_INPUT];
static int  g_input_len = 0;

void terminal_add_line(const char *s);

/* ── Приведение строки к верхнему регистру ── */
static void str_to_upper(char *s) {
    for (int i = 0; s[i]; i++) {
        if (s[i] >= 'a' && s[i] <= 'z') {
            s[i] = s[i] - 'a' + 'A';
        }
    }
}

/* ── Инициализация ── */
void terminal_init(void) {
    g_line_count = 0;
    g_input_len = 0;
    g_scroll = 0;
    g_auto_scroll = true;
    terminal_add_line("=== Egorov OS Terminal v1.0 ===");
    terminal_add_line("Введите 'HELP' для списка команд.");
    terminal_add_line("-----------------------------------");
}

/* ── Добавить строку в историю ── */
void terminal_add_line(const char *s) {
    int old_count = g_line_count;

    int i = 0;
    while (s[i] && i < 79) {
        g_lines[g_line_count][i] = s[i];
        i++;
    }
    g_lines[g_line_count][i] = 0;
    if (g_line_count < TERMINAL_MAX_LINES - 1) {
        g_line_count++;
    } else {
        // сдвиг всех строк вверх (если переполнение)
        for (int j = 1; j < TERMINAL_MAX_LINES; j++) {
            memcpy(g_lines[j-1], g_lines[j], 80);
        }
        g_line_count = TERMINAL_MAX_LINES - 1;
    }

    // Если автопрокрутка включена, или мы были внизу (показывали последнюю строку),
    // то сдвигаем g_scroll вниз, чтобы видеть новые строки.
    // Используем условие: если g_scroll был равен old_count - 1 (т.е. показывал самую последнюю строку),
    // то увеличиваем его на 1 (чтобы показывать новую последнюю).
    // Если пользователь прокрутил вверх, g_scroll < old_count - 1, и мы оставляем его на месте.
    if (g_auto_scroll || g_scroll == old_count - 1) {
        g_scroll = g_line_count - 1;   // всегда показываем последнюю строку (внизу)
    }
    // Убедимся, что g_scroll не выходит за границы
    if (g_scroll < 0) g_scroll = 0;
    if (g_scroll >= g_line_count) g_scroll = g_line_count - 1;
}

/* ── Обработка команд (регистронезависимая) ── */
static void terminal_process_command(const char *cmd) {
    char cmd_upper[80];
    int i;
    for (i = 0; cmd[i] && i < 79; i++) {
        char c = cmd[i];
        if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
        cmd_upper[i] = c;
    }
    cmd_upper[i] = 0;

    // HELP
    if (strcmp(cmd_upper, "HELP") == 0) {
        terminal_add_line("Доступные команды:");
        terminal_add_line("  HELP          - показать эту справку");
        terminal_add_line("  CLEAR         - очистить экран");
        terminal_add_line("  ECHO <текст>  - вывести текст");
        terminal_add_line("  DATE          - показать текущую дату и время");
        terminal_add_line("  MEM           - показать информацию о памяти");
        terminal_add_line("  COREINFO      - журнал ядра (лог COM1)");
        terminal_add_line("  EXIT          - закрыть терминал");
        return;
    }
    // CLEAR
    if (strcmp(cmd_upper, "CLEAR") == 0) {
        g_line_count = 0;
        g_scroll = 0;
        g_auto_scroll = true;
        terminal_add_line("Экран очищен.");
        terminal_add_line("-----------------------------------");
        return;
    }
    // ECHO
    if (strncmp(cmd_upper, "ECHO ", 5) == 0) {
        terminal_add_line(cmd + 5); // выводим оригинальный текст (сохраняем регистр)
        return;
    }
    // DATE
    if (strcmp(cmd_upper, "DATE") == 0) {
        rtc_time_t t;
        rtc_read(&t);
        char buf[32];
        int p = 0;
        char tmp[8];
        itoa(t.hour, tmp, 10);
        if (t.hour < 10) buf[p++] = '0';
        for (int i = 0; tmp[i]; i++) buf[p++] = tmp[i];
        buf[p++] = ':';
        itoa(t.min, tmp, 10);
        if (t.min < 10) buf[p++] = '0';
        for (int i = 0; tmp[i]; i++) buf[p++] = tmp[i];
        buf[p++] = ':';
        itoa(t.sec, tmp, 10);
        if (t.sec < 10) buf[p++] = '0';
        for (int i = 0; tmp[i]; i++) buf[p++] = tmp[i];
        buf[p++] = ' ';
        itoa(t.day, tmp, 10);
        if (t.day < 10) buf[p++] = '0';
        for (int i = 0; tmp[i]; i++) buf[p++] = tmp[i];
        buf[p++] = '.';
        itoa(t.month, tmp, 10);
        if (t.month < 10) buf[p++] = '0';
        for (int i = 0; tmp[i]; i++) buf[p++] = tmp[i];
        buf[p++] = '.';
        itoa(t.year, tmp, 10);
        for (int i = 0; tmp[i]; i++) buf[p++] = tmp[i];
        buf[p] = 0;
        terminal_add_line(buf);
        return;
    }
    // MEM
    if (strcmp(cmd_upper, "MEM") == 0) {
        terminal_add_line("Память: ~16 МБ (эмуляция)");
        terminal_add_line("Свободно: ~14 МБ");
        return;
    }
    // EXIT
    if (strcmp(cmd_upper, "EXIT") == 0) {
        terminal_add_line("Завершение работы терминала...");
        terminal_add_line("Закройте окно крестиком или ESC.");
        return;
    }
    // COREINFO: журнал ядра — зеркало serial-вывода (COM1) в ОЗУ.
    // Печатает klog-кольцо построчно, с номерами; длинные строки
    // переносятся с отступом.  Замена удалённого окна «Журнал ядра».
    if (strcmp(cmd_upper, "COREINFO") == 0) {
        static char kbuf[4096];
        static char kbuf2[4096];   /* хвост близнеца для сверки */
        uint32_t total = klog_total();
        uint32_t mtot  = klog_mirror_total();   /* близнец */
        if (total < 16 && mtot < 16) {
            terminal_add_line("Журнал пуст: serial-вывода не было.");
            return;
        }
        /* сверка колец байт-в-байт. В здоровой системе кольца
         * идентичны: klog_putc пишет каждый байт в оба сразу.  Разница
         * счётчиков или содержимого = одно из колец затёрто (бродячий
         * DMA; симптом реального ПК: «1: 2» при total=2713 — счётчик
         * уцелел, буфер нет; старая проверка total<16 это пропускала).
         * Тогда показываем близнец-зеркало: оно лежит в «жертвенной»
         * зоне в самом конце .bss, за 3 МБ fb_backbuf. */
        klog_read_tail(kbuf, (int)sizeof(kbuf));
        klog_mirror_tail(kbuf2, (int)sizeof(kbuf2));
        int same = (total == mtot);
        if (same) {
            uint32_t n = total;
            if (n > sizeof(kbuf) - 1u) n = sizeof(kbuf) - 1u;
            for (uint32_t i = 0; i < n; i++) {
                if (kbuf[i] != kbuf2[i]) { same = 0; break; }
            }
        }
        if (!same) {
            terminal_add_line("!!! Кольца журнала РАЗЛИЧАЮТСЯ: одно затёрто (DMA?).");
            terminal_add_line("!!! Показываю близнец-зеркало (.klogsec):");
            for (int i = 0; i < (int)sizeof(kbuf); i++) kbuf[i] = kbuf2[i];
            total = mtot;
        }
        int klen = 0;
        while (kbuf[klen]) klen++;
        {
            char hdr[80];
            const char *s1 = (total >= sizeof(kbuf))
                             ? "Журнал ядра (хвост), всего "
                             : "Журнал ядра, всего ";
            int p = 0;
            while (*s1) hdr[p++] = *s1++;
            char tmp[12];
            itoa(total, tmp, 10);
            for (int i = 0; tmp[i] && p < 70; i++) hdr[p++] = tmp[i];
            const char *s2 = " байт";
            while (*s2 && p < 74) hdr[p++] = *s2++;
            hdr[p] = 0;
            terminal_add_line(hdr);
        }
        // Построчно, с номерами; длинные строки переносятся с отступом
        int lineno = 0, seg = 0;
        for (int i = 0; i <= klen; i++) {
            if (kbuf[i] == '\n' || kbuf[i] == 0) {
                lineno++;
                int pos = seg, end = i, part = 0;
                do {
                    char out[80];
                    int p = 0;
                    if (part == 0) {
                        char tmp[12];
                        itoa(lineno, tmp, 10);
                        for (int j = 0; tmp[j]; j++) out[p++] = tmp[j];
                        out[p++] = ':';
                        out[p++] = ' ';
                    } else {
                        out[p++] = ' '; out[p++] = ' ';
                        out[p++] = ' '; out[p++] = ' ';
                    }
                    part++;
                    int room = 79 - p;
                    int n = end - pos;
                    if (n > room) n = room;
                    for (int j = 0; j < n; j++) out[p++] = kbuf[pos + j];
                    out[p] = 0;
                    terminal_add_line(out);
                    pos += n;
                } while (pos < end);
                seg = i + 1;
            }
        }
        return;
    }
    // Неизвестная команда
    char buf[80];
    int p = 0;
    const char *s = "Неизвестная команда: ";
    while (s[p]) { buf[p] = s[p]; p++; }
    int j = 0;
    while (cmd[j] && p < 78) { buf[p++] = cmd[j++]; }
    buf[p] = 0;
    terminal_add_line(buf);
}

/* ── Отрисовка ── */
void terminal_draw(int wx, int wy) {
    int ww = TERMINAL_WIN_W;
    int wh = TERMINAL_WIN_H;

    // Чёрный фон
    fb_rect(wx + 2, wy + 28, ww - 4, wh - 30, TERMINAL_BG);

    // Вычисляем высоту, доступную для текста (отступ сверху 4, снизу оставляем место для поля ввода)
    int top_offset = 4;
    int input_height = FONT_GLYPH_H + 10; // высота поля ввода с отступами
    int text_height = (wh - 30) - top_offset - input_height - 8; // 8 - нижний отступ
    if (text_height < 0) text_height = 0;
    int max_lines = text_height / FONT_GLYPH_H;
    if (max_lines < 1) max_lines = 1;

    // Корректируем g_scroll, чтобы он не выходил за границы
    if (g_scroll > g_line_count - max_lines) {
        g_scroll = g_line_count - max_lines;
        if (g_scroll < 0) g_scroll = 0;
    }
    if (g_scroll < 0) g_scroll = 0;

    int start = g_scroll;
    int cy = wy + 32 + top_offset; // смещение сверху
    for (int i = start; i < g_line_count && i - start < max_lines; i++) {
        text_draw_str(wx + 8, cy, g_lines[i], TERMINAL_FG);
        cy += FONT_GLYPH_H;
    }

    // Поле ввода (чёрное с зелёным приглашением) — рисуем внизу
    int py = wy + wh - input_height - 4;
	unsigned y_min = (unsigned)(wy + 28);
	if ((unsigned)py < y_min) py = (int)y_min;


    fb_rect(wx + 6, py, ww - 12, input_height, rgb(0x10,0x10,0x10));
    fb_rect_outline(wx + 6, py, ww - 12, input_height, rgb(0x40,0x40,0x40));

    char prompt[TERMINAL_MAX_INPUT + 4];
    prompt[0] = '>';
    prompt[1] = ' ';
    int i;
    for (i = 0; i < g_input_len && i < TERMINAL_MAX_INPUT - 2; i++) {
        prompt[i+2] = g_input[i];
    }
    prompt[i+2] = 0;
    text_draw_str(wx + 10, py + 4, prompt, TERMINAL_PROMPT);
}

/* ── Обработка клавиш ── */
void terminal_key(uint32_t k) {
    // Вычисляем количество видимых строк (нужно для прокрутки)
    int top_offset = 4;
    int input_height = FONT_GLYPH_H + 10;
    int text_height = (TERMINAL_WIN_H - 30) - top_offset - input_height - 8;
    if (text_height < 0) text_height = 0;
    int max_lines = text_height / FONT_GLYPH_H;
    if (max_lines < 1) max_lines = 1;

    // PageUp / PageDown
    if (k == KEY_PGUP) {
        g_auto_scroll = false;
        g_scroll -= 2;   /* PgUp — на 2 строки вверх */
        if (g_scroll < 0) g_scroll = 0;
        return;
    }
    if (k == KEY_PGDN) {
        g_auto_scroll = false;
        g_scroll += 2;   /* PgDn — на 2 строки вниз */
        int max_start = g_line_count - max_lines;
        if (g_scroll > max_start) g_scroll = max_start;
        if (g_scroll < 0) g_scroll = 0;
        // Если дошли до низа, включаем автопрокрутку
        if (g_scroll >= g_line_count - max_lines) {
            g_auto_scroll = true;
        }
        return;
    }
    // Стрелки вверх/вниз для построчной прокрутки
    if (k == KEY_UP) {
        g_auto_scroll = false;
        if (g_scroll > 0) g_scroll--;
        return;
    }
    if (k == KEY_DOWN) {
        g_auto_scroll = false;
        if (g_scroll < g_line_count - max_lines) g_scroll++;
        if (g_scroll >= g_line_count - max_lines) {
            g_auto_scroll = true;
        }
        return;
    }
    // Клавиша End – перейти вниз и включить автопрокрутку
    if (k == KEY_END) {
        g_auto_scroll = true;
        g_scroll = g_line_count - max_lines;
        if (g_scroll < 0) g_scroll = 0;
        return;
    }

    // Обработка ввода команд
    if (k == KEY_ENTER) {
        if (g_input_len > 0) {
            char line[80];
            int i;
            for (i = 0; i < g_input_len && i < 78; i++) {
                line[i] = g_input[i];
            }
            line[i] = 0;
            terminal_add_line(line);
            terminal_process_command(line);
        } else {
            terminal_add_line("> ");
        }
        g_input_len = 0;
        g_input[0] = 0;
        // После ввода команды включаем автопрокрутку
        g_auto_scroll = true;
        return;
    }
    if (k == KEY_BS) {
        if (g_input_len > 0) {
            g_input_len--;
            g_input[g_input_len] = 0;
        }
        return;
    }
    if (k >= 0x20 && k < 0x7F) {
        if (g_input_len < TERMINAL_MAX_INPUT - 1) {
            g_input[g_input_len++] = (char)k;
            g_input[g_input_len] = 0;
        }
        return;
    }
}

void terminal_click(int wx, int wy, int mx, int my) {
    (void)wx; (void)wy; (void)mx; (void)my;
}
