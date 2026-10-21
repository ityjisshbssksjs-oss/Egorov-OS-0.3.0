/* Калькулятор Egorov OS — десятичные дроби на BCD-массивах.
 *
 * Считаем в фиксированной точке: число = массив цифр (младшие вперёд)
 * со шкалой 10^-FRAC.  Никакого FPU и libgcc: сложение и вычитание —
 * столбиком, умножение — столбиком с двойным запасом разрядов, деление —
 * «уголком» (школьный алгоритм с подбором цифры).  Переполнение ловим
 * по старшим разрядам и честно показываем «Переполнение».
 *
 * 5 rows x 4 cols layout:
 *   0:  C    ±    ⌫    /
 *   1:  7    8    9    *
 *   2:  4    5    6    -
 *   3:  1    2    3    +
 *   4:  0    .    =    (пусто)
 */
#include "types.h"
#include "calc.h"
#include "fb.h"
#include "text.h"
#include "font.h"
#include "gui.h"
#include "string.h"

static const char *btn_label[20] = {
    "C", "+/-", "<-", "/",
    "7", "8", "9", "*",
    "4", "5", "6", "-",
    "1", "2", "3", "+",
    "0", ".", "=", " "
};

#define BTN_W 60
#define BTN_H 38
#define BTN_GAP 4
#define DISP_H 36
#define INNER_PAD 8

/* ── Десятичное число ─────────────────────────────────────────────────────── */

#define FRAC 4                    /* знаков после запятой                    */
#define NINT 9                    /* знаков до запятой при вводе             */
#define NDIG (NINT + FRAC + 1)    /* всего разрядов: 9 целых + 4 дробных
                                   * + 1 запасной на перенос                */

typedef struct {
    uint8_t d[NDIG];              /* цифры 0..9, d[0] — самый младший        */
    bool    neg;
    bool    ovf;                  /* результат не влез в разрядную сетку     */
} num_t;

static void num_zero(num_t *n) {
    for (int i = 0; i < NDIG; i++) n->d[i] = 0;
    n->neg = false;
    n->ovf = false;
}

static bool num_is_zero(const num_t *n) {
    for (int i = 0; i < NDIG; i++)
        if (n->d[i]) return false;
    return true;
}

/* |a| против |b|: +1, 0, -1 */
static int num_cmp_abs(const num_t *a, const num_t *b) {
    for (int i = NDIG - 1; i >= 0; i--) {
        if (a->d[i] != b->d[i]) return a->d[i] > b->d[i] ? 1 : -1;
    }
    return 0;
}

/* r = |a| + |b| (перенос за старший разряд = переполнение) */
static void num_add_abs(num_t *r, const num_t *a, const num_t *b) {
    int carry = 0;
    for (int i = 0; i < NDIG; i++) {
        int t = a->d[i] + b->d[i] + carry;
        r->d[i] = (uint8_t)(t % 10);
        carry   = t / 10;
    }
    r->neg = false;
    r->ovf = carry != 0;
}

/* r = a - b, предполагаем |a| >= |b| (заём за границу не возникает) */
static void num_sub_abs(num_t *r, const num_t *a, const num_t *b) {
    int borrow = 0;
    for (int i = 0; i < NDIG; i++) {
        int t = a->d[i] - b->d[i] - borrow;
        if (t < 0) { t += 10; borrow = 1; } else borrow = 0;
        r->d[i] = (uint8_t)t;
    }
    r->neg = false;
    r->ovf = false;
}

/* r = a + b со знаками */
static void num_add(num_t *r, const num_t *a, const num_t *b) {
    if (a->neg == b->neg) {
        num_add_abs(r, a, b);
        r->neg = a->neg;
        if (num_is_zero(r)) r->neg = false;
        return;
    }
    /* разные знаки: из большего по модулю вычитаем меньший */
    int c = num_cmp_abs(a, b);
    if (c >= 0) {
        num_sub_abs(r, a, b);
        r->neg = a->neg;
    } else {
        num_sub_abs(r, b, a);
        r->neg = b->neg;
    }
    if (num_is_zero(r)) r->neg = false;
}

/* r = a * b.  Произведение двух NDIG-разрядных чисел держим в двойном
 * массиве, потом сдвигаем на FRAC разрядов (у обоих множителей шкала
 * 10^-FRAC) и округляем по первой отброшенной цифре. */
static void num_mul(num_t *r, const num_t *a, const num_t *b) {
    uint8_t acc[2 * NDIG];
    for (int i = 0; i < 2 * NDIG; i++) acc[i] = 0;

    for (int i = 0; i < NDIG; i++) {
        if (!a->d[i]) continue;
        int carry = 0;
        int j = 0;
        for (; j < NDIG; j++) {
            int t = acc[i + j] + a->d[i] * b->d[j] + carry;
            acc[i + j] = (uint8_t)(t % 10);
            carry      = t / 10;
        }
        for (; i + j < 2 * NDIG && carry; j++) {   /* хвост переноса */
            int t = acc[i + j] + carry;
            acc[i + j] = (uint8_t)(t % 10);
            carry      = t / 10;
        }
    }

    num_zero(r);
    /* отброшенная часть начинается с acc[FRAC-1] — округляем */
    bool round_up = acc[FRAC - 1] >= 5;
    for (int i = 0; i < NDIG; i++) r->d[i] = acc[i + FRAC];
    /* всё, что не поместилось в старшие разряды — переполнение */
    for (int i = FRAC + NDIG; i < 2 * NDIG; i++)
        if (acc[i]) r->ovf = true;
    if (round_up) {
        int carry = 1;
        for (int i = 0; i < NDIG && carry; i++) {
            int t = r->d[i] + carry;
            r->d[i] = (uint8_t)(t % 10);
            carry   = t / 10;
        }
        if (carry) r->ovf = true;
    }
    r->neg = a->neg != b->neg;
    if (num_is_zero(r)) r->neg = false;
}

/* r = a / b «уголком».  Делим |a| с запасом 2*FRAC нулей: лишние цифры
 * частного — страховка для округления по четвёртой отброшенной. */
static void num_div(num_t *r, const num_t *a, const num_t *b) {
    #define DDIG (NDIG + 2 * FRAC)            /* 14 + 8 = 22                 */
    uint8_t dividend[DDIG];
    uint8_t divisor[DDIG];
    uint8_t rem[DDIG];
    for (int i = 0; i < DDIG; i++) {
        dividend[i] = 0;
        divisor[i]  = 0;
        rem[i]      = 0;
    }
    for (int i = 0; i < NDIG; i++) {
        dividend[i + 2 * FRAC] = a->d[i];     /* делим |a|*10^8             */
        divisor[i]             = b->d[i];
    }

    num_zero(r);
    r->neg = a->neg != b->neg;

    bool round_up = false;
    for (int pos = DDIG - 1; pos >= 0; pos--) {
        /* rem = rem * 10 + очередная цифра делимого: десятичный сдвиг
         * делаем ВСЕГДА (гейт по carry ломал сдвиг при цифре 0) */
        int carry = dividend[pos];
        for (int i = 0; i < DDIG; i++) {
            int t = rem[i] * 10 + carry;
            rem[i] = (uint8_t)(t % 10);
            carry  = t / 10;
        }
        /* carry здесь 0: rem < делителя <= 10^NDIG, rem*10+9 < 10^DDIG */
        /* сколько раз делитель помещается в остаток (максимум 9) */
        int q = 0;
        for (;;) {
            bool fits = true;
            for (int i = DDIG - 1; i >= 0; i--) {
                if (rem[i] != divisor[i]) {
                    fits = rem[i] > divisor[i];
                    break;
                }
            }
            if (!fits || q == 9) break;
            int borrow = 0;                    /* rem -= divisor */
            for (int i = 0; i < DDIG; i++) {
                int t = rem[i] - divisor[i] - borrow;
                if (t < 0) { t += 10; borrow = 1; } else borrow = 0;
                rem[i] = (uint8_t)t;
            }
            q++;
        }
        /* цифра частного на месте 10^pos от A*10^8/B; шкала результата
         * 10^-FRAC: разряд r.d = pos - 2*FRAC + FRAC = pos - FRAC */
        int ri = pos - FRAC;
        if (ri >= 0 && ri < NDIG) {
            if (q) r->d[ri] = (uint8_t)q;
        } else if (ri >= NDIG && q) {
            r->ovf = true;                     /* цифра левее сетки           */
        } else if (pos == FRAC - 1) {
            round_up = (q >= 5);               /* первая отброшенная цифра    */
        }
    }
    if (round_up) {
        int carry = 1;
        for (int i = 0; i < NDIG && carry; i++) {
            int t = r->d[i] + carry;
            r->d[i] = (uint8_t)(t % 10);
            carry   = t / 10;
        }
        if (carry) r->ovf = true;
    }
    if (num_is_zero(r)) r->neg = false;
    #undef DDIG
}

/* число -> строку: знак, целые без ведущих нулей, дробные без хвостовых */
static void num_to_str(const num_t *n, char *buf) {
    int j = 0;
    if (n->neg && !num_is_zero(n)) buf[j++] = '-';

    int top = -1;
    for (int i = NDIG - 1; i >= FRAC; i--)
        if (n->d[i]) { top = i; break; }

    if (top < 0) {
        buf[j++] = '0';
    } else {
        for (int i = top; i >= FRAC; i--) buf[j++] = (char)('0' + n->d[i]);
    }

    int last = -1;
    for (int i = 0; i < FRAC; i++)
        if (n->d[i]) { last = i; break; }

    if (last >= 0) {
        buf[j++] = '.';
        for (int i = FRAC - 1; i >= last; i--) buf[j++] = (char)('0' + n->d[i]);
    }
    buf[j] = 0;
}

/* ── Состояние калькулятора ───────────────────────────────────────────────── */

static num_t calc_acc;                /* накопитель                          */
static char  calc_op;                 /* 0, '+', '-', '*', '/'               */
static char  calc_buf[24];            /* то, что набираем (цифры и '.')      */
static int   calc_buflen;
static bool  calc_entering;
static bool  calc_neg;                /* знак набираемого числа              */
static bool  calc_error;              /* фатальная ошибка (сообщение в дисплее) */
static const char *calc_errmsg;
static char     calc_text[24];

/* буфер -> число (знак учитывается).  Находим точку: целые цифры кладём
 * справа налево от точки-1 в разряды 10^0, 10^1, ... (d[FRAC] и выше),
 * дробные — слева направо после точки в 10^-1, 10^-2, ... (d[FRAC-1] и ниже). */
static void input_to_num(num_t *n) {
    num_zero(n);
    int dot = -1;
    for (int i = 0; i < calc_buflen; i++)
        if (calc_buf[i] == '.') { dot = i; break; }

    int ii = FRAC;
    for (int i = (dot < 0 ? calc_buflen : dot) - 1; i >= 0; i--) {
        if (ii < NDIG) n->d[ii] = (uint8_t)(calc_buf[i] - '0');
        ii++;
    }
    if (dot >= 0) {
        int fi = 0;
        for (int i = dot + 1; i < calc_buflen; i++) {
            if (fi < FRAC) n->d[FRAC - 1 - fi] = (uint8_t)(calc_buf[i] - '0');
            fi++;
        }
    }
    n->neg = calc_neg;
}

static void update_display(void) {
    if (calc_error) {
        int i = 0;
        for (; calc_errmsg[i]; i++) calc_text[i] = calc_errmsg[i];
        calc_text[i] = 0;
        return;
    }
    if (calc_entering) {
        /* Глубокий аудит #224: clamp на случай аномально длинного буфера
         * (инварианты press_digit/press_dot держат buflen <= 14, но
         * страховка дешевле переполнения calc_text[24]). */
        int j = 0;
        if (calc_neg) calc_text[j++] = '-';
        for (int i = 0; i < calc_buflen && j < (int)sizeof(calc_text) - 1; i++)
            calc_text[j++] = calc_buf[i];
        calc_text[j] = 0;
        return;
    }
    num_to_str(&calc_acc, calc_text);
}

void calc_reset(void) {
    num_zero(&calc_acc);
    calc_op       = 0;
    calc_buflen   = 0;
    calc_buf[0]   = 0;
    calc_entering = false;
    calc_neg      = false;
    calc_error    = false;
    calc_errmsg   = "";
    update_display();
}

/* выполнить накопленную операцию: acc = acc OP entry */
static void apply_op(const num_t *entry) {
    num_t r;
    num_zero(&r);
    switch (calc_op) {
        case 0:
            r = *entry;
            break;
        case '+':
            num_add(&r, &calc_acc, entry);
            break;
        case '-': {
            num_t m = *entry;
            m.neg = !m.neg;               /* вычитание = сложение с обратным знаком */
            num_add(&r, &calc_acc, &m);
            break;
        }
        case '*':
            num_mul(&r, &calc_acc, entry);
            break;
        case '/':
            if (num_is_zero(entry)) {
                calc_error  = true;
                calc_errmsg = "Деление на 0";
                return;
            }
            num_div(&r, &calc_acc, entry);
            break;
        default:
            r = *entry;
            break;
    }
    if (r.ovf) {
        calc_error  = true;
        calc_errmsg = "Переполнение";
        return;
    }
    calc_acc = r;
}

/* текущий ввод как число */
static num_t current_entry(void) {
    num_t n;
    input_to_num(&n);
    return n;
}

static int buf_int_count(void) {
    int c = 0;
    for (int i = 0; i < calc_buflen; i++) {
        if (calc_buf[i] == '.') break;
        c++;
    }
    return c;
}

static bool buf_has_dot(void) {
    for (int i = 0; i < calc_buflen; i++)
        if (calc_buf[i] == '.') return true;
    return false;
}

static int buf_frac_count(void) {
    int c = 0;
    bool dot = false;
    for (int i = 0; i < calc_buflen; i++) {
        if (calc_buf[i] == '.') { dot = true; continue; }
        if (dot) c++;
    }
    return c;
}

static void press_digit(int d) {
    if (calc_error) return;
    if (!calc_entering) {
        calc_buflen   = 0;
        calc_buf[0]   = 0;
        calc_neg      = false;
        calc_entering = true;
    }
    if (buf_has_dot()) {
        if (buf_frac_count() >= FRAC) return;       /* дробные кончились     */
    } else {
        if (buf_int_count() >= NINT) return;        /* целые кончились       */
        if (calc_buflen == 1 && calc_buf[0] == '0')
            calc_buflen = 0;                        /* «0» не лепим впереди  */
    }
    if (calc_buflen + 1 >= (int)sizeof(calc_buf)) return;
    calc_buf[calc_buflen++] = (char)('0' + d);
    calc_buf[calc_buflen] = 0;
}

static void press_op(char op) {
    if (calc_error) return;
    if (calc_entering) {
        num_t e = current_entry();
        apply_op(&e);
        calc_entering = false;
        calc_buflen   = 0;
    }
    if (!calc_error) calc_op = op;
}

static void press_eq(void) {
    if (calc_error) return;
    if (!calc_entering && calc_op == 0) return;
    if (calc_entering) {
        num_t e = current_entry();
        apply_op(&e);
    } else {
        /* «5 + =» — второй операнд равен первому, как в обычных калькуляторах */
        apply_op(&calc_acc);
    }
    calc_op        = 0;
    calc_entering  = false;
    calc_buflen    = 0;
    if (!calc_error) calc_neg = false;
}

static void press_backspace(void) {
    if (calc_error) { calc_reset(); return; }
    if (!calc_entering) return;
    if (calc_buflen > 0) {
        calc_buf[--calc_buflen] = 0;
    }
    if (calc_buflen == 0) calc_entering = false;
}

/* «±»: меняет знак того, что на экране.  Во время ввода — знак набираемого
 * числа.  Если на экране результат — он становится текущим вводом с
 * обратным знаком, и набор можно продолжать; ноль даёт «-0», чтобы минус
 * можно было набрать ДО цифр (как на обычных калькуляторах). */
static void press_negate(void) {
    if (calc_error) return;
    if (calc_entering) {
        calc_neg = !calc_neg;
        return;
    }
    num_to_str(&calc_acc, calc_buf);
    int j = (calc_buf[0] == '-') ? 1 : 0;   /* знак уйдёт в calc_neg      */
    int k = 0;
    while (calc_buf[j]) calc_buf[k++] = calc_buf[j++];
    calc_buf[k] = 0;
    calc_buflen   = k;
    calc_neg      = !calc_acc.neg;
    calc_entering = true;
}

static void press_dot(void) {
    if (calc_error) return;
    if (!calc_entering) {
        calc_buflen   = 0;
        calc_buf[0]   = 0;
        calc_neg      = false;
        calc_entering = true;
        calc_buf[calc_buflen++] = '0';
    }
    if (!buf_has_dot()) {
        calc_buf[calc_buflen++] = '.';
        calc_buf[calc_buflen] = 0;
    }
}

void calc_click(int btn) {
    switch (btn) {
        case 0:  calc_reset(); break;                 /* C  */
        case 1:  press_negate(); break;               /* ±  */
        case 2:  press_backspace(); break;            /* ⌫  */
        case 3:  press_op('/'); break;
        case 4:  press_digit(7); break;
        case 5:  press_digit(8); break;
        case 6:  press_digit(9); break;
        case 7:  press_op('*'); break;
        case 8:  press_digit(4); break;
        case 9:  press_digit(5); break;
        case 10: press_digit(6); break;
        case 11: press_op('-'); break;
        case 12: press_digit(1); break;
        case 13: press_digit(2); break;
        case 14: press_digit(3); break;
        case 15: press_op('+'); break;
        case 16: press_digit(0); break;
        case 17: press_dot(); break;                  /* .  — дроби         */
        case 18: press_eq(); break;
        default: break;
    }
    update_display();
}

void calc_key(uint32_t k) {
    if (k >= '0' && k <= '9') { press_digit((int)(k - '0')); update_display(); return; }
    switch (k) {
        case '+': press_op('+'); break;
        case '-': press_op('-'); break;
        case '*': press_op('*'); break;
        case '/': press_op('/'); break;
        case '=':
        case '\n': press_eq(); break;
        case 'c':
        case 'C':
        case 27:  calc_reset(); return;
        case 8:   press_backspace(); break;
        case '.':
        case ',': press_dot(); break;                 /* и запятая тоже     */
        default:  return;
    }
    update_display();
}

/* ─── Drawing ──────────────────────────────────────────────────────────── */

static void btn_xy(int wx, int wy, int idx, int *out_x, int *out_y) {
    int row = idx / 4;
    int col = idx % 4;
    int gx = wx + INNER_PAD + col * (BTN_W + BTN_GAP);
    int gy = wy + 36 + INNER_PAD + DISP_H + INNER_PAD + row * (BTN_H + BTN_GAP);
    *out_x = gx;
    *out_y = gy;
}

void calc_draw(int wx, int wy, int hover_btn, int pressed_btn) {
    /* Display panel */
    int dx = wx + INNER_PAD;
    int dy = wy + 36 + INNER_PAD;
    int dw = INNER_PAD + 4 * BTN_W + 3 * BTN_GAP - INNER_PAD; /* matches grid width = 4*60+3*4=252 */
    fb_rect(dx, dy, dw, DISP_H, rgb(0xE8, 0xEF, 0xD8));
    fb_rect_outline(dx, dy, dw, DISP_H, rgb(0x40, 0x40, 0x40));
    fb_hline(dx + 1, dy + 1, dw - 2, rgb(0x80, 0x80, 0x80));
    fb_vline(dx + 1, dy + 1, DISP_H - 2, rgb(0x80, 0x80, 0x80));

    /* Right-aligned text in display */
    int tw = text_width(calc_text);
    int tx = dx + dw - tw - 8;
    int ty = dy + (DISP_H - FONT_GLYPH_H) / 2;
    text_draw_str(tx, ty, calc_text, COL_BLACK);

    /* Buttons */
    for (int i = 0; i < 20; i++) {
        if (i == 19) continue;  /* unused slot */
        int bx, by;
        btn_xy(wx, wy, i, &bx, &by);
        bool hover   = (i == hover_btn);
        bool pressed = (i == pressed_btn);
        gui_button(bx, by, BTN_W, BTN_H, btn_label[i], hover, pressed);
    }
}

int calc_hit_test(int wx, int wy, int mx, int my) {
    for (int i = 0; i < 20; i++) {
        if (i == 19) continue;
        int bx, by;
        btn_xy(wx, wy, i, &bx, &by);
        if (mx >= bx && mx < bx + BTN_W && my >= by && my < by + BTN_H) return i;
    }
    return -1;
}
