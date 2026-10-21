/* timer.c — калиброванные задержки (PIT ch2, поллинг, без IRQ).
 *
 * Метод (канон Linux calibrate_delay):
 *   1. GATE2 (порт 0x61 бит 0) = 1, спикер-бит (бит 1) = 0.
 *   2. PIT канал 2, режим 0 (interrupt on terminal count),
 *      счётчик 59659 => 59659/1193182 ≈ 50 мс эталонного окна.
 *   3. Крутим цикл С ТЕМ ЖЕ телом, что и kdelay_us (io_wait()),
 *      проверяя OUT2 (бит 5) раз в 64 итерации — калибровка
 *      измеряет именно итерацию задержки, а не чтение порта.
 *   4. Считаем итерации на микросекунду (×4 для машин, где одна
 *      итерация длиннее микросекунды).
 *
 * Если PIT недоступен (OUT2 не поднимается ~2.5 c) — фолбэк
 * «1 итерация = 1 мкс»: поведение старых драйверов на реальном
 * железе, без регрессии.
 */
#include "timer.h"
#include "io.h"

/* io.h не тянет serial-декларации (они в kernel.c) — объявляем. */
void serial_puts(const char *s);
void serial_puthex(uint32_t v);

#define PIT_CH2      0x42u
#define PIT_CMD      0x43u
#define PORT_GATE61  0x61u

/* Счётчик окна: 1193182 Гц * 50 мс = 59659.1 -> 59659 тиков. */
#define PIT_WINDOW_TICKS 59659u
#define PIT_WINDOW_US    50000u

/* Проверять OUT2 раз в столько итераций (погрешность < 2%). */
#define CAL_CHK_EVERY 64u

/* Лимит проходов внешнего цикла до признания «PIT молчит».
 * Один проход = 64 итерации io_wait: на реальном железе ~64 мкс, в
 * QEMU/TCG ~2 мкс.  За 50-мс окно калибровка успевает сделать максимум
 * ~100-160 тыс. проходов даже на ультрабыстрой VM - 200 000 даёт
 * запас и НЕ даёт ложного фолбэка; при реально молчащем PIT калибровка
 * заканчивается за <= ~13 с на реальном железе и за ~0.4 с в VM.
 * (Раньше было 40 000 000: на реальном ПК с мёртвым PIT это ~40 МИНУТ
 * зависания на старте.) */
#define CAL_MAX_PASSES 200000u

/* ×4-фиксированный тариф «итераций/мкс» (до калибровки: 4 = 1
 * итерация/мкс — консервативная оценка реального железа). */
static uint32_t t_iters_per_us_x4 = 4u;

void timer_init(void) {
    uint8_t  save61;
    uint32_t iters = 0;
    uint32_t cal_x4;
    uint32_t guard = 0;

    save61 = inb(PORT_GATE61);
    /* GATE2=1 (счёт идёт), спикер выключен (бит 1 = 0). */
    outb(PORT_GATE61, (uint8_t)((save61 | 0x01u) & ~0x02u));

    /* Канал 2, режим 0, lo/hi, двоичный: 0b10_11_000_0 = 0xB0. */
    outb(PIT_CMD, 0xB0);
    outb(PIT_CH2, PIT_WINDOW_TICKS & 0xFFu);
    outb(PIT_CH2, (PIT_WINDOW_TICKS >> 8) & 0xFFu);

    /* Окно 50 мс: тело цикла совпадает с kdelay_us (io_wait +
     * инкремент + сравнение). OUT2 проверяем редко. */
    for (;;) {
        uint32_t j;
        for (j = 0; j < CAL_CHK_EVERY; j++) io_wait();
        iters += CAL_CHK_EVERY;
        if (inb(PORT_GATE61) & 0x20u) break;          /* окно вышло */
        if (++guard > CAL_MAX_PASSES) { iters = 0u; break; } /* PIT молчит */
    }

    /* GATE2 обратно как было (спикер остаётся выключенным). */
    outb(PORT_GATE61, (uint8_t)(save61 & ~0x02u));

    if (iters == 0u) {
        t_iters_per_us_x4 = 4u;
        serial_puts("TIMER: PIT ch2 не отвечает - фолбэк 1 итер/мкс\n");
        return;
    }

    /* 32-битной арифметики хватает: guard ограничивает iters
     * 200 000 проходов × 64 = 12.8 млн, ×4 = 51.2 млн < 2^32
     * (а 64-битное деление в freestanding тянет __udivdi3). */
    cal_x4 = (iters * 4u) / PIT_WINDOW_US;
    if (cal_x4 == 0u)    cal_x4 = 1u;
    if (cal_x4 > 4000000u) cal_x4 = 4000000u;   /* мусорный OUT2 — защита */
    t_iters_per_us_x4 = cal_x4;

    serial_puts("TIMER: PIT-калибровка: ");
    serial_puthex(iters);
    serial_puts(" итераций / 50 мс = ");
    serial_puthex(t_iters_per_us_x4 / 4u);
    serial_puts(".");
    serial_puthex(((t_iters_per_us_x4 % 4u) * 10u) / 4u);
    serial_puts(" итераций/мкс\n");
}

void kdelay_us(uint32_t us) {
    /* 64-битное умножение: длинные задержки (50 мс = 5e4 мкс при
     * 4e6 итер/мкс => 2e11) не должны переполнять uint32. */
    uint64_t n64 = ((uint64_t)t_iters_per_us_x4) * (uint64_t)us;
    uint32_t n = (uint32_t)(n64 >> 2);
    uint32_t i;
    for (i = 0; i < n; i++) io_wait();
}

void kdelay_ms(uint32_t ms) {
    while (ms-- > 0) kdelay_us(1000u);
}
