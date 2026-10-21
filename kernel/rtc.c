#include "rtc.h"
#include "io.h"

#define CMOS_ADDR 0x70
#define CMOS_DATA 0x71

static uint8_t cmos_get(uint8_t reg) {
    /* бит 7 = NMI off во время обращения: NMI между outb и inb
     * портил индексный регистр CMOS (стандартная практика) */
    outb(CMOS_ADDR, (uint8_t)(reg | 0x80u));
    return inb(CMOS_DATA);
}

static uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + ((v >> 4) * 10); }

static int update_in_progress(void) {
    return cmos_get(0x0A) & 0x80;
}

void rtc_read(rtc_time_t *t) {
    /* Wait for the RTC to not be in the middle of an update. */
    int safety = 100000;
    while (update_in_progress() && safety-- > 0) {}

    uint8_t s, mn, h, wd, d, mo, y, b;
    uint8_t s2, mn2, h2, wd2, d2, mo2, y2;
    /* Read twice to avoid catching mid-update bytes.
     * Лимит попыток: без него зависший RTC (батарея!) вешал ОС навечно. */
    int tries = 64;
    do {
        s  = cmos_get(0x00);
        mn = cmos_get(0x02);
        h  = cmos_get(0x04);
        wd = cmos_get(0x06);
        d  = cmos_get(0x07);
        mo = cmos_get(0x08);
        y  = cmos_get(0x09);
        s2 = cmos_get(0x00);
        mn2 = cmos_get(0x02);
        h2 = cmos_get(0x04);
        wd2 = cmos_get(0x06);
        d2 = cmos_get(0x07);
        mo2 = cmos_get(0x08);
        y2 = cmos_get(0x09);
    } while ((s != s2 || mn != mn2 || h != h2 || wd != wd2
              || d != d2 || mo != mo2 || y != y2) && --tries > 0);

    b = cmos_get(0x0B);

    int pm = 0;
    if (!(b & 2)) {
        /* 12-hour mode: top bit signals PM in BCD too. */
        if (h & 0x80) { pm = 1; h &= 0x7F; }
    }

    if (!(b & 4)) {  /* values are BCD */
        s  = bcd2bin(s);
        mn = bcd2bin(mn);
        h  = bcd2bin(h);
        d  = bcd2bin(d);
        mo = bcd2bin(mo);
        y  = bcd2bin(y);
        wd = bcd2bin(wd);
    }

    if (!(b & 2) && pm && h < 12) h += 12;
    if (!(b & 2) && !pm && h == 12) h = 0;

    t->sec = s;
    t->min = mn;
    t->hour = h;
    t->weekday = wd;
    t->day = d;
    t->month = mo;
    /* Century register 0x32 (аудит F5 п.21): на большинстве RTC он
     * 0/0xFF («нет регистра»), на PC-AT/ACPI-платах содержит век
     * (в BCD, если данные в BCD). Используем как век, когда он
     * осмысленный (19..30 -> 1999..3099); иначе 2-значный год
     * трактуем как 21-й век, как раньше. */
    uint8_t cent = cmos_get(0x32);
    if (!(b & 4) && cent != 0x00u && cent != 0xFFu)
        cent = bcd2bin(cent);   /* данные в BCD — и век в BCD */
    if (cent >= 19u && cent <= 30u) {
        t->year = (uint16_t)cent * 100u + (uint16_t)y;
    } else {
        /* CMOS year is 2-digit; assume 21st century. */
        t->year = (uint16_t)y + 2000;
    }
}

static void put2(char *o, int v) {
    o[0] = '0' + ((v / 10) % 10);
    o[1] = '0' + (v % 10);
}

void rtc_format_hms(const rtc_time_t *t, char *out) {
    put2(out + 0, t->hour); out[2] = ':';
    put2(out + 3, t->min);  out[5] = ':';
    put2(out + 6, t->sec);
    out[8] = 0;
}

void rtc_format_date(const rtc_time_t *t, char *out) {
    put2(out + 0, t->day);   out[2] = '.';
    put2(out + 3, t->month); out[5] = '.';
    int y = t->year;
    out[6] = '0' + (y / 1000) % 10;
    out[7] = '0' + (y / 100)  % 10;
    out[8] = '0' + (y / 10)   % 10;
    out[9] = '0' + (y        ) % 10;
    out[10] = 0;
}

/* Zeller's congruence fallback (returns 0=Sun..6=Sat). */
static int zeller(int y, int m, int d) {
    if (m < 3) { m += 12; y -= 1; }
    int K = y % 100;
    int J = y / 100;
    int h = (d + (13 * (m + 1)) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;
    /* Convert: h=0 -> Saturday in Zeller */
    int dow = (h + 6) % 7; /* 0=Sun..6=Sat */
    return dow;
}

const char *rtc_weekday_ru(const rtc_time_t *t) {
    static const char *names[] = {
        "Воскресенье", "Понедельник", "Вторник", "Среда",
        "Четверг", "Пятница", "Суббота"
    };
    int dow;
    if (t->weekday >= 1 && t->weekday <= 7) {
        dow = (t->weekday - 1) % 7;
    } else {
        dow = zeller(t->year, t->month, t->day);
    }
    return names[dow & 7];
}

const char *rtc_month_ru(const rtc_time_t *t) {
    static const char *names[] = {
        "—",
        "января", "февраля", "марта", "апреля", "мая", "июня",
        "июля",   "августа", "сентября", "октября", "ноября", "декабря"
    };
    if (t->month < 1 || t->month > 12) return names[0];
    return names[t->month];
}
