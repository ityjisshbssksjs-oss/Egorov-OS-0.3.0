#ifndef EGOROV_RTC_H
#define EGOROV_RTC_H

#include "types.h"

typedef struct {
    uint16_t year;          /* full 4-digit year, e.g. 2026 */
    uint8_t  month;         /* 1..12 */
    uint8_t  day;           /* 1..31 */
    uint8_t  weekday;       /* 1..7 (CMOS-defined; not always populated) */
    uint8_t  hour;          /* 0..23 */
    uint8_t  min;           /* 0..59 */
    uint8_t  sec;           /* 0..59 */
} rtc_time_t;

void rtc_read(rtc_time_t *t);

/* Format "HH:MM:SS" into out (>=9 bytes). */
void rtc_format_hms(const rtc_time_t *t, char *out);
/* Format "DD.MM.YYYY" into out (>=11 bytes). */
void rtc_format_date(const rtc_time_t *t, char *out);
/* Russian weekday name from t->weekday + computed Zeller fallback. */
const char *rtc_weekday_ru(const rtc_time_t *t);
/* Russian month name. */
const char *rtc_month_ru(const rtc_time_t *t);

#endif
