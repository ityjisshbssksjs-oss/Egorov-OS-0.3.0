#include "string.h"

void *memset(void *dst, int c, size_t n) {
    uint8_t *d = (uint8_t*)dst;
    for (size_t i = 0; i < n; i++) d[i] = (uint8_t)c;
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n) {
    uint8_t *d = (uint8_t*)dst;
    const uint8_t *s = (const uint8_t*)src;
    for (size_t i = 0; i < n; i++) d[i] = s[i];
    return dst;
}

int memcmp(const void *a, const void *b, size_t n) {
    const uint8_t *x = (const uint8_t*)a;
    const uint8_t *y = (const uint8_t*)b;
    for (size_t i = 0; i < n; i++) if (x[i] != y[i]) return (int)x[i] - (int)y[i];
    return 0;
}

size_t strlen(const char *s) {
    size_t n = 0;
    while (s[n]) n++;
    return n;
}

int strcmp(const char *a, const char *b) {
    while (*a && *a == *b) { a++; b++; }
    return (int)(uint8_t)*a - (int)(uint8_t)*b;
}

char *itoa(uint32_t v, char *buf, int base) {
    static const char digits[] = "0123456789ABCDEF";
    /* Защита: base вне 2..16 читала бы digits за границей (индекс >= 16).
     * Все текущие вызовы - base 10/16; clamp не меняет поведение. */
    if (base < 2 || base > 16) base = 10;
    char tmp[32];
    int i = 0;
    if (v == 0) { buf[0] = '0'; buf[1] = 0; return buf; }
    while (v > 0 && i < 31) {
        tmp[i++] = digits[v % (uint32_t)base];
        v /= (uint32_t)base;
    }
    int j = 0;
    while (i > 0) buf[j++] = tmp[--i];
    buf[j] = 0;
    return buf;
}
int strncmp(const char *a, const char *b, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (a[i] != b[i]) return (int)(unsigned char)a[i] - (int)(unsigned char)b[i];
        if (a[i] == 0) return 0;
    }
    return 0;
}
