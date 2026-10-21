#ifndef EGOROV_STRING_H
#define EGOROV_STRING_H

#include "types.h"

void *memset(void *dst, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);
int memcmp(const void *a, const void *b, size_t n);
size_t strlen(const char *s);
int strcmp(const char *a, const char *b);
char *itoa(uint32_t v, char *buf, int base);
int strncmp(const char *a, const char *b, size_t n);
#endif
