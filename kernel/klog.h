#ifndef EGOROV_KLOG_H
#define EGOROV_KLOG_H

#include "types.h"

/* ── KLOG: журнал ядра — зеркало serial-вывода (COM1) в RAM-кольцо ──────────
 * Реализация в kernel.c: каждый байт serial_puts/serial_puthex дублируется
 * в буфер 8 КБ.  Просмотр — команда COREINFO в Терминале (F12).
 * На реальном ПК без COM1 это единственный способ увидеть MOUSEHW/POWER
 * логи (диагностика «почему мышь не нашлась») прямо на целевой машине. */

/* Скопировать хвост журнала: последние min(total, dstsz-1) байт,
 * NUL-терминировано.  dst==NULL или dstsz<=0 — безопасный no-op. */
void klog_read_tail(char *dst, int dstsz);

/* Всего байт записано в журнал с момента старта ядра. */
uint32_t klog_total(void);

/* [LOGS] Старейший ещё-живой байт журнала (переполнение кольца:
 * всё записанное раньше УЖЕ ЗАТЁРТО — выгрузка начинается отсюда). */
uint32_t klog_oldest(void);

/* [LOGS] РАННИЙ БУФЕР (build 4): линейный буфер старта журнала —
 * первые байты загрузки НЕ затираются кольцом и доживают до выгрузки
 * на флешку.  early_len == klog_total() → журнал полный с первого
 * байта.  Размер — cmdline logearly=N КБ (по умолчанию 128). */
uint32_t klog_early_len(void);

/* [LOGS] Копия байт [from..) раннего буфера (не более dstsz);
 * возвращает число скопированных байт (0 — from за концом). */
uint32_t klog_early_copy(uint32_t from, char *dst, uint32_t dstsz);

/* [LOGS] Безопасный снимок журнала для выгрузки на накопитель:
 * копирует байты [from..) в dst (не более dstsz), с корректной
 * обработкой переполнения кольца (from подтягивается к klog_oldest())
 * и конца журнала.  Возвращает число скопированных байт. */
uint32_t klog_snapshot(uint32_t from, char *dst, uint32_t dstsz);

/* близнец-зеркало журнала (второе кольцо в резерве _klog2_start —
 * самый конец .bss за 3 МБ fb_backbuf, задаёт linker.ld).  COREINFO
 * показывает его, если основное кольцо повреждено (бродячий DMA затирает
 * klog_pos/кольцо — симптом «COREINFO: 1: 2» на реальном ПК). */
uint32_t klog_mirror_total(void);
void klog_mirror_tail(char *dst, int dstsz);

#endif
