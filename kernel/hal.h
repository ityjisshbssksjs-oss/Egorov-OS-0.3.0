#ifndef UHCI_KBD_HAL_H
#define UHCI_KBD_HAL_H

/* ─────────────────────────────────────────────────────────────────────────
 *  hal.h — порт HAL для ядра Egorov OS (bare metal, freestanding).
 *
 *  Этот файл кладётся в kernel/ рядом с исходниками драйвера (см.
 *  INTEGRATION.md). Реализация не-инлайн функций — в usb_uhci_kbd.c.
 *
 *  Особенности окружения Egorov OS, на которые опирается порт:
 *    • типы uint8_t/uint16_t/uint32_t и inb/outb/inw/outw/inl/outl —
 *      из kernel/types.h и kernel/io.h;
 *    • ядро работает с физическими адресами 1:1 (GRUB/multiboot,
 *      страничной трансляции нет) — физический адрес = виртуальный;
 *    • memcpy/memset — из kernel/string.c;
 *    • журнал — в последовательный порт (serial_puts/serial_puthex),
 *      как и остальные USB-драйверы этого ядра;
 *    • прерываний нет (ядро работает с замаскированным PIC) — поллинг.
 * ───────────────────────────────────────────────────────────────────────── */

#include "types.h"
#include "io.h"
#include "string.h"

#define UK_HAL_NAME "Egorov OS (bare metal)"

/* определён в kernel.c */
extern void serial_puthex(uint32_t v);

/* ── Порты ввода-вывода: прямые инструкции x86 из io.h ── */
static inline void    hal_out8(uint16_t port, uint8_t value)  { outb(port, value); }
static inline void    hal_out16(uint16_t port, uint16_t value) { outw(port, value); }
static inline void    hal_out32(uint16_t port, uint32_t value) { outl(port, value); }
static inline uint8_t hal_in8(uint16_t port)   { return inb(port); }
static inline uint16_t hal_in16(uint16_t port) { return inw(port); }
static inline uint32_t hal_in32(uint16_t port) { return inl(port); }

/* ── Не-инлайн части порта — реализованы в usb_uhci_kbd.c ── */
void hal_msleep(uint32_t ms);
void hal_log(const char *msg);
void hal_log_hex(const char *msg, uint32_t value);
int  hal_dma_alloc(uint32_t size, void **vaddr, uint32_t *paddr);
uint32_t hal_dma_phys(void *vaddr);
void hal_dma_free(void *vaddr);
int  hal_pci_find_uhci(int index, uint32_t *io_base, char *name, int name_sz);
int  hal_pci_take(uint32_t io_base);
int  hal_pci_release(uint32_t io_base);

#endif /* UHCI_KBD_HAL_H */
