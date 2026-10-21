#ifndef UHCI_KBD_INT_H
#define UHCI_KBD_INT_H

/* ─────────────────────────────────────────────────────────────────────────
 *  uhci_int.h — внутренний API между модулями ядра драйвера.
 *  НЕ для внешних пользователей: включается только из src/uhci_core.c,
 *  src/uhci_ctl.c и src/uhci_kbd.c.
 * ───────────────────────────────────────────────────────────────────────── */

#include "uhci_hw.h"
#include "usb_std.h"
#include "uhci_kbd.h"

/* ── из uhci_core.c ── */

/* Сброс контроллера, инициализация пула/QH, запуск.
 * 0 = контроллер работает.
 *
 * hook_qh_va == 0 — драйвер становится ПОЛНЫМ владельцем контроллера:
 * HCRESET, программирование своего frame list и запуск (USBCMD=RS).
 *
 * hook_qh_va != 0 — «подключение к чужому расписанию» (shared-режим):
 * контроллер уже работает под другим драйвером (например, UHCI-мышью
 * из usb_mouse.c). Свой frame list НЕ строится, регистры контроллера
 * не трогаются вовсе; QH прерываний этого драйвера подцепляется к
 * чужому расписанию вызывающим кодом (портом) — см. uhci_kbd_attach_shared. */
int  uk_core_start(uint32_t io_base, const char *name, uint32_t hook_qh_va);

/* Остановить контроллер (USBCMD=0), забыть состояние. */
void uk_core_stop(void);

/* Полный сброс и включение корневого порта.
 * 0 = порт включён; *low_speed = скорость устройства. */
int  uk_port_reset(int port, int *low_speed);

/* Прогнать одиночный TD через QH управляющих передач и дождаться
 * завершения. 0 = успех; <0 = ошибка/таймаут (TD деактивирован). */
int  uk_run_td(volatile uhci_td_t *td, const char *tag);

/* [#403] Сброс кэш-когерентности DMA-пула (wbinvd): контроллер UHCI
 * читает TD/QH/frame list прямо из ОЗУ и пишет статус туда же. На
 * реальном железе с write-back кэшем CPU обязан выталкивать свои
 * записи ДО запуска TD и обновлять кэш в цикле опроса — QEMU кэша
 * не имеет, там wbinvd безвреден (как и в OHCI/EHCI/xHCI).
 * Фото-симптом без него: TD «висит» активным либо «отвечает» нулями. */
void uk_dma(void);

/* Выделить очередной вращающийся TD: адрес никогда не переписывается,
 * пока контроллер мог недавно его закешировать. */
volatile uhci_td_t *uk_td_slot(void);

/* Выделить очередной вращающийся 8-байтовый слот SETUP-пакета. */
uint8_t *uk_setup_slot(void);

/* Буфер данных управляющих передач (512 байт в пуле). */
uint8_t *uk_ctl_data_buf(void);

/* Доступ к пулу и перевод адресов. */
volatile uint8_t *uk_pool(void);
uint32_t uk_phys(const void *pool_ptr);

/* Чтение/запись регистров контроллера. */
uint16_t uk_rd16(uint32_t reg);
void     uk_wr16(uint32_t reg, uint16_t v);
void     uk_wr32(uint32_t reg, uint32_t v);

/* Общее состояние (определено в uhci_core.c). */
struct uk_state {
    int      attached;      /* контроллер захвачен и запущен      */
    int      shared;        /* работаем в чужом расписании        */
    uint32_t io;            /* I/O база                           */
    char     name[24];      /* PCI-слот для журнала               */
    int      low_speed;     /* скорость привязанного устройства   */
    uint8_t  dev_addr;      /* текущий USB-адрес устройства       */
    uint8_t  ep0_maxp;      /* maxpacket EP0                      */
    int      kbd_bound;     /* клавиатура привязана               */
    int      kbd_port;
    uint8_t  kbd_iface;
    uint8_t  kbd_ep;
    uint8_t  kbd_mps;
    uint8_t  kbd_toggle;    /* ожидаемый data toggle interrupt-EP */
    uint32_t kbd_errs;      /* ошибки TD подряд                   */
    uint32_t poll_cnt;      /* счётчик опросов (для редких задач) */
};
extern struct uk_state uk;

/* ── из uhci_ctl.c ── */

/* Управляющая передача: SETUP + опциональная дата (кадрами по ep0_maxp)
 * + статусная стадия. data == NULL означает wLength = 0.
 * Возвращает wLength (>=0) при успехе, <0 при ошибке. */
int  uk_ctl_transfer(uint8_t rt, uint8_t req, uint16_t val, uint16_t idx,
                     uint16_t wlen, void *data);

/* ── из uhci_kbd.c ── */

/* Привязать найденную boot-клавиатуру: ставит TD прерываний в расписание. */
int  uk_kbd_bind(int port, uint8_t iface, uint8_t ep, uint8_t mps);

/* Снять клавиатуру (без остановки контроллера). */
void uk_kbd_unbind(const char *why);

#endif /* UHCI_KBD_INT_H */
