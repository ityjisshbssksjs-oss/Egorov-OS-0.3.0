#ifndef EGOROV_USB_UHCI_KBD_H
#define EGOROV_USB_UHCI_KBD_H

#include "types.h"

/* ── USB-KBD: транспорт UHCI (HID boot protocol) ─────────────────────────
 *
 * Самодостаточный UHCI-транспорт клавиатуры для Egorov OS. Находит
 * UHCI-контроллер по PCI, инициализирует его (frame list, QH/TD),
 * перечисляет устройства на корневых портах и, найдя boot-клавиатуру
 * (интерфейс class=HID, protocol=1), переключает её в boot-протокол и
 * начинает опрос interrupt-IN. Отчёты вливаются в общий конвейер
 * usb_kbd.c: usb_kbd_parse_boot(pkt, len, "UHCI") — сканкоды набора 1
 * уходят в очередь ps2.c, и вся ОС (терминал, Блокнот, рабочий стол)
 * получает USB-клавиатуру поверх UHCI как родную.
 *
 * Реализация переносимого ядра драйвера:
 *   uhci_core.c — контроллер, расписание, исполнитель TD
 *   uhci_ctl.c  — управляющие передачи, перечисление
 *   uhci_kbd.c  — boot-отчёты, LED, таблицы usage
 * Этот файл — HAL-глюй к окружению Egorov OS (io.h, string.h, PCI,
 * 1:1 физика) и тонкая обёртка публичного API.
 *
 * Разделение контроллера с usb_mouse.c (UHCI-мышь): если мышь уже
 * владеет UHCI-контроллером, драйвер НЕ сбрасывает его, а подцепляет
 * свой QH прерываний горизонтальной ссылкой за QH мыши
 * (usb_mouse_sched_chain) — мышь и клавиатура работают на ОДНОМ
 * контроллере в общем frame list. Свободный контроллер забирается
 * целиком (HCRESET, свой frame list).
 *
 * Порядок вызова (kernel.c):
 *     usb_uhci_kbd_init();          // после usb_mouse_init()/ehci_init
 *     usb_uhci_kbd_poll();          // из usb_kbd_poll() -> ps2_poll_key()
 */

/* Сканирует PCI, находит свободный UHCI, перечисляет порты, привязывает
 * boot-клавиатуру. Безопасно вызывать один раз при старте. */
void usb_uhci_kbd_init(void);

/* 0.3.0: регистрация/освобождение слота реестра usbdev. */
int  usb_uhci_kbd_registry_ok(void);
void usb_uhci_kbd_registry_detach(void);

/* Кадровый опрос (не блокирует). Вызывать из главного цикла ввода —
 * достаточно 30..1000 Гц. */
void usb_uhci_kbd_poll(void);

/* true, если UHCI-клавиатура привязана и жива. */
bool usb_uhci_kbd_present(void);

#endif
