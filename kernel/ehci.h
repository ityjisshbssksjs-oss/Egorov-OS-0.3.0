#ifndef EGOROV_EHCI_H
#define EGOROV_EHCI_H

#include "types.h"
#include "ps2.h"

/* ── EHCI-драйвер (USB 2.0, «порты 2.0») ────────────────────────────────────
 * До 0.3.0 EHCI умел только отдавать порты компаньонам (ehci_handoff в
 * mouse_hw.c) - USB 2.0 был «мёртвой зоной».  Теперь это полноценный HCD:
 *   • свой PCI-скан (class 0x0C / 0x03 / prog-if 0x20, ДО ДВУХ контроллеров);
 *   • handoff у BIOS (USBLEGSUP + глушение USBLEGCTLSTS), HCRESET,
 *     расписание (async + periodic) — у КАЖДОГО инстанса СВОЁ;
 *   • CONFIGFLAG=1 - порты забираются у компаньонов ПОСЛЕ того, как
 *     расписание реально поднялось (если инициализация сорвалась - флаг
 *     НЕ ставится, компаньоны и BIOS-эмуляция продолжают работать);
 *   • high-speed устройство (PSPD=2) - перечисление НАПРЯМУЮ: мышь и
 *     клавиатура (HID boot);
 *   • full/low-speed устройство - порт отдаётся компаньону (OHCI/UHCI,
 *     там свои драйверы).  Если компаньона НЕТ - прямая попытка
 *     перечисления (QEMU умеет; часть реальных чипсетов - тоже);
 *   • поллинг без прерываний, как вся Egorov OS.
 *
 * «ЭТАП 2» (реальное железо AMD FCH, 2×EHCI + 2×OHCI): DATA-стадия
 * контрольных передач наконец с тогглом DATA1 (была DATA0 — реальные
 * устройства отклоняли дескрипторы, QEMU не замечал), состояние
 * ПЕР-ИНСТАНСНОЕ (два EHCI не затирают друг друга), порты с неудачным
 * перечислением ретраятся до 3 раз в секунду, PED-порты BIOS
 * сбрасываются перед перечислением, rescan под PSE-стопом.  Подробно —
 * шапка kernel/ehci.c.
 *
 * Клавиатура EHCI разбирает отчёты через usb_kbd_parse_boot() (usb_kbd.c) -
 * один разбор на все шины; мышь - собственное состояние + ehci_mouse_poll.
 *
 * Подключение (вшито):
 *   kernel.c:      ehci_init() ПЕРЕД mouse_hw_init_ohci() (компаньоны
 *                  зондятся уже после раздачи им LS/FS-портов);
 *   ps2.c:         ps2_poll(): ветка ehci_mouse_active() -> ehci_mouse_poll;
 *   usb_kbd.c:     usb_kbd_poll() -> ehci_kbd_poll() (клавиатура 2.0).
 */

bool ehci_init(int fb_w, int fb_h, int force_usb);

/* 0.3.0: полный обход портов EHCI — каждую секунду из usbdev_tick(). */
void ehci_hotplug_sweep(void);

/* true, пока EHCI-мышь является активным источником указателя. */
bool ehci_mouse_active(void);
void ehci_mouse_poll(mouse_state_t *out);

/* опрос клавиатурного qTD (вызывается из usb_kbd_poll каждый кадр). */
void ehci_kbd_poll(void);

/* [#446] LED EHCI-клавиатуры (SET_REPORT по hid_leds: бит0 NumLock,
 * бит1 CapsLock) — вызывается диспетчером usb_kbd_leds_all(). */
void ehci_kbd_set_leds(uint8_t hid_leds);
bool ehci_kbd_present(void);

/* controller уже под нашим драйвером? (mouse_hw.c: ehci_handoff пропустит) */
int ehci_is_ours(uint8_t bus, uint8_t dev, uint8_t fn);

/* СНИМОК EHCI - регистры, порты, устройства (конец экрана, для фото). */
void ehci_snapshot(void);

/* [#407] USB-КАРТА ПОРТОВ: блок EHCI. Печатает все порты всех
 * поднятых инстансов (тип, скорость, хозяин, занятость, счётчик
 * неудач). ports_total/devs_bound — счётчики для итоговой строки
 * общей карты (можно NULL). Вызывается из mouse_hw_portmap().
 * [#472] ЭТАП 9: sock_no — СКВОЗНОЙ счётчик гнезд ОБЕЙ карты
 * (нумерация не начинается заново на каждом контроллере); можно
 * NULL — тогда печатается локальный номер порта. */
void ehci_portmap(int *ctrls, int *lports, int *devs_bound, int *eh_ports,
                  int *sock_no);

#endif
