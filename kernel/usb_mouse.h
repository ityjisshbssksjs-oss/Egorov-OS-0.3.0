#ifndef EGOROV_USB_MOUSE_H
#define EGOROV_USB_MOUSE_H

#include "types.h"
#include "ps2.h"

/* ── USB HID Optical Mouse driver (boot protocol) ──────────────────────────
 *
 * Polled USB mouse on a UHCI host controller.  The driver is fully
 * self-contained (own PCI scan, frame list, TD/QH pools) and does not
 * interfere with the mass-storage stack in usb.c.
 *
 * Typical boot order (see kernel.c):
 *     usb_mouse_init(fb.width, fb.height);   // once, after fb_init
 *     ...desktop loop: usb_mouse_poll() via ps2_poll()
 *
 * When no USB mouse is found — or when the device disappears — the driver
 * deactivates itself and the PS/2 driver takes over automatically.
 */

/* Scan PCI for a UHCI controller, reset/enumerate the device on its ports,
 * switch it to the HID boot protocol (or absolute mode for tablets) and
 * start interrupt-IN polling.  Safe to call once at boot; has no effect
 * if already initialised. */
void usb_mouse_init(int fb_w, int fb_h);

/* true if a HID mouse was enumerated on the USB bus. */
bool usb_mouse_present(void);

/* true while the USB mouse is the active pointer source (device alive,
 * interrupt queue healthy).  Checked by ps2_poll() on every frame. */
bool usb_mouse_active(void);

/* Poll the interrupt-IN endpoint and update the shared mouse state
 * (same mouse_state_t layout as the PS/2 driver).  Never blocks. */
void usb_mouse_poll(mouse_state_t *out);

/* ── Разделение UHCI-контроллера с драйвером клавиатуры (0.4.0) ──────────────
 *
 * Два драйвера — мышь (usb_mouse.c) и клавиатура (usb_uhci_kbd.c) — могут
 * работать на ОДНОМ UHCI-контроллере (QEMU: -usb -device usb-mouse
 * -device usb-kbd).  Контроллер и frame list остаются у мыши, а QH/TD
 * клавиатуры подцепляются к её расписанию горизонтальной ссылкой:
 *
 *     frame_list[i] -> QH мыши (TD мыши) -> QH клавиатуры -> QH управлений
 *
 * Каждая переустановка TD мыши сохраняет цепочку, поэтому клавиатура
 * не отваливается на ходу. */

/* true, если мышь держит UHCI-контроллер (расписание живо). */
bool usb_mouse_uhci_busy(void);

/* I/O база занятого контроллера (0 = контроллер свободен). */
uint16_t usb_mouse_uhci_io(void);

/* Номер корневого порта, на котором сидит мышь (0..3 — столько портов
 * бывает у UHCI; -1 = не найдена). */
int usb_mouse_uhci_port(void);

/* Подцепить чужую очередь за QH мыши: qh_link — адрес QH (уже с флагом |2)
 * или 1 для отключения.  Возвращает прежнее значение связи.  Вызывается
 * только из usb_uhci_kbd.c до/после его работы на этом контроллере. */
uint32_t usb_mouse_sched_chain(uint32_t qh_link);

/* ── 0.3.2: hotplug-перескан UHCI + координация с драйвером клавиатуры ── */

/* Периодический (внутренне ограниченный по частоте) перескан СВОБОДНЫХ
 * UHCI-контроллеров: мышь могли воткнуть после загрузки или в порт 2.0,
 * отданный EHCI компаньону позже init-скана. Вызывается из ps2_poll(),
 * когда ни одна USB-мышь не активна. */
void usb_mouse_rescan(void);

/* 0.3.0: то же без троттлинга — секундный вызов из usbdev_tick(). */
void usb_mouse_rescan_now(void);

/* Каким UHCI сейчас владеет драйвер клавиатуры (usb_uhci_kbd.c)?
 * 0 = никем. rescan эти контроллеры не трогает. */
void usb_mouse_set_kbd_controller(uint16_t io);

/* [LOGS] ЭТАП 11: чтение того же значения (usb_init() mass storage
 * скипает контроллер, пока живая UHCI-клавиатура). */
uint16_t usb_mouse_kbd_controller(void);

/* [LOGS] ЭТАП 11: UHCI-контроллер, которым владеет MSD (usb.c, режим
 * sticklog): rescan мыши его не трогает — повторное перечисление
 * флешки убивало живой MSD-адрес и выгрузку журнала. 0 = никем. */
void usb_mouse_set_msd_controller(uint16_t io);
uint16_t usb_mouse_msd_controller(void);

#endif
