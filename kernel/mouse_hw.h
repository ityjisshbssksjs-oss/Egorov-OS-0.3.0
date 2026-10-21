#ifndef EGOROV_MOUSE_HW_H
#define EGOROV_MOUSE_HW_H

#include "types.h"
#include "ps2.h"

/* ── MOUSEHW: отдельный драйвер мыши для РЕАЛЬНОГО ЖЕЛЕЗА (дополнение) ──────
 *
 * Написан с нуля как ДОПОЛНЕНИЕ и ничего не трогает из работающего:
 *   • kernel/usb_mouse.c (v1.5, UHCI) — не изменён ни на байт;
 *   • PS/2-путь в ps2.c — работает как раньше;
 *   • QEMU-сценарии (piix3-usb-uhci + usb-mouse/usb-tablet) ведут себя
 *     точно так же: без xHCI-контроллера этот драйвер молчит.
 *
 * Что добавляет: поддержку USB-мышей на xHCI-контроллерах (prog-if 0x30) —
 * именно они стоят на всех современных ПК (Intel Haswell 2013+, AMD FM2+).
 * У пользователя раньше был только такой контроллер, и UHCI-драйвер его
 * не видел.
 *
 * Архитектура:
 *   • собственный PCI-скан (class 0x0C / subclass 0x03 / prog-if 0x30);
 *   • MMIO BAR0, BIOS/SMI hand-off через USB Legacy xECP;
 *   • halt → HCRST → CNR; DCBAA (+scratchpad), command ring, event ring
 *     (ERST 1 сегмент, поллинг БЕЗ прерываний — как весь Egorov OS);
 *   • Enable Slot → Address Device (BSR=0) → GET_DESCRIPTOR(18) →
 *     Evaluate Context (EP0 mps) → SET_CONFIG → Configure Endpoint;
 *   • HID boot-протокол (SET_PROTOCOL(boot) + SET_IDLE), относительные
 *     отчёты [btn,dx,dy(,dz)]; авто-детект абсолютного планшета по
 *     HID report descriptor (usb-tablet) с масштабированием LOGICAL_MAX;
 *   • один outstanding interrupt-IN TRB, докрутка из mouse_hw_poll();
 *   • при ошибках трансферов/смене состояния порта драйвер деактивируется
 *     и указатель автоматически возвращается к PS/2 (через ps2_poll).
 *
 * Подключение (уже вшито):
 *   kernel.c:      mouse_hw_init(fb.width, fb.height) после usb_mouse_init();
 *   ps2.c:         ps2_poll(): ветка mouse_hw_active() → mouse_hw_poll(out);
 *   desktop.c:     статус «Мышь: USB (xHCI)» + pointer_visible().
 */

/* Сканировать PCI, инициализировать xHCI и перечислить HID мышь.
 * Вызывается один раз на старте; без xHCI-контроллера ничего не делает
 * (кроме serial-строки «no xHCI controller»).
 * «ЭТАП 2»: здесь ТОЛЬКО xHCI-часть — OHCI-компаньоны зондит отдельная
 * mouse_hw_init_ohci() СТРОГО ПОСЛЕ ehci_init() (см. kernel.c). */
void mouse_hw_init(int fb_w, int fb_h);

/* «ЭТАП 2»: зонд OHCI-компаньонов (AMD FCH: 0:12.0/0:13.0 — туда EHCI
 * раздаёт LS/FS-устройства из портов 2.0).  Вызывать из kernel.c СТРОГО
 * ПОСЛЕ ehci_init(): EHCI уже раздал порты компаньонам и перечислил
 * HS-устройства — компаньоны зондятся в финальном состоянии, и
 * CONFIGFLAG больше не «отбирает» у них порты под работающими
 * устройствами.  Использует тот же PCI-скан, что mouse_hw_init(). */
void mouse_hw_init_ohci(void);

/* режим «usbmouse» (cmdline): взять USB-контроллеры под контроль
 * ДАЖЕ без PS/2-клавиатуры.  BIOS-эмуляция Клавы умрёт - поэтому вместе
 * с мышью драйвер ищет и USB HID-клавиатуру (boot protocol) и вливает её
 * сканкоды в общую очередь ps2.c.  Вызвать ДО mouse_hw_init(). */
void mouse_hw_force_usb(int force);

/* true, если HID мышь перечислена на xHCI-контроллере. */
bool mouse_hw_present(void);

/* true, пока MOUSEHW является активным источником указателя.
 * Проверяется ps2_poll() каждый кадр (после usb_mouse_active()). */
bool mouse_hw_active(void);

/* true, пока жива USB-КЛАВИАТУРА на OHCI или xHCI (сканкоды идут в
 * очередь ps2.c).  Нужно usb_kbd_present() - сводка «клава есть на
 * какой-нибудь USB-шине». */
bool mouse_hw_kbd_present(void);

/* Опросить event ring и обновить состояние мыши (тот же mouse_state_t,
 * что и у PS/2/USB-драйверов). Никогда не блокирует. */
void mouse_hw_poll(mouse_state_t *out);

/* 0.3.0: полный обход портов ВСЕХ контроллеров (xHCI×2 + OHCI×2) —
 * вызывается диспетчером usbdev_tick() КАЖДУЮ СЕКУНДУ, независимо от
 * обстоятельств: переподключения, выдёргивания, новые устройства. */
void mouse_hw_hotplug_sweep(void);

/* [#412] USB-КАРТА ПОРТОВ → [#459] ЕДИНАЯ КАРТА ФИЗИЧЕСКИХ ГНЕЗД:
 * один раз на загрузке (kernel.c, после подъёма всех USB-контроллеров)
 * печатает карту ГНЕЗД (peer-пары линий USB2+USB3 xHCI по Linux-модели
 * find_and_link_peer; EHCI-гнёзда USB 2.0; OHCI/UHCI-линии 1.x с честной
 * пометкой «компаньон — соответствие без ACPI неизвестно»), под каждым
 * гнездом — технические сведения (контроллер, лог.порты, скорость,
 * перечисление, драйвер) и [#462] счётчик живых HID-отчётов.
 * [#451] горячее обновление: привязка/отключение ставят флаг
 * (mouse_hw_portmap_mark), секундный тик печатает свежую карту
 * (mouse_hw_portmap_flush) — карта больше не устаревает. */
void mouse_hw_portmap(void);
void mouse_hw_portmap_mark(void);
void mouse_hw_portmap_flush(void);

/* [#459/#462] хелперы КАРТЫ ГНЕЗД — используются и из ehci.c:
 * печать числа и строка живости «HID-отчётов: N (ДАННЫЕ ИДУТ)». */
void portmap_pm_n(int v);
void portmap_pm_alive(int role);

/* [#446] LED USB-клавиатур xHCI/OHCI (SET_REPORT по hid_leds:
 * бит0 NumLock, бит1 CapsLock). Вызывается диспетчером LED
 * (usb_kbd_leds_all) при тумблерах NumLock/CapsLock. */
void mouse_hw_kbd_set_leds(uint8_t hid_leds);

/* Опрос ТОЛЬКО клавиатурной части (OHCI/xHCI): вызывается из
 * ps2_poll_key(), когда мышь обслуживается другим драйвером (UHCI) и
 * mouse_hw_poll не выполняется - иначе USB-клавиатуры замирали. */
void mouse_hw_kbd_poll(void);

/* v1.9: компактная сводка PCI-скана USB-контроллеров для ЭКРАНА:
 * «USB PCI: EHCI 0:1d.0, xHCI 0:14.0» / «USB PCI: контроллеров нет».
 * Заполняется в mouse_hw_init(); показывается в приветственном диалоге
 * (desktop.c) — диагностика «почему мышь не нашлась» без журнала и COM1. */
const char *mouse_hw_usb_diag(void);

/* ── USB Mass Storage на OHCI (режим sticklog — журнал на флешку) ───────────
 * Флешку перечисляет OHCI-драйвер (класс 0x08/0x06/0x50, bulk-пара EP).
 * Транспорт BOT+SCSI (CBW/DATA/CSW, READ(10)/WRITE(10)) живёт в mouse_hw.c;
 * LBA — абсолютные секторы по 512 байт.  Вызывать из kernel.c только в
 * режиме sticklog ПОСЛЕ mouse_hw_init().  mouse_hw_msd_name() — строка
 * «vendor product» из SCSI INQUIRY (готова после перечисления). */
bool mouse_hw_msd_present(void);
uint32_t mouse_hw_msd_sectors(void);
const char *mouse_hw_msd_name(void);
bool mouse_hw_msd_read(uint32_t lba, void *buf512);
bool mouse_hw_msd_write(uint32_t lba, const void *buf512);

/* ── [LOGS] ЭТАП 11: USB Mass Storage на xHCI (тот же контракт, что и
 * OHCI-вариант: единый блочный интерфейс logstick.c поверх обоих).
 * Перечисляется автоматически при загрузке (как OHCI), буферы вызывающего
 * обязаны быть физически непрерывны (logstick передаёт 4096-выровненный
 * секторный буфер).  Проверено в QEMU (qemu-xhci + usb-storage);
 * на реальном железе НЕ проверено — см. README, ЭТАП 11. */
bool mouse_hw_xhci_msd_present(void);
uint32_t mouse_hw_xhci_msd_sectors(void);
const char *mouse_hw_xhci_msd_name(void);
bool mouse_hw_xhci_msd_read(uint32_t lba, void *buf512);
bool mouse_hw_xhci_msd_write(uint32_t lba, const void *buf512);

/* однострочная OHCI-сводка - последний шаг поиска флешки (для ИТОГА
 * sticklog в kernel.c).  Примеры: «OHCI: контроллер найден, портов 5» /
 * «SET ADDRESS не прошёл (порт 1)» / «флешка ГОТОВА (BOT+SCSI работает)» /
 * «скан ещё не выполнялся».  Безопасна в любой момент. */
void mouse_hw_ohci_diag(char *out, int outsz);

/* «OHCI СНИМОК» - конечный блок для ФОТО: рамка + ВСЕ критичные
 * значения OHCI (регистры HC с расшифровкой, тикание кадров, ED/TD, cc,
 * порты, последний шаг).  kernel.c печатает его ПОСЛЕ «ИТОГ: НЕ ЗАПИСАНО»,
 * поэтому блок всегда оказывается в самом низу экрана sticklog.
 * Безопасна в любой момент, ничего не ждёт (кроме 2 мс на проверку FM). */
void mouse_hw_ohci_snapshot(void);
void mouse_hw_xhci_snapshot(void);   /* СНИМОК xHCI-мыши (конец экрана) */

#endif
