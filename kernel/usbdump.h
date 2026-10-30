#ifndef EGOROV_USBDUMP_H
#define EGOROV_USBDUMP_H

/* ═══ [#494] МЕГА-ЛОГ: «в логе пишем ОБ ОВСЁМ» + ID-таблицы ═════════════════
 *
 * Запрос пользователя (EGOROV.LOG Сборки 5, реальная AMD FCH): «Сделай так
 * чтобы в логе писало обо всём. посмотри почему у линукс работает на любом
 * пк, а у нас даже на моём не работает. может там список из этих нужен».
 *
 * Три честных ответа — и всё три реализованы:
 *  1) ЛОГ ПИШЕТ ВСЁ: полный PCI-дамп (все устройства, имена из таблицы),
 *     полные USB-дескрипторы (девайс + конфиг + интерфейсы + эндпоинты +
 *     СТРОКИ производителя/продукта/серийника), побитовая расшифровка
 *     регистров портов (OHCI/EHCI/xHCI) и статуса контроллера, метки
 *     времени TS=Nс у ключевых событий (RTC, секунды от старта ядра).
 *  2) СПИСОК: pci.ids/usb.ids у Linux нужен для ИМЁН, а не для работы:
 *     драйверы Linux — КЛАССОВЫЕ (HID, MSD), «списка поддерживаемых
 *     устройств» там нет. Мы добавили компактные таблицы имён (ниже) —
 *     для диагностики; работать это не мешает и не помогает.
 *  3) «ПОЧЕМУ У LINUX РАБОТАЕТ»: см. фиксы #492/#493 — Linux (а) повторяет
 *     сброс порта, пока порт не СТАНЕТ ВКЛЮЧЁННЫМ (PES), (б) НИКОГДА не
 *     отправляет передачи в остановленный контроллер (HCHalted), (в) не
 *     полагается на «висячие» биты статуса. Мы делаем то же самое.
 *
 * Всё печатается через serial_puts/serial_puthex → автоматически попадает
 * и в COM1, и в RAM-кольцо klog → EGOROV.LOG на флешке.
 */

#include "types.h"   /* фристендинг-типы (uint8_t и т.д.) */

/* Зафиксировать секунду старта ядра (вызов из kernel_main, ПОСЛЕ RTC
 * доступности; до этого ud_seconds() честно возвращает 0). */
void ud_boot_mark(void);

/* Секунды от старта ядра (RTC CMOS, разрешение 1 с; wraps через 24 ч). */
uint32_t ud_seconds(void);

/* Метка времени в журнал: «TS=123с <label>». */
void ud_ts(const char *label);

/* ── Полный PCI-дамп: все устройства на всех шинах, с именами ── */
void ud_pci_dump_all(void);

/* Имена из таблиц (NULL/"" если не знаем — печатаем честное «не в базе»). */
const char *ud_pci_vendor(uint16_t vid);
const char *ud_pci_device(uint16_t vid, uint16_t pid);   /* имя USB-HC и др. */
const char *ud_usb_vendor(uint16_t vid);                /* имена USB-вендоров */

/* ── Расшифровка дескрипторов (побайтно + по-человечески) ── */

/* Device descriptor: dd[0..len) (обычно 18 байт). */
void ud_dev_desc(const char *where, const uint8_t *dd, int len);

/* Configuration descriptor: полная упаковка (cfg[0..total)). */
void ud_cfg_desc(const char *where, const uint8_t *cfg, int total);

/* String descriptor: sd[0]=bLength, sd[1]=0x03, дальше UTF-16LE.
 * Печатает «...» (UTF-16LE) БЕЗ префикса — контекст печатает вызывающий. */
void ud_str_desc(const uint8_t *sd, int len);

/* ── Побитовая расшифровка регистров (одна строка в журнал) ── */

/* OHCI HcRhPortStatus (биты как в драйвере: CCS=0 PES=1 PRS=4 PPS=8
 * LSDA=9 CSC=16 PRSC=20). */
void ud_ohci_port_bits(uint32_t v);

/* EHCI PORTSC (PS_* из ehci.c: CCS=0 CSC=1 PED=2 PEC=3 OCC=5 PR=8
 * LINE=10-11 PP=12 PO=13). */
void ud_ehci_port_bits(uint32_t v);

/* xHCI PORTSC (CCS=0 PED=1 OCA=2 PR=3 PP=4 PLS=5-8 SPEED=9-12). */
void ud_xhci_port_bits(uint32_t v);

/* EHCI USBCMD+USBSTS в одну строку (RS/PSE/ASE/IAAD + IAA/HALT/ASS/PSS/
 * RECL/PCD/FLR) — для таймлайна передач [#493]. */
void ud_ehci_status_bits(uint32_t cmd, uint32_t sts);

#endif /* EGOROV_USBDUMP_H */
