#ifndef UHCI_KBD_UHCI_KBD_H
#define UHCI_KBD_UHCI_KBD_H

/* ─────────────────────────────────────────────────────────────────────────
 *  uhci_kbd.h — публичный API драйвера HID-клавиатуры для UHCI.
 *
 *  Драйвер переносим: всё железо спрятано за hal.h, который реализует
 *  порт окружения (Linux userspace или bare-metal ядро Egorov OS).
 *  Модель работы — чистый поллинг, без прерываний и потоков:
 *
 *      uhci_kbd_on_key(my_key_handler);        // необязательно
 *      uhci_kbd_on_report(my_report_handler);  // необязательно
 *      if (uhci_kbd_attach(io_base, "0000:00:07.2") == 0) {
 *          for (;;) { uhci_kbd_poll(); — с паузой 1..16 мс
 *          }
 *      }
 *      uhci_kbd_detach();
 *
 *  uhci_kbd_attach находит на корневых портах контроллера boot-клавиатуру
 *  (интерфейс класса HID с protocol=1) и запускает опрос interrupt-IN.
 *  Колбэки вызываются из uhci_kbd_poll(), то есть из вашего цикла.
 * ───────────────────────────────────────────────────────────────────────── */

#include "hal.h"

/* 1 = подробный журнал (перечисление, диагностика TD). */
extern int uhci_kbd_verbose;

/* Событие клавиши: usage id из таблицы HID, байт модификаторов
 * (бит0 LCtl, 1 LShift, 2 LAlt, 3 LGui, 4 RCtl, 5 RShift, 6 RAlt, 7 RGui)
 * и ASCII-перевод (-1, если клавиша непечатаемая — имя даст
 * uhci_kbd_key_name()). Вызывается только на нажатиях. */
typedef void (*uhci_kbd_key_fn)(uint8_t usage, uint8_t modifiers, int ascii);

/* Сырой 8-байтовый boot-отчёт [mods][rsv][k1..k6] на каждое событие.
 * Удобно, если разбор нажатий/отпусканий ведёт чужой парсер — например,
 * usb_kbd_parse_boot() ОС Egorov, который сам отслеживает make/break. */
typedef void (*uhci_kbd_report_fn)(const uint8_t *report, int len);

void uhci_kbd_on_key(uhci_kbd_key_fn fn);
void uhci_kbd_on_report(uhci_kbd_report_fn fn);

/* Инициализация на данном UHCI-контроллере (io_base — I/O база из PCI BAR).
 *   0  — клавиатура найдена и привязана;
 *   -2 — клавиатуры нет (контроллер освобождён);
 *   -1 / -5 / -12 — ошибка захвата / запуска / памяти. */
int  uhci_kbd_attach(uint32_t io_base, const char *name);

/* Подключение к контроллеру, которым УЖЕ владеет другой драйвер
 * (контроллер запущен, расписание чужое — например, UHCI-мышь).
 * Контроллер не сбрасывается, свои регистры не пишутся; QH прерываний
 * этого драйвера строится в собственном DMA-пуле.
 *
 * ВНИМАНИЕ — порядок обязателен (расписание должно исполнять наш QH
 * УЖЕ во время перечисления, иначе управляющие передачи не пройдут):
 *     uhci_kbd_attach_shared_begin(io, name);      // 1. тихий старт
 *     usb_mouse_sched_chain(uhci_kbd_int_qh_link()); // 2. подцепить QH
 *     uhci_kbd_probe(skip_port);                   // 3. перечислить
 * При неудаче: usb_mouse_sched_chain(1) и всё — контроллер-владелец
 * продолжает работать (probe сам останавливает наш тихий старт).
 * skip_port — корневой порт, занятый устройством владельца расписания
 * (его сброс сбил бы чужое устройство). */
int  uhci_kbd_attach_shared_begin(uint32_t io_base, const char *name);

/* Скан корневых портов и перечисление boot-клавиатуры (после
 * uhci_kbd_attach/attach_shared_begin). 0 = привязана; -2 = клавиатуры
 * нет (контроллер остановлен). */
int  uhci_kbd_probe(int skip_port);

/* Физическая ссылка (адрес | бит QH) на QH прерываний драйвера —
 * для подключения к чужому расписанию после uhci_kbd_attach_shared_begin().
 * 0 = драйвер не запущен. */
uint32_t uhci_kbd_int_qh_link(void);

/* Ежекадровый опрос. Никогда не блокирует надолго: управляющие передачи
 * короткие, а NAK обрабатывается переармом TD. */
void uhci_kbd_poll(void);

/* true, пока boot-клавиатура привязана и жива. */
int  uhci_kbd_present(void);
int  uhci_kbd_port(void);       /* 0.3.0: порт привязанной клавиатуры (-1 = нет) */

/* Порт привязанной клавиатуры на контроллере с I/O-базой io (0..3),
 * -1 — клавиатура не привязана или живёт на другом контроллере.
 * Нужен соседям по контроллеру (usb_mouse.c, um_quiesce_others):
 * гасить порт работающей клавиатуры нельзя — см. аудит #234. */
int  uhci_kbd_bound_port(uint16_t io);

/* true, пока ядро драйвера держит контроллер (устройство могло быть
 * отвязано — например, клавиатуру выдернули). Для hotplug-перепривязки:
 * в этом состоянии достаточно uhci_kbd_probe(), полный attach вернул
 * бы -16 («драйвер уже привязан»). */
int  uhci_kbd_attached(void);

/* LED-индикаторы (Num/Caps/Scroll). Драйвер и сам обновляет их при
 * нажатии соответствующих клавиш; этой функцией можно задать принудительно. */
int  uhci_kbd_set_leds(int num_lock, int caps_lock, int scroll_lock);

/* Остановить опрос, остановить контроллер, вернуть его окружению. */
void uhci_kbd_detach(void);

/* Имя непечатаемой клавиши по usage id ("Enter", "F1", "Left"...),
 * NULL для печатаемых/неизвестных. */
const char *uhci_kbd_key_name(uint8_t usage);

#endif /* UHCI_KBD_UHCI_KBD_H */
