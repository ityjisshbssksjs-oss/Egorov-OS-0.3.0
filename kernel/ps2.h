#ifndef EGOROV_PS2_H
#define EGOROV_PS2_H

#include "types.h"

#define KEY_NONE   0
#define KEY_ENTER  '\n'
#define KEY_ESC    27
#define KEY_BS     8
#define KEY_TAB    '\t'
#define KEY_UP     0xF1
#define KEY_DOWN   0xF2
#define KEY_LEFT   0xF3
#define KEY_RIGHT  0xF4
#define KEY_F1     0xE1
#define KEY_F12    0xEC
#define KEY_SHIFT_LEFT  0xF5
#define KEY_SHIFT_RIGHT 0xF6

/* Добавленные константы для навигации */
#define KEY_PGUP   0xF7
#define KEY_PGDN   0xF8
#define KEY_END    0xF9
#define KEY_HOME   0xFA

/* Ctrl+J - переключение управления курсором клавиатура/мышь */
#define KEY_CTRL_J 0xFB

/* Delete (E0 53) - раньше не замаплен вовсе */
#define KEY_DEL   0xFD

/* «ЭТАП 3»: маркер ОТПУСКАНИЯ клавиши (OR к KEY_UP/DOWN/LEFT/RIGHT).
 * Нужен режиму «курсор с клавиатуры» (Ctrl+J): зажатая стрелка должна
 * двигать мышь НЕПРЕРЫВНО до отпускания — а для этого цикл должен
 * ЗНАТЬ момент отпускания. Все KEY_* ≥ 0xF1 имеют бит 7 — бит 8 (0x100)
 * свободен и коллизий не даёт. */
#define KEY_BREAK 0x100u

typedef struct {
    int x, y;
    bool left, right;
    bool moved;
    bool clicked_left;
    bool clicked_right;
} mouse_state_t;

void ps2_init(int fb_w, int fb_h);
uint32_t ps2_poll_key(void);
void ps2_poll(mouse_state_t *out);
mouse_state_t *ps2_mouse(void);
void ps2_init_usb_hid(void);
/* true, если при старте реально ответила PS/2 мышь (aux reset: ACK 0xFA + 0xAA). */
bool ps2_mouse_present(void);

/* true, пока физически удерживается Enter (make 0x1C / break 0x9C,
 * трекинг в ps2_poll_key).  Нужно для «зажатия» левой кнопки мыши в
 * режиме клавиатуры: в Paint перо не поднимается, пока Enter не отпущен. */
bool ps2_enter_down(void);

/* USB-клавиатура отключилась: сбросить её зажатия (Enter -> «ЛКМ»). */
void ps2_kbd_detached(void);


/* тихий детект клавиатуры (печатный «проверятор» удалён).
 * ps2_kbd_alive - клавиатура ответила ACK на сброс (0xFF -> 0xFA):
 *                 это настоящая PS/2-клава, USB можно трогать; false ->
 *                 USB-контроллеры трогать НЕЛЬЗЯ (handoff убьёт
 *                 BIOS-эмуляцию внутренней клавиатуры), кроме режима
 *                 «usbmouse» - там клавиатуру берёт на себя mouse_hw. */
bool ps2_kbd_alive(void);

/* положить сканкод набора 1 (make или 0x80|make = break) в очередь
 * клавиатуры.  Вызывается mouse_hw.c (USB HID-клавиатура на OHCI);
 * ps2_poll_key() разбирает очередь первой - весь остальной код ОС
 * (Ctrl+J, Enter, текстлог, терминал) работает без изменений. */
void ps2_kbd_inject(uint8_t sc);

/* [#446] текущее состояние тумблеров NumLock/CapsLock (для проб LED при
 * привязке USB-клавиатур и синхронизации индикаторов). */
void ps2_kbd_led_state(int *num, int *caps);

/* полный байт модификаторов из СВЕЖЕГО отчёта USB-клавиатуры.
 * Boot-протокол повторяет состояние в каждом пакете, поэтому Ctrl/Shift
 * на этом пути залипнуть не могут (синтетический make/break-поток ушёл). */
void ps2_kbd_mods(uint8_t mods);

/* есть ли входной байт (очередь USB-клавы или клавиатурный порт)?
 * Циклу рабочего стола нужен признак «разбирай дальше»: байты мыши
 * (aux, бит 5 статуса) ожиданием не считаются.
 * «ЭТАП 5»: true и тогда, когда пора синтезировать автоповтор
 * зажатой USB-клавиши (ps2_typematic_due). */
bool ps2_key_pending(void);

/* ══ «ЭТАП 5» (G3): софт-typematic USB-клавиатур («как в Windows») ══════
 * USB HID не шлёт повторы зажатой клавиши - ps2.c синтезирует их сам
 * (задержка ~500 мс, период ~30 мс; реализация и обоснование - блок
 * typematic в ps2.c). PS/2-клавиатуры имеют АППАРАТНЫЙ повтор, для них
 * синтеза нет. ps2_poll_key() отдаёт повторы как обычные make-события -
 * все потребители (блокнот/терминал/змейка/калькулятор) получают их
 * автоматически. */
bool ps2_typematic_due(void);    /* очередь пуста && пора повторить        */
bool ps2_typematic_held(void);   /* зажата ЛЮБАЯ USB-клавиша (для темпинга
                                  * кадра desktop-цикла: sleep 15 мс)      */

#endif
