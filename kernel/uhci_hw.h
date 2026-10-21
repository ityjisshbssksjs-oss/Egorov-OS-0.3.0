#ifndef UHCI_KBD_UHCI_HW_H
#define UHCI_KBD_UHCI_HW_H

/* ─────────────────────────────────────────────────────────────────────────
 *  uhci_hw.h — «железная» часть UHCI: регистры, Transfer Descriptor,
 *  Queue Head, кодирование link-указателей и слов token/status.
 *
 *  Контроллер UHCI (Intel, USB 1.1, PCI-класс 0x0C0300) отображает
 *  регистры в I/O-пространстве (обычно BAR4, окно 0x20 байт) и работает
 *  со структурами в физически непрерывной памяти: списком кадров
 *  (frame list, 1024 элемента по 4 байта) и очередями QH/TD.
 *
 *  Раскладки полей сверены с рабочими реализациями (Linux uhci-hcd,
 *  эмулятор QEMU hw/usb/hcd-uhci.c, драйвер usb_mouse.c ОС Egorov) и с
 *  osdev-вики. Мелочи, о которых легко споткнуться:
 *
 *   • Порт сбрасывается битом 9 PORTSC (не 6/8, как в OHCI/EHCI!).
 *   • Скорость устройства читается из бита 8 PORTSC (LSD).
 *   • Слово token: pid[7:0] addr[14:8] ep[18:15] toggle[19] maxlen[31:21];
 *     поле maxlen хранит (длина-1), 0x7FF означает нулевую длину.
 *   • Пакетный PID задаётся СЫРЫМ байтом: SETUP=0x2D, IN=0x69, OUT=0xE1
 *     (контроллер достраивает инверсные биты сам).
 *   • В управляющей передаче: SETUP всегда DATA0; первый дата-пакет —
 *     DATA1, далее чередование; статусная стадия — всегда DATA1.
 *   • На реальном железе NAK деактивирует TD — драйвер обязан переармить
 *     его сам; QEMU, наоборот, оставляет TD активным и ретраит сам.
 * ───────────────────────────────────────────────────────────────────────── */

#include "hal.h"

/* ── Регистры UHCI (смещения от I/O базы) ── */
#define UHCI_USBCMD    0x00u   /* R/W, 16 бит                        */
#define UHCI_USBSTS    0x02u   /* R/W1C, 16 бит                      */
#define UHCI_USBINTR   0x04u   /* R/W, 16 бит (работаем без прерываний) */
#define UHCI_FRNUM     0x06u   /* R/W, 16 бит                        */
#define UHCI_FRBASE    0x08u   /* R/W, 32 бит: физ. адрес frame list */
#define UHCI_SOFMOD    0x0Cu   /* R/W, 8 бит                         */
#define UHCI_PORTSC(n) (0x10u + 2u * (uint32_t)(n))  /* 16 бит, порт n */
/* «ЭТАП F1»: регистра числа портов у UHCI НЕТ — порты сканируют до
 * первого несуществующего PORTSC (так делает Linux uhci-hcd):
 * у PIIX3 их 2, но некоторые контроллеры имеют 4.  Верхняя граница
 * скана — 4; за ней PORTSC всё равно читается 0xFFFF. */
#define UHCI_N_PORTS   4       /* лимит скана; реальные — до 0xFFFF */

/* Порт «существует»?  Два признака несуществующего порта:
 *   1) PORTSC == 0xFFFF — за пределами реализованных регистров
 *      (классическое поведение реального железа);
 *   2) взведены reserved-биты 13-15 — по спеке UHCI они читаются
 *      ВСЕГДА 0.  QEMU на несуществующих портах piix3 возвращает
 *      0xFF7F: бит CCS=1 («устройство подключено!») — драйвер
 *      гонял reset+перечисление ФАНТОМОВ и отравлял edge-детект
 *      уступки клавиатуры («уже уступали» — реальная мышь в порту
 *      1 больше не замечалась).  Бит 12 (Suspend) — легитимный,
 *      его не трогаем. */
static inline int uhci_port_exists(uint32_t io, int n)
{
    uint16_t st = hal_in16((uint16_t)(io + UHCI_PORTSC(n)));
    if (st == 0xFFFFu) return 0;
    if (st & 0xE000u)  return 0;   /* reserved биты взведены — фантом */
    return 1;
}

/* USBCMD */
#define USBCMD_RS      (1u << 0)   /* Run/Stop                              */
#define USBCMD_HCRESET (1u << 1)   /* сброс контроллера, снимается сам      */
#define USBCMD_GRESET  (1u << 2)   /* глобальный сброс шины — не используем */
#define USBCMD_CF      (1u << 6)   /* Configure Flag — нужен реальному HW   */
#define USBCMD_MAXP    (1u << 7)   /* 64-байтные пакеты на full-speed       */

/* USBSTS */
#define USBSTS_CLEAR   0x003Fu     /* записать 1 для сброса флагов 0..5 */
#define USBSTS_HCH     (1u << 5)   /* контроллер остановлен (RO)        */

/* PORTSC */
#define PORTSC_CCS     (1u << 0)   /* устройство подключено (RO)              */
#define PORTSC_CSC     (1u << 1)   /* изменение подключения (сброс записью 1)  */
#define PORTSC_PE      (1u << 2)   /* порт включён                             */
#define PORTSC_PEC     (1u << 3)   /* изменение включения (сброс записью 1)    */
#define PORTSC_LSD     (1u << 8)   /* подключено low-speed устройство (RO)     */
#define PORTSC_PR      (1u << 9)   /* Port Reset: 1 = держать сброс на порту   */
#define PORTSC_SUSP    (1u << 12)  /* порт в suspend                           */

/* ── Transfer Descriptor: 16 байт, выравнивание 16 ──
 * Поля volatile: контроллер меняет их параллельно с драйвером. */
typedef struct {
    volatile uint32_t link;      /* следующий TD/QH (горизонтальная связь) */
    volatile uint32_t ctrl_sts;  /* слово управления/статуса               */
    volatile uint32_t token;     /* PID, адрес, точка, toggle, длина       */
    volatile uint32_t buffer;    /* физ. адрес буфера данных               */
} uhci_td_t;

/* ── Queue Head: 8 байт, выравнивание 16 ── */
typedef struct {
    volatile uint32_t head_link; /* горизонтальная связь (следующая очередь) */
    volatile uint32_t elem_link; /* вертикальная связь (первый TD очереди)   */
} uhci_qh_t;

/* ── Кодирование link-указателей ──
 * Адрес выровнен на 16, младшие биты — флаги:
 *   бит0 = T (1 = конец списка), бит1 = Q (1 = ссылка на QH). */
#define UK_LINK_TERM   1u
#define UK_LINK_QH(pa) (((pa) & 0xFFFFFFF0u) | 2u)
#define UK_LINK_TD(pa) ((pa) & 0xFFFFFFF0u)

/* ── PID-байты USB (кладутся в младшие 8 бит token) ── */
#define UK_PID_SETUP 0x2Du
#define UK_PID_IN    0x69u
#define UK_PID_OUT   0xE1u

/* ── Слово token ──
 *   pid[7:0] | devaddr[14:8] | endpoint[18:15] | D-toggle[19] | maxlen[31:21]
 *   Поле maxlen хранит (длина-1); 0x7FF — нулевая длина (статусная стадия). */
#define UK_TD_TOKEN(pid, addr, ep, tog, len)                            \
    ((uint32_t)(pid) |                                                   \
     ((uint32_t)((uint32_t)(addr) & 0x7Fu) << 8) |                       \
     ((uint32_t)((uint32_t)(ep)   & 0x0Fu) << 15) |                      \
     ((tog) ? (1u << 19) : 0u) |                                         \
     (((len) ? ((uint32_t)(len) - 1u) : 0x7FFu) << 21))

/* ── Слово control/status ── */
#define UK_TD_BITSTUFF (1u << 17)  /* ошибка бит-стаффинга                    */
#define UK_TD_CRCTIMO  (1u << 18)  /* CRC-ошибка / таймаут ответа             */
#define UK_TD_NAK      (1u << 19)  /* устройство ответило NAK                 */
#define UK_TD_BABBLE   (1u << 20)  /* babble — линия «залипла»                */
#define UK_TD_DBUFERR  (1u << 21)  /* ошибка DMA-буфера                       */
#define UK_TD_STALLED  (1u << 22)  /* endpoint ответил STALL                  */
#define UK_TD_ACTIVE   (1u << 23)  /* TD ждёт исполнения контроллером         */
#define UK_TD_IOC      (1u << 24)  /* прерывание по завершении — не используем */
#define UK_TD_LS       (1u << 26)  /* транзакция с low-speed устройством      */
#define UK_TD_CERR3    (3u << 27)  /* счётчик ошибок: 3 ретрая                */
#define UK_TD_SPD      (1u << 29)  /* short-packet detect — не используем     */

#define UK_TD_ERR_MASK (UK_TD_BITSTUFF | UK_TD_CRCTIMO | UK_TD_BABBLE |    \
                        UK_TD_DBUFERR | UK_TD_STALLED)

/* Фактическая длина выполненной передачи: поле actlen хранит N-1
 * (0x7FF = нулевая длина) — та же конвенция, что и у maxlen. */
static inline uint32_t uk_td_actlen(uint32_t sts)
{
    uint32_t r = sts & 0x7FFu;
    return (r == 0x7FFu) ? 0u : r + 1u;
}

/* ── Валидация bMaxPacketSize0 по скорости устройства ──
 * Аудит #204/#294: поле bMaxPacketSize0 читается из дескриптора (данные
 * с шины!) и без проверки попадало в размер чанков дата-стадии EP0.
 * Правила спецификации USB 9.6.1: low-speed устройство обязано иметь
 * EP0=8; full-speed допускает 8/16/32/64.  Мусор (0, 1, 3, 128…)
 * заменяем безопасными 8 — перечисление продолжается на мелких пакетах.
 * usb.c и usb_mouse.c держат собственную копию: включение uhci_hw.h
 * туда конфликтует с их локальными TD/QH-структурами (volatile/раскладка). */
static inline uint8_t uhci_mps_fix(uint8_t mps, int ls)
{
    if (ls)
        return 8;                          /* low-speed: только 8 */
    if (mps == 8 || mps == 16 || mps == 32 || mps == 64)
        return mps;
    return 8;                              /* нечисловой мусор -> 8 */
}

/* ── Раскладка DMA-пула драйвера ──
 * Один физически непрерывный, 4КБ-выровненный блок. Смещения фиксированы
 * здесь, чтобы ядро драйвера не зависело от аллокатора порта.
 *
 *   0x0000  frame list — 1024 * 4 байта
 *   0x1000  QH прерываний (клавиатура)
 *   0x1010  QH управляющих передач
 *   0x1020  постоянный TD клавиатуры
 *   0x1030  32 вращающихся TD управляющих передач
 *   0x1240  4 вращающихся слота SETUP-пакетов (8 байт каждый)
 *   0x1280  буфер данных управляющих передач (512 байт)
 *   0x1800  буфер отчёта клавиатуры (16 байт)
 *   0x1810  байт LED-отчёта
 */
#define UK_POOL_SIZE     0x4000u
#define UK_OFF_FRAMELIST 0x0000u
#define UK_OFF_QH_INT    0x1000u
#define UK_OFF_QH_CTRL   0x1010u
#define UK_OFF_TD_KBD    0x1020u
#define UK_OFF_TD_ROT    0x1030u
#define UK_N_CTRL_TD     32u
#define UK_OFF_SETUP     0x1240u
#define UK_N_SETUP_SLOT  4u
#define UK_OFF_DATA      0x1280u
#define UK_MAX_CTL_DATA  512u
#define UK_OFF_REPORT    0x1800u
#define UK_OFF_LED       0x1810u

#endif /* UHCI_KBD_UHCI_HW_H */
