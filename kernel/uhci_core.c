/* ─────────────────────────────────────────────────────────────────────────
 *  uhci_core.c — движок UHCI-контроллера.
 *
 *  Здесь живёт всё, что относится к «железу» контроллера:
 *    • захват и сброс контроллера (HCRESET), программирование frame list;
 *    • расписание: каждый из 1024 слотов списка кадров указывает на QH
 *      прерываний (клавиатура), а тот по горизонтальной связи — на QH
 *      управляющих передач. Итог: клавиатура опрашивается ~1 кГц,
 *      управляющая передача доступна в каждом кадре;
 *    • сброс корневого порта (PORTSC.PR, бит 9) с QEMU-проверенной
 *      последовательностью записей;
 *    • исполнитель одиночных TD (uk_run_td) с NAK-переармом — этим
 *      владеет uhci_ctl.c, строя стадию за стадией.
 *
 *  Файл freestanding-чистый: только uhci_int.h (и, значит, hal.h).
 * ───────────────────────────────────────────────────────────────────────── */

#include "uhci_int.h"

int uhci_kbd_verbose = 0;
struct uk_state uk;

static volatile uint8_t *g_pool;      /* виртуальная база DMA-пула   */
static uint32_t g_pool_phys;          /* физическая база DMA-пула    */
static uint8_t  g_td_rot;             /* вращение контрольных TD     */
static uint8_t  g_setup_rot;          /* вращение SETUP-слотов       */

/* ── Регистры контроллера ── */
uint16_t uk_rd16(uint32_t reg)            { return hal_in16((uint16_t)(uk.io + reg)); }
void uk_wr16(uint32_t reg, uint16_t v)    { hal_out16((uint16_t)(uk.io + reg), v); }
void uk_wr32(uint32_t reg, uint32_t v)    { hal_out32((uint16_t)(uk.io + reg), v); }

/* ── Пул и физические адреса ── */
volatile uint8_t *uk_pool(void) { return g_pool; }

uint32_t uk_phys(const void *pool_ptr)
{
    return g_pool_phys +
           (uint32_t)((const volatile uint8_t *)pool_ptr - g_pool);
}

/* ── Вращающиеся слоты ──
 * UHCI читает структуры из памяти независимо от процессора и может
 * держать «закешированный» link на недавно выполненный TD. Поэтому
 * контрольные TD и SETUP-пакеты никогда не переписываются на месте:
 * каждый раз берётся следующий слот кольца. */
volatile uhci_td_t *uk_td_slot(void)
{
    volatile uhci_td_t *td =
        (volatile uhci_td_t *)(void *)(g_pool + UK_OFF_TD_ROT +
                                       (uint32_t)g_td_rot * 16u);
    g_td_rot = (uint8_t)((g_td_rot + 1u) % UK_N_CTRL_TD);
    return td;
}

uint8_t *uk_setup_slot(void)
{
    uint8_t *p = (uint8_t *)(void *)(g_pool + UK_OFF_SETUP +
                                    (uint32_t)g_setup_rot * 8u);
    g_setup_rot = (uint8_t)((g_setup_rot + 1u) % UK_N_SETUP_SLOT);
    return p;
}

uint8_t *uk_ctl_data_buf(void)
{
    return (uint8_t *)(void *)(g_pool + UK_OFF_DATA);
}

/* [#403] Кэш-когерентность DMA-пула UHCI — см. комментарий в uhci_int.h.
 * Реальные Intel-платы (ICH) держат UHCI-пул в обычной кэшируемой
 * памяти: без wbinvd контроллер читает устаревшие TD/QH, а CPU —
 * устаревший статус.  В QEMU (без кэша) вызовы безвредны. */
void uk_dma(void) { __asm__ volatile("wbinvd" ::: "memory"); }

/* ── Исполнитель одиночного TD ──
 * Ставит TD первым элементом QH управлений и ждёт завершения.
 *
 * Тонкость NAK: спецификация UHCI завершает TD по NAK (снимая Active),
 * и драйвер обязан переармить его сам; эмулятор QEMU, наоборот, сам
 * ретраит активный TD каждый кадр. Оба поведения обслуживаются здесь:
 * видим NAK без ошибок — переармливаем и продолжаем ждать. */
int uk_run_td(volatile uhci_td_t *td, const char *tag)
{
    volatile uhci_qh_t *qh =
        (volatile uhci_qh_t *)(void *)(g_pool + UK_OFF_QH_CTRL);
    uint32_t waited = 0;

    qh->elem_link = UK_LINK_TD(uk_phys((const void *)td));
    uk_dma();                    /* свежий TD/QH — в ОЗУ до старта   */

    for (;;) {
        uk_dma();                /* статус пишет контроллер в ОЗУ    */
        uint32_t sts = td->ctrl_sts;

        if (!(sts & UK_TD_ACTIVE)) {
            if (sts & UK_TD_NAK) {
                /* устройство не готово: переарм и продолжение ожидания */
                td->ctrl_sts = UK_TD_ACTIVE | UK_TD_CERR3 |
                               (uk.low_speed ? UK_TD_LS : 0u);
                qh->elem_link = UK_LINK_TD(uk_phys((const void *)td));
                uk_dma();            /* переарм тоже виден сразу    */
            } else if (sts & UK_TD_ERR_MASK) {
                hal_log(tag);
                hal_log_hex("UHCI-KBD:      отклонено, sts=", sts);
                qh->elem_link = UK_LINK_TERM;
                return -5;
            } else {
                /* успешно; кадр на «устойку» — контроллер мог держать
                 * ссылку на буфер до конца текущего кадра */
                hal_msleep(1);
                uk_dma();            /* буфер от контроллера — свежий */
                return 0;
            }
        }

        if (++waited > 250) {                 /* ~250 мс на стадию */
            hal_log(tag);
            hal_log("UHCI-KBD:      таймаут TD");
            qh->elem_link = UK_LINK_TERM;
            td->ctrl_sts &= ~UK_TD_ACTIVE;
            return -110;
        }
        hal_msleep(1);
    }
}

/* ── Сброс и включение корневого порта ──
 * Последовательность записей PORTSC (биты CSC/PEC сбрасываются записью 1):
 *   1) CSC|PEC|PR  — сбросить флаги, держать сброс (SE0 на линии);
 *   2) 0           — отпустить сброс (>= 50 мс по спецификации USB);
 *   3) PE|CSC|PEC  — включить порт (на живом железе PE обычно
 *                    выставляется сам, запись безвредна и idempotent).
 * Скорость устройства — из бита 8 (LSD): у low-speed клавиатур D- подтянута. */
int uk_port_reset(int port, int *low_speed)
{
    uint32_t reg = UHCI_PORTSC(port);
    uint16_t s = uk_rd16(reg);
    if (!(s & PORTSC_CCS))
        return -6;                            /* устройства нет */

    /* «ЭТАП 5 / #186»: дебаунс — ДВА чтения CCS вокруг паузы (как
     * usb_mouse.c): устройство, выдернувшееся за 100 мс, не сбрасываем
     * (прежний код мог держать RESET на уже пустом порту). */
    hal_msleep(100);                         /* стабилизация подключения */
    if (!(uk_rd16(reg) & PORTSC_CCS))
        return -6;                           /* дребезг/отключение */

    uk_wr16(reg, (uint16_t)(PORTSC_CSC | PORTSC_PEC | PORTSC_PR));
    hal_msleep(60);
    uk_wr16(reg, 0);
    /* «ЭТАП 3»: recovery после сброса 50 мс (TRSTRCY; было 5 мс — на
     * реальном железе устройство не успевало к первому GET_DESCRIPTOR).
     * См. usb_mouse.c — там та же правка. */
    hal_msleep(50);

    uk_wr16(reg, (uint16_t)(PORTSC_PE | PORTSC_CSC | PORTSC_PEC));
    hal_msleep(10);

    s = uk_rd16(reg);
    *low_speed = (s & PORTSC_LSD) ? 1 : 0;
    return ((s & PORTSC_PE) && (s & PORTSC_CCS)) ? 0 : -6;
}

/* ── Запуск контроллера ──
 * hook_qh_va == 0: полный захват (HCRESET, свой frame list, USBCMD=RS).
 * hook_qh_va != 0: shared-режим — контроллер уже работает под чужим
 * драйвером: регистры не трогаем, свой frame list не встраиваем;
 * QH прерываний будет подцеплен к чужому расписанию портом. */
int uk_core_start(uint32_t io_base, const char *name, uint32_t hook_qh_va)
{
    if (uk.attached) {
        hal_log("UHCI-KBD: драйвер уже привязан к контроллеру");
        return -16;
    }

    uk.io = io_base;
    uk.shared = (hook_qh_va != 0u) ? 1 : 0;
    {
        int i = 0;
        if (name)
            while (name[i] && i < (int)sizeof(uk.name) - 1) { uk.name[i] = name[i]; i++; }
        uk.name[i] = 0;
    }

    if (hal_pci_take(io_base) != 0) {
        hal_log("UHCI-KBD: не удалось захватить контроллер (root? unbind?)");
        return -1;
    }

    {
        void *v = NULL;
        uint32_t p = 0;
        if (hal_dma_alloc(UK_POOL_SIZE, &v, &p) != 0) {
            hal_log("UHCI-KBD: нет физически непрерывной памяти под DMA-пул");
            hal_pci_release(io_base);
            return -12;
        }
        g_pool = (volatile uint8_t *)v;
        g_pool_phys = p;
    }
    memset((void *)g_pool, 0, UK_POOL_SIZE);

    if (uk.shared) {
        /* Чужое расписание должно быть ЖИВЫМ (контроллер работает).
         * Остановленный контроллер в shared-режиме не оживает — здесь
         * нет кода, который его запустил бы. */
        if (uk_rd16(UHCI_USBSTS) & USBSTS_HCH) {
            hal_log("UHCI-KBD: чужое расписание мертво (HCH) — shared невозможен");
            hal_dma_free((void *)g_pool);
            g_pool = NULL;
            g_pool_phys = 0;
            return -5;
        }
    } else {
    /* Сброс контроллера. Бит HCRESET снимается сам; запись в USBCMD во
     * время сброса часть чипов игнорирует — ждём фактического снятия. */
    uk_wr16(UHCI_USBCMD, (uint16_t)USBCMD_HCRESET);
    hal_msleep(50);
    uk_wr16(UHCI_USBCMD, 0);
    for (uint32_t t = 0; t < 1000; t++) {
        if (!(uk_rd16(UHCI_USBCMD) & USBCMD_HCRESET))
            break;
        hal_msleep(1);
    }
    }

    /* Очереди:
     *   QH прерываний: elem -> (пока пусто), head -> QH управлений;
     *   QH управлений: elem -> (пока пусто), head -> конец списка.     */
    {
        volatile uhci_qh_t *qh_int =
            (volatile uhci_qh_t *)(void *)(g_pool + UK_OFF_QH_INT);
        volatile uhci_qh_t *qh_ctrl =
            (volatile uhci_qh_t *)(void *)(g_pool + UK_OFF_QH_CTRL);
        volatile uhci_td_t *td_kbd =
            (volatile uhci_td_t *)(void *)(g_pool + UK_OFF_TD_KBD);
        volatile uint32_t *fl =
            (volatile uint32_t *)(void *)(g_pool + UK_OFF_FRAMELIST);
        uint32_t entry;
        int i;

        qh_ctrl->head_link = UK_LINK_TERM;
        qh_ctrl->elem_link = UK_LINK_TERM;
        qh_int->head_link = UK_LINK_QH(uk_phys((const void *)qh_ctrl));
        qh_int->elem_link = UK_LINK_TERM;

        td_kbd->link = UK_LINK_TERM;
        td_kbd->ctrl_sts = 0;                 /* неактивен до привязки */

        entry = UK_LINK_QH(uk_phys((const void *)qh_int));
        for (i = 0; i < 1024; i++)
            fl[i] = entry;
    }

    /* Программирование и запуск — только когда владеем контроллером.
     * CF (Configure Flag) обязателен на реальном железе; MAXP — 64-байтные
     * пакеты full-speed. Прерывания не включаем: модель драйвера — поллинг.
     * В shared-режиме всё это уже сделано драйвером-владельцем. */
    if (!uk.shared) {
        uk_wr32(UHCI_FRBASE, uk_phys((const void *)(g_pool + UK_OFF_FRAMELIST)));
        uk_wr16(UHCI_FRNUM, 0);
        uk_wr16(UHCI_USBINTR, 0);
        uk_wr16(UHCI_USBSTS, (uint16_t)USBSTS_CLEAR);
        uk_wr16(UHCI_USBCMD, (uint16_t)(USBCMD_RS | USBCMD_CF | USBCMD_MAXP));
        hal_msleep(5);

        if (uk_rd16(UHCI_USBSTS) & USBSTS_HCH) {
            hal_log("UHCI-KBD: контроллер встал сразу после запуска (HCH)");
            uk_core_stop();
            return -5;
        }
    }

    /* Живость тактов: номер кадра должен идти */
    {
        uint16_t f1 = uk_rd16(UHCI_FRNUM);
        hal_msleep(3);
        if (f1 == uk_rd16(UHCI_FRNUM))
            hal_log("UHCI-KBD: предупреждение — FRNUM стоит (нет SOF?)");
    }

    uk.attached = 1;
    uk.dev_addr = 0;
    uk.ep0_maxp = 8;
    uk.low_speed = 0;
    uk.kbd_bound = 0;
    uk.kbd_errs = 0;
    uk.poll_cnt = 0;
    g_td_rot = 0;
    g_setup_rot = 0;

    if (uk.shared)
        hal_log("UHCI-KBD: подключён к чужому расписанию (контроллер не сбрасывал), слот:");
    else
        hal_log("UHCI-KBD: контроллер запущен, слот:");
    hal_log(uk.name);
    hal_log_hex("UHCI-KBD: I/O база 0x", io_base);
    return 0;
}

/* ── Останов контроллера ──
 * В shared-режиме контроллер принадлежит другому драйверу — USBCMD
 * не трогаем (иначе остановили бы и чужие устройства), снимаем только
 * свою очередь: QH прерываний отцепляется портом отдельно. */
void uk_core_stop(void)
{
    if (!uk.attached)
        return;

    {
        volatile uhci_qh_t *qh_int =
            (volatile uhci_qh_t *)(void *)(g_pool + UK_OFF_QH_INT);
        qh_int->elem_link = UK_LINK_TERM;
    }

    if (!uk.shared) {
        uk_wr16(UHCI_USBCMD, 0);              /* RS=0 — останов */
        hal_msleep(2);
    }

    uk.attached = 0;
    uk.kbd_bound = 0;
    uk.shared = 0;
    hal_pci_release(uk.io);
    if (g_pool) {
        hal_dma_free((void *)g_pool);
        g_pool = NULL;
    }
    g_pool_phys = 0;
}
