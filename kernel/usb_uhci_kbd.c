/* ─────────────────────────────────────────────────────────────────────────
 *  usb_uhci_kbd.c — порт драйвера UHCI HID-клавиатуры на Egorov OS.
 *
 *  Здесь две вещи:
 *   1) реализация HAL (ports/egorov-os/hal.h) поверх окружения ядра:
 *      порты через io.h, задержка — калиброванный busy-wait (как в
 *      usb_mouse.c), DMA-пул — статический массив в .bss (физика 1:1),
 *      PCI — скан конфигурационного пространства через 0xCF8/0xCFC,
 *      журнал — serial_puts/serial_puthex;
 *   2) обёртка публичного API (usb_uhci_kbd_init/poll/present) с
 *      пересылкой сырых boot-отчётов в usb_kbd_parse_boot("UHCI").
 *
 *  Переносимое ядро драйвера — uhci_core.c / uhci_ctl.c / uhci_kbd.c —
 *  не знает про эту ОС ровным счётом ничего.
 * ───────────────────────────────────────────────────────────────────────── */

#include "types.h"
#include "io.h"
#include "string.h"
#include "timer.h"      /* kdelay_us(): PIT-калиброванные задержки ( hal_msleep ) */
#include "hal.h"                 /* HAL-интерфейс (egorov-вариант)        */
#include "uhci_hw.h"            /* UK_POOL_SIZE для статического пула    */
#include "uhci_kbd.h"            /* переносимое ядро драйвера            */
#include "usb_uhci_kbd.h"        /* API этого порта                      */

#include "usb_kbd.h"             /* usb_kbd_parse_boot() — общий конвейер */
#include "usb_mouse.h"           /* usb_mouse_present() — конфликт-чек    */
#include "usbdev.h"              /* реестр шести драйверов (0.3.0)        */

/* ── HAL: время ──
 * «ЭТАП F1»: переведено на kdelay_us() из kernel/timer.c —
 * PIT-калиброванные задержки (старый busy-wait был ~4х короче/дольше
 * на разном железе — USB-тайминги плыли). До timer_init() внутри
 * kdelay_us работает консервативный фолбэк — регрессии нет. */
void hal_msleep(uint32_t ms)
{
    kdelay_us(ms * 1000u);
}

/* ── HAL: журнал ── */
void hal_log(const char *msg)
{
    serial_puts("UHCIKBD: ");
    serial_puts(msg);
    serial_puts("\n");
}

void hal_log_hex(const char *msg, uint32_t value)
{
    serial_puts("UHCIKBD: ");
    serial_puts(msg);
    serial_puthex(value);
    serial_puts("\n");
}

/* ── HAL: DMA-пул ──
 * Ядро Egorov OS работает с физическими адресами 1:1 (multiboot без
 * страничной трансляции — GRUB грузит ядро по физическому адресу),
 * поэтому «физический адрес» — это просто адрес массива. 16 КБ в .bss. */
static uint8_t uk_pool_static[UK_POOL_SIZE] __attribute__((aligned(4096)));

int hal_dma_alloc(uint32_t size, void **vaddr, uint32_t *paddr)
{
    if (size > (uint32_t)sizeof uk_pool_static)
        return -1;
    *vaddr = uk_pool_static;
    *paddr = (uint32_t)uk_pool_static;   /* 1:1 */
    return 0;
}

uint32_t hal_dma_phys(void *vaddr)
{
    return (uint32_t)vaddr;              /* 1:1 */
}

void hal_dma_free(void *vaddr)
{
    (void)vaddr;
}

/* ── HAL: PCI ── */
static uint32_t pci_read32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off)
{
    outl(0xCF8, 0x80000000u | ((uint32_t)bus << 16) |
                ((uint32_t)dev << 11) | ((uint32_t)fn << 8) |
                ((uint32_t)off & 0xFCu));
    io_wait();
    return inl(0xCFC);
}

/* «ЭТАП F1» (главный фикс реального железа): запись в конфигурационное
 * пространство PCI — раньше была только чтение. */
static void pci_write32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t off, uint32_t v)
{
    outl(0xCF8, 0x80000000u | ((uint32_t)bus << 16) |
                ((uint32_t)dev << 11) | ((uint32_t)fn << 8) |
                ((uint32_t)off & 0xFCu));
    io_wait();
    outl(0xCFC, v);
}

int hal_pci_find_uhci(int index, uint32_t *io_base, char *name, int name_sz)
{
    uint32_t bus, dev, fn;
    int cur = 0;

    for (bus = 0; bus < 256; bus++) {
        for (dev = 0; dev < 32; dev++) {
            for (fn = 0; fn < 8; fn++) {
                uint32_t vid, classcode, bar;
                uint32_t base = 0;

                vid = pci_read32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0);
                if ((vid & 0xFFFFu) == 0xFFFFu)
                    break;                /* устройства на этом слоте нет */

                classcode = pci_read32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 8);
                /* UHCI: class 0x0C, subclass 0x03, prog-if 0x00.
                 * (Слово: [31:24]=class [23:16]=subclass [15:8]=prog-if.) */
                if ((classcode >> 16) != 0x0C03u ||
                    ((classcode >> 8) & 0xFFu) != 0u)
                    continue;             /* не UHCI (класс/prog-if не те) */

                if (cur++ != index)
                    continue;

                /* UHCI живёт в I/O BAR: обычно BAR4 (0x20), иногда BAR0 */
                bar = pci_read32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0x20);
                if (bar & 1u)
                    base = bar & 0xFFFFFFFCu;
                if (!base) {
                    bar = pci_read32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn, 0x10);
                    if (bar & 1u)
                        base = bar & 0xFFFFFFFCu;
                }
                if (!base) {
                    serial_puts("UHCIKBD: контроллер без I/O BAR — пропуск\n");
                    return -2;
                }

                /* «ЭТАП F1» — ГЛАВНЫЙ фикс реального железа: включить
                 * PCI Command (регистр 0x04): IO-space (бит 0) +
                 * bus-master (бит 2).  На платах, где BIOS не отдал
                 * контроллер (не включил IO+BM), UHCI «находился, но
                 * молчал»: любое обращение к I/O-баре игнорировалось.
                 * usb.c/usb_mouse.c делают это с самого начала — тут
                 * записи не было ВООБЩЕ. */
                {
                    uint32_t cmd = pci_read32((uint8_t)bus, (uint8_t)dev,
                                              (uint8_t)fn, 0x04);
                    pci_write32((uint8_t)bus, (uint8_t)dev, (uint8_t)fn,
                                0x04, cmd | 0x05u);
                    serial_puts("UHCIKBD: PCI cmd -> IO+BM\n");
                }

                if (name && name_sz > 0) {
                    /* слот в формате "BB:DD.F" */
                    char *p = name;
                    static const char hex[] = "0123456789ABCDEF";
                    *p++ = hex[(bus >> 4) & 0xF];
                    *p++ = hex[bus & 0xF];
                    *p++ = ':';
                    *p++ = hex[(dev >> 4) & 0xF];
                    *p++ = hex[dev & 0xF];
                    *p++ = '.';
                    *p++ = (char)('0' + (fn & 7));
                    *p = 0;
                }
                *io_base = base;
                return 0;
            }
        }
    }
    return -1;
}

/* Bare metal: контроллер никем не занят — захватывать нечего. */
int hal_pci_take(uint32_t io_base)     { (void)io_base; return 0; }
int hal_pci_release(uint32_t io_base)  { (void)io_base; return 0; }

/* ── Пересылка boot-отчётов в общий конвейер usb_kbd.c ──
 * usb_kbd_parse_boot сам отслеживает make/break и модификаторы,
 * а очередь ps2.c делает USB-клавиатуру неотличимой от PS/2. */
static void egorov_report(const uint8_t *report, int len)
{
    usb_kbd_parse_boot(report, len, "UHCI");
}

/* ── 0.3.2: hotplug-перепривязка клавиатуры ────────────────────────────
 * Клавиатуру выдернули и воткнули обратно (тот же разъём/контроллер)
 * или воткнули после загрузки — usb_uhci_kbd_poll() периодически
 * пробует привязать её заново. Полная проба выполняется только когда
 * на портах (кроме порта мыши) видно устройство: пустые контроллеры
 * не трогаем вовсе (два чтения PORTSC на попытку). */
static uint32_t s_io;            /* контроллер последней привязки */
static char     s_name[16];
static uint8_t  s_div;

static void kbd_remember(uint32_t io, const char *name)
{
    int k = 0;
    s_io = io;
    if (name)
        while (name[k] && k < (int)sizeof s_name - 1) {
            s_name[k] = name[k];
            k++;
        }
    s_name[k] = 0;
    usb_mouse_set_kbd_controller((uint16_t)io);
}

/* Есть ли на корневых портах (кроме skip) хоть одно устройство?
 * PORTSC.CCS — бит 0; порт n = +0x10+2n.
 * «ЭТАП F1»: порты 0..3 до первого несуществующего PORTSC (Linux
 * uhci-hcd): было жёстко 2 — устройства в портах 3-4 не замечались.
 * Ghost-фикс: 0xFFFF ИЛИ reserved-биты 13-15 — порта нет (QEMU piix3
 * возвращает 0xFF7F с CCS=1 — фантомы вечно держали yield-детект). */
static int kbd_ports_have_device(int skip)
{
    if (s_io == 0)
        return 0;
    for (int p = 0; p < 4; p++) {
        uint16_t st;
        if (p == skip)
            continue;
        st = inw((uint16_t)(s_io + 0x10u + 2u * (uint32_t)p));
        if (st == 0xFFFFu || (st & 0xE000u) != 0u)
            break;                       /* дальше портов нет (или фантом) */
        if (st & 0x0001u)
            return 1;
    }
    return 0;
}

static void kbd_try_rebind(void)
{
    int shared;
    int skip = -1;

    /* 0.3.0: реестр — клавиатурный слот занят?  Перепривязка не нужна. */
    if (usbdev_kbd_bound()) {
        if (s_io != 0 && kbd_ports_have_device(-1))
            serial_puts("UHCIKBD: клавиатура уже есть (реестр usbdev) -\n"
                        "UHCIKBD: вторая клавиатура: НЕИЗВЕСТНОЕ УСТРОЙСТВО\n");
        return;
    }

    if (usb_mouse_uhci_busy() && usb_mouse_uhci_io() == (uint16_t)s_io) {
        shared = 1;
        skip = usb_mouse_uhci_port();
    } else {
        shared = 0;
    }
    if (!kbd_ports_have_device(skip))
        return;                  /* пусто — контроллер не дёргаем */

    serial_puts("UHCIKBD: hotplug: устройство в порту - перепривязываю\n");

    /* Гигиена парсера: отпустить зажатые клавиши прошлой сессии
     * (break-коды) и обнулить состояние — журнал «первый отчёт»
     * загорится заново, видно живой поток данных. */
    usb_kbd_parse_boot(0, -1, "UHCI");
    usb_kbd_init();

    /* Ядро УЖЕ держит контроллер (отвязали только устройство: выдернули
     * клавиатуру)? Полный uhci_kbd_attach() вернул бы -16 вечно, а
     * неудача сбрасывала владение — и перескан мыши тут же сбрасывал
     * контроллер под ногами (замкнутый цикл). Достаточно перепробовать
     * порты уже на живом расписании. */
    if (uhci_kbd_attached()) {
        if (uhci_kbd_probe(skip) == 0) {
            (void)usb_uhci_kbd_registry_ok();   /* 0.3.0: реестр */
            kbd_remember(s_io, s_name);
        } else if (!uhci_kbd_attached()) {
            if (shared)
                usb_mouse_sched_chain(1);   /* отцепиться от расписания мыши */
            usb_mouse_set_kbd_controller(0);
        }
        return;
    }

    if (shared) {
        if (uhci_kbd_attach_shared_begin(s_io, s_name) == 0) {
            usb_mouse_sched_chain(uhci_kbd_int_qh_link());
            if (uhci_kbd_probe(skip) == 0) {
                (void)usb_uhci_kbd_registry_ok();   /* 0.3.0: реестр */
                kbd_remember(s_io, s_name);
                return;
            }
            usb_mouse_sched_chain(1);
        }
        usb_mouse_set_kbd_controller(0);
        return;
    }

    if (uhci_kbd_attach(s_io, s_name) == 0)
        kbd_remember(s_io, s_name);
    else
        usb_mouse_set_kbd_controller(0);
}

/* ── Публичный API порта ── */
void usb_uhci_kbd_init(void)
{
    int idx;

    /* 0.3.0: клавиатурный слот реестра уже занят (другая шина)? —
     * UHCI-клавиатуру не ищем вовсе. */
    if (usbdev_kbd_bound()) {
        serial_puts("UHCIKBD: клавиатура уже есть (реестр usbdev) - не ищу\n");
        return;
    }

    /* Состояние парсера usb_kbd.c не должно зависеть от мусора в ОЗУ. */
    usb_kbd_init();

    /* Отчёты драйвера уходят в общий конвейер usb_kbd.c ("UHCI"). */
    uhci_kbd_on_report(egorov_report);

    for (idx = 0; idx < 8; idx++) {
        uint32_t io;
        char name[16];
        int r = hal_pci_find_uhci(idx, &io, name, (int)sizeof name);

        if (r == -1)
            break;                       /* больше UHCI нет */
        if (r != 0)
            continue;

        serial_puts("UHCIKBD: пробую контроллер ");
        serial_puts(name);
        serial_puts("\n");

        /* Контроллер занят драйвером UHCI-мыши (usb_mouse.c)? Тогда
         * НЕ сбрасываем его и не строим свой frame list, а работаем в
         * общем расписании. Порядок обязателен: сначала «тихий» старт
         * (QH строится), ЗАТЕМ подцепить наш QH за QH мыши — и только
         * потом перечислять: управляющие передачи перечисления должны
         * уже исполняться из общего расписания. */
        if (usb_mouse_uhci_busy() && usb_mouse_uhci_io() == (uint16_t)io) {
            int mport = usb_mouse_uhci_port();

            serial_puts("UHCIKBD: контроллер занят UHCI-мышью — цепляюсь к её расписанию\n");

            if (uhci_kbd_attach_shared_begin(io, name) == 0) {
                usb_mouse_sched_chain(uhci_kbd_int_qh_link());
                serial_puts("UHCIKBD: очередь клавиатуры в общем расписании с мышью\n");

                if (uhci_kbd_probe(mport) == 0) {
                    (void)usb_uhci_kbd_registry_ok();   /* 0.3.0: реестр */
                    kbd_remember(io, name);   /* 0.3.2: для hotplug */
                    return;              /* клавиатура привязана */
                }

                /* Не вышло — отцепиться от чужого расписания (тихий
                 * старт уже остановлен внутри probe). */
                usb_mouse_sched_chain(1);
            }
            continue;                    /* следующий контроллер */
        }

        if (uhci_kbd_attach(io, name) == 0) {
            (void)usb_uhci_kbd_registry_ok();   /* 0.3.0: реестр */
            kbd_remember(io, name);      /* 0.3.2: для hotplug */
            return;                      /* клавиатура привязана */
        }
        /* не повезло — пробуем следующий контроллер */
    }

    serial_puts("UHCIKBD: boot-клавиатура по UHCI не найдена\n");
}

/* 0.3.0: «порт, кто ты?» + «драйвер клавиатуры 1.x/2.0» из ШЕСТИ.
 * Вызывается ядром uhci_kbd после УСПЕШНОЙ привязки; при отказе
 * реестра (слот занят) возвращает 0 и ядро отвязывается. */
int usb_uhci_kbd_registry_ok(void)
{
    int ver = usbdev_port_version(0x00);
    int kp = uhci_kbd_port();   /* [#438] настоящий порт */
    if (kp < 0) kp = 0;
    usbdev_ask_port("UHCI", ver, kp + 1);   /* [#458] 1-базированный, как в карте */
    return usbdev_try_bind(ver, 1, 0, "UHCI", kp + 1, 0, 0)
           == USBDEV_ROLE_KBD;
}

/* 0.3.0: клавиатура выдернута — освободить слот реестра. */
void usb_uhci_kbd_registry_detach(void)
{
    usbdev_release(USBDEV_ROLE_KBD, "UHCI kbd detach");
}

void usb_uhci_kbd_poll(void)
{
    /* 0.3.0: детект отключения — освобождаем слот реестра сразу. */
    static int s_was;
    int is = uhci_kbd_present() != 0;
    if (s_was && !is)
        usbdev_release(USBDEV_ROLE_KBD, "UHCI kbd detach");
    s_was = is;

    /* 0.3.0: «уступка контроллера».  Клава живёт на UHCI, слот МЫШИ
     * свободен, а в СОСЕДНЕМ порту появилось устройство (опрос каждую
     * секунду это видит)?  Отвязываемся -> rescan мыши поднимает её
     * ПЕРВОЙ (как при загрузке) -> kbd_try_rebind возвращает НАС в
     * общем расписании (shared-режим).  Так работает и подключение
     * мыши к контроллеру клавиатуры, и ЛЮБЫЕ переподключения.
     *
     * «ЭТАП 5» (найдено воспроизведением add→del→add): сэмпл CCS и
     * edge-состояние ведём ВСЕГДА — и когда слот мыши ЗАНЯТ тоже.
     * Раньше весь блок был под `!usbdev_mouse_bound()`: пока мышь
     * была привязана, флаг «уже уступали» НЕ сбрасывался даже по
     * спаду CCS.  Сценарий «выдернул мышь и БЫСТО воткнул обратно»
     * (деактивация по 2048 ошибок занимала секунды, повторное
     * подключение приходило раньше): s_yield_ccs оставался = 1 →
     * «уже уступали — ждём переподключения» → НАВСЕГДА.  Теперь
     * сброс флага: чужие порты опустели ИЛИ мышь владела слотом —
     * обе ситуации означают «уступка завершила свою работу». */
    {
        static uint8_t s_yield_ccs;   /* чужое CCS-устройство уже отработано уступкой */
        int kp = uhci_kbd_port();
        uint8_t ccs = 0;
        if (kp >= 0 && s_io != 0) {
            /* «ЭТАП F1»: сосед — ЛЮБОЙ другой порт (не только kp^1:
             * на 4-портовых UHCI устройство может появиться и в 3-4).
             * Ghost-фикс: фантомные порты (0xFFFF / reserved-биты
             * 13-15, QEMU 0xFF7F) устройством НЕ считаются. */
            for (int p = 0; p < 4; p++) {
                uint16_t st;
                if (p == kp)
                    continue;
                st = inw((uint16_t)(s_io + 0x10u + 2u * (uint32_t)p));
                if (st == 0xFFFFu || (st & 0xE000u) != 0u)
                    break;           /* дальше портов нет (или фантом) */
                if (st & 0x0001u)
                    ccs = 1;
            }
        }
        /* сброс «уже уступали»: порты опустели ИЛИ слот мыши занят
         * (мышь поднялась по нашей уступке — цикл завершён). */
        if (!ccs || usbdev_mouse_bound())
            s_yield_ccs = 0;

        if (is && !usbdev_mouse_bound() && s_io != 0 && kp >= 0 &&
            !s_yield_ccs && kbd_ports_have_device(kp)) {
            s_yield_ccs = 1;              /* до смены устройства/мыши — один раз */
            serial_puts("UHCIKBD: в соседнем порту устройство - уступаю контроллер мыши\n");
            uhci_kbd_detach();
            usbdev_release(USBDEV_ROLE_KBD, "уступаю контроллер мыши");
            usb_mouse_set_kbd_controller(0);
            s_was = 0;
            usb_mouse_rescan_now();       /* мышь поднимется сейчас же */
            kbd_try_rebind();             /* а мы вернёмся в общем расписании */
            return;
        }
    }

    uhci_kbd_poll();

    /* раз в ~1024 опросов — не появилась ли клавиатура заново;
     * ГЛАВНЫЙ секундный шанс даёт usbdev_tick(). */
    if (s_io != 0 && !is && (++s_div & 0x3FFu) == 0u)
        kbd_try_rebind();
}

bool usb_uhci_kbd_present(void)
{
    return uhci_kbd_present() ? true : false;
}
