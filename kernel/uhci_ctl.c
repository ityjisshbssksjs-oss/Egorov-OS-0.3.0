/* ─────────────────────────────────────────────────────────────────────────
 *  uhci_ctl.c — управляющие передачи USB и перечисление устройств.
 *
 *  Управляющая передача строится стадия за стадией (SETUP -> дата ->
 *  статус), каждая стадия — один TD во вращающемся слоте. Так устройство
 *  успевает полностью обработать SETUP до даты, а драйвер видит точную
 *  диагностику каждой стадии.
 *
 *  Data toggle — по спецификации USB:
 *    • SETUP-пакет всегда DATA0;
 *    • первый дата-пакет DATA1, далее чередование;
 *    • статусная стадия всегда DATA1, нулевая длина, обратное направление.
 *
 *  Перечисление boot-клавиатуры:
 *    сброс порта -> GET_DESCRIPTOR(device,8) на адресе 0 -> SET_ADDRESS(2)
 *    -> полный дескриптор устройства -> дескриптор конфигурации -> поиск
 *    интерфейса (class=HID, protocol=1) с interrupt-IN точкой ->
 *    SET_CONFIGURATION -> SET_IDLE -> SET_PROTOCOL(boot) -> привязка.
 * ───────────────────────────────────────────────────────────────────────── */

#include "uhci_int.h"

/* ── Управляющая передача ──
 * data == NULL означает передачу без дата-стадии (wLength = 0).
 * Возвращает wLength при успехе, отрицательный код при ошибке. */
int uk_ctl_transfer(uint8_t rt, uint8_t req, uint16_t val, uint16_t idx,
                    uint16_t wlen, void *data)
{
    volatile uhci_td_t *td;
    uint8_t *setup;
    uint32_t ls = uk.low_speed ? UK_TD_LS : 0u;

    if (!uk.attached)
        return -1;
    if ((uint32_t)wlen > UK_MAX_CTL_DATA)
        return -22;
    if (wlen && !data)
        return -22;

    /* SETUP-пакет — во вращающийся слот (8 байт, little-endian) */
    setup = uk_setup_slot();
    setup[0] = rt;
    setup[1] = req;
    setup[2] = (uint8_t)val;
    setup[3] = (uint8_t)(val >> 8);
    setup[4] = (uint8_t)idx;
    setup[5] = (uint8_t)(idx >> 8);
    setup[6] = (uint8_t)wlen;
    setup[7] = (uint8_t)(wlen >> 8);

    /* Стадия 1: SETUP — всегда DATA0 */
    td = uk_td_slot();
    td->link = UK_LINK_TERM;
    td->ctrl_sts = UK_TD_ACTIVE | UK_TD_CERR3 | ls;
    td->token = UK_TD_TOKEN(UK_PID_SETUP, uk.dev_addr, 0, 0, 8);
    td->buffer = uk_phys(setup);
    if (uk_run_td(td, "UHCI-KBD: ctl: стадия SETUP не прошла") != 0)
        return -5;

    /* Стадия 2: дата — кадрами по ep0_maxp, начиная с DATA1 */
    if (wlen) {
        uint8_t pid = (rt & USB_RT_IN) ? UK_PID_IN : UK_PID_OUT;
        uint8_t *dbuf = uk_ctl_data_buf();
        uint32_t rem = wlen;
        uint32_t off = 0;
        uint8_t tog = 1;

        while (rem) {
            uint32_t chunk = (rem > uk.ep0_maxp) ? uk.ep0_maxp : rem;

            if (pid == UK_PID_OUT)
                memcpy(dbuf, (const uint8_t *)data + off, chunk);

            td = uk_td_slot();
            td->link = UK_LINK_TERM;
            td->ctrl_sts = UK_TD_ACTIVE | UK_TD_CERR3 | ls;
            td->token = UK_TD_TOKEN(pid, uk.dev_addr, 0, tog, chunk);
            td->buffer = uk_phys(dbuf);
            if (uk_run_td(td, "UHCI-KBD: ctl: дата-стадия не прошла") != 0)
                return -5;

            if (pid == UK_PID_IN)
                memcpy((uint8_t *)data + off, dbuf, chunk);

            off += chunk;
            rem -= chunk;
            tog ^= 1;
        }
    }

    /* Стадия 3: статус — DATA1, нулевая длина, направление обратное дате */
    {
        uint8_t spid = (wlen && (rt & USB_RT_IN)) ? UK_PID_OUT : UK_PID_IN;
        td = uk_td_slot();
        td->link = UK_LINK_TERM;
        td->ctrl_sts = UK_TD_ACTIVE | UK_TD_CERR3 | ls;
        td->token = UK_TD_TOKEN(spid, uk.dev_addr, 0, 1, 0);
        td->buffer = 0;
        if (uk_run_td(td, "UHCI-KBD: ctl: статус-стадия не прошла") != 0)
            return -5;
    }

    return (int)wlen;
}

/* ── GET_DESCRIPTOR в буфер ── */
static int uk_get_desc(uint8_t dtype, uint16_t len, void *buf)
{
    return uk_ctl_transfer(USB_RT_IN, USB_REQ_GET_DESCRIPTOR,
                           (uint16_t)((uint16_t)dtype << 8), 0, len, buf);
}

/* ── Перечисление устройства на порту ──
 *   0  — boot-клавиатура найдена и привязана;
 *   1  — устройство живо, но клавиатурой не является;
 *  <0  — перечисление сорвалось. */
static int uk_enumerate_port(int port)
{
    uint8_t buf[UK_MAX_CTL_DATA];
    usb_device_desc_t *dd;
    int low = 0;
    int ok = 0;
    int t;
    uint16_t total;
    uint8_t cfgval;
    int kbd_iface = -1, ep = -1, mps = 0;
    int cur_iface_is_kbd = 0;
    uint32_t p;

    if (uk_port_reset(port, &low) != 0) {
        hal_log("UHCI-KBD: порт не включился — устройство мертво?");
        return -1;
    }
    uk.low_speed = low;
    uk.dev_addr = 0;
    uk.ep0_maxp = 8;                    /* до чтения дескриптора — безопасные 8 */

    hal_log(low ? "UHCI-KBD: порт сброшен, устройство low-speed"
                : "UHCI-KBD: порт сброшен, устройство full-speed");
    if (low && uhci_kbd_verbose)
        hal_log("UHCI-KBD: low-speed: LS-бит во всех TD, пакеты по 8 байт");

    /* 1. Короткий дескриптор устройства на адресе 0 (с ретраями —
     *    устройство сразу после сброса отвечает не сразу). */
    for (t = 0; t < 3 && !ok; t++) {
        if (t)
            hal_msleep(20);
        memset(buf, 0, 8);
        if (uk_get_desc(USB_DT_DEVICE, 8, buf) == 8 &&
            buf[0] >= 8 && buf[1] == USB_DT_DEVICE)
            ok = 1;
    }
    if (!ok) {
        hal_log("UHCI-KBD: GET_DESCRIPTOR(8) на адресе 0 не удался");
        return -1;
    }

    /* «ЭТАП 5 / #204/#294»: bMaxPacketSize0 клампится по скорости
     * устройства (uhci_mps_fix из uhci_hw.h): LS -> строго 8,
     * FS -> 8/16/32/64, мусор (0,1,3,...) -> безопасные 8.  (low уже
     * известен из uk_port_reset — запрос идёт после сброса порта.) */
    uk.ep0_maxp = uhci_mps_fix(buf[7], low);

    /* 2. Назначаем адрес 2 (адрес 0 занят «широковещательным» адресом). */
    if (uk_ctl_transfer(USB_RT_OUT, USB_REQ_SET_ADDRESS, 2, 0, 0, NULL) < 0) {
        hal_log("UHCI-KBD: SET_ADDRESS отклонён");
        return -1;
    }
    uk.dev_addr = 2;
    /* «ЭТАП F1»: 20 мс на освоение адреса (было 10 — выровнено с
     * usb_mouse.c/usb.c; noname-устройства успевают хуже). */
    hal_msleep(20);

    /* 3. Полный дескриптор устройства. */
    memset(buf, 0, 18);
    if (uk_get_desc(USB_DT_DEVICE, 18, buf) != 18) {
        hal_log("UHCI-KBD: полный дескриптор устройства не прочитан");
        return -1;
    }
    /* «ЭТАП 5 / #295»: тип полного дескриптора устройства */
    if (buf[1] != USB_DT_DEVICE) {
        hal_log("UHCI-KBD: полный dev-дескриптор: неверный bDescriptorType");
        return -1;
    }
    dd = (usb_device_desc_t *)(void *)buf;
    hal_log_hex("UHCI-KBD: VID:PID =", ((uint32_t)dd->idVendor << 16) | dd->idProduct);
    if (uhci_kbd_verbose)
        hal_log_hex("UHCI-KBD: класс устройства =", dd->bDeviceClass);

    /* 4. Конфигурация: сначала 9 байт — узнать wTotalLength. */
    memset(buf, 0, 9);
    if (uk_get_desc(USB_DT_CONFIG, 9, buf) != 9) {
        hal_log("UHCI-KBD: дескриптор конфигурации (9 байт) не прочитан");
        return -1;
    }
    /* «ЭТАП 5 / #295»: тип конфиг-дескриптора обязан быть CONFIG —
     * мусорный ответ (нули/чужие данные) не парсим. */
    if (buf[1] != USB_DT_CONFIG) {
        hal_log("UHCI-KBD: cfg-дескриптор: неверный bDescriptorType");
        return -1;
    }
    total = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    if (total < 9 || total > UK_MAX_CTL_DATA) {
        hal_log("UHCI-KBD: безумный wTotalLength — пропуск устройства");
        return -1;
    }
    /* «ЭТАП 5 / #296b» (проверено): дальше читается РОВНО total байт и
     * возврат проверяется на == total — парсинг всегда в границах
     * реально прочитанных данных, усечения не бывает (буфер 512 Б
     * покрывает все реальные конфигурации). */
    memset(buf, 0, total);
    if (uk_get_desc(USB_DT_CONFIG, total, buf) != total) {
        hal_log("UHCI-KBD: полная конфигурация не прочитана");
        return -1;
    }
    cfgval = buf[5];

    /* 5. Обход дескрипторов: ищем интерфейс (class=HID, protocol=1 —
     *    boot-клавиатура) и его interrupt-IN endpoint. Дескрипторы
     *    endpoint'ов идут следом за своим интерфейсом. */
    p = 0;
    while (p + 2 <= total) {
        uint8_t len = buf[p];
        uint8_t type = buf[p + 1];
        if (len < 2 || p + len > total)
            break;
        if (type == USB_DT_INTERFACE && len >= 9) {
            cur_iface_is_kbd =
                (buf[p + 5] == USB_CLASS_HID) && (buf[p + 7] == HID_PROTO_KBD);
            if (cur_iface_is_kbd)
                kbd_iface = buf[p + 2];
        } else if (type == USB_DT_ENDPOINT && len >= 7 && cur_iface_is_kbd) {
            uint8_t addr = buf[p + 2];
            uint8_t attr = buf[p + 3];
            if ((attr & 0x03) == 0x03 && (addr & 0x80)) {  /* interrupt IN */
                ep = addr & 0x0F;
                mps = (int)buf[p + 4] | ((int)buf[p + 5] << 8);
            }
        }
        p += len;
    }

    if (kbd_iface < 0 || ep < 0) {
        hal_log("UHCI-KBD: в конфигурации нет boot-клавиатуры");
        return 1;                        /* не клавиатура — не ошибка */
    }
    /* «ЭТАП 5 / #244» (проверено, НЕ баг): кламп int-EP MPS в 2..16 не
     * ломает FS-клавиатуры с mps=64: boot-протокол шлёт отчёты ровно
     * по 8 байт; TD с len=8 при mps устройства 64 легитимен (короткий
     * пакет в кадре нормален), actlen даёт реальную длину отчёта. */
    if (mps < 2 || mps > 16)
        mps = 8;

    /* 6. Активируем конфигурацию.
     * «ЭТАП 5 / #297»: отказ SET_CONFIGURATION ФАТАЛЕН — устройство не
     * сконфигурировано, interrupt-EP не будет отвечать (это не
     * «необязательная» настройка вроде SET_IDLE/SET_PROTOCOL).
     * «ЭТАП F1»: пауза ~20 мс ПОСЛЕ SET_CONFIGURATION — дешевые
     * noname-клавиатуры молча игнорируют запросы, посланные сразу
     * за сменой конфигурации (интерфейс ещё «переключается»). */
    if (uk_ctl_transfer(USB_RT_OUT, USB_REQ_SET_CONFIGURATION,
                        cfgval, 0, 0, NULL) < 0) {
        hal_log("UHCI-KBD: SET_CONFIGURATION отклонён");
        return -1;
    }
    hal_msleep(20);

    /* 7. HID: выключаем автоповтор (SET_IDLE 0) и включаем boot-протокол.
     *    Boot-протокол закрепляет формат отчёта 8 байт — парсер
     *    дескрипторов отчётов не нужен вовсе.
     *    bmRequestType = 0x21: классовый запрос К ИНТЕРФЕЙСУ (без бита
     *    получателя = 0x20 QEMU и реальные устройства отвечают STALL —
     *    проверено трассировкой QEMU).
     *    «ЭТАП 5 / #203/#252»: STALL на SET_IDLE/SET_PROTOCOL — НЕ провал
     *    перечисления: halt на EP0 самоочищается к следующему SETUP
     *    (CLEAR_FEATURE(ENDPOINT_HALT) для EP0 не нужен), а каждый
     *    uk_ctl_transfer начинается со стадии SETUP=DATA0 — data-toggle
     *    EP0 после STALL консистентен автоматически (interrupt-EP тоггл
     *    выставит uk_kbd_bind в DATA0).  Стандартный дескриптор отчёта
     *    клавиатуры совпадает с boot-раскладкой (модификаторы + 6 usage),
     *    поэтому живём и с STALL на SET_PROTOCOL. */
    uk_ctl_transfer(USB_RT_OUT | USB_REQTYPE_CLASS | USB_RECIPIENT_INTERFACE,
                    HID_REQ_SET_IDLE, 0, (uint16_t)kbd_iface, 0, NULL);
    if (uk_ctl_transfer(USB_RT_OUT | USB_REQTYPE_CLASS | USB_RECIPIENT_INTERFACE,
                        HID_REQ_SET_PROTOCOL, 0, (uint16_t)kbd_iface, 0, NULL) < 0) {
        hal_log("UHCI-KBD: SET_PROTOCOL(boot) STALL — живу с форматом отчёта по умолчанию");
    }

    /* 8. Привязка: ставит TD прерываний в расписание. */
    if (uk_kbd_bind(port, (uint8_t)kbd_iface, (uint8_t)ep, (uint8_t)mps) != 0)
        return -1;

    hal_log_hex("UHCI-KBD: клавиатура привязана, endpoint =", (uint32_t)ep);
    if (uhci_kbd_verbose)
        hal_log_hex("UHCI-KBD: maxpacket interrupt-EP =", (uint32_t)mps);
    return 0;
}

/* ── Публичное: захват контроллера и поиск клавиатуры на его портах ── */
int uhci_kbd_attach(uint32_t io_base, const char *name)
{
    int r = uk_core_start(io_base, name, 0u);
    if (r != 0)
        return r;
    return uhci_kbd_probe(-1);
}

int uhci_kbd_attach_shared_begin(uint32_t io_base, const char *name)
{
    return uk_core_start(io_base, name, 1u);
}

/* Скан портов: пропуск порт владельца расписания (skip_port, -1 = никого),
 * перечисление boot-клавиатуры. Неудача — «тихий» shared-старт
 * останавливается без касания USBCMD (см. uk_core_stop). */
int uhci_kbd_attached(void)
{
    return uk.attached ? 1 : 0;
}

int uhci_kbd_probe(int skip_port)
{
    for (int port = 0; port < UHCI_N_PORTS; port++) {
        if (port == skip_port)
            continue;                    /* порт занят устройством владельца */
        /* «ЭТАП F1»: за последним реальным портом PORTSC читается 0xFFFF
         * (Linux uhci-hcd): было жёстко 2 — клавиатуры в портах 3-4
         * молча не находились. */
        if (!uhci_port_exists(uk.io, port))
            break;                       /* дальше портов нет */
        uint16_t s = uk_rd16(UHCI_PORTSC(port));
        if (!(s & PORTSC_CCS))
            continue;
        hal_log_hex("UHCI-KBD: порт с устройством:", (uint32_t)port);
        if (uk_enumerate_port(port) == 0)
            return 0;                    /* клавиатура найдена        */
        /* не клавиатура или сбой — пробуем следующий порт */
    }

    hal_log("UHCI-KBD: на этом контроллере boot-клавиатуры нет");
    uk_core_stop();
    return -2;
}

/* ── Публичное: останов драйвера ── */
void uhci_kbd_detach(void)
{
    int was_shared;

    if (!uk.attached)
        return;
    uk_kbd_unbind("вызван uhci_kbd_detach()");
    was_shared = uk.shared;
    uk_core_stop();
    hal_log(was_shared ? "UHCI-KBD: отключён от чужого расписания"
                       : "UHCI-KBD: контроллер освобождён");
}
