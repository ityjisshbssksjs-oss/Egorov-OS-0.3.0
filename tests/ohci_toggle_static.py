#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ohci_toggle_static.py — статические проверки Сборки 6 (0.3.1).

#498 (повтор #481 из 0.3.0 build 10): кодировка тогглов общего TD OHCI
    обязана совпадать с Linux drivers/usb/host/ohci.h (torvalds/linux):
      TD_T_DATA0 = 0x02000000 (10b), TD_T_DATA1 = 0x03000000 (11b),
      TD_T_TOGGLE(carry) = 0x00000000.
    Пара 01b/10b (жила в 0.3.1 до Сборки 6) — ОБРАТНАЯ трактовка:
    01 = RESERVED (реальные HC трактуют по-разному), 10 = DATA0.
    SETUP с 01b «везло» через carry-подобное поведение, DATA/STATUS
    уходили DATA0 вместо DATA1 -> устройство игнорирует IN-токены ->
    cc=5 DeviceNotResponding НА КАЖДОЙ передаче (симптом «чёрных
    гнёзд» EGOROV.LOG Сборки 5, OHCI#2 порты 4/5). QEMU контрольные
    тогглы НЕ моделирует — регрессия ловится ТОЛЬКО статикой.

#495 EHCI: квитанция IAA (R/WC) ДО звона и внутри ожидания: парковка
    async-обхода по неподтверждённому IAA (EHCI 1.0 §4.8.5) не
    допускает исполнение новых qH — qTD вечно ACTIVE.
#496 строки: wValue = (0x03<<8)|index (тип в СТАРШЕМ байте).
#497 OHCI: FIT-переключение при записи HcFmInterval (Linux parity).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MH = (ROOT / "kernel" / "mouse_hw.c").read_text(encoding="utf-8")
EH = (ROOT / "kernel" / "ehci.c").read_text(encoding="utf-8")
UD = (ROOT / "kernel" / "usbdump.c").read_text(encoding="utf-8")
VER = (ROOT / "kernel" / "version.h").read_text(encoding="utf-8")

checks = {
    "#498 TD_T_DATA0 = 0x02000000 (Linux: 10b)": '#define TD_T_DATA0    (2u << 24)' in MH,
    "#498 TD_T_DATA1 = 0x03000000 (Linux: 11b)": '#define TD_T_DATA1    (3u << 24)' in MH,
    "#498 TD_T_CARRY = 0x00000000 (Linux TD_T_TOGGLE)":
        '#define TD_T_CARRY    (0u << 24)' in MH,
    "#498 прежняя обратная пара 01/10 удалена":
        '#define TD_T_DATA0    (1u << 24)' not in MH and
        '#define TD_T_DATA1    (2u << 24)' not in MH,
    "#498 комментарий ссылается на Linux ohci.h честно":
        "Linux ohci.h" in MH and "torvalds/linux" in MH,
    "#498 контрольная фаза SETUP помечена DATA0 (не carry)":
        "TD_DP_SETUP | TD_DI_NOINT | TD_T_DATA0" in MH,
    "#498 DATA-фаза помечена DATA1":
        "| dir | TD_DI_NOINT | TD_T_DATA1 | TD_ROUND" in MH,
    "#498 STATUS-фаза помечена DATA1":
        "| sdir | TD_DI_NOW | TD_T_DATA1" in MH,
    "#498 interrupt/bulk TD остаются на carry (ED_C)":
        "TD_DP_IN | TD_DI_NOW | TD_T_CARRY | TD_ROUND" in MH,

    "#495 doorbell квитирует IAA ДО звона":
        "R/WC: квитанция старого" in EH,
    "#495 eh_ctrl_run квитирует IAA до nudge":
        "квитировать протухший IAA ДО звона" in EH,
    "#495 ожидание фазы 1 квитирует приходящий IAA":
        "if (eh_r32(g_op + R_USBSTS) & STS_IAA) eh_w32(g_op + R_USBSTS, STS_IAA);" in EH,
    "#495 ожидание фазы 2 тоже квитирует IAA":
        "if (eh_r32(g_op + R_USBSTS) & STS_IAA) eh_w32(g_op + R_USBSTS, STS_IAA); /* [#495] */" in EH,

    "#496 EHCI: wValue строки = 0x0300|index":
        "(uint16_t)(0x0300u | (uint32_t)idxs[si])" in EH,
    "#496 xHCI: wValue строки = 0x0300|index":
        "(uint16_t)(0x0300u | (uint32_t)idxs[i])" in MH,
    "#496 обратная сборка (index<<8|0x03) удалена из EHCI":
        "((uint32_t)idxs[si] << 8) | 0x03u" not in EH,
    "#496 обратная сборка (index<<8|0x03) удалена из xHCI":
        "((uint32_t)idxs[i] << 8) | 0x03u" not in MH,

    "#497 FIT-переключение при записи HcFmInterval":
        "fit ^ 0x80000000u" in MH,

    "#494 декодер USBSTS: HALT=бит12, RECL=13, PSS=14, ASS=15":
        "ud_bit(sts, 0x1000u)" in UD and "ud_bit(sts, 0x8000u)" in UD and
        "ud_bit(sts, 0x2000u)" in UD and "ud_bit(sts, 0x4000u)" in UD,

    "#494 мега-лог: PCI-дамп всех устройств":
        "ud_pci_dump_all" in (ROOT / "kernel" / "usbdump.h").read_text(encoding="utf-8"),

    "сборка 0.3.1 Сборка 6": '#define EGOROV_OS_BUILD_S    "6"' in VER and
        '#define EGOROV_OS_VER        "0.3.1"' in VER,
}

fails = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(("PASS: " if ok else "FAIL: ") + name)
if fails:
    print(f"\n{len(fails)} FAILED")
    sys.exit(1)
print("\nВсе статические проверки Сборки 6 прошли (#494-#498).")
print("Тогглы контрольных труб и парковка IAA в QEMU не моделируются")
print("полностью — реальное поведение покажет EGOROV.LOG Сборки 6.")
