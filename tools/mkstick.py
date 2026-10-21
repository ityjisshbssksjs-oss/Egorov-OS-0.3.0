#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build/usbstick.img — синтетическая флешка с разделом LOGS (ЭТАП 11).

[LOGS] Теперь это НАСТОЯЩИЙ FAT32-том с меткой «LOGS»: ядро пункта 2 GRUB
ищет FAT32-раздел с меткой тома «LOGS» и ДОПИСЫВАЕТ журнал в файл
EGOROV.LOG (8.3, корень) — файл читается в Windows.  Образ создаётся
инструментом (это и есть «подготовка предназначенной для журналов
области» руками пользователя); ОС НИКОГДА ничего не форматирует.

Режимы (аргумент 2):
  logs    — раздел 2, тип 0x0C FAT32 LBA, метка тома «LOGS» (по умолчанию)
  nolabel — то же, но метка тома «USB»: тест «неверный раздел» (ядро
            обязано отказаться: метка не LOGS, чужие данные не трогать)
  toobig  — MBR заявляет раздел 20480+ размером больше диска (диск 2 МБ):
            тест «недостаточная ёмкость» (границы против READ CAPACITY)
  full    — том LOGS, но ВСЕ кластеры помечены занятыми (FAT=0xFFFFFF7):
            тест «место кончилось» (честный отказ без порчи тома)

БЕЗОПАСНОСТЬ: скрипт пишет ТОЛЬКО файл по указанному пути (создаёт образ
с нуля).  Пути /dev/* отклоняются — никаких dd/форматирования физических
дисков.  Тестовый образ не связан с реальными пользовательскими дисками.

Проверка на хосте после записи журнала ядром:
    python3 - <<'EOF'
    img = open("build/usbstick.img","rb").read()
    ... или см. os/tests/e2e_logs.py (host-читалка FAT32 встроена туда)
EOF
Запуск: python3 tools/mkstick.py [путь] [режим]
"""
import sys, os

SECT = 512
PART_START = 20480          # раздел 2 (LOGS): с LBA 20480
TOTAL_SECT = 131072         # 64 МБ (режимы toobig/tiny — свои размеры)

def refuse_unsafe(path):
    """Образ — всегда обычный файл. /dev/* и block-устройства запрещены."""
    if path.startswith("/dev/") or path.startswith("/sys/") or path.startswith("/proc/"):
        raise SystemExit("ОТКАЗ: путь %s — устройство; mkstick.py создаёт только "
                         "файлы-образы и НИКОГДА не трогает физические диски" % path)
    if os.path.exists(path) and not os.path.isfile(path):
        raise SystemExit("ОТКАЗ: %s существует и не файл" % path)

def make_mbr(img, part_start, part_sects, part_type=0x0C, active=0x00):
    e = 446 + 16 * 1                     # ВТОРОЙ вход таблицы
    img[e + 0] = active                  # неактивен (не загрузочный!)
    img[e + 1] = 0x18                    # head
    img[e + 2] = 0x02                    # sector/cyl
    img[e + 3] = 0x00                    # cyl
    img[e + 4] = part_type               # 0x0C = FAT32 LBA
    img[e + 5] = 0x1A
    img[e + 6] = 0x3F
    img[e + 7] = 0x03
    img[e + 8:e + 12] = part_start.to_bytes(4, "little")
    img[e + 12:e + 16] = part_sects.to_bytes(4, "little")
    img[510:512] = b"\x55\xAA"

def make_fat32(img, p, label, spc=8, total_sect=None):
    """Собрать минимально-ПОЛНЫЙ FAT32 в разделе по смещению p (байты).

    Возвращает (root_clus, fat_lba, data_lba, clus_count).  Строим:
      VBR (BPB + метка «LOGS» + FSInfo), FAT[0..2], корень (кластер 2)
    с единственной записью — меткой тома.  Данные — свободны.
    """
    part_sect = total_sect
    # подбор FATSz32: data = part - rsvd - nfats*F; clus = data/spc;
    # FAT байт = 4*clus <= 512*F  ->  F >= (part-32)/(2*spc + 0.5)... подбираем
    rsvd, nfats = 32, 2
    fatsz = 0
    while True:
        fatsz += 1
        data = part_sect - rsvd - nfats * fatsz
        if data <= 0:
            raise SystemExit("mkstick: раздел слишком мал для FAT32")
        clus = data // spc
        if 4 * clus <= 512 * fatsz:
            break
    data_sect = part_sect - rsvd - nfats * fatsz
    clus = data_sect // spc
    # урезаем хвост, чтобы data занимал целые кластеры
    data_sect = clus * spc
    fat_lba = PART_START + rsvd
    data_lba = fat_lba + nfats * fatsz

    # ── VBR (LBA part_start) ──
    img[p + 0:p + 3]   = b"\xEB\x3C\x90"
    img[p + 3:p + 11]  = b"EGOROV  "                 # OEM
    img[p + 11:p + 13] = SECT.to_bytes(2, "little")
    img[p + 13:p + 14] = bytes([spc])
    img[p + 14:p + 16] = rsvd.to_bytes(2, "little")
    img[p + 16:p + 17] = bytes([nfats])
    img[p + 17:p + 19] = (0).to_bytes(2, "little")   # root entries (FAT32: 0)
    img[p + 19:p + 21] = (0).to_bytes(2, "little")   # total16 = 0
    img[p + 21:p + 22] = b"\xF8"                     # media
    img[p + 22:p + 24] = (0).to_bytes(2, "little")   # FATSz16 = 0 (FAT32)
    img[p + 24:p + 26] = (63).to_bytes(2, "little")
    img[p + 26:p + 28] = (255).to_bytes(2, "little")
    img[p + 28:p + 32] = (0).to_bytes(4, "little")   # hidden
    img[p + 32:p + 36] = part_sect.to_bytes(4, "little")
    img[p + 36:p + 40] = fatsz.to_bytes(4, "little") # FATSz32 (offset 0x24!)
    img[p + 40:p + 42] = (0).to_bytes(2, "little")   # ext flags
    img[p + 42:p + 44] = (0).to_bytes(2, "little")   # fs version
    img[p + 44:p + 48] = (2).to_bytes(4, "little")   # root cluster
    img[p + 48:p + 50] = (1).to_bytes(2, "little")   # FSInfo sector
    img[p + 50:p + 52] = (6).to_bytes(2, "little")   # backup boot
    img[p + 0x47:p + 0x52] = label.ljust(11).encode("ascii")[:11]
    img[p + 0x52:p + 0x57] = b"FAT32"
    img[p + 510:p + 512] = b"\x55\xAA"

    # ── FSInfo (LBA part_start+1) ──
    q = p + SECT
    img[q + 0:q + 4]     = b"\x52\x52\x61\x41"       # 'RRaA'
    img[q + 484:q + 488] = b"\x72\x72\x41\x61"       # 'rrAa'
    free_clus = clus - 1                              # кластер 2 — корень
    img[q + 488:q + 492] = free_clus.to_bytes(4, "little")
    img[q + 492:q + 496] = (3).to_bytes(4, "little") # next free
    img[q + 510:q + 512] = b"\x55\xAA"

    # ── FAT (обе копии): [0]=media/EOC, [1]=EOC, [2]=EOC (корень) ──
    for copy in range(nfats):
        f = p + (rsvd + copy * fatsz) * SECT
        img[f + 0:f + 4] = (0xFFFFFFF8).to_bytes(4, "little")
        img[f + 4:f + 8] = (0xFFFFFFFF).to_bytes(4, "little")
        img[f + 8:f + 12] = (0x0FFFFFFF).to_bytes(4, "little")

    # ── Корень (кластер 2): запись-метка тома, дальше 0x00 (конец кат.) ──
    root_off = (rsvd + nfats * fatsz) * SECT     # смещение внутри раздела
    r = p + root_off
    ent = bytearray(32)
    lbl = label.ljust(11).encode("ascii")[:11]
    ent[0:11] = lbl
    ent[11] = 0x08                                 # ATTR_VOLUME_ID
    img[r:r + 32] = bytes(ent)

    return 2, fat_lba, data_lba, clus, rsvd, nfats, fatsz

def fill_fat(img, part_start, rsvd, nfats, fatsz, clus_count, value):
    """Пометить ВСЕ кластеры данных занятыми (0x0FFFFFF7) — режим full."""
    for copy in range(nfats):
        f = (part_start + rsvd + copy * fatsz) * SECT
        for c in range(2, clus_count):
            img[f + 4 * c:f + 4 * c + 4] = value.to_bytes(4, "little")

def main(out, mode):
    refuse_unsafe(out)

    if mode == "toobig":
        total = 4096                                   # 2 МБ «диск»
        part_start, part_sects = 2048, 131072          # больше диска в разы
        img = bytearray(total * SECT)
        make_mbr(img, part_start, part_sects)
        print("toobig: диск %d секторов, раздел заявлен %d+%d — ядро обязано"
              % (total, part_start, part_sects))
        print("        ОТКЛОНИТЬ раздел (границы против READ CAPACITY)")
    elif mode == "full":
        total = 16384                                   # 8 МБ «диск»
        part_start, part_sects = 2048, total - 2048
        img = bytearray(total * SECT)
        make_mbr(img, part_start, part_sects)
        root, fat_lba, data_lba, clus, rsvd, nfats, fatsz = make_fat32(
            img, part_start * SECT, "LOGS", spc=8, total_sect=part_sects)
        # ВСЕ кластеры данных «заняты» (reserved) — как забитый том
        fill_fat(img, part_start, rsvd, nfats, fatsz, clus, 0x0FFFFFF7)
        print("full: том LOGS, все %d кластеров заняты — ядро обязано"
              % clus)
        print("       ответить «нет свободных кластеров» и НЕ портить том")
    else:
        label = "USB" if mode == "nolabel" else "LOGS"
        total = TOTAL_SECT
        part_start = PART_START
        part_sects = total - part_start
        img = bytearray(total * SECT)
        make_mbr(img, part_start, part_sects)
        root, fat_lba, data_lba, clus, rsvd, nfats, fatsz = make_fat32(
            img, part_start * SECT, label, spc=8, total_sect=part_sects)
        print("usbstick.img: %d секторов (%d МБ), раздел 2: LBA %d +%d, "
              "тип 0x0C, метка %r" % (total, total * SECT // (1024 * 1024),
                                      part_start, part_sects, label))
        print("FAT32: spc=8 (4К кластеры), rsvd=32, fats=2, корень=кластер 2, "
              "кластеров %d, свободно %d" % (clus, clus - 1))
        print("ядро: ищет FAT32 с меткой 'LOGS', создаёт EGOROV.LOG в корне")

    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    open(out, "wb").write(bytes(img))
    print("записан %s (%d байт)" % (out, len(img)))

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "build/usbstick.img"
    mode = sys.argv[2] if len(sys.argv) > 2 else "logs"
    if mode not in ("logs", "nolabel", "toobig", "full"):
        raise SystemExit("неизвестный режим: %s (logs|nolabel|toobig|full)" % mode)
    main(path, mode)
