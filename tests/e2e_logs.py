#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2E-стенд [LOGS] ЭТАП 11: журнал ядра → EGOROV.LOG на USB-накопителе.

QEMU 10 headless + QMP (sendkey для выхода из textlog).  Каждый сценарий
грузит ядро с cmdline «textlog sticklog» (пункт 2 GRUB) на СИНТЕТИЧЕСКОМ
дисковом образе (tools/mkstick.py; образ — обычный файл в /tmp/e2e,
НИКАКОЙ связи с реальными пользовательскими дисками) и проверяет:

  • serial-маркеры честной диагностики (найден/раздел/файл/итог);
  • САМ ОБРАЗ — хост-читалка FAT32 ниже: MBR → BPB → FAT (обе копии) →
    корень → EGOROV.LOG (цепочка кластеров сходится с размером, записи
    каталога валидны, содержимое содержит маркеры журнала).

Сценарии (пункт ТЗ №10 — все восемь требуемых + дополнительные, build 4):
  L1  UHCI  + том LOGS: запись и чтение EGOROV.LOG (полная проверка тома)
  L2  OHCI  + том LOGS: то же через OHCI-транспорт (реальное железо AMD)
  L3  xHCI  + том LOGS: то же через xHCI-транспорт (ЭТАП 11, новые ПК)
  L4  нет накопителя: честный «ИТОГ: накопитель НЕ найден», ОС жива
  L5  неверный раздел (метка «USB»): отказ, файл НЕ создан
  L6  недостаточная ёмкость (раздел за границей диска): отклонён, отказ
  L7  место кончилось (все кластеры заняты): честный отказ без порчи тома
  L8  сбой записи (readonly): честный отказ, ОС жива
  L9  USB-транзакция умерла (device_del): честный отказ, ОС жива
  L10 повторная загрузка (L10+L10b): тот же образ дважды — ДОПОЛНЕНИЕ
      (не потеря): размер растёт, обе сессии в файле, префикс сохранён
  L11 переполнение журнала (logcap=4 КБ): лимит файла честно достигнут
  L12 ДВЕ флешки: первая (OHCI) БЕЗ раздела LOGS, вторая (UHCI) с ним —
      ядро обязано ПЕРЕЙТИ ко второй, файл создан ТОЛЬКО на ней
      (проблема 1 ревью build 3: выбор второй флешки)
  L13 потеря старта (logearly=1 КБ): честный маркер «ПОТЕРЯНО N байт»
      в файле и в ИТОГе; полный журнал НЕ заявляется (проблема 3)

Проверка полноты (build 4): файл обязан содержать САМЫЙ ПЕРВЫЙ баннер
ядра «=== Egorov OS … kernel_main ===» — ранний буфер держит журнал с
первого байта; в файле НЕ должно быть маркеров «ДЫРА»/«ПОТЕРЯНО» (L1-L3).

Запуск: python3 tests/e2e_logs.py [подстроки имён сценариев]
"""
import subprocess, socket, json, time, os, sys, re, shutil

# переносимый поиск QEMU — как в e2e_audit.py [#477]
def find_qemu():
    env = os.environ.get("EGOROV_QEMU")
    if env and os.path.isfile(env) and os.access(env, os.X_OK):
        return env
    for name in ("qemu-system-i386", "qemu-system-x86_64"):
        p = shutil.which(name)
        if p:
            return p
    fallback = "/tmp/bin/qemu-system-i386"
    if os.path.isfile(fallback) and os.access(fallback, os.X_OK):
        return fallback
    return ""

QEMU = find_qemu()
HERE = os.path.dirname(os.path.abspath(__file__))
MKSTICK = os.path.join(HERE, "..", "tools", "mkstick.py")
KERNEL = os.path.abspath(os.path.join(HERE, "..", "build", "egorov.stage1"))
WORK = "/tmp/e2e"
os.makedirs(WORK, exist_ok=True)

NODEF = ["-nodefaults", "-vga", "std", "-m", "128"]

# ═══ хост-читалка FAT32: разбор тома и чтение EGOROV.LOG ═════════════════
def fat32_read(imgpath, want_label="LOGS"):
    """Разобрать образ: вернуть dict с томом (метка want_label) и EGOROV.LOG.

    Не зависит от кода ОС (двойная проверка): MBR → раздел с меткой
    «LOGS» → BPB → FAT → корень → EGOROV.LOG → цепочка → данные.
    Возвращает None, если тома нет; file=None, если файла нет.
    """
    img = open(imgpath, "rb").read()
    def r32(o): return int.from_bytes(img[o:o+4], "little")
    def r16(o): return int.from_bytes(img[o:o+2], "little")

    assert img[510:512] == b"\x55\xAA", "нет MBR-подписи 55AA"
    logs = None
    for i in range(4):
        e = 446 + 16 * i
        if img[e + 4] == 0 or img[e + 4] == 0xEE:
            continue
        st, sz = r32(e + 8), r32(e + 12)
        if sz == 0 or st == 0:
            continue
        p = st * 512
        if p + 512 > len(img):
            continue
        if img[p + 0x52:p + 0x57] != b"FAT32":
            continue
        label = img[p + 0x47:p + 0x52].decode("ascii", "replace")
        if label.strip() == want_label:
            logs = (st, sz, label)
            break
    if logs is None:
        return None
    st, sz, label = logs
    p = st * 512
    spc, rsvd, nfats = img[p + 13], r16(p + 14), img[p + 16]
    fatsz, rootc = r32(p + 0x24), r32(p + 0x2C)
    fat_lba = st + rsvd
    data_lba = fat_lba + nfats * fatsz
    clus_bytes = spc * 512
    clus_count = (st + sz - data_lba) // spc + 2

    def fat_entry(c, copy=0):
        off = (fat_lba + copy * fatsz) * 512 + 4 * c
        return r32(off) & 0x0FFFFFFF

    def read_chain(first, limit=4096):
        chain, c, seen = [], first, set()
        while 2 <= c < clus_count and c not in seen and len(chain) < limit:
            seen.add(c)
            chain.append(c)
            c = fat_entry(c)
        return chain, c

    root_chain, _ = read_chain(rootc, 64)
    entry = None
    for c in root_chain:
        base = (data_lba + (c - 2) * spc) * 512
        for s in range(spc):
            sec = base + s * 512
            for o in range(0, 512, 32):
                if sec + o + 32 > len(img):
                    break
                fb = img[sec + o]
                if fb == 0x00:
                    break
                if fb == 0xE5:
                    continue
                attr = img[sec + o + 11]
                if attr == 0x0F or (attr & 0x08):
                    continue
                if img[sec + o:sec + o + 11] == b"EGOROV  LOG":
                    entry = (sec, o)
                    break
            if entry:
                break
        if entry:
            break
    if entry is None:
        return {"label": label, "file": None}

    sec, o = entry
    clus = r16(sec + o + 20) << 16 | r16(sec + o + 26)
    size = r32(sec + o + 28)
    attr = img[sec + o + 11]
    chain, eoc = read_chain(clus) if clus >= 2 else ([], 0)
    data = b""
    for c in chain:
        base = (data_lba + (c - 2) * spc) * 512
        data += img[base:base + clus_bytes]
    data = data[:size]
    fat2_ok = all(fat_entry(c, 1) == fat_entry(c, 0) for c in chain)
    need = (size + clus_bytes - 1) // clus_bytes
    return {
        "label": label, "fat2_ok": fat2_ok,
        "file": {
            "attr": attr, "clus": clus, "size": size,
            "chain": chain, "eoc": eoc, "need": need,
            "chain_ok": len(chain) == need and eoc >= 0x0FFFFFF8,
            "data": data,
        },
    }

# ═══ мини-обёртка QEMU+QMP (как e2e_audit.QV, но ждёт маркер sticklog) ═══
class LV:
    def __init__(self, name, extra, append="textlog sticklog"):
        self.name = name
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
        self.log = f"{WORK}/{safe}.log"
        self.qmp = f"{WORK}/{safe}.qmp"
        for f in (self.log, self.qmp):
            if os.path.exists(f):
                os.unlink(f)
        args = [QEMU, "-kernel", KERNEL, "-display", "none",
                "-serial", f"file:{self.log}",
                "-qmp", f"unix:{self.qmp},server,nowait",
                "-no-reboot"] + extra
        if append:
            args += ["-append", append]
        self.p = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.STDOUT)
        self.sock = None
        for _ in range(100):
            if os.path.exists(self.qmp):
                try:
                    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    s.settimeout(30)
                    s.connect(self.qmp)
                    self.f = s.makefile("rw", encoding="utf-8", newline="\n")
                    self.f.readline()
                    self.cmd("qmp_capabilities")
                    self.sock = s
                    return
                except Exception:
                    pass
            time.sleep(0.1)
        raise RuntimeError(f"{self.name}: QMP не подключился")

    def cmd(self, execute, **arguments):
        msg = {"execute": execute}
        if arguments:
            msg["arguments"] = arguments
        self.f.write(json.dumps(msg) + "\n")
        self.f.flush()
        while True:
            line = self.f.readline()
            if not line:
                return None
            r = json.loads(line)
            if "return" in r or "error" in r:
                return r

    def sendkey(self, keys):
        self.cmd("human-monitor-command", **{"command-line": f"sendkey {keys}"})

    def dev_del(self, did):
        return self.cmd("device_del", id=did)

    def logtext(self):
        try:
            with open(self.log, encoding="utf-8", errors="replace") as f:
                return f.read()
        except FileNotFoundError:
            return ""

    def wait_marker(self, pattern, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if re.search(pattern, self.logtext()):
                return True
            if self.p.poll() is not None:
                return False
            time.sleep(0.3)
        return False

    def kill(self):
        try:
            self.p.kill()
        except Exception:
            pass
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        try:
            self.p.wait()
        except Exception:
            pass

def make_img(path, mode):
    r = subprocess.run([sys.executable, MKSTICK, path, mode],
                       capture_output=True, text=True)
    assert r.returncode == 0, f"mkstick {mode}: {r.stderr}"

# ═══ проверочные функции для образа ══════════════════════════════════════
def img_written_ok(tr_tag):
    """Том LOGS + EGOROV.LOG записан корректно (структуры + контент).

    build 4: журнал проверяется С ПЕРВОГО БАЙТА — в файле обязан быть
    самый первый баннер ядра («kernel_main»), заголовок сессии с транспортом,
    строка «журнал: ПОЛНЫЙ…» и НЕ должно быть маркеров потерь.
    """
    def f(img, log):
        v = fat32_read(img)
        assert v is not None, "том LOGS не найден в образе"
        fl = v["file"]
        assert fl is not None, "EGOROV.LOG не найден в корне"
        assert fl["attr"] & 0x20 and not (fl["attr"] & 0x18), \
            f"attr записи {fl['attr']:#x} не файл"
        assert fl["size"] > 1000, f"размер {fl['size']} слишком мал"
        assert fl["chain_ok"], (f"цепочка {len(fl['chain'])} != нужно "
                                f"{fl['need']} или EOC {fl['eoc']:#x}")
        assert v["fat2_ok"], "копия FAT 2 не совпадает с FAT 1"
        d = fl["data"]
        assert d.startswith(b"\xef\xbb\xbf"), "нет UTF-8 BOM (создание)"
        assert f"(USB transport {tr_tag})".encode() in d, "нет заголовка сессии"
        assert b"hw_init done" in d or b"USBDEV" in d, "нет журнала в файле"
        # [build 4] ЖУРНАЛ С ПЕРВОГО БАЙТА: самый первый баннер ядра — в
        # файле (ранний буфер дотянул старт до готовности флешки)
        assert re.search(rb"=== Egorov OS [^\n]* kernel_main ===", d), \
            "нет ПЕРВОГО баннера ядра — журнал не с первого байта!"
        # [build 4] честность: потерь быть НЕ должно (полный журнал)
        assert "ДЫРА".encode() not in d, "в файле маркер потери (ДЫРА) — не полный"
        assert "журнал: ПОЛНЫЙ с первого байта".encode() in d, \
            "нет строки «журнал: ПОЛНЫЙ с первого байта» в заголовке"
        try:
            d.decode("utf-8")
        except UnicodeDecodeError as e:
            raise AssertionError(f"журнал в файле не UTF-8 (порча): {e}")
        assert b"USBDEV" in d or b"TEXTLOG" in d, "нет поздних строк журнала"
        return True
    return f

def img_file_none():
    """Том «USB» (неверный раздел) есть, EGOROV.LOG НЕ создан."""
    def f(img, log):
        v = fat32_read(img, want_label="USB")
        assert v is not None, "том с чужой меткой не найден"
        return v["file"] is None
    return f

def img_grew(prev_content):
    """Файл ДОПОЛНЕН: размер больше, старый префикс сохранён байт-в-байт."""
    def f(img, log):
        v = fat32_read(img)
        assert v is not None and v["file"] is not None
        d = v["file"]["data"]
        assert len(d) > len(prev_content), \
            f"файл не вырос ({len(d)} <= {len(prev_content)})"
        assert d[:len(prev_content)] == prev_content, \
            "старое содержимое затёрто (нарушение append-only!)"
        assert d.count(b"LOGS session") >= 2, "нет второй сессии в файле"
        assert v["file"]["chain_ok"], "цепочка не сходится после дополнения"
        return True
    return f

def img_gap_marker():
    """L13: в EGOROV.LOG есть честный маркер потери (ДЫРА: потеряно N байт)."""
    def f(img, log):
        v = fat32_read(img)
        assert v is not None and v["file"] is not None, "том/файл нет"
        d = v["file"]["data"]
        assert re.search(r"=== LOGS: ДЫРА: потеряно \d+ байт".encode(), d), \
            "в файле нет маркера ДЫРА (честная потеря не отмечена!)"
        assert "журнал: ПОТЕРЯНО".encode() in d, "в заголовке нет строки ПОТЕРЯНО"
        assert "журнал: ПОЛНЫЙ".encode() not in d, "заявлен ПОЛНЫЙ журнал при потере!"
        return True
    return f

# ═══ конфигурации устройств ═══════════════════════════════════════════════
def uhci_dev(img):
    return NODEF + ["-usb",
        "-drive", f"if=none,id=ustick,file={img},format=raw",
        "-device", "usb-storage,drive=ustick"]

def ohci_dev(img):
    return NODEF + ["-device", "pci-ohci,id=oh0",
        "-drive", f"if=none,id=ustick,file={img},format=raw",
        "-device", "usb-storage,drive=ustick,bus=oh0.0"]

def xhci_dev(img):
    return NODEF + ["-device", "qemu-xhci,id=x0",
        "-drive", f"if=none,id=ustick,file={img},format=raw",
        "-device", "usb-storage,drive=ustick,bus=x0.0"]

def esc_later(v):
    v.sendkey("esc")

def imgpath(name):
    return f"{WORK}/{re.sub(r'[^A-Za-z0-9_.-]+', '_', name)}.img"

# ═══ сценарии: (имя, режим mkstick, extra_fn, шаги, serial-проверки,
#                проверки образа, append) ═══════════════════════════════════
TESTS = [
 ("L1 UHCI: запись и чтение EGOROV.LOG", "logs", uhci_dev,
  [(7.0, esc_later), (3.0, None)],
  [("накопитель", r"LOGS: накопитель 1/1: UHCI"),
   ("выбран", r"LOGS: ВЫБРАН накопитель 1/1: UHCI"),
   ("раздел", r"LOGS: раздел LOGS НАЙДЕН"),
   ("файл-создан", r"EGOROV\.LOG СОЗДАН"),
   ("итог-записано", r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG"),
   ("полный-журнал", r"LOGS: ИТОГ: журнал ПОЛНЫЙ с первого байта загрузки"),
   ("!потерь", r"ПОТЕРЯНО \d|ДЫРА"),
   ("резерв-жив", r"COM1 и RAM-кольцо \(COREINFO\) работают независимо"),
   ("!отказов-выгрузки", r"выгрузка не удалась")],
  [("том и файл валидны", img_written_ok("UHCI"))],
  "textlog sticklog"),

 ("L2 OHCI: запись и чтение EGOROV.LOG", "logs", ohci_dev,
  [(7.0, esc_later), (3.0, None)],
  [("накопитель", r"LOGS: накопитель 1/1: OHCI"),
   ("выбран", r"LOGS: ВЫБРАН накопитель 1/1: OHCI"),
   ("раздел", r"LOGS: раздел LOGS НАЙДЕН"),
   ("файл-создан", r"EGOROV\.LOG СОЗДАН"),
   ("итог-записано", r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG"),
   ("полный-журнал", r"LOGS: ИТОГ: журнал ПОЛНЫЙ с первого байта загрузки"),
   ("!потерь", r"ПОТЕРЯНО \d|ДЫРА"),
   ("!отказов-выгрузки", r"выгрузка не удалась")],
  [("том и файл валидны", img_written_ok("OHCI"))],
  "textlog sticklog"),

 ("L3 xHCI: запись и чтение EGOROV.LOG", "logs", xhci_dev,
  [(7.0, esc_later), (3.0, None)],
  [("накопитель", r"LOGS: накопитель 1/1: xHCI"),
   ("выбран", r"LOGS: ВЫБРАН накопитель 1/1: xHCI"),
   ("раздел", r"LOGS: раздел LOGS НАЙДЕН"),
   ("файл-создан", r"EGOROV\.LOG СОЗДАН"),
   ("итог-записано", r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG"),
   ("полный-журнал", r"LOGS: ИТОГ: журнал ПОЛНЫЙ с первого байта загрузки"),
   ("!потерь", r"ПОТЕРЯНО \d|ДЫРА"),
   ("!отказов-выгрузки", r"выгрузка не удалась")],
  [("том и файл валидны", img_written_ok("xHCI"))],
  "textlog sticklog"),

 ("L4 нет накопителя: честный отказ", None, lambda img: NODEF,
  [(6.0, esc_later), (2.0, None)],
  [("не-найден", r"LOGS: ИТОГ: накопитель НЕ найден"),
   ("резерв", r"резервные каналы \(COM1 \+ RAM-кольцо 64К\) работают всегда"),
   ("итог-не-записано", r"LOGS: ИТОГ: НЕ ЗАПИСАНО: накопитель не найден"),
   ("ОС-жива", r"-> рабочий стол|desktop_run")],
  [],
  "textlog sticklog"),

 ("L5 неверный раздел (метка USB)", "nolabel", uhci_dev,
  [(6.0, esc_later), (2.0, None)],
  [("метка-не-та", r"метка не 'LOGS' - раздел НЕ предназначен"),
   ("следующий-носитель", r"на этом носителе раздела LOGS нет - проверяю следующий"),
   ("отказ", r"LOGS: ИТОГ: раздел LOGS НЕ найден"),
   ("защита", r"НИЧЕГО не записано - чужие разделы под защитой")],
  [("файл не создан", img_file_none())],
  "textlog sticklog"),

 ("L6 недостаточная ёмкость (раздел за диском)", "toobig", uhci_dev,
  [(6.0, esc_later), (2.0, None)],
  [("за-границами", r"ВЫХОДИТ ЗА ГРАНИЦЫ НОСИТЕЛЯ"),
   ("отклонён", r"ОТКЛОНЁН"),
   ("итог-не-записано", r"LOGS: ИТОГ: НЕ ЗАПИСАНО: раздел LOGS не найден")],
  [],
  "textlog sticklog"),

 ("L7 место кончилось (кластеры заняты)", "full", uhci_dev,
  [(8.0, esc_later), (3.0, None)],
  [("нет-кластеров", r"НЕДОСТАТОЧНО МЕСТА: нет свободных кластеров"),
   ("файл-цел", r"файл остаётся целым"),
   ("резерв", r"COM1/RAM живут")],
  [("том не испорчен",
    lambda img, log: fat32_read(img) is not None)],
  "textlog sticklog"),

 ("L8 сбой записи (readonly)", "logs",
  lambda img: NODEF + ["-usb",
      "-drive", f"if=none,id=ustick,file={img},format=raw,readonly=on",
      "-device", "usb-storage,drive=ustick"],
  [(10.0, esc_later), (4.0, None)],
  [("накопитель-есть", r"LOGS: накопитель 1/1: UHCI"),
   ("раздел-есть", r"LOGS: раздел LOGS НАЙДЕН"),
   ("честный-отказ", r"ИТОГ: НЕ ЗАПИСАНО: запись каталога не создана|"
                      r"выгрузка не удалась \(3/3 подряд\)|"
                      r"ВЫГРУЗКА ОСТАНОВЛЕНА"),
   ("резерв-жив", r"COM1 и RAM-кольцо \(COREINFO\) работают независимо"),
   ("ОС-жива", r"-> рабочий стол|desktop_run")],
  [],
  "textlog sticklog"),

 ("L9 USB-транзакция умерла (device_del)", "logs",
  lambda img: NODEF + ["-usb",
      "-drive", f"if=none,id=ustick,file={img},format=raw",
      "-device", "usb-storage,drive=ustick,id=ust"],
  [(7.0, lambda v: v.dev_del("ust")),
   (10.0, esc_later), (3.0, None)],
  [("накопитель-есть", r"LOGS: накопитель 1/1: UHCI"),
   ("раздел-есть", r"LOGS: раздел LOGS НАЙДЕН"),
   ("честный-отказ", r"выгрузка не удалась \(\d/3 подряд\)|"
                       r"FAT не читается|каталог не обновлён|"
                       r"ВЫГРУЗКА ОСТАНОВЛЕНА"),
   ("ОС-жива", r"-> рабочий стол|desktop_run")],
  [],
  "textlog sticklog"),

 # L10/L10b обрабатываются в main() как двухтактный сценарий
 ("L10 повторная загрузка: первая сессия", "logs", uhci_dev,
  [(7.0, esc_later), (3.0, None)],
  [("первый-итог", r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG")],
  [],
  "textlog sticklog"),

 ("L11 переполнение журнала (logcap=4КБ)", "logs", uhci_dev,
  [(7.0, esc_later), (3.0, None)],
  [("лимит", r"EGOROV\.LOG достиг лимита \(4096 байт\)"),
   ("честно-про-резерв", r"журнал продолжает идти в COM1 и RAM-кольце"),
   ("итог-лимит", r"файл достиг лимита/места - выгрузка остановлена")],
  [],
  "textlog sticklog logcap=4"),

 # L12: ДВЕ флешки — первая (OHCI, метка «USB») без раздела LOGS,
 # вторая (UHCI) с ним.  Проверка проблемы 1 ревью build 3: ядро обязано
 # ПЕРЕЙТИ к следующему накопителю и создать файл ТОЛЬКО на втором.
 # (режим/образы — спец-обработка в main(), тут только маркеры)
 ("L12 две флешки: выбор ВТОРОЙ (раздел LOGS)", None, None,
  [(7.0, esc_later), (3.0, None)],
  [("найдено-2", r"LOGS: найдено накопителей: 2"),
   ("первый-ohci", r"LOGS: накопитель 1/2: OHCI"),
   ("первый-пропущен", r"на этом носителе раздела LOGS нет - проверяю следующий"),
   ("второй-uhci", r"LOGS: накопитель 2/2: UHCI"),
   ("выбран-второй", r"LOGS: ВЫБРАН накопитель 2/2: UHCI"),
   ("итог-записано", r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG"),
   ("полный-журнал", r"LOGS: ИТОГ: журнал ПОЛНЫЙ с первого байта загрузки"),
   ("!записи-на-первой", r"ВЫБРАН накопитель 1/2")],
  [],
  "textlog sticklog"),

 # L13: ранний буфер урезан до 1 КБ и кольцо до 8 КБ (logearly=1 logring=8)
 # — старт журнала НЕ влезает; ядро обязано честно написать «ПОТЕРЯНО N
 # байт» (в файл маркер ДЫРЫ и в ИТОГ) и НЕ заявлять полный журнал.
 ("L13 потеря старта: честный маркер (logearly=1КБ)", "logs", uhci_dev,
  [(7.0, esc_later), (3.0, None)],
  [("потеря-при-подготовке", r"LOGS: ВНИМАНИЕ: до готовности носителя ПОТЕРЯНО \d+ байт"),
   ("итог-потеряно", r"LOGS: ИТОГ: ПОТЕРЯНО \d+ байт"),
   ("итог-не-полный", r"журнал НЕ полный"),
   ("!заявления-о-полноте", r"ИТОГ: журнал ПОЛНЫЙ"),
   ("записано-частично", r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG")],
  [("маркер ДЫРЫ в файле", img_gap_marker())],
  "textlog sticklog logearly=1 logring=8"),
]


def run_one(name, extra, steps, checks, imgchecks, append, img=None):
    print(f"── {name} ", end="", flush=True)
    v = LV(name, extra, append)
    try:
        if not v.wait_marker(r"TEXTLOG: пункт 2|LOGS: ИТОГ: накопитель НЕ найден|"
                             r"LOGS: раздел LOGS НЕ найден", 60):
            print(f"BOOT FAIL ({len(v.logtext())} байт)")
            return False
        ok = True
        for delay, fn in steps:
            time.sleep(delay)
            if fn:
                fn(v)
        time.sleep(2.0)
        log = v.logtext()
        for label, pattern in checks:
            if label.startswith("!"):
                if re.search(pattern, log):
                    print(f"\n    FAIL: недопустимый «{label[1:]}»")
                    ok = False
                continue
            if not re.search(pattern, log):
                print(f"\n    FAIL: нет маркера «{label}» ({pattern!r})")
                ok = False
        if ok and imgchecks and img:
            try:
                for label, fn in imgchecks:
                    if not fn(img, log):
                        print(f"\n    FAIL(образ): {label}")
                        ok = False
            except AssertionError as e:
                print(f"\n    FAIL(образ): {e}")
                ok = False
        if ok:
            print("PASS")
        return ok
    finally:
        v.kill()

def main():
    if not QEMU:
        print("ОШИБКА СТЕНДА: QEMU не найден (см. tests/e2e_audit.py [#477])")
        sys.exit(2)
    if not os.path.isfile(KERNEL):
        print("ОШИБКА СТЕНДА: ядра нет — сначала собери ОС: make")
        sys.exit(2)
    print(f"стенд: QEMU = {QEMU}")
    print(f"       ядро = {KERNEL}")
    print(f"       образы = {WORK}/*.img (синтетика mkstick.py; /dev/* запрещён)")
    only = sys.argv[1:] if len(sys.argv) > 1 else None
    results = []

    for t in TESTS:
        name, mode, extra_fn, steps, checks, imgchecks, append = t
        if only and not any(o in name for o in only):
            continue

        # ── L12: ДВА образа (первый — «чужая» флешка без LOGS, второй —
        # целевая).  Проверяем: файл создан ТОЛЬКО на второй.
        if name.startswith("L12 "):
            bad = imgpath("L12a первая флешка без LOGS (метка USB)")
            good = imgpath("L12b вторая флешка с разделом LOGS")
            for p in (bad, good):
                if os.path.exists(p):
                    os.unlink(p)
            make_img(bad, "nolabel")
            make_img(good, "logs")
            extra = NODEF + [
                "-device", "pci-ohci,id=oh0",
                "-drive", f"if=none,id=badstick,file={bad},format=raw",
                "-device", "usb-storage,drive=badstick,bus=oh0.0",
                "-device", "piix3-usb-uhci,id=uh0",   # ЯВНЫЙ UHCI: без него
                # usb-storage без bus= садится на первую попавшуюся шину
                # (oh0) — обе флешки на OHCI, транспорт держит один MSD
                "-drive", f"if=none,id=ustick,file={good},format=raw",
                "-device", "usb-storage,drive=ustick,bus=uh0.0"]

            def img12(good_path):
                def f(img, log):
                    vg = fat32_read(good_path)
                    assert vg is not None, "на второй флешке нет тома LOGS"
                    assert vg["file"] is not None, \
                        "EGOROV.LOG не создан на второй флешке"
                    d = vg["file"]["data"]
                    assert re.search(rb"=== Egorov OS [^\n]* kernel_main ===", d), \
                        "журнал на второй флешке не с первого байта"
                    assert "ДЫРА".encode() not in d, "непредвиденная потеря на второй"
                    return True
                return f

            def img12_bad(bad_path):
                def f(img, log):
                    vb = fat32_read(bad_path, want_label="USB")
                    assert vb is not None, "первая флешка (метка USB) не видна"
                    assert vb["file"] is None, \
                        "EGOROV.LOG создан на ПЕРВОЙ флешке — нарушен выбор!"
                    return True
                return f

            results.append((name, run_one(
                name, extra, steps, checks,
                [("файл на второй, полного покрытия", img12(good)),
                 ("на первой файла НЕТ", img12_bad(bad))],
                append, good)))
            continue

        img = None
        if mode:
            img = imgpath(name)
            if os.path.exists(img):
                os.unlink(img)
            make_img(img, mode)
        results.append((name, run_one(name, extra_fn(img) if img else extra_fn(None),
                                      steps, checks, imgchecks, append, img)))

        # ── L10b: вторая загрузка НА ТОМ ЖЕ образе (дополнение) ──
        if name.startswith("L10 ") and results[-1][1]:
            v1 = fat32_read(img)
            if v1 and v1["file"]:
                prev = v1["file"]["data"]
                name2 = "L10b повторная загрузка: дополнение"
                checks2 = [
                    ("найден-дополнение",
                     r"EGOROV\.LOG найден, размер \d+ - ДОПОЛНЯЮ \(не перезаписываю\)"),
                    ("итог-записано",
                     r"LOGS: ИТОГ: ЗАПИСАНО \d+ байт в EGOROV\.LOG"),
                    ("!перезаписи", r"НЕ перезаписываю чужие|затёрто"),
                ]
                imgchecks2 = [("файл вырос, префикс цел", img_grew(prev))]
                results.append((name2, run_one(
                    name2, uhci_dev(img), [(7.0, esc_later), (3.0, None)],
                    checks2, imgchecks2, "textlog sticklog", img)))
            else:
                results.append(("L10b повторная загрузка", False))

    print("\n════ ИТОГ [LOGS] ════")
    npass = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        npass += ok
    print(f"{npass}/{len(results)} PASS")
    sys.exit(0 if npass == len(results) else 1)

if __name__ == "__main__":
    main()
