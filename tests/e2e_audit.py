#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2E-стенд Egorov OS 0.3.0 «ANYPORT+» (после аудита 185 багов).

QEMU 10 headless + QMP: инъекция клавиш (sendkey/input-send-event),
движения мыши (mouse_move), hotplug (device_add/device_del).
Проверка — по serial-маркерам (первый отчёт/typematic/НЕИЗВЕСТНОЕ).

[#477] ЭТАП 10: ИНСТРУМЕНТЫ ИЩУТСЯ ПЕРЕНОСИМО:
  1) переменная окружения EGOROV_QEMU (полный путь);
  2) PATH (qemu-system-i386 / qemu-system-x86_64);
  3) восстановленный вручную стенд /tmp/bin/qemu-system-i386.
  Отсутствие QEMU — ошибка СТЕНДА, а не USB-драйвера: сообщение
  говорит, ЧТО поставить, и не роняет результаты тестов молча.

Сценарии (порт × устройство × ситуация):
  T1  UHCI: клава+мышь (порты 1.x у piix3)
  T2  EHCI: оба напрямую (high-speed)
  T3  EHCI#1 клава + EHCI#2 мышь (два контроллера, как AMD FCH)
  T4  q35 «порты 2.0»: FS-клава+мышь через компаньонов ICH9
  T5  OHCI: оба (AMD FCH модель)
  T6  xHCI: оба (порты 3.0)
  T7  клава 2.0 (EHCI) + мышь 3.0 (xHCI)
  T8  клава 3.0 (xHCI) + мышь 2.0 (EHCI)
  T9  hotplug: мышь add→del→add при живой клаве (UHCI)
  T10 НЕИЗВЕСТНОЕ: флешка на свободном порту
  T11 typematic USB: Ctrl+J + удержание стрелки
  T12 PS/2-клава + USB-мышь + typematic по PS/2
  T13 ЭТАП 9 [#466/#467]: ТРИ OHCI — мышь на ТРЕТЬЕМ контроллере
      (регрессия «oi < 2»: раньше отчёты 3-го/4-го OHCI не читались)
  T14 ЭТАП 9 [#465]: ТРИ xHCI — мышь на ТРЕТЬЕМ контроллере
      (регрессия «кольца [2]»: раньше g_xi=2 писал за границы массива)
  T15 ЭТАП 10 [#474]: ЖИВАЯ мышь xHCI + перечисление ВТОРОГО
      устройства (usb-tablet с мышиным интерфейсом): реестр отклоняет,
      а конфигурация живой мыши (MPS/EP/раскладка/absolute) раньше
      ПОРТИЛАСЬ скрэтчем перечисления → мышь «видна, но не работает».
  T16 ЭТАП 10 [#475]: «нулевые БАЙТЫ ≠ нулевая ДЛИНА»: отчёт без
      нажатых клавиш (все байты 0) — легальный boot-отчёт (Ситуация A),
      НЕ ошибка и НЕ «нулевая длина» (Ситуация B): маркер
      «НУЛЕВОЙ ДЛИНЫ» появиться НЕ должен (негативная проверка «!»).
"""
import subprocess, socket, json, time, os, sys, signal, re, shutil

# ── [#477] переносимый поиск QEMU ─────────────────────────────────────
def find_qemu():
    """env EGOROV_QEMU → PATH → /tmp/bin. Возвращает путь или ''."""
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
KERNEL = os.path.join(os.path.dirname(__file__), "..", "build", "egorov.stage1")
KERNEL = os.path.abspath(KERNEL)
WORK = "/tmp/e2e"
os.makedirs(WORK, exist_ok=True)

NODEF = ["-nodefaults", "-vga", "std", "-m", "128"]

class QV:
    def __init__(self, name, extra):
        self.name = name
        # безопасное имя для путей: QEMU парсит -qmp unix:PATH[,opts]
        # по запятым/равно — пробелы и '=' в пути убивают его
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
        self.log = f"{WORK}/{safe}.log"
        self.qmp = f"{WORK}/{safe}.qmp"
        for f in (self.log, self.qmp):
            if os.path.exists(f): os.unlink(f)
        args = [QEMU, "-kernel", KERNEL, "-display", "none",
                "-serial", f"file:{self.log}",
                "-qmp", f"unix:{self.qmp},server,nowait"] + extra
        self.p = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.STDOUT)
        self.sock = None
        self._connect()

    def _connect(self):
        for _ in range(100):
            if os.path.exists(self.qmp):
                try:
                    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    s.settimeout(30)
                    s.connect(self.qmp)
                    self.f = s.makefile("rw", encoding="utf-8", newline="\n")
                    self.f.readline()  # greeting
                    self.cmd("qmp_capabilities")
                    self.sock = s
                    return
                except Exception:
                    pass
            time.sleep(0.1)
        raise RuntimeError(f"{self.name}: QMP не подключился")

    def cmd(self, execute, **arguments):
        msg = {"execute": execute}
        if arguments: msg["arguments"] = arguments
        self.f.write(json.dumps(msg) + "\n"); self.f.flush()
        while True:
            line = self.f.readline()
            if not line: return None
            r = json.loads(line)
            if "return" in r or "error" in r: return r

    def hmp(self, line):
        return self.cmd("human-monitor-command", **{"command-line": line})

    def sendkey(self, keys):
        self.hmp(f"sendkey {keys}")

    def key(self, qcode, down):
        return self.cmd("input-send-event", events=[
            {"type": "key", "data": {"down": down,
             "key": {"type": "qcode", "data": qcode}}}])

    def mouse_move(self, dx, dy):
        self.hmp(f"mouse_move {dx} {dy}")

    def dev(self, action, driver, did, port=None):
        if action == "add":
            kw = {"driver": driver, "id": did}
            if port: kw["port"] = str(port)
            return self.cmd("device_add", **kw)
        return self.cmd("device_del", id=did)

    def logtext(self):
        try:
            with open(self.log, encoding="utf-8", errors="replace") as f:
                return f.read()
        except FileNotFoundError:
            return ""

    def wait_boot(self, timeout=60):
        """Дождаться конца инициализации USB (desktop запущен)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if "секундный опрос портов ЗАПУЩЕН" in self.logtext():
                return True
            if self.p.poll() is not None:
                return False
            time.sleep(0.3)
        return False

    def kill(self):
        try: self.p.send_signal(signal.SIGKILL)
        except Exception: pass
        if self.sock:
            try: self.sock.close()
            except Exception: pass
        self.p.wait()

def run_test(t):
    name, extra, steps, checks = t
    print(f"── {name} ", end="", flush=True)
    v = QV(name, extra)
    try:
        if not v.wait_boot():
            print(f"BOOT FAIL (лог {len(v.logtext())} байт)")
            return False
        ok = True
        for delay, fn in steps:
            time.sleep(delay)
            fn(v)
        time.sleep(2.0)              # дать отчётам дойти
        log = v.logtext()
        for label, pattern in checks:
            # [#477] негативные проверки: метка начинается с «!» —
            # паттерн НЕ должен встречаться в журнале.
            if label.startswith("!"):
                if re.search(pattern, log):
                    print(f"FAIL: найден недопустимый «{label[1:]}» ({pattern!r})")
                    ok = False
                continue
            if not re.search(pattern, log):
                print(f"FAIL: нет маркера «{label}» ({pattern!r})")
                ok = False
        if ok: print("PASS")
        return ok
    finally:
        v.kill()

# ── конфигурации устройств ────────────────────────────────────────────
UHCI_BOTH = NODEF + ["-usb",
    "-device", "usb-kbd,port=1", "-device", "usb-mouse,port=2"]
EHCI_BOTH = NODEF + ["-device", "usb-ehci,id=eh0",
    "-device", "usb-kbd,bus=eh0.0,port=1",
    "-device", "usb-mouse,bus=eh0.0,port=2"]
EHCI_TWO = NODEF + ["-device", "usb-ehci,id=eh0", "-device", "usb-ehci,id=eh1",
    "-device", "usb-kbd,bus=eh0.0,port=1",
    "-device", "usb-mouse,bus=eh1.0,port=1"]
Q35_USB20 = NODEF + ["-M", "q35",
    "-device", "ich9-usb-uhci1,bus=pcie.0,addr=1d.0,id=uh1,multifunction=on",
    "-device", "ich9-usb-uhci2,bus=pcie.0,addr=1d.1,id=uh2",
    "-device", "ich9-usb-uhci3,bus=pcie.0,addr=1d.2,id=uh3",
    "-device", "ich9-usb-ehci1,bus=pcie.0,addr=1d.7,id=eh0",
    "-device", "usb-kbd,bus=eh0.0,port=3",
    "-device", "usb-mouse,bus=eh0.0,port=4"]
OHCI_BOTH = NODEF + ["-device", "pci-ohci,id=oh0",
    "-device", "usb-kbd,bus=oh0.0,port=1",
    "-device", "usb-mouse,bus=oh0.0,port=2"]
XHCI_BOTH = NODEF + ["-device", "qemu-xhci,id=x0",
    "-device", "usb-kbd,bus=x0.0,port=1",
    "-device", "usb-mouse,bus=x0.0,port=2"]
K2M3 = NODEF + ["-device", "usb-ehci,id=eh0", "-device", "qemu-xhci,id=x0",
    "-device", "usb-kbd,bus=eh0.0,port=1",
    "-device", "usb-mouse,bus=x0.0,port=1"]
K3M2 = NODEF + ["-device", "qemu-xhci,id=x0", "-device", "usb-ehci,id=eh0",
    "-device", "usb-kbd,bus=x0.0,port=1",
    "-device", "usb-mouse,bus=eh0.0,port=1"]
HOTPLUG = NODEF + ["-usb", "-device", "usb-kbd,port=1"]
UNKNOWN = NODEF + ["-usb",
    "-device", "usb-kbd,port=1", "-device", "usb-mouse,port=2",
    "-device", "usb-ehci,id=eh1",
    "-drive", "if=none,id=ud,file=/tmp/e2e/stick.img,format=raw",
    "-device", "usb-storage,drive=ud,bus=eh1.0"]
PS2_USB = ["-m", "128", "-vga", "std", "-usb", "-device", "usb-mouse,port=1"]

# ── ЭТАП 9 [#465-#467]: контроллеры №3 — регрессия инстансов ───────
OHCI_THREE = NODEF + ["-device", "pci-ohci,id=oh0",
    "-device", "pci-ohci,id=oh1", "-device", "pci-ohci,id=oh2",
    "-device", "usb-kbd,bus=oh0.0,port=1",
    "-device", "usb-mouse,bus=oh2.0,port=1"]
XHCI_THREE = NODEF + ["-device", "qemu-xhci,id=x0",
    "-device", "qemu-xhci,id=x1", "-device", "qemu-xhci,id=x2",
    "-device", "usb-kbd,bus=x0.0,port=1",
    "-device", "usb-mouse,bus=x2.0,port=1"]

# ── ЭТАП 10 [#474]: живая мышь + ВТОРОЕ устройство (usb-tablet) ──────
XHCI_LIVE_2ND = NODEF + ["-device", "qemu-xhci,id=x0",
    "-device", "usb-kbd,bus=x0.0,port=1",
    "-device", "usb-mouse,bus=x0.0,port=2"]

# ── типовые инъекции ─────────────────────────────────────────────────
def inject_kbd_b(v):    v.sendkey("b")
def inject_mouse(v):    v.mouse_move(25, 15)

def inject_typematic(v):
    """Ctrl+J (вкл режим курсора) + удержание стрелки влево 2.5 с + отпускание."""
    v.sendkey("ctrl-j")
    time.sleep(0.3)
    v.key("left", True)
    time.sleep(2.5)
    v.key("left", False)

TESTS = [
 ("T1 UHCI клава+мышь", UHCI_BOTH,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(UHCI, 0x\w+ байт\): 0x00000000 0x00000000 0x00000005"),
   ("мышь", r"USBMOUSE: первый отчёт \(\d+ байт\): 0x00000000 0x00000019 0x0000000F")]),

 ("T2 EHCI оба напрямую", EHCI_BOTH,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(EHCI"),
   ("мышь", r"EHCI: первый отчёт мыши")]),

 ("T3 EHCI#1 клава + EHCI#2 мышь", EHCI_TWO,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(EHCI"),
   ("мышь", r"EHCI: первый отчёт мыши")]),

 ("T4 q35 порты 2.0 (ICH9)", Q35_USB20,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(EHCI"),
   ("мышь", r"EHCI: первый отчёт мыши")]),

 ("T5 OHCI оба", OHCI_BOTH,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"MOUSEHW: первый отчёт КЛАВИАТУРЫ \(OHCI"),
   ("мышь", r"MOUSEHW: first report \(OHCI,")]),

 ("T6 xHCI оба", XHCI_BOTH,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"MOUSEHW: первый отчёт КЛАВИАТУРЫ \(xHCI"),
   ("мышь", r"MOUSEHW: first report \(")]),

 ("T7 клава 2.0 + мышь 3.0", K2M3,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(EHCI"),
   ("мышь", r"MOUSEHW: first report \(")]),

 ("T8 клава 3.0 + мышь 2.0", K3M2,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"MOUSEHW: первый отчёт КЛАВИАТУРЫ \(xHCI"),
   ("мышь", r"EHCI: первый отчёт мыши")]),

 ("T9 hotplug мышь add→del→add", HOTPLUG,
  [(1.0, inject_kbd_b),
   (1.0, lambda v: v.dev("add", "usb-mouse", "mm", 2)),
   (2.5, inject_mouse),
   (1.0, lambda v: v.dev("del", "usb-mouse", "mm")),
   (1.5, lambda v: v.dev("add", "usb-mouse", "mm", 2)),
   (2.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(UHCI, 0x\w+ байт\): 0x00000000 0x00000000 0x00000005"),
   ("yield-клавы", r"уступаю контроллер мыши"),
   ("мышь-ready", r"USBMOUSE: ready \(порт 2\)"),
   ("мгновенный-детект-выдёргивания", r"CCS=0 - мышь выдернута"),
   ("re-add-подъём", r"USBMOUSE: ready \(порт 2\)[\s\S]*CCS=0[\s\S]*USBMOUSE: ready \(порт 2\)")]),

 ("T10 флешка = НЕИЗВЕСТНОЕ", UNKNOWN,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава", r"USB-KBD: первый отчёт \(UHCI"),
   ("мышь", r"USBMOUSE: первый отчёт"),
   ("флешка-неизвестное", r"НЕИЗВЕСТНОЕ")]),

 ("T11 typematic USB (Ctrl+J+стрелка)", UHCI_BOTH,
  [(1.0, inject_typematic)],
  [("удержана", r"KBD-CURSOR: стрелка УДЕРЖАНА"),
   ("отпущена", r"KBD-CURSOR: стрелка отпущена")]),

 ("T12 PS/2 клава + USB мышь + typematic PS/2", PS2_USB,
  [(1.0, inject_typematic)],
  [("typematic-PS/2", r"KBD-CURSOR: стрелка УДЕРЖАНА"),
   ("отпущена", r"KBD-CURSOR: стрелка отпущена"),
   ("USB-мышь жива", r"USBMOUSE: ready|USBMOUSE: первый отчёт|usb=YES")]),

 ("T13 ТРИ OHCI: мышь на ТРЕТЬЕМ (#466)", OHCI_THREE,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава-на-1-м", r"MOUSEHW: первый отчёт КЛАВИАТУРЫ \(OHCI"),
   ("мышь-на-3-м", r"MOUSEHW: first report \(OHCI,"),
   ("сквозные-№-гнёзд", r"ГНЕЗДО №"),
   ("сборка-в-баннере", r"Сборка \d+ kernel_main")]),

 ("T14 ТРИ xHCI: мышь на ТРЕТЬЕМ (#465)", XHCI_THREE,
  [(1.0, inject_kbd_b), (0.5, inject_mouse)],
  [("клава-на-1-м", r"MOUSEHW: первый отчёт КЛАВИАТУРЫ \(xHCI"),
   ("мышь-на-3-м", r"MOUSEHW: first report \("),
   ("сквозные-№-гнёзд", r"ГНЕЗДО №"),
   ("сборка-в-баннере", r"Сборка \d+ kernel_main")]),

 # [#474] живая мышь + перечисление второго устройства (ВТОРАЯ мышь):
 # hw_find_hid ПИСАЛА скрэтч (g_mps/g_ms_xep/g_iface) ДО отказа реестра,
 # живая мышь теряла конфигурацию. Снимок g_lm защищает; реестр отклоняет
 # вторую мышь, а клавиатура продолжает работать (typematic ПОСЛЕ отказа)
 ("T15 [#474] живая мышь + ВТОРОЕ устройство (мышь)", XHCI_LIVE_2ND,
  [(1.0, inject_kbd_b),                 # клава жива (первый отчёт)
   (0.7, inject_mouse),                 # мышь дала первый отчёт
   (0.3, lambda v: v.dev("add", "usb-mouse", "m2", 3)),  # вторая мышь → отказ
   (2.0, inject_mouse),                 # отчёты живой мыши продолжаются
   (1.0, inject_typematic)],            # клава работает ПОСЛЕ отказа реестра
  [("клава-жива-до", r"MOUSEHW: первый отчёт КЛАВИАТУРЫ \(xHCI"),
   ("мышь-жива-до", r"MOUSEHW: first report \("),
   ("второе-отклонено-реестром", r"уже занят"),
   ("снимок-защитил-живых", r"\[#474\] второе устройство отклонено - конфигурация ЖИВЫХ"),
   ("клава-жива-ПОСЛЕ-отказа", r"KBD-CURSOR: стрелка УДЕРЖАНА"),
   ("клава-жива-ПОСЛЕ-отказа-2", r"KBD-CURSOR: стрелка отпущена"),
   ("!мышь-не-деактивирована", r"MOUSEHW: deactivated"),
   ("сборка-в-баннере", r"Сборка \d+ kernel_main")]),

 # [#475] «нулевые БАЙТЫ ≠ нулевая ДЛИНА»: boot-отчёт «нет нажатых
 # клавиш» — это нули ДАННЫХ (Ситуация A, легально), а НЕ нулевая длина
 # (Ситуация B). Держим стрелку 2.5 с и ОТПУСКАЕМ: финальный отчёт —
 # ВСЕ НУЛИ; он обязан разобраться как «отпускание» (маркер «стрелка
 # отпущена»), БЕЗ маркера нулевой длины.
 ("T16 [#475] нулевые байты = «нет клавиш» (НЕ ошибка)", UHCI_BOTH,
  [(1.0, inject_typematic)],
  [("удержана", r"KBD-CURSOR: стрелка УДЕРЖАНА"),
   ("отпущена-нулевым-отчётом", r"KBD-CURSOR: стрелка отпущена"),
   ("!нул-длина-не-легитимна", r"НУЛЕВОЙ ДЛИНЫ"),
   ("!zero-length-не-в-наличии", r"ZERO LENGTH")]),
]

def main():
    # [#477] внятные сообщения о стенде ДО запуска сценариев
    if not QEMU:
        print("ОШИБКА СТЕНДА: QEMU не найден. Это НЕ ошибка USB-драйвера Egorov OS.")
        print("  Установи qemu-system-x86 (пакет qemu-system-x86) или укажи путь:")
        print("     EGOROV_QEMU=/путь/к/qemu-system-i386 python3 tests/e2e_audit.py")
        print("  Поиск выполнялся: $EGOROV_QEMU, PATH (qemu-system-i386,")
        print("  qemu-system-x86_64), /tmp/bin/qemu-system-i386.")
        sys.exit(2)
    if not os.path.isfile(KERNEL):
        print("ОШИБКА СТЕНДА: ядра нет — сначала собери ОС:  make  (в корне os/).")
        print(f"  Ожидался файл: {KERNEL}")
        print("  Это НЕ ошибка USB-драйвера: тестам нужен build/egorov.stage1.")
        sys.exit(2)
    print(f"стенд: QEMU = {QEMU}")
    print(f"       ядро = {KERNEL}")
    only = sys.argv[1:] if len(sys.argv) > 1 else None
    results = []
    for t in TESTS:
        if only and not any(o in t[0] for o in only):
            continue
        results.append((t[0], run_test(t)))
    print("\n════ ИТОГ ════")
    npass = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        npass += ok
    print(f"{npass}/{len(results)} PASS")
    sys.exit(0 if npass == len(results) else 1)

if __name__ == "__main__":
    main()
