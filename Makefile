# Egorov OS 0.3.0 - kernel + ISO build
# Pipeline: kernel objects -> stage1 ELF -> flat egorov.bin -> embed.o -> egorov.kernel -> ISO
# Run `make` to build; `make clean` removes all build artefacts.

CC      := gcc
LD      := ld
NASM    := $(shell which nasm 2>/dev/null || echo /home/runner/.nix-profile/bin/nasm)
OBJCOPY := objcopy

# Locate grub-mkrescue from nix profile first, then PATH.
GRUB := $(shell ls /home/runner/.nix-profile/bin/grub-mkrescue 2>/dev/null || \
	which grub-mkrescue 2>/dev/null || \
	echo grub-mkrescue)

# ─── Запрет FPU/SIMD-регистров в коде ядра (аудит #221, п.10a) ───────────────
# -mgeneral-regs-only — самый строгий вариант, но флаг появился в GCC 7
# (2017).  На Ubuntu 16.04/17.04 (GCC 5/6) он неизвестен и ронял сборку
# на ПЕРВОЙ же команде: «gcc: error: unrecognized command line option
# '-mgeneral-regs-only'».  Поэтому флаг выбирается ПРОБОЙ: пробуем
# скомпилировать пустой файл с ним; если компилятор флага не знает —
# откат на -msoft-float (для целочисленного ядра эквивалент: x87-кода
# нет, а SSE/MMX уже запрещены -mno-sse/-mno-mmx/-mno-sse2 выше).
# Сборка работает на GCC от 4.8 до 14+ без правки Makefile руками.
ifeq ($(shell $(CC) -mgeneral-regs-only -fsyntax-only -x c /dev/null >/dev/null 2>&1 && echo OK),OK)
REGS_ONLY := -mgeneral-regs-only
else
REGS_ONLY := -msoft-float
endif
$(info GCC-режим: REGS_ONLY = $(REGS_ONLY))

CFLAGS  := -m32 -ffreestanding -fno-stack-protector -fno-pic -fno-pie \
	-nostdinc -nostdlib -fno-builtin \
	-O2 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
	-Wno-unused-variable \
	-fno-asynchronous-unwind-tables -mno-sse -mno-mmx -mno-sse2 \
	-fno-strict-aliasing \
	-fno-tree-loop-distribute-patterns \
	-fno-optimize-sibling-calls \
	-Ikernel \
	-std=gnu99 \
	$(REGS_ONLY)

# [#480] ЭТАП 10: ЗАВИСИМОСТИ ОТ ЗАГОЛОВКОВ. Прежние правила
# «%.o: %.c $(FONT)» не зависели НИ ОТ ОДНОГО .h: правка kernel/version.h
# (номер СБОРКИ!) не пересобирала kernel.o/mouse_hw.o — в свежем образе
# оставался СТАРЫЙ баннер «Сборка 1», и по флешке нельзя было понять,
# записался ли новый релиз (сам сценарий из [#473] ломался молча).
# -MMD -MP пишет .d рядом с .o; строки -include внизу подключают их.
DEPFLAGS := -MMD -MP
CFLAGS   += $(DEPFLAGS)
# На boot.S (.S-цель) REGS_ONLY не влияет: ассемблерная часть про CR0
# там своя; gcc передаёт target-опции дальше без ошибок.

LDFLAGS := -m elf_i386 -nostdlib -T linker.ld

KERNEL_C := \
	kernel/kernel.c \
	kernel/timer.c \
	kernel/power.c \
	kernel/string.c \
	kernel/fb.c \
	kernel/text.c \
	kernel/gui.c \
	kernel/desktop.c \
	kernel/installer.c \
	kernel/ps2.c \
	kernel/ata.c \
	kernel/ahci.c \
	kernel/usb.c \
	kernel/usb_mouse.c \
	kernel/uhci_core.c \
	kernel/uhci_ctl.c \
	kernel/uhci_kbd.c \
	kernel/usb_uhci_kbd.c \
	kernel/ehci.c \
	kernel/usb_kbd.c \
	kernel/usbmap.c \
	kernel/usbdev.c \
	kernel/logstick.c \
	kernel/mouse_hw.c \
	kernel/rtc.c \
	kernel/calc.c \
	kernel/clock_app.c \
	kernel/paint.c \
	kernel/explorer.c \
	kernel/notepad.c \
	kernel/sysinfo.c \
	kernel/snake.c \
	kernel/exe_runner.c \
	kernel/terminal.c

# boot/boot.S MUST come first so that _start is the very first symbol in
# .text (= 0x101000).  The HDD bootloader hard-codes that jump address.
# GRUB is unaffected — it always reads the ELF entry-point field.
BUILD  := build
KOBJS  := $(BUILD)/boot/boot.o \
	$(patsubst %.c,$(BUILD)/%.o,$(KERNEL_C))

BOOT_BIN  := $(BUILD)/hdd_boot.bin
EMBED_OBJ := $(BUILD)/boot/embed.o

STAGE1 := $(BUILD)/egorov.stage1
FLAT   := $(BUILD)/egorov.bin
KERNEL := $(BUILD)/egorov.kernel
ISO    := $(BUILD)/egorov-os.iso
FONT   := kernel/font.h

all: $(ISO)

# ─── Font generation ──────────────────────────────────────────────────────────
# Use pre-generated font.h if it already exists (avoids needing scripts/).
# If missing, attempt node / python3 generation.
$(FONT):
	@if [ -f $(FONT) ]; then \
	echo "$(FONT) already exists, skipping generation."; \
	elif command -v node >/dev/null 2>&1 && [ -f scripts/gen_font_node.js ]; then \
	node scripts/gen_font_node.js; \
	elif command -v python3 >/dev/null 2>&1 && [ -f scripts/gen_font.py ]; then \
	python3 scripts/gen_font.py; \
	else \
	echo "ERROR: $(FONT) not found and no generator available" && exit 1; \
	fi

# ─── Kernel C/S compilation ───────────────────────────────────────────────────
$(BUILD)/%.o: %.c $(FONT)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# ─── HDD bootloader (NASM flat binary) ───────────────────────────────
# Глубокий аудит #218: перед сборкой загрузчика генерируется
# build/kernel_sum.h — 32-битная сумма dwords (+хвостовые байты)
# egorov.bin; загрузчик %include'ит её (nasm -I build, синтаксис
# %define — NASM НЕ понимает C-шный #define) и сверяет
# скопированный в PM образ ПЕРЕД прыжком на 0x101000 (код «KS» при
# несовпадении).  Порядок целей: FLAT -> kernel_sum.h -> hdd_boot.bin
# -> embed.o: зависимости ниже держат порядок автоматически, циклов нет.
# Расчёт без python: od+awk (сумма точна в double до 2^53, наша < 2^48).
KSUM_H := $(BUILD)/kernel_sum.h

$(KSUM_H): $(FLAT)
	@mkdir -p $(BUILD)
	@sz=$$(stat -c%s $(FLAT)); dw=$$(( sz / 4 )); tb=$$(( sz % 4 )); \
	s=$$(( $$(od -An -v -tu4 -w4 $(FLAT) | head -n $$dw | awk '{t+=$$1} END{printf "%.0f", t}') )); \
	if [ $$tb -gt 0 ]; then \
		s=$$(( s + $$(tail -c $$tb $(FLAT) | od -An -v -tu1 | awk '{t+=$$1} END{printf "%.0f", t}') )); \
	fi; \
	s=$$(( s % 4294967296 )); \
	printf '%%define KERNEL_SUM 0x%08X\n%%define KERNEL_DWORDS %d\n%%define KERNEL_TAIL %d\n' $$s $$dw $$tb > $@ && \
	echo "KSUM: sum=0x$$(printf '%08X' $$s) dwords=$$dw tail=$$tb -> $@"

$(BOOT_BIN): boot/hdd_boot.asm $(KSUM_H)
	@mkdir -p $(BUILD)
	$(NASM) -f bin -I$(BUILD)/ -o $@ $<


# ─── Pass 1: link kernel without embedded sections ────────────────────────────
$(STAGE1): $(KOBJS) linker.ld
	$(LD) $(LDFLAGS) -o $@ $(KOBJS)

# ─── Flat binary dump (written to LBA 2048 by the installer) ─────────────────
# HDD-загрузчик читает 6 × 128 секторов (LBA 2048..2815) = 384 КБ стейджинга.
$(FLAT): $(STAGE1)
	$(OBJCOPY) -O binary $< $@
	@sz=$$(stat -c%s $@); [ $$sz -le 393216 ] || { echo "ERROR: egorov.bin is $$sz bytes - HDD loader reads only 768 sectors (384 KB)"; exit 1; }

# ─── Embedding stub: bakes egorov.bin + hdd_boot.bin into ELF sections ───────
$(EMBED_OBJ): boot/embed.S $(FLAT) $(BOOT_BIN)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# ─── Pass 2: re-link with embedded sections ───────────────────────────────────
$(KERNEL): $(KOBJS) $(EMBED_OBJ) linker.ld
	$(LD) $(LDFLAGS) -o $@ $(KOBJS) $(EMBED_OBJ)

# ─── ISO image ────────────────────────────────────────────────────────────────
$(ISO): $(KERNEL) boot/grub.cfg
	@mkdir -p $(BUILD)/iso/boot/grub
	@mkdir -p $(BUILD)/iso/boot/grub/fonts
	cp $(KERNEL) $(BUILD)/iso/boot/egorov.kernel && \
	cp boot/grub.cfg $(BUILD)/iso/boot/grub/grub.cfg   # [#457]: && - cp-падение не соберёт ISO без ядра
	@for d in /usr/share/grub /usr/share/grub2 /boot/grub/fonts \
	/nix/var/nix/profiles/default/share/grub; do \
	if [ -f "$$d/unicode.pf2" ]; then \
	cp "$$d/unicode.pf2" $(BUILD)/iso/boot/grub/fonts/unicode.pf2; \
	break; \
	fi; \
	done; true
	$(GRUB) -o $(ISO) $(BUILD)/iso

# ─── QEMU targets (require qemu-system-i386) ─────────────────────────────────
run: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std
run-usbmouse: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std -usb -device usb-mouse \
	-serial stdio 2>&1 | tee qemu.log
run-usbtablet: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std -usb -device usb-tablet \
	-serial stdio 2>&1 | tee qemu.log

# OHCI-путь мыши: usb-mouse на pci-ohci (тот же класс контроллера, что
# на ноутбуке).  Проверка перечисления HID по OHCI в виртуалке.
run-ohci-mouse: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-device pci-ohci,id=ohci0 -device usb-mouse,bus=ohci0.0 -serial stdio 2>&1 | tee qemu.log

# EHCI (USB 2.0): usb-kbd + usb-mouse on usb-ehci. No companions - the
# FS keyboard takes the DIRECT EHCI path, the HS mouse enumerates normally.
run-ehci: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-device usb-ehci,id=ehci0 \
	-device usb-kbd,bus=ehci0.0 -device usb-mouse,bus=ehci0.0 \
	-serial stdio 2>&1 | tee qemu_ehci.log

# UHCI-клавиатура (драйвер uhci_kbd.c, HID boot protocol, поллинг):
# единственный USB-девайс на штатном PIIX3 UHCI — контроллер целиком
# у клавиатуры. Печать — через usb_kbd_parse_boot("UHCI") в ps2-очередь.
run-uhci-kbd: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-usb -device usb-kbd -serial stdio 2>&1 | tee qemu_uhci_kbd.log

# Мышь И клавиатура на ОДНОМ UHCI-контроллере: расписание общее —
# QH клавиатуры подцеплен за QH мыши (usb_mouse_sched_chain),
# каждый переарм TD мыши цепочку сохраняет. Оба устройства живые.
# ВАЖНО: явные port= — иначе QEMU 10 сам вставляет USB-хаб между
# корневым портом и клавиатурой (хаб-топология драйвером не решается).
run-uhci-kbd-mouse: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-usb -device usb-mouse,port=1 -device usb-kbd,port=2 \
	-serial stdio 2>&1 | tee qemu_uhci_both.log

# 0.3.2: быстрый запуск БЕЗ ISO/grub-mkrescue — multiboot ELF напрямую
# (qemu-system-i386 -kernel build/egorov.stage1).
KQ := qemu-system-i386 -kernel $(BUILD)/egorov.stage1 -m 128 -vga std

run2-kernel: $(BUILD)/egorov.stage1
	$(KQ)

run2-uhci-kbd-mouse: $(BUILD)/egorov.stage1
	$(KQ) -usb -device usb-mouse,port=1 -device usb-kbd,port=2 \
	-serial stdio 2>&1 | tee qemu_uhci_both.log

# «Порты 2.0» как на реальном железе: q35 (ICH9) — EHCI + 3 UHCI-
# компаньона; клава и мышь воткнуты в разъёмы 2.0 (порты 3-4): EHCI
# отдаёт low/full-speed-устройства компаньону, ОС принимает их на UHCI.
run2-usb20: $(BUILD)/egorov.stage1
	$(KQ) -M q35 -device usb-kbd,bus=usb.0,port=3 \
	-device usb-mouse,bus=usb.0,port=4 -serial stdio 2>&1 | tee qemu_usb20.log

run-ehci-kbd: $(ISO)
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-device usb-ehci,id=ehci0 -device usb-kbd,bus=ehci0.0 \
	-serial stdio 2>&1 | tee qemu_ehci_kbd.log

# MOUSEHW (xHCI):
# Модель xHCI выбирается автоматически: qemu-xhci (QEMU >= 4.0) или
# nec-usb-xhci (есть во ВСЕХ версиях QEMU начиная с 1.5). Машина —
# обычная pc (БЕЗ -M q35): у старых QEMU (например 2.0.0) SeaBIOS может
# не грузить CD на q35 («No bootable device»); xHCI-устройству чипсет
# безразличен, а мышь жёстко привязана к шине xHCI (bus=xhci.0), чтобы
# QEMU не подключил её к дефолтному UHCI-контроллеру.
XHCI_DEV := $(shell qemu-system-i386 -device help 2>/dev/null | grep -qw qemu-xhci && echo qemu-xhci || echo nec-usb-xhci)
run-xhci: $(ISO)
	@echo "XHCI model: $(XHCI_DEV)"
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std -device $(XHCI_DEV),id=xhci \
	-device usb-mouse,bus=xhci.0 -serial stdio 2>&1 | tee qemu.log
run-xhci-tablet: $(ISO)
	@echo "XHCI model: $(XHCI_DEV)"
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std -device $(XHCI_DEV),id=xhci \
	-device usb-tablet,bus=xhci.0 -serial stdio 2>&1 | tee qemu.log


# STICKLOG - журнал ядра на флешку (раздел LOGS, FAT32 с LBA 20480).
# QEMU-проверка записи: usb-storage на штатном UHCI (путь usb.c).
# usbstick.img: MBR + FAT32-раздел с сектора 20480 (раздел LOGS).
run-sticklog: $(ISO) $(BUILD)/usbstick.img
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-drive if=none,id=ustick,file=$(BUILD)/usbstick.img,format=raw \
	-usb -device usb-storage,drive=ustick -serial stdio 2>&1 | tee qemu.log

# OHCI-путь: usb-storage на pci-ohci, без UHCI.
# Шина USB-устройства с id=ohci0 называется «ohci0.0» (не ohci.0 -
# QEMU 2.0 отвечает «Bus not found»).  Если QEMU не знает bus= -
# убери параметр: устройство встанет на единственную USB-шину.
run-sticklog-ohci: $(ISO) $(BUILD)/usbstick.img
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-drive if=none,id=ustick,file=$(BUILD)/usbstick.img,format=raw \
	-device pci-ohci,id=ohci0 -device usb-storage,drive=ustick,bus=ohci0.0 \
	-serial stdio 2>&1 | tee qemu.log

# UHCI-запасной путь: если OHCI-путь падает, флешка на штатном UHCI
# подтвердит, что стек sticklog/BOT в целом работает.
run-sticklog-uhci: $(ISO) $(BUILD)/usbstick.img
	qemu-system-i386 -cdrom $(ISO) -m 128 -vga std \
	-drive if=none,id=ustick,file=$(BUILD)/usbstick.img,format=raw \
	-usb -device usb-storage,drive=ustick -serial stdio 2>&1 | tee qemu_uhci.log

# Синтетическая флешка: MBR + раздел 2 (тип 0x0C) с LBA 20480, внутри
# FAT32 BPB reserved=32, fats=2, FATSz32=14976 ->
# write_lba = 20480 + 32 + 2*14976 + 128 = 50592
# Ядро читает FAT-размер из BPB и печатает итог ДЕСЯТИЧНЫМИ на экране.
$(BUILD)/usbstick.img: tools/mkstick.py
	@mkdir -p $(BUILD)
	python3 $< $@

run-install: $(ISO) $(BUILD)/disk.img
	qemu-system-i386 -cdrom $(ISO) -hda $(BUILD)/disk.img -m 128 -vga std

run-hdd: $(BUILD)/disk.img
	qemu-system-i386 -hda $(BUILD)/disk.img -m 128 -vga std

$(BUILD)/disk.img:
	@mkdir -p $(BUILD)
	dd if=/dev/zero of=$@ bs=1M count=128

clean:
	rm -rf $(BUILD)

# [#480] подключить автозависимости (.d пишутся компилятором рядом с .o)
KERNEL_DEPS := $(wildcard $(BUILD)/kernel/*.d)
BOOT_DEPS   := $(wildcard $(BUILD)/boot/*.d)
ifneq ($(words $(KERNEL_DEPS) $(BOOT_DEPS)),0)
  -include $(KERNEL_DEPS) $(BOOT_DEPS)
endif

.PHONY: all clean run run-usbmouse run-usbtablet run-ohci-mouse run-xhci run-xhci-tablet run-sticklog run-sticklog-ohci run-sticklog-uhci run-install run-hdd run-uhci-kbd run-uhci-kbd-mouse run2-kernel run2-uhci-kbd-mouse run2-usb20 run3-usb1-both run3-usb2-both run3-usb3-both run3-usb3-two-ctrl run3-mixed run3-ohci run3-unknown run3-hotplug run3-k3m2 run3-k2m3

# ═══ 0.3.0: тесты диспетчера usbdev (шесть драйверов + реестр + 1 Гц опрос) ═══
# KQ3 = запуск с serial-логом в файл (без окна)
KQ3 := qemu-system-i386 -kernel $(BUILD)/egorov.stage1 -m 128 -vga std -display none

# T1: оба устройства в портах 1.x (UHCI)
run3-usb1-both: $(BUILD)/egorov.stage1
	$(KQ3) -usb -device usb-kbd,port=1 -device usb-mouse,port=2 \
	-serial file:q3_t1_usb1.log; echo "лог: qemu T1 -> q3_t1_usb1.log"

# T2: оба в портах 2.0 (q35: ICH9 EHCI + 3 UHCI-компаньона; устройства
# в EHCI-портах 3-4: HS напрямую, FS/LS уходят компаньону — как на железе)
run3-usb2-both: $(BUILD)/egorov.stage1
	$(KQ3) -M q35 \
	-device ich9-usb-uhci1,bus=pcie.0,addr=1d.0,id=uh1,multifunction=on \
	-device ich9-usb-uhci2,bus=pcie.0,addr=1d.1,id=uh2 \
	-device ich9-usb-uhci3,bus=pcie.0,addr=1d.2,id=uh3 \
	-device ich9-usb-ehci1,bus=pcie.0,addr=1d.7,id=eh0 \
	-device usb-kbd,bus=eh0.0,port=3 -device usb-mouse,bus=eh0.0,port=4 \
	-serial file:q3_t2_usb2.log; echo "лог: q3_t2_usb2.log"

# T3: оба в портах 3.0 (один qemu-xhci)
run3-usb3-both: $(BUILD)/egorov.stage1
	$(KQ3) -device qemu-xhci,id=x0 \
	-device usb-kbd,bus=x0.0,port=1 -device usb-mouse,bus=x0.0,port=2 \
	-serial file:q3_t3_usb3.log; echo "лог: q3_t3_usb3.log"

# T4: оба в портах 3.0, но НА РАЗНЫХ xHCI-контроллерах (AMD FCH модель)
run3-usb3-two-ctrl: $(BUILD)/egorov.stage1
	$(KQ3) -device qemu-xhci,id=x0 -device qemu-xhci,id=x1 \
	-device usb-kbd,bus=x0.0,port=1 -device usb-mouse,bus=x1.0,port=1 \
	-serial file:q3_t4_x2.log; echo "лог: q3_t4_x2.log"

# T5: клавиатура в 1.x (UHCI) + мышь в 3.0 (xHCI)
run3-mixed: $(BUILD)/egorov.stage1
	$(KQ3) -usb -device usb-kbd,port=1 \
	-device qemu-xhci,id=x0 -device usb-mouse,bus=x0.0,port=1 \
	-serial file:q3_t5_mixed.log; echo "лог: q3_t5_mixed.log"

# T6: оба на OHCI (pci-ohci)
run3-ohci: $(BUILD)/egorov.stage1
	$(KQ3) -device pci-ohci,id=oh0 \
	-device usb-kbd,bus=oh0.0,port=1 -device usb-mouse,bus=oh0.0,port=2 \
	-serial file:q3_t6_ohci.log; echo "лог: q3_t6_ohci.log"

# T7: «неизвестные устройства»: флешка + ВТОРАЯ клавиатура (слот занят)
run3-unknown: $(BUILD)/egorov.stage1
	$(KQ3) -usb -device usb-kbd,port=1 -device usb-mouse,port=2 \
	-device usb-storage,port=3 \
	-serial file:q3_t7_unknown.log; echo "лог: q3_t7_unknown.log"

# T9: клавиатура 3.0 (xHCI) + мышь 2.0 (EHCI) — кросс-комбинация
run3-k3m2: $(BUILD)/egorov.stage1
	$(KQ3) -device qemu-xhci,id=x0 -device usb-kbd,bus=x0.0,port=1 \
	-device usb-ehci,id=eh0 -device usb-mouse,bus=eh0.0,port=1 \
	-serial file:q3_t9_k3m2.log; echo "лог: q3_t9_k3m2.log"

# T9b: клавиатура 2.0 (EHCI) + мышь 3.0 (xHCI) — кросс-комбинация
run3-k2m3: $(BUILD)/egorov.stage1
	$(KQ3) -device usb-ehci,id=eh0 -device usb-kbd,bus=eh0.0,port=1 \
	-device qemu-xhci,id=x0 -device usb-mouse,bus=x0.0,port=1 \
	-serial file:q3_t9b_k2m3.log; echo "лог: q3_t9b_k2m3.log"

# T8: hotplug через монитор QEMU: device_add/device_del usb-мыши
run3-hotplug: $(BUILD)/egorov.stage1
	-$(KQ3) -usb -device usb-kbd,port=1 \
	-monitor unix:/tmp/q3mon.sock,server,nowait \
	-serial file:q3_t8_hotplug.log & sleep 6; \
	echo "device_add usb-mouse,id=mm,port=2" | socat - unix-connect:/tmp/q3mon.sock; \
	sleep 3; echo "device_del mm" | socat - unix-connect:/tmp/q3mon.sock; \
	sleep 3; echo "device_add usb-mouse,id=mm,port=2" | socat - unix-connect:/tmp/q3mon.sock; \
	sleep 4; pkill -f egorov.stage1; echo "лог: q3_t8_hotplug.log"
