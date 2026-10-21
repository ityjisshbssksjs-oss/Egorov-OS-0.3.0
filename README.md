# Egorov OS

**Egorov OS** — экспериментальная 32-битная операционная система для x86,
разрабатываемая с нуля. Текущая версия — **0.3.0 (сборка 4)**.

Проект не является POSIX- или Linux-совместимым: ядро, драйверы, GUI и
пользовательские приложения реализованы собственными средствами.

## Возможности

- **Собственное ядро** на C и ассемблере, сборка через `Makefile`
  (`gcc -m32`, `nasm`, `ld`, `objcopy`, `grub-mkrescue`).
- **Два пути загрузки:**
  - GRUB / Multiboot 1;
  - собственный двухстадийный HDD-загрузчик (MBR + stage2).
- **Графика:** VBE linear framebuffer 1024×768×32, fallback через BochsDispi VBE.
- **GUI-десктоп:** окна, панель задач, курсор, шрифт Terminus 8×16.
- **Приложения:** терминал, notepad, калькулятор, часы, paint, explorer,
  snake, sysinfo, установщик.
- **Драйверы:**
  - PS/2 keyboard/mouse;
  - ATA / AHCI;
  - USB UHCI / OHCI / EHCI / xHCI;
  - USB HID keyboard/mouse;
  - USB Mass Storage (BOT/SCSI), FAT32.
- **Установка на HDD:** создание MBR, разметка, запись загрузчика и ядра,
  flush cache и read-back verify.
- **Логирование:** COM1 + RAM-ring, запись `EGOROV.LOG` на FAT32-флешку
  (раздел `LOGS`), диагностика через serial.
- **Сборка и запуск:**
  ```bash
  make
  make run
  make run2-kernel
  make run-uhci-kbd-mouse
