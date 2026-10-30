# Egorov OS — System Programs source bundle

This directory contains the source code for the built-in applications exposed by the Start menu. In this early monolithic build these applications are linked into the kernel; they are not independent PE/ELF executables yet.

- `calc.c` / `calc.h` — Calculator
- `clock_app.c` / `clock_app.h` — Clock
- `paint.c` / `paint.h` — Paint
- `notepad.c` / `notepad.h` — Notepad
- `snake.c` / `snake.h` — Snake
- `explorer.c` / `explorer.h` — read-only file explorer
- `terminal.c` / `terminal.h` — Terminal
- `sysinfo.c` / `sysinfo.h` — System information
- `exe_runner.c` / `exe_runner.h` — current EXE-runner prototype (not a general Win32 runtime)
- `logwriter_ui.c` — log export UI; its shared interface is in `kernel/logwriter.h`

The ISO build includes a copy of this directory at `/System Programs/` as a source bundle. These are source files, not standalone executables. In this monolithic build the apps are compiled into the kernel and are not independent programs that Explorer can launch.

The current installer writes the bootloader and kernel to raw disk sectors and does not yet copy files into an installed FAT32 filesystem. Copying a real `System Programs` directory into the installed system requires FAT32 write support (Roadmap 0.3.2); this release does not pretend that this copy has happened.
