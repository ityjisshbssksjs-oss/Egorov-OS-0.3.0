#!/usr/bin/env python3
"""Static smoke checks for LOGWRITER FAT32-any-label and guarded RAW flow."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
terminal = (ROOT / "system_programs/terminal.c").read_text(encoding="utf-8")
desktop = (ROOT / "kernel/desktop.c").read_text(encoding="utf-8")
ui = (ROOT / "system_programs/logwriter_ui.c").read_text(encoding="utf-8")
fs = (ROOT / "kernel/logstick.c").read_text(encoding="utf-8")
makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
version = (ROOT / "kernel/version.h").read_text(encoding="utf-8")
checks = {
    "terminal command": 'strcmp(cmd_upper, "LOGWRITER")' in terminal,
    "desktop app registration": "APP_LOGWRITER" in desktop,
    "terminal-to-desktop entry point": "desktop_logwriter_open" in desktop and "desktop_logwriter_open" in terminal,
    "device picker and export button": "Записать и проверить" in ui and "logwriter_list_devices" in ui,
    "selected transport FAT32 export": "logwriter_export(g_devices[g_selected].transport" in ui,
    "raw requires separate confirmation": 'if (g_raw_available && !g_raw_armed)' in ui and 'ДА, ЗАПИСАТЬ RAW' in ui,
    "FAT32 any label for interactive flow": "ls_find_fat32_part(false)" in fs and "любая метка" in fs,
    "automatic sticklog still label-restricted": "ls_find_fat32_part(true)" in fs and "LS_LABEL" in fs,
    "RAW export code": "logwriter_export_raw" in fs and 'memcpy(expected_header, "EGORAW1", 7)' in fs,
    "raw writes explicitly bounded": "lba >= allowed_sectors || lba >= g_ls_cap_sect" in fs,
    "read-back verification for FAT file": "lw_verify_file(g_logwriter_snapshot, total)" in fs,
    "byte-by-byte FAT verify": "g_sec[j] != expected[compared + j]" in fs,
    "byte-by-byte raw verify": "g_sec[j] != g_logwriter_snapshot[offset + j]" in fs,
    "UI included in build": "system_programs/logwriter_ui.c" in makefile,
    "build number is 6 while product version remains 0.3.1": (
        '#define EGOROV_OS_VER        "0.3.1"' in version
        and '#define EGOROV_OS_BUILD_N    6' in version
        and '#define EGOROV_OS_BUILD_S    "6"' in version
        and '#define EGOROV_OS_FULL       "0.3.1 Сборка 6"' in version
    ),
    "FAT32 recognition is BPB-based, not dependent on type label": 'memcmp(g_sec + 0x52, "FAT32", 5)' not in fs,
    "superfloppy FAT32 supported": "g_disk_superfloppy = true" in fs and "whole.start = 0u" in fs and "g_part_guard_enabled = true" in fs,
    "FAT32 BPB structural checks": "root_entries != 0u || total16 != 0u || fatsz16 != 0u" in fs,
    "GPT EFI GUID compared fully": "static const uint8_t ls_efi_guid[16]" in fs and "memcmp(ent, ls_efi_guid, 16)" in fs,
    "malformed partition map blocks RAW": "g_disk_map_untrusted" in fs and "карта разделов недостоверна" in fs,
    "GPT header CRC is validated": "header_crc_actual != header_crc_expected" in fs,
    "GPT entry-array CRC is validated": "~entry_crc_state != entries_crc_expected" in fs,
}
failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(("PASS" if ok else "FAIL") + ": " + name)
if failed:
    raise SystemExit("Static smoke checks failed: " + ", ".join(failed))
print("Static LOGWRITER checks passed; USB/FAT32 and destructive RAW still need runtime tests.")
