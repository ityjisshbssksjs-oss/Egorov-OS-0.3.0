#ifndef EGOROV_POWER_H
#define EGOROV_POWER_H

/* ── ACPI power management (v1.7) ──────────────────────────────────────────
 * On REAL hardware the old code only wrote to QEMU/Bochs/VirtualBox
 * fixed ports, which do nothing on a physical machine, and the final
 * 0x64←0xFE fallback is a RESET, not a power-off — so "Завершение
 * работы" either did nothing or rebooted the PC.
 *
 * power_shutdown(): parses ACPI (RSDP → RSDT/XSDT → FADT → DSDT \_S5_
 *   package) and issues the S5 sleep transition via PM1a/PM1b_CNT_BLK.
 *   Falls back to the VM convenience ports, otherwise halts with a
 *   serial message (manual power-off required — never a surprise reset).
 * power_reboot(): tries FADT RESET_REG, then port 0xCF9, then the i8042
 *   pulse, then a triple fault.  Never returns.                       */

void power_shutdown(void);
void power_reboot(void);

#endif
