#!/usr/bin/env python3
"""Static checks for preserving live xHCI HID events during synchronous waits."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
s = (root / "kernel/mouse_hw.c").read_text(encoding="utf-8")
checks = {
    "live event dispatcher exists": "static bool hw_route_live_xhci_event" in s,
    "command wait routes unrelated events": "(void)hw_route_live_xhci_event(type, p, st, ctl);" in s,
    "control transfer matches EP0 endpoint ID": "((ctl >> 16) & 0x1Fu) == 1u" in s,
    "bulk wait routes live HID events": s.count("(void)hw_route_live_xhci_event(type, p, st, ctl);") >= 3,
    "mouse endpoint is filtered by slot and endpoint": "ep_id == (2u * (uint32_t)lm_xep() + 1u)" in s,
    "keyboard endpoint is filtered by slot and endpoint": "ep_id == (2u * (uint32_t)lk_xep() + 1u)" in s,
    "successful mouse data gets routed": "hw_hid_report(hw_pkt, act)" in s and "hw_boot_report(hw_pkt, act)" in s,
    "keyboard data gets routed": 'oh_kbd_process(hw_kbd_pkt, act, "xHCI/deferred")' in s,
    "mouse transfer is re-armed": "if (g_active) hw_arm();" in s,
    "keyboard transfer is re-armed": "hw_kbd_arm();" in s,
    "mouse pending flags preserved": "g_pending_mouse_moved" in s and "g_pending_mouse_left" in s,
    "bounded mouse input diagnostics": "g_mouse_activity_logs >= 8u" in s,
    "keyboard key transition diagnostics": 'hw_trace_kbd_transition("key-up"' in s and 'hw_trace_kbd_transition("key-down"' in s,
}
failed = []
for name, ok in checks.items():
    print(("PASS" if ok else "FAIL") + ": " + name)
    if not ok:
        failed.append(name)
if failed:
    raise SystemExit("xHCI event-routing static checks failed: " + ", ".join(failed))
print("Static checks passed; runtime behavior still needs QEMU and physical-device testing.")
