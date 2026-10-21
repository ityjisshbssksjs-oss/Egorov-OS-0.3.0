#ifndef EGOROV_USB_XHCI_H
#define EGOROV_USB_XHCI_H

#include "types.h"
#include "ps2.h"

/* ── xHCI (USB 3.x) host-controller mouse driver ───────────────────────────
 *
 * Covers ALL modern machines (Intel Haswell 2013+, AMD FM2+ and later)
 * whose PCI bus carries only an xHCI controller (class 0x0C, subclass
 * 0x03, prog-if 0x30).  The UHCI driver in usb_mouse.c cannot see a mouse
 * there BY DESIGN — this driver closes that gap.
 *
 * Flow: usb_mouse_init() calls usb_xhci_init() after its UHCI scan finds
 * no mouse.  usb_mouse_poll() dispatches to usb_xhci_poll() while the
 * xHCI transport is active.  All shared pointer state lives in the
 * transport-independent core of usb_mouse.c.
 */

/* Scan PCI for xHCI controllers; reset/enumerate a HID mouse on any of
 * their ports (low/full/high speed).  Safe to call once at boot. */
void usb_xhci_init(void);

/* Drains pending event-ring entries and re-arms the interrupt-IN
 * endpoint.  Never blocks.  Called from usb_mouse_poll(). */
void usb_xhci_poll(void);

/* release the pointer (usb_mouse_deactivate dispatches here
 * for the xHCI transport).  Idempotent. */
void usb_xhci_deactivate(void);

#endif
