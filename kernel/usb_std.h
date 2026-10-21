#ifndef UHCI_KBD_USB_STD_H
#define UHCI_KBD_USB_STD_H

/* ─────────────────────────────────────────────────────────────────────────
 *  usb_std.h — стандартные константы USB / HID и структуры дескрипторов.
 *
 *  Только определения данных: файл одинаково компилируется и в hosted-
 *  среде (Linux userspace), и во freestanding-ядре (Egorov OS).
 *  Все структуры packed: дескрипторы USB идут по байтово, little-endian,
 *  а x86-цели позволяют прямое чтение полей без конверсии порядка байт.
 * ───────────────────────────────────────────────────────────────────────── */

#include "hal.h"

/* ── bmRequestType: направление ── */
#define USB_RT_IN   0x80u   /* устройство -> хост (data stage = IN)  */
#define USB_RT_OUT  0x00u   /* хост -> устройство (data stage = OUT) */

/* ── bmRequestType: тип запроса ── */
#define USB_REQTYPE_STANDARD 0x00u
#define USB_REQTYPE_CLASS    0x20u
#define USB_REQTYPE_VENDOR   0x40u

/* ── bmRequestType: получатель ── */
#define USB_RECIPIENT_DEVICE    0u
#define USB_RECIPIENT_INTERFACE 1u
#define USB_RECIPIENT_ENDPOINT  2u

/* ── Стандартные bRequest ── */
#define USB_REQ_GET_STATUS        0x00u
#define USB_REQ_CLEAR_FEATURE     0x01u
#define USB_REQ_SET_FEATURE       0x03u
#define USB_REQ_SET_ADDRESS       0x05u
#define USB_REQ_GET_DESCRIPTOR    0x06u
#define USB_REQ_SET_DESCRIPTOR    0x07u
#define USB_REQ_GET_CONFIGURATION 0x08u
#define USB_REQ_SET_CONFIGURATION 0x09u
#define USB_REQ_GET_INTERFACE     0x0Au
#define USB_REQ_SET_INTERFACE     0x0Bu
#define USB_REQ_SYNCH_FRAME       0x0Cu

/* ── Селекторы feature для CLEAR/SET_FEATURE ── */
#define USB_FEATURE_ENDPOINT_HALT 0x0000u

/* ── Типы дескрипторов (старший байт wValue в GET_DESCRIPTOR) ── */
#define USB_DT_DEVICE    0x01u
#define USB_DT_CONFIG    0x02u
#define USB_DT_STRING    0x03u
#define USB_DT_INTERFACE 0x04u
#define USB_DT_ENDPOINT  0x05u
#define USB_DT_HID       0x21u
#define USB_DT_REPORT    0x22u

/* ── Классы/протоколы ── */
#define USB_CLASS_HID     0x03u
#define HID_SUBCLASS_BOOT 0x01u
#define HID_PROTO_KBD     0x01u   /* boot-протокол: клавиатура */
#define HID_PROTO_MOUSE   0x02u

/* ── Классовые запросы HID (bmRequestType 0x21 / 0xA1) ── */
#define HID_REQ_GET_REPORT   0x01u
#define HID_REQ_GET_IDLE     0x02u
#define HID_REQ_SET_REPORT   0x09u
#define HID_REQ_SET_IDLE     0x0Au
#define HID_REQ_SET_PROTOCOL 0x0Bu

/* ── SETUP-пакет управляющей передачи (8 байт, little-endian) ── */
typedef struct __attribute__((packed)) {
    uint8_t  bRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

/* ── Дескриптор устройства (18 байт) ── */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} usb_device_desc_t;

/* ── Дескриптор конфигурации (9 байт, дальше — интерфейсы и точки) ── */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} usb_config_desc_t;

/* ── Дескриптор интерфейса (9 байт) ── */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} usb_interface_desc_t;

/* ── Дескриптор конечной точки (7 байт) ── */
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;   /* бит7 = направление (1 = IN) */
    uint8_t  bmAttributes;       /* биты1:0 = тип передачи (3 = interrupt) */
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;          /* период опроса, мс */
} usb_endpoint_desc_t;

#endif /* UHCI_KBD_USB_STD_H */
