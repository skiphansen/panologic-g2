/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

#ifndef _USB_CH9_H_
#define _USB_CH9_H_

/* USB chapter 9 structures */
#define _ch9_h_

/* Misc.USB constants */
#define DEV_DESCR_LEN   18      //device descriptor length
#define CONF_DESCR_LEN  9       //configuration descriptor length
#define INTR_DESCR_LEN  9       //interface descriptor length
#define EP_DESCR_LEN    7       //endpoint descriptor length

/* Standard Device Requests */

#define USB_REQUEST_GET_STATUS                  0       // Standard Device Request - GET STATUS
#define USB_REQUEST_CLEAR_FEATURE               1       // Standard Device Request - CLEAR FEATURE
#define USB_REQUEST_SET_FEATURE                 3       // Standard Device Request - SET FEATURE
#define USB_REQUEST_SET_ADDRESS                 5       // Standard Device Request - SET ADDRESS
#define USB_REQUEST_GET_DESCRIPTOR              6       // Standard Device Request - GET DESCRIPTOR
#define USB_REQUEST_SET_DESCRIPTOR              7       // Standard Device Request - SET DESCRIPTOR
#define USB_REQUEST_GET_CONFIGURATION           8       // Standard Device Request - GET CONFIGURATION
#define USB_REQUEST_SET_CONFIGURATION           9       // Standard Device Request - SET CONFIGURATION
#define USB_REQUEST_GET_INTERFACE               10      // Standard Device Request - GET INTERFACE
#define USB_REQUEST_SET_INTERFACE               11      // Standard Device Request - SET INTERFACE
#define USB_REQUEST_SYNCH_FRAME                 12      // Standard Device Request - SYNCH FRAME

/*
 * HID requests
 */
#define USB_REQ_GET_REPORT                      0x01
#define USB_REQ_GET_IDLE                        0x02
#define USB_REQ_GET_PROTOCOL                    0x03
#define USB_REQ_SET_REPORT                      0x09
#define USB_REQ_SET_IDLE                        0x0A
#define USB_REQ_SET_PROTOCOL                    0x0B

#define USB_FEATURE_ENDPOINT_HALT               0       // CLEAR/SET FEATURE - Endpoint Halt
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP        1       // CLEAR/SET FEATURE - Device remote wake-up
#define USB_FEATURE_TEST_MODE                   2       // CLEAR/SET FEATURE - Test mode

/* Setup Data Constants */

#define USB_SETUP_HOST_TO_DEVICE                0x00    // Device Request bmRequestType transfer direction - host to device transfer
#define USB_SETUP_DEVICE_TO_HOST                0x80    // Device Request bmRequestType transfer direction - device to host transfer
#define USB_SETUP_TYPE_STANDARD                 0x00    // Device Request bmRequestType type - standard
#define USB_SETUP_TYPE_CLASS                    0x20    // Device Request bmRequestType type - class
#define USB_SETUP_TYPE_VENDOR                   0x40    // Device Request bmRequestType type - vendor
#define USB_SETUP_RECIPIENT_DEVICE              0x00    // Device Request bmRequestType recipient - device
#define USB_SETUP_RECIPIENT_INTERFACE           0x01    // Device Request bmRequestType recipient - interface
#define USB_SETUP_RECIPIENT_ENDPOINT            0x02    // Device Request bmRequestType recipient - endpoint
#define USB_SETUP_RECIPIENT_OTHER               0x03    // Device Request bmRequestType recipient - other

#define USB_RECIP_DEVICE                        USB_SETUP_RECIPIENT_DEVICE
#define USB_RECIP_INTERFACE                     USB_SETUP_RECIPIENT_INTERFACE
#define USB_RECIP_ENDPOINT                      USB_SETUP_RECIPIENT_ENDPOINT
#define USB_RECIP_OTHER                         USB_SETUP_RECIPIENT_OTHER

#define USB_TYPE_STANDARD                       USB_SETUP_TYPE_STANDARD
#define USB_TYPE_CLASS                          USB_SETUP_TYPE_CLASS
#define USB_TYPE_VENDOR                         USB_SETUP_TYPE_VENDOR

#define USB_DT_HID                              (USB_TYPE_CLASS | 0x01)
#define USB_DT_REPORT                           (USB_TYPE_CLASS | 0x02)
#define USB_DT_PHYSICAL                         (USB_TYPE_CLASS | 0x03)
#define USB_DT_HUB                              (USB_TYPE_CLASS | 0x09)

/* USB descriptors  */

#define USB_DESCRIPTOR_DEVICE                   0x01    // bDescriptorType for a Device Descriptor.
#define USB_DESCRIPTOR_CONFIGURATION            0x02    // bDescriptorType for a Configuration Descriptor.
#define USB_DESCRIPTOR_STRING                   0x03    // bDescriptorType for a String Descriptor.
#define USB_DESCRIPTOR_INTERFACE                0x04    // bDescriptorType for an Interface Descriptor.
#define USB_DESCRIPTOR_ENDPOINT                 0x05    // bDescriptorType for an Endpoint Descriptor.
#define USB_DESCRIPTOR_DEVICE_QUALIFIER         0x06    // bDescriptorType for a Device Qualifier.
#define USB_DESCRIPTOR_OTHER_SPEED              0x07    // bDescriptorType for a Other Speed Configuration.
#define USB_DESCRIPTOR_INTERFACE_POWER          0x08    // bDescriptorType for Interface Power.
#define USB_DESCRIPTOR_OTG                      0x09    // bDescriptorType for an OTG Descriptor.

#define HID_DESCRIPTOR_HID                      0x21



/* OTG SET FEATURE Constants    */
#define OTG_FEATURE_B_HNP_ENABLE                3       // SET FEATURE OTG - Enable B device to perform HNP
#define OTG_FEATURE_A_HNP_SUPPORT               4       // SET FEATURE OTG - A device supports HNP
#define OTG_FEATURE_A_ALT_HNP_SUPPORT           5       // SET FEATURE OTG - Another port on the A device supports HNP

/* USB Endpoint Transfer Types  */
#define USB_TRANSFER_TYPE_CONTROL               0x00    // Endpoint is a control endpoint.
#define USB_TRANSFER_TYPE_ISOCHRONOUS           0x01    // Endpoint is an isochronous endpoint.
#define USB_TRANSFER_TYPE_BULK                  0x02    // Endpoint is a bulk endpoint.
#define USB_TRANSFER_TYPE_INTERRUPT             0x03    // Endpoint is an interrupt endpoint.
#define bmUSB_TRANSFER_TYPE                     0x03    // bit mask to separate transfer type from ISO attributes


/* Standard Feature Selectors for CLEAR_FEATURE Requests    */
#define USB_FEATURE_ENDPOINT_STALL              0       // Endpoint recipient
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP        1       // Device recipient
#define USB_FEATURE_TEST_MODE                   2       // Device recipient

// USB Device Classes
#define USB_CLASS_USE_CLASS_INFO        0x00    // Use Class Info in the Interface Descriptors
#define USB_CLASS_AUDIO                 0x01    // Audio
#define USB_CLASS_COM_AND_CDC_CTRL      0x02    // Communications and CDC Control
#define USB_CLASS_HID                   0x03    // HID
#define USB_CLASS_PHYSICAL              0x05    // Physical
#define USB_CLASS_IMAGE                 0x06    // Image
#define USB_CLASS_PRINTER               0x07    // Printer
#define USB_CLASS_MASS_STORAGE          0x08    // Mass Storage
#define USB_CLASS_HUB                   0x09    // Hub
#define USB_CLASS_CDC_DATA              0x0a    // CDC-Data
#define USB_CLASS_SMART_CARD            0x0b    // Smart-Card
#define USB_CLASS_CONTENT_SECURITY      0x0d    // Content Security
#define USB_CLASS_VIDEO                 0x0e    // Video
#define USB_CLASS_PERSONAL_HEALTH       0x0f    // Personal Healthcare
#define USB_CLASS_DIAGNOSTIC_DEVICE     0xdc    // Diagnostic Device
#define USB_CLASS_WIRELESS_CTRL         0xe0    // Wireless Controller
#define USB_CLASS_MISC                  0xef    // Miscellaneous
#define USB_CLASS_APP_SPECIFIC          0xfe    // Application Specific
#define USB_CLASS_VENDOR_SPECIFIC       0xff    // Vendor Specific

/* descriptor data structures */

/* Device descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // DEVICE descriptor type (USB_DESCRIPTOR_DEVICE).
        uint16_t bcdUSB; // USB Spec Release Number (BCD).
        uint8_t bDeviceClass; // Class code (assigned by the USB-IF). 0xFF-Vendor specific.
        uint8_t bDeviceSubClass; // Subclass code (assigned by the USB-IF).
        uint8_t bDeviceProtocol; // Protocol code (assigned by the USB-IF). 0xFF-Vendor specific.
        uint8_t bMaxPacketSize0; // Maximum packet size for endpoint 0.
        uint16_t idVendor; // Vendor ID (assigned by the USB-IF).
        uint16_t idProduct; // Product ID (assigned by the manufacturer).
        uint16_t bcdDevice; // Device release number (BCD).
        uint8_t iManufacturer; // Index of String Descriptor describing the manufacturer.
        uint8_t iProduct; // Index of String Descriptor describing the product.
        uint8_t iSerialNumber; // Index of String Descriptor with the device's serial number.
        uint8_t bNumConfigurations; // Number of possible configurations.
} __attribute__((packed)) USB_DEVICE_DESCRIPTOR;

/* Configuration descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // CONFIGURATION descriptor type (USB_DESCRIPTOR_CONFIGURATION).
        uint16_t wTotalLength; // Total length of all descriptors for this configuration.
        uint8_t bNumInterfaces; // Number of interfaces in this configuration.
        uint8_t bConfigurationValue; // Value of this configuration (1 based).
        uint8_t iConfiguration; // Index of String Descriptor describing the configuration.
        uint8_t bmAttributes; // Configuration characteristics.
        uint8_t bMaxPower; // Maximum power consumed by this configuration.
} __attribute__((packed)) USB_CONFIGURATION_DESCRIPTOR;

/* Interface descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // INTERFACE descriptor type (USB_DESCRIPTOR_INTERFACE).
        uint8_t bInterfaceNumber; // Number of this interface (0 based).
        uint8_t bAlternateSetting; // Value of this alternate interface setting.
        uint8_t bNumEndpoints; // Number of endpoints in this interface.
        uint8_t bInterfaceClass; // Class code (assigned by the USB-IF).  0xFF-Vendor specific.
        uint8_t bInterfaceSubClass; // Subclass code (assigned by the USB-IF).
        uint8_t bInterfaceProtocol; // Protocol code (assigned by the USB-IF).  0xFF-Vendor specific.
        uint8_t iInterface; // Index of String Descriptor describing the interface.
} __attribute__((packed)) USB_INTERFACE_DESCRIPTOR;

/* Endpoint descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.
        uint8_t bDescriptorType; // ENDPOINT descriptor type (USB_DESCRIPTOR_ENDPOINT).
        uint8_t bEndpointAddress; // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
        uint8_t bmAttributes; // Endpoint transfer type.
        uint16_t wMaxPacketSize; // Maximum packet size.
        uint8_t bInterval; // Polling interval in frames.
} __attribute__((packed)) USB_ENDPOINT_DESCRIPTOR;

/* HID descriptor */
typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t bcdHID; // HID class specification release
        uint8_t bCountryCode;
        uint8_t bNumDescriptors; // Number of additional class specific descriptors
        uint8_t bDescrType; // Type of class descriptor
        uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) USB_HID_DESCRIPTOR;

typedef struct {
        uint8_t bDescrType; // Type of class descriptor
        uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) HID_CLASS_DESCRIPTOR_LEN_AND_TYPE;

#endif   // _USB_CH9_H_
