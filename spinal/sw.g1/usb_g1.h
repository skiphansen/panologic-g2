#ifndef _USB_G1_H_
#define _USB_G1_H_
#include "top_defines.h"

#ifdef __GNUC__
#define GCC_PACKED __attribute__ ((packed))
#else
#define GCC_PACKED
#endif

// Support a maximum of 6 devices (setup scratch, root hub, 
// built in external hub, 3 external devices)
#define MAX_USB_DEVICES    6
#define MAX_ENDPOINTS      4  // maybe this is enough ? setup, in/out/control ??

typedef int (ControlCB)(uint8_t Adr);

typedef struct {
   ControlCB *ControlInCB;
   uint16_t CtrlInBuf;        // address in 1780 memory of control data buffer
   uint8_t *UCtrlInBuf;
   size_t UCtrlInBufLen;
   uint8_t PipeType[MAX_ENDPOINTS];
   uint8_t Interval[MAX_ENDPOINTS];

   uint16_t LangID;

   uint8_t TTPort;
   uint8_t HubDevnum;
   uint8_t bDeviceClass;      // Class code (assigned by the USB-IF). 0xFF-Vendor specific.
   uint8_t bDeviceSubClass;   // Subclass code (assigned by the USB-IF).
   uint16_t MaxPacketSize;
   uint32_t UsbSpeed:2;
   uint32_t Toggle:1;
   uint32_t Ping:1;
   uint32_t Present:1;        // This device is present
   uint32_t LastPacket:1;
} GCC_PACKED PanoUsbDevice;

typedef int (ClaimDevice)(uint8_t Adr,uint8_t *Descriptors);

typedef struct UsbDriverIf_TAG {
   struct UsbDriverIf_TAG *pNext;
   ClaimDevice *ClaimDevice;
   const char *DriverName;
   uint8_t Adr;
   uint8_t EndPoint;
} UsbDriverIf;


void UsbInit(void);
bool UsbProbe(void);
void UsbTest(void);
void UsbRegDump(void);
void UsbInit(void);
void UsbRegisterDriver(UsbDriverIf *pDriverIf);
uint8_t *FindDesc(uint8_t Type,uint8_t *Buf);
int OpenControlInPipe(uint8_t Adr,uint8_t Endpoint,ControlCB *Funct,uint8_t *InBuf,size_t Len);
int SendUsbCtrlMsg(uint8_t Adr,unsigned char request,unsigned char requesttype,
                   unsigned short value,unsigned short index,void *data, 
                   unsigned short size,int timeout);
#endif   // _USB_G1_H_

