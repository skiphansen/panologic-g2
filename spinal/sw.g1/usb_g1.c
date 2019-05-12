/* 
 
Code in this file is free software; you can redistribute it and/or modify
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

Code in this file is derived and/or copied from many sources including:
 
GPL:
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
http://www.circuitsathome.com 
 
Linux driver for the NXP ISP1760 chip
Copyright 2014 Laurent Pinchart
Copyright 2007 Sebastian Siewior 
 
This is a very minimal USB host driver with specific goals and limitations. 
 
Goals in priority order:
1. Provide USB game controller support for vintage games. 
2. Provide mass storage support for loading programs and games.
3. Provide keyboard support for development and vintage computing. 
 
This effort was started prior to Wenting Zhang's amazing work with getting 
the Pano G1's SDRAM running so code space was at an extreem premimum. 
 
After Wenting got SDRAM working he also ported the u-boot USB stack to the 
Pano so there's probably little to contiue to use this driver.  However it 
might be useful for a low foot print bootrom or other memory constrained 
usages someday.
 
Limitations: 
1. Only support up to 3 USB perpherials connected DIRECTLY to one of 
the 3 USB ports on the Pano.  Code is much simplifed by limiting the number 
of devices as well as eliminating the need for general USB HUB support. 
 
2. Game controller support will be limited to a few specific devices, no 
attempt will be made to parse the HID decsciptors. 
 
3. Keyboard support will be limited to keyboards that support the optional boot 
mode. 
 
4. Hot pulling of USB devices is not supported to simplify the logic.
*/
 
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "usb_g1.h"
#include "global.h"
#include "reg.h"
#include "isp1760-regs.h"
#include "print.h"
#include "usb_ch9.h"
#include "gpio.h"
#include "usbhub.h"
#include "printf.h"

int button_pressed(void);

#define PRINTF(format, ...) printf(format, ## __VA_ARGS__)
// Error logging macro, always enabled
#define ELOG(format, ...) printf("%s: " format,__FUNCTION__ ,## __VA_ARGS__)

#define USB_VERBOSE_PRINT
#ifdef USB_VERBOSE_PRINT
   #define LOG(format, ...) printf("%s: " format,__FUNCTION__ ,## __VA_ARGS__)
   #define LOG_RAW(format, ...) printf(format,## __VA_ARGS__)
// Debugging printouts
   void DumpInterfaceDesc(uint8_t Adr,USB_INTERFACE_DESCRIPTOR *p);
   void DumpEndpointDesc(USB_ENDPOINT_DESCRIPTOR *p);
   void DumpClass(const char *Msg,uint8_t bClass);
   void DumpStringDesc(const char *Label,uint8_t Index,uint8_t Adr);
   void UsbRegDump(void);
   void DumpConfigDesc(uint8_t Adr,USB_CONFIGURATION_DESCRIPTOR *pConfigDesc);
   void DumpDeviceDesc(uint8_t Adr,USB_DEVICE_DESCRIPTOR *pDevDesc);
   void DumpPtd(const char *msg,u32 *p);
   void Dump1760Mem(void);
   void DumpHubDesc(uint8_t Adr,struct HubDescriptor *pHubDesc);
   void DumpPortStatus(uint16_t Port,uint32_t Status);
   void DumpHidDesc(USB_HID_DESCRIPTOR *pDesc);
#else
   #define LOG(format, ...)
   #define LOG_RAW(format, ...)
// Debugging printouts
   #define DumpInterfaceDesc(x,y)
   #define DumpEndpointDesc(x)
   #define DumpClass(x,y)
   #define DumpStringDesc(x,y,z)
   #define UsbRegDump()
   #define DumpConfigDesc(x,y)
   #define DumpDeviceDesc(x,y)
   #define DumpPtd(x,y)
   #define Dump1760Mem(x)
   #define ReadAndDumpIntPtd(x)
   #define DumpHubDesc(x,y)
   #define DumpPortStatus(x,y)
   #define DumpHidDesc(x)
#endif


#define WAIT_CYCLES_1MS    1190

#define RW_TEST_VALUE   0x5555aaaa
#define INT_REG_RESERVED_BITS 0xfffffc15

#define ROOT_HUB_ADR       1     // hub built into the isp1760
#define EXTERNAL_HUB_ADR   2     // Pano's built in hub

/* Common setup data constant combinations  */
#define bmREQ_GET_DESCR     USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE     //get descriptor request type
#define bmREQ_SET           USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE     //set request type for all but 'set feature' and 'set interface'
#define bmREQ_SET_INTF      USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_INTERFACE  //set interface request type
#define bmREQ_CL_GET_INTF   USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_INTERFACE     //get interface request type

/* USB Setup Packet Structure   */
typedef struct {
   union {
// byte 0: Bit-map of request type
      uint8_t bmRequestType;  
      struct {
         uint8_t recipient : 5;  // Recipient of the request
         uint8_t type : 2;       // Type of request
         uint8_t direction : 1;  // Direction of data X-fer
      } GCC_PACKED;
   } ReqType_u;

// byte 1: Request
   uint8_t bRequest;          

// bytes 2,3
   union {
      uint16_t wValue;        
      struct {
         uint8_t wValueLo;
         uint8_t wValueHi;
      } GCC_PACKED;
   } wVal_u;

// bytes 4,5
   uint16_t wIndex;           
// bytes 6,7
   uint16_t wLength;
} GCC_PACKED SetupPkt;


// 
#define USB_SPEED_LOW      0
#define USB_SPEED_USB11    1  // 12 Mb
#define USB_SPEED_HIGH     2  // 480 Mb

#define PTYPE_BULK         0
#define PTYPE_INT          1
#define PTYPE_CONTROL      2

UsbDriverIf *gDriverHead;

uint8_t gDumpPtd = 0;

// Indexed by USB address
PanoUsbDevice gUsbDevice[MAX_USB_DEVICES + 1];

/* Philips Proprietary Transfer Descriptor (PTD) */
struct ptd {
   u32 dw0;
   u32 dw1;
   u32 dw2;
   u32 dw3;
   u32 dw4;
   u32 dw5;
   u32 dw6;
   u32 dw7;
};

#define PTD_OFFSET         0x0400
#define ISO_PTD_OFFSET     0x0400
#define INT_PTD_OFFSET     0x0800
#define ATL_PTD_OFFSET     0x0c00
#define PAYLOAD_OFFSET     0x1000

/* ATL */
/* DW0 */
#define DW0_VALID_BIT         1
#define FROM_DW0_VALID(x)     ((x) & 0x01)
#define TO_DW0_LENGTH(x)      (((u32) x) << 3)
#define TO_DW0_MAXPACKET(x)      (((u32) x) << 18)
#define TO_DW0_MULTI(x)       (((u32) x) << 29)
#define TO_DW0_ENDPOINT(x)    (((u32)  x) << 31)
/* DW1 */
#define TO_DW1_DEVICE_ADDR(x)    (((u32) x) << 3)
#define TO_DW1_PID_TOKEN(x)      (((u32) x) << 10)
#define DW1_TRANS_BULK        ((u32) 2 << 12)
#define DW1_TRANS_INT         ((u32) 3 << 12)
#define DW1_TRANS_SPLIT       ((u32) 1 << 14)
#define DW1_SE_USB_LOSPEED    ((u32) 2 << 16)
#define TO_DW1_PORT_NUM(x)    (((u32) x) << 18)
#define TO_DW1_HUB_NUM(x)     (((u32) x) << 25)
/* DW2 */
#define TO_DW2_DATA_START_ADDR(x)   (((u32) x) << 8)
#define TO_DW2_RL(x)       ((x) << 25)
#define FROM_DW2_RL(x)        (((x) >> 25) & 0xf)
/* DW3 */
#define FROM_DW3_NRBYTESTRANSFERRED(x)    ((x) & 0x7fff)
#define FROM_DW3_SCS_NRBYTESTRANSFERRED(x)   ((x) & 0x0fff) // wsa 0x7ff
#define TO_DW3_NAKCOUNT(x)    ((x) << 19)
#define FROM_DW3_NAKCOUNT(x)     (((x) >> 19) & 0xf)
#define TO_DW3_CERR(x)        ((x) << 23)
#define FROM_DW3_CERR(x)      (((x) >> 23) & 0x3)
#define DW3_DATA_TOGGLE_BIT   (1 << 25)
#define TO_DW3_DATA_TOGGLE(x)    ((x) << 25)
#define FROM_DW3_DATA_TOGGLE(x)     (((x) >> 25) & 0x1)
#define TO_DW3_PING(x)        ((x) << 26)
#define FROM_DW3_PING(x)      (((x) >> 26) & 0x1)
#define DW3_SC_BIT            (1 << 27)
#define DW3_ERROR_BIT         (1 << 28)
#define DW3_BABBLE_BIT        (1 << 29)
#define DW3_HALT_BIT       (1 << 30)
#define DW3_ACTIVE_BIT        (1 << 31)
#define FROM_DW3_ACTIVE(x)    (((x) >> 31) & 0x01)

#define INT_UNDERRUN       (1 << 2)
#define INT_BABBLE         (1 << 1)
#define INT_EXACT       (1 << 0)

#define SETUP_PID (2)
#define IN_PID    (1)
#define OUT_PID      (0)

/* Errata 1 */
#define RL_COUNTER   (0)
#define NAK_COUNTER  (0)
//#define ERR_COUNTER  (2)
#define ERR_COUNTER  (1)

// Internal 1760 memory
#define SETUP_CMD_BUF      0x2000
#define SETUP_RESP_BUF     (SETUP_CMD_BUF + 8)

// Max control payload is 1024 bytes for high speed
// one per device except for device 0 (setup scratch) and device 1(root hub)
#define INTERRUPT_IN_BUF   (SETUP_RESP_BUF + 4096)
#define INTERRUPT_IN_SIZE  1024
// Address 0 is only used during setup and address 1 is the root hub, neither
// need interrupt buffers
#define INT_IN_BUF_ADR(x)  (INTERRUPT_IN_BUF + ((x- 2) * INTERRUPT_IN_SIZE))
#define INT_PTD_ADR(x)     (INT_PTD_OFFSET + (x* 8 * sizeof(uint32_t)))

// 
static u32 isp1760_read32(u32 reg);
static void isp1760_write32(u32 reg,u32 Value);
static void isp1760_bits(u32 reg,u32 Sets,u32 Resets);

void UsbTest(void);
void print_1cr(const char *label,int value);
void InitTest(void);
int _DoTransfer(u32 *ptd,const char *Func,int Line);
#define DoTransfer(x)  _DoTransfer(x,__FUNCTION__,__LINE__)

int SetConfiguration(uint8_t Adr,uint8_t Configuration);
int SetHubFeature(uint16_t Adr,uint8_t bmRequestType,uint8_t bRequest,uint16_t wIndex);
int ClearHubFeature(uint8_t Adr,uint8_t bmRequestType,uint8_t bRequest,uint16_t wIndex);
int SetInterface(uint8_t Adr,uint16_t AltSetting,uint16_t Interface);
int GetPortStatus(uint8_t Adr,uint16_t Port,uint32_t *pStatus);
int GetHubDesc(uint16_t Adr);
void InitPtd(u32 *Ptd,uint8_t Adr,uint8_t EndPoint,uint8_t Pid,u16 PayLoadAdr,int Len);
int SetupTransaction(uint8_t Adr,SetupPkt *p,void *pResponse,int ResponseLen);
int GetDevDesc(uint8_t Adr);
int GetConfigDesc(uint8_t Adr);
int SetUsbAddress(uint8_t Adr);
u32 base_to_chip(u32 base);
void SetDebugLED(bool bOn);
u32 PollUsbInt(void);
void TransformPtd2Int(u32 *Ptd,uint8_t Adr,uint8_t EndPoint);
int GetStringDesc(uint8_t Adr,uint16_t LangId,uint8_t Index,uint8_t *Buf,size_t MaxLen);
void ConfigureDev(uint8_t Adr);

void msleep(int ms)
{
    volatile int cnt = 0;
    int cycles = ms * WAIT_CYCLES_1MS;

    while(cnt++ < cycles);
}

static u32 isp1760_read32(u32 reg)
{
   uint16_t Lsb;
   uint16_t Msb;

   Lsb = REG16_RD(USB_BASE_ADDR + (reg * 2));
   Msb = REG16_RD(USB_BASE_ADDR + (reg * 2) + 4);

   return Lsb + (Msb << 16);
}

static void isp1760_write32(u32 reg,u32 Value)
{
   REG16_WR(USB_BASE_ADDR + (reg * 2),(u16) (Value & 0xffff));
   REG16_WR(USB_BASE_ADDR + (reg  * 2) + 4,(u16) ((Value >> 16) & 0xffff));
}

/*
 * Access functions for isp176x memory
 */
static void mem_reads8(u32 src, u32 *dst, u32 bytes)
{
   u32 val;
   unsigned char *dst_byteptr = (unsigned char *) dst;
   unsigned char *src_byteptr;
   int Bytes2Copy;

   if(src >= PTD_OFFSET) {
      isp1760_write32(HC_MEMORY_REG,src);
   }

   while(bytes > 0) {
      val = isp1760_read32(src);
      src_byteptr = (unsigned char *) &val;
      Bytes2Copy = bytes;
      if(Bytes2Copy > 4) {
         Bytes2Copy = 4;
      }
      bytes -= Bytes2Copy;
      while(Bytes2Copy-- > 0) {
         *dst_byteptr++ = *src_byteptr++;
      }
   }
}

static void mem_writes8(u32 dst,u32 *src, u32 bytes)
{
   while (bytes >= 4) {
      isp1760_write32(dst,*src++);
      bytes -= 4;
      dst += 4;
   }

   if(bytes > 0) {
   // Handle remaining bytes
      u32 val = 0;
      unsigned char *src_byteptr = (unsigned char *) src;
      unsigned char *dst_byteptr = (unsigned char *) &val;
      while(bytes-- > 0) {
         *dst_byteptr++ = *src_byteptr++;
      }
      isp1760_write32(dst,val);
   }
}

/*
 * Read and write ptds. 'ptd_offset' should be one of ISO_PTD_OFFSET,
 * INT_PTD_OFFSET, and ATL_PTD_OFFSET. 'slot' should be less than 32.
 */
static void ptd_read(u32 ptd_offset, u32 slot,struct ptd *ptd)
{
   mem_reads8(ptd_offset + slot*sizeof(*ptd),(u32 *) ptd, sizeof(*ptd));
}

static void ptd_write(u32 ptd_offset, u32 slot,struct ptd *ptd)
{
   mem_writes8(ptd_offset + slot*sizeof(*ptd) + sizeof(ptd->dw0),
                  &ptd->dw1, 7*sizeof(ptd->dw1));
   /* Make sure dw0 gets written last (after other dw's and after payload)
      since it contains the enable bit */
   mem_writes8(ptd_offset + slot*sizeof(*ptd), &ptd->dw0,sizeof(ptd->dw0));
}

static void isp1760_bits(u32 reg,u32 Sets,u32 Resets)
{
   u32 Value = isp1760_read32(reg);
   Value |= Sets;
   Value &= ~Resets;
   isp1760_write32(reg,Value);
}


void UsbInit()
{
   int i;
   InitTest();
// clear the memory 
   for(i = 0; i < 16128; i++) {
      isp1760_write32(0x400 + (i * 4),0);
   }
   return;

// Select 16 bit mode
   isp1760_write32(HC_HW_MODE_CTRL,ALL_ATX_RESET);
   msleep(15);
   isp1760_write32(HC_HW_MODE_CTRL,0);
   isp1760_write32(HC_HW_MODE_CTRL,0);
}

#if 0
// Return 1 if USB controller is detected
bool UsbProbe(void)
{
   bool Ret = false;  // Assume the worse
   u32 Value;
   const char *Err = NULL;
   u32 i;

   do {
      isp1760_write32(HC_SCRATCH_REG,RW_TEST_VALUE);
      Value = isp1760_read32(HC_CHIP_ID_REG);
      if(Value != 0x11761) {
         Err = "HC_CHIP_ID_REG";
         break;
      }
      Value = isp1760_read32(HC_SCRATCH_REG);
      if(Value != RW_TEST_VALUE) {
         Err = "HC_SCRATCH_REG";
         break;
      }
      isp1760_write32(HC_SCRATCH_REG,~RW_TEST_VALUE);
      Value = isp1760_read32(HC_SCRATCH_REG);
      if(Value != (u32) ~RW_TEST_VALUE) {
         Err = "HC_SCRATCH_REG #2";
         break;
      }
      print("Memory test\n");
      for(i = 0; i < 16128; i++) {
         isp1760_write32(0x400 + (i * 4),i);
      }

      isp1760_write32(HC_MEMORY_REG,0x400);
      for(i = 0; i < 16128; i++) {
         Value = isp1760_read32(0x400);
         if(Value != i) {
            print_int(i,1);

            Err = "\nMemory test";
            break;
         }
      }

      if(i == 16128) {
         print("\npassed\n");
      }
      for(i = 0; i < 16128; i++) {
         isp1760_write32(0x400 + (i * 4),0);
      }

      Ret = true;
      UsbRegDump();
   } while(false);

   if(Err != NULL) {
      print(Err);
      print(" failed, read ");
      print_int(Value,1);
      print("\n");
   }

   return Ret;
}
#endif



void UsbTest()
{
   int i;
   uint32_t PortStatus;
   uint8_t Adr;

#if 0
   u32 Value;
// Reset
   print("Reseting isp1760...");
//   isp1760_write32(HC_RESET_REG,SW_RESET_RESET_HC | SW_RESET_RESET_ALL);
//   isp1760_write32(HC_RESET_REG,SW_RESET_RESET_HC);
// Wait for reset to complete
   msleep(50);
   for( ; ; ) {
      Value = isp1760_read32(HC_RESET_REG);
      if((Value & SW_RESET_RESET_HC) == 0) {
         break;
      }
   }

   // isp1760_write32(HC_HW_MODE_CTRL,0);
   UsbInit();  // Back to 16 bit bus mode & ATX reset

// Enable port 1
   isp1760_bits(HC_PORT1_CTRL,PORT1_POWER | PORT1_INIT2 | PORT1_INIT1 ,0);
//    isp1760_write32(HC_PORT1_CTRL,0x80018);  // note 1 following table 54

// Reset host controller
   isp1760_write32(HC_USBCMD,CMD_RESET);

#if 0
   print("\nSet FLAG_CF\n");
   isp1760_write32(HC_CONFIGFLAG,FLAG_CF);   // Set configure flag
#endif

// clear the PTD memory region
   for(i = PTD_OFFSET; i < PAYLOAD_OFFSET; i += 4) {
      isp1760_write32(i,0);
   }

   print("Reset root hub...");
   isp1760_write32(HC_PORTSC1,PORT_POWER);
   isp1760_write32(HC_PORTSC1,PORT_RESET | PORT_POWER);
   msleep(50);
   isp1760_write32(HC_PORTSC1,PORT_POWER);

// Wait for controller to clear PORT_RESET
   for( ; ; ) {
      Value = isp1760_read32(HC_PORTSC1);
      if((Value & PORT_RESET) == 0) {
         break;
      }
   }
   print_1cr("\nHC_PORTSC1 after reset",Value);

// ARG !  AN10037 uses an example of 0x0001 0021 which changes some "reserved"
// bits that the spec sheet sheet says should never be changed, but the
// EHCI spec documents these "reserved" bits.  The comment in AN10037
// is "Result: R/S = 1; ITC[7:0] = 01h." where ITC is apparently the
// Interrupt Threshold Control field.  The EHCI spec documents bit 5 as the 
// Asynchronouse Schedule Enable bit.

// isp1760_write32(HC_USBCMD,0x10021);

   isp1760_bits(HC_USBCMD,CMD_RUN,0);
#if 0
// Set ATL Skip Map register
   isp1760_write32(HC_ATL_PTD_SKIPMAP_REG,0xffffffe);
#endif

   print("\nSet FLAG_CF\n");
   isp1760_write32(HC_CONFIGFLAG,FLAG_CF);   // Set configure flag

// Set ATL last PTD register
   isp1760_write32(HC_ATL_PTD_LASTPTD_REG,0x2);

// Wait 2ms for reset recovery
   msleep(2);
#endif

   isp1760_write32(HC_ATL_IRQ_MASK_OR_REG,1);
   isp1760_write32(HC_INTERRUPT_ENABLE,HC_ATL_INT | HC_SOT_INT);

#if 0
   LOG("Waiting for button press\n");
   while(!button_pressed());
#endif

// Get device descriptor from the root hub
   LOG("Get root hub device desc\n");
   gUsbDevice[0].MaxPacketSize = 64;
   gUsbDevice[0].UsbSpeed = USB_SPEED_HIGH;
   GetDevDesc(0);
   LOG("Set root hub adr to %d\n",ROOT_HUB_ADR);
   SetUsbAddress(ROOT_HUB_ADR);
   LOG("Set root hub configuration\n");
   SetConfiguration(ROOT_HUB_ADR,1);
   LOG("Get root hub desc\n");
   GetHubDesc(ROOT_HUB_ADR);

   LOG("Enable power on port 3\n");
// Only Port 3 is connected
   SetHubFeature(ROOT_HUB_ADR,bmREQ_SET_PORT_FEATURE,HUB_FEATURE_PORT_POWER,3);

   msleep(150);
   LOG("After powering up port 3\n");
   ClearHubFeature(ROOT_HUB_ADR,bmREQ_CLEAR_PORT_FEATURE,HUB_FEATURE_C_PORT_CONNECTION,3);
   SetHubFeature(ROOT_HUB_ADR,bmREQ_SET_PORT_FEATURE,HUB_FEATURE_PORT_RESET,3);
   GetPortStatus(ROOT_HUB_ADR,3,&PortStatus);
   DumpPortStatus(3,PortStatus);

   msleep(1000);
   GetPortStatus(ROOT_HUB_ADR,3,&PortStatus);
   DumpPortStatus(3,PortStatus);

// Get device descriptor from the root hub
   LOG("\nGet external hub device desc\n");
   GetDevDesc(0);
   LOG("Set adr to %d\n",EXTERNAL_HUB_ADR);
   SetUsbAddress(EXTERNAL_HUB_ADR);
   LOG("Set configuration to 1\n");
   SetConfiguration(EXTERNAL_HUB_ADR,1);
   LOG("Get external hub desc\n");
   GetHubDesc(EXTERNAL_HUB_ADR);
   LOG("Get external hub configuration desc\n");
   GetConfigDesc(EXTERNAL_HUB_ADR);

// Power up all three ports
   for(i = 0; i < 3; i++) {
      LOG("Power UP external hub port %d\n",i+1);
      SetHubFeature(EXTERNAL_HUB_ADR,
                    bmREQ_SET_PORT_FEATURE,HUB_FEATURE_PORT_POWER,i+1);
   }
   msleep(1000);
   LOG("Port status after power up\n");
// Check status of ports
   Adr = EXTERNAL_HUB_ADR + 1;
   for(i = 0; i < 3; i++) {
      GetPortStatus(EXTERNAL_HUB_ADR,i+1,&PortStatus);
      DumpPortStatus(i+1,PortStatus);
      if(PortStatus & bmHUB_PORT_STATUS_PORT_CONNECTION) {

         LOG("Device connected to port %d, power up port\n",i+1);
         SetHubFeature(EXTERNAL_HUB_ADR,
                       bmREQ_SET_PORT_FEATURE,HUB_FEATURE_PORT_POWER,i+1);
      // 9.1.2: The host then waits for at least 100 ms to allow completion 
      // of an insertion process and for power at the device to become stable.
         msleep(150);
         LOG("Reset port %d\n",i+1);
         SetHubFeature(EXTERNAL_HUB_ADR,bmREQ_SET_PORT_FEATURE,
                       HUB_FEATURE_PORT_RESET,i+1);
         msleep(20);
         GetPortStatus(EXTERNAL_HUB_ADR,i+1,&PortStatus);
         DumpPortStatus(i+1,PortStatus);
         LOG("Get device desc, port %d\n",i+1);
         gUsbDevice[0].TTPort = (uint8_t) (i + 1);
         gUsbDevice[0].HubDevnum = EXTERNAL_HUB_ADR;

      // Maximum packet length of control transfers for low speed: 8 bytes,
      // high speed: 8, 16, 32 or 64 bytes,
      // full speed: 64 bytes.

      // Maximum data payload size low-speed: 8 bytes, full-speed: 64 bytes,
      // high-speed: 1024 bytes
         if(PortStatus & bmHUB_PORT_STATUS_PORT_LOW_SPEED) {
            gUsbDevice[0].MaxPacketSize = 8;
            gUsbDevice[0].UsbSpeed = USB_SPEED_LOW;
         }
         else if(PortStatus & bmHUB_PORT_STATUS_PORT_HIGH_SPEED) {
            gUsbDevice[0].MaxPacketSize = 1024;
            gUsbDevice[0].UsbSpeed = USB_SPEED_HIGH;
         }
         else {
            gUsbDevice[0].MaxPacketSize = 8;
            gUsbDevice[0].UsbSpeed = USB_SPEED_USB11;
         }
         ConfigureDev(Adr);
#if 0
         GetDevDesc(0);
         ELOG("Set adr to %d for device on port %d\n",Adr,i+1);
         SetUsbAddress(Adr);
         LOG("Get configuration descriptor for device on port %d\n",i+1);
         GetConfigDesc(Adr);
         LOG("Set configuration to 1\n");
         SetConfiguration(Adr,1);
#endif
      }
      Adr++;
   }

   for( ; ; ) {
      PollUsbInt();
      if(usb_kbd_testc()) {
         printf("%c",usb_kbd_getc());
      }
      if(button_pressed()) {
         ReadAndDumpIntPtd(4);
         ReadAndDumpIntPtd(5);
         UsbRegDump();
         Dump1760Mem();
         while(button_pressed());
      }
   }

//   Dump1760Mem();
}

int SetUsbAddress(uint8_t Adr)
{
   SetupPkt Pkt;
   int Ret;

// copy data read from device descriptor while was address zero
   memcpy(&gUsbDevice[Adr],&gUsbDevice[0],sizeof(PanoUsbDevice));
   gUsbDevice[0].LangID = 0;  // reset for next time

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_SET;
   Pkt.bRequest = USB_REQUEST_SET_ADDRESS;
   Pkt.wVal_u.wValueLo = Adr;
   Pkt.wVal_u.wValueHi = 0;
   Pkt.wIndex = 0;
   Pkt.wLength = 0;
   Ret = SetupTransaction(0,&Pkt,NULL,0);
/* After a device receives a SetAddress() request, the device must be able
   to complete processing of the request and be able to successfully complete
   the Status stage of the request within 50 ms.*/
   msleep(50);
   return Ret;
}

int SetConfiguration(uint8_t Adr,uint8_t Configuration)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_SET;
   Pkt.bRequest = USB_REQUEST_SET_CONFIGURATION;
   Pkt.wVal_u.wValueLo = Configuration;
   Pkt.wVal_u.wValueHi = 0;
   Pkt.wIndex = 0;
   Pkt.wLength = 0;

   return SetupTransaction(Adr,&Pkt,NULL,0);
}

int SetHubFeature(uint16_t Adr,uint8_t bmRequestType,uint8_t bRequest,uint16_t wIndex)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmRequestType;
   Pkt.bRequest = USB_REQUEST_SET_FEATURE;
   Pkt.wVal_u.wValueLo = bRequest;
   Pkt.wVal_u.wValueHi = 0;
   Pkt.wIndex = wIndex;
   Pkt.wLength = 0;

   return SetupTransaction(Adr,&Pkt,NULL,0);
}

int ClearHubFeature(uint8_t Adr,uint8_t bmRequestType,uint8_t bRequest,uint16_t wIndex)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmRequestType;
   Pkt.bRequest = USB_REQUEST_CLEAR_FEATURE;
   Pkt.wVal_u.wValueLo = bRequest;
   Pkt.wVal_u.wValueHi = 0;
   Pkt.wIndex = wIndex;
   Pkt.wLength = 0;

   return SetupTransaction(Adr,&Pkt,NULL,0);
}

int SetInterface(uint8_t Adr,uint16_t AltSetting,uint16_t Interface)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_SET;
   Pkt.bRequest = USB_REQUEST_SET_INTERFACE;
   Pkt.wVal_u.wValue = AltSetting;
   Pkt.wIndex = Interface;
   Pkt.wLength = 0;

   return SetupTransaction(Adr,&Pkt,NULL,0);
}


int GetPortStatus(uint8_t Adr,uint16_t Port,uint32_t *pStatus)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_GET_PORT_STATUS;
   Pkt.bRequest = USB_REQUEST_GET_STATUS;
   Pkt.wVal_u.wValueLo = 0;
   Pkt.wVal_u.wValueHi = 0;
   Pkt.wIndex = Port;
   Pkt.wLength = sizeof(*pStatus);

   return SetupTransaction(Adr,&Pkt,pStatus,sizeof(*pStatus));
}

int GetHubDesc(uint16_t Adr)
{
   SetupPkt Pkt;
   struct HubDescriptor Desc;
   int Err;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_GET_HUB_DESCRIPTOR;
   Pkt.bRequest = USB_REQUEST_GET_DESCRIPTOR;
   Pkt.wVal_u.wValueLo = 0;
   Pkt.wVal_u.wValueHi = 0x29;
   Pkt.wIndex = 0;
   Pkt.wLength = sizeof(Desc);

   do {
      Err = SetupTransaction(Adr,&Pkt,(int8_t *)&Desc,sizeof(Desc));
      if(Err < 0) {
         break;
      }
      DumpHubDesc(Adr,&Desc);
      if(Desc.bDescLength > 16) {
      }
   } while(false);

   return Err;
}

// return number of bytes read or < 0 for error
int GetDesc(uint8_t ReqType, uint8_t Type,uint8_t Adr,uint8_t *Buf,size_t BufLen)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = ReqType;
   Pkt.bRequest = USB_REQUEST_GET_DESCRIPTOR;
   Pkt.wVal_u.wValueLo = 0;
   Pkt.wVal_u.wValueHi = Type;
   Pkt.wIndex = 0;
   Pkt.wLength = BufLen;

   return SetupTransaction(Adr,&Pkt,Buf,BufLen);
}

int GetDevDesc(uint8_t Adr)
{
   USB_DEVICE_DESCRIPTOR DevDesc;
   SetupPkt Pkt;
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   int Err = 0;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_GET_DESCR;
   Pkt.bRequest = USB_REQUEST_GET_DESCRIPTOR;
   Pkt.wVal_u.wValueLo = 0;
   Pkt.wVal_u.wValueHi = USB_DESCRIPTOR_DEVICE;
   Pkt.wIndex = 0;
   Pkt.wLength = sizeof(USB_DEVICE_DESCRIPTOR);

   do {
      Err = SetupTransaction(Adr,&Pkt,(int8_t *)&DevDesc,sizeof(DevDesc));
      if(Err < 0) {
         break;
      }

      DumpDeviceDesc(Adr,&DevDesc);

      pDev->bDeviceClass = DevDesc.bDeviceClass;
      pDev->bDeviceSubClass = DevDesc.bDeviceSubClass;
      pDev->MaxPacketSize = DevDesc.bMaxPacketSize0;
   } while(false);

   return Err;
}

int GetConfigDesc(uint8_t Adr)
{
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   union {
      USB_CONFIGURATION_DESCRIPTOR ConfigDesc;
      uint8_t OtherDesc[350];
   } u;
   SetupPkt Pkt;
   int BytesLeft;
   int Err = 0;
   uint8_t *p = u.OtherDesc;
   uint8_t bMultiTT = false;
   uint8_t bAlternateSetting;
   uint8_t bInterfaceNumber;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_GET_DESCR;
   Pkt.bRequest = USB_REQUEST_GET_DESCRIPTOR;
   Pkt.wVal_u.wValueLo = 0;
   Pkt.wVal_u.wValueHi = USB_DESCRIPTOR_CONFIGURATION;
   Pkt.wIndex = 0;
   Pkt.wLength = sizeof(u);

   do {
      Err = SetupTransaction(Adr,&Pkt,(int8_t *)&u,sizeof(u));
      LOG("SetupTransaction returned: %d\n",Err);
      if(Err < 0) {
         break;
      }
      BytesLeft = Err;
      DumpConfigDesc(Adr,&u.ConfigDesc);

/*  A request for a configuration descriptor returns the configuration
    descriptor, all interface descriptors, and endpoint descriptors for all
    of the interfaces in a single request.
 
    The first interface descriptor follows the configuration descriptor.
    The endpoint descriptors for the first interface follow the first
    interface descriptor.
 
    If there are additional interfaces, their interface descriptor and
    endpoint descriptors follow the first interface�s endpoint descriptors.
 
    Class-specific and/or vendor-specific descriptors follow the standard
    descriptors they extend or modify.
*/
      if(u.ConfigDesc.wTotalLength > sizeof(u)) {
         ELOG("Error: OtherDesc buffer too small\n");
         break;
      }
      p += u.ConfigDesc.bLength;
      BytesLeft -= u.ConfigDesc.bLength;

      while(BytesLeft > 0) {
         if(BytesLeft < *p) {
            LOG("Error: not enough bytes left %d < %d\n",BytesLeft,*p);
            break;
         }
         switch(p[1]) {
            case USB_DESCRIPTOR_INTERFACE: {
               USB_INTERFACE_DESCRIPTOR *pDesc = (USB_INTERFACE_DESCRIPTOR *) p;
               DumpInterfaceDesc(Adr,pDesc);
               if(pDesc->bInterfaceClass == USB_CLASS_HUB &&
                  pDesc->bInterfaceProtocol == 2) 
               {
                  bMultiTT = true;
                  bAlternateSetting = pDesc->bAlternateSetting;
                  bInterfaceNumber = pDesc->bInterfaceNumber;
               }
               break;
            }

            case USB_DESCRIPTOR_ENDPOINT: {
               USB_ENDPOINT_DESCRIPTOR *pDesc = (USB_ENDPOINT_DESCRIPTOR *) p;
               uint8_t Endpoint = pDesc->bEndpointAddress & 0x7f;
               DumpEndpointDesc(pDesc);
               if(Endpoint < MAX_ENDPOINTS) {
                  pDev->Interval[Endpoint] = pDesc->bInterval;
                  LOG("Endpoint %d interval %d\n",Endpoint,pDesc->bInterval);
               }
               else {
                  ELOG("Error Endpoint %d > MAX_ENDPOINTS\n",Endpoint);
               }
               break;
            }

            case HID_DESCRIPTOR_HID: {
               USB_HID_DESCRIPTOR *pDesc = (USB_HID_DESCRIPTOR *) p;
               DumpHidDesc(pDesc);
               break;
            }

            default:
               LOG_RAW("  Skipping descriptor type 0x%x\n",p[1]);
               break;
         }
         BytesLeft -= *p;
         p += *p;
      }
   } while(false);

   if(bMultiTT) {
      LOG("MultiTT hub, selecting altsetting %d on interface %d\n",
          bAlternateSetting,bInterfaceNumber);
      Pkt.ReqType_u.bmRequestType = bmREQ_SET_INTF;
      Pkt.bRequest = USB_REQUEST_SET_INTERFACE;
      Pkt.wVal_u.wValue = bAlternateSetting;
      Pkt.wIndex = bInterfaceNumber;
      Pkt.wLength = 0;
      SetupTransaction(Adr,&Pkt,NULL,0);
   }

   return Err;
}

int SendUsbCtrlMsg(
   uint8_t Adr,
   unsigned char request,
   unsigned char requesttype,
   unsigned short value,
   unsigned short index,
   void *data, 
   unsigned short size,
   int timeout
)
{
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   SetupPkt Pkt;
   int Err;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = requesttype;
   Pkt.bRequest = request;
   Pkt.wVal_u.wValue = value;
   Pkt.wIndex = index;
   Pkt.wLength = size;

   Err = SetupTransaction(Adr,&Pkt,(int8_t *)data,size);

   return Err;
}



/*
[ISP176x] � How does the ISP176x host controller Linux 2.6.9 HCD perform host
controller initialization? What is the register initialization sequence and register
content during the host controller initialization?
Resetting the ISP176x host controller:
1. Write 0h to the Buffer Status register (334h).
2. Write FFFF FFFFh to the ATL PTD Skip Map register (154h).
3. Write FFFF FFFFh to the INT PTD Skip Map register (144h).
4. Write FFFF FFFFh to the ISO PTD Skip Map register (134h).
5. Write 0h to the ATL PTD Done Map register (150h).
6. Write 0h to the INT PTD Done Map register (140h).
7. Write 0h to the ISO PTD Done Map register (130h).
8. Write 1h (RESET_ALL) to the SW Reset register (30Ch).
9. Write 2h (RESET_HC) to the SW Reset register (30Ch).
10. Write 2h (HCRESET) to the USBCMD register (20h).
11. Wait until 2h of the USBCMD register is cleared.
 
Register dump after step 11:
USBCMD[0x20]: 0x00080b00
USBSTS[0x24]: 0x00000000
USBINTR[0x28]: 0x00000000
CONFIGFLAG[0x60]: 0x00000000
PORTSC1[0x64]: 0x00002000
HW Mode Control[0x300]: 0x00000100
HC Chip ID[0x304]: 0x00011761
HcBufferStatus[0x334]: 0x00000000
HcInterrupt[0x310]: 0x00000050
HcInterruptEnable[0x314]: 0x00000000 
 
Enabling interrupts (SOF ITL interrupt enable):
12. Write 2h (SOFITLINT) to the HcInterrupt register (310h) to clear any pending
interrupts.
13. Write 2h (SOFITLINT_E) to the HcInterruptEnable register (314h).
If port 1 is used as a host port, perform the next three steps:
14. Perform an OR of the 8000 0000h (ALL_ATX_RESET) on the HW Mode Control
register (300h).
15. Sleep or delay for 10 ms to allow the ATX to reset.
16. CLEAR bit 31 (ALL_ATX_RESET) of the HW Mode Control register to get out of the
ATX reset state.
17. Write 0h to the ATL IRQ Mask AND register (32Ch).
18. Write 0h to the ATL IRQ Mask OR register (320h).
19. Write 0h to the INT IRQ Mask AND register (328h).
20. Write 0h to the INT IRQ Mask OR register (31Ch).
21. Write 0h to the ISO IRQ Mask AND register (324h).
22. Write FFFF FFFFh to the ISO IRQ Mask OR register (318h).
23. Perform an OR of the value 101h (GLOBAL_INTR_EN and DATA_BUS_WIDTH) on
the HW Mode Control register (300h).
Remark: For the 16-bit bus system, the DATA_BUS_WIDTH bit must not be set.
Register dump after step 23:
USBCMD[0x20]: 0x00080b00
USBSTS[0x24]: 0x00000000
USBINTR[0x28]: 0x00000000
CONFIGFLAG[0x60]: 0x00000000
PORTSC1[0x64]: 0x00002000
HW Mode Control[0x300]: 0x00000101
HC Chip ID[0x304]: 0x00011761
HcBufferStatus[0x334]: 0x00000000
HcInterrupt[0x310]: 0x00000050
HcInterruptEnable[0x314]: 0x00000002
Putting the host controller in operational mode:
24. Write 1h (RS) to the USBCMD register (20h).
25. Wait for the RS bit of the USBCMD register to be set by continuously reading the
register until the RS bit is set.
26. Write 1h (CF) to the CONFIGFLAG register (60h).
27. Wait for the CF bit of the CONFIGFLAG register to be set by continuously reading
the register until the CF bit is set.
Register dump after step 27:
USBCMD[0x20]: 0x00000001
USBSTS[0x24]: 0x00000000
USBINTR[0x28]: 0x00000000
CONFIGFLAG[0x60]: 0x00000001
PORTSC1[0x64]: 0x00000000
HW Mode Control[0x300]: 0x00000101
HC Chip ID[0x304]: 0x00011761
HcBufferStatus[0x334]: 0x00000000
HcInterrupt[0x310]: 0x00000050
HcInterruptEnable[0x314]: 0x00000002
PTD register initialization part 2:
28. Write 8000 0000h to the ATL PTD Last PTD register (158h).
29. Write 8000 0000h to the INT PTD Last PTD register (148h).
30. Write 1h to the ISO PTD Last PTD register (138h).
Register dump after step 30:
USBCMD[0x20]: 0x00000001
USBSTS[0x24]: 0x00000000
USBINTR[0x28]: 0x00000000
CONFIGFLAG[0x60]: 0x00000001
PORTSC1[0x64]: 0x00000000
HW Mode Control[0x300]: 0x00000101
HC Chip ID[0x304]: 0x00011761
HcBufferStatus[0x334]: 0x00000000
HcInterrupt[0x310]: 0x00000000
HcInterruptEnable[0x314]: 0x00000002
Remark: HcInterrupt status is now cleared by the interrupt handler, which has become
active.
Powering the root port:
31. Write 1000h (PP) to the PORTSC1 register (64h).
32. Read the PORTSC1 register (64h) and wait until the ECSC bit (2h) is set.
33. Perform an OR of 2h (ECSC) on the PORTSC1 register (64h).
34. Write 1901h to the PORTSC1 register (64h).
35. Wait for 50 ms before clearing the Port Reset (PR) bit.
36. Clear the PR bit (100h) of the PORTSC1 register (64h).
Remark: The hub driver can now enumerate the internal hub.
The HcInterrupt register value becomes zero after step 2 because the ISP176x host
controller driver is running in a Linux environment and there are two execution-contexts
of the code in this environment. The first is the normal context that runs the host
controller initialization while the second is the interrupt context that checks for the
interrupt status once it arrives.
The interrupt context is not shown in preceding steps. It only highlights what you must
get when you are initializing registers. In brief, when the system has an interrupt, the
interrupt context interprets it, clears the Interrupt Status register, and performs any action
that the interrupt has indicated. Therefore, you will see HcInterrupt zero at step 30.

*/

void InitTest()
{
   u32 Value;

   isp1760_write32(HC_HW_MODE_CTRL,0);
   isp1760_write32(HC_HW_MODE_CTRL,0);
#if 0
   LOG("power on register dump\n");
   UsbRegDump();
#endif

// 1. Write 0h to the Buffer Status register (334h).
   isp1760_write32(HC_BUFFER_STATUS_REG,0);

// 2. Write FFFF FFFFh to the ATL PTD Skip Map register (154h).
   isp1760_write32(0x154,0xffffffff);

// 3. Write FFFF FFFFh to the INT PTD Skip Map register (144h).
   isp1760_write32(0x144,0xffffffff);

// 4. Write FFFF FFFFh to the ISO PTD Skip Map register (134h).
   isp1760_write32(0x134,0xffffffff);

// 5. Write 0h to the ATL PTD Done Map register (150h).
   isp1760_write32(0x150,0);

// 6. Write 0h to the INT PTD Done Map register (140h).
   isp1760_write32(0x140,0);

// 7. Write 0h to the ISO PTD Done Map register (130h).
   isp1760_write32(0x130,0);

// 8. Write 1h (RESET_ALL) to the SW Reset register (30Ch).
   isp1760_write32(0x30c,1);

// 9. Write 2h (RESET_HC) to the SW Reset register (30Ch).
   isp1760_write32(0x30c,2);

// 10. Write 2h (HCRESET) to the USBCMD register (20h).
   Value = isp1760_read32(0x20);
   Value |= 2;
   print_1cr("USBCMD\n",Value);
   isp1760_write32(0x20,Value);

// 11. Wait until 2h of the USBCMD register is cleared.
#if 0
   print("Wait for bit 2 of USBCMD\n");

   while((isp1760_read32(0x20) & 2) != 0);
#else
   msleep(100);
#endif

   isp1760_write32(HC_HW_MODE_CTRL,0);
   isp1760_write32(HC_HW_MODE_CTRL,0);

//   print("Step 11 register dump\n");
//   UsbRegDump();
   Value &= ~2;
   isp1760_write32(0x20,Value);

//Enabling interrupts (SOF ITL interrupt enable):
//12. Write 2h (SOFITLINT) to the HcInterrupt register (310h) to clear any pending
// interrupts.

   isp1760_write32(0x310,2);

// 13. Write 2h (SOFITLINT_E) to the HcInterruptEnable register (314h).

   isp1760_write32(0x314,2);

// If port 1 is used as a host port, perform the next three steps:
// 14. Perform an OR of the 8000 0000h (ALL_ATX_RESET) on the HW Mode Control
// register (300h).

   Value = isp1760_read32(0x300);
   Value |= 0x80000000;
   isp1760_write32(0x300,Value);

// 15. Sleep or delay for 10 ms to allow the ATX to reset.
   msleep(50);

// 16. CLEAR bit 31 (ALL_ATX_RESET) of the HW Mode Control register to get out of the
// ATX reset state.
   Value &= ~0x80000000;
   isp1760_write32(0x300,Value);

// 17. Write 0h to the ATL IRQ Mask AND register (32Ch).
   isp1760_write32(0x32c,0);

// 18. Write 0h to the ATL IRQ Mask OR register (320h).
   isp1760_write32(0x320,0);

// 19. Write 0h to the INT IRQ Mask AND register (328h).
   isp1760_write32(0x328,0);

// 20. Write 0h to the INT IRQ Mask OR register (31Ch).
   isp1760_write32(0x31c,0);

// 21. Write 0h to the ISO IRQ Mask AND register (324h).
   isp1760_write32(0x324,0);

// 22. Write FFFF FFFFh to the ISO IRQ Mask OR register (318h).
   isp1760_write32(0x318,0xffffffff);

// 23. Perform an OR of the value 101h (GLOBAL_INTR_EN and DATA_BUS_WIDTH) on
// the HW Mode Control register (300h).

   Value = isp1760_read32(0x300);
   Value |= 1;
   isp1760_write32(0x300,Value);

//   print("Step 23 register dump\n");
//   UsbRegDump();

// Putting the host controller in operational mode:
// 24. Write 1h (RS) to the USBCMD register (20h).
   Value |= 1;
   isp1760_write32(0x20,Value);

// 25. Wait for the RS bit of the USBCMD register to be set by continuously reading the
// register until the RS bit is set.
   print("Waiting for run bit\n");
   while((isp1760_read32(0x20) & 1) == 0);

// 26. Write 1h (CF) to the CONFIGFLAG register (60h).

   isp1760_write32(0x60,1);

// 27. Wait for the CF bit of the CONFIGFLAG register to be set by continuously reading
// the register until the CF bit is set.
   print("Waiting for cf bit\n");
   while((isp1760_read32(0x60) & 1) == 0);

//   print("Register dump after step 27:\n");
//   UsbRegDump();

// PTD register initialization part 2:
// 28. Write 8000 0000h to the ATL PTD Last PTD register (158h).
   isp1760_write32(0x158,0x80000000);

// 29. Write 8000 0000h to the INT PTD Last PTD register (148h).
   isp1760_write32(0x148,0x80000000);

// 30. Write 1h to the ISO PTD Last PTD register (138h).
   isp1760_write32(0x138,0x80000000);
//   print("Register dump after step 30:\n");
//   UsbRegDump();

// Powering the root port:
// 31. Write 1000h (PP) to the PORTSC1 register (64h).
   isp1760_write32(0x64,0x1000);

// 32. Read the PORTSC1 register (64h) and wait until the ECSC bit (2h) is set.
   print("Waiting for ECSC bit\n");
   while((isp1760_read32(0x64) & 2) == 0);

// 33. Perform an OR of 2h (ECSC) on the PORTSC1 register (64h).
   Value = isp1760_read32(0x64);
   Value |= 2;
   isp1760_write32(0x64,Value);

// 34. Write 1901h to the PORTSC1 register (64h).
   isp1760_write32(0x64,0x1901);

// 35. Wait for 50 ms before clearing the Port Reset (PR) bit.
   msleep(100);

//36. Clear the PR bit (100h) of the PORTSC1 register (64h).
   isp1760_write32(0x64,0x1801);

//   print("Register dump after step 36:\n");
//   UsbRegDump();
}

// return 0 on success
int _DoTransfer(u32 *ptd,const char *Func,int Line)
{
   u32 Value;
   int Ret = 1;   // assume the worse
   u32 PtdBuf[8];

   if(gDumpPtd) {
      DumpPtd("Ptd before execution:",ptd);
   }
   mem_writes8(ATL_PTD_OFFSET+4,&ptd[1],28);
   mem_writes8(ATL_PTD_OFFSET,ptd,4);

// Set ATL Skip Map register
   isp1760_write32(HC_ATL_PTD_LASTPTD_REG,0x80000000);
   isp1760_write32(HC_ATL_PTD_SKIPMAP_REG,0);

   isp1760_write32(HC_ATL_IRQ_MASK_OR_REG,1);
   Value = isp1760_read32(HC_INTERRUPT_ENABLE);
   isp1760_write32(HC_INTERRUPT_ENABLE,Value | HC_ATL_INT | HC_SOT_INT);
   Value = isp1760_read32(HC_BUFFER_STATUS_REG);
   isp1760_write32(HC_BUFFER_STATUS_REG,Value | ATL_BUF_FILL);

// Poll interrupt register for up to 2 seconds...
   {
#if 1
      for( ; ; ) {
         if((Value = PollUsbInt()) & HC_ATL_INT) {
         // Read the Done bit map to clear the bits
            u32 Done = isp1760_read32(HC_ATL_PTD_DONEMAP_REG);
            if(Done == 0) {
               LOG("Int reg: 0x%x, done: 0x%x!\n",Value,Done);
            }
            Done = isp1760_read32(HC_ATL_PTD_DONEMAP_REG);
            if(Done != 0) {
               LOG("Done wasn't cleared by read back! 0x%x\n",Done);
            }
         // Read back the Ptd to check status
            mem_reads8(ATL_PTD_OFFSET,PtdBuf,sizeof(PtdBuf));
            if((PtdBuf[3] >> 31) & 0x1) {
               DumpPtd("PTD still active!",PtdBuf);
               continue;
            }
            Ret = 0;
            break;
         }
      }
#else 
      int i = 0;
      int Toggle = 0;
      while(i < 2000) {
         Value = isp1760_read32(HC_INTERRUPT_REG) & ~INT_REG_RESERVED_BITS;
         if(Value != 0) {
            i++;
            if(Value == HC_SOT_INT) {
               i++;
#if 0
               uint32_t Leds = REG_RD(GPIO_READ_ADDR);
               if(Toggle) {
                  Toggle = 0;
                  Leds |= GPIO_BIT_LED_RED;
               }
               else {
                  Toggle = 1;
                  Leds &= ~GPIO_BIT_LED_RED;
               }
               REG_WR(GPIO_WRITE_ADDR,Leds);
#endif
            }
            else {
#if 1
               if(Value & (1 << 8)) {
                  i = 0x7fffffff;
                  Ret = 0;
               }
#else
               print("ints: ");
               print_int(Value,3);
               if(Value & HC_SOT_INT) {
                  i++;
                  print(" SOT");
               }
               if(Value & HC_EOT_INT) {
                  print(" EOT");
               }
               if(Value & (1 << 5)) {
                  print(" SUSP");
               }
               if(Value & (1 << 6)) {
                  print(" CLKREADY");
               }
               if(Value & (1 << 7)) {
                  print(" INT");
               }
               if(Value & (1 << 8)) {
                  print(" ATL");
                  i = 0x7fffffff;
                  Ret = 0;
               }
               if(Value & (1 << 9)) {
                  print(" ISO");
               }
               print("\n");
#endif
            }
         }
         isp1760_write32(HC_INTERRUPT_REG,Value);
      }
      if(i >= 2000) {
         LOG("%s#%d: Timeout!\n",Func,Line);
      }
#endif
   }

   isp1760_write32(HC_ATL_PTD_SKIPMAP_REG,0xffffffff);
   Value = isp1760_read32(HC_BUFFER_STATUS_REG);
   isp1760_write32(HC_BUFFER_STATUS_REG,Value & ~ATL_BUF_FILL);

   if(Ret == 0) {
      if(((PtdBuf[3] >> 28) & 0x1) || ((PtdBuf[3] >> 30) & 0x1)) {
         Ret = 1;
      }
   }

   if(Ret != 0) {
      SetDebugLED(true);
      LOG("%s#%d: ",Func,Line);
      LOG("Transfer failed!\n");
      SetDebugLED(false);
      if(!gDumpPtd) {
         DumpPtd("Ptd before execution:",ptd);
      }
      DumpPtd("\nPtd after execution:",PtdBuf);
      Dump1760Mem();
      UsbRegDump();
   }
   else if(gDumpPtd) {
      DumpPtd("Ptd after execution",PtdBuf);
   }

// Copy the PTD read from 1760 back into the orginal buffer
   memcpy(ptd,PtdBuf,sizeof(PtdBuf));

   return Ret;
}

void InitPtd(u32 *Ptd,uint8_t Adr,uint8_t EndPoint,uint8_t Pid,u16 PayLoadAdr,int Len)
{
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   u32 maxpacket;
   u32 multi;
   u32 rl = RL_COUNTER;
   u32 nak = NAK_COUNTER;
   uint8_t PipeType = pDev->PipeType[EndPoint];

   memset(Ptd,0,8 * sizeof(u32));

   /* according to 3.6.2, max packet len can not be > 0x400 */
   maxpacket = pDev->MaxPacketSize;
   multi =  1 + ((maxpacket >> 11) & 0x3);
   maxpacket &= 0x7ff;

   /* DW0 */
   Ptd[0] = DW0_VALID_BIT;
   Ptd[0] |= TO_DW0_LENGTH(Len);
   Ptd[0] |= TO_DW0_MAXPACKET(maxpacket);
   Ptd[0] |= TO_DW0_ENDPOINT(EndPoint);

   /* DW1 */
   Ptd[1] = EndPoint >> 1;
   Ptd[1] |= TO_DW1_DEVICE_ADDR(Adr);
   Ptd[1] |= TO_DW1_PID_TOKEN(Pid);

   if(PipeType == PTYPE_BULK) {
      Ptd[1] |= DW1_TRANS_BULK;
   }
   if(PipeType == PTYPE_INT) {
      Ptd[1] |= DW1_TRANS_INT;
   }

   if(pDev->UsbSpeed != USB_SPEED_HIGH) {
      /* split transaction */
      Ptd[1] |= DW1_TRANS_SPLIT;
      if(pDev->UsbSpeed == USB_SPEED_LOW) {
         Ptd[1] |= DW1_SE_USB_LOSPEED;
      }
      Ptd[1] |= TO_DW1_PORT_NUM(pDev->TTPort);
      Ptd[1] |= TO_DW1_HUB_NUM(pDev->HubDevnum);

      /* SE bit for Split INT transfers */
      // sh: not necessary?  Already set above for all pipe types...
      if(PipeType == PTYPE_INT && pDev->UsbSpeed == USB_SPEED_LOW) {
         Ptd[1] |= DW1_SE_USB_LOSPEED;
      }
#if 0
      if(Pid == IN_PID) {
      // end split ???
//         Ptd[3] |= DW3_SC_BIT;
         Ptd[1] |= DW1_TRANS_BULK;
         Ptd[1] &= ~DW1_TRANS_SPLIT;
      }
#endif

      rl = 0;
      nak = 0;
   }
   else {
   // High speed
      Ptd[0] |= TO_DW0_MULTI(multi);
      if(PipeType == PTYPE_CONTROL || PipeType == PTYPE_BULK) {
         Ptd[3] |= TO_DW3_PING(pDev->Ping);
      }
   }

   /* DW2 */
   Ptd[2] |= TO_DW2_DATA_START_ADDR(base_to_chip(PayLoadAdr));
   Ptd[2] |= TO_DW2_RL(rl);

   /* DW3 */
   Ptd[3] |= TO_DW3_NAKCOUNT(nak);
   Ptd[3] |= TO_DW3_DATA_TOGGLE(pDev->Toggle);
   if(PipeType == PTYPE_CONTROL) {
      if(Pid == SETUP_PID) {
         Ptd[3] &= ~TO_DW3_DATA_TOGGLE(1);
      }
      else if(pDev->LastPacket) {
         Ptd[3] |= TO_DW3_DATA_TOGGLE(1);
      }
   }

   Ptd[3] |= DW3_ACTIVE_BIT;
   /* Cerr */
   Ptd[3] |= TO_DW3_CERR(ERR_COUNTER);
}

// return number of bytes read or < 0 for error
int SetupTransaction(uint8_t Adr,SetupPkt *p,void *pResponse,int ResponseLen)
{
   u32 Ptd[8];
   u16 CmdPayloadAdr = SETUP_CMD_BUF;
   u16 RespPayloadAdr = SETUP_RESP_BUF;
   uint8_t Pid;
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   int Ret = 0;   // assume the best

   if(p->wLength != ResponseLen) {
      LOG("p->wLength != ResponseLen!\n");
   }

   pDev->PipeType[0] = PTYPE_CONTROL;
// Temp kludge
   pDev->LastPacket = true;

   if(gDumpPtd) {
      LOG("Setup phase\n");
   }
   do {
   // copy setup packet into payload memory
      mem_writes8(CmdPayloadAdr,(u32 *)p,sizeof(SetupPkt));
      InitPtd(Ptd,Adr,0,SETUP_PID,CmdPayloadAdr,sizeof(SetupPkt));

      if(gDumpPtd) {
#if 0
         LOG("Waiting for button press\n");
         while(!button_pressed());
#endif
      }
      if((Ret = DoTransfer(Ptd)) != 0) {
         Ret = -1;
         break;
      }

   // Update the toggle an ping bits
      pDev->Toggle = FROM_DW3_DATA_TOGGLE(Ptd[3]);
      pDev->Ping = FROM_DW3_PING(Ptd[3]);

      if(p->wLength > 0) {
      // There is a data phase, read the data
         if(gDumpPtd) {
            LOG("Data phase\n");
         }

         if(p->ReqType_u.bmRequestType & USB_SETUP_DEVICE_TO_HOST) {
            Pid = IN_PID;
         }
         else {
            Pid = OUT_PID;
            mem_writes8(RespPayloadAdr,(u32 *) pResponse,ResponseLen);
         }
         InitPtd(&Ptd[0],Adr,0,Pid,RespPayloadAdr,ResponseLen);
         Ret = DoTransfer(Ptd);
         if(Ret != 0) {
            Ret = -1;
            break;
         }
         if(p->ReqType_u.bmRequestType & USB_SETUP_DEVICE_TO_HOST) {
            Ret = (Ptd[3] & 0x7fff);
            mem_reads8(RespPayloadAdr,(u32 *) pResponse,ResponseLen);
         }
      }

   /* The direction of data and status transfer depends on whether the host 
      is sending data to the device or the device is sending data to the host. 
      The Status stage transfer is always in the opposite direction of the Data 
      stage. If there is no Data stage, the Status stage is from the device to 
      the host.

      Control requests may need a terminating data "status" ack;
      bulk ones may need a terminating short packet (zero length). */
      if(gDumpPtd) {
         LOG("Status phase\n");
      }
      if(p->wLength > 0) {
      // There was a data phase
         if(p->ReqType_u.bmRequestType & USB_SETUP_DEVICE_TO_HOST) {
            Pid = OUT_PID;
         }
         else {
            Pid = IN_PID;
         }
      }
      else {
      // for zero length DATA stages, STATUS is always IN
         Pid = IN_PID;
      }
      InitPtd(&Ptd[0],Adr,0,Pid,RespPayloadAdr,0);
      if(DoTransfer(&Ptd[0]) != 0) {
         Ret = -1;
         break;
      }
   } while(false);

   return Ret;
}

u32 base_to_chip(u32 base)
{
   return ((base - 0x400) >> 3);
}

void SetDebugLED(bool bOn)
{
   uint32_t Leds = REG_RD(GPIO_READ_ADDR);
   if(bOn) {
      Leds &= ~GPIO_BIT_LED_RED;
   }
   else {
      Leds |= GPIO_BIT_LED_RED;
   }
   REG_WR(GPIO_WRITE_ADDR,Leds);
}

// Return 0 on success
int OpenControlInPipe(uint8_t Adr,uint8_t Endpoint,ControlCB *Funct,uint8_t *Buf,size_t Len)
{
   u32 Ptd[8];
   uint16_t DevBuf = INT_IN_BUF_ADR(Adr);
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   u32 PtdAdr = INT_PTD_OFFSET + (Adr * 8 * sizeof(uint32_t));
   u32 PtdBit = 1 << Adr;
   u32 Bits;
   int Ret = 1;   // Assume the worse

   do {
      Endpoint &= 0x7f;
      LOG("Adr: %d, Endpoint: %d, Len: %d, Buf: %p\n",Adr,Endpoint,Len,Buf);
      if(Endpoint >= MAX_ENDPOINTS) {
         break;
      }
      pDev->ControlInCB = Funct;
      pDev->PipeType[Endpoint] = PTYPE_INT;
      pDev->UCtrlInBuf = Buf;
      pDev->UCtrlInBufLen = Len;
      pDev->Toggle = 0;
      InitPtd(&Ptd[0],Adr,Endpoint,IN_PID,DevBuf,Len);
      TransformPtd2Int(&Ptd[0],Adr,Endpoint);
   //   Ptd[5] = 0xff; /* Execute Complete Split on any uFrame */
      Ptd[5] = 0x80;

      // LOG("PtdAdr 0x%x\n",PtdAdr);
      // DumpPtd("Ptd before execution:",Ptd);

      mem_writes8(PtdAdr+4,&Ptd[1],28);
      mem_writes8(PtdAdr,Ptd,4);


   // Enable completion interrupts for our PTD
      Bits = isp1760_read32(HC_INT_IRQ_MASK_OR_REG);
      Bits |= PtdBit;
      isp1760_write32(HC_INT_IRQ_MASK_OR_REG,Bits);
      LOG("Set HC_INT_IRQ_MASK_OR_REG to 0x%x\n",Bits);

      Bits = isp1760_read32(HC_INTERRUPT_ENABLE);
      if((Bits & HC_INTL_INT) == 0) {
         Bits |= HC_INTL_INT;
         isp1760_write32(HC_INTERRUPT_ENABLE,Bits);
         LOG("Set HC_INTERRUPT_ENABLE to 0x%x\n",Bits);
      }

      Bits = isp1760_read32(HC_BUFFER_STATUS_REG);
      if((Bits & INT_BUF_FILL) == 0) { 
         Bits |= INT_BUF_FILL;
         isp1760_write32(HC_BUFFER_STATUS_REG,Bits);
         LOG("Set HC_BUFFER_STATUS_REG to 0x%x\n",Bits);
      }
   // Set ATL Skip Map register
      isp1760_write32(HC_INT_PTD_LASTPTD_REG,0x80000000);
      Bits = isp1760_read32(HC_INT_PTD_SKIPMAP_REG);
   // Enable our PTD
      Bits &= ~PtdBit;
      isp1760_write32(HC_INT_PTD_SKIPMAP_REG,Bits);
      LOG("Set HC_INT_PTD_SKIPMAP_REG to 0x%x\n",Bits);
      Ret = 0;
   } while(false);

   return Ret;
}

u32 PollUsbInt()
{
   u32 Value;
   u32 Done;
   static bool bDoIt = true;

   Value = isp1760_read32(HC_INTERRUPT_REG);
   isp1760_write32(HC_INTERRUPT_REG,Value);
   if(Value & HC_ATL_INT) {
   // Read the Done bit map to clear the bits
//      Done = isp1760_read32(HC_ATL_PTD_DONEMAP_REG);
//    LOG("Int reg: 0x%x, ATL done: 0x%x!\n",Value,Done);
   }

   if(Value & HC_INTL_INT) {
   // Read the Done bit map to clear the bits
      Done = isp1760_read32(HC_INT_PTD_DONEMAP_REG);
//    LOG("Int reg: 0x%x, Int done: 0x%x!\n",Value,Done);
      if(Done != 0) {
         int8_t Adr;
         u32 DoneBit = 1;
         for(Adr = 0; Adr < MAX_USB_DEVICES; Adr++) {
            if(Done & DoneBit) {
            // Got one!
               u32 PtdBuf[8];
               u32 PtdAdr = INT_PTD_ADR(Adr);
               size_t Len;
               size_t UBufLen = gUsbDevice[Adr].UCtrlInBufLen;
               u32 *UBuf = gUsbDevice[Adr].UCtrlInBuf;

               mem_reads8(PtdAdr,PtdBuf,sizeof(PtdBuf));
               if(bDoIt) {
               // Restart the PTD
                  if((PtdBuf[3] >> 28) & 0x1) {
                  // error
                  }
                  else {
                     Len = FROM_DW3_SCS_NRBYTESTRANSFERRED(PtdBuf[3]);
                     if(Len > 0 && Len <= UBufLen) {
                     // copy the data from device memory to RAM
                        mem_reads8(INT_IN_BUF_ADR(Adr),UBuf,UBufLen);
                        gUsbDevice[Adr].ControlInCB(Adr);
                     }
                     else {
                        LOG("Error: Len %d, data not read\n",Len);
                     }
                     //DumpPtd("After interrupt",PtdBuf);
                  }

                  PtdBuf[0] |= DW0_VALID_BIT;
//                  PtdBuf[3] &= ~(0x0fff | DW3_SC_BIT);
                  PtdBuf[3] &= DW3_DATA_TOGGLE_BIT;
                  PtdBuf[3] |= DW3_ACTIVE_BIT | TO_DW3_CERR(ERR_COUNTER);
//                LOG_RAW(" %x\n",PtdBuf[3]);
//                  TransformPtd2Int(PtdBuf,Adr,Endpoint);
               // when we set uSCS blindly to 0xff we might complete
               // another transfer immediately.
                  PtdBuf[5] = 0xff; /* Execute Complete Split on any uFrame */
                  PtdBuf[4] = 0x1;
//                  PtdBuf[5] = 0x80;

//                LOG("Adr: %d, Endpoint: %d\n",Adr,Endpoint);
//                  DumpPtd("After reset",PtdBuf);
                  mem_writes8(PtdAdr+4,&PtdBuf[1],28);
               // Enable our PTD
                  mem_writes8(PtdAdr,PtdBuf,4);
               }
            }
            DoneBit <<= 1;
         }
      }
   }
   return Value;
}

void TransformPtd2Int(u32 *Ptd,uint8_t Adr,uint8_t EndPoint)
{
   u32 usof;
   u32 period;
   PanoUsbDevice *pDev = &gUsbDevice[Adr];
   uint8_t Interval = pDev->Interval[EndPoint];

   /*
    * Most of this is guessing. ISP1761 datasheet is quite unclear, and
    * the algorithm from the original Philips driver code, which was
    * pretty much used in this driver before as well, is quite horrendous
    * and, i believe, incorrect. The code below follows the datasheet and
    * USB2.0 spec as far as I can tell, and plug/unplug seems to be much
    * more reliable this way (fingers crossed...).
    */

   if(pDev->UsbSpeed == USB_SPEED_HIGH) {
      /* urb->interval is in units of microframes (1/8 ms) */
      period = Interval >> 3;

      if(Interval > 4) {
         usof = 0x01; /* One bit set => interval 1 ms * uFrame-match */
      }
      else if(Interval > 2) {
         usof = 0x22; /* Two bits set => interval 1/2 ms */
      }
      else if(Interval > 1) {
         usof = 0x55; /* Four bits set => interval 1/4 ms */
      }
      else {
         usof = 0xff; /* All bits set => interval 1/8 ms */
      }
   }
   else {
   /* According to the ISP1760 spec sheet pg 81:
      Bits 7 to 3 is the polling rate in milliseconds. Polling rate is
      defined as 2(b - 1) mSOF; where b = 4 to 16. When b is 4, executed
      every millisecond.
    
      The Linux driver seems to be way off here.  My test gamepad had an
      interval of 10 which resulted in uFrame of zero the Linux code.
    
      Table 79:
      B  Rate  uFrame[7:3]
      5  2ms   0 0001
      6  4ms   0 0010 or 0 0011
      7  8ms   0 0100 or 0 0111
      8  16ms  0 1000 or 0 1111
      9  32ms  1 0000 or 1 1111
   */
      if(Interval < 4) {
         period = 0x08;
      }
      else if(Interval < 8) {
         period = 0x10;
      }
      else if(Interval < 16) {
         period = 0x20;
      }
      else if(Interval < 32) {
         period = 0x40;
      }
      else {
         period = 0x80;
      }
//      usof = 0x0f; /* Execute Start Split on any of the four first uFrames */
      usof = 1;
#if 0 // sh: don't do this here !
      /*
       * First 8 bits in dw5 is uSCS and "specifies which uSOF the
       * complete split needs to be sent. Valid only for IN." Also,
       * "All bits can be set to one for every transfer." (p 82,
       * ISP1761 data sheet.) 0x1c is from Philips driver. Where did
       * that number come from? 0xff seems to work fine...
       */
      /* ptd->dw5 = 0x1c; */
      Ptd[5] = 0xff; /* Execute Complete Split on any uFrame */
#endif
   }

   Ptd[2] |= period;
   Ptd[4] = usof;
}

void print_1cr(const char *label,int value)
{
   LOG_RAW("%s: 0x%x\n",label,value);
}

// Device is currently @ address 0
void ConfigureDev(uint8_t Adr)
{
   uint8_t DescBuf[350];
   int Len;
   uint8_t *pBuf = DescBuf;
   int BytesLeft = sizeof(DescBuf) - 1;
   USB_DEVICE_DESCRIPTOR *pDevDesc = (USB_DEVICE_DESCRIPTOR *) DescBuf;
   UsbDriverIf *pDriverIf = gDriverHead;
   int Err;

   do {
      Len = GetDesc(bmREQ_GET_DESCR,USB_DESCRIPTOR_DEVICE,0,pBuf,BytesLeft);
      if(Len < 0) {
         break;
      }
      if(Len != DescBuf[0]) {
         LOG("GetDesc returned %d, descriptor says %d\n",Len,DescBuf[0]);
         break;
      }
      pBuf += Len;
      BytesLeft -= Len;
      DumpDeviceDesc(0,pDevDesc);
      ELOG("Set adr to %d\n",Adr);
      if(SetUsbAddress(Adr) < 0) {
         break;
      }
      LOG("Get configuration descriptor for device @ %d\n",Adr);
      Len = GetDesc(bmREQ_GET_DESCR,USB_DESCRIPTOR_CONFIGURATION,Adr,pBuf,BytesLeft);
      if(Len < 0) {
         break;
      }
      LOG("Get configuration descriptor len %d, requested: %d\n",Len,BytesLeft);
      DumpDeviceDesc(Adr,pDevDesc);
      DumpConfigDesc(Adr,(USB_CONFIGURATION_DESCRIPTOR *) pBuf);
      LOG("Calling GetConfigDesc\n");
      GetConfigDesc(Adr);

      pBuf += Len;
      BytesLeft -= Len;
      *pBuf = 0;  // terminate descriptor chain
      if(pDevDesc->bNumConfigurations == 1) {
      // There's only one configuration, select it so the driver doesn't need to
         LOG("Set configuration to 1\n");
         SetConfiguration(Adr,1);
      }
      while(pDriverIf != NULL) {
         Err = pDriverIf->ClaimDevice(Adr,DescBuf,sizeof(DescBuf));
         PRINTF("Device %sclaimed by %s driver\n",Err == 0 ? "" : "not ",
                pDriverIf->DriverName);

         if(Err == 0) {
         // Found a driver!
            break;
         }
         pDriverIf = pDriverIf->pNext;
      }
   } while(false);
}

uint8_t *FindDesc(uint8_t Type,uint8_t *pBuf)
{
   uint8_t *Ret = NULL;

   while(pBuf[0] != 0) {
      if(pBuf[1] == Type) {
         Ret = pBuf;
         break;
      }
      pBuf += pBuf[0];
   }

   if(Ret == NULL) {
      LOG("Failed to find %d\n",Type);
   }
   else {
      LOG("Found type %d\n",Type);
   }

   return Ret;
}

void UsbRegisterDriver(UsbDriverIf *pDriverIf)
{
   if(gDriverHead == NULL) {
      gDriverHead = pDriverIf;
   }
   else {
      gDriverHead->pNext = pDriverIf;
   }
}

#ifdef USB_VERBOSE_PRINT
void DumpPortStatus(uint16_t Port,uint32_t Status)
{
   const char *Sep = "";
   const struct {
      const char *Desc;
      uint8_t Bit;
   } GCC_PACKED Bits[] = {
      {"conn",0},
      {"enabled",1},
      {"suspend",2},
      {"oc",3},
      {"rst",4},
      {"pwr",8},
      {"lo_spd",9},
      {"hi_spd",10},
      {"tst",11},
      {"ind",12},
      {"conn_ch",16},
      {"en_ch",17},
      {"suspend_ch",18},
      {"oc_ch",19},
      {"rst_ch",20},
      {NULL}
   };
   int i;

   LOG_RAW("Port %d status 0x%x: ",Port,Status);
   if(Status != 0) {
      LOG_RAW(" (");
      for(i = 0; Bits[i].Desc != NULL; i++) {
         if(Status & (1 << Bits[i].Bit) ) {
            LOG_RAW("%s%s",Sep,Bits[i].Desc);
            Sep = ", ";
         }
      }
      LOG_RAW(")");
   }
   LOG_RAW("\n");
}

void UsbRegDump()
{
   const uint16_t Regs[] = {
      0x0,0x8,
      0x20,0x2c,
      0x60,0x64,
      0x130,0x138,
      0x140,0x148,
      0x150,0x158,
      0x300,0x32c,
      0x334,0x344,
      0x354,0x354,
      0x374,0x374,
      1        // end of table
   };
   int i;
   int j;
   u32 Value;

   LOG_RAW("isp1760 regs:\n");
   for(i = 0; Regs[i] != 1; i += 2) {
      for(j = Regs[i]; j <= Regs[i+1]; j += 4) {
         Value = isp1760_read32(j);
         LOG_RAW("%x: 0x%x\n",j,Value);
      }
   }
}

void DumpInterfaceDesc(uint8_t Adr,USB_INTERFACE_DESCRIPTOR *p)
{
   LOG_RAW("\n  Interface descriptor:\n");
   print_1cr("  bLength",p->bLength);
   print_1cr("  bDescriptorType",p->bDescriptorType);
   print_1cr("  bInterfaceNumber",p->bInterfaceNumber);
   print_1cr("  bAlternateSetting",p->bAlternateSetting);
   print_1cr("  bNumEndpoints",p->bNumEndpoints);
   DumpClass("  bInterfaceClass",p->bInterfaceClass);
   print_1cr("  bInterfaceSubClass",p->bInterfaceSubClass);
   print_1cr("  bInterfaceProtocol",p->bInterfaceProtocol);
   DumpStringDesc("  iInterface",p->iInterface,Adr);
}

void DumpEndpointDesc(USB_ENDPOINT_DESCRIPTOR *p)
{
   const char *TTLookup[] = {
      "Control",
      "Isochronous",
      "Bulk",
      "Interrupt"
   };
   LOG_RAW("\n  Endpoint descriptor:\n");
   print_1cr("  bLength",p->bLength);
   print_1cr("  bDescriptorType",p->bDescriptorType);
   print_1cr("  bEndpointAddress",p->bEndpointAddress);
   LOG_RAW("  bmAttributes: 0x%x - %s\n",p->bmAttributes,
           TTLookup[p->bmAttributes & 0x3]);
   print_1cr("  wMaxPacketSize",p->wMaxPacketSize);
   print_1cr("  bInterval",p->bInterval);
}

void DumpClass(const char *Msg,uint8_t bClass)
{
   const char *ClassNames[] = {
      "AUDIO",             // 0x01, Audio
      "COM_AND_CDC_CTRL",  // 0x02, Communications and CDC Control
      "HID",               // 0x03, HID
      NULL,                // 0x04
      "PHYSICAL",          // 0x05, Physical
      "IMAGE",             // 0x06, Image
      "PRINTER",           // 0x07, Printer
      "MASS_STORAGE",      // 0x08, Mass Storage
      "HUB",               // 0x09, Hub
      "CDC_DATA",          // 0x0a, CDC-Data
      "SMART_CARD",        // 0x0b, Smart-Card
      NULL,
      "CONTENT_SECURITY",  // 0x0d, Content Security
      "VIDEO",             // 0x0e, Video
      "PERSONAL_HEALTH",   // 0x0f, Personal Healthcare
   };

   if(bClass > 0 && bClass <= 0xf && ClassNames[bClass-1] != NULL) {
      LOG_RAW("%s: %s (%d)\n",Msg,ClassNames[bClass-1],bClass);
   }
   else {
      LOG_RAW("%s: %d\n",Msg,bClass);
   }
}

void DumpPtd(const char *msg,u32 *p)
{
   int i;
   const char *TokenTbl[] = {
      "OUT",
      "IN",
      "SETUP",
      "PING"
   };
   const char *EpTypeTbl[] = {
      "control",
      "???",
      "bulk",
      "interrupt"
   };
   const char *SeTypeTbl[] = {
      "full-speed",
      "???",
      "low-speed",
      "???"
   };
   int EndPt;

   LOG_RAW("%s\n",msg);
   EndPt = ((p[0] >> 31) & 1) + ((p[1] & 07) << 1);

   print_1cr("V",p[0] & 1);
   if((p[3] >> 29) & 0x1) {
      print("Babble!\n");
   }
   if((p[3] >> 30) & 0x1) {
      print("Halt!\n");
   }

   if((p[3] >> 28) & 0x1) {
      print("Error!\n");
   }
   print_1cr("A",(p[3] >> 31) & 0x1);
   print_1cr("BytesTodo",(p[0] >> 3) & 0x7fff);
   print_1cr("BytesDone",p[3] & 0x7fff);

   print_1cr("NakCnt",(p[3] >> 19) & 0xf);
   print_1cr("RL",(p[2] >> 25) & 0xf);

   print_1cr("MaxPak",(p[0] >> 18) & 0x7ff);
   print_1cr("Multp",(p[0] >> 29) & 0x3);
   print_1cr("EndPt",EndPt);
   print_1cr("DevAdr",(p[1] >> 3) & 0x7f);

   LOG_RAW("Token: %s\n",TokenTbl[(p[1] >> 10) & 0x3]);
   LOG_RAW("EpType: %s\n",EpTypeTbl[(p[1] >> 12) & 0x3]);
   print_1cr("DT",(p[3] >> 25) & 0x1);

   if((p[1] >> 14) & 0x1) {
      if(p[3] & (1 << 27)) {
         LOG_RAW("End Split\n");
      }
      else {
         LOG_RAW("Start Split\n");
      }
      LOG_RAW("  SE: %s\n",SeTypeTbl[(p[1] >> 16) & 0x3]);
      print_1cr("  Port",(p[1] >> 18) & 0x7f);
      print_1cr("  HubAdr",(p[1] >> 25) & 0x7f);
   }
   print_1cr("Start Adr",(((p[2] >> 8) & 0xffff) << 3) + 0x400);
   print_1cr("Cerr",(p[3] >> 23) & 0x3);
   print_1cr("Ping",(p[3] >> 26) & 0x1);

   print_1cr("J",(p[4] >> 5) & 0x1);

   if(((p[1] >> 12) & 0x3) == 3) {
   // Interrupt PTD
      print_1cr("uSA",p[4] & 0xf);
      print_1cr("uFrame",p[2] & 0xff);
      print_1cr("uSCS",p[5] & 0xff);
   }
   else {
      print_1cr("NextPTD",p[4] & 0x1f);
   }

   for(i = 0; i < 8; i++) {
      LOG_RAW("DW%d: 0x%08x\n",i,p[i]);
   }
}

void Dump1760Mem()
{
   int i;
   u32 Value;

   LOG_RAW("1760 Memdump:\n");

   isp1760_write32(HC_MEMORY_REG,0x400);
   for(i = 0; i < 16128; i++) {
      Value = isp1760_read32(0x400);
      if(Value != 0) {
         LOG_RAW("0x%x => 0x%x\n",0x400 + (i * 4),Value);
      }
   }
}

void ReadAndDumpIntPtd(uint8_t Adr)
{
   u32 PtdBuf[8];
   u32 PtdAdr = INT_PTD_OFFSET + (Adr * 8 * sizeof(uint32_t));

   LOG("Reading PTD for Adr %d from 0x%x\n",Adr,PtdAdr);
   mem_reads8(PtdAdr,PtdBuf,sizeof(PtdBuf));
   DumpPtd("Int PTD",PtdBuf);
}

// return number of bytes read or < 0 for error
int GetStringDesc(uint8_t Adr,uint16_t LangId,uint8_t Index,uint8_t *Buf,size_t MaxLen)
{
   SetupPkt Pkt;

   /* fill in setup packet */
   Pkt.ReqType_u.bmRequestType = bmREQ_GET_DESCR;
   Pkt.bRequest = USB_REQUEST_GET_DESCRIPTOR;
   Pkt.wVal_u.wValueLo = Index;
   Pkt.wVal_u.wValueHi = USB_DESCRIPTOR_STRING;
   Pkt.wIndex = LangId;
   Pkt.wLength = MaxLen;

   return SetupTransaction(Adr,&Pkt,Buf,MaxLen);
}

void DumpStringDesc(const char *Label,uint8_t Index,uint8_t Adr)
{
   if(Index == 0) {
      print_1cr(Label,Index);
   }
   else do {
      uint8_t Buf[64];
      int i;
      int Len;

      PanoUsbDevice *pDev = &gUsbDevice[Adr];

      if(pDev->LangID == 0) {
      // Get LangID
         if((Len = GetStringDesc(Adr,0,0,Buf,sizeof(Buf))) < 4) {
            break;
         }
         pDev->LangID = (Buf[3] << 8) + Buf[2];
      }

      if((Len = GetStringDesc(Adr,pDev->LangID,Index,Buf,sizeof(Buf))) < 0) {
         break;
      }
      LOG_RAW("%s: ",Label);
      for(i = 2; i < Len; i += 2) {
         LOG_RAW("%c",Buf[i]);
      }
      if(Buf[0] > sizeof(Buf)) {
         LOG_RAW(" (truncated %d > %d)",Buf[0],sizeof(Buf));
      }
      LOG_RAW("\n");
   } while(false);
}

void DumpConfigDesc(uint8_t Adr,USB_CONFIGURATION_DESCRIPTOR *pConfigDesc)
{
   print_1cr("  bLength",pConfigDesc->bLength);
   print_1cr("  bDescriptorType",pConfigDesc->bDescriptorType);
   print_1cr("  wTotalLength",pConfigDesc->wTotalLength);
   print_1cr("  bNumInterfaces",pConfigDesc->bNumInterfaces);
   print_1cr("  bConfigurationValue",pConfigDesc->bConfigurationValue);
   DumpStringDesc("  iConfiguration",pConfigDesc->iConfiguration,Adr);
   print_1cr("  bmAttributes",pConfigDesc->bmAttributes);
   print_1cr("  bMaxPower",pConfigDesc->bMaxPower);
}

void DumpDeviceDesc(uint8_t Adr,USB_DEVICE_DESCRIPTOR *pDevDesc)
{
   print_1cr("  bLength",pDevDesc->bLength);
   print_1cr("  bDescriptorType",pDevDesc->bDescriptorType);
   print_1cr("  bcdUSB",pDevDesc->bcdUSB);
   DumpClass("  bDeviceClass",pDevDesc->bDeviceClass);
   print_1cr("  bDeviceSubClass",pDevDesc->bDeviceSubClass);
   print_1cr("  bDeviceProtocol",pDevDesc->bDeviceProtocol);
   print_1cr("  bMaxPacketSize0",pDevDesc->bMaxPacketSize0);
   print_1cr("  idVendor",pDevDesc->idVendor);
   print_1cr("  idProduct",pDevDesc->idProduct);
   print_1cr("  bcdDevice",pDevDesc->bcdDevice);
   DumpStringDesc("  iManufacturer",pDevDesc->iManufacturer,Adr);
   DumpStringDesc("  iProduct",pDevDesc->iProduct,Adr);
   DumpStringDesc("  iSerialNumber",pDevDesc->iSerialNumber,Adr);
   print_1cr("  bNumConfigurations",pDevDesc->bNumConfigurations);
}

void DumpHubDesc(uint8_t Adr,struct HubDescriptor *pHubDesc)
{
   print_1cr("  Length",pHubDesc->bDescLength);
   print_1cr("  bDescriptorType",pHubDesc->bDescriptorType);
   print_1cr("  bNbrPorts",pHubDesc->bNbrPorts);
   print_1cr("  wHubCharacteristics",pHubDesc->wHubCharacteristics);
   print_1cr("  bPwrOn2PwrGood",pHubDesc->bPwrOn2PwrGood);
   print_1cr("  bHubContrCurrent",pHubDesc->bHubContrCurrent);
}

void DumpHidDesc(USB_HID_DESCRIPTOR *pDesc)
{
   LOG_RAW("\n  HID descriptor:\n");
   print_1cr("  Length",pDesc->bLength);
   print_1cr("  bDescriptorType",pDesc->bDescriptorType);
   print_1cr("  bcdHID",pDesc->bcdHID);
   print_1cr("  bCountryCode",pDesc->bCountryCode);
   print_1cr("  bDescrType",pDesc->bDescrType);
   print_1cr("  wDescriptorLength",pDesc->wDescriptorLength);
}

#endif


