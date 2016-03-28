
#ifndef __HUAWEI_USB_H__
#define __HUAWEI_USB_H__

/***** VENDOR and COUNTRY *****/
#define COUNTRY_US         "us"
#define COUNTRY_JAPAN      "jp"
#define VENDOR_TMOBILE     "t-mobile"
#define VENDOR_EMOBILE     "emobile"
#define VENDOR_TRACFONE    "tracfone"
#define VENDOR_CC          "consumercellular"
#define VENDOR_SOFTBANK    "softbank"
#define VENDOR_SPRINT    "sprint"

#define USB_DEFAULT_SN     "0123456789ABCDEF"
#define USB_SERIAL_LEN     32
#define VENDOR_NAME_LEN    32
#define COUNTRY_NAME_LEN   32

/* support 3 luns at most, 1 lun for cdrom and 2 luns for udisk */
#define USB_MAX_LUNS       3

#define SC_REWIND          0x01
#define SC_REWIND_11       0x11

#define ORI_INDEX         0

/* READ_TOC command structure */
typedef struct _usbsdms_read_toc_cmd_type
{
   u8  op_code;  
   u8  msf;             /* bit1 is MSF, 0: address format is LBA form
                                        1: address format is MSF form */
   u8  format;          /* bit3~bit0,   MSF Field   Track/Session Number
                           0000b:       Valid       Valid as a Track Number
                           0001b:       Valid       Ignored by Drive
                           0010b:       Ignored     Valid as a Session Number
                           0011b~0101b: Ignored     Ignored by Drive
                           0110b~1111b: Reserved
                        */
   u8  reserved1;  
   u8  reserved2;  
   u8  reserved3;  
   u8  session_num;     /* a specific session or a track */
   u8  allocation_length_msb;
   u8  allocation_length_lsb;
   u8  control;
} usbsdms_read_toc_cmd_type;

typedef struct
{   
   unsigned char usb_serial[USB_SERIAL_LEN];
   unsigned char vender_name[VENDOR_NAME_LEN];
   unsigned char country_name[COUNTRY_NAME_LEN];
} usb_param;

#endif  /* __HUAWEI_USB_H__ */

extern void usb_port_switch_request(int usb_pid_index);
extern usb_param usb_parameter;

