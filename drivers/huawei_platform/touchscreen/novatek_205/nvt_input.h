#ifndef _LINUX_NVT_PROTOCAOL_H
#define _LINUX_NVT_PROTOCAOL_H

#include <linux/kernel.h>
#include <linux/version.h>

#define MT_PROTOCOL_B 1

//---Gesture Control Bit Define-------------
#define GESTURE_BIT_WORD_C            0
#define GESTURE_BIT_WORD_W            1
#define GESTURE_BIT_WORD_V            2
#define GESTURE_BIT_DOUBLE_CLICK    3
#define GESTURE_BIT_WORD_Z            4
#define GESTURE_BIT_WORD_M            5
#define GESTURE_BIT_WORD_O            6
#define GESTURE_BIT_WORD_e            7
#define GESTURE_BIT_WORD_S            8
#define GESTURE_BIT_SLIDE_UP        9
#define GESTURE_BIT_SLIDE_DOWN        10
#define GESTURE_BIT_SLIDE_LEFT        11
#define GESTURE_BIT_SLIDE_RIGHT        12

//---Gesture ID of Novatek----------------
#define GESTURE_ID_WORD_C                12
#define GESTURE_ID_WORD_W               13
#define GESTURE_ID_WORD_V                14
#define GESTURE_ID_DOUBLE_CLICK      15
#define GESTURE_ID_WORD_Z                16
#define GESTURE_ID_WORD_M                17
#define GESTURE_ID_WORD_O                18
#define GESTURE_ID_WORD_e                19
#define GESTURE_ID_WORD_S                20
#define GESTURE_ID_SLIDE_UP              21
#define GESTURE_ID_SLIDE_DOWN        22
#define GESTURE_ID_SLIDE_LEFT           23
#define GESTURE_ID_SLIDE_RIGHT        24

#define IS_GESTURE_ENBALED(x)         (x&0x0FFF)
#define IS_GESTURE_SUPPORTED(x)     (x&0x0FFF)


uint16_t toNovaGestureBits(uint16_t);

#endif




