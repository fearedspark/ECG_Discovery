#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "shared.h"

typedef struct
{
    uint8_t detected_touches;
    uint16_t touch_x[5];
    uint16_t touch_y[5];
    uint8_t weigth[5];
    uint8_t event[5];
    uint8_t area[5];
    uint8_t gesture;
}
touchState;

#define TOUCH_I2C_ADDR                  0x70

#define TOUCH_REG_DEVMODE               0x00 //0x00
#define TOUCH_REG_GEST_ID               0x01 //0xFF -> 0x00
#define TOUCH_REG_STATUS                0x02 //0xFF -> 0x00 (changes after first touch)

#define TOUCH_REG_T0_XH                 0x03
#define TOUCH_REG_T0_XL                 0x04 //Y?? Max = 0x010F = 271
#define TOUCH_REG_T0_YH                 0x05
#define TOUCH_REG_T0_YL                 0x06
#define TOUCH_REG_T0_WEIGHT             0x07
#define TOUCH_REG_T0_MISC               0x08

#define TOUCH_REG_T1_XH                 0x09
#define TOUCH_REG_T1_XL                 0x0A
#define TOUCH_REG_T1_YH                 0x0B
#define TOUCH_REG_T1_YL                 0x0C
#define TOUCH_REG_T1_WEIGHT             0x0D
#define TOUCH_REG_T1_MISC               0x0E

#define TOUCH_REG_T2_XH                 0x0F
#define TOUCH_REG_T2_XL                 0x10
#define TOUCH_REG_T2_YH                 0x11
#define TOUCH_REG_T2_YL                 0x12
#define TOUCH_REG_T2_WEIGHT             0x13
#define TOUCH_REG_T2_MISC               0x14

#define TOUCH_REG_T3_XH                 0x15
#define TOUCH_REG_T3_XL                 0x16
#define TOUCH_REG_T3_YH                 0x17
#define TOUCH_REG_T3_YL                 0x18
#define TOUCH_REG_T3_WEIGHT             0x19
#define TOUCH_REG_T3_MISC               0x1A

#define TOUCH_REG_T4_XH                 0x1B
#define TOUCH_REG_T4_XL                 0x1C
#define TOUCH_REG_T4_YH                 0x1D
#define TOUCH_REG_T4_YL                 0x1E
#define TOUCH_REG_T4_WEIGHT             0x1F
#define TOUCH_REG_T4_MISC               0x20

#define TOUCH_REG_TH_GROUP              0x80 //0x17
#define TOUCH_REG_TH_DIFF               0x85 //0xA0

#define TOUCH_REG_CTRL                  0x86 //0x01

#define TOUCH_REG_TIME_ENTER_MONITOR    0x87 //0x1E
#define TOUCH_REG_PERIOD_ACTIVE         0x88 //0x09
#define TOUCH_REG_PERIOD_MONITOR        0x89 //0x1E

#define TOUCH_REG_MIN_ROT_ANGLE         0x91 //0x00
#define TOUCH_REG_MAX_X_MOTION_OFFSET   0x92 //0x00
#define TOUCH_REG_MAX_Y_MOTION_OFFSET   0x93 //0x00
#define TOUCH_REG_MIN_X_MOTION_DIST     0x94 //0x00
#define TOUCH_REG_MIN_Y_MOTION_DIST     0x95 //0x00
#define TOUCH_REG_MAX_DIST_ZOOM         0x96 //0x00

#define TOUCH_REG_LIB_VER_H             0xA1 //0x05
#define TOUCH_REG_LIB_VER_L             0xA2 //0x0C
#define TOUCH_REG_CIPHER                0xA3 //0x14

#define TOUCH_REG_INT_MODE              0xA4 //0x01
#define TOUCH_REG_PWR_MODE              0xA5 //0x01
#define TOUCH_REG_FIRMWARE_ID           0xA6 //0x12
#define TOUCH_REG_CHIP_ID               0xA8 //2 bytes: 0x51 0x01

#define TOUCH_RELEASE_CODE_ID           0xAF //0x0C

#define TOUCH_REG_STATE_R               0xBC //0x00

#endif
