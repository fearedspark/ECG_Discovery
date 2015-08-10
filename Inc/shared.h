#ifndef __SHARED_H__
#define __SHARED_H__
#define ARM_MATH_CM7

#include "stm32f7xx_hal.h"
#include <math.h>
#include "main.h"
#include "functions.h"
#include "fonts.h"
#include "touch.h"
#include "ecg.h"

//#define PI 3.14159265
#define RGB(a,r,g,b) ((((uint32_t)a&0xFF)<<24) | (((uint32_t)r&0xFF)<<16) | (((uint32_t)g&0xFF)<<8) | ((uint32_t)b&0xFF))
#define MIN(x,y) ((x<y)?x:y)
#define MAX(x,y) ((x<y)?y:x)
#define ABS(x) ((x<0)?(-x):x)

//#define DISPLAY_LAYER0_ADDR 0xC0000000
#define DISPLAY_LAYER0_BUFFER0_ADDR 0xC0000000
#define DISPLAY_LAYER0_BUFFER1_ADDR 0xC0060000
#define DISPLAY_LAYER0_BUFFER2_ADDR 0xC00C0000
#define DISPLAY_LAYER0_BUFFER3_ADDR 0xC0120000
#define DISPLAY_LAYER0_BUFFER4_ADDR 0xC0180000
#define DISPLAY_LAYER1_ADDR 0xC0200000
//#define DISPLAY_LAYER1_BUFFER0_ADDR 0xC0100000
//#define DISPLAY_LAYER1_BUFFER1_ADDR 0xC0180000

//extern uint32_t * Display_layer0;
extern uint32_t * Display_layer0[];
extern uint32_t * Display_layer1;
//extern uint32_t * Display_layer1_buffer0;
//extern uint32_t * Display_layer1_buffer1;

#define TFT_LCD_EN() GPIOI->BSRR = 0x00001000
#define TFT_LCD_DIS() GPIOI->BSRR = 0x10000000
#define TFT_BL_ON() GPIOK->BSRR = 0x00000008
#define TFT_BL_OFF() GPIOK->BSRR = 0x00080000

#define TFT_RED         0xFFFF0000
#define TFT_YELLOW      0xFFFFFF00
#define TFT_GREEN       0xFF00FF00
#define TFT_BLUE        0xFF0000FF
#define TFT_WHITE       0xFFFFFFFF
#define TFT_TRANSPARENT 0x00000000

#endif
