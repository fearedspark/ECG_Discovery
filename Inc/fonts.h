#ifndef __FONTS_H__
#define __FONTS_H__

#include "shared.h"

typedef struct
{
    uint32_t * output_buffer_address;
    int output_buffer_width;
    int output_buffer_height;
    int start_x;
    int position_x;
    int position_y;
    const uint8_t * font;
    uint32_t fg_color;
    uint32_t bg_color;
}
TextDisplayInfo;

void initTextBuffer(uint32_t * output_buffer_address, int output_buffer_width, int output_buffer_height, int x, int y, const uint8_t * font, uint32_t fg_color, uint32_t bg_color);
void setTextBuffer(uint32_t * output_buffer_address, int output_buffer_width, int output_buffer_height);
void setTextPosition(int x, int y);
void setTextFont(const uint8_t * font);
void setTextColorBG(uint32_t fg_color, uint32_t bg_color);

int display_putc(uint8_t c);

extern const uint8_t Arial12x12[];
extern const uint8_t Arial24x23[];
extern const uint8_t Arial28x28[];
extern const uint8_t Neu42x35[];
extern const uint8_t Tahoma42x37[];

#endif
