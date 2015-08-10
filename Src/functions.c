#include "functions.h"

void wait(float sec)
{
    unsigned int usec = sec * 1000000;
    if(sec <= 0)
        return;
    wait_us(usec);
}

void wait_ms(unsigned int ms)
{
    wait_us(ms * 1000);
}

void wait_us(unsigned int us)
{
    StopTim2();
    SetTim2Counter(us);
    ResetTim2Flag();
    StartTim2();
    while((TIM2->SR & 0x00000001) == 0);
}
