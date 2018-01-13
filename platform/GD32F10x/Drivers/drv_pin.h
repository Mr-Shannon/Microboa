#ifndef __DRV_PIN_H__
#define __DRV_PIN_H__

#include <rtdevice.h>
#include "gd32f10x.h"
#include "board.h"

#ifdef RT_USING_PIN

#define GD32F10X_PIN_NUMBERS    48

struct gd32_hw_pin_userdata
{
    int pin;
    uint32_t mode;
};

#define PIN_USERDATA_END {-1,0}

extern struct gd32_hw_pin_userdata gd32_pins[];

int gd32_hw_pin_init(void);

#endif /* RT_USING_PIN */
#endif

