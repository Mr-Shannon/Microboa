#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32l1xx.h"
#include <rtthread.h>

extern int Image$$RW_IRAM1$$ZI$$Limit;

#define STM32_SRAM_SIZE					16
#define STM32_SRAM_END 					(0x20000000 + STM32_SRAM_SIZE * 1024)
#define HEAP_BEGIN  					((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#define HEAP_END    					STM32_SRAM_END

void rt_hw_board_init(void);

#endif  /*__BOARD_H__*/
