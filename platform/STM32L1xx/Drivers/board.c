#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "drv_usart.h"

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * Sets the vector table location and Offset.
 *
 */
void stm32_nvic_configure(void)
{

#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x00);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
#endif /* VECT_TAB_RAM */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/**
 * This function will initial GD32 board.
 */
void rt_hw_board_init(void)
{
    /* STM32L1xx system initial */
    SystemInit();
    
    /* Configure the NVIC */
    stm32_nvic_configure();

    /* Configure the SysTick */
    SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND );
	
#ifdef RT_USING_HEAP
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
    
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}
