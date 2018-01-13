#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "drv_usart.h"

/**
 * This is GD32F10x system clock configuration.
 *
 */
void gd32_system_clock_configure(void)
{
	RCC_DeInit();
	RCC_HSI_Enable(ENABLE);
	while (RCC_GetBitState(RCC_FLAG_HSISTB) == RESET); 

	RCC_PLLConfig(RCC_PLLSOURCE_HSI_DIV2, RCC_PLLMUL_27);//108MHz
	RCC_PLL_Enable(ENABLE); //使能锁相环PLL 
	while (RCC_GetBitState(RCC_FLAG_PLLSTB) == RESET){};

	RCC_CK_SYSConfig(RCC_SYSCLKSOURCE_PLLCLK);
	while (RCC_GetCK_SYSSource() != 0x08);

	RCC_AHBConfig(RCC_SYSCLK_DIV1);  // 108MHz
	RCC_APB1Config(RCC_SYSCLK_DIV2); // 54MHz
	RCC_APB2Config(RCC_SYSCLK_DIV1); // 108MHz
        
	RCC_APB2PeriphClock_Enable( RCC_APB2PERIPH_AF, ENABLE);
}

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

void gd32_nvic_configure(void)
{
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_VectTableSet(NVIC_VECTTAB_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_VectTableSet(NVIC_VECTTAB_FLASH, 0x0);
#endif
    NVIC_PRIGroup_Enable(NVIC_PRIGROUP_2);
}

/**
 * This function will initial GD32 board.
 */
void rt_hw_board_init(void)
{
		/* Configure the System clk */
    gd32_system_clock_configure();
	
		/* Configure the NVIC */
    gd32_nvic_configure();

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
