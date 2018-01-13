#include "drv_enc28j60.h"

#ifdef RT_USING_SPI1
/**
 * This is the enc28j60 interrupt configure.
 *
 */
void enc28j60_interrupt_configure(void)
{
    NVIC_InitPara NVIC_InitStruct;
    GPIO_InitPara GPIO_InitStruct;
    EXTI_InitPara EXTI_InitStruct;
    
    RCC_APB2PeriphClock_Enable(ENC28J60_INT_RCC, ENABLE); 
    
    /* GPIO */
    GPIO_InitStruct.GPIO_Pin = ENC28J60_INT_PIN;	            
    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IPU;
    GPIO_Init(ENC28J60_INT_PORT, &GPIO_InitStruct);	
    GPIO_EXTILineConfig(GPIO_PORT_SOURCE_GPIOB, GPIO_PINSOURCE0);    
    
    /* NVIC */
    NVIC_InitStruct.NVIC_IRQ = ENC28J60_INT_IRQ;
    NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStruct.NVIC_IRQSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQEnable = ENABLE;	
    NVIC_Init(&NVIC_InitStruct);
    
    /* EXTI1 */
    EXTI_InitStruct.EXTI_LINE = ENC28J60_INT_LINE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LINEEnable = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    EXTI_ClearIntBitState(ENC28J60_INT_LINE);
}

/**
 * This is the enc28j60 IRQHandler.
 *
 */
void EXTI0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    enc28j60_isr();
    
    EXTI_ClearIntBitState(ENC28J60_INT_LINE);

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This is the enc28j60 hardware initial.
 *
 */
static int enc28j60_hw_init(void)
{
    static struct rt_spi_device spi10_device;
    static struct gd32_spi_cs  spi10_cs;
    GPIO_InitPara GPIO_InitStruct;
    
    enc28j60_interrupt_configure(); 
    RCC_APB2PeriphClock_Enable(ENC28J60_CS_RCC, ENABLE);

    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUT_PP;

    spi10_cs.GPIOx = ENC28J60_CS_PORT;
    spi10_cs.GPIO_Pin = ENC28J60_CS_PIN;
    
    GPIO_InitStruct.GPIO_Pin = spi10_cs.GPIO_Pin;
    GPIO_SetBits(spi10_cs.GPIOx, spi10_cs.GPIO_Pin);
    GPIO_Init(spi10_cs.GPIOx, &GPIO_InitStruct);

    rt_spi_bus_attach_device(&spi10_device, "spi10", "spi1", (void*)&spi10_cs);
    enc28j60_attach("spi10");
    
    return 0;
}
INIT_DEVICE_EXPORT(enc28j60_hw_init);

#endif /* RT_USING_SPI1 */
