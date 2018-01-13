#include "drv_w25qxx.h"

#ifdef RT_USING_SPI1
/**
 * This is the W25QXX hardware initial.
 *
 */
static int w25qxx_hw_init(void)
{
    static struct rt_spi_device spi20_device;
    static struct gd32_spi_cs  spi20_cs;
    GPIO_InitPara GPIO_InitStruct;
    
    RCC_APB2PeriphClock_Enable(W25QXX_RCC_CS, ENABLE);

    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUT_PP;

    spi20_cs.GPIOx = W25QXX_PORT_CS;
    spi20_cs.GPIO_Pin = W25QXX_PIN_CS;
    
    GPIO_InitStruct.GPIO_Pin = spi20_cs.GPIO_Pin;
    GPIO_SetBits(spi20_cs.GPIOx, spi20_cs.GPIO_Pin);
    GPIO_Init(spi20_cs.GPIOx, &GPIO_InitStruct);

    rt_spi_bus_attach_device(&spi20_device, "spi20", "spi2", (void*)&spi20_cs);
    
    w25qxx_init("m0", "spi20");
    
    return 0;
}
INIT_DEVICE_EXPORT(w25qxx_hw_init);

#endif /* RT_USING_SPI */
