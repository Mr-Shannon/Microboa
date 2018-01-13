#include "drv_usart.h"

#ifdef RT_USING_SERIAL

/**
 * This is GD32F10x usart device configuration.
 *
 */
static rt_err_t gd32_usart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct gd32_usart* usart;
    USART_InitPara USART_InitStruct;
    
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    
    usart = (struct gd32_usart *)serial->parent.user_data;
    
    USART_InitStruct.USART_BRR = cfg->baud_rate;
    
    switch (cfg->data_bits)
    {
    case DATA_BITS_8:
        USART_InitStruct.USART_WL = USART_WL_8B;
        break;
    case DATA_BITS_9:
        USART_InitStruct.USART_WL = USART_WL_9B;
        break;
    default:
        USART_InitStruct.USART_WL = USART_WL_8B;
        break;
    }
    
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        USART_InitStruct.USART_STBits = USART_STBITS_1;
        break;
    case STOP_BITS_2:
        USART_InitStruct.USART_STBits = USART_STBITS_2;
        break;
    default:
        USART_InitStruct.USART_STBits = USART_STBITS_1;
        break;
    }
    
    switch (cfg->parity)
    {
    case PARITY_NONE:
        USART_InitStruct.USART_Parity = USART_PARITY_RESET;
        break;
    case PARITY_EVEN:
        USART_InitStruct.USART_Parity = USART_PARITY_SETEVEN;
        break;
    case PARITY_ODD:
        USART_InitStruct.USART_Parity = USART_PARITY_SETODD;
        break;
    default:
        USART_InitStruct.USART_Parity = USART_PARITY_RESET;
    break;
    }

    USART_InitStruct.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
    USART_InitStruct.USART_RxorTx = USART_RXORTX_RX | USART_RXORTX_TX;
    USART_DeInit(usart->usart_device);
    USART_Init(usart->usart_device, &USART_InitStruct);

    /* Enable USART */
    USART_Enable(usart->usart_device, ENABLE);

    return RT_EOK;
}

#ifdef USART_USING_DMA
/**
 * This is the usart DMA configuration.
 *
 */
static void gd32_usart_dma_configure(struct rt_serial_device *serial) 
{
    struct gd32_usart *usart = (struct gd32_usart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
    DMA_InitPara DMA_InitStruct;
    NVIC_InitPara NVIC_InitStruct;

    /* enable transmit idle interrupt */
    USART_INT_Set(usart->usart_device, USART_INT_IDLEF , ENABLE);

    /* DMA clock enable */
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA2, ENABLE);

    /* rx dma config */
    DMA_DeInit(usart->dma.rx_ch);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(usart->usart_device->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) rx_fifo->buffer;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_BufferSize = serial->config.bufsz;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_HIGH;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(usart->dma.rx_ch, &DMA_InitStruct);
    DMA_ClearBitState(usart->dma.rx_gl_flag);
    DMA_INTConfig(usart->dma.rx_ch, DMA_INT_TC, ENABLE);
    USART_DMA_Enable(usart->usart_device, USART_DMAREQ_RX, ENABLE);
    DMA_Enable(usart->dma.rx_ch, ENABLE);

    /* rx dma interrupt config */
    NVIC_InitStruct.NVIC_IRQ = usart->dma.rx_irq_ch;
    NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStruct.NVIC_IRQSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
#endif /* USART_USING_DMA */

/**
 * This is the usart control.
 *
 */
rt_err_t gd32_usart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct gd32_usart *usart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);
    
    RT_ASSERT(serial != RT_NULL);
    
    usart = (struct gd32_usart *) serial->parent.user_data;
    
    switch (cmd)
    {
        /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(usart->irq);
        /* disable interrupt */
        USART_INT_Set(usart->usart_device, USART_INT_RBNE, DISABLE);
        break;
        /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_EnableIRQ(usart->irq);
        /* enable interrupt */
        USART_INT_Set(usart->usart_device, USART_INT_RBNE, ENABLE);
        break;
        /* USART config */
		#ifdef USART_USING_DMA
    case RT_DEVICE_CTRL_CONFIG :
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
            gd32_usart_dma_configure(serial);
        }
        break;
		#endif /* USART_USING_DMA */
    }
    return RT_EOK;
}

/**
 * This is the usart putchar.
 *
 */
static int gd32_usart_putc(struct rt_serial_device *serial, char c)
{
    struct gd32_usart* usart;

    RT_ASSERT(serial != RT_NULL);
    
    usart = (struct gd32_usart *)serial->parent.user_data;

    if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        if (!(usart->usart_device->STR & USART_FLAG_TBE))
        {
            USART_INT_Set(usart->usart_device, USART_INT_TC, ENABLE);
            return -1;
        }
        usart->usart_device->DR = c;
        USART_INT_Set(usart->usart_device, USART_INT_TC, ENABLE);
    }
    else
    {
        usart->usart_device->DR = c;
        while (!(usart->usart_device->STR & USART_FLAG_TC));
    }

    return 1;
}

/**
 * This is the usart getchar.
 *
 */
static int gd32_usart_getc(struct rt_serial_device *serial)
{
    int ch;
    struct gd32_usart* usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct gd32_usart *)serial->parent.user_data;

    ch = -1;
    if (usart->usart_device->STR & USART_FLAG_RBNE)
    {
        ch = usart->usart_device->DR & 0xff;
    }

    return ch;
}

/**
 * This is the usart NVIC configuration.
 *
 */
static void gd32_usart_nvic_configure(struct gd32_usart* uart)
{
    NVIC_InitPara NVIC_InitStruct;

    /* Enable the USART1 Interrupt */
    NVIC_InitStruct.NVIC_IRQ = uart->irq;
    NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStruct.NVIC_IRQSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

#ifdef USART_USING_DMA
/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void gd32_usart_dma_rx_idle_isr(struct rt_serial_device *serial) 
{
    struct gd32_usart *usart = (struct gd32_usart *) serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    recv_total_index = usart->dma.setting_recv_len - DMA_GetCurrDataCounter(usart->dma.rx_ch);
    recv_len = recv_total_index - usart->dma.last_recv_index;
    usart->dma.last_recv_index = recv_total_index;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    USART_DataReceive(usart->usart_device);
    DMA_ClearBitState(usart->dma.rx_gl_flag);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void gd32_usart_dma_rx_done_isr(struct rt_serial_device *serial) 
{
    struct gd32_usart *usart = (struct gd32_usart *) serial->parent.user_data;
    rt_size_t recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    recv_len = usart->dma.setting_recv_len - usart->dma.last_recv_index;
    /* reset last recv index */
    usart->dma.last_recv_index = 0;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    DMA_ClearBitState(usart->dma.rx_gl_flag);
}
#endif /* USART_USING_DMA */

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void gd32_usart_isr(struct rt_serial_device *serial) 
{
    struct gd32_usart *usart = (struct gd32_usart *) serial->parent.user_data;

    RT_ASSERT(usart != RT_NULL);

    if(USART_GetIntBitState(usart->usart_device, USART_INT_RBNE) != RESET)
    {
        if(USART_GetBitState(usart->usart_device, USART_FLAG_PE) == RESET)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        }
        /* clear interrupt */
        USART_ClearIntBitState(usart->usart_device, USART_INT_RBNE);
    }
    if(USART_GetIntBitState(usart->usart_device, USART_INT_IDLEF) != RESET)
    {
#ifdef USART_USING_DMA
        gd32_usart_dma_rx_idle_isr(serial);
#else
        USART_ClearIntBitState(usart->usart_device, USART_INT_IDLEF);
#endif /* USART_USING_DMA */
    }
    if (USART_GetIntBitState(usart->usart_device, USART_INT_TC) != RESET)
    {
        /* clear interrupt */
        if(serial->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
        {
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DONE);
        }
        USART_INT_Set(usart->usart_device, USART_INT_TC, DISABLE);
        USART_ClearIntBitState(usart->usart_device, USART_INT_TC);
    }
    if (USART_GetBitState(usart->usart_device, USART_FLAG_ORE) == SET)
    {
        gd32_usart_getc(serial);
    }
}

/**
 * This is the usart option.
 *
 */
static const struct rt_uart_ops gd32_usart_ops =
{
    gd32_usart_configure,
    gd32_usart_control,
    gd32_usart_putc,
    gd32_usart_getc,
};

#if defined(RT_USING_USART1)
/* USART1 device driver structure */
struct gd32_usart usart1 =
{
    USART1,
#ifdef USART1_USING_DMA
    USART1_IRQn,
    {
        DMA1_CHANNEL5,
        DMA1_FLAG_GL5,
        DMA1_Channel5_IRQn,
        0,
    },
#endif /* USART1_USING_DMA */
};
struct rt_serial_device serial1;

/**
 * This is the usart1 IRQHandler.
 *
 */
void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    gd32_usart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef USART1_USING_DMA
/**
 * This is the usart1 DMA IRQHandler.
 *
 */
void DMA1_Channel5_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();

    gd32_usart_dma_rx_done_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USART1_USING_DMA */

#endif /* RT_USING_USART1 */

#if defined(RT_USING_USART2)
/* USART2 device driver structure */
struct gd32_usart usart2 =
{
    USART2,
#ifdef USART2_USING_DMA
    USART2_IRQn,
    {
        DMA1_CHANNEL6,
        DMA1_FLAG_GL6,
        DMA1_Channel6_IRQn,
        0,
    },
#endif /* USART2_USING_DMA */
};

struct rt_serial_device serial2;

/**
 * This is the usart2  IRQHandler.
 *
 */
void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    gd32_usart_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#ifdef USART2_USING_DMA
/**
 * This is the usart2 DMA IRQHandler.
 *
 */
void DMA1_Channel6_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();

    gd32_usart_dma_rx_done_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USART2_USING_DMA */

#endif /* RT_USING_USART2 */

#if defined(RT_USING_USART3)
/* USART3 device driver structure */
struct gd32_usart usart3 =
{
    USART3,
#ifdef USART3_USING_DMA
    USART3_IRQn,
    {
        DMA1_CHANNEL3,
        DMA1_FLAG_GL3,
        DMA1_Channel3_IRQn,
        0,
    },
#endif /* USART3_USING_DMA */
};
struct rt_serial_device serial3;

/**
 * This is the usart3 IRQHandler.
 *
 */
void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    gd32_usart_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef USART3_USING_DMA
/**
 * This is the usart3 DMA IRQHandler.
 *
 */
void DMA1_Channel3_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();

    gd32_usart_dma_rx_done_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USART3_USING_DMA */

#endif /* RT_USING_USART3 */

/**
 * This is the usart rcc configuration.
 *
 */
static void gd32_usart_rcc_configure(void)
{
	
#if defined(RT_USING_USART1)
    /* Enable USART GPIO clocks */
    RCC_APB2PeriphClock_Enable(USART1_TX_RCC | USART1_RX_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_USART1, ENABLE);
#endif /* RT_USING_USART1 */

#if defined(RT_USING_USART2)
    /* Enable USART GPIO clocks */
    RCC_APB2PeriphClock_Enable(USART2_TX_RCC | USART2_RX_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART2, ENABLE);
#endif /* RT_USING_USART2 */
	
#if defined(RT_USING_USART3)
    /* Enable USART GPIO clocks */
    RCC_APB2PeriphClock_Enable(USART3_TX_RCC | USART3_RX_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART3, ENABLE);
#endif /* RT_USING_USART3 */
}

/**
 * This is the usart GPIO configuration.
 *
 */
static void gd32_usart_gpio_configure(void)
{
    GPIO_InitPara GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;

#if defined(RT_USING_USART1)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = USART1_RX_PIN;
    GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = USART1_TX_PIN;
    GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
#endif /* RT_USING_USART1 */

#if defined(RT_USING_USART2)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = USART2_RX_PIN;
    GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = USART2_TX_PIN;
    GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);
#endif /* RT_USING_USART2 */

#if defined(RT_USING_USART3)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = USART3_RX_PIN;
    GPIO_Init(USART3_RX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = USART3_TX_PIN;
    GPIO_Init(USART3_TX_PORT, &GPIO_InitStruct);
#endif /* RT_USING_USART3 */
}

/**
 * This is the usart device initial.
 *
 */
static int rt_hw_usart_init(void)
{
    struct gd32_usart* usart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    gd32_usart_rcc_configure();
    gd32_usart_gpio_configure();

#if defined(RT_USING_USART1)
    usart = &usart1;
    config.baud_rate = BAUD_RATE_115200;

    serial1.ops = &gd32_usart_ops;
    serial1.config = config;

    gd32_usart_nvic_configure(usart);

    /* register USART1 device */
    rt_hw_serial_register(&serial1, "usart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
#ifdef USART1_USING_DMA  
                          RT_DEVICE_FLAG_DMA_RX |
#endif /* USART1_USING_DMA */
                          RT_DEVICE_FLAG_INT_TX,
                          usart);
#endif /* RT_USING_USART1 */

#if defined(RT_USING_USART2)
    usart = &usart2;

    config.baud_rate = BAUD_RATE_115200;
    serial2.ops = &gd32_usart_ops;
    serial2.config = config;

    gd32_usart_nvic_configure(usart);

    /* register USART2 device */
    rt_hw_serial_register(&serial2, "usart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
#ifdef USART2_USING_DMA  
                          RT_DEVICE_FLAG_DMA_RX |
#endif /* USART2_USING_DMA */
                          RT_DEVICE_FLAG_INT_TX,
                          usart);
                          
#endif /* RT_USING_USART2 */

#if defined(RT_USING_USART3)
    usart = &usart3;

    config.baud_rate = BAUD_RATE_115200;

    serial3.ops = &gd32_usart_ops;
    serial3.config = config;

    gd32_usart_nvic_configure(usart);

    /* register USART3 device */
    rt_hw_serial_register(&serial3, "usart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
#ifdef USART3_USING_DMA  
                          RT_DEVICE_FLAG_DMA_RX |
#endif /* USART3_USING_DMA */
                          RT_DEVICE_FLAG_INT_TX,
                          usart);
#endif /* RT_USING_USART3 */
	return 0;
}
INIT_BOARD_EXPORT(rt_hw_usart_init);		
#endif /* RT_USING_SERIAL */
