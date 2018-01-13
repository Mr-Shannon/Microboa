#include "drv_spi.h"

#ifdef RT_USING_SPI

#ifdef SPI_USING_DMA
static uint8_t dummy = 0xFF;
#endif /* SPI_USING_DMA */

#ifdef SPI_USING_DMA
/**
 * This is GD32F10x SPI DMA configuration.
 *
 */
static void gd32_spi_dma_configure(struct gd32_spi_bus * gd32_spi, const void * send_addr, void * recv_addr, rt_size_t size)
{
    DMA_InitPara DMA_InitStruct;

    if(!gd32_spi->dma)
    {
         return;
    }

    DMA_ClearBitState(gd32_spi->dma->priv_data->DMA_Channel_RX_FLAG_TC |
					gd32_spi->dma->priv_data->DMA_Channel_RX_FLAG_TE |
                    gd32_spi->dma->priv_data->DMA_Channel_TX_FLAG_TC | 
                    gd32_spi->dma->priv_data->DMA_Channel_TX_FLAG_TE);

    /* RX channel configuration */
    DMA_Enable(gd32_spi->dma->priv_data->DMA_Channel_RX, DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(gd32_spi->SPI->DTR));
    DMA_InitStruct.DMA_DIR = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    
    DMA_InitStruct.DMA_BufferSize = size;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) recv_addr;
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    }
    else
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) (&dummy);
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_DISABLE;
    }

    DMA_Init(gd32_spi->dma->priv_data->DMA_Channel_RX, &DMA_InitStruct);

    DMA_INTConfig(gd32_spi->dma->priv_data->DMA_Channel_RX, DMA_INT_TC, ENABLE);
    DMA_Enable(gd32_spi->dma->priv_data->DMA_Channel_RX, ENABLE);

    /* TX channel configuration */
    DMA_Enable(gd32_spi->dma->priv_data->DMA_Channel_TX, DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(gd32_spi->SPI->DTR));
    DMA_InitStruct.DMA_DIR = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_MEDIUM;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;

    DMA_InitStruct.DMA_BufferSize = size;

    if(send_addr != RT_NULL)
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)send_addr;
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    }
    else
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(&dummy);
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_DISABLE;
    }

    DMA_Init(gd32_spi->dma->priv_data->DMA_Channel_TX, &DMA_InitStruct);

    DMA_INTConfig(gd32_spi->dma->priv_data->DMA_Channel_TX, DMA_INT_TC, ENABLE);
    DMA_Enable(gd32_spi->dma->priv_data->DMA_Channel_TX, ENABLE);
}

#ifdef SPI1_USING_DMA
static const struct gd32_spi_dma_private dma1_priv =
{
    DMA1_CHANNEL3,
    DMA1_CHANNEL2,
    DMA1_FLAG_TC3,
    DMA1_FLAG_ERR3,
    DMA1_FLAG_TC2,
    DMA1_FLAG_ERR2,
    DMA1_Channel3_IRQn,
    DMA1_Channel2_IRQn,
    DMA1_FLAG_GL3,
    DMA1_FLAG_GL2,
};

static struct gd32_spi_dma dma1 =
{
    &dma1_priv,
};

void DMA1_Channel2_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma1.event, SPI_DMA_TX_DONE);
    DMA_ClearBitState(dma1.priv_data->tx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel3_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma1.event, SPI_DMA_RX_DONE);
    DMA_ClearBitState(dma1.priv_data->rx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* SPI1_USING_DMA */

#ifdef SPI2_USING_DMA
static const struct gd32_spi_dma_private dma2_priv =
{
    DMA1_CHANNEL5,
    DMA1_CHANNEL4,
    DMA1_FLAG_TC5,
    DMA1_FLAG_ERR5,
    DMA1_FLAG_TC4,
    DMA1_FLAG_ERR4,
    DMA1_Channel5_IRQn,
    DMA1_Channel4_IRQn,
    DMA1_FLAG_GL5,
    DMA1_FLAG_GL4,
};

static struct gd32_spi_dma dma2 =
{
    &dma2_priv,
};

void DMA1_Channel4_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma2.event, SPI_DMA_TX_DONE);
    DMA_ClearBitState(dma2.priv_data->tx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel5_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma2.event, SPI_DMA_RX_DONE);
    DMA_ClearBitState(dma2.priv_data->rx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* SPI2_USING_DMA */

#ifdef SPI3_USING_DMA
static const struct gd32_spi_dma_private dma3_priv =
{
    DMA2_CHANNEL2,
    DMA2_CHANNEL1,
    DMA2_FLAG_TC2,
    DMA2_FLAG_TE2,
    DMA2_FLAG_TC1,
    DMA2_FLAG_TE1,
    DMA2_Channel2_IRQn,
    DMA2_Channel1_IRQn,
    DMA2_FLAG_GL2,
    DMA2_FLAG_GL1,
};

static struct gd32_spi_dma dma3 =
{
    &dma3_priv,
};

void DMA2_Channel1_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma3.event, SPI_DMA_TX_DONE);
    DMA_ClearBitState(dma3.priv_data->tx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel2_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma3.event, SPI_DMA_RX_DONE);
    DMA_ClearBitState(dma3.priv_data->rx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* SPI3_USING_DMA */
#endif /* SPI_USING_DMA */

/**
 * This is GD32F10x SPI get baudrate perscaler.
 *
 */
rt_inline uint16_t get_spi_baudrate_prescaler(rt_uint32_t max_hz)
{
    uint16_t get_baudrate_prescaler;

    /* GD32F10x SPI MAX 18Mhz */
    if(max_hz >= SystemCoreClock/2 && SystemCoreClock/2 <= 36000000)
    {
        get_baudrate_prescaler = SPI_PSC_2;
    }
    else if(max_hz >= SystemCoreClock/4)
    {
        get_baudrate_prescaler = SPI_PSC_4;
    }
    else if(max_hz >= SystemCoreClock/8)
    {
        get_baudrate_prescaler = SPI_PSC_8;
    }
    else if(max_hz >= SystemCoreClock/16)
    {
        get_baudrate_prescaler = SPI_PSC_16;
    }
    else if(max_hz >= SystemCoreClock/32)
    {
        get_baudrate_prescaler = SPI_PSC_32;
    }
    else if(max_hz >= SystemCoreClock/64)
    {
        get_baudrate_prescaler = SPI_PSC_64;
    }
    else if(max_hz >= SystemCoreClock/128)
    {
        get_baudrate_prescaler = SPI_PSC_128;
    }
    else
    {
        /* min prescaler 256 */
        get_baudrate_prescaler = SPI_PSC_256;
    }

    return get_baudrate_prescaler;
}

/**
 * This is GD32F10x SPI device configuration.
 *
 */
static rt_err_t gd32_spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    struct gd32_spi_bus * gd32_spi = (struct gd32_spi_bus *)device->bus;
    SPI_InitPara SPI_InitStruct;

    SPI_ParaInit(&SPI_InitStruct);

    /* data_width */
    if(configuration->data_width <= 8)
    {
        SPI_InitStruct.SPI_FrameFormat = SPI_FRAMEFORMAT_8BIT;
    }
    else if(configuration->data_width <= 16)
    {
        SPI_InitStruct.SPI_FrameFormat = SPI_FRAMEFORMAT_16BIT;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    SPI_InitStruct.SPI_PSC = get_spi_baudrate_prescaler(configuration->max_hz);
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        SPI_InitStruct.SPI_SCKPL = SPI_SCKPL_HIGH;
    }
    else
    {
        SPI_InitStruct.SPI_SCKPL = SPI_SCKPL_LOW;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        SPI_InitStruct.SPI_SCKPH = SPI_SCKPH_2EDGE;
    }
    else
    {
        SPI_InitStruct.SPI_SCKPH = SPI_SCKPH_1EDGE;
    }
    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FIRSTBIT_MSB;
    }
    else
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FIRSTBIT_LSB;
    }
    SPI_InitStruct.SPI_TransType = SPI_TRANSTYPE_FULLDUPLEX;
    SPI_InitStruct.SPI_Mode = SPI_MODE_MASTER;
    SPI_InitStruct.SPI_SWNSSEN  = SPI_SWNSS_SOFT;

    /* init SPI */
    SPI_I2S_DeInit(gd32_spi->SPI);
    SPI_Init(gd32_spi->SPI, &SPI_InitStruct);
    /* Enable SPI_MASTER */
    SPI_Enable(gd32_spi->SPI, ENABLE);
    SPI_CRC_Enable(gd32_spi->SPI, DISABLE);

    return RT_EOK;
};

/**
 * This is GD32F10x SPI xfer.
 *
 */
static rt_uint32_t gd32_spi_xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct gd32_spi_bus * spi_bus = (struct gd32_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
    SPI_TypeDef * SPI = spi_bus->SPI;
    struct gd32_spi_cs * gd32_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if(message->cs_take && gd32_spi_cs)
    {
        GPIO_ResetBits(gd32_spi_cs->GPIOx, gd32_spi_cs->GPIO_Pin);
    }
#ifdef SPI_USING_DMA
    if(
       (spi_bus->bus.parent.flag & (RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX)) &&
        spi_bus->dma &&
        message->length > 32)
    {
        if(config->data_width <= 8)
        {
            rt_uint32_t ev = 0;
            gd32_spi_dma_configure(spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMA_Enable(SPI, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, ENABLE);
            rt_event_recv(&spi_bus->dma->event, SPI_DMA_COMPLETE,
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &ev);
            SPI_I2S_DMA_Enable(SPI, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, DISABLE);
            DMA_INTConfig(spi_bus->dma->priv_data->DMA_Channel_TX, DMA_INT_TC, DISABLE);
            DMA_INTConfig(spi_bus->dma->priv_data->DMA_Channel_RX, DMA_INT_TC, DISABLE);
        }
    }
    else
#endif /* SPI_USING_DMA */
    {
        if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_TBE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_RBNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint16_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_TBE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_RBNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }
    /* release CS */
    if(message->cs_release && gd32_spi_cs)
    {
        GPIO_SetBits(gd32_spi_cs->GPIOx, gd32_spi_cs->GPIO_Pin);
    }

    return message->length;
};

static struct rt_spi_ops gd32_spi_ops =
{
    gd32_spi_configure,
    gd32_spi_xfer
};

/** \brief init and register gd32 spi bus.
 *
 * \param SPI: GD32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param gd32_spi: gd32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
static rt_err_t gd32_spi_register(SPI_TypeDef * SPI,
                                struct gd32_spi_bus * spi_bus,
                                const char * spi_bus_name)
{
    rt_err_t res = RT_EOK;
#ifdef SPI_USING_DMA
    NVIC_InitPara NVIC_InitStruct;
#endif
    rt_uint32_t flags = 0;

    if(SPI == SPI1)
    {
        spi_bus->SPI = SPI1;
#ifdef SPI_USING_DMA
#ifdef SPI1_USING_DMA
        {
            rt_event_init(&dma1.event, "spi1ev", RT_IPC_FLAG_FIFO);
            spi_bus->dma = &dma1;
            /* rx dma interrupt config */
            NVIC_InitStruct.NVIC_IRQ = dma1.priv_data->tx_irq_ch;
            NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
            NVIC_InitStruct.NVIC_IRQSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            NVIC_InitStruct.NVIC_IRQ = dma1.priv_data->rx_irq_ch;
            NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Enable the DMA1 Clock */
            RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);
            flags |= RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX;
        }
#else /* !SPI1_USING_DMA */
        spi_bus->dma = RT_NULL;
#endif /* SPI1_USING_DMA */
#endif /* SPI_USING_DMA */
    }
    else if(SPI == SPI2)
    {
        spi_bus->SPI = SPI2;
#ifdef SPI_USING_DMA
#ifdef SPI2_USING_DMA
        {
            rt_event_init(&dma2.event, "spi2ev", RT_IPC_FLAG_FIFO);
            spi_bus->dma = &dma2;
            /* rx dma interrupt config */
            NVIC_InitStruct.NVIC_IRQ = dma2.priv_data->tx_irq_ch;
            NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
            NVIC_InitStruct.NVIC_IRQSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            NVIC_InitStruct.NVIC_IRQ = dma2.priv_data->rx_irq_ch;
            NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Enable the DMA1 Clock */
            RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);
            flags |= RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX;
        }
#else /* !SPI2_USING_DMA */
        spi_bus->dma = RT_NULL;
#endif /* SPI2_USING_DMA */
#endif /* SPI_USING_DMA */
    }
    else if(SPI == SPI3)
    {
    	spi_bus->SPI = SPI3;
#ifdef SPI_USING_DMA
#ifdef SPI3_USING_DMA
        {
            rt_event_init(&dma3.event, "spi3ev", RT_IPC_FLAG_FIFO);
            spi_bus->dma = &dma3;
            /* rx dma interrupt config */
            NVIC_InitStruct.NVIC_IRQ = dma3.priv_data->tx_irq_ch;
            NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
            NVIC_InitStruct.NVIC_IRQSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            NVIC_InitStruct.NVIC_IRQ = dma3.priv_data->rx_irq_ch;
            NVIC_InitStruct.NVIC_IRQEnable= ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Enable the DMA1 Clock */
            RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);
            flags |= RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX;
        }
#else /* !SPI3_USING_DMA */
        spi_bus->dma = RT_NULL;
#endif /* SPI3_USING_DMA */
#endif /* SPI_USING_DMA */
    }
    else
    {
        return RT_ENOSYS;
    }
    res = rt_spi_bus_register(&spi_bus->bus, spi_bus_name, &gd32_spi_ops);
    spi_bus->bus.parent.flag |= flags;

    return res;
}

/**
 * This is the SPI rcc configuration.
 *
 */
static void gd32_spi_rcc_configure(void)
{	
#if defined(RT_USING_SPI1)
    /* Enable USART GPIO clocks */
    RCC_APB2PeriphClock_Enable(SPI1_SCK_RCC | SPI1_MOSI_RCC | SPI1_MISO_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_SPI1, ENABLE);
#endif /* RT_USING_SPI11 */

#if defined(RT_USING_SPI2)
    /* Enable USART GPIO clocks */
    RCC_APB2PeriphClock_Enable(SPI2_SCK_RCC | SPI2_MOSI_RCC | SPI2_MISO_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_SPI2, ENABLE);
#endif /* RT_USING_SPI2 */
	
#if defined(RT_USING_SPI3)
    /* Enable USART GPIO clocks */
    RCC_APB2PeriphClock_Enable(SPI3_SCK_RCC | SPI3_MOSI_RCC | SPI3_MISO_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_SPI3, ENABLE);
#endif /* RT_USING_SPI3 */
}

/**
 * This is the SPI GPIO configuration.
 *
 */
static void gd32_spi_gpio_configure(void)
{
    GPIO_InitPara GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
	  GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
	
#if defined(RT_USING_SPI1)
    /* Configure SPI1 SCK MOSI MISO PIN */
    
    GPIO_InitStruct.GPIO_Pin = SPI1_SCK_PIN;
    GPIO_Init(SPI1_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SPI1_MOSI_PIN;
    GPIO_Init(SPI1_MOSI_PORT, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = SPI1_MISO_PIN;
	  GPIO_Init(SPI1_MISO_PORT, &GPIO_InitStruct);
#endif /* RT_USING_SPI1 */
	
#if defined(RT_USING_SPI2)
    /* Configure SPI2 SCK MOSI MISO PIN */
    
    GPIO_InitStruct.GPIO_Pin = SPI2_SCK_PIN;
    GPIO_Init(SPI2_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SPI2_MOSI_PIN;
    GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = SPI2_MISO_PIN;
    GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStruct);
#endif /* RT_USING_SPI2 */

#if defined(RT_USING_SPI3)
    /* Configure SPI3 SCK MOSI MISO PIN */
    
    GPIO_InitStruct.GPIO_Pin = SPI3_SCK_PIN;
    GPIO_Init(SPI3_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SPI3_MOSI_PIN;
    GPIO_Init(SPI3_MOSI_PORT, &GPIO_InitStruct);
	
	  GPIO_InitStruct.GPIO_Pin = SPI3_MISO_PIN;
    GPIO_Init(SPI3_MISO_PORT, &GPIO_InitStruct);
#endif /* RT_USING_SPI3 */
}

static int rt_hw_spi_init(void)
{
    /* Enable periph clock */
    gd32_spi_rcc_configure();

    gd32_spi_gpio_configure();
    
#ifdef RT_USING_SPI1
    /* register spi1 bus */
    static struct gd32_spi_bus spi1_bus;
    
    gd32_spi_register(SPI1, &spi1_bus, "spi1");
#endif /* RT_USING_SPI1 */
    
#ifdef RT_USING_SPI2
    /* register spi2 bus */
    static struct gd32_spi_bus spi2_bus;

    gd32_spi_register(SPI2, &spi2_bus, "spi2");
#endif /* RT_USING_SPI2 */
    
#ifdef RT_USING_SPI3
    /* register spi3 bus */
    static struct gd32_spi_bus spi3_bus;

    gd32_spi_register(SPI3, &spi3_bus, "spi3");
#endif /* RT_USING_SPI3 */
    return 0;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);
#endif /* RT_USING_SPI */
