#include "drv_spi.h"

#ifdef RT_USING_SPI

#ifdef SPI_USING_DMA
static uint8_t dummy = 0xFF;
#endif /* SPI_USING_DMA */

#ifdef SPI_USING_DMA
/**
 * This is STM32F10x SPI DMA configuration.
 *
 */
static void stm32_spi_dma_configure(struct stm32_spi_bus * stm32_spi, const void * send_addr, void * recv_addr, rt_size_t size)
{
    DMA_InitTypeDef DMA_InitStruct;

    if(!stm32_spi->dma)
    {
         return;
    }

    DMA_ClearFlag(stm32_spi->dma->priv_data->DMA_Channel_RX_FLAG_TC |
                  stm32_spi->dma->priv_data->DMA_Channel_RX_FLAG_TE |
                  stm32_spi->dma->priv_data->DMA_Channel_TX_FLAG_TC | 
                  stm32_spi->dma->priv_data->DMA_Channel_TX_FLAG_TE);

    /* RX channel configuration */
    DMA_Cmd(stm32_spi->dma->priv_data->DMA_Channel_RX, DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(stm32_spi->SPI->DR));
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    
    DMA_InitStruct.DMA_BufferSize = size;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) recv_addr;
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) (&dummy);
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi->dma->priv_data->DMA_Channel_RX, &DMA_InitStruct);

    DMA_ITConfig(stm32_spi->dma->priv_data->DMA_Channel_RX, DMA_IT_TC, ENABLE);
    DMA_Cmd(stm32_spi->dma->priv_data->DMA_Channel_RX, ENABLE);

    /* TX channel configuration */
    DMA_Cmd(stm32_spi->dma->priv_data->DMA_Channel_TX, DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(stm32_spi->SPI->DR));
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStruct.DMA_BufferSize = size;

    if(send_addr != RT_NULL)
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)send_addr;
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(&dummy);
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi->dma->priv_data->DMA_Channel_TX, &DMA_InitStruct);

    DMA_ITConfig(stm32_spi->dma->priv_data->DMA_Channel_TX, DMA_IT_TC, ENABLE);
    DMA_Cmd(stm32_spi->dma->priv_data->DMA_Channel_TX, ENABLE);
}

#ifdef SPI1_USING_DMA
static const struct stm32_spi_dma_private dma1_priv =
{
    DMA1_Channel3,
    DMA1_Channel2,
    DMA1_FLAG_TC3,
    DMA1_FLAG_TE3,
    DMA1_FLAG_TC2,
    DMA1_FLAG_TE2,
    DMA1_Channel3_IRQn,
    DMA1_Channel2_IRQn,
    DMA1_FLAG_GL3,
    DMA1_FLAG_GL2,
};

static struct stm32_spi_dma dma1 =
{
    &dma1_priv,
};

void DMA1_Channel2_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma1.event, SPI_DMA_TX_DONE);
    DMA_ClearFlag(dma1.priv_data->tx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel3_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma1.event, SPI_DMA_RX_DONE);
    DMA_ClearFlag(dma1.priv_data->rx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* SPI1_USING_DMA */

#ifdef SPI2_USING_DMA
static const struct stm32_spi_dma_private dma2_priv =
{
    DMA1_Channel5,
    DMA1_Channel4,
    DMA1_FLAG_TC5,
    DMA1_FLAG_TE5,
    DMA1_FLAG_TC4,
    DMA1_FLAG_TE4,
    DMA1_Channel5_IRQn,
    DMA1_Channel4_IRQn,
    DMA1_FLAG_GL5,
    DMA1_FLAG_GL4,
};

static struct stm32_spi_dma dma2 =
{
    &dma2_priv,
};

void DMA1_Channel4_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma2.event, SPI_DMA_TX_DONE);
    DMA_ClearFlag(dma2.priv_data->tx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel5_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma2.event, SPI_DMA_RX_DONE);
    DMA_ClearFlag(dma2.priv_data->rx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* SPI2_USING_DMA */

#ifdef SPI3_USING_DMA
static const struct stm32_spi_dma_private dma3_priv =
{
    DMA2_Channel2,
    DMA2_Channel1,
    DMA2_FLAG_TC2,
    DMA2_FLAG_TE2,
    DMA2_FLAG_TC1,
    DMA2_FLAG_TE1,
    DMA2_Channel2_IRQn,
    DMA2_Channel1_IRQn,
    DMA2_FLAG_GL2,
    DMA2_FLAG_GL1,
};

static struct stm32_spi_dma dma3 =
{
    &dma3_priv,
};

void DMA2_Channel1_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma3.event, SPI_DMA_TX_DONE);
    DMA_ClearFlag(dma3.priv_data->tx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel2_IRQHandler(void) 
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_event_send(&dma3.event, SPI_DMA_RX_DONE);
    DMA_ClearFlag(dma3.priv_data->rx_gl_flag);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* SPI3_USING_DMA */
#endif /* SPI_USING_DMA */

/**
 * This is STM32F10x SPI get baudrate perscaler.
 *
 */
rt_inline uint16_t get_spi_baudrate_prescaler(rt_uint32_t max_hz)
{
    uint16_t get_baudrate_prescaler;

    /* STM32F10x SPI MAX 18Mhz */
    if(max_hz >= SystemCoreClock/2 && SystemCoreClock/2 <= 36000000)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_2;
    }
    else if(max_hz >= SystemCoreClock/4)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_4;
    }
    else if(max_hz >= SystemCoreClock/8)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_8;
    }
    else if(max_hz >= SystemCoreClock/16)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_16;
    }
    else if(max_hz >= SystemCoreClock/32)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_32;
    }
    else if(max_hz >= SystemCoreClock/64)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_64;
    }
    else if(max_hz >= SystemCoreClock/128)
    {
        get_baudrate_prescaler = SPI_BaudRatePrescaler_128;
    }
    else
    {
        /* min prescaler 256 */
        get_baudrate_prescaler = SPI_BaudRatePrescaler_256;
    }

    return get_baudrate_prescaler;
}

/**
 * This is STM32F10x SPI device configuration.
 *
 */
static rt_err_t stm32_spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    struct stm32_spi_bus * stm32_spi = (struct stm32_spi_bus *)device->bus;
    SPI_InitTypeDef SPI_InitStruct;

    SPI_StructInit(&SPI_InitStruct);

    /* data_width */
    if(configuration->data_width <= 8)
    {
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    }
    else if(configuration->data_width <= 16)
    {
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    SPI_InitStruct.SPI_BaudRatePrescaler = get_spi_baudrate_prescaler(configuration->max_hz);
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    }
    else
    {
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else
    {
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    }
    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    }
    else
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;

    /* init SPI */
    SPI_I2S_DeInit(stm32_spi->SPI);
    SPI_Init(stm32_spi->SPI, &SPI_InitStruct);
    /* Enable SPI_MASTER */
    SPI_Cmd(stm32_spi->SPI, ENABLE);
    SPI_CalculateCRC(stm32_spi->SPI, DISABLE);

    return RT_EOK;
};

/**
 * This is STM32F10x SPI xfer.
 *
 */
static rt_uint32_t stm32_spi_xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct stm32_spi_bus * spi_bus = (struct stm32_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
    SPI_TypeDef * SPI = spi_bus->SPI;
    struct stm32_spi_cs * stm32_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if(message->cs_take && stm32_spi_cs)
    {
        GPIO_ResetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
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
            stm32_spi_dma_configure(spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
            rt_event_recv(&spi_bus->dma->event, SPI_DMA_COMPLETE,
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &ev);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
            DMA_ITConfig(spi_bus->dma->priv_data->DMA_Channel_TX, DMA_IT_TC, DISABLE);
            DMA_ITConfig(spi_bus->dma->priv_data->DMA_Channel_RX, DMA_IT_TC, DISABLE);
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
                while (SPI_I2S_GetFlagStatus(SPI, SPI_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_FLAG_RXNE) == RESET);
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
                while (SPI_I2S_GetFlagStatus(SPI, SPI_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_FLAG_RXNE) == RESET);
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
    if(message->cs_release && stm32_spi_cs)
    {
        GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    }

    return message->length;
};

static struct rt_spi_ops stm32_spi_ops =
{
    stm32_spi_configure,
    stm32_spi_xfer
};

/** \brief init and register stm32 spi bus.
 *
 * \param SPI: STM32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param stm32_spi: stm32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
static rt_err_t stm32_spi_register(SPI_TypeDef * SPI,
                            struct stm32_spi_bus * spi_bus,
                            const char * spi_bus_name)
{
    rt_err_t res = RT_EOK;
#ifdef SPI_USING_DMA
    NVIC_InitTypeDef NVIC_InitStruct;
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
            NVIC_InitStruct.NVIC_IRQChannel = dma1.priv_data->tx_irq_ch;
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            NVIC_InitStruct.NVIC_IRQChannel = dma1.priv_data->rx_irq_ch;
            NVIC_InitStruct.NVIC_IRQChannel = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Enable the DMA1 Clock */
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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
            NVIC_InitStruct.NVIC_IRQChannel = dma2.priv_data->tx_irq_ch;
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            NVIC_InitStruct.NVIC_IRQChannel = dma2.priv_data->rx_irq_ch;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Enable the DMA1 Clock */
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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
            NVIC_InitStruct.NVIC_IRQChannel = dma3.priv_data->tx_irq_ch;
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            NVIC_InitStruct.NVIC_IRQChannel = dma3.priv_data->rx_irq_ch;
            NVIC_InitStruct.NVIC_IRQChannelCmd= ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Enable the DMA1 Clock */
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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
    res = rt_spi_bus_register(&spi_bus->bus, spi_bus_name, &stm32_spi_ops);
    spi_bus->bus.parent.flag |= flags;

    return res;
}

/**
 * This is the SPI rcc configuration.
 *
 */
static void stm32_spi_rcc_configure(void)
{	
#if defined(RT_USING_SPI1)
    /* Enable USART GPIO clocks */
    RCC_AHBPeriphClockCmd(SPI1_SCK_RCC | SPI1_MOSI_RCC| SPI1_MISO_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
#endif /* RT_USING_SPI11 */

#if defined(RT_USING_SPI2)
    /* Enable USART GPIO clocks */
    RCC_AHBPeriphClockCmd(SPI2_SCK_RCC | SPI2_MOSI_RCC | SPI2_MISO_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
#endif /* RT_USING_SPI2 */
	
#if defined(RT_USING_SPI3)
    /* Enable USART GPIO clocks */
    RCC_AHBPeriphClockCmd(SPI3_SCK_RCC | SPI3_MOSI_RCC | SPI3_MISO_RCC, ENABLE);
    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
#endif /* RT_USING_SPI3 */
}

/**
 * This is the SPI GPIO configuration.
 *
 */
static void stm32_spi_gpio_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	
#if defined(RT_USING_SPI1)
    /* Configure SPI1 SCK MOSI MISO PIN */
    
    GPIO_InitStruct.GPIO_Pin = SPI1_SCK_PIN;
    GPIO_Init(SPI1_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SPI1_MOSI_PIN;
    GPIO_Init(SPI1_MOSI_PORT, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = SPI1_MISO_PIN;
    GPIO_Init(SPI1_MISO_PORT, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(SPI1_SCK_PORT, SPI1_SCK_AFIO, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_MOSI_PORT, SPI1_MOSI_AFIO, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_MISO_PORT, SPI1_MISO_AFIO, GPIO_AF_SPI1);
#endif /* RT_USING_SPI1 */
	
#if defined(RT_USING_SPI2)
    /* Configure SPI2 SCK MOSI MISO PIN */
    
    GPIO_InitStruct.GPIO_Pin = SPI2_SCK_PIN;
    GPIO_Init(SPI2_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SPI2_MOSI_PIN;
    GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = SPI2_MISO_PIN;
    GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(SPI2_SCK_PORT, SPI2_SCK_AFIO, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_MOSI_PORT, SPI2_MOSI_AFIO, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_MISO_PORT, SPI2_MISO_AFIO, GPIO_AF_SPI2);
#endif /* RT_USING_SPI2 */

#if defined(RT_USING_SPI3)
    /* Configure SPI3 SCK MOSI MISO PIN */
    
    GPIO_InitStruct.GPIO_Pin = SPI3_SCK_PIN;
    GPIO_Init(SPI3_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = SPI3_MOSI_PIN;
    GPIO_Init(SPI3_MOSI_PORT, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = SPI3_MISO_PIN;
    GPIO_Init(SPI3_MISO_PORT, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(SPI3_SCK_PORT, SPI3_SCK_AFIO, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_MOSI_PORT, SPI3_MOSI_AFIO, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_MISO_PORT, SPI3_MISO_AFIO, GPIO_AF_SPI3);
#endif /* RT_USING_SPI3 */
}

static int rt_hw_spi_init(void)
{
    /* Enable periph clock */
    stm32_spi_rcc_configure();

    stm32_spi_gpio_configure();
    
#ifdef RT_USING_SPI1
    /* register spi1 bus */
    static struct stm32_spi_bus spi1_bus;
    
    stm32_spi_register(SPI1, &spi1_bus, "spi1");
#endif /* RT_USING_SPI1 */
    
#ifdef RT_USING_SPI2
    /* register spi2 bus */
    static struct stm32_spi_bus spi2_bus;

    stm32_spi_register(SPI2, &spi2_bus, "spi2");
#endif /* RT_USING_SPI2 */
    
#ifdef RT_USING_SPI3
    /* register spi3 bus */
    static struct stm32_spi_bus spi3_bus;

    stm32_spi_register(SPI3, &spi3_bus, "spi3");
#endif /* RT_USING_SPI3 */
    return 0;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);
#endif /* RT_USING_SPI */
