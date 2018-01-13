#include <stdint.h>
#include "spi_flash_pn25fxx.h"

#define FLASH_DEBUG

#ifdef FLASH_DEBUG
#define FLASH_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* FLASH_DEBUG */

static struct spi_flash_pn25fxx  flash_pn25fxx;

static void flash_lock(struct spi_flash_pn25fxx * flash_device)
{
    rt_mutex_take(&flash_device->lock, RT_WAITING_FOREVER);
}

static void flash_unlock(struct spi_flash_pn25fxx * flash_device)
{
    rt_mutex_release(&flash_device->lock);
}

static uint8_t pn25fxx_read_status(void)
{
    return rt_spi_sendrecv8(flash_pn25fxx.rt_spi_device, CMD_RDSR1);
}

static void pn25fxx_wait_busy(void)
{
    while( pn25fxx_read_status() & (0x01));
}

/** \brief read [size] byte from [offset] to [buffer]
 *
 * \param offset uint32_t unit : byte
 * \param buffer uint8_t*
 * \param size uint32_t   unit : byte
 * \return uint32_t byte for read
 *
 */
static uint32_t pn25fxx_read(uint32_t offset, uint8_t * buffer, uint32_t size)
{
    uint8_t send_buffer[4];

    send_buffer[0] = CMD_WRDI;
    rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_READ;
    send_buffer[1] = (uint8_t)(offset>>16);
    send_buffer[2] = (uint8_t)(offset>>8);
    send_buffer[3] = (uint8_t)(offset);

    rt_spi_send_then_recv(flash_pn25fxx.rt_spi_device,
                          send_buffer, 4,
                          buffer, size);

    return size;
}

/** \brief write N page on [page]
 *
 * \param page_addr uint32_t unit : byte (4096 * N,1 page = 4096byte)
 * \param buffer const uint8_t*
 * \return uint32_t
 *
 */
uint32_t pn25fxx_page_write(uint32_t page_addr, const uint8_t* buffer)
{
    uint32_t index;
    uint8_t send_buffer[4];

    RT_ASSERT((page_addr&0xFF) == 0); /* page addr must align to 256byte. */

    send_buffer[0] = CMD_WREN;
    rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_ERASE_4K;
    send_buffer[1] = (page_addr >> 16);
    send_buffer[2] = (page_addr >> 8);
    send_buffer[3] = (page_addr);
    rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 4);

    pn25fxx_wait_busy(); // wait erase done.

    for(index=0; index < (PAGE_SIZE / 256); index++)
    {
        send_buffer[0] = CMD_WREN;
        rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 1);

        send_buffer[0] = CMD_PP;
        send_buffer[1] = (uint8_t)(page_addr >> 16);
        send_buffer[2] = (uint8_t)(page_addr >> 8);
        send_buffer[3] = (uint8_t)(page_addr);

        rt_spi_send_then_send(flash_pn25fxx.rt_spi_device,
                              send_buffer,
                              4,
                              buffer,
                              256);

        buffer += 256;
        page_addr += 256;
        pn25fxx_wait_busy();
    }

    send_buffer[0] = CMD_WRDI;
    rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 1);

    return PAGE_SIZE;
}

/* RT-Thread device interface */
static rt_err_t pn25fxx_flash_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t pn25fxx_flash_open(rt_device_t dev, rt_uint16_t oflag)
{
    uint8_t send_buffer[3];

    flash_lock((struct spi_flash_pn25fxx *)dev);

    send_buffer[0] = CMD_WREN;
    rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_WRSR;
    send_buffer[1] = 0;
    send_buffer[2] = 0;
    rt_spi_send(flash_pn25fxx.rt_spi_device, send_buffer, 3);

    pn25fxx_wait_busy();

    flash_unlock((struct spi_flash_pn25fxx *)dev);

    return RT_EOK;
}

static rt_err_t pn25fxx_flash_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t pn25fxx_flash_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = flash_pn25fxx.geometry.bytes_per_sector;
        geometry->sector_count = flash_pn25fxx.geometry.sector_count;
        geometry->block_size = flash_pn25fxx.geometry.block_size;
    }

    return RT_EOK;
}

static rt_size_t pn25fxx_flash_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
    flash_lock((struct spi_flash_pn25fxx *)dev);

    pn25fxx_read(pos*flash_pn25fxx.geometry.bytes_per_sector,
                buffer,
                size*flash_pn25fxx.geometry.bytes_per_sector);

    flash_unlock((struct spi_flash_pn25fxx *)dev);

    return size;
}

static rt_size_t pn25fxx_flash_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
    rt_size_t i = 0;
    rt_size_t block = size;
    const uint8_t * ptr = buffer;

    flash_lock((struct spi_flash_pn25fxx *)dev);

    while(block--)
    {
        pn25fxx_page_write((pos + i)*flash_pn25fxx.geometry.bytes_per_sector,
                          ptr);
        ptr += PAGE_SIZE;
        i++;
    }

    flash_unlock((struct spi_flash_pn25fxx *)dev);

    return size;
}

rt_err_t pn25fxx_init(const char * flash_device_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;

    /* initialize mutex */
    if (rt_mutex_init(&flash_pn25fxx.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init sd lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        FLASH_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    flash_pn25fxx.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(flash_pn25fxx.rt_spi_device, &cfg);
    }

    /* init flash */
    {
        rt_uint8_t cmd;
        rt_uint8_t id_recv[3];
        uint16_t memory_type_capacity;

        flash_lock(&flash_pn25fxx);

        cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
        rt_spi_send(flash_pn25fxx.rt_spi_device, &cmd, 1);

        cmd = CMD_WRDI;
        rt_spi_send(flash_pn25fxx.rt_spi_device, &cmd, 1);

        /* read flash id */
        cmd = CMD_JEDEC_ID;
        rt_spi_send_then_recv(flash_pn25fxx.rt_spi_device, &cmd, 1, id_recv, 3);

        flash_unlock(&flash_pn25fxx);

        if(id_recv[0] != PN_ID)
        {
            FLASH_TRACE("Manufacturers ID error!\r\n");
            FLASH_TRACE("JEDEC Read-ID Data : %02X %02X %02X\r\n", id_recv[0], id_recv[1], id_recv[2]);
            return -RT_ENOSYS;
        }

        flash_pn25fxx.geometry.bytes_per_sector = 4096;
        flash_pn25fxx.geometry.block_size = 4096; /* block erase: 4k */

        /* get memory type and capacity */
        memory_type_capacity = id_recv[1];
        memory_type_capacity = (memory_type_capacity << 8) | id_recv[2];
        
        if(memory_type_capacity == MTC_PN25F128)
        {
            FLASH_TRACE("PN25F128xx detection\r\n");
            flash_pn25fxx.geometry.sector_count = 4096;
        }
        else if(memory_type_capacity == MTC_PN25F64)
        {
            FLASH_TRACE("PN25F64xx detection\r\n");
            flash_pn25fxx.geometry.sector_count = 2048;
        }
        else if(memory_type_capacity == MTC_PN25F32)
        {
            FLASH_TRACE("PN25F32xx detection\r\n");
            flash_pn25fxx.geometry.sector_count = 1024;
        }
        else if(memory_type_capacity == MTC_PN25F16)
        {
            FLASH_TRACE("PN25F16xx detection\r\n");
            flash_pn25fxx.geometry.sector_count = 512;
        }
        else if(memory_type_capacity == MTC_PN25F08)
        {
            FLASH_TRACE("PN25F08xx detection\r\n");
            flash_pn25fxx.geometry.sector_count = 256;
        }
        else if(memory_type_capacity == MTC_PN25F04)
        {
            FLASH_TRACE("PN25F04xx detection\r\n");
            flash_pn25fxx.geometry.sector_count = 128;
        }
        else
        {
            FLASH_TRACE("Memory Capacity error!\r\n");
            return -RT_ENOSYS;
        }
    }

    /* register device */
    flash_pn25fxx.flash_device.type    = RT_Device_Class_Block;
    flash_pn25fxx.flash_device.init    = pn25fxx_flash_init;
    flash_pn25fxx.flash_device.open    = pn25fxx_flash_open;
    flash_pn25fxx.flash_device.close   = pn25fxx_flash_close;
    flash_pn25fxx.flash_device.read    = pn25fxx_flash_read;
    flash_pn25fxx.flash_device.write   = pn25fxx_flash_write;
    flash_pn25fxx.flash_device.control = pn25fxx_flash_control;
    /* no private */
    flash_pn25fxx.flash_device.user_data = RT_NULL;

    rt_device_register(&flash_pn25fxx.flash_device, flash_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}
