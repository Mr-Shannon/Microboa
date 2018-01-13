#include "drv_spirit1.h"

static struct spi_spirit1_device spirit1_device;

static void spirit1_lock(struct spi_spirit1_device * spi_spirit1)
{
    rt_mutex_take(&spi_spirit1->lock, RT_WAITING_FOREVER);
}

static void spirit1_unlock(struct spi_spirit1_device * spi_spirit1)
{
    rt_mutex_release(&spi_spirit1->lock);
}

void spirit1_deinit(void)
{
    return ;
}

void spirit1_init(void)
{
     /* initialize mutex */
    if (rt_mutex_init(&spirit1_device.lock, "spi20", RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init sd lock mutex failed\n");
        return ;
    }
    
    spirit1_device.spi_device = (struct rt_spi_device *)rt_device_find("spi20");
    if(spirit1_device.spi_device == RT_NULL)
    {
        rt_kprintf("\nspi_device %s for spirit1 not found!\n", "spi20");
        return ;
    }
}

StatusBytes spirit1_write_reg(uint8_t addr, uint8_t len, uint8_t* buffer)
{
    uint8_t header[TMP_BUFFER_LEN], tmp[TMP_BUFFER_LEN];
    uint16_t tmp_status = 0x0000;
    StatusBytes status;
    
    header[0] = SPIRIT1_WRITE;
    header[1] = addr;
    
    for (uint8_t i = 0; i < len; i++)
        tmp[i+2] = buffer[i];
    
    spirit1_lock(&spirit1_device);
    rt_spi_transfer(spirit1_device.spi_device, &header, &tmp, len + 2);
    spirit1_unlock(&spirit1_device);
    
    tmp_status = (tmp[0] << 8) + tmp[1];  
    
    *((uint16_t*)&status) = tmp_status;
    
    return status;
}

StatusBytes spirit1_read_reg(uint8_t addr, uint8_t len, uint8_t* buffer)
{
    uint8_t header[TMP_BUFFER_LEN], tmp[TMP_BUFFER_LEN];
    uint16_t tmp_status = 0x0000;
    StatusBytes status;
    
    header[0] = SPIRIT1_READ;
    header[1] = addr;
    
    spirit1_lock(&spirit1_device);
    rt_spi_transfer(spirit1_device.spi_device, &header, &tmp, len + 2);
    spirit1_unlock(&spirit1_device);
    
    for (uint8_t i = 0; i < len; i++)
        buffer[i] = tmp[i+2];
    
    tmp_status = (tmp[0] << 8) + tmp[1];
    
    *((uint16_t*)&status) = tmp_status;
    
    return status;
}

StatusBytes spirit1_command_strobe(uint8_t command)
{
    uint8_t header[TMP_BUFFER_LEN], tmp[TMP_BUFFER_LEN];
    uint16_t tmp_status = 0x0000;
    StatusBytes status;
    
    header[0] = SPIRIT1_COMMAND;
    header[1] = command;
    
    spirit1_lock(&spirit1_device);
    rt_spi_transfer(spirit1_device.spi_device, &header, &tmp, 2);
    spirit1_unlock(&spirit1_device);
    
    tmp_status = (tmp[0] << 8) + tmp[1];
    
    *((uint16_t*)&status) = tmp_status;
    
    return status;
}

StatusBytes spirit1_write_fifo(uint8_t len, uint8_t* buffer)
{
    uint8_t header[TMP_BUFFER_LEN], tmp[TMP_BUFFER_LEN];
    uint16_t tmp_status = 0x0000;
    StatusBytes status;
    
    header[0] = SPIRIT1_WRITE;
    header[1] = SPIRIT1_FIFO_ADDR;
    
    for (uint8_t i = 0; i < len; i++)
        tmp[i+2] = buffer[i];
    
    spirit1_lock(&spirit1_device);
    rt_spi_transfer(spirit1_device.spi_device, &header, &tmp, len + 2);
    spirit1_unlock(&spirit1_device);
    
    tmp_status = (tmp[0] << 8) + tmp[1];  
    
    *((uint16_t*)&status) = tmp_status;
    
    return status;
}

StatusBytes spirit1_read_fifo(uint8_t len, uint8_t* buffer)
{
    uint8_t header[TMP_BUFFER_LEN], tmp[TMP_BUFFER_LEN];
    uint16_t tmp_status = 0x0000;
    StatusBytes status;
    
    header[0] = SPIRIT1_READ;
    header[1] = SPIRIT1_FIFO_ADDR;
    
    spirit1_lock(&spirit1_device);
    rt_spi_transfer(spirit1_device.spi_device, &header, &tmp, len + 2);
    spirit1_unlock(&spirit1_device);
    
    for (uint8_t i = 0; i < len; i++)
        buffer[i] = tmp[i+2];
    
    tmp_status = (tmp[0] << 8) + tmp[1]; 
    
    *((uint16_t*)&status) = tmp_status;
    
    return status;
}

void spirit1_enter_shutdown(void)
{
    GPIO_SetBits(SPIRIT1_SDN_PORT, SPIRIT1_SDN_PIN);
}

void spirit1_exit_shutdown(void)
{
    GPIO_ResetBits(SPIRIT1_SDN_PORT, SPIRIT1_SDN_PIN);
    for(volatile uint32_t i = 0; i < 0x1fff; i++);
}

SpiritFlagStatus spirit1_check_shutdown(void)
{
  if(GPIO_ReadInputDataBit(SPIRIT1_SDN_PORT, SPIRIT1_SDN_PIN))
    return S_SET;
  else
    return S_RESET;
}

static int spirit1_hw_init(void)
{
    static struct rt_spi_device spi20_device;
    static struct stm32_spi_cs  spi20_cs;
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* attach spi20 */
    {
        RCC_AHBPeriphClockCmd(SPIRIT1_CS_RCC, ENABLE);
        
        spi20_cs.GPIOx = SPIRIT1_CS_PORT;
        spi20_cs.GPIO_Pin = SPIRIT1_CS_PIN;
        
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_Pin = spi20_cs.GPIO_Pin;
        GPIO_SetBits(spi20_cs.GPIOx, spi20_cs.GPIO_Pin);
        GPIO_Init(spi20_cs.GPIOx, &GPIO_InitStruct);

        rt_spi_bus_attach_device(&spi20_device, "spi20", "spi2", (void*)&spi20_cs);;
    } /* attach spi20 */ 
    
    /* config spi20 */
    {
        struct rt_spi_configuration spi20_cfg;
        spi20_cfg.data_width = 8;
        spi20_cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        spi20_cfg.max_hz = 20 * 1000 * 1000; /* SPI Interface with Clock Speeds Up to 20 MHz */
        rt_spi_configure(&spi20_device, &spi20_cfg);
    } /* config spi20 */
    SpiritSpiInit();
    return 0;
}
INIT_APP_EXPORT(spirit1_hw_init);

#ifdef USING_SPIRIT1_DEBUG
void Spirit1_StatusInfo(StatusBytes *status)
{
    rt_kprintf("Spirit1 Status:\n");
    rt_kprintf("{\n");
    rt_kprintf("    ANT_SELECT:%x\n",status->ANT_SELECT);
    rt_kprintf("    ERROR_LOCK:%x\n",status->ERROR_LOCK);
    rt_kprintf("    MC_STATE:%x\n",status->MC_STATE);
    rt_kprintf("    RX_FIFO_EMPTY:%x\n",status->RX_FIFO_EMPTY);
    rt_kprintf("    TX_FIFO_FULL:%x\n",status->TX_FIFO_FULL);
    rt_kprintf("    XO_ON:%x\n",status->XO_ON);
    rt_kprintf("}\n");
}

void Write_Operation(uint8_t addr, uint8_t data)
{
    StatusBytes status;
    
    status = SpiritSpiWriteRegisters(addr, 1, &data);
    Spirit1_StatusInfo(&status);
    rt_kprintf("Write register:\n");
    rt_kprintf("reg[%x]:%3x\n", addr, data);    
}
FINSH_FUNCTION_EXPORT(Write_Operation, Write data to Spirit1 register.)
    
void Read_Operation(uint8_t addr, uint8_t length)
{
    uint8_t buffer[TMP_BUFFER_LEN];
    StatusBytes status;
    
    status = SpiritSpiReadRegisters(addr, length, buffer);
    Spirit1_StatusInfo(&status);
    rt_kprintf("Read register:\n");
    for (uint8_t i = 0; i < length; i++)
    {
        rt_kprintf("reg[%x]:0x%x", addr+i, buffer[i]);
        rt_kprintf("\n");
    }      
}
FINSH_FUNCTION_EXPORT(Read_Operation, Read data form Spirit1 register.)

void Switch_Mode(uint8_t cmd)
{
    StatusBytes status;
    uint8_t mode;
    
    status = SpiritSpiCommandStrobes(cmd);
    for(volatile uint8_t i=0; i!=0xFF; i++);
    SpiritRefreshStatus();
    status = g_xStatus;
    Spirit1_StatusInfo(&status);
    mode = g_xStatus.MC_STATE;
    switch (mode)
    {
    case MC_STATE_STANDBY:
        rt_kprintf("Switch to [Standby] mode.\n");
        break;
    case MC_STATE_SLEEP:
        rt_kprintf("Switch to [Sleep] mode.\n");
        break;
    case MC_STATE_READY:
        rt_kprintf("Switch to [Ready] mode.\n");
        break;
    case MC_STATE_LOCK:
        rt_kprintf("Switch to [Lock] mode.");
        break;
    case MC_STATE_RX:
        rt_kprintf("Switch to [Rx] mode.\n");
        break;
    case MC_STATE_TX:
        rt_kprintf("Switch to [Tx] mode.\n");
        break;
    default :
        rt_kprintf("Value:%x, Switch mode failed.\n", mode);
        break;
    }
}
FINSH_FUNCTION_EXPORT(Switch_Mode, Switch Spirit1 operating mode.)

void Refresh_Status(void)
{
    StatusBytes status;
    uint8_t mode;
    
    SpiritRefreshStatus();
    status = g_xStatus;
    Spirit1_StatusInfo(&status); 
    mode = g_xStatus.MC_STATE;
    switch (mode)
    {
    case MC_STATE_STANDBY:
        rt_kprintf("Runing to [Standby] mode.\n");
        break;
    case MC_STATE_SLEEP:
        rt_kprintf("Runing to [Sleep] mode.\n");
        break;
    case MC_STATE_READY:
        rt_kprintf("Runing to [Ready] mode.\n");
        break;
    case MC_STATE_LOCK:
        rt_kprintf("Runing to [Lock] mode.");
        break;
    case MC_STATE_RX:
        rt_kprintf("Runing to [Rx] mode.\n");
        break;
    case MC_STATE_TX:
        rt_kprintf("Runing to [Tx] mode.\n");
        break;
    default :
        rt_kprintf("Value:%x, Unknown mode.\n", mode);
        break;
    }    
}
FINSH_FUNCTION_EXPORT(Refresh_Status, Refresh Spirit1 StatusBytes.)
#endif /* USING_SPIRIT1_DEBUG */
