#include "drv_pn25fxx.h"
#include "dfs_fs.h"
#include <dfs_posix.h>
#include <finsh.h>

#ifdef RT_USING_SPI2

static int pn25fxx_hw_init(void)
{
    static struct rt_spi_device spi20_device;
    static struct gd32_spi_cs  spi20_cs;
    GPIO_InitPara GPIO_InitStruct;
    
    RCC_APB2PeriphClock_Enable(PN25FXX_CS_RCC, ENABLE);

    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUT_PP;

    spi20_cs.GPIOx = PN25FXX_CS_PORT;
    spi20_cs.GPIO_Pin = PN25FXX_CS_PIN;
    
    GPIO_InitStruct.GPIO_Pin = spi20_cs.GPIO_Pin;
    GPIO_SetBits(spi20_cs.GPIOx, spi20_cs.GPIO_Pin);
    GPIO_Init(spi20_cs.GPIOx, &GPIO_InitStruct);

    rt_spi_bus_attach_device(&spi20_device, "spi20", "spi2", (void*)&spi20_cs);
    
    pn25fxx_init("flash", "spi20");
    
	return 0;
}
INIT_DEVICE_EXPORT(pn25fxx_hw_init);


void mount(const char* device_name, const char* path)
{
    if (dfs_mount(device_name, path, "elm", 0, 0) == 0)
        rt_kprintf("File System initialized!\n");
    else
        rt_kprintf("File System init failed!\n");
}
FINSH_FUNCTION_EXPORT(mount, mount device);

void file(const char *name)
{
    int fd;
    char s[] = "#include <stdio.h>\r\nint main(void)\r\n{\r\n\tint i = 0;\r\n\ti = i + 1;\r\n\treturn i;\r\n}\r\n";
    /* 打开/text.txt 作写入，如果该文件不存在则建立该文件*/
    fd = open(name, O_WRONLY | O_CREAT, 0);
    if (fd >= 0)
    {
        write(fd, s, sizeof(s));
        close(fd);
    }
    else
        rt_kprintf("create file faile.\n");
}
FINSH_FUNCTION_EXPORT(file, create file test.);


#endif /* RT_USING_SPI2 */
