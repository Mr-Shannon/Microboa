#ifndef __SPI_FLASH_PN25FXX_H__
#define __SPI_FLASH_PN25FXX_H__

#include <rtthread.h>
#include <drivers/spi.h>


#define PAGE_SIZE                   4096

/* JEDEC Manufacturers ID */
#define PN_ID                       (0xEF)

/* JEDEC Device ID: Memory type and Capacity */
#define MTC_PN25F04                 (0x4013) /* PN25F04xx */
#define MTC_PN25F08                 (0x4014) /* PN25F08xx */
#define MTC_PN25F16                 (0x4015) /* PN25F16xx */
#define MTC_PN25F32                 (0x4016) /* PN25F32xx */
#define MTC_PN25F64                 (0x4017) /* PN25F64xx */
#define MTC_PN25F128                (0x4018) /* PN25F128xx */

/* command list */
#define CMD_WRSR                    (0x01)  /* Write Status Register */
#define CMD_PP                      (0x02)  /* Page Program */
#define CMD_READ                    (0x03)  /* Read Data */
#define CMD_WRDI                    (0x04)  /* Write Disable */
#define CMD_RDSR1                   (0x05)  /* Read Status Register-1 */
#define CMD_WREN                    (0x06)  /* Write Enable */
#define CMD_FAST_READ               (0x0B)  /* Fast Read */
#define CMD_ERASE_4K                (0x20)  /* Sector Erase:4K */
#define CMD_RDSR2                   (0x35)  /* Read Status Register-2 */
#define CMD_ERASE_32K               (0x52)  /* 32KB Block Erase */
#define CMD_JEDEC_ID                (0x9F)  /* Read JEDEC ID */
#define CMD_ERASE_full              (0xC7)  /* Chip Erase */
#define CMD_ERASE_64K               (0xD8)  /* 64KB Block Erase */

#define DUMMY                       (0xFF)

struct spi_flash_pn25fxx
{
    struct rt_device                flash_device;
    struct rt_device_blk_geometry   geometry;
    struct rt_spi_device *          rt_spi_device;
    struct rt_mutex                 lock;
};

extern rt_err_t pn25fxx_init(const char * flash_device_name,
                            const char * spi_device_name);


#endif /* __SPI_FLASH_PN25FXX_H__ */ 
