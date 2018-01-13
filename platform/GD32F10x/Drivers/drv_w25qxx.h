#ifndef __DRV_W25QXX_H__
#define __DRV_W25QXX_H__

#include "drv_spi.h"
#include "spi_flash_w25qxx.h"

#define W25QXX_RCC_CS		            RCC_APB2PERIPH_GPIOB
#define W25QXX_PORT_CS        	        GPIOB
#define W25QXX_PIN_CS					GPIO_PIN_12

#endif /* __DRV_W25QXX_H__ */
