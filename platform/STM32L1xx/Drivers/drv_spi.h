#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include <rtdevice.h>

#include "stm32l1xx_spi.h"
#include "board.h"

#ifdef RT_USING_SPI

/* RT_USING_SPI1 is not set */
#define RT_USING_SPI2
/* RT_USING_SPI3 is not set */

/* SPI_USING_DMA is not set */
/* SPI1_USING_DMA is not set */
/* SPI2_USING_DMA is not set */
/* SPI3_USING_DMA is not set */

/* SPI1 */
#define SPI1_SCK_RCC					          RCC_AHBPeriph_GPIOA
#define SPI1_SCK_PORT       	          GPIOA
#define SPI1_SCK_PIN					          GPIO_Pin_5
#define SPI1_SCK_AFIO                   GPIO_PinSource5

#define SPI1_MISO_RCC					          RCC_AHBPeriph_GPIOA
#define SPI1_MISO_PORT                  GPIOA
#define SPI1_MISO_PIN					          GPIO_Pin_6
#define SPI1_MISO_AFIO                  GPIO_PinSource6

#define SPI1_MOSI_RCC					          RCC_AHBPeriph_GPIOA
#define SPI1_MOSI_PORT                  GPIOA
#define SPI1_MOSI_PIN				            GPIO_Pin_7
#define SPI1_MOSI_AFIO                  GPIO_PinSource7

/* SPI2 */
#define SPI2_SCK_RCC					          RCC_AHBPeriph_GPIOB
#define SPI2_SCK_PORT        	          GPIOB
#define SPI2_SCK_PIN					          GPIO_Pin_13
#define SPI2_SCK_AFIO                   GPIO_PinSource13

#define SPI2_MISO_RCC					          RCC_AHBPeriph_GPIOB
#define SPI2_MISO_PORT                  GPIOB
#define SPI2_MISO_PIN					          GPIO_Pin_14
#define SPI2_MISO_AFIO                  GPIO_PinSource14

#define SPI2_MOSI_RCC					          RCC_AHBPeriph_GPIOB
#define SPI2_MOSI_PORT                  GPIOB
#define SPI2_MOSI_PIN					          GPIO_Pin_15
#define SPI2_MOSI_AFIO                  GPIO_PinSource15

/* SPI3 */
#define SPI3_SCK_RCC					          RCC_AHBPeriph_GPIOA
#define SPI3_SCK_PORT        	          GPIOA
#define SPI3_SCK_PIN					          GPIO_Pin_0
#define SPI3_SCK_AFIO                   GPIO_PinSource0

#define SPI3_MISO_RCC					          RCC_AHBPeriph_GPIOA
#define SPI3_MISO_PORT                  GPIOA
#define SPI3_MISO_PIN					          GPIO_Pin_0
#define SPI3_MISO_AFIO                  GPIO_PinSource0

#define SPI3_MOSI_RCC					          RCC_AHBPeriph_GPIOA
#define SPI3_MOSI_PORT                  GPIOA
#define SPI3_MOSI_PIN					          GPIO_Pin_0
#define SPI3_MOSI_AFIO                  GPIO_PinSource0

#ifdef SPI_USING_DMA

#define SPI_DMA_RX_DONE    0x01
#define SPI_DMA_TX_DONE    0x02
#define SPI_DMA_COMPLETE   (SPI_DMA_RX_DONE | SPI_DMA_TX_DONE)

struct stm32_spi_dma_private
{
    DMA_Channel_TypeDef * DMA_Channel_TX;
    DMA_Channel_TypeDef * DMA_Channel_RX;
    uint32_t DMA_Channel_TX_FLAG_TC;
    uint32_t DMA_Channel_TX_FLAG_TE;
    uint32_t DMA_Channel_RX_FLAG_TC;
    uint32_t DMA_Channel_RX_FLAG_TE;
    uint8_t tx_irq_ch;
    uint8_t rx_irq_ch;
    uint32_t tx_gl_flag;
    uint32_t rx_gl_flag;
};
struct stm32_spi_dma
{
    const struct stm32_spi_dma_private *priv_data;
    struct rt_event event;
};
#endif /*SPI_USING_DMA*/
struct stm32_spi_bus
{
    struct rt_spi_bus bus;
    SPI_TypeDef * SPI;
#ifdef SPI_USING_DMA
    struct stm32_spi_dma *dma;
#endif /* SPI_USING_DMA */
};

struct stm32_spi_cs
{
    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_Pin;
};

#endif /* RT_USING_SPI */                          
#endif /* __DRV_SPI_H__ */
