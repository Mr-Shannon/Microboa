#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "board.h"

#ifdef RT_USING_SERIAL

#define RT_USING_USART1 
/* RT_USING_USART2 is not set */
/* RT_USING_USART3 is not set */

#define USART_USING_DMA
#define USART1_USING_DMA
/* USART2_USING_DMA is not set */
/* USART3_USING_DMA is not set */

/* USART1 */
#define USART1_TX_RCC					          RCC_APB2PERIPH_GPIOA
#define USART1_TX_PORT                  GPIOA
#define USART1_TX_PIN					          GPIO_PIN_9

#define USART1_RX_RCC					          RCC_APB2PERIPH_GPIOA
#define USART1_RX_PORT       	          GPIOA
#define USART1_RX_PIN					          GPIO_PIN_10

/* USART2 */
#define USART2_TX_RCC					          RCC_APB2PERIPH_GPIOA
#define USART2_TX_PORT                  GPIOA
#define USART2_TX_PIN					          GPIO_PIN_2

#define USART2_RX_RCC					          RCC_APB2PERIPH_GPIOA
#define USART2_RX_PORT       	          GPIOA
#define USART2_RX_PIN					          GPIO_PIN_3

/* USART3_REMAP[1:0] = 00 */
#define USART3_TX_RCC					          RCC_APB2PERIPH_GPIOB
#define USART3_TX_PORT                  GPIOB
#define USART3_TX_PIN					          GPIO_PIN_10

#define USART3_RX_RCC					          RCC_APB2PERIPH_GPIOB
#define USART3_RX_PORT       	          GPIOB
#define USART3_RX_PIN					          GPIO_PIN_11

/**
 * GD32 usart driver.
 *
 */
struct gd32_usart
{
    USART_TypeDef *usart_device;
    IRQn_Type irq;
#ifdef USART_USING_DMA
    struct gd32_usart_dma 
    {
        /* dma channel */
        DMA_Channel_TypeDef *rx_ch;
        /* dma global flag */
        uint32_t rx_gl_flag;
        /* dma irq channel */
        uint8_t rx_irq_ch;
        /* setting receive len */
        rt_size_t setting_recv_len;
        /* last receive index */
        rt_size_t last_recv_index;
    } dma;
#endif /* USART_USING_DMA */		
};

#endif /* RT_USING_SERIAL */
#endif /* __DRV_USART_H__ */
