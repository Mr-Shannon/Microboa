#ifndef __ENC28J60_H__
#define __ENC28J60_H__

#include "drv_spi.h"
#include "enc28j60.h"

/* CS */
#define ENC28J60_CS_RCC					      RCC_APB2PERIPH_GPIOA
#define ENC28J60_CS_PORT       	      GPIOA
#define ENC28J60_CS_PIN					      GPIO_PIN_4

/* INT */
#define ENC28J60_INT_RCC				      RCC_APB2PERIPH_GPIOB
#define ENC28J60_INT_PORT        	    GPIOB
#define ENC28J60_INT_PIN			        GPIO_PIN_0
#define ENC28J60_INT_IRQ              EXTI0_IRQn
#define ENC28J60_INT_LINE             EXTI_LINE0             

#endif /* __ENC28J60_H__ */
