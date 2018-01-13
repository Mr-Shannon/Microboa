#ifndef __DRV_SPIRIT1_H__
#define __DRV_SPIRIT1_H__

#include "drv_spi.h"
#include "SPIRIT_Config.h"
#include "shell.h"

#define USING_SPIRIT1_DEBUG

#define TMP_BUFFER_LEN                     50

/* spirit1 CS */
#define SPIRIT1_CS_RCC					            RCC_AHBPeriph_GPIOB
#define SPIRIT1_CS_PORT       	            GPIOB
#define SPIRIT1_CS_PIN					            GPIO_Pin_12

/* spirit1 SDN */
#define SPIRIT1_SDN_RCC					            RCC_AHBPeriph_GPIOC
#define SPIRIT1_SDN_PORT        	          GPIOC
#define SPIRIT1_SDN_PIN					            GPIO_Pin_13

/* spirit1 GPIO0 */
#define SPIRIT1_GPIO0_RCC					          RCC_AHBPeriph_GPIOC
#define SPIRIT1_GPIO0_PORT        	        GPIOC
#define SPIRIT1_GPIO0_PIN					          GPIO_Pin_7
#define SPIRIT1_GPIO0_SOURCE_PIN            EXTI_PinSource7
#define SPIRIT1_GPIO0_SOURCE_PORT           EXTI_PortSourceGPIOC
#define SPIRIT1_GPIO0_LINE                  EXTI_Line7  
#define SPIRIT1_GPIO0_TRIGGER               EXTI_Trigger_Falling
#define SPIRIT1_GPIO0_IRQN                  EXTI9_5_IRQn

/* spirit1 GPIO1 */
#define SPIRIT1_GPIO1_RCC					          RCC_AHBPeriph_GPIOC
#define SPIRIT1_GPIO1_PORT        	        GPIOC
#define SPIRIT1_GPIO1_PIN					          GPIO_Pin_8
#define SPIRIT1_GPIO1_SOURCE_PIN            EXTI_PinSource8
#define SPIRIT1_GPIO1_SOURCE_PORT           EXTI_PortSourceGPIOC
#define SPIRIT1_GPIO1_LINE                  EXTI_Line8  
#define SPIRIT1_GPIO1_TRIGGER               EXTI_Trigger_Falling
#define SPIRIT1_GPIO1_IRQN                  EXTI9_5_IRQn

/* spirit1 GPIO2 */
#define SPIRIT1_GPIO2_RCC					          RCC_AHBPeriph_GPIOC
#define SPIRIT1_GPIO2_PORT        	        GPIOC
#define SPIRIT1_GPIO2_PIN				            GPIO_Pin_9
#define SPIRIT1_GPIO2_SOURCE_PIN            EXTI_PinSource9
#define SPIRIT1_GPIO2_SOURCE_PORT           EXTI_PortSourceGPIOC
#define SPIRIT1_GPIO2_LINE                  EXTI_Line9  
#define SPIRIT1_GPIO2_TRIGGER               EXTI_Trigger_Falling
#define SPIRIT1_GPIO2_IRQN                  EXTI9_5_IRQn

/* spirit1 GPIO3 */
#define SPIRIT1_GPIO3_RCC					          RCC_AHBPeriph_GPIOC
#define SPIRIT1_GPIO3_PORT        	        GPIOC
#define SPIRIT1_GPIO3_PIN					          GPIO_Pin_10
#define SPIRIT1_GPIO3_SOURCE_PIN            EXTI_PinSource10
#define SPIRIT1_GPIO3_SOURCE_PORT           EXTI_PortSourceGPIOC
#define SPIRIT1_GPIO3_LINE                  EXTI_Line10  
#define SPIRIT1_GPIO3_TRIGGER               EXTI_Trigger_Falling
#define SPIRIT1_GPIO3_IRQN                  EXTI15_10_IRQn


#define SPIRIT1_WRITE                       0x00
#define SPIRIT1_READ                        0x01
#define SPIRIT1_COMMAND                     0x80
#define SPIRIT1_FIFO_ADDR                   0xFF

typedef SpiritStatus StatusBytes;

typedef enum
{
  SPIRIT1_GPIO_PIN_0     = 0x00, 
  SPIRIT1_GPIO_PIN_1     = 0x01,
  SPIRIT1_GPIO_PIN_2     = 0x02, 
  SPIRIT1_GPIO_PIN_3     = 0x03,
  SPIRIT1_GPIO_PIN_SDN   = 0x04,  

}spirit1_gpio_pin_t;

typedef enum
{
  SPIRIT1_MODE_GPIO_IN  = 0x00,        
  SPIRIT1_MODE_GPIO_OUT = 0x01,
  SPIRIT1_MODE_EXTI_OUT = 0x02,          

}spirit1_gpio_mode_t;

struct spi_spirit1_device
{
    struct rt_spi_device *          spi_device;
    struct rt_mutex                 lock;
};

void spirit1_deinit(void);
void spirit1_init(void);
StatusBytes spirit1_write_reg(uint8_t addr, uint8_t len, uint8_t* buffer);
StatusBytes spirit1_read_reg(uint8_t addr, uint8_t len, uint8_t* buffer);
StatusBytes spirit1_command_strobe(uint8_t command);
StatusBytes spirit1_write_fifo(uint8_t len, uint8_t* buffer);
StatusBytes spirit1_read_fifo(uint8_t len, uint8_t* buffer);

void spirit1_enter_shutdown(void);
void spirit1_exit_shutdown(void);
SpiritFlagStatus spirit1_check_shutdown(void);



#endif
