/**
 * @file boards/subarue-eg33/mcuconf.h
 *
 * @brief In this header we can override mcuconf.h.
 *
 * @date Feb 06, 2021
 * @author Andrey Gusakov, 2021
 */
#ifndef _MCUCONF_SUBARUEG33_H_
#define _MCUCONF_SUBARUEG33_H_

#include "../../../hw_layer/ports/stm32/stm32f7/cfg/mcuconf.h"

/* clocks adjust for 25 MHz ocs */
#undef STM32_PLLM_VALUE
#define STM32_PLLM_VALUE				25

//#undef CORTEX_ENABLE_WFI_IDLE
#define CORTEX_ENABLE_WFI_IDLE			FALSE

//#undef STM32_LSE_ENABLED
//#define STM32_LSE_ENABLED FALSE

#undef STM32_RTCSEL
#define STM32_RTCSEL					STM32_RTCSEL_HSEDIV // STM32_RTCSEL_LSI

/* serials and uarts */ 
#undef STM32_SERIAL_USE_USART1
#undef STM32_UART_USE_USART1
//#if TS_UART_DMA_MODE
//#define STM32_SERIAL_USE_UART1		FALSE
//#else
#define STM32_SERIAL_USE_USART1			TRUE
#define STM32_UART_USE_USART1			FALSE
//#endif

/* enable serial driver for USART2 to make usbconsole.c happy:
 * as '#define USB_SERIAL_DRIVER SD2' in usbconsole.h and
 * sdStart(&USB_SERIAL_DRIVER, NULL); is called unconditionaly */
#undef STM32_SERIAL_USE_USART2
#undef STM32_UART_USE_USART2
#define STM32_SERIAL_USE_USART2			TRUE
#define STM32_UART_USE_USART2			FALSE

#undef STM32_SERIAL_USE_USART3
#undef STM32_UART_USE_USART3
#define STM32_SERIAL_USE_USART3			FALSE
#define STM32_UART_USE_USART3			FALSE

#undef STM32_UART_USE_USART3
#define STM32_UART_USE_USART3			FALSE

#undef STM32_UART_USE_UART4
#define STM32_UART_USE_UART4			FALSE

#undef STM32_USB_USE_OTG1
#define STM32_USB_USE_OTG1				TRUE

#undef STM32_USB_USE_USB1
#define STM32_USB_USE_USB1				TRUE

#undef STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1				FALSE

#undef STM32_SPI_USE_SPI2
#define STM32_SPI_USE_SPI2				FALSE

#undef STM32_SPI_USE_SPI4
#define STM32_SPI_USE_SPI4				TRUE

#undef STM32_SPI_USE_SPI5
#define STM32_SPI_USE_SPI5				TRUE

#undef STM32_ADC_USE_ADC3
#define STM32_ADC_USE_ADC3				TRUE

/* default STM32_DMA_STREAM_ID(2, 4) used by ADC1 */
#undef STM32_SPI_SPI5_TX_DMA_STREAM
#define STM32_SPI_SPI5_TX_DMA_STREAM	STM32_DMA_STREAM_ID(2, 6)

/* To remove futher possible conflict */
#undef STM32_SPI_SPI6_RX_DMA_STREAM

#undef STM32_ICU_USE_TIM3
#define STM32_ICU_USE_TIM3				TRUE

#undef STM32_CAN_USE_CAN2
#define STM32_CAN_USE_CAN2				FALSE

//#undef STM32_CAN_CAN1_IRQ_PRIORITY
//#define STM32_CAN_CAN1_IRQ_PRIORITY 4

#endif /* _MCUCONF_SUBARUEG33_H_ */
