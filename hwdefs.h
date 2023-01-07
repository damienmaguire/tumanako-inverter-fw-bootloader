#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

//#define HWCONFIG_TUMANAKO_KIWIAC
//#define HWCONFIG_OLIMEX_H107
//#define HWCONFIG_OLIMEX
#define HWCONFIG_LOGGER

#ifdef HWCONFIG_TUMANAKO_KIWIAC
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_16mhz_out_72mhz

#define TERM_USART USART1
#define TERM_USART_TXPIN GPIO_USART1_TX
#define TERM_USART_TXPORT GPIOA
#endif

#ifdef HWCONFIG_OLIMEX
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define TERM_USART USART3
#define TERM_USART_TXPIN GPIO_USART3_TX
#define TERM_USART_TXPORT GPIOB
#define USART_DMA_CHAN 3
#endif

#ifdef HWCONFIG_LOGGER
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define TERM_USART USART2
#define TERM_USART_TXPIN GPIO_USART2_TX
#define TERM_USART_TXPORT GPIOA
#define USART_DMA_CHAN 6
#endif

#ifdef HWCONFIG_OLIMEX_H107
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_25mhz_out_72mhz

#define TERM_USART         USART3
#define TERM_USART_TXPIN   GPIO_USART3_FR_TX
#define TERM_USART_TXPORT  GPIOD
#define USART_DMA_CHAN 3
#endif

#define USART_BAUDRATE  115200

#endif // HWDEFS_H_INCLUDED
