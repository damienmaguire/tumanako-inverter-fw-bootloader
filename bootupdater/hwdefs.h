#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

#if defined(STM32F4)
/*
 * Same USART3 pins as the F407 bootloader and the VCU terminal:
 * PB10 TX, PB11 RX, AF7, DMA1 stream 1 request channel 4.
 * HSE_HZ defaults to 8 MHz. 25000000 is the other profile that
 * still produces 168 / 42 / 84 MHz. 180 MHz is not used.
 */
#define TERM_USART        USART3
#define TERM_USART_TXPORT GPIOB
#define TERM_USART_TXPIN  GPIO10
#define TERM_USART_RXPIN  GPIO11
#define TERM_USART_AF     GPIO_AF7
#define USART_DMA_CHAN    DMA_STREAM1
#define USART_DMA_REQ     DMA_SxCR_CHSEL_4

#else

//#define HWCONFIG_TUMANAKO_KIWIAC
//#define HWCONFIG_OLIMEX
#define HWCONFIG_OLIMEX

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

#ifdef HWCONFIG_OLIMEX_H107
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_25mhz_out_72mhz


#define TERM_USART         USART2
#define TERM_USART_TXPIN   GPIO_USART2_RE_TX
#define TERM_USART_TXPORT  GPIOD
#endif

#endif /* STM32F4 */

#define USART_BAUDRATE  115200

#endif // HWDEFS_H_INCLUDED
