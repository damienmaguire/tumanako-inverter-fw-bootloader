#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

#if defined(STM32F4)
/*
 * ZombieVerter VCU on STM32F407, LQFP100.
 *
 * The VCU terminal is USART3 at 115200. Its constructor is
 * Terminal(USART3, ..., remap=false). On F4 that is PB10 TX and
 * PB11 RX, alternate function 7, not an AFIO remap. RX DMA is
 * DMA1 stream 1, request channel 4. TX is not used with DMA here.
 *
 * The F1 header names TERM_USART_TXPORT as GPIOB and TERM_USART_TXPIN
 * as GPIO_USART3_PR_TX. Those look inconsistent, but the "PR" macro is
 * only the pin bit (GPIO10). The live firmware does not turn the
 * partial remap on, so the port really is B. PB10/PB11 is the pair
 * that talks to the same connector as the VCU firmware.
 *
 * The crystal on the new board has not been confirmed. HSE_HZ defaults
 * to 8 MHz, which is what the F107 firmware assumes and what the VCU
 * uses unless VCU_HSE_HZ=25000000. Both profiles produce HCLK 168 MHz,
 * APB1 42 MHz and APB2 84 MHz. The 180 MHz profiles are not used.
 * libopencm3's table is the 3.3 V one (voltage scale and 5 wait states).
 *
 * Pin definitions live at a fixed sector. Define both symbols before
 * stm32_loader.h is included. Do not count back from the end of flash:
 * on F4 that can land in the same large sector as something else.
 */
#define TERM_USART        USART3
#define TERM_USART_TXPORT GPIOB
#define TERM_USART_TXPIN  GPIO10
#define TERM_USART_RXPIN  GPIO11
#define TERM_USART_AF     GPIO_AF7
#define USART_DMA_CHAN    DMA_STREAM1
#define USART_DMA_REQ     DMA_SxCR_CHSEL_4

#define PINDEF_ADDRESS    0x0800C000u
#define PINDEF_SECTOR     3u

#else /* STM32F1, unchanged board selection */

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

#endif /* STM32F4 */

#define USART_BAUDRATE  115200

#endif // HWDEFS_H_INCLUDED
