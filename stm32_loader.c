/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* `make` builds the F1 bootloader. `make f4` passes -DSTM32F4. */
#if !defined(STM32F4)
#define STM32F1
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/cortex.h>
#include <stdbool.h>
#include "hwdefs.h"
#include "stm32_loader.h"

#define FLASH_START 0x08000000
#define SMALLEST_PAGE_WORDS 256
#define PROGRAM_WORDS       512
#define BOOTLOADER_MAGIC 0xAA

#if defined(STM32F4)
/* Same map as the VCU image. The application is sectors 4-7.
 * FLASH_APP_END is one past the last byte, and it exists on both
 * the 512 KiB VET6 and the 1 MiB VGT6.
 */
#define APP_FLASH_START 0x08010000u
#define FLASH_APP_END   0x08080000u

/* F1 busy-loops were counted at 72 MHz. 168/72 = 7/3, which keeps the
 * same order of magnitude at the F407's 168 MHz HCLK. Not a measured
 * millisecond delay. Both allowed crystals produce that HCLK.
 */
#define DELAY_100 (((1u << 20) * 7u) / 3u)
#define DELAY_200 (((1u << 21) * 7u) / 3u)

/* A 128 KiB sector erase stalls the flash for up to about two seconds.
 * The VCU pets the watchdog every 100 ms once it is running, so this
 * wider window is only for the erase. It is put back to 2 s afterwards.
 */
#define ERASE_WATCHDOG_MS 10000u
#define RUN_WATCHDOG_MS   2000u

#define FLASH_SR_ERR (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | \
                      FLASH_SR_WRPERR | FLASH_SR_OPERR)

_Static_assert(PINDEF_ADDRESS == 0x0800C000u, "pin definitions must be at 0x0800C000");
_Static_assert(PINDEF_SECTOR == 3u, "pin definitions must be sector 3");
_Static_assert(PINDEF_SECTOR < 4u, "pindef sector must stay below the application");
_Static_assert(FLASH_APP_END - APP_FLASH_START == 448u * 1024u, "application is 448 KiB");
_Static_assert(APP_FLASH_START - FLASH_START == 64u * 1024u, "bootloader and stores are 64 KiB");

#ifndef HSE_HZ
#define HSE_HZ 8000000
#endif
#else
#define APP_FLASH_START 0x08001000
#define DELAY_100 (1 << 20)
#define DELAY_200 (1 << 21)
#endif

#if defined(STM32F4)
static void clear_flash_errors(void)
{
   flash_clear_status_flags();
   /* OPERR is write-1-to-clear and the library helper leaves it. */
   FLASH_SR = FLASH_SR_OPERR;
}

static bool flash_operation_ok(void)
{
   return (FLASH_SR & FLASH_SR_ERR) == 0u;
}

static void clock_setup(void)
{
#if HSE_HZ == 8000000
   rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#elif HSE_HZ == 25000000
   rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#else
#error "F407 bootloader needs HSE_HZ of 8000000 or 25000000"
#endif

   rcc_osc_on(RCC_LSI);

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_GPIOE);
   rcc_periph_clock_enable(RCC_USART3);
   rcc_periph_clock_enable(RCC_DMA1);
   rcc_periph_clock_enable(RCC_CRC);

   rcc_wait_for_osc_ready(RCC_LSI);
   iwdg_set_period_ms(RUN_WATCHDOG_MS);
   iwdg_start();
}

static void usart_setup(void)
{
   /* F4 does not connect the USART until the pin is in AF mode.
    * TX and RX are both AF7. Push-pull 50 MHz matches the VCU terminal.
    */
   gpio_set_af(TERM_USART_TXPORT, TERM_USART_AF, TERM_USART_TXPIN | TERM_USART_RXPIN);
   gpio_set_output_options(TERM_USART_TXPORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                           TERM_USART_TXPIN);
   gpio_mode_setup(TERM_USART_TXPORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                   TERM_USART_TXPIN | TERM_USART_RXPIN);

   usart_set_baudrate(TERM_USART, USART_BAUDRATE);
   usart_set_databits(TERM_USART, 8);
   usart_set_stopbits(TERM_USART, USART_STOPBITS_2);
   usart_set_mode(TERM_USART, USART_MODE_TX_RX);
   usart_set_parity(TERM_USART, USART_PARITY_NONE);
   usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);
   usart_enable_rx_dma(TERM_USART);
   usart_enable(TERM_USART);
}

/* The reference manual ignores stream writes until EN reads back as 0. */
static void dma_wait_idle(void)
{
   dma_disable_stream(DMA1, USART_DMA_CHAN);
   while (DMA_SCR(DMA1, USART_DMA_CHAN) & DMA_SxCR_EN)
      ;
}

/** @brief setup DMA for serial reception
 * @param data data buffer
 * @param len length of data buffer in 32-bit words
 */
static void dma_setup(void *data, uint32_t len)
{
   dma_wait_idle();

   /* Channel select ORs its bits in. Clear them so a repeated setup
    * cannot accumulate a different request channel.
    */
   DMA_SCR(DMA1, USART_DMA_CHAN) &= ~DMA_SxCR_CHSEL_MASK;
   dma_channel_select(DMA1, USART_DMA_CHAN, USART_DMA_REQ);
   dma_set_transfer_mode(DMA1, USART_DMA_CHAN, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
   dma_set_peripheral_address(DMA1, USART_DMA_CHAN, (uint32_t)&USART_DR(TERM_USART));
   dma_set_memory_address(DMA1, USART_DMA_CHAN, (uint32_t)data);
   dma_set_number_of_data(DMA1, USART_DMA_CHAN, (uint16_t)(len * 4u));
   dma_set_peripheral_size(DMA1, USART_DMA_CHAN, DMA_SxCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, USART_DMA_CHAN, DMA_SxCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, USART_DMA_CHAN);
   dma_clear_interrupt_flags(DMA1, USART_DMA_CHAN,
                             DMA_TCIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
   dma_enable_stream(DMA1, USART_DMA_CHAN);
}

static bool gpio_port_ok(uint32_t port)
{
   return port == GPIOA || port == GPIOB || port == GPIOC ||
          port == GPIOD || port == GPIOE;
}

static void initialize_pins(void)
{
   const struct pincommands *pincommands = (const struct pincommands *)PINDEF_ADDRESS;
   uint32_t crc = crc_calculate_block((uint32_t *)pincommands, PINDEF_NUMWORDS);

   if (crc != pincommands->crc)
      return;

   for (int idx = 0; idx < NUM_PIN_COMMANDS && pincommands->pindef[idx].port > 0; idx++)
   {
      uint32_t port = pincommands->pindef[idx].port;
      uint16_t pin = pincommands->pindef[idx].pin;
      uint8_t out = pincommands->pindef[idx].inout;
      uint8_t level = pincommands->pindef[idx].level;

      /* Port addresses are the F4 GPIO bases written by the VCU.
       * PA13/PA14 stay SWD so a pin block cannot take the debugger.
       */
      if (!gpio_port_ok(port) || pin == 0)
         continue;
      if (port == GPIOA && (pin & (GPIO13 | GPIO14)) != 0)
         continue;

      if (out)
      {
         /* Write the level while the pin is still an input, then
          * switch to an output, so a high pin does not glitch low.
          */
         if (level)
            gpio_set(port, pin);
         else
            gpio_clear(port, pin);
         gpio_set_output_options(port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, pin);
         gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
      }
      else
      {
         /* F1 selected pull-up or pull-down with the output register.
          * F4 stores that choice in PUPDR. level 1 is pull-up.
          */
         gpio_mode_setup(port, GPIO_MODE_INPUT,
                         level ? GPIO_PUPD_PULLUP : GPIO_PUPD_PULLDOWN, pin);
      }
   }
}

/* Sector that contains addr, or -1 if it is not application flash.
 * Sector 4 is 64 KiB. Sectors 5, 6 and 7 are 128 KiB each.
 */
static int app_sector(uint32_t addr)
{
   if (addr < APP_FLASH_START || addr >= FLASH_APP_END)
      return -1;
   if (addr < 0x08020000u)
      return 4;
   if (addr < 0x08040000u)
      return 5;
   if (addr < 0x08060000u)
      return 6;
   return 7;
}

static uint8_t sectors_erased;

static bool erase_app_sector(int sector)
{
   /* Sector 0 is this bootloader. Sectors 1-3 are the VCU stores.
    * Saving a store erases its whole sector, so one wrong erase
    * wipes parameters, the CAN map or the pin definitions.
    */
   if (sector < 4 || sector > 7 || sector <= (int)PINDEF_SECTOR)
      return false;

   uint8_t bit = (uint8_t)(1u << sector);
   if ((sectors_erased & bit) != 0u)
      return true;

   iwdg_reset();
   clear_flash_errors();
   /* x32 programming matches a 3.3 V supply. x64 would need Vpp. */
   flash_erase_sector((uint8_t)sector, FLASH_CR_PROGRAM_X32);
   if (!flash_operation_ok())
      return false;

   sectors_erased |= bit;
   iwdg_reset();
   return true;
}

static void clear_page_buffer(uint32_t *page_buffer)
{
   for (uint32_t i = 0; i < PROGRAM_WORDS; i++)
      page_buffer[i] = 0;
}

/* Program one 2 KiB transfer. Erase only the application sector that
 * contains it, and only the first time we write there.
 */
static bool write_flash(uint32_t addr, uint32_t *page_buffer)
{
   uint32_t nbytes = PROGRAM_WORDS * 4u;

   if (addr < APP_FLASH_START || (addr + nbytes) > FLASH_APP_END)
      return false;

   int first = app_sector(addr);
   int last = app_sector(addr + nbytes - 1u);
   if (first < 0 || last < 0)
      return false;
   if (!erase_app_sector(first) || !erase_app_sector(last))
      return false;

   for (uint32_t idx = 0; idx < PROGRAM_WORDS; idx++)
   {
      uint32_t dest = addr + idx * 4u;
      if (dest < APP_FLASH_START || dest >= FLASH_APP_END)
         return false;
      flash_program_word(dest, page_buffer[idx]);
      if (!flash_operation_ok())
         return false;
   }
   return true;
}

/* The F1 path jumps without writing MSP. On F4, load the application's
 * initial stack pointer first. libopencm3's reset handler does not
 * reload MSP itself; it expects the CPU to have done that at reset.
 */
static void start_application(void)
{
   uint32_t stack = *(volatile uint32_t *)APP_FLASH_START;
   uint32_t reset = *(volatile uint32_t *)(APP_FLASH_START + 4u);
   uint32_t entry = reset & ~1u;

   cm_disable_interrupts();
   usart_disable(TERM_USART);
   dma_wait_idle();

   bool image_ok = (stack >= 0x20000000u) && (stack <= 0x20020000u) &&
                    ((stack & 7u) == 0u) &&
                    ((reset & 1u) != 0u) &&
                    (entry >= APP_FLASH_START) && (entry < FLASH_APP_END);

   if (!image_ok)
   {
      /* Blank or nonsense vector table. The watchdog resets us. */
      for (;;)
         ;
   }

   /* Drop inherited USART and DMA state before the application starts. */
   rcc_periph_reset_pulse(RST_USART3);
   rcc_periph_reset_pulse(RST_DMA1);

   SCB_VTOR = APP_FLASH_START;
   __asm volatile("dsb" ::: "memory");
   __asm volatile("isb" ::: "memory");
   __asm volatile("msr msp, %0\n\tbx %1" :: "r"(stack), "r"(reset) : "memory");

   for (;;)
      ;
}
#else
static void clock_setup(void)
{
   RCC_CLOCK_SETUP();

   rcc_osc_on(RCC_LSI);
   /* Enable all needed GPIOx clocks */
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);

   #ifdef HWCONFIG_OLIMEX
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
   #endif
   #ifdef HWCONFIG_LOGGER
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
   #endif
   #ifdef HWCONFIG_OLIMEX_H107
   rcc_periph_clock_enable(RCC_AFIO);
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_USART3_REMAP_FULL_REMAP);
   #endif
   #ifdef HWCONFIG_TUMANAKO_KIWIAC
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
   #endif

   /* Enable DMA1 clock */
   rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

   /* Enable CRC clock */
   rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_CRCEN);

   rcc_wait_for_osc_ready(RCC_LSI);
   iwdg_set_period_ms(2000);
   iwdg_start();
}

static void usart_setup(void)
{
    gpio_set_mode(TERM_USART_TXPORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);

    /* Setup UART parameters. */
    usart_set_baudrate(TERM_USART, USART_BAUDRATE);
    usart_set_databits(TERM_USART, 8);
    usart_set_stopbits(TERM_USART, USART_STOPBITS_2);
    usart_set_mode(TERM_USART, USART_MODE_TX_RX);
    usart_set_parity(TERM_USART, USART_PARITY_NONE);
    usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);
    usart_enable_rx_dma(TERM_USART);

    /* Finally enable the USART. */
    usart_enable(TERM_USART);
}

/** @brief setup DMA for serial reception
 * @param data data buffer
 * @param len length of data buffer in 32-bit words
 */
static void dma_setup(void *data, uint32_t len)
{
   dma_disable_channel(DMA1, USART_DMA_CHAN);
   dma_set_peripheral_address(DMA1, USART_DMA_CHAN, (uint32_t)&USART2_DR);
   dma_set_memory_address(DMA1, USART_DMA_CHAN, (uint32_t)data);
   dma_set_number_of_data(DMA1, USART_DMA_CHAN, len * 4);
   dma_set_peripheral_size(DMA1, USART_DMA_CHAN, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, USART_DMA_CHAN, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, USART_DMA_CHAN);
   dma_enable_channel(DMA1, USART_DMA_CHAN);
   dma_clear_interrupt_flags(DMA1, USART_DMA_CHAN, DMA_TCIF);
}

static void initialize_pins()
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* pincommands = (struct pincommands*)pindefAddr;

   uint32_t crc = crc_calculate_block(((uint32_t*)pincommands), PINDEF_NUMWORDS);

   if (crc == pincommands->crc)
   {
      for (int idx = 0; idx < NUM_PIN_COMMANDS && pincommands->pindef[idx].port > 0; idx++)
      {
         uint8_t cnf = pincommands->pindef[idx].inout ? GPIO_CNF_OUTPUT_PUSHPULL : GPIO_CNF_INPUT_PULL_UPDOWN;
         uint8_t mode = pincommands->pindef[idx].inout ? GPIO_MODE_OUTPUT_50_MHZ : GPIO_MODE_INPUT;
         gpio_set_mode(pincommands->pindef[idx].port, mode, cnf, pincommands->pindef[idx].pin);

         if (pincommands->pindef[idx].level)
         {
            gpio_set(pincommands->pindef[idx].port, pincommands->pindef[idx].pin);
         }
      }
   }
}

//Check 1k of flash whether it contains only 0xFF = erased
static bool check_erased(uint32_t* baseAddress)
{
   uint32_t check = 0xFFFFFFFF;

   for (int i = 0; i < SMALLEST_PAGE_WORDS; i++, baseAddress++)
      check &= *baseAddress;

   return check == 0xFFFFFFFF;
}

//We always write 2kb pages. After erasing the possible first page we check the
//data content of the possible second page. If it is not erased, it will be.
static void write_flash(uint32_t addr, uint32_t *pageBuffer)
{
   flash_erase_page(addr);

   if (!check_erased(((uint32_t*)addr) + SMALLEST_PAGE_WORDS))
      flash_erase_page(addr + SMALLEST_PAGE_WORDS * 4);

   for (uint32_t idx = 0; idx < PROGRAM_WORDS; idx++)
   {
      flash_program_word(addr + idx * 4, pageBuffer[idx]);
   }
}
#endif

void wait(void)
{
   for (volatile uint32_t i = DELAY_100; i > 0; i--);
}

int main(void)
{
   const uint32_t receiveWords = SMALLEST_PAGE_WORDS;
   uint32_t page_buffer[PROGRAM_WORDS];
   uint32_t addr = APP_FLASH_START;
   uint32_t bufferOffset = 0;

#if defined(STM32F4)
   /* Padding past a short final page is zeros, not leftover stack. */
   clear_page_buffer(page_buffer);
#endif

   clock_setup();
   initialize_pins();
   usart_setup();
   dma_setup(page_buffer, receiveWords);

   wait();
   usart_send_blocking(TERM_USART, '2');
   wait();
   char magic = usart_recv(TERM_USART);

   if (magic == BOOTLOADER_MAGIC)
   {
      usart_send_blocking(TERM_USART, 'S');
      wait();
#if defined(STM32F4)
      /* The version-2 count is one byte. Read it as unsigned so
       * 128..255 pages (up to 255 KiB) are not treated as negative.
       * updater.py sends that same single byte.
       */
      uint8_t numPages = (uint8_t)usart_recv(TERM_USART);
      bool updating = numPages > 0;
      if (updating)
      {
         iwdg_set_period_ms(ERASE_WATCHDOG_MS);
         iwdg_reset();
         flash_unlock();
      }
#else
      char numPages = usart_recv(TERM_USART);
      flash_unlock();
#endif

      while (numPages > 0)
      {
         uint32_t recvCrc = 0;
         uint32_t timeOut = DELAY_200;

         crc_reset();
         dma_setup(page_buffer + bufferOffset, receiveWords);
         usart_send_blocking(TERM_USART, 'P');

         while (!dma_get_interrupt_flag(DMA1, USART_DMA_CHAN, DMA_TCIF))
         {
            timeOut--;

            //When the buffer is not full after about 200ms
            //Request the entire page again
            if (0 == timeOut)
            {
               timeOut = DELAY_200;
               dma_setup(page_buffer + bufferOffset, receiveWords);
               usart_send_blocking(TERM_USART, 'T');
            }
            iwdg_reset();
         }

         uint32_t crc = crc_calculate_block(page_buffer + bufferOffset, receiveWords);

         dma_setup(&recvCrc, 1);
         usart_send_blocking(TERM_USART, 'C');
         while (!dma_get_interrupt_flag(DMA1, USART_DMA_CHAN, DMA_TCIF));

         if (crc == recvCrc)
         {
            /* Write to flash when we have sufficient amount of data or last page was received */
            if (bufferOffset == receiveWords || numPages == 1)
            {
#if defined(STM32F4)
               if (!write_flash(addr, page_buffer))
               {
                  /* Do not erase or program anything further. */
                  usart_send_blocking(TERM_USART, 'E');
                  for (;;)
                     ;
               }
               clear_page_buffer(page_buffer);
#else
               write_flash(addr, page_buffer);
#endif
               addr += sizeof(page_buffer);
               bufferOffset = 0;
            }
            else
            {
               bufferOffset += receiveWords;
            }

            numPages--;
         }
         else
         {
            usart_send_blocking(TERM_USART, 'E');
         }
      }

#if defined(STM32F4)
      if (updating)
      {
         flash_lock();
         iwdg_set_period_ms(RUN_WATCHDOG_MS);
         iwdg_reset();
      }
#else
      flash_lock();
#endif
   }

   usart_send_blocking(TERM_USART, 'D');
   wait();

#if defined(STM32F4)
   start_application();
#else
   usart_disable(TERM_USART);

   void (*app_main)(void) = (void (*)(void)) *(volatile uint32_t*)(APP_FLASH_START + 4);
   SCB_VTOR = APP_FLASH_START;
   app_main();
#endif

   return 0;
}
