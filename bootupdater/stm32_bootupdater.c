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

/* `make` in this directory is the F1 updater. `make f4` passes -DSTM32F4. */
#if !defined(STM32F4)
#define STM32F1
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>
#include <stdbool.h>
#include <stdint.h>
#include "hwdefs.h"

#define BOOTLOADER_MAGIC 0xAA

#if defined(STM32F4)
/* Sector 0 only. 16 KiB is 16 of the protocol's 1 KiB pages. */
#define PAGE_SIZE 1024
#define PAGE_WORDS (PAGE_SIZE / 4)
#define BOOT_FLASH_START 0x08000000u
#define BOOT_FLASH_END   0x08004000u
#define BOOT_MAX_PAGES   16u
#define APP_FLASH_START  BOOT_FLASH_START

#define DELAY_100 (((1u << 20) * 7u) / 3u)
#define DELAY_200 (((1u << 21) * 7u) / 3u)

#define FLASH_SR_ERR (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | \
                      FLASH_SR_WRPERR | FLASH_SR_OPERR)

_Static_assert(BOOT_FLASH_END - BOOT_FLASH_START == 16u * 1024u, "sector 0 is 16 KiB");
_Static_assert(BOOT_MAX_PAGES * PAGE_SIZE == BOOT_FLASH_END - BOOT_FLASH_START,
               "page limit must fill sector 0 and nothing past it");

#ifndef HSE_HZ
#define HSE_HZ 8000000
#endif
#else
#define PAGE_SIZE 1024
#define PAGE_WORDS (PAGE_SIZE / 4)
#define APP_FLASH_START 0x08000000
#define DELAY_100 (1 << 20)
#define DELAY_200 (1 << 21)
#endif

#if defined(STM32F4)
static void clear_flash_errors(void)
{
   flash_clear_status_flags();
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
#error "F407 boot updater needs HSE_HZ of 8000000 or 25000000"
#endif

   rcc_osc_on(RCC_LSI);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_USART3);
   rcc_periph_clock_enable(RCC_DMA1);
   rcc_periph_clock_enable(RCC_CRC);

   rcc_wait_for_osc_ready(RCC_LSI);
   iwdg_set_period_ms(2000);
   iwdg_start();
}

static void usart_setup(void)
{
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

static void dma_wait_idle(void)
{
   dma_disable_stream(DMA1, USART_DMA_CHAN);
   while (DMA_SCR(DMA1, USART_DMA_CHAN) & DMA_SxCR_EN)
      ;
}

/* len is a byte count, matching the F1 updater. */
static void dma_setup(void *data, uint32_t len)
{
   dma_wait_idle();
   DMA_SCR(DMA1, USART_DMA_CHAN) &= ~DMA_SxCR_CHSEL_MASK;
   dma_channel_select(DMA1, USART_DMA_CHAN, USART_DMA_REQ);
   dma_set_transfer_mode(DMA1, USART_DMA_CHAN, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
   dma_set_peripheral_address(DMA1, USART_DMA_CHAN, (uint32_t)&USART_DR(TERM_USART));
   dma_set_memory_address(DMA1, USART_DMA_CHAN, (uint32_t)data);
   dma_set_number_of_data(DMA1, USART_DMA_CHAN, (uint16_t)len);
   dma_set_peripheral_size(DMA1, USART_DMA_CHAN, DMA_SxCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, USART_DMA_CHAN, DMA_SxCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, USART_DMA_CHAN);
   dma_clear_interrupt_flags(DMA1, USART_DMA_CHAN,
                             DMA_TCIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
   dma_enable_stream(DMA1, USART_DMA_CHAN);
}

/* Program one 1 KiB page. The caller has already erased sector 0.
 * Nothing outside 0x08000000..0x08003FFF is written.
 */
static bool write_flash(uint32_t addr, uint32_t *pageBuffer)
{
   if (addr < BOOT_FLASH_START || (addr + PAGE_SIZE) > BOOT_FLASH_END)
      return false;

   for (uint32_t idx = 0; idx < PAGE_WORDS; idx++)
   {
      uint32_t dest = addr + idx * 4u;
      if (dest < BOOT_FLASH_START || dest >= BOOT_FLASH_END)
         return false;
      flash_program_word(dest, pageBuffer[idx]);
      if (!flash_operation_ok())
         return false;
   }
   return true;
}
#else
static void clock_setup(void)
{
   RCC_CLOCK_SETUP();

   rcc_osc_on(RCC_LSI);
   /* Enable all needed GPIOx clocks */
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);

   #ifdef HWCONFIG_OLIMEX
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
   #endif
   #ifdef HWCONFIG_OLIMEX_H107
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, AFIO_MAPR_USART2_REMAP);
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

static void dma_setup(void *data, uint32_t len)
{
   dma_disable_channel(DMA1, USART_DMA_CHAN);
   dma_set_peripheral_address(DMA1, USART_DMA_CHAN, (uint32_t)&USART3_DR);
   dma_set_memory_address(DMA1, USART_DMA_CHAN, (uint32_t)data);
   dma_set_number_of_data(DMA1, USART_DMA_CHAN, len);
   dma_set_peripheral_size(DMA1, USART_DMA_CHAN, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, USART_DMA_CHAN, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, USART_DMA_CHAN);
   dma_enable_channel(DMA1, USART_DMA_CHAN);
   dma_clear_interrupt_flags(DMA1, USART_DMA_CHAN, DMA_TCIF);
}

static void write_flash(uint32_t addr, uint32_t *pageBuffer)
{
   flash_erase_page(addr);

   for (int idx = 0; idx < PAGE_WORDS; idx++)
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
   uint32_t page_buffer[PAGE_WORDS];
   uint32_t addr = APP_FLASH_START;

#if defined(STM32F4)
   /* This image is linked at 0x08010000. Point VTOR there before any
    * later interrupt. The bootloader also sets VTOR before the jump.
    */
   SCB_VTOR = (uint32_t)&vector_table;
   uint8_t numPages = 0;
#else
   char numPages = 0;
#endif
   char magic = 0;

   clock_setup();
   usart_setup();
   dma_setup(page_buffer, PAGE_SIZE);

   do
   {
      do
      {
         usart_send_blocking(TERM_USART, '2');
         magic = usart_recv(TERM_USART);
         iwdg_reset();
         wait();
      } while (magic != BOOTLOADER_MAGIC);

      usart_send_blocking(TERM_USART, 'S');
      wait();
      numPages = usart_recv(TERM_USART);
#if defined(STM32F4)
   } while (numPages == 0 || numPages > BOOT_MAX_PAGES);
#else
   } while (numPages == 0 || numPages > 4);
#endif

   flash_unlock();

#if defined(STM32F4)
   /* One erase of sector 0, then program inside it. A 16 KiB erase
    * can outlast the 2 s watchdog, so open that window first.
    * Sectors 1-7 are not erased. A failed update of this sector
    * needs a debugger; do not "fix" it by erasing the VCU.
    */
   iwdg_set_period_ms(10000);
   iwdg_reset();
   clear_flash_errors();
   flash_erase_sector(0, FLASH_CR_PROGRAM_X32);
   if (!flash_operation_ok())
   {
      for (;;)
         ;
   }
#endif

   while (numPages > 0)
   {
      uint32_t recvCrc = 0;
      uint32_t timeOut = DELAY_200;

      crc_reset();
      dma_setup(page_buffer, PAGE_SIZE);
      usart_send_blocking(TERM_USART, 'P');

      while (!dma_get_interrupt_flag(DMA1, USART_DMA_CHAN, DMA_TCIF))
      {
         timeOut--;

         //When the buffer is not full after about 200ms
         //Request the entire page again
         if (0 == timeOut)
         {
            timeOut = DELAY_200;
            dma_setup(page_buffer, PAGE_SIZE);
            usart_send_blocking(TERM_USART, 'T');
         }
         iwdg_reset();
      }

      uint32_t crc = crc_calculate_block(page_buffer, PAGE_WORDS);

      dma_setup(&recvCrc, sizeof(recvCrc));
      usart_send_blocking(TERM_USART, 'C');
      while (!dma_get_interrupt_flag(DMA1, USART_DMA_CHAN, DMA_TCIF));

      if (crc == recvCrc)
      {
#if defined(STM32F4)
         if (!write_flash(addr, page_buffer))
         {
            usart_send_blocking(TERM_USART, 'E');
            for (;;)
               ;
         }
#else
         write_flash(addr, page_buffer);
#endif
         numPages--;
         addr += PAGE_SIZE;
      }
      else
      {
         usart_send_blocking(TERM_USART, 'E');
      }
   }

   flash_lock();
   usart_send_blocking(TERM_USART, 'D');
   char cmdBuffer[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
   uint8_t startIdx = 0;
   dma_setup(cmdBuffer, sizeof(cmdBuffer));

   while (cmdBuffer[startIdx + 1] != 'e' ||
          cmdBuffer[startIdx + 2] != 's' ||
          cmdBuffer[startIdx + 3] != 'e' ||
          cmdBuffer[startIdx + 4] != 't')
   {
      dma_setup(cmdBuffer, sizeof(cmdBuffer));
      iwdg_reset();
      wait();

      for (startIdx = 0; startIdx < (sizeof(cmdBuffer) - 5) && cmdBuffer[startIdx] != 'r'; startIdx++);
   }

   usart_disable(TERM_USART);
   scb_reset_system();
   return 0;
}
