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

#define STM32F1

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/scb.h>
#include "hwdefs.h"
#include "stm32_loader.h"

#define FLASH_START 0x08000000
#define SMALLEST_PAGE_WORDS 256
#define PROGRAM_WORDS       512
#define APP_FLASH_START 0x08001000
#define BOOTLOADER_MAGIC 0xAA
#define DELAY_100 (1 << 20)
#define DELAY_200 (1 << 21)

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
   dma_set_peripheral_address(DMA1, USART_DMA_CHAN, (uint32_t)&USART3_DR);
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
      char numPages = usart_recv(TERM_USART);
      flash_unlock();

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
               write_flash(addr, page_buffer);
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

      flash_lock();
   }

   usart_send_blocking(TERM_USART, 'D');
   wait();
   usart_disable(TERM_USART);

   void (*app_main)(void) = (void (*)(void)) *(volatile uint32_t*)(APP_FLASH_START + 4);
   SCB_VTOR = APP_FLASH_START;
   app_main();

   return 0;
}
