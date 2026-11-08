/*
 * STM32F407 bootloader, sector 0 only.
 *
 * This is the reservation already compiled into the ZombieVerter F407
 * image (stm32_vcu_f407.ld / include/hwdefs.h):
 *
 *   0x08000000  16 KiB  sector 0   this bootloader
 *   0x08004000  16 KiB  sector 1   VCU parameters
 *   0x08008000  16 KiB  sector 2   VCU CAN map
 *   0x0800C000  16 KiB  sector 3   pin definitions
 *   0x08010000  448 KiB sectors 4-7  VCU application
 *
 * RAM is the 128 KiB SRAM at 0x20000000. The 64 KiB CCM at 0x10000000
 * is left out on purpose: the DMA controller cannot read or write it,
 * and the USART receive buffer lives in ordinary RAM.
 *
 * Sectors 8-11 exist only on the 1 MiB VGT6. This image does not use
 * them, so the same bootloader fits a 512 KiB VET6.
 */

MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

INCLUDE cortex-m-generic.ld

/* .data is copied out of the image, so count it as well as .text. */
ASSERT(_etext + (_edata - _data) <= ORIGIN(rom) + LENGTH(rom),
       "F407 bootloader must fit in sector 0 (16 KiB at 0x08000000)")
