/*
 * STM32F407 bootloader updater.
 *
 * The F407 bootloader loads this like any other application, at
 * 0x08010000, then jumps to it. This program's only flash write is
 * sector 0 (the bootloader, 16 KiB). It must not erase the VCU
 * stores in sectors 1-3 or the application sectors 4-7.
 *
 * RAM is the 128 KiB SRAM at 0x20000000. CCM is not used.
 */

MEMORY
{
	rom (rx)  : ORIGIN = 0x08010000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

INCLUDE cortex-m-generic.ld

ASSERT(_etext + (_edata - _data) <= 0x08020000,
       "F407 boot updater must stay inside sector 4")
