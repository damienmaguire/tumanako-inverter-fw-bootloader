--------------------------------------------
-- STM32 Bootloader
--
-- Johannes Hübner <dev@johanneshuebner.com>
--------------------------------------------

This boot loader does not interact with the built-in STM32 boot loader. It
uses a serial port of your choice. The interface flexibility and the
independence from the BOOT pins are the main reasons for implementing it.

`make` is the original STM32F1 bootloader. `make f4` is the STM32F407
bootloader for Damien Maguire's ZombieVerter VCU (BMW-F11-Dev, image
stm32_vcu_f407). The version-2 update protocol is the same for both, so
updater.py and the usual OpenInverter tools still apply.

New update protocol
- 115200-8-N-2 (2 stop bits necessary when using a ZigBee module)

1 Send '2' indicating version 2 bootloader, wait about 500ms for magic byte 0xAA
2 If no reply goto 7
3 Send an 'S' indicating that it is awaiting an update size in pages
4 If no reply within about 500ms go to step 7
4.1 otherwise send a 'P' indicating that it is awaiting the actual page
as a binary image
4.2 When page not received within about 1s, print 'T' and keep waiting
5 When page received send a 'C' indicating that it is awaiting the pages
checksum
6 When checksum is correct and more pages need to be received, go to
step 4.1
6.1 if all pages have been received go to step 7
6.2 When checksum isn't correct print an 'E' then go to step 4.1
7 When done print a 'D' and start main firmware

Notes:
- By checksum I mean the one calculated by the STMs integrated CRC32 unit.
- A page on the wire is 1024 bytes. The bootloader still gathers two of
  those before a 2048 byte program, as before.
- The page count is one byte. updater.py therefore cannot send an image
  larger than 255 KiB. The F407 application region is 448 KiB; a VCU
  binary bigger than 255 KiB needs a protocol change before this
  bootloader can carry it.
- The actual firmware has a reset command to cycle through the bootloader.
- F1: the application is linked at 0x08001000. The bootloader occupies
  0x08000000 and must fit in 4 KiB. It erases 1 KiB or 2 KiB pages.
- F407: see the flash map below. Do not link an F407 application at
  the F1 address.

--------------------------------------------
-- STM32F407 / ZombieVerter VCU
--------------------------------------------

Flash map, matching include/hwdefs.h and stm32_vcu_f407.ld on
damienmaguire/Stm32-vcu branch BMW-F11-Dev. The same image is valid on
the 512 KiB VET6 and the 1 MiB VGT6. Sectors 8-11 are VG-only and are
not used.

  0x08000000  16 KiB  sector 0   this bootloader
  0x08004000  16 KiB  sector 1   VCU parameters
  0x08008000  16 KiB  sector 2   VCU CAN map
  0x0800C000  16 KiB  sector 3   pin definitions
  0x08010000  448 KiB sectors 4-7  VCU application, through 0x0807FFFF

An application update erases only sectors 4, 5, 6 and 7, and only the
sectors the image actually reaches. It never erases sector 0 (that
would destroy this bootloader) or sectors 1-3 (a VCU save erases a
whole sector, so one wrong erase wipes a store). Pin definitions are
read from 0x0800C000, sector 3. They are not computed from the end of
flash.

Clock is 168 MHz HCLK, 42 MHz APB1, 84 MHz APB2, from libopencm3's
3.3 V profile. The default crystal is 8 MHz, which is what the F107
firmware assumes and what the VCU uses unless VCU_HSE_HZ=25000000.
The crystal on the new board has not been measured. If it is 25 MHz:

  make clean
  make f4 HSE_HZ=25000000

Both profiles produce the same bus clocks. 180 MHz is not used.
`make` will not rebuild objects when only HSE_HZ changes, so clean
first. The boot updater needs the same HSE_HZ as the bootloader.

Serial is USART3 at 115200 8N2, the VCU terminal. The VCU constructs
Terminal(USART3, ..., remap=false), which on F4 is PB10 TX and PB11 RX,
alternate function 7. RX DMA is DMA1 stream 1, request channel 4.
The old F1 macro GPIO_USART3_PR_TX is only the pin bit GPIO10; the
live firmware does not apply the partial remap, so the port is B.
That is the pair that talks to the same connector as the VCU.

After an update, or when the magic byte is not 0xAA, the bootloader
disables interrupts, stops USART3 and its DMA, sets VTOR to
0x08010000, loads MSP from the vector table there, and branches to
the application's reset handler. The F1 path still jumps without
writing MSP.

The first copy of this bootloader has to be programmed with a
debugger, at 0x08000000 only. There is nothing in a blank part that
can receive it, and flash.sh / the OpenOCD recipe in the Makefile are
the old F1 board file. Do not use them on an F407. After this
bootloader is in sector 0, bootupdater/ can replace a later bootloader
without a debugger: it is loaded like an application at 0x08010000
and then writes only sector 0 (16 KiB, 16 pages). It does not erase
the VCU stores or the application. A failed bootloader update leaves
sector 0 blank and needs the debugger again.

RAM is the 128 KiB SRAM at 0x20000000. CCM at 0x10000000 is not used,
because DMA cannot see it.

--------------------------------------------
-- STM32 Bootloader Updater
--
-- Johannes Hübner <dev@johanneshuebner.com>
--------------------------------------------

To be able to upgrade the boot loader code without a JTAG programmer an
upgrade application was developed. It follows the same protocol as the
boot loader but with different timing:

1. Keep printing "2" until magic and a legal page count is received
   (F1: 1..4 pages, F407: 1..16 pages)
2. Do the update process. F1 writes 1 KiB pages at 0x08000000.
   F407 erases sector 0 once and programs only inside 0x08000000..0x08003FFF.
3. Wait for "reset" command

To upgrade the bootloader, do the following, either using the web interface
or the python script. On F407 the three files are the F407 builds, and
the bootloader file must be the same HSE_HZ as the updater.

1. Send file stm32_bootupdater.bin
2. Send file stm32_loader.bin
3. Send the application (on F407, stm32_vcu_f407.bin from `make f4`
   in the VCU tree, linked at 0x08010000)

--------------------------------------------
-- Compiling
--------------------------------------------
You will need the arm-none-eabi toolchain:
https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external dependency is Johannes Huebner's libopencm3 fork.
Download and build the F1 and F4 libraries with:

make get-deps

F1 bootloader (4 KiB at 0x08000000, application at 0x08001000):

make
make -C bootupdater

F407 bootloader (16 KiB sector 0, application at 0x08010000):

make f4
make -C bootupdater f4

Outputs are stm32_loader.bin / .hex and stm32_loader_f407.bin / .hex,
and the matching files under bootupdater/. Nothing in this tree was
run on a chip.
