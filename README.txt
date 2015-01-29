
>>>>>>>>>>>>>>>>>>>>>>>  Altera MPL (Minimal Preloader)   <<<<<<<<<<<<<<<<<<

********
Summary:
********

    The MPL is a non-GPL Preloader. The MPL supports a subset of features currently
    supported by Altera's GPL-based Preloader. This version of MPL supports booting
    from either QSPI or SDMMC. Also, Set CSEL to 0.

    The MPL initializes the PLLs, reset signals, IOCSR, pinmuxing etc., based on
    preloader generator file settings. The MPL uses Altera HWLib drivers for a bulk
    of its functionality. It additionally uses Altera HWLib SoCAL folders for the
    memory map definitions and basic read write commands used throughout MPL.

    Either the ARMCC or GNU GCC for ARM compilers may be used. The MPL supports both
    Cyclone V SoC & Arria V SoC devices. The example commands shown below are for
    Cyclone V SoC & QSPI or SDMMC as boot source. Please modify to select appropriate
    file names.

    You must have SoC Embedded Design Suite (EDS) installed if building in Windows.

    In Windows, the project needs to be imported into DS-5 first.
    In Embedded Command Shell, all commands are executed under the workspace directory.
    (e.g. C:\DS-5 Workspace\Altera-SoCFPGA-HardwareLib-MPL)

    The MPL project can be built in 3 ways:
    (1) In Embedded Command Shell in Windows (e.g., C:\altera\14.0\embedded\Embedded_Command_Shell.bat).
    (2) In DS-5.
    (3) From Linux host machine.

    The serial port should be set to 115200 baud, 8N1.


*********************
Directory Structure:
*********************

  ./
     - Makefile
          -- This will compile with armcc
     - mpl.scat
          -- ARM linker script, which will link to OCRAM memory space 0xffff0000.
     - mpl_gcc.lds
          -- GCC linker script, which will link to OCRAM memory space 0xffff0000.
     - mpl_config.h
          -- Header file to include configuration options which are not part of the
             handoff files mainly for selecting fpga configuration options.
  ./core
     - board_init.c
          -- core chip initialization, plls, iocsr, resets etc.
     - mpl.c
          -- main mpl file which reads passdown boot source and jumps to appropriate
             source to load the next image (qspi, sdmmc)
     - startup.s or startup_gcc.s (based on compiler version)
          -- low level startup to initialize basic cortexA9 based on the MPL documentation
     - qspi_load.c/sdmmc_load.c
          -- flash reads and interprets header from secondary image, then reads and loads
             entire secondary image and jumps to it
     - alt_printf.c - this provides the functionality for taking print commands and
             outputting characters to a stream
     - semihost.c/mpl_log.c - this files provide the code to output characters to a stream

  ./core/altera_ip
          -- drivers for core Altera blocks within Cyclone5 for iocsr/pinmux programming
             and sdram init and calibration


***************
General Notes:
***************

   -> The MPL output file name varies based on options passed. For example,
         - mpl_C5_q-mkpimage.bin => generated for Cyclone V SoC & QSPI boot source
         - mpl_C5_s-mkpimage.bin => generated for Cyclone V SoC & SDMMC boot source
         - mpl_A5_q-mkpimage.bin => generated for Arria V SoC & QSPI boot source
         - mpl_A5_s-mkpimage.bin => generated for Arria V SoC & SDMMC boot source


********************************
Building and Booting From QSPI:
********************************

     (1) Ensure the preloader header file '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/build.h'
         must match the boot selection below. For QSPI the CONFIG_PRELOADER_BOOT_FROM_QSPI must
         be set to 1 in 'build.h'.

     (2) Ensure the preloader header file '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/sdram/sequencer_defines.h'
         is either configured for Arria V SoC (set ARRIAV to 1) or Cyclone V SoC (set CYCLONEV to 1).

     (3) Build mpl_A5_q-mkpimage.bin or mpl_C5_q-mkpimage.bin. In the following example
         instructions will be given for the C5 variation.

         (3.1) In DS-5, Clean Project; In Embedded Command Shell or Linux, type make clean.

         (3.2) set correct values in <config.mk>:

               BOOT_SOURCE ?= SDMMC ( SDMMC = Boot from SDMMC,   QSPI = Boot from QSPI    )
               DEVICE      ?= C5    ( C5    = build code for C5, A5   = build code for A5 )
               COMPILER    ?= GNU   ( GNU   = GCC toolchain,     ARM  = ARM toolchain     )

               HANDOFF_BASE      ?= [Root directory of FPGA project]
               HANDOFF_SDRAM_DIR ?= [Quartus generated files with SDRAM data directory]
               HANDOFF_DIR       ?= [Preloader generated files directory]

               Then in DS-5 click Build Project; in Embedded Command Shell or Linux, type make.

         The above gives you a suitable MPL image for the BootROM for booting from QSPI.

     (4) Build your application binary and process it with the mkimage tool.
         Please consult SoCEDS documentation for more information on this step. For the purposes of
         this example, application.img will denote the application image.

     (5) Open a SoCEDS Command shell.

         Type the following commands to flash images:

         (5.1) quartus_hps -c 1 -o P -a 0x0000 mpl_C5_q-mkpimage.bin

               Option -c 1 instructs the programmer to use USB-BlasterII port as programming port.
               If there are multiple USB-BlasterII connected to the computer, this option may need to be
               adjusted.

         (5.2) quartus_hps -c 1 -o P -a 0x60000 application.img
               --> Note: In handoff file, the CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE is set to 0x60000.

     (6) Adjust the BOOTSEL Jumpers to QSPI. This is as follows:
         -- BOOTSEL0: . [. .]
         -- BOOTSEL1: [. .] .
         -- BOOTSEL2: [. .] .

     (7) Power up the board.


********************************
Building and Booting From SDMMC:
********************************

     (1) Ensure the preloader header file '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/build.h'
         must match the boot selection below.  For SDMMC the CONFIG_PRELOADER_BOOT_FROM_SDMMC must
         be set to 1 in 'build.h'.

     (2) Ensure the preloader header file '[SoCFPGA]/software/preloader/uboot-socfpga/board/altera/socfpga/sdram/sequencer_defines.h'
         is either configured for Arria V SoC (set ARRIAV to 1) or Cyclone V SoC (set CYCLONEV to 1).

     (3) Build mpl_A5_s-mkpimage.bin or mpl_C5_s-mkpimage.bin. In the following example
         instructions will be given for the C5 variation.

         (3.1) In DS-5, Clean Project; In Embedded Command Shell or Linux, type make clean.

         (3.2) set correct values in <config.mk>:

               BOOT_SOURCE ?= QSPI  ( SDMMC = Boot from SDMMC,   QSPI = Boot from QSPI    )
               DEVICE      ?= C5    ( C5    = build code for C5, A5   = build code for A5 )
               COMPILER    ?= GNU   ( GNU   = GCC toolchain,     ARM  = ARM toolchain     )

               HANDOFF_BASE      ?= [Root directory of FPGA project]
               HANDOFF_SDRAM_DIR ?= [Quartus generated files with SDRAM data directory]
               HANDOFF_DIR       ?= [Preloader generated files directory]

               Then in DS-5 click Build Project; in Embedded Command Shell or Linux, type make.

         The above gives you a suitable MPL image for the BootROM for booting from SDMMC.

     (4) Build your application binary and process it with the mkimage tool to create its image.
         Please consult SoCEDS documentation for more information on this step. For the purposes of
         this example, application.img will denote the application image.

     (5) Open a SoCEDS Command Shell.

         Follow the following commands to flash images:

         (5.1) Plug in your SD card and determine which partition it is.

               Linux: Type "dmesg | tail":
                 [2736576.121602] sd 15:0:0:0: [sdb] Attached SCSI removable disk

               Windows: A new drive letter should be created

         (5.2) Flash the MPL image to sdb3 using dd command:

               Linux:
               >>> sudo alt-boot-disk-util -p mpl_C5_s-mkpimage.bin -a write -d /dev/sdb

               Windows: (assuming drive E was assigned)
               >>> alt-boot-disk-util.exe -p mpl_C5_s-mkpimage.bin -a write -d e

         (5.3) Flash the application.img also to sdb3 after the mpl_C5_s.bin

               Linux:
               >>> sudo alt-boot-disk-util -b application.img -a write -d /dev/sdb

               Windows: (assuming drive E was assigned)
               >>> alt-boot-disk-util.exe -b application.img -a write -d e

     (6) Adjust the BOOTSEL Jumpers to SDMMC. This is as follows:
              -- BOOTSEL0: . [. .]
              -- BOOTSEL1: . [. .]
              -- BOOTSEL2: [. .] .

     (6) Insert the SDMMC card into the DevKit and power up the board.


**************************************
Adding FPGA configuration Programming:
**************************************

     (1) Update the 'mpl_config.h' settings:
           -> CONFIG_MPL_FPGA_LOAD must be set to 1
           -> CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR or CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR
              must be set accordingly depending on the boot source.

     (2) Rebuild MPL if CONFIG_MPL_FPGA_LOAD is changed.

     (3) Obtain the FPGA configuration file in the raw format, with the '.rbf' file extension.
         This example assumes the file is name 'fpga.rbf'.

     (4) Process the file with the mkimage tool to create the FPGA image file.
         >>> mkimage -A arm -O u-boot -T firmware -a 0xA00000 -e 0 -name fpga-rbf -d fpga.rbf -C none fpga.img
         --> The load address (-a) is where the MPL will first copy the entire RBF image
             from flash before programming the FPGA.

     (5) Program the RBF file to flash media.

         QSPI:

         The FPGA configuration location in QSPI is set to 0x800000.
         (see CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR in mpl_config.h)

         To program the FPGA image file into the QSPI flash:
         quartus_hps.exe -c USB-BlasterII -o P -a 0x800000 fpga.img

         SDMMC:

         For SDMMC the address is set to offset 0x100000 of the A2 partition.
         (see CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR in mpl_config.h). If the handoff stage is larger
         than 768 KiB (256 KiB for preloader + 768 KiB for handoff), this value may need to be adjusted
         to a larger value.

         To program the rbf file into SDMMC:
         >>> sudo dd if=fpga.img of=/dev/sdb3 bs=64k seek=16
         --> In this example since address was set to 0x100000 (1MB) and in command bs=64k,
             this corresponds to setting seek to 16 in the dd command.

     (6) Adjust the MSEL settings on DevKit (SW3) according to the FPGA configuration compression.

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> End <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
