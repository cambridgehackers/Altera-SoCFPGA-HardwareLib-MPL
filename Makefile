#
# Copyright Altera 2014
# All Rights Reserved.
#
include config.mk

ifeq ($(BOOT_SOURCE), SDMMC)
SDMMC_BOOT = 1
else
 ifeq ($(BOOT_SOURCE), QSPI)
  QSPI_BOOT = 1
 else
  $(error You must specify BOOT_SOURCE=QSPI or SDMMC in config.mk)
 endif
endif

ifneq ($(DEVICE), C5)
 ifneq ($(DEVICE), A5)
  $(error You must specify either DEVICE=C5 or A5 in config.mk)
 endif
endif

ifneq ($(COMPILER), ARM)
 ifeq ($(COMPILER), GNU)
  GNU=1
 else
  $(error You must specify either COMPILER=ARM or GNU in config.mk)
 endif
endif

ifndef HANDOFF_DIR
$(error You must specify HANDOFF_DIR to point to your generated code directory)
endif

HWLIBS_ROOT ?= $(SOCEDS_DEST_ROOT)/ip/altera/hps/altera_hps/hwlib
HWLIBS_DIR  ?= $(HWLIBS_ROOT)/src/hwmgr
SOCAL_DIR ?= $(HWLIBS_ROOT)/include/socal

HWLIBS_SRC:= 	$(HWLIBS_DIR)/alt_watchdog.c \
		$(HWLIBS_DIR)/alt_clock_manager.c \
		$(HWLIBS_DIR)/alt_timers.c \
		$(HWLIBS_DIR)/alt_globaltmr.c \
		$(HWLIBS_DIR)/alt_fpga_manager.c
 
CORE_SRC:= 	core/mpl.c \
		core/crc32.c \
		core/board_init.c 

ALTERA_IP_SRC:= core/altera_ip/freeze_controller.c \
		core/altera_ip/scan_manager.c \
		core/altera_ip/system_manager_pinmux.c \
		core/altera_ip/sdram.c

SDRAM_SRC:=	$(HANDOFF_SDRAM_DIR)/sequencer_auto_ac_init.c \
		$(HANDOFF_SDRAM_DIR)/sequencer_auto_inst_init.c \
		$(HANDOFF_SDRAM_DIR)/sequencer.c

# headers and include paths
HEADER_FILES := $(HANDOFF_DIR)/build.h

INC_PATH := 	-I. \
		-I./core/ -I./core/altera_ip \
		-I$(HWLIBS_ROOT)/include \
		-I$(SOCAL_DIR) \
		-I$(HANDOFF_DIR) \
		-I$(HANDOFF_SDRAM_DIR)

CROSS_COMPILE := arm-altera-eabi-
CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar
NM := $(CROSS_COMPILE)nm
OD := $(CROSS_COMPILE)objdump
OC := $(CROSS_COMPILE)objcopy

RM := rm 
CP := cp -f

#compile optimization flag
OFLAG ?= -O3

ifdef GNU

ASM_SRC			:=core/startup_gcc.s
ASMFLAGS_COMPILER 	:=-c

COMMON_FLAGS		:=-Wno-sign-compare -mcpu=cortex-a9 -march=armv7-a -ffunction-sections -fdata-sections -fno-builtin -fno-stack-protector -Os
CFLAGS_HWLIBS		:= $(COMMON_FLAGS) -std=c99
CFLAGS_COMPILER		:= $(COMMON_FLAGS)
CFLAGS_HANDOFF		:= $(COMMON_FLAGS) -msoft-float -fstack-usage -fno-common

LINKER_SCRIPT 		:=mpl_gcc.lds
LDFLAGS_COMPILER 	:=-nostartfiles -Wl,--entry=mpl_vectors -Wl,-script=$(LINKER_SCRIPT) -Wl,--gc-sections

else ############ Not GNU = armcc build

CC := armcc
AS := armasm
LD := armlink
AR := armar
COREDIR := core

ASM_SRC			:= core/startup.s

# Suppress ARMCC warning 9931: "Yours License for Compiler (feature compiler5) will expire in X days"

COMMON_FLAGS		:=  --diag_suppress=9931 --cpu=Cortex-A9.no_neon.no_vfp --split_sections --c99 --no_unaligned_access -DARMCOMPILER
CFLAGS_HWLIBS		:= $(COMMON_FLAGS) 
CFLAGS_COMPILER 	:= $(COMMON_FLAGS)
CFLAGS_HANDOFF		:= $(COMMON_FLAGS) --diag_suppress=68 --diag_suppress=177 --diag_suppress=550
ASMFLAGS_COMPILER 	:= --cpu=Cortex-A9.no_neon.no_vfp --cpreproc 
LINKER_SCRIPT 		:= mpl.scat
LDFLAGS_COMPILER 	:= --diag_suppress=9931  --cpu=Cortex-A9.no_neon.no_vfp --strict --remove --datacompressor off --diag_error=warning --scatter=$(LINKER_SCRIPT) --entry=mpl_vectors 

endif # GNU

CFLAGS   := -g $(OFLAG) $(INC_PATH) -DARMCC 

ASMFLAGS := -g $(ASMFLAGS_COMPILER)
LDFLAGS  := $(LDFLAGS_COMPILER)

ifdef LOAD_FPGA
CFLAGS += -DCONFIG_MPL_FPGA_LOAD
endif

ifeq ($(DEVICE), A5)
BOARD_SRC:= 	$(HANDOFF_DIR)/iocsr_config_arria5.c \
		$(HANDOFF_DIR)/pinmux_config_arria5.c
else
BOARD_SRC:= 	$(HANDOFF_DIR)/iocsr_config_cyclone5.c \
		$(HANDOFF_DIR)/pinmux_config_cyclone5.c

ifneq ($(DEVICE), C5)
$(warning **** When building, select device, otherwise C5 is assumed. ****)
endif
endif

ifeq ($(QSPI_BOOT),1)
CORE_SRC+= 	core/qspi_load.c 
HWLIBS_SRC+= 	$(HWLIBS_ROOT)/alt_qspi.c
CFLAGS     += -DQSPI_BOOT
OBJ_ELF = mpl_$(DEVICE)_q.axf 
else 
CORE_SRC += 	core/sdmmc_load.c 
HWLIBS_SRC+=	$(HWLIBS_ROOT)/alt_sdmmc.c
CFLAGS     += -DSDMMC_BOOT
OBJ_ELF = mpl_$(DEVICE)_s.axf 
endif

ifeq ($(DEVICE), A5)
CFLAGS     += -DCONFIG_SOCFPGA_ARRIA5
else
ifeq ($(DEVICE), C5)
CFLAGS     += -DCONFIG_SOCFPGA_CYCLONE5
endif
endif

ifdef PRINTF_HOST
HWLIBS_SRC+= 	core/alt_printf.c \
		core/semihost.c

CFLAGS     := $(CFLAGS) -DPRINTF_HOST
ifdef GNU
$(error *** You cannot build semihosted (PRINTF_HOST) with GNU!!)
endif

else
ifeq ($(PRINTF_UART),1)
HWLIBS_SRC+= 	core/alt_printf.c \
		core/mpl_log.c \
		$(HWLIBS_DIR)/alt_16550_uart.c

CFLAGS     := $(CFLAGS) -DPRINTF_UART

ifdef UART_BAUD_RATE
CFLAGS     := $(CFLAGS) -DBAUD_RATE=$(UART_BAUD_RATE)
endif

endif
endif

C_SRC:= $(CORE_SRC) $(HWLIBS_SRC) $(ALTERA_IP_SRC) $(BOARD_SRC) $(SDRAM_SRC)

OBJS:= $(patsubst %.s, bin/%.o,$(notdir $(ASM_SRC))) $(patsubst %.c, bin/%.o,$(notdir $(C_SRC)))

.PHONY: all
all: $(OBJ_ELF)

.DEFAULT_GOAL := $(OBJ_ELF)
$(OBJ_ELF): $(OBJS) Makefile
	$(LD) $(LDFLAGS) $(OBJS) -o $@
	##### modified
	$(OC) -O binary $@ $(basename $@).bin
	mkpimage --header-version 0 -o $(basename $@)-mkpimage.bin $(basename $@).bin $(basename $@).bin $(basename $@).bin $(basename $@).bin
	#arm-altera-eabi-objdump -DS $@ > $@.txt

bin/%.o: core/%.s $(HEADER_FILES) bin
	$(AS) -o $@ $(ASMFLAGS) $<

bin/%.o: core/%.c $(HEADER_FILES) bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_COMPILER) $<

bin/%.o: $(HWLIBS_DIR)/%.c $(HEADER_FILES) bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HWLIBS) $<

bin/%.o: $(HANDOFF_DIR)/%.c $(HEADER_FILES) bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HANDOFF) $<

bin/%.o: $(HANDOFF_SDRAM_DIR)/%.c $(HEADER_FILES) bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HANDOFF) $<

bin/%.o: core/altera_ip/%.c $(HEADER_FILES) bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_COMPILER) $<

bin:
	mkdir $@

.PHONY: clean
clean:
	$(RM) -rf $(OBJ_ELF) bin *.bin

