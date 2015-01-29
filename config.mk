#
# Copyright Altera 2014
# All Rights Reserved.
#

# Must be set to either SDMMC or QSPI
#BOOT_SOURCE := SDMMC

# Must be set to either C5 (Cyclone5) or A5 (Arria5)
#DEVICE      := C5

# Set this to have MPL print status to serial out, or 0 for no status
PRINTF_UART := 1

# Must be either ARM or GNU
COMPILER := GNU

# Should point to the directory of your generated handoff code
# In windows this should start with drive letter (eg c:), not /cygwin/c
ifeq ($(DEVICE), A5)
HANDOFF_BASE := $(SOCEDS_DEST_ROOT)/examples/hardware/av_soc_devkit_ghrd
HANDOFF_SDRAM_DIR ?= $(HANDOFF_BASE)/hps_isw_handoff/ghrd_5astfd5k3_hps_0
else
HANDOFF_BASE := $(SOCEDS_DEST_ROOT)/examples/hardware/cv_soc_devkit_ghrd
HANDOFF_SDRAM_DIR ?= $(HANDOFF_BASE)/hps_isw_handoff/soc_system_hps_0
endif
HANDOFF_DIR ?= $(HANDOFF_BASE)/software/preloader/generated

