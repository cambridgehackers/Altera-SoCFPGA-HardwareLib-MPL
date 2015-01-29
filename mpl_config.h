/******************************************************************************
 *
 * Copyright 2014 Altera Corporation. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 ******************************************************************************/

/*! \file
 *  Altera - MPL Configuration Header definitions
 */

#ifndef _MPL_CONFIG_H
#define _MPL_CONFIG_H

#include "build.h"

/* to enable printf to uart you must also make mpl with PRINTF_UART defined */
#if (CONFIG_PRELOADER_SERIAL_SUPPORT == 1)
#include "alt_printf.h"
#else
#ifndef ALT_PRINTF
#define ALT_PRINTF(...) (void)(0)
#endif
#endif

#ifdef PRINTF_UART
void alt_log_done(void);
void alt_log_init(void);
#define LOG_INIT() alt_log_init()
#define LOG_DONE() alt_log_done()
#else
#define LOG_INIT()
#define LOG_DONE()
#endif

#if (CONFIG_PRELOADER_STATE_REG_ENABLE == 1)
#define CONFIG_PRELOADER_STATE_REG      (0xFFD080C8)
#define CONFIG_PRELOADER_STATE_VALID    (0x49535756)
#endif

/* enabled program the FPGA */
#ifndef CONFIG_MPL_FPGA_LOAD
#define CONFIG_MPL_FPGA_LOAD    (0)
#endif

/* location of FPGA RBF image within QSPI */
#define CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR   (0x800000)
#define CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR  (0x100000)

#endif
