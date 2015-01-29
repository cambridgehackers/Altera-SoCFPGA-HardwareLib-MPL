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

#include <stdio.h>
#include <build.h>
#include <socal.h>
#include <hps.h>
#include <hwlib.h>
#include <mpl_config.h>
#include <header.h>

// For ARMCOMPILER, ensure semihosting functions are not being linked. And, resolved the ones that are absolutely needed locally.
#if defined(ARMCOMPILER) && defined(PRINTF_UART)
#pragma import(__use_no_semihosting)
#endif

#ifdef ARMCOMPILER
void _sys_exit(int return_code)
{
    while(1);
}
int _sbrk = 0;
#endif

volatile img_header_t img_hdr;
//TO-DO
#ifndef CONFIG_PRELOADER_DEBUG_MEMORY_ADDR  // default needed for startup.s
#define CONFIG_PRELOADER_DEBUG_MEMORY_ADDR	0xfffffd00
#define CONFIG_PRELOADER_DEBUG_MEMORY_SIZE	(0x200)
#endif
uint32_t debug_memory_ptr = CONFIG_PRELOADER_DEBUG_MEMORY_ADDR;

extern ALT_STATUS_CODE board_init(void);
extern ALT_STATUS_CODE qspi_load(void);
extern ALT_STATUS_CODE sdmmc_load(void);

int main(){
	uint32_t ret;
	// do board initialization (plls, sdram, pinmux/io, etc.. )
	ret = board_init();
	if (ret != ALT_E_SUCCESS){ return ret;}
	ALT_PRINTF("MPL board-init completed succesfully\n\r");
	
#ifdef QSPI_BOOT 
	ALT_PRINTF("Booting from QSPI\n\r");
	ret = qspi_load();
#elif SDMMC_BOOT
	ALT_PRINTF("Booting from SDMMC\n\r");
	ret = sdmmc_load();
#else	
	//#ifdef NAND_BOOT 
	//	ALT_PRINTF("NAND boot\n\r");
#error "No valid boot source set in build.h It must be QSPI or SDMMC"	
#endif
	
	return ret;	
}

