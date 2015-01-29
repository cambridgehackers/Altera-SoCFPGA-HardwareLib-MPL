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

// include socal headers
#include <socal.h>
#include <hps.h>
#include <alt_rstmgr.h>
#include <alt_sysmgr.h>
#include <alt_l3.h>
#include <alt_sdr.h>
#include "alt_qspi.h"
#include "alt_qspi_private.h"
#include "header.h"
#include "mpl_common.h"
#include "alt_fpga_manager.h"
#include "mpl_config.h"

// include passdown info for next image location
#include "build.h"

extern 	 void sdram_applycfg_ocram(void);

// default value
#ifndef CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE
#define CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE (0x60000)
#endif

uint32_t qspi_device_size;
extern volatile img_header_t img_hdr;

// from crc32.c
extern uint32_t crc32(uint32_t crc, const uint8_t *buf, uint32_t size);
typedef void (*FXN_PTR)(void);

ALT_STATUS_CODE qspi_load(void){
	 ALT_STATUS_CODE status = ALT_E_SUCCESS;
	 uint32_t header_size = sizeof(struct img_header);
	 uint32_t image_addr;
	 	 
	 // initialize qspi
	 status = alt_qspi_init();
	 if (status != ALT_E_SUCCESS){return status;}
	 ALT_PRINTF("QSPI Init ok.n\r");
	 
	 // enable qspi
	 status = alt_qspi_enable();
	 if (status != ALT_E_SUCCESS){return status;}
	 // check if idle
     if (!alt_qspi_is_idle()){return ALT_E_ERROR;}
	 
     // restart wdog before getting next image 
     MPL_WATCHDOG
                    
     //read next image header mkimage format
     //alt_qspi_read(void * dest, uint32_t src, size_t size);
     alt_qspi_read((void *)&img_hdr, CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE, header_size);
     // swap endianess
     if(MKIMG_MAGIC != SWAP_UINT32(img_hdr.magic)){
    	 ALT_PRINTF("Invalid Image Magic #\n\r");
         return ALT_E_ERROR;
     }; 
     img_hdr.img_size = SWAP_UINT32(img_hdr.img_size);
     img_hdr.load_addr = SWAP_UINT32(img_hdr.load_addr);
     img_hdr.entry_point = SWAP_UINT32(img_hdr.entry_point);
     alt_qspi_read((void *)img_hdr.load_addr, (CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE+header_size), img_hdr.img_size);
     image_addr = img_hdr.load_addr+img_hdr.entry_point;
     MPL_WATCHDOG
     ALT_PRINTF("QSPI: Read from 0x%x, Load Address is 0x%x\n\r", 
                          (CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE+header_size), img_hdr.load_addr);     
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
     uint32_t crc_val = 0;
     img_hdr.dcrc = SWAP_UINT32(img_hdr.dcrc);
     // validate crc checksum
     crc_val = crc32(0, (uint8_t *)img_hdr.load_addr, img_hdr.img_size);
     if (crc_val != img_hdr.dcrc) {
          ALT_PRINTF("QSPI: CRC Mismatch! Expected=0x%x, Calculated=0x%x\n\r", img_hdr.dcrc, crc_val);
          return ALT_E_ERROR;
     }
#endif
     
#if (CONFIG_PRELOADER_STATE_REG_ENABLE == 1)
     alt_write_word(CONFIG_PRELOADER_STATE_REG, CONFIG_PRELOADER_STATE_VALID);
#endif
     
#if (CONFIG_MPL_FPGA_LOAD == 1)
     // Verify FPGA power is on
     if (alt_fpga_state_get() == ALT_FPGA_STATE_POWER_OFF)
     {
    	 ALT_PRINTF("ERROR: FPGA Monitor reports FPGA is powered off.\n");
         return 1;
     }
     status = alt_fpga_control_enable();
     if(status != ALT_E_SUCCESS){ALT_PRINTF("ERROR: fpga-ctrl-en\n"); return status;}
     
     alt_qspi_read((void *)&img_hdr, CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR, header_size);
     img_hdr.load_addr = SWAP_UINT32(img_hdr.load_addr);
     img_hdr.img_size = SWAP_UINT32(img_hdr.img_size);
     alt_qspi_read((void *)img_hdr.load_addr, (CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR+header_size), img_hdr.img_size);
     MPL_WATCHDOG
     
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)     
     crc_val = 0;
     img_hdr.dcrc = SWAP_UINT32(img_hdr.dcrc);
     // validate crc checksum
     crc_val = crc32(0, (uint8_t *)img_hdr.load_addr, img_hdr.img_size);
     if (crc_val != img_hdr.dcrc) { ALT_PRINTF("FPGA IMG CRC ERROR\n\r");}
     MPL_WATCHDOG
#endif     
     status = alt_fpga_configure((void *)img_hdr.load_addr, img_hdr.img_size);
     if(status != ALT_E_SUCCESS){ALT_PRINTF("FPGA Programming ERR\n\r");} 
     alt_fpga_control_disable();    

     // clear bridges & apply config
     ALT_PRINTF("Resetting SDRAM Ports...\n\r");       
        
     // ALT_SYSMGR_FPGAINTF_MODULE_OFST
    	//*(volatile uint32_t *)(0xFFD08028) =  0;   
    	alt_write_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR, 0);

    	// FPGA Ports Reset Control Register
    	//*(volatile uint32_t *)(0xFFC25080) =  0x3fff;
     alt_write_word(ALT_SDR_CTL_FPGAPORTRST_ADDR, 0x3fff);

    	sdram_applycfg_ocram();

    	// Bridge Module Reset Register
    	//*(volatile uint32_t *)(0xFFD0501C) =  7;
    	//*(volatile uint32_t *)(0xFFD0501C) =  0;
    	alt_write_word(ALT_RSTMGR_BRGMODRST_ADDR, 7);
    	alt_write_word(ALT_RSTMGR_BRGMODRST_ADDR, 0);

    	// Make H2F bridges visible & remap to OCR
    	//*(volatile uint32_t *)(0xFF800000) = 0x18;
    	alt_write_word(ALT_L3_REMAP_ADDR, 0x1B); 

     ALT_PRINTF("Completed FPGA configuration contents. Launching next-stage app ...\r\n");
#endif

     MPL_WATCHDOG
     LOG_DONE();

     // jump to entry point
     (*(FXN_PTR)(image_addr))();
     
     return status;
}
