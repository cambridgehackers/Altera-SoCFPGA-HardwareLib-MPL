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
#include "alt_sdmmc.h"
#include "header.h"
#include "alt_printf.h"
#include "mpl_common.h"
#include "sdmmc_common.h"
#include "mpl_config.h"
#include "alt_fpga_manager.h"
#include "alt_clock_manager.h"

// include passdown info for next image location
#include "build.h"

extern void sdram_applycfg_ocram(void);

//default value
#ifndef CONFIG_PRELOADER_SDMMC_NEXT_BOOT_IMAGE
#define CONFIG_PRELOADER_SDMMC_NEXT_BOOT_IMAGE    (0x40000)
#endif

ALT_SDMMC_CARD_INFO_t card_info;
ALT_SDMMC_CARD_MISC_t card_misc_cfg;

#define SDMMC_BLOCK_SZ    512
#define SDMMC_BLK_ALING_MSK 0xFFFFFE00

uint8_t sdmmc_buf[SDMMC_BLOCK_SZ];
img_header_t *img_hdr_p;

//from crc32.c
extern uint32_t crc32(uint32_t crc, const uint8_t * buf, uint32_t size);
typedef void (*FXN_PTR) (void);

ALT_STATUS_CODE sdmmc_load(void)
{
    int i;
    uint32_t *data_src_p, *data_dest_p;
    uint32_t img_block_sz, image_addr, sd_base = 0, raw = 0, sd_addr = 0;

    //initialize sdmmc
    ALT_STATUS_CODE status = alt_sdmmc_init();
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("SDMMC init ERROR\n\r");
        return status;
    }
    status = alt_sdmmc_card_pwr_on();
    status = alt_sdmmc_card_identify(&card_info);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("SDMMC id ERROR\n\r");
        return status;
    }
    status = alt_sdmmc_card_bus_width_set(&card_info, ALT_SDMMC_BUS_WIDTH_4);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("SDMMC bus width set ERROR\n\r");
        return status;
    }
    alt_sdmmc_fifo_param_set((ALT_SDMMC_FIFO_NUM_ENTRIES >> 3) - 1,
                 ALT_SDMMC_FIFO_NUM_ENTRIES >> 3, (ALT_SDMMC_MULT_TRANS_t) 0);
    alt_sdmmc_dma_enable();
    alt_sdmmc_card_misc_get(&card_misc_cfg);
    alt_sdmmc_card_speed_set(&card_info, (card_info.high_speed ? 2 : 1) * card_info.xfer_speed);
    //restart wdog before getting next image
    MPL_WATCHDOG
    // read first block check for MBR Table
    status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *) 0, SDMMC_BLOCK_SZ);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("SDMMC read MBR ERROR\n\r");
        return status;
    }
    if (alt_read_hword(&sdmmc_buf[MBR_SIG_ADDR]) == MBR_SIGNATURE) {
        ALT_PRINTF("Parsing MBR Table Found\n\r");
        for (i = 0; i < 4; i++) {
            mbr_partition_entry_t *mbr_entry = (mbr_partition_entry_t *)
                &sdmmc_buf[(MBR_P1_ADDR + (MBR_PENTRY_SZ * i))];
            if (0xA2 == mbr_entry->p_type) { //0xA2 is custom - partition type
                raw++;
                sd_base = (((mbr_entry->lba_hw2) << 16) | mbr_entry->lba_hw1); //get block address
                ALT_PRINTF("using custom partition %d\n\r", i);
            }
        }
        if (raw == 0) {
            ALT_PRINTF("No custom partition found, Raw Mode\n\r");
            sd_base = 0;
        }
    }
    else {
        ALT_PRINTF("Raw Mode\n\r");
        sd_base = 0;
    }
    ///read next image header using sd_base found above as the base address
    sd_addr = (sd_base * SDMMC_BLOCK_SZ) + CONFIG_PRELOADER_SDMMC_NEXT_BOOT_IMAGE; //start addr is lba needs to be byte address for read func below
    status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *) (sd_addr), SDMMC_BLOCK_SZ);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("SDMMC read header ERROR\n\r");
        return status;
    }
    img_hdr_p = (img_header_t *) sdmmc_buf;
    //swap endianess
    if (MKIMG_MAGIC != SWAP_UINT32(img_hdr_p->magic)) {
        ALT_PRINTF("Invalid Image Magic #\n\r");
        return -1;
    };
    img_hdr_p->img_size = SWAP_UINT32(img_hdr_p->img_size);
    img_hdr_p->load_addr = SWAP_UINT32(img_hdr_p->load_addr);
    img_hdr_p->entry_point = SWAP_UINT32(img_hdr_p->entry_point);
    image_addr = img_hdr_p->load_addr + img_hdr_p->entry_point;
    data_src_p = (uint32_t *) & sdmmc_buf[IMG_HDR_SZ]; //start copying after the img header
    data_dest_p = (uint32_t *) img_hdr_p->load_addr;
    if (img_hdr_p->img_size > (SDMMC_BLOCK_SZ - IMG_HDR_SZ)) {
        //copy data in first sdmcc buff without the img - header
        for (i = IMG_HDR_SZ; i < SDMMC_BLOCK_SZ; i += sizeof(uint32_t)) {
            alt_write_word(data_dest_p, alt_read_word(data_src_p));
            data_src_p++;
            data_dest_p++;
        }
        //read rest of image
        img_block_sz = (img_hdr_p->img_size & 0xFFFFFE00) + SDMMC_BLOCK_SZ; //round - up to nearest block boundary
        sd_addr += SDMMC_BLOCK_SZ;
        while (img_block_sz) {
            uint32_t chunk_sz = 256 * 1024;
            if (chunk_sz > img_block_sz)
                chunk_sz = img_block_sz;
            MPL_WATCHDOG;
            status = alt_sdmmc_read(&card_info, data_dest_p, (void *) (sd_addr), chunk_sz);
            if (status != ALT_E_SUCCESS) {
                ALT_PRINTF("SDMMC read img ERROR\n\r");
                return status;
            }
            data_dest_p += chunk_sz / sizeof(uint32_t);
            sd_addr += chunk_sz;
            img_block_sz -= chunk_sz;
        }
    }
    else {
        for (i = IMG_HDR_SZ; i < (img_hdr_p->img_size); i += sizeof(uint32_t)) {
            alt_write_word(data_dest_p, alt_read_word(data_src_p));
            data_src_p++;
            data_dest_p++;
        }
    }
    ALT_PRINTF("SDMMC next image read complete\n\r");
    MPL_WATCHDOG
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
    img_hdr_p->dcrc = SWAP_UINT32(img_hdr_p->dcrc);
    //validate crc checksum
    uint32_t crc_val = crc32(0, (uint8_t *) img_hdr_p->load_addr, img_hdr_p->img_size);
    if (crc_val != img_hdr_p->dcrc) {
        ALT_PRINTF("IMG CRC ERROR\n\r");
        return ALT_E_ERROR;
    }
    MPL_WATCHDOG
#endif
#if (CONFIG_PRELOADER_STATE_REG_ENABLE == 1)
    alt_write_word(CONFIG_PRELOADER_STATE_REG, CONFIG_PRELOADER_STATE_VALID);
#endif
#if (CONFIG_MPL_FPGA_LOAD == 1)
    //Verify FPGA power is on
    if (alt_fpga_state_get() == ALT_FPGA_STATE_POWER_OFF) {
        ALT_PRINTF("ERROR: FPGA Monitor reports FPGA is powered off.\r\n");
        return 1;
    }
    status = alt_fpga_control_enable();
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: fpga-ctrl-en\n");
        return status;
    }
    ALT_PRINTF("Starting to read FPGA configuration from SDMMC.\r\n");
    ///read fpga image header using sd_base found above as the base address
    sd_addr = (sd_base * SDMMC_BLOCK_SZ) + CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR; //start addr is lba needs to be byte address for read func below
    status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *) (sd_addr), SDMMC_BLOCK_SZ);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("SDMMC read header ERROR\n\r");
        return status;
    }
    img_hdr_p = (img_header_t *) sdmmc_buf;
    //swap endianess
    img_hdr_p->img_size = SWAP_UINT32(img_hdr_p->img_size);
    img_hdr_p->load_addr = SWAP_UINT32(img_hdr_p->load_addr);
    data_src_p = (uint32_t *) & sdmmc_buf[IMG_HDR_SZ]; //start copying after the img header
    data_dest_p = (uint32_t *) img_hdr_p->load_addr;
    if (img_hdr_p->img_size > (SDMMC_BLOCK_SZ - IMG_HDR_SZ)) {
        //copy data in first sdmcc buff without the img - header
        for (i = IMG_HDR_SZ; i < SDMMC_BLOCK_SZ; i += sizeof(uint32_t)) {
            alt_write_word(data_dest_p, alt_read_word(data_src_p));
            data_src_p++;
            data_dest_p++;
        }
        //read rest of image
        img_block_sz = (img_hdr_p->img_size & 0xFFFFFE00) + SDMMC_BLOCK_SZ; //round - up to nearest block boundary
        status = alt_sdmmc_read(&card_info, data_dest_p, (void *) (sd_addr + SDMMC_BLOCK_SZ), img_block_sz);
    }
    else { //case where img_size is less than the initial block
        for (i = 0; i < (img_hdr_p->img_size); i += sizeof(uint32_t)) {
            alt_write_word(data_dest_p, alt_read_word(data_src_p));
            data_src_p++;
            data_dest_p++;
        }
    }
    ALT_PRINTF("Completed reading of FPGA configuration contents. Configure FPGA next...\r\n");
    MPL_WATCHDOG
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
    img_hdr_p->dcrc = SWAP_UINT32(img_hdr_p->dcrc);
    //validate crc checksum
    crc_val = crc32(0, (uint8_t *) img_hdr_p->load_addr, img_hdr_p->img_size);
    if (crc_val != img_hdr_p->dcrc) {
        ALT_PRINTF("FPGA IMG CRC ERROR\n\r");
        return ALT_E_ERROR;
    }
    MPL_WATCHDOG
#endif
    status = alt_fpga_configure((void *) img_hdr_p->load_addr, img_hdr_p->img_size);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("FPGA Programming ERR\r\n");
    }
    alt_fpga_control_disable();
    //clear bridges & apply config
    ALT_PRINTF("Resetting SDRAM Ports...\n\r");
    //ALT_SYSMGR_FPGAINTF_MODULE_OFST
    alt_write_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR, 0);
    //FPGA Ports Reset Control Register
    alt_write_word(ALT_SDR_CTL_FPGAPORTRST_ADDR, 0x3fff);
    sdram_applycfg_ocram();
    //Bridge Module Reset Register
    alt_write_word(ALT_RSTMGR_BRGMODRST_ADDR, 7);
    alt_write_word(ALT_RSTMGR_BRGMODRST_ADDR, 0);
    //Make H2F bridges visible & remap to OCR
    alt_write_word(ALT_L3_REMAP_ADDR, 0x1B);
    ALT_PRINTF("Completed FPGA configuration contents. Launching next-stage app ...\r\n");
#endif
    MPL_WATCHDOG
    LOG_DONE();
    (*(FXN_PTR) (image_addr)) (); //jump to entry point
    return status;
}
