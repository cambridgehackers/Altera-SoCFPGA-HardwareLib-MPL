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
    
	@=====  Mode bit definitions, and interrupt flag masks
	.equ	Mode_USR,	0x10	@ un-privileged mode
	.equ	Mode_FIQ,	0x11	@ entered as result of fast interrupt
	.equ	Mode_IRQ,	0x12	@ entered as result of normal interrupt
	.equ	Mode_SVC,	0x13	@ entered on reset, or execution of SVC instruction
	.equ	Mode_MON,	0x16	@ mode to swith between secure and non-secure state, or SMC call
	.equ	Mode_ABT,	0x17	@ entered as result of a data abort exception or prefetch abort exception 
	.equ	Mode_UNDEF,	0x1B	@ entered as a result of an instruction-related error
	.equ	Mode_SYS,	0x1F	@ priveleged accesses 
	.equ	IRQ_disable,	0x80 	@ mask to disable IRQ bit in CPSR
	.equ	FIQ_disable,	0x40 	@ mask to disable FIQ bit in CPSR

	@===== CR1 bits 
	.equ	CP15_CR_V,	(1 << 13) 

	.global	mpl_vectors	
mpl_vectors:
	LDR PC, mpl_reset_addr
    LDR PC, mpl_undef_addr
    LDR PC, mpl_svc_addr
    LDR PC, mpl_prefetch_addr
    LDR PC, mpl_abort_addr
    LDR PC, mpl_reserved_addr   @ reserved/not-used
    LDR PC, mpl_irq_addr
    LDR PC, mpl_fiq_addr

@=====  place-holder for the pointers to the exception vectors    
mpl_reset_addr:	.word	mpl_reset
mpl_undef_addr:	.word	mpl_hdlr_undef
mpl_svc_addr:	.word	mpl_hdlr_svc
mpl_prefetch_addr:	.word	mpl_hdlr_prefetch
mpl_abort_addr:	.word	mpl_hdlr_abort
mpl_reserved_addr:	.word	mpl_hdlr_reserve
mpl_irq_addr:	.word	mpl_hdlr_irq
mpl_fiq_addr:	.word	mpl_hdlr_fiq


@===== Program Header
@ the header required by BootROM, should start at Ox0040 
@ this is only a place holder outlining required format
@ The final header is to be added by mkimage tool 
mpl_header:
	.word 0x31305341	@ Magic 
	.byte 0x00		@ Version
	.byte 0x00		@ Flags
	.hword 0x0000   @ crc
	.hword 0x0000		@ Length 
	.hword 0x0000		@ Spare
	.word 0x0000		@ Checksum
	@ entry point
	LDR PC, mpl_reset_addr  

	.balign 4

@ Import external symbols
_main:	.word	main
_debug_memory_ptr:	.word	debug_memory_ptr
	
	@ import symbols from .lds linker script
	.global	_bss_start
_bss_start:	.word __bss_start
	.global _bss_end
_bss_end:	.word __bss_end
_user_stack:	.word user_stack_end
	
	.balign 4

	.global	mpl_reset
@===== Reset Handler and true entry point
mpl_reset:
	@ After BootROM executes, r0-r3 contain useful infomation on entry. 
	@ As a first step we need to preserve that information.
	@ write the debug memory header and value of R0 - R3 (which passed
	@ by BootROM) into debug memory region (which is on ocram)
	ldr r4, =debug_memory_ptr	@ r4= address of global var
	@ldr	r5, [r4]	@ read value of preloader debub memory pointer
	ldr r5, =0xFFFFFD00 @ temp value for now for debug mem ptr
	ldr	r4, =0x01444D	@ version id (0x01) and Magic # (0x444d)
	stmia	r5!, {r4}	
	stmia	r5!, {r0 -r3} @ r0-r3 from BootROM
	ldr	r4, =debug_memory_ptr   @ update debug memory pointer
	str	r5, [r4]

	@ switch to SVC32 mode with disabled interrupts
	mrs	r0, cpsr
	bic	r0, r0, #0x1F
	orr	r0, r0, #(Mode_SVC + IRQ_disable + FIQ_disable) 		@  0xD3
	msr	cpsr_cxsf,r0
	    
    @ check which cpu is executing (just in-case)
    mrc		p15, 0, r0, c0, c0, 5	@ read arm affinity register
    ands	r0, r0, #3				@ AND with 3 since only 2 cpus available
    beq		primary_cpu_init
    
    @ if here one of secondary cpu's reached this point
secondary_cpu_loop:    
    @wfe
    b secondary_cpu_loop
    
    @ primary cpu init
    @ must invalidate instruction cache left-on/used by BootROM
primary_cpu_init:    

	@ Set bit V of SCTRL Register
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, #CP15_CR_V
	mcr	p15, 0, r0, c1, c0, 0

	@ Set VBAR
	ldr	r0, =mpl_vectors
	mcr	p15, 0, r0, c12, c0, 0    
	
    @ set up stack pointer with symbol from linker script
    LDR     sp, _user_stack
    
    mov	r0, #0

    @ invalidate TLBs
  	mcr	p15, 0, r0, c8, c7, 0  	

    @ invalidate icache
  	mcr		p15, 0, r0, c7, c5, 0
  	
    @ invalidate branch predictor
    mcr		p15, 0, r0, c7, c5, 6

	@ DSB instruction
	mcr     p15, 0, r0, c7, c10, 4
	@ ISB instruction
	mcr     p15, 0, r0, c7, c5, 4

	@ MMU and cache setup:  clear V, C, A, M bits
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00000007
	bic	r0, r0, #0x00002000

	@ set A, Z, I bits
	orr	r0, r0, #0x00000002
	orr	r0, r0, #0x00001800
	mcr	p15, 0, r0, c1, c0, 0
    
    @ clear bss section (+ZI in scatter, zero-init)
    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    mov     r2, #0
zi_loop:
    str     r2, [r0]	@ zi location
    cmp     r0, r1      @ compare addres
    add     r0, r0, #4  @ ptr+=4
    bne     zi_loop     @ loop until section end
    
    @ branch to ARM main to finish scatter loading, ZI init, etc..
    @ once ARM libs finish scatter loading, will jump to mpl main function in mpl.c 
	B main


@=====  dummy exceptions, mpl should not get exceptions	
mpl_hdlr_undef:
    B mpl_hdlr_undef
mpl_hdlr_svc:
    B mpl_hdlr_svc
mpl_hdlr_prefetch:
    B mpl_hdlr_prefetch
mpl_hdlr_abort:
    B mpl_hdlr_abort
mpl_hdlr_reserve:
    B mpl_hdlr_reserve
mpl_hdlr_irq:
    B mpl_hdlr_irq
mpl_hdlr_fiq:
    B mpl_hdlr_fiq

 @ Configure the fpga2sdram register

	.global	sdram_applycfg_ocram
sdram_applycfg_ocram:
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, #(1 << 11)				@ Disable branch predictor (bit Z)
	bic	r0, #(1 << 12)				@ Disable i-cache (bit I)
	mcr	p15, 0, r0, c1, c0, 0
	
	mcr  p15, 0, r0, c7, c10, 4		@ DSB
	mcr  p15, 0, r0, c7, c5, 4    	@ ISB

	ldr	r1, =0xFFC2505C			@ SOCFPGA_SDR_ADDRESS
	ldr	r0, [r1]
	mov	r3, #8					@ SDR_CTRLGRP_STATICCFG_APPLYCFG_MASK
	orr	r0, r0, r3
	str	r0, [r1]

	mcr     p15, 0, r0, c7, c10, 4	@ DSB
	mcr     p15, 0, r0, c7, c5, 4    	@ ISB

	mrc	p15, 0, r4, c1, c0, 0
	orr	r4, r0, #(1 << 11)		@ Enable back branch predictor
	orr	r4, r0, #(1 << 12)		@ Enable back i-cache
	mcr	p15, 0, r4, c1, c0, 0

	mov	pc, lr

	.global	reset_clock_manager
@ Relocate the sdram_applycfg_ocram function to OCRAM and call it
reset_clock_manager:
	@/* Put Main PLL and Peripheral PLL in bypass */
	ldr     r0, SOCFPGA_CLKMGR
	mov     r1, #0x4	@CLKMGR_BYPASS_ADDRESS
	mov     r2, #0x19	@CLKMGR_BYPASS_MAIN_PER_PLL_MASK
	add     r3, r0, r1
	ldr     r4, [r3]
	orr     r5, r4, r2
	str     r5, [r3]
	dsb
	isb
	mov     r1, #0x54	@CLKMGR_MAINPLLGRP_MAINQSPICLK_ADDRESS
	mov     r2, #0x3	@CLKMGR_MAINQSPICLK_RESET_VALUE
	add     r3, r0, r1
	str     r2, [r3]
	mov     r1, #0x58	@CLKMGR_MAINPLLGRP_MAINNANDSDMMCCLK_ADDRESS
	mov     r2, #0x3	@CLKMGR_MAINNANDSDMMCCLK_RESET_VALUE
	add     r3, r0, r1
	str     r2, [r3]
	mov     r1, #0x90	@CLKMGR_PERPLLGRP_PERQSPICLK_ADDRESS
	mov     r2, #0x1	@CLKMGR_PERQSPICLK_RESET_VALUE
	add     r3, r0, r1
	str     r2, [r3]
	mov     r1, #0x94	@CLKMGR_PERPLLGRP_PERNANDSDMMCCLK_ADDRESS
	mov     r2, #0x1	@CLKMGR_PERNANDSDMMCCLK_RESET_VALUE
	add     r3, r0, r1
	str     r2, [r3]

	@/* Disable the RAM boot */
	ldr     r0, SOCFPGA_RSTMGR
	ldr     r1, SYSMGR_WARMRAMGRP_ENABLE
	mov     r2, #0
	str     r2, [r1]

	@/* Trigger warm reset to continue boot normally */
	mov     r1, #0x00000004	@RSTMGR_CTRL_OFFSET
	add     r2, r0, r1
	mov     r3, #1
	mov     r3, r3, LSL #1	@RSTMGR_CTRL_SWWARMRSTREQ_LSB
	str     r3, [r2]

reset_clock_manager_loop:
	dsb
	isb
	b	reset_clock_manager_loop

SOCFPGA_CLKMGR: .word	0xffd04000	@ SOCFPGA_CLKMGR_ADDRESS
SOCFPGA_RSTMGR: .word	0xffd05000	@ SOCFPGA_RSTMGR_ADDRESS
SYSMGR_WARMRAMGRP_ENABLE: .word	0xffd080e0	@ CONFIG_SYSMGR_WARMRAMGRP_ENABLE =>  (SOCFPGA_SYSMGR_ADDRESS + 0xe0)

	.global	reset_clock_manager_size
reset_clock_manager_size: .word	(reset_clock_manager_size-reset_clock_manager)

@=====  mark end of file
@    END
