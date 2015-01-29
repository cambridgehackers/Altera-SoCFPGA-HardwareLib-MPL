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

#include <string.h>
#include "socal.h"
#include "socal/alt_uart.h"
#include "alt_printf.h"
#include "alt_16550_uart.h"

#define BAUD_RATE      (115200)

static ALT_16550_HANDLE_t mUart;

void upc(void *p_port, char toprint)
{
  alt_16550_fifo_write(&mUart, &toprint, 1);
  if(toprint == '\n')
    upc(NULL, '\r');
}

FILEOP term0_st = {upc, (void *)&mUart};

ALT_STATUS_CODE alt_log_init(void)
{
    uint32_t uart_location = 0;
    ALT_STATUS_CODE status;

    status = alt_16550_init( ALT_16550_DEVICE_SOCFPGA_UART0,  
                                 (void *)&uart_location, 
                                 400, 
                                 &mUart);
    if (status == ALT_E_SUCCESS)
    {   
        status = alt_16550_baudrate_set(&mUart, BAUD_RATE); 
    }
   
    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_line_config_set(&mUart,  
                                             ALT_16550_DATABITS_8, 
                                             ALT_16550_PARITY_DISABLE, 
                                             ALT_16550_STOPBITS_1); 
    }
    
    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_fifo_enable(&mUart);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_enable(&mUart);
    }
    return status;
}

void alt_log_done(void)
{
    // Ensure that the TX FIFO is empty.

    uint32_t level_tx;
    do
    {
        alt_16550_fifo_level_get_tx(&mUart, &level_tx);
    } while (level_tx);
    alt_clrbits_word(ALT_UART_IER_DLH_ADDR(mUart.location), ALT_UART_IER_DLH_PTIME_DLH7_SET_MSK);

    // Leave the UARTs enabled.
    // Many subsequent stages expect the UART to be already setup by preloader.
}

