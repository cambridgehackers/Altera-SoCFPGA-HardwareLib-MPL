/******************************************************************************
Copyright (c) 2013-2014, Altera Corporation.  All rights reserved.

Redistribution and use of this software, in source and binary code forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of Altera Corporation nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. This software may only be used to run on Altera products, or to program Altera devices.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF THE COPYRIGHT HOLDERS OR CONTRIBUTORS ARE ADVISED OF THE POSSIBILITY DAMAGE IS LIKELY TO OCCUR.
******************************************************************************/
#include <stdarg.h>
#include "alt_printf.h"

#define USE_DIVIDE
#define PAD_RIGHT       1
#define PAD_ZERO        2
#define PAD_0x          4
#define PAD_PLUS        8

void null_print_function(void *context, char toprint)
{}

// If you are printing to uart then you need to redefine these elsewhere
__attribute__((weak)) FILEOP term0_st = {null_print_function, NULL};

static int prints(FILEOP *stream, const char *string, int width, int pad);
static int printi(FILEOP *stream, int i, int b, int sg, int width, int pad, int letbase);
int vfprintf(FILE *stream, const char *format, va_list args);

int puts(const char *str)
{
    return prints((FILEOP *)term0, str, 0, 0);
}

size_t fwrite(const void *ptr, size_t size, size_t count, FILE *stream)
{
    size_t a;
    FILEOP *p_op = (FILEOP *)stream;
    for(a=0; a < size*count; a++)
    	p_op->print_function(p_op->context, ((char *) ptr)[a]);
    return a;
}


/** @Function Description:  Writes to the standard output (STDOUT) a sequence of 
  *                         data format argument specifies. Supported format are:
  *                         %x, %s, %c, %%, %X, %d or %i and %u.
  * @API Type:              External
  * @param str              String that contains the text to be written to STDOUT.
  * @return                 None 
  */

int printf(const char *format, ...)
{
    int retval;
    va_list args;
    va_start( args, format );
    retval = vfprintf(term0, format, args);
    va_end(args);
    return retval;
}

int fprintf(FILE *stream, const char *format, ...)
{
    int retval;
    va_list args;
    va_start( args, format );
    retval = vfprintf(stream, format, args);
    va_end(args);
    return retval;
}

int vfprintf(FILE *stream, const char *format, va_list args)
{
    int     width, pad;
    char    scr[3];
    int     printed = 0;
    FILEOP *p_op = (FILEOP *)stream;

    for (; *format != 0; ++format)
    {
        if (*format == '%')
        {
            ++format;
            width = pad = 0;
            if( *format == '\0') break;
            if( *format == '%') goto out;
            if( *format == '-')
            {
                ++format;
                pad = PAD_RIGHT;
            }
            if( *format == '.')
            {
                ++format;
                pad |= PAD_ZERO;
            }// proceed with 0's
            if( *format == '*')
            {
                ++format;
                width = va_arg( args, int );
            }// width is a paramter
            if( *format == '#')
            {
		++format;
                pad |= PAD_0x;
            }// preceed with 0x
            if( *format == '+')
            {
		++format;
		pad |= PAD_PLUS;
            }// proceed with + if positive

            while( *format == '0')
            {
                ++format;
                pad |= PAD_ZERO;
            }

            for ( ; *format >= '0' && *format <= '9'; ++format)
            {
                width *= 10;
                width += *format - '0';
            }

            if( *format == 's' )
            {
                char *s = (char *)va_arg( args, void * );
                printed += prints(p_op, s ? s:"(null)", width, pad);
                continue;
            }
            if( *format == 'l')
            {
                format++;
            }// proceed to 'd'
            if( *format == 'd' || *format == 'i' )
            {
                printed += printi(p_op, va_arg( args, int ), 10, 1, width, pad, 'a');
                continue;
            }
            if( *format == 'p' )
            {
                pad |= PAD_0x;
                printed += printi(p_op, va_arg( args, int ), 16, 0, width, pad, 'a');
            }
            if( *format == 'x' )
            {
                printed += printi(p_op, va_arg( args, int ), 16, 0, width, pad, 'a');
                continue;
            }

            if( *format == 'X' )
            {
                printed += printi(p_op, va_arg( args, int ), 16, 0, width, pad, 'A');
                continue;
            }

            if( *format == 'u' )
            {
                printed += printi(p_op, va_arg( args, int ), 10, 0, width, pad, 'a');
                continue;
            }

            if( *format == 'c' )
            {
                /* char are converted to int then pushed on the stack */
                scr[0] = (char)va_arg( args, int );
                scr[1] = '\0';
                printed += prints(p_op, scr, width, pad);
                continue;
            }
        }
        else
        {
            out:
    	    p_op->print_function(p_op->context, *format);
            printed ++;
        }
    }

    return printed;
}


static int prints(FILEOP *p_op, const char *string, int width, int pad)
{
    int padchar = ' ';
    int printed = 0;

    if (width > 0)
    {
        int len = 0;
        const char *ptr;
        
        for (ptr = string; *ptr; ++ptr) 
            ++len;
        
        if (len >= width) 
            width = 0;
        else 
            width -= len;
        
        if (pad & PAD_ZERO) 
            padchar = '0';
    }

    if (!(pad & PAD_RIGHT))
    {
        for ( ; width > 0; --width)
        {
    	    p_op->print_function(p_op->context, padchar);
            printed++;
        }
    }

    for ( ; *string ; ++string)
    {
    	p_op->print_function(p_op->context, *string);
        printed++;
    }

    for ( ; width > 0; --width)
    {
    	p_op->print_function(p_op->context, padchar);
        printed++;
    }

    return printed;
}


/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12


static int printi(FILEOP *p_op, int i, int b, int sg, int width, int pad, int letbase)
{
    char           print_buf[PRINT_BUF_LEN];
    char*          s;
    int            t, neg = 0;
    unsigned int   u = i;
    int            printed=0;
    char           scr[3];

    if(pad & PAD_0x)
    {
        scr[0] = '0';
        scr[1] = 'x';
        scr[2] = 0;
        printed += prints(p_op, scr, width, pad);
    }
    if (i == 0)
    {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return prints(p_op, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0)
    {
        neg = 1;
        u = -i;
    }

    s  = print_buf + PRINT_BUF_LEN-1;
    *s = '\0';

#ifdef USE_DIVIDE
    while (u)
    {
        t = u % b;
        if( t >= 10 )
            t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;
    }

#else
    if(b == 16)
    {
	while(u)
        {
            int count=0;
            t = u & 0xF;
            if( t >= 10 )
                t += letbase - '0' - 10;
            *--s = t + '0';
            u >>= 4;
        }
    }
    else
    {
	unsigned int int_tables[10][9] = 
	{
        {4000000000u, 3000000000u, 2000000000, 1000000000, 1000000000, 
             1000000000, 1000000000, 1000000000, 1000000000}, 
        {900000000, 800000000, 700000000, 600000000, 
             500000000, 400000000, 300000000, 200000000, 100000000},
        {90000000, 80000000, 70000000, 60000000, 
             50000000, 40000000, 30000000, 20000000, 10000000},
        {9000000, 8000000, 7000000, 6000000, 
             5000000, 4000000, 3000000, 2000000, 1000000},
        {900000, 800000, 700000, 600000, 
             500000, 400000, 300000, 200000, 100000},
        {90000, 80000, 70000, 60000, 50000, 40000, 30000, 20000, 10000},
        {9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000},
        {900, 800, 700, 600, 500, 400, 300, 200, 100},
        {90, 80, 70, 60, 50, 40, 30, 20, 10},
        {9, 8, 7, 6, 5, 4, 3, 2, 1}};

        int count;
        int offset;
        int hasprinted=0;

        s = &print_buf[0];
        for(offset = 0; offset < 10; offset++)
        {
            t = 0;
            for(count=0; count < 9; count++)
            {
                if(int_tables[offset][count] <= u)
                {
                    u -= int_tables[offset][count];
                    t = 9-count;
                    if(offset == 0)
                        t = 4-count;
                }
            }
            if(hasprinted || t != 0)
            {
                *(++s) = t + '0';
                hasprinted = 1;
            }
        }
        *++s = '\0';
        s = &print_buf[1];
    }
#endif
    if (pad & PAD_PLUS)
    {
        printed++;
    	p_op->print_function(p_op->context, '+');
        --width;
    }
    if (neg)
    {
        if( width && (pad & PAD_ZERO) )
        {
            printed++;
    	    p_op->print_function(p_op->context, '-');
            --width;
        }
        else
        {
            *--s = '-';
        }
    }

    return printed + prints(p_op, s, width, pad);
}

