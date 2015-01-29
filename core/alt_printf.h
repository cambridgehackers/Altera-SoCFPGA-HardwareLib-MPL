/******************************************************************************
Copyright (c) 2013-2014, Altera Corporation.  All rights reserved.

Redistribution and use of this software, in source and binary code forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of Altera Corporation nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. This software may only be used to run on Altera products, or to program Altera devices.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF THE COPYRIGHT HOLDERS OR CONTRIBUTORS ARE ADVISED OF THE POSSIBILITY DAMAGE IS LIKELY TO OCCUR.
******************************************************************************/

#if !defined(ALT_PRINTF_H)
#define ALT_PRINTF_H
#include <stdio.h>
#include <stdarg.h>

#if defined(ARMCOMPILER)
#define printf printf_function
#endif

void *memcpy(void *to, const void *from, size_t size);
int printf(const char *format, ...);
int fprintf(FILE *stream, const char *format, ...);

typedef struct _FILEOP
{
  void (*print_function)(void *context, char toprint);
  void *context;
} FILEOP;

extern FILEOP term0_st;
extern FILEOP term1_st;

#define term0 ((FILE *) &term0_st)
#define term1 ((FILE *) &term1_st)

#if defined (PRINTF_HOST) || defined (PRINTF_UART)
#define ALT_PRINTF(...) printf(__VA_ARGS__)
#else
#define ALT_PRINTF(...) (void)(0)
#endif

#endif //ALT_PRINTF_H
