/****************************************************************************************
* FileName     : crc_compute.c
*
* Description  : This file compute the different type (16/32/64Bit) of CRC of given data.
*
****************************************************************************************
* (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
*---------------------------------------------------------------------------------------
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided that the following conditions are met:
*
*    Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
*  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <stdint.h>

/************************************************************/
/* Type Definitions                                         */
/************************************************************/
#ifndef _UINT64_DECLARED
typedef uint64_t uint64;
#define    _UINT64_DECLARED
#endif

#ifndef _UINT32_DECLARED
typedef uint32_t uint32;
#define    _UINT32_DECLARED
#endif

#ifndef _UINT16_DECLARED
typedef uint16_t uint16;
#define    _UINT16_DECLARED
#endif

#ifndef _UINT8_DECLARED
typedef uint8_t uint8;
#define    _UINT8_DECLARED
#endif

#ifndef _SINT64_DECLARED
typedef int64_t sint64;
#define    _SINT64_DECLARED
#endif

#ifndef _SINT32_DECLARED
typedef int32_t sint32;
#define    _SINT32_DECLARED
#endif

#ifndef _SINT16_DECLARED
typedef int16_t sint16;
#define    _SINT16_DECLARED
#endif

#ifndef _SINT8_DECLARED
typedef int8_t sint8;
#define    _SINT8_DECLARED
#endif

#ifndef _FLOAT32_DECLARED
typedef float float32;
#define    _FLOAT32_DECLARED
#endif

#ifndef _FLOAT64_DECLARED
typedef double float64;
#define    _FLOAT64_DECLARED
#endif

/* CRC parameters (default values are for CRC-32): */
sint32             order   = 32;
uint64             polynom = 0x4c11db7;
sint32             direct  = 1;
uint64             crcinit = 0xffffffff;
uint64             crcxor  = 0xffffffff;
sint32             refin   = 1;
sint32             refout  = 1;

/* internal global values: */
uint64 crcmask;
uint64 crchighbit;
uint64 crcinit_direct;


/* 'order' [1..64] is the CRC polynom order, counted without the leading '1' bit
   'polynom' is the CRC polynom without leading '1' bit
   'direct' [0,1] specifies the kind of algorithm: 1=direct, no augmented zero bits
   'crcinit' is the initial CRC value belonging to that algorithm
   'crcxor' is the final XOR value
   'refin' [0,1] specifies if a data byte is reflected before processing (UART) or not
   'refout' [0,1] specifies if the CRC will be reflected before XOR
*/

/** @fn reflect(uint64 crc, sint32 bitnum)
*   @brief  internal subroutine to compute CRC
*
*   this function used as internal subroutine to compute CRC.
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */

uint64 reflect(uint64 crc, sint32 bitnum)
{
    // reflects the lower 'bitnum' bits of 'crc'

    uint64 i, j=1, crcout=0;

    for (i=(uint64)1<<(bitnum-1); i; i>>=1) {
        if (crc & i) crcout|=j;
        j<<= 1;
    }
    return (crcout);
}


/** @fn crcbitbybitfast(uint8 *p, uint64 len)
*   @brief  internal subroutine to compute CRC
*
*   this function used as internal subroutine to compute CRC.
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
uint64 crcbitbybitfast(uint8 *p, uint64 len)
{
    // fast bit by bit algorithm without augmented zero bytes.
    // does not use lookup table, suited for polynom orders between 1...32.

    uint64 i, j, c, bit;
    uint64 crc = crcinit_direct;

    for (i=0; i<len; i++) {

        c = (uint64)*p++;
        if (refin) c = reflect(c, 8);

        for (j=0x80; j; j>>=1) {

            bit  = crc & crchighbit;
            crc<<= 1;
            if (c & j) bit^= crchighbit;
            if (bit) crc^= polynom;
        }
    }

    if (refout) crc=reflect(crc, order);
    crc^= crcxor;
    crc&= crcmask;

    return(crc);
}

/** @fn computeCRC(uint8 *p, uint32 len, uint8 width)
*   @brief  calculate the 32 bit CRC
*
*   this function used to calculate the 32 bit CRC.
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
uint64 computeCRC(uint8 *p, uint32 len, uint8 width)
{
    if(32 == width)
    {
        // CRC parameters (default values are for CRC-32):
        order   = 32;
        polynom = 0x4c11db7;
        direct  = 1;
        crcinit = 0xffffffff;
        crcxor  = 0xffffffff;
        refin   = 1;
        refout  = 1;
    }
    else if(16 == width)
    {
        // CRC parameters (default values are for CRC-16):
        order   = 16;
        polynom = 0x1021;
        direct  = 1;
        crcinit = 0xffffffff;
        crcxor  = 0x0;
        refin   = 0;
        refout  = 0;
    }
    else //64
    {
        // CRC parameters (default values are for CRC-64):
        order   = 64;
        polynom = 0x1B;
        direct  = 1;
        crcinit = 0x0;
        crcxor  = 0x0;
        refin   = 0;
        refout  = 0;
    }

    // at first, compute constant bit masks for whole CRC and CRC high bit
    crcmask        = ((((unsigned long long)1<<(order-1))-1)<<1)|1;
    crchighbit     = (unsigned long long)1<<(order-1);
    crcinit_direct = crcinit;

    return(crcbitbybitfast(p, len));
}