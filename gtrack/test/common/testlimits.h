/**
 *   @file  testlimits.h
 *
 *   @brief
 *      Provides testlimits for testing gtrack library
 *  \par
 *  NOTE:
 *      (C) Copyright 2017-2021 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TESTLIMITS_H
#define _TESTLIMITS_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SUBSYS_MSS
    #ifdef GTRACK_3D
        /* Acceptable Resource Limits for 3D Tracker @ R5F */
        #define LIMIT_PROG_MEMORY_SIZE          23000   /* shall take < 23k of proggram memory */
        #define LIMIT_DATA_MEMORY_SIZE_MODULE   3200    /* 3.2k data memory for module */
        #define LIMIT_DATA_MEMORY_SIZE_UNIT     1300    /* 1.3k of data memory per unit, so 20 tracks shall take < 3.2k+20*1.3 = 29.2k data memory  */
        #define LIMIT_USEC_PER_TRACK_AVE        250     /* shall take < 250us per track at average */
        #define LIMIT_USEC_PER_TRACK_MAX        350     /* shall take < 350us per track at maximum */
    #else
        /* Acceptable Resource Limits for 2D Tracker @ R5F */
        #define LIMIT_PROG_MEMORY_SIZE          22000   /* shall take  < 22k of program memory */
        #define LIMIT_DATA_MEMORY_SIZE_MODULE   2800    /* 2.8k data memory for module */
        #define LIMIT_DATA_MEMORY_SIZE_UNIT     800     /* 0.8k of data memory per unit, so 20 tracks shall take < 2.8k+20*0.8 = 18.8k data memory  */
        #define LIMIT_USEC_PER_TRACK_AVE        200     /* shall take < 200us per track at average */
        #define LIMIT_USEC_PER_TRACK_MAX        250     /* shall take < 250us per track at maximum */
    #endif
#else
    #ifdef GTRACK_3D
        /* Acceptable Resource Limits for 3D Tracker @ C66 */
        #define LIMIT_PROG_MEMORY_SIZE          39000   /* shall take < 39k of proggram memory */
        #define LIMIT_DATA_MEMORY_SIZE_MODULE   3200    /* 3.2k data memory for module */
        #define LIMIT_DATA_MEMORY_SIZE_UNIT     1300    /* 1.3k of data memory per unit, so 20 tracks shall take < 3.2k+20*1.3 = 29.2k data memory  */
        #define LIMIT_USEC_PER_TRACK_AVE       	120     /* shall take < 120us per track at average */
        #define LIMIT_USEC_PER_TRACK_MAX        200     /* shall take < 200us per track at maximum */
    #else
        /* Acceptable Resource Limits for 2D Tracker @ C66 */
        #define LIMIT_PROG_MEMORY_SIZE          36000   /* shall take  < 36k of program memory */
        #define LIMIT_DATA_MEMORY_SIZE_MODULE   2800    /* 3k data memory for module */
        #define LIMIT_DATA_MEMORY_SIZE_UNIT     800     /* 0.8k of data memory per unit, so 20 tracks shall take < 2.8k+20*0.8 = 18.8k data memory  */
        #define LIMIT_USEC_PER_TRACK_AVE        80     	/* shall take < 80us per tarck at average */
        #define LIMIT_USEC_PER_TRACK_MAX        120     /* shall take < 120us per track at maximum */
    #endif
#endif

#ifdef __cplusplus
}
#endif

#endif

