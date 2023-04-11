/**
 *   @file  gtrack_unit_get.c
 *
 *   @brief
 *      Unit level stop function for the GTRACK Algorithm
 *
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
#include <string.h>

#include <gtrack.h>
#include <gtrack_int.h>


/**
*  @b Description
*  @n
*		GTRACK Module calls this function to get target expected measurement matrix, H_s.
*
*  @param[in]  handle
*		This is handle to GTRACK unit
*  @param[in]  pH
*		This is a pointer to the measurement array H that is filled by the function
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void gtrack_unitGetH(void *handle, float *pH)
{
    GtrackUnitInstance *inst;

    inst = (GtrackUnitInstance *)handle;	
    memcpy(pH, inst->H_s.array, sizeof(inst->H_s.array));
}

/**
*  @b Description
*  @n
*		GTRACK Module calls this function to get mesurments centroid
*
*  @param[in]  handle
*		This is handle to GTRACK unit
*  @param[in]  pC
*		This is a pointer to the measurement array H that is filled by the function
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void gtrack_unitGetC(void *handle, float *pC)
{
    GtrackUnitInstance *inst;

    inst = (GtrackUnitInstance *)handle;	
    memcpy(pC, inst->uCenter.array, sizeof(inst->uCenter.array));
}

/**
*  @b Description
*  @n
*		GTRACK Module calls this function to get target expected spread along measurment dimensions
*
*  @param[in]  handle
*		This is handle to GTRACK unit
*  @param[in]  pSpread
*		This is a pointer to the measurement array H that is filled by the function
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void gtrack_unitGetSpread(void *handle, float *pSpread)
{
    GtrackUnitInstance *inst;

    inst = (GtrackUnitInstance *)handle;	
    memcpy(pSpread, inst->estSpread.array, sizeof(inst->estSpread.array));
}

/**
*  @b Description
*  @n
*		GTRACK Module calls this function to get a centroid of associated measurements points in cartesian form
*
*  @param[in]  handle
*		This is handle to GTRACK unit
*  @param[in]  pPos
*		This is a pointer to the measurement array H that is filled by the function
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void gtrack_unitGetMCenter(void *handle, float *pPos)
{
    GtrackUnitInstance *inst;

    inst = (GtrackUnitInstance *)handle;	
    memcpy(pPos, (float *)&inst->uPos, sizeof(inst->uPos));
}
