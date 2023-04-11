/**
*   @file  gtrack_unit_create.c
*
*   @brief
*      Unit level create function for the GTRACK Algorithm
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
#include <math.h>

#include <gtrack.h>
#include <gtrack_int.h>

/**
*  @b Description
*  @n
*		GTRACK Module calls this function to instantiate GTRACK Unit with desired configuration parameters. 
*		Function returns a handle, which is used my module to call units' methods
*
*  @param[in]  params
*		This is a pointer to the configuration structure. The structure contains all parameters that are exposed by GTRACK alrorithm.
*		The configuration does not need to persist.
*  @param[out] errCode
*      Error code populated on error, see \ref GTRACK_ERROR_CODE
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      Handle to GTRACK unit
*/

void *gtrack_unitCreate(TrackingParams *params, int32_t *errCode)
{
    GtrackUnitInstance *inst;

    *errCode = GTRACK_EOK;

    inst = (GtrackUnitInstance *)gtrack_alloc(1, sizeof(GtrackUnitInstance));
    if(inst == NULL) {
        *errCode = GTRACK_ENOMEM;
        goto exit;
    }

    memset(inst, 0, sizeof(GtrackUnitInstance));
    /* Parameters */
    inst->gatingParams = &params->gatingParams;
    inst->stateParams = &params->stateParams;
    inst->allocationParams = &params->allocationParams;
    inst->unrollingParams = &params->unrollingParams;
    inst->sceneryParams = &params->sceneryParams;
    inst->transormParams = &params->transormParams;

    memcpy(inst->maxAcceleration, params->maxAcceleration, sizeof(inst->maxAcceleration));

    inst->uid = params->uid;
    inst->isTargetStatic = false;
    inst->maxRadialVelocity = params->maxRadialVelocity;


    inst->radialVelocityResolution = params->radialVelocityResolution;
    inst->verbose = params->verbose;
    inst->initialRadialVelocity = params->initialRadialVelocity;

    inst->F = params->F; 
    inst->Q = params->Q; 

    switch(params->stateVectorType) {

    case GTRACK_STATE_VECTORS_2DA:
        inst->stateVectorType = GTRACK_STATE_VECTORS_2DA;
        inst->stateVectorDimNum = 2;
        inst->stateVectorDimLength = 3;
        inst->stateVectorLength = 6;
        inst->measurementVectorLength = 3;

        inst->minStaticVelocityOne = 0.1f;
        inst->minStaticVelocityTwo = 0.1f;
        inst->isAssociationGhostMarking = true;
        inst->isAssociationHeightIgnore = false;

        inst->isEnablePointNumberEstimation = true;
        inst->isSnrWeighting = true;
        inst->estNumOfPoints = inst->allocationParams->pointsThre;
        inst->estSpreadAlpha = 0.05f;
        inst->confidenceLevelThreshold = 0.7f;

        break;

    case GTRACK_STATE_VECTORS_3DA:
        inst->stateVectorType = GTRACK_STATE_VECTORS_3DA;
        inst->stateVectorDimNum = 3;
        inst->stateVectorDimLength = 3;
        inst->stateVectorLength = 9;
        inst->measurementVectorLength = 4;

        inst->minStaticVelocityOne = 0.5f;
        inst->minStaticVelocityTwo = 0.5f;
        inst->isAssociationGhostMarking = false;
        inst->estSpreadAlpha = 0.01f;

        if(fabs(90.0f - inst->sceneryParams->sensorOrientation.elevTilt) < 20.5f) {
            /* Ceiling mount specific initialization */
            inst->isAssociationHeightIgnore = true;
            inst->isEnablePointNumberEstimation = false;
            inst->isSnrWeighting = false;
            inst->estNumOfPoints = 100.0f;
            inst->confidenceLevelThreshold = 0.3f;
        }
        else {
            /* Wall mount specific initialization */
            inst->isAssociationHeightIgnore = false;
            inst->isEnablePointNumberEstimation = true;
            inst->isSnrWeighting = true;
            inst->estNumOfPoints = inst->allocationParams->pointsThre;
            inst->confidenceLevelThreshold = 0.5f;
        }
        break;

    default:
        *errCode = GTRACK_EINVAL;
        goto exit;
    }

    inst->dt = params->deltaT;
    inst->state = TRACK_STATE_FREE;

exit:
    if(*errCode != GTRACK_EOK) {
        if(inst != NULL)
            gtrack_free(inst, sizeof(GtrackUnitInstance));
        inst = NULL;
    }
    return (void *)inst;
}

