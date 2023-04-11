/**
 *   @file  gtrack_step.c
 *
 *   @brief
 *      Top level functions for the single step of GTRACK Algorithm
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
#include <float.h>
#include <gtrack.h>
#include <gtrack_int.h>

/**
*  @b Description
*  @n
*	   Algorithm level step funtion
*      Application shall call this function to process one frame of measurements with a given instance of the algorithm
*
*  @param[in]  handle
*      Handle to GTRACK module
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  var
*      Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
*  @param[in]  mNum
*      Number of input measurements
*  @param[out]  t
*      Pointer to an array of \ref GTRACK_targetDesc. Application shall provide sufficient space for the expected number of targets. 
*      This function populates the descritions for each of the tracked target 
*  @param[out]  tNum
*      Pointer to a uint16_t value.
*      Function returns a number of populated target descriptos 
*  @param[out]  mIndex
*      Pointer to an array of uint8_t indices. Application shall provide sufficient space to index all measurment points.
*      This function populates target indices, indicating which tracking ID was assigned to each measurment. See Target ID defeinitions, example \ref GTRACK_ID_POINT_NOT_ASSOCIATED
*	   Shall be set to NULL when indices aren't required.
*  @param[out]  uIndex
*      Pointer to an array of uint8_t bytes. The array represents a bit array, indicating whether measurment point was uniquely associated to the target (1), or was shared among two or more targets (0). 
*      Application shall provide sufficient space for all measurment points.
*      This function populates the bit array. The unique-ness of measurement N is represented by a bit = (N & 0xFF) in (N-1)>>3 byte.
*	   Shall be set to NULL when bit array isn't required.
*  @param[out]  presence
*      Pointer to boolean presence indication. If configured, the function returns presence indication, FALSE if occupancy area is empty and TRUE if occupied.
*  @param[out]  bench
*      Pointer to an array of benchmarking results. Each result is a 32bit timestamp. The array size shall be \ref GTRACK_BENCHMARK_SIZE. 
*      This function populates the array with the timestamps of free runing CPU cycles count.  
*	   Shall be set to NULL when benchmarking isn't required.
*
*  \ingroup GTRACK_ALG_EXTERNAL_FUNCTION
*
*  @retval
*      None
*/

void gtrack_step(void *handle, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t mNum, GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint8_t *uIndex, uint8_t *presence, uint32_t *bench)
{
    GtrackModuleInstance *inst;
	uint16_t n;
    uint16_t numBoundaryBoxes;
    uint16_t numStaticBoxes;
    

    GTRACK_cartesian_position pos;
    GTRACK_cartesian_position posW;


    inst = (GtrackModuleInstance *)handle;
	
	inst->heartBeat++;
    inst->presenceDetectionRaw = 0;

#if 0
#ifdef GTRACK_LOG_ENABLED
	if(inst->verbose & VERBOSE_WARNING_INFO)
		gtrack_log(GTRACK_VERBOSE_WARNING, "Frame #%llu, %u Target(s), %hu Measurements\n", inst->heartBeat, gtrack_listGetCount(&inst->activeList), mNum);
#endif
#endif
	if(mNum > inst->maxNumPoints)
		mNum = inst->maxNumPoints;
    
    memset(inst->isUniqueIndex, 0xFF, (inst->maxNumPoints>>3)+1);
    memset(inst->isStaticIndex, 0x0, (inst->maxNumPoints>>3)+1);

	for(n=0; n< mNum; n++) {
        inst->bestScore[n] = FLT_MAX;

#ifdef GTRACK_3D
        /* If in ceiling mount configuration, ignore points at direct boresight */
        if(inst->isCeilingMounted) {
            if(gtrack_isInsideBoresightStaticZone(&point[n].vector)) {
    		    inst->bestIndex[n] = GTRACK_ID_POINT_BEHIND_THE_WALL;
                continue;
            }
        }
#endif

        if(inst->params.sceneryParams.numBoundaryBoxes) {

            /* If boundaries exists, set index to outside, and overwrite if inside the boundary */
		    inst->bestIndex[n] = GTRACK_ID_POINT_BEHIND_THE_WALL;
            if(inst->params.transormParams.transformationRequired) {
                gtrack_sph2cart(&point[n].vector, &pos);
                gtrack_censor2world(&pos, &inst->params.transormParams, &posW);
            }
            else
                gtrack_sph2cart(&point[n].vector, &posW);

            for (numBoundaryBoxes = 0; numBoundaryBoxes < inst->params.sceneryParams.numBoundaryBoxes; numBoundaryBoxes++) {
                if(gtrack_isPointInsideBox(&posW, &inst->params.sceneryParams.boundaryBox[numBoundaryBoxes])) {
                    /* inside boundary box */
                    if(fabs(point[n].vector.doppler) > FLT_MIN) {
                        /* Valid dynamic point */
                        inst->bestIndex[n] = GTRACK_ID_POINT_NOT_ASSOCIATED;
                    }
                    else {
                        /* Additional check for static points */
                        if(inst->params.sceneryParams.numStaticBoxes) {
                            for (numStaticBoxes = 0; numStaticBoxes < inst->params.sceneryParams.numStaticBoxes; numStaticBoxes++) {
                                if(gtrack_isPointInsideBox(&posW, &inst->params.sceneryParams.staticBox[numStaticBoxes])) {
                                    /* Valid static point */
                                    inst->bestIndex[n] = GTRACK_ID_POINT_NOT_ASSOCIATED;
                                    break;
                                }
                            }
                        }
                        else {
                            /* No static boxes, hence static point is valid */
                            inst->bestIndex[n] = GTRACK_ID_POINT_NOT_ASSOCIATED;
                        }
                    }
                    break;
                }
            }
        }
        else {
            /* No boundaries, hence point is valid */
		    inst->bestIndex[n] = GTRACK_ID_POINT_NOT_ASSOCIATED;
        }
	}

	if(bench != NULL)
	{
		/* Collect benchamrks */
		bench[GTRACK_BENCHMARK_SETUP] = gtrack_getCycleCount();
		gtrack_modulePredict(inst);
		bench[GTRACK_BENCHMARK_PREDICT] = gtrack_getCycleCount();
		gtrack_moduleAssociate(inst, point, mNum);
		bench[GTRACK_BENCHMARK_ASSOCIATE] = gtrack_getCycleCount();
		gtrack_moduleAllocate(inst, point, mNum);
		bench[GTRACK_BENCHMARK_ALLOCATE] = gtrack_getCycleCount();
		gtrack_moduleUpdate(inst, point, var, mNum);
		bench[GTRACK_BENCHMARK_UPDATE] = gtrack_getCycleCount();
        gtrack_modulePresence(inst, presence);
		bench[GTRACK_BENCHMARK_PRESENCE] = gtrack_getCycleCount();
		gtrack_moduleReport(inst, t, tNum);
		bench[GTRACK_BENCHMARK_REPORT] = gtrack_getCycleCount();        
	}
	else
	{
		gtrack_modulePredict(inst);
		gtrack_moduleAssociate(inst, point, mNum);
		gtrack_moduleAllocate(inst, point, mNum);
		gtrack_moduleUpdate(inst, point, var, mNum);
        gtrack_modulePresence(inst, presence);
		gtrack_moduleReport(inst, t, tNum);
	}

	/* If requested, report unique bitmap */
	if(uIndex != 0) {
		if(mNum) {
			for(n=0; n< ((mNum-1)>>3)+1; n++) {
				uIndex[n] = inst->isUniqueIndex[n];
			}
		}
	}

	/* If requested, report uids associated with measurment vector */
	if(mIndex != 0) {
		for(n=0; n< mNum; n++) {
			mIndex[n] = inst->bestIndex[n];
		}
	}
}
