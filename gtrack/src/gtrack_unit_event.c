/**
 *   @file  gtrack_unit_event.c
 *
 *   @brief
 *      Unit level event management function for the GTRACK Algorithm
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

#include <math.h>
#include <string.h>

#include <gtrack.h>
#include <gtrack_int.h>

/**
*  @b Description
*  @n
*		GTRACK Module calls this function to run GTRACK unit level state machine 
*
*  @param[in]  handle
*		This is handle to GTARCK unit
*  @param[in]  num
*		This is number of associated measurements
*  @param[in]  numReliable
*		This is number of reliable (dynamic AND unique) measurements
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/
void gtrack_unitEvent(void *handle, uint16_t num, uint16_t numReliable)
{
    GtrackUnitInstance *inst;
    GTRACK_cartesian_position posW;
	uint16_t thre;
    uint16_t numBoxes;
    bool isInsideBoundary = false;
    bool isInsideStatic = false;
    
	inst = (GtrackUnitInstance *)handle;

    if(inst->transormParams->transformationRequired) {
        gtrack_censor2world((GTRACK_cartesian_position *)inst->S_hat, inst->transormParams, &posW);                
    }
    else {
        memcpy(&posW, &inst->S_hat, sizeof(GTRACK_cartesian_position));
    }

    for (numBoxes = 0; numBoxes < inst->sceneryParams->numBoundaryBoxes; numBoxes++) {
        if(gtrack_isPointInsideBox(&posW, &inst->sceneryParams->boundaryBox[numBoxes])) {
            isInsideBoundary = true;
            break;
        }
    }    
    if(isInsideBoundary)
        inst->outside2freeCount = 0;    
    else {
            inst->outside2freeCount ++;
            if(inst->outside2freeCount >= inst->stateParams->exit2freeThre) {
                inst->state = TRACK_STATE_FREE;
#ifdef GTRACK_LOG_ENABLED
                if(inst->verbose & VERBOSE_STATE_INFO)
                    gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] OUTSIDE => FREE\n", inst->heartBeatCount, inst->uid);
#endif
        }
    }

	switch (inst->state)
	{
		case TRACK_STATE_DETECTION:
			if(numReliable > 3)
			{
				/* Hit Event */
				inst->detect2freeCount = 0;
				inst->detect2activeCount++;
				if(inst->detect2activeCount > inst->stateParams->det2actThre)
				{
					inst->state = TRACK_STATE_ACTIVE;
#ifdef GTRACK_LOG_ENABLED
					if(inst->verbose & VERBOSE_STATE_INFO)
						gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] DET=>ACTIVE\n", inst->heartBeatCount, inst->uid);
#endif
				}
			}
			else 
			{
				if(numReliable == 0)
				{
					/* Miss */
					inst->detect2freeCount++;
					if(inst->detect2activeCount > 0)
						inst->detect2activeCount--;

					if(inst->detect2freeCount > inst->stateParams->det2freeThre)
					{
						inst->state = TRACK_STATE_FREE;
#ifdef GTRACK_LOG_ENABLED
						if(inst->verbose & VERBOSE_STATE_INFO)
                            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] DET (%d > %d) => FREE\n", inst->heartBeatCount, inst->uid, inst->detect2freeCount, inst->stateParams->det2freeThre);
#endif
					}
				}
				else
					inst->detect2freeCount = 0;
			}
		break;

		case TRACK_STATE_ACTIVE:

            for (numBoxes = 0; numBoxes < inst->sceneryParams->numStaticBoxes; numBoxes++) {
                if(gtrack_isPointInsideBox(&posW, &inst->sceneryParams->staticBox[numBoxes])) {
                    isInsideStatic = true;
                    break;
                }
            }

            if(inst->stateParams->sleep2freeThre != 0) {
                if(inst->isTargetStatic) {
                    inst->sleep2freeCount++;
                    if(isInsideStatic)
                        thre = inst->stateParams->sleep2freeThre;
                    else
                        thre = inst->stateParams->exit2freeThre;

                    if(inst->confidenceLevel < inst->confidenceLevelThreshold)
                        thre = inst->stateParams->exit2freeThre;

				    if(inst->sleep2freeCount > thre) {
					    inst->state = TRACK_STATE_FREE;
#ifdef GTRACK_LOG_ENABLED
					    if(inst->verbose & VERBOSE_STATE_INFO)
                            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] ACTIVE (SLEEP %d > %d) => FREE\n", inst->heartBeatCount, inst->uid, inst->sleep2freeCount, thre);
#endif
                        break;
				    }
                }
                else
                    inst->sleep2freeCount = 0;
            }

			if(num) 
			{
				/* Hit Event */
				inst->active2freeCount = 0;
			}
			else 
			{
	            /* Miss */
		        inst->active2freeCount++;

				/* Set threshold based on whether target is static, or is in exit zone */
                if(isInsideStatic) {        					
                    if(inst->isTargetStatic)								
    				/* If target is static and inside the static box */
                        thre = inst->stateParams->static2freeThre;	
                    else								
    					/* Normal moving target */
                        thre = inst->stateParams->active2freeThre;
				}
                else
				{
                    /* Exit zone threshold */
    				thre = inst->stateParams->exit2freeThre;
				}

				if(inst->active2freeCount > thre) {
					inst->state = TRACK_STATE_FREE;
#ifdef GTRACK_LOG_ENABLED
					if(inst->verbose & VERBOSE_STATE_INFO)
						gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] ACTIVE (%d > %d) => FREE\n", inst->heartBeatCount, inst->uid, inst->active2freeCount, thre);
#endif
				}
			}
		break;

		default:
			break;
	}
}
