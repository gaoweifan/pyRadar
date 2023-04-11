/**
*   @file  gtrack_module.c
*
*   @brief
*      Implementation of the GTRACK Algorithm MODULE
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
#include <float.h>
#include <gtrack.h>
#include <gtrack_int.h>

#define PI 3.14159265358979323846f
#define GTRACK_NOMINAL_ALLOCATION_RANGE     	                (6.0f) /* Range for allocation SNR scaling */
#define GTARCK_MAX_SNR_RANGE                                    (2.5f) /* Range for SNR maximum */
#define GTARCK_FIXED_SNR_RANGE                                  (1.0f) /* Range for SNR fixed */
#define GTARCK_MAX_SNR_RATIO                                    ((GTRACK_NOMINAL_ALLOCATION_RANGE)/(GTARCK_MAX_SNR_RANGE))
#define GTARCK_MAX_SNR_RATIO_P4                                 (GTARCK_MAX_SNR_RATIO)*(GTARCK_MAX_SNR_RATIO)*(GTARCK_MAX_SNR_RATIO)*(GTARCK_MAX_SNR_RATIO)

typedef struct
{
    bool isValid;
    uint16_t numAllocatedPoints;
    float totalSNR;
    GTRACK_measurementUnion mCenter;

} allocationSet;

/**
*  @b Description
*  @n
*      This is a MODULE level predict function. The function is called by external step function to perform unit level kalman filter predictions
*
*  @param[in]  inst
*      Pointer to GTRACK module instance
*
*  \ingroup GTRACK_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void gtrack_modulePredict(GtrackModuleInstance *inst)
{
    GTrack_ListElem *tElem;
    uint16_t uid;

    tElem = gtrack_listGetFirst(&inst->activeList);
    while(tElem != 0)
    {
        uid = tElem->data;
        if(uid > inst->maxNumTracks) {
            /* This should never happen */
            gtrack_assert(0);
        }

        gtrack_unitPredict(inst->hTrack[uid]);

        tElem = gtrack_listGetNext(tElem);
    }
}

/**
*  @b Description
*  @n
*      This is a MODULE level associatiation function. The function is called by external step function to associate measurement points with known targets
*
*  @param[in]  inst
*      Pointer to GTRACK module instance
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  num
*      Number of input measurements
*
*  \ingroup GTRACK_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void gtrack_moduleAssociate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num)
{
    GTrack_ListElem *tElem;
    uint16_t uid;

    tElem = gtrack_listGetFirst(&inst->activeList);
    while(tElem != 0)
    {
        uid = tElem->data;
        gtrack_unitScore(inst->hTrack[uid], point, inst->bestScore, inst->bestIndex, inst->isUniqueIndex, inst->isStaticIndex, num);

        tElem = gtrack_listGetNext(tElem);
    }
}

/**
*  @b Description
*  @n
*      This is a MODULE level allocation function. The function is called by external step function to allocate new targets for the non-associated measurement points
*
*  @param[in]  inst
*      Pointer to GTRACK module instance
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  num
*      Number of input measurements
*
*  \ingroup GTRACK_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/
void gtrack_moduleAllocate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num)
{
    uint16_t n, k;

    //	float un[3], uk[3];
    //	float unSum[3];
    GTRACK_measurementUnion mCenter;
    GTRACK_measurementUnion mCurrent;
    GTRACK_measurementUnion mSum;
    GTRACK_cartesian_position mPos;

    GTRACK_measurementUnion uCenter;
    GTRACK_measurementUnion spread;

    uint16_t allocNum;
    float dist;
    float allocSNR;
    GTrack_ListElem *tElemFree;
    GTrack_ListElem *tElemActive;
    uint16_t uid;
    bool isBehind;
    uint8_t numBoxes;

    float snrRatio;
    float snrRatio4;
    float snrThreshold;
    float snrThreMax;
    float snrThreFixed;
    uint16_t maxAllocNum;

    allocationSet set;

    snrThreMax = GTARCK_MAX_SNR_RATIO_P4*inst->params.allocationParams.snrThre;
    snrThreFixed = snrThreMax/3.0f;
    maxAllocNum = 1;
    set.isValid = false;

    for(n=0; n<num; n++) 
    {
        /* Allocation procedure is for points NOT associated or for the unique points that are associated to the ghost behind the target further away */
        /* NOTE: Points that are LIKELY ghosts (associated to the ghost due to single multipath) are excluded from the association */
        if((inst->bestIndex[n] == GTRACK_ID_POINT_NOT_ASSOCIATED) ||        // Allocation is for points NOT associated, OR
            ((inst->bestIndex[n] == GTRACK_ID_GHOST_POINT_BEHIND) &&         // Unique Point is associated to the ghost behind the target further away
            (inst->isUniqueIndex[n>>3] & (0x1 << (n & 0x0007))))) 
        {

            if(fabsf(point[n].vector.doppler) < FLT_EPSILON)
                continue;

            tElemFree = gtrack_listGetFirst(&inst->freeList);
            if(tElemFree == 0) 
            {

#ifdef GTRACK_LOG_ENABLED
                if(inst->verbose & VERBOSE_WARNING_INFO)
                    gtrack_log(GTRACK_VERBOSE_WARNING, "Maximum number of tracks reached!");
#endif
                return;
            }

            inst->allocIndexCurrent[0] = n;
            allocNum = 1;
            allocSNR = point[n].snr;

            mCenter.vector  = point[n].vector;
            mSum.vector = point[n].vector;

            for(k=n+1; k<num; k++) 
            {
                if((inst->bestIndex[k] == GTRACK_ID_POINT_NOT_ASSOCIATED) ||        // Allocation is for points NOT associated, OR
                    ((inst->bestIndex[k] == GTRACK_ID_GHOST_POINT_BEHIND) &&         // Unique Point is associated to the ghost behind the target further away
                    (inst->isUniqueIndex[k>>3] & (0x1 << (k & 0x0007)))))
                {
                    if(fabsf(point[k].vector.doppler) < FLT_EPSILON)
                        continue;

                    mCurrent.vector = point[k].vector;
                    mCurrent.vector.doppler = gtrack_unrollRadialVelocity(inst->params.maxRadialVelocity, mCenter.vector.doppler, mCurrent.vector.doppler);

                    if(fabsf(mCurrent.vector.doppler - mCenter.vector.doppler) < inst->params.allocationParams.maxVelThre) 
                    {
                        dist = gtrack_calcDistance(&mCenter.vector, &mCurrent.vector);
                        if(sqrtf(dist) < inst->params.allocationParams.maxDistanceThre) 
                        {

                            inst->allocIndexCurrent[allocNum] = k;

                            allocNum++;
                            allocSNR +=point[k].snr;
                            // Update the centroid
                            gtrack_vectorAdd(GTRACK_MEASUREMENT_VECTOR_SIZE, mCurrent.array, mSum.array, mSum.array);
                            gtrack_vectorScalarMul(GTRACK_MEASUREMENT_VECTOR_SIZE, mSum.array, 1.0f/(float)allocNum, mCenter.array);
                        }
                    }
                }
            }
            if((allocNum > maxAllocNum) && (fabsf(mCenter.vector.doppler) >= inst->params.allocationParams.velocityThre) )
            {
                maxAllocNum = allocNum;
                set.isValid = true;
                set.numAllocatedPoints = allocNum;
                set.mCenter = mCenter;
                set.totalSNR = allocSNR;

                for(k=0; k<allocNum; k++) 
                {
                    inst->allocIndexStored[k] = inst->allocIndexCurrent[k];
                }
            }
        }
    }
    /* Presence detection */
    if(inst->isPresenceDetectionEnabled && set.isValid && (inst->presenceDetectionRaw == 0))
    {
        /* If presence detection enabled AND we have a valid set and haven't detected yet => proceed with presence detection */
        if( (set.numAllocatedPoints >= inst->params.presenceParams.pointsThre) &&
            ((inst->isPresenceDetectionInitial == true) || (set.mCenter.vector.doppler <= -inst->params.presenceParams.velocityThre) ))
        {
            gtrack_sph2cart(&set.mCenter.vector, &mPos);

            for (numBoxes = 0; numBoxes < inst->params.presenceParams.numOccupancyBoxes; numBoxes++)
            {
                if(gtrack_isPointInsideBox(&mPos, &inst->params.presenceParams.occupancyBox[numBoxes]))
                {
#ifdef GTRACK_LOG_ENABLED
                    if(inst->verbose & VERBOSE_DEBUG_INFO) {
#ifdef GTRACK_3D
                        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: Presence set %d points, doppler = %.2f, at (%.2f, %.2f, %.2f)\n", inst->heartBeat, set.numAllocatedPoints, set.mCenter.vector.doppler, mPos.posX, mPos.posY, mPos.posZ);
#else
                        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: Presence set %d points, doppler = %.2f, at (%.2f, %.2f)\n", inst->heartBeat, set.numAllocatedPoints, set.mCenter.vector.doppler, mPos.posX, mPos.posY);
#endif
                    }
#endif
                    inst->presenceDetectionRaw = 1;
                    break;
                }
            }
        }
    } 

    if(set.isValid) 
    {
        if( (set.numAllocatedPoints >= inst->params.allocationParams.pointsThre) &&
            (fabsf(set.mCenter.vector.doppler) >= inst->params.allocationParams.velocityThre) )
        {
            isBehind = false;
            tElemActive = gtrack_listGetFirst(&inst->activeList);
            while(tElemActive != 0)
            {
                uid = tElemActive->data;
                gtrack_unitGetC(inst->hTrack[uid], uCenter.array);
                gtrack_unitGetSpread(inst->hTrack[uid], spread.array);

                if(gtrack_isPointBehindTarget(&set.mCenter.vector, &uCenter.vector, &spread.vector)) 
                {  
                    isBehind = true;
                    break;
                }
                tElemActive = gtrack_listGetNext(tElemActive);
            }

            if(set.mCenter.vector.range < GTARCK_FIXED_SNR_RANGE)
                snrThreshold = snrThreFixed;
            else 
            {
                snrRatio = GTRACK_NOMINAL_ALLOCATION_RANGE/set.mCenter.vector.range;
                snrRatio4 = snrRatio*snrRatio*snrRatio*snrRatio;

                if (isBehind)
                    snrThreshold = snrRatio4*inst->params.allocationParams.snrThreObscured;
                else
                {
                    if(set.mCenter.vector.range < GTARCK_MAX_SNR_RANGE)
                        snrThreshold = set.mCenter.vector.range*(snrThreMax-snrThreFixed)/(GTARCK_MAX_SNR_RANGE - GTARCK_FIXED_SNR_RANGE);
                    else
                        snrThreshold = snrRatio4*inst->params.allocationParams.snrThre;
                }
            }
#ifdef GTRACK_LOG_ENABLED
                if(inst->verbose & VERBOSE_DEBUG_INFO) {
                    gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: Allocation set %d points, centroid at [", inst->heartBeat, set.numAllocatedPoints);
                    for(n=0; n<GTRACK_MEASUREMENT_VECTOR_SIZE-1; n++) {					
                        gtrack_log(GTRACK_VERBOSE_DEBUG, "%.2f,", set.mCenter.array[n]);
                    }
                    gtrack_log(GTRACK_VERBOSE_DEBUG, "%.2f], ", set.mCenter.array[GTRACK_MEASUREMENT_VECTOR_SIZE-1]);
                    gtrack_log(GTRACK_VERBOSE_DEBUG, "total SNR %.1f > %.1f\n", set.totalSNR, snrThreshold);
                }
#endif
            if(set.totalSNR > snrThreshold)
            {
                /* Associate points with new uid  */
                for(k=0; k < set.numAllocatedPoints; k++)
                    inst->bestIndex[inst->allocIndexStored[k]] = (uint8_t)tElemFree->data;

                /* Allocate new tracker */
                inst->targetNumTotal ++;
                inst->targetNumCurrent ++;
                tElemFree = gtrack_listDequeue(&inst->freeList);
                if(tElemFree!=NULL)
                {
                    gtrack_unitStart(inst->hTrack[tElemFree->data], inst->heartBeat, inst->targetNumTotal, &set.mCenter.vector, isBehind);
                    gtrack_listEnqueue(&inst->activeList, tElemFree);
                }
            }
        }
    }
}

/**
*  @b Description
*  @n
*      This is a MODULE level update function. The function is called by external step function to perform unit level kalman filter updates
*
*  @param[in]  inst
*      Pointer to GTRACK module instance
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  var
*      Pointer to an array of input measurment variances. Set to NULL if variances are unknown
*  @param[in]  num
*      Number of input measurements
*
*  \ingroup GTRACK_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void gtrack_moduleUpdate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t num)
{
    GTrack_ListElem *tElem;
    GTrack_ListElem *tElemToRemove;
    uint16_t uid;
    TrackState state;

    tElem = gtrack_listGetFirst(&inst->activeList);
    while(tElem != 0)
    {
        uid = tElem->data;
        state = gtrack_unitUpdate(inst->hTrack[uid], point, var, inst->bestIndex, inst->isUniqueIndex, num);
        if(state == TRACK_STATE_FREE) {
            tElemToRemove = tElem;
            tElem = gtrack_listGetNext(tElem);
            gtrack_listRemoveElement(&inst->activeList, tElemToRemove);

            gtrack_unitStop(inst->hTrack[tElemToRemove->data]);
            inst->targetNumCurrent --;
            gtrack_listEnqueue(&inst->freeList, tElemToRemove);
        }
        else
            tElem = gtrack_listGetNext(tElem);
    }
}

/**
*  @b Description
*  @n
*      This is a MODULE level presence detections function. The function is called by external step function
*
*  @param[in]  inst
*      Pointer to GTRACK module instance
*  @param[out]  presence indication
*
*  \ingroup GTRACK_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void gtrack_modulePresence(GtrackModuleInstance *inst, uint8_t *presence)
{
    GTrack_ListElem *tElem;
    uint16_t uid;
    uint8_t numBoxes;
    uint8_t oldPresence;

    GTRACK_cartesian_position mCenter;
    GTRACK_cartesian_position posW;

    oldPresence = inst->presenceDetectionOutput;

    /* Presence can be detected at allocation time (i.e. presenceDetectionRaw can be set there), or */
    /* it can be set below based on existing tracks */
    if(inst->isPresenceDetectionEnabled)
    {
        tElem = gtrack_listGetFirst(&inst->activeList);
        while((tElem != 0) && (inst->presenceDetectionRaw == 0))
        {
            uid = tElem->data;
            if(inst->params.transormParams.transformationRequired) {
                gtrack_unitGetMCenter(inst->hTrack[uid], (float *)&mCenter);        
                gtrack_censor2world(&mCenter, &inst->params.transormParams, &posW);
            }
            else
                gtrack_unitGetMCenter(inst->hTrack[uid], (float *)&posW);

            for (numBoxes = 0; numBoxes < inst->params.presenceParams.numOccupancyBoxes; numBoxes++) {
                if(gtrack_isPointInsideBox(&posW, &inst->params.presenceParams.occupancyBox[numBoxes])) {
                    inst->presenceDetectionRaw = 1;
                    break;
                }
            }
            tElem = gtrack_listGetNext(tElem);
        }
    }

    if(inst->presenceDetectionRaw) {
        inst->presenceDetectionOutput = 1;
        inst->presenceDetectionCounter = 0;
    }
    else {
        if(inst->presenceDetectionOutput == 1) {
            inst->presenceDetectionCounter ++;

#ifdef GTRACK_LOG_ENABLED
            if(inst->verbose & VERBOSE_STATE_INFO) {            
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: Presence ON, Counter %d\n", inst->heartBeat, inst->presenceDetectionCounter);        
            }
#endif
            if(inst->presenceDetectionCounter >= inst->params.presenceParams.on2offThre) {
                inst->isPresenceDetectionInitial = false;
                inst->presenceDetectionOutput = 0;
                inst->presenceDetectionCounter = 0;
            }
        }
    }

#ifdef GTRACK_LOG_ENABLED
    if(inst->verbose & VERBOSE_STATE_INFO) {
        if(oldPresence != inst->presenceDetectionOutput)  {
            if(inst->presenceDetectionOutput == 0) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: Presence OFF\n", inst->heartBeat);
            }
            else {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: Presence ON\n", inst->heartBeat);
            }
        }
    }
#endif
    if(presence != 0)
        *presence = inst->presenceDetectionOutput;
}

/**
*  @b Description
*  @n
*      This is a MODULE level report function. The function is called by external step function to obtain unit level data
*
*  @param[in]  inst
*      Pointer to GTRACK module instance
*  @param[out]  t
*      Pointer to an array of \ref GTRACK_targetDesc.
*      This function populates the descritions for each of the tracked target
*  @param[out]  tNum
*      Pointer to a uint16_t value.
*      Function returns a number of populated target descriptos 
*
*  \ingroup GTRACK_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void gtrack_moduleReport(GtrackModuleInstance *inst, GTRACK_targetDesc *t, uint16_t *tNum)
{
    GTrack_ListElem *tElem;
    uint16_t uid;
    uint16_t num = 0;


    tElem = gtrack_listGetFirst(&inst->activeList);
    while(tElem != 0)
    {
        uid = tElem->data;
        gtrack_unitReport(inst->hTrack[uid], &t[num++]);
        tElem = gtrack_listGetNext(tElem);
    }
    *tNum = num;
}
