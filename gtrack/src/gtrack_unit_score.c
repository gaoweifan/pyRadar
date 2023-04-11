/**
*   @file  gtrack_unit_score.c
*
*   @brief
*      Unit level scoring function for the GTRACK Algorithm
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
#include <string.h>

#include <gtrack.h>
#include <gtrack_int.h>


/**
*  @b Description
*  @n
*		GTRACK Module calls this function to obtain the measurement vector scoring from the GTRACK unit perspective
*
*  @param[in]  handle
*		This is handle to GTRACK unit
*  @param[in]  point
*		This is an array of measurement points
*  @param[inout]  bestScore
*		This is a pointer current scoresheet with best scores. Only better scores are updated
*  @param[inout]  bestInd
*		This is a pointer current scoresheet winners. Only better scores winners are updated
*  @param[out]  isUnique
*       This is an array indicating whether point belongs to a single target (1) or not (0)
*  @param[out]  isStatic
*       This is an array indicating whether point belongs to a static target (1) or not (0)
*  @param[in]  num
*		Number of measurement points
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/
void gtrack_unitScore(void *handle, GTRACK_measurementPoint *point, float *bestScore,
                      uint8_t *bestInd, uint8_t *isUnique, uint8_t *isStatic, uint16_t num)
{
    GtrackUnitInstance *inst;
    uint16_t n;
    uint16_t m;
    uint8_t tid;

    bool isWithinLimits;
    bool isInsideGate;
    bool isInsideInnerGate;
    bool isGhostPoint;
    bool isDynamicPoint;
    bool staticIndication;
    bool clearUniqueInd;

    float mdp, md;

    GTRACK_measurementUnion hs_ghost;
    GTRACK_measurementUnion u_tilda_ghost;
    GTRACK_measurementUnion u_tilda;
    GTRACK_measurementUnion limits;

    GTRACK_cartesian_position posHs;
    GTRACK_cartesian_position posU;
    GTRACK_measurementUnion hs_projection;

    float score;
    float score_ghost;
    float score_gate;
    float logdet;

    float rvOut;

    inst = (GtrackUnitInstance *)handle;


#ifdef GTRACK_LOG_ENABLED
    if(inst->verbose & VERBOSE_ASSOSIATION_INFO) {
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Scoring: G=%5.2f\n",inst->heartBeatCount, inst->uid, inst->G);
    }
#endif

    limits = inst->H_limits;
    /* Add doppler agility */
    limits.vector.doppler = GTRACK_MIN(2*inst->H_limits.vector.doppler, 2*inst->estSpread.vector.doppler);
    
    inst->numAssosiatedPoints = 0;

        
    if(inst->isAssociationGhostMarking)    
    {    
        /* Compute single multipath ghost location */
        hs_ghost = inst->H_s;
        hs_ghost.vector.range += 1.2f;
    }

    if(inst->isAssociationHeightIgnore) {
        /* Ignore logdet for 3D CM */
        logdet = 0;
        gtrack_sph2cart(&inst->H_s.vector, &posHs);
    }
    else {
        /* Compute logdet otherwise */
        logdet = logf(inst->gC_det);
    }

    for(n=0; n < num; n++) {

        if(bestInd[n] == GTRACK_ID_POINT_BEHIND_THE_WALL)
            continue;                           
    
        if(inst->isAssociationHeightIgnore)
        {
            gtrack_sph2cart(&point[n].vector, &posU);
            posHs.posY = posU.posY;
            gtrack_cart2sph(&posHs, &hs_projection.vector);

            gtrack_vectorSub(GTRACK_MEASUREMENT_VECTOR_SIZE, point[n].array, hs_projection.array, u_tilda.array);
        }
        else
        {
            gtrack_vectorSub(GTRACK_MEASUREMENT_VECTOR_SIZE, point[n].array, inst->H_s.array, u_tilda.array);
        }

        rvOut = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->H_s.vector.doppler, point[n].vector.doppler);
        u_tilda.vector.doppler = rvOut - inst->H_s.vector.doppler;

        /* Any point outside the limits is outside the gate */
        isWithinLimits = true;
        isInsideGate = false;
        isInsideInnerGate = false;
        isGhostPoint = false;
        isDynamicPoint = fabsf(rvOut) > FLT_EPSILON;

        if(inst->isAssociationGhostMarking)
        {
            /* Ghost point marking */
            if( (inst->isTargetStatic == false) &&                                      // Only moving targets can produce ghosts
                (point[n].vector.range > inst->H_s.vector.range) &&                     // Behind the target
                gtrack_isInsideSolidAngle(&u_tilda.vector, &inst->estSpread.vector) &&  // Only within solid angle spread
                isDynamicPoint &&                                                       // Only dynamic points can be marked
                (fabs(u_tilda.vector.doppler) < 2*inst->estSpread.vector.doppler))      // Only similar doppler
            {
                /* This is a point behnd existing target, hence mark it as potential multipath reflection */
                isGhostPoint  = true;
                tid = GTRACK_ID_GHOST_POINT_BEHIND;
                score_ghost = 100.0f;

                /* Check whether it is likely to be a single multipath reflection */
                gtrack_vectorSub(GTRACK_MEASUREMENT_VECTOR_SIZE, point[n].array, hs_ghost.array, u_tilda_ghost.array);
                gtrack_computeMahalanobis(u_tilda_ghost.array, inst->gC_inv, &md);

                if(md < 1.0f) {
                    /* Yes, this is very likely a ghost point */
                    tid = GTRACK_ID_GHOST_POINT_LIKELY;
                    score_ghost = logdet + md;
                }
                score = score_ghost;
            }
        }

        for(m=0; m<GTRACK_MEASUREMENT_VECTOR_SIZE; m++)
        {
            if(fabs(u_tilda.array[m]) > limits.array[m])
            {
                isWithinLimits = false;
                break;
            }
        }

        if(isWithinLimits == true)
        {
            /* For the gating purposes we compute partial Mahalanobis distance, ignoring doppler */		
            gtrack_computeMahalanobisPartial(u_tilda.array, inst->gC_inv, &mdp);
            /* Gating Step */
            if(mdp < inst->G) {
                /* Within the Gate, compute scoring function using all dimensions*/
                isInsideGate = true;
                gtrack_computeMahalanobis(u_tilda.array, inst->gC_inv, &md);

                if(md < inst->G)
                    isInsideInnerGate = true;

                score_gate = logdet + md;

                if(isGhostPoint) {
                    if(score_gate < score_ghost) {
                        score = score_gate;
                        tid = (uint8_t)inst->uid;
                        isGhostPoint = false;
        
                        if(isDynamicPoint)
                            inst->numAssosiatedPoints++;
                    }
                }
                else {
                    score = score_gate;
                    tid = (uint8_t)inst->uid;
    
                    if(isDynamicPoint)
                        inst->numAssosiatedPoints++;
                }
            }
        }

        if((isInsideGate == true) || (isGhostPoint == true)) 
        {
            if(bestInd[n] > GTRACK_ID_GHOST_POINT_BEHIND) 
            {
                /* No competition, we won */
                /* Register our score, and the index */
                bestScore[n] = score;
                bestInd[n] = tid;
                if(inst->isTargetStatic) {
                    /* Set the static indication */
                    isStatic[n>>3] |= (1<<(n & 0x0007));
                }
                point[n].vector.doppler = rvOut;
            }
            else {
                /* We have a competitor */
                clearUniqueInd = true;
                /* Read competitor's static indication */
                staticIndication = isStatic[n>>3] & (1<<(n & 0x0007));

                if(score < bestScore[n]) {
                    /* We won the competition */

                    /* Check whether we need to clear unique indicator */
                    /* We don't clear the indicator when dynamic beats static or dynamic beats ghost */
                    if((inst->isTargetStatic == false) &&                   // Dynamic target is a winner
                       ((staticIndication == true) ||                       // Dynamic target wins against static one
                        (bestInd[n] == GTRACK_ID_GHOST_POINT_LIKELY) ||     // Dynamic target wins agains likely ghost
                        (bestInd[n] == GTRACK_ID_GHOST_POINT_BEHIND)))      // Dynamic target wins agains further away ghost
                    {
                        clearUniqueInd = false;
                        isStatic[n>>3] &= ~(1<<(n & 0x0007));
                    }

                    /* Register our score, and the index */
                    bestScore[n] = score;
                    bestInd[n] = tid;
                    point[n].vector.doppler = rvOut;

                }
                else {
                    /* We lost */
                    /* Check whether we need to clear the unqie indication */
                    /* We don't clear the indicator we lose to dynamic point and we aren't that sure (i.e: we are static, we are ghost point, we are outside of inner gate */
                    if((staticIndication == false) &&                   // Loss to dynamic point
                       ((inst->isTargetStatic == true) ||               // Static loses to dynamic
                        (tid == GTRACK_ID_GHOST_POINT_BEHIND) ||        // Weak ghost looses to dynamic
                        (isInsideInnerGate == false)))                  // Point outsude inner gate loses to dynamic
                    {
                        clearUniqueInd = false;
                    }
                }

                /* If required, clear unque indicator */
                if(clearUniqueInd)
                    isUnique[n>>3] &= ~(1<<(n & 0x0007));
            }
        }
    }	
    memcpy(inst->ec, inst->gC_inv, sizeof(inst->ec));
}


