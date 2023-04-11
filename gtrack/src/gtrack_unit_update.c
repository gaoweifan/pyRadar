/**
*   @file  gtrack_unit_update.c
*
*   @brief
*      Unit level update function for the GTRACK Algorithm
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

#define MSIZE (GTRACK_MEASUREMENT_VECTOR_SIZE)
#define SSIZE (GTRACK_STATE_VECTOR_SIZE)

extern const float spreadMin[];


/**
*  @b Description
*  @n
*		GTRACK Module calls this function to perform an update step for the tracking unit. 
*
*  @param[in]  handle
*		This is handle to GTRACK unit
*  @param[in]  point
*		This is an array of measurement points
*  @param[in]  var
*      Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
*  @param[in]  pInd
*		This is an array of associated UIDs. After association and allocation steps, each measurment shall have a UID assigned.
*  @param[out] isUnique
*       This is an array indicating whether point belongs to a single target (1) or not (0).
*  @param[in]  num
*		Number of measurement points
*
*  \ingroup GTRACK_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

TrackState gtrack_unitUpdate(void *handle, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var,
                             uint8_t *pInd, uint8_t *isUnique, uint16_t num)
{
    GtrackUnitInstance *inst;
    uint16_t n, m;

    uint32_t myPointNum;
    uint32_t myGoodPointNum;
    uint32_t myStaticPointNum;
    uint32_t myDynamicPointNum;

    float alpha;
    float J[MSIZE*SSIZE];
    float PJ[SSIZE*MSIZE];
    float JPJ[MSIZE*MSIZE];
    float u_tilda[MSIZE];
    float cC[MSIZE*MSIZE], cC_inv[MSIZE*MSIZE]; /* centroid Covariance and centroid Covariance inverse */
    float K[SSIZE*MSIZE];
    float rvPilot;
    float vel;

    GTRACK_measurementUnion 	u;
    GTRACK_measurementUnion 	u_goodPointsSum;

    float u_max[MSIZE];
    float u_min[MSIZE];
    float goodPointsSnr = 0.f;
    float cartVelocity = 0.f;
    float confidenceUpdate;
    float spread;
    float sigma;
    float rvIn;

    GTRACK_measurementUnion 	uvar;
    GTRACK_measurementUnion     uvar_sum;
    GTRACK_measurementUnion 	uvar_mean;

    GTRACK_measurementUnion 	rm_diag;

    /* D is Dispersion matrix */
    float D[MSIZE*MSIZE];
    /* Rm is Measurement error covariance matrix */
    float Rm[MSIZE*MSIZE];
    /* Rc is Measurment error covariance matrix for the centroid used for Kalman update */
    float Rc[MSIZE*MSIZE];

    float temp1[SSIZE*SSIZE];
    float *f_ptr;

    inst = (GtrackUnitInstance *)handle;


    myPointNum = 0;
    myGoodPointNum = 0;
    myDynamicPointNum = 0;
    myStaticPointNum = 0;

    gtrack_matrixInit(MSIZE, SSIZE, 0.f, J);
    gtrack_matrixInit(MSIZE, MSIZE, 0.f, D);
    gtrack_vectorInit(MSIZE, 0.f, u_goodPointsSum.array);
    gtrack_vectorInit(MSIZE, 0.f, uvar_sum.array);
    gtrack_vectorInit(MSIZE, -FLT_MAX, u_max);
    gtrack_vectorInit(MSIZE, FLT_MAX, u_min);

    /* Compute means of associated measurement points */
    /* Accumulate measurements */
    for(n = 0; n < num; n++) {
        if(pInd[n] == inst->uid) {

            u.vector = point[n].vector;

            if(var != 0) {
                uvar.vector = var[n];
                gtrack_vectorAdd(GTRACK_MEASUREMENT_VECTOR_SIZE, uvar.array, uvar_sum.array, uvar_sum.array); 
            }

            if(fabs(u.vector.doppler) > FLT_EPSILON) {
                /* This is dynamic point */
                myDynamicPointNum++;
                /* Check whether the point is unique */
                if(isUnique[n>>3] & (0x1 << (n & 0x0007))) {
                    /* Yes, the point is "good" = Dynamic AND Unique */

                    /* Unroll doppler using a pilot */
                    if(myGoodPointNum == 0)
                        rvPilot = u.vector.doppler;
                    else
                        u.vector.doppler = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, rvPilot, u.vector.doppler); /* Unroll using rvPilot */

                    /* Compute maximums and minimums for each measurement */
                    for(m = 0; m < MSIZE; m++) {
                        if(u.array[m] > u_max[m])
                            u_max[m] = u.array[m];
                        if(u.array[m] < u_min[m])
                            u_min[m] = u.array[m];
                    }
                    /* "Good" points = dynamic AND unique points */
                    myGoodPointNum++;
                    
                    if(inst->isSnrWeighting) {
                        goodPointsSnr += point[n].snr;
                        gtrack_vectorScalarMulAcc(GTRACK_MEASUREMENT_VECTOR_SIZE, u.array,  point[n].snr, u_goodPointsSum.array); 
                    }
                    else {
                        gtrack_vectorAdd(GTRACK_MEASUREMENT_VECTOR_SIZE, u.array, u_goodPointsSum.array, u_goodPointsSum.array); 
                    }

                    pInd[n] = GTRACK_ID_RESERVED_GOOD_POINT; /* Mark points as RELIABLE */
                }				
            }
            else {	
                myStaticPointNum++;
            }
            myPointNum++;
        }
    }

    if(inst->isEnablePointNumberEstimation) {
        if(myGoodPointNum) {
            /* Update estimated number of points */
            if(myGoodPointNum > inst->estNumOfPoints)
                inst->estNumOfPoints = (float)myGoodPointNum;
            else
                inst->estNumOfPoints = 0.9f*inst->estNumOfPoints + 0.1f*(float)myGoodPointNum;

            /* lower bound for the estimated number of points is allocation threshold */ 
            if(inst->estNumOfPoints < inst->allocationParams->pointsThre)
                inst->estNumOfPoints = inst->allocationParams->pointsThre;
        }
    }

    if(myGoodPointNum) {
        /* Compute measurement centroid as SNR-weighted mean of associated good points */
        if(inst->isSnrWeighting)
            gtrack_vectorScalarMul(GTRACK_MEASUREMENT_VECTOR_SIZE, u_goodPointsSum.array, 1.0f/goodPointsSnr, inst->uCenter.array);
        else
            gtrack_vectorScalarMul(GTRACK_MEASUREMENT_VECTOR_SIZE, u_goodPointsSum.array, 1.0f/myGoodPointNum, inst->uCenter.array);

        inst->isTargetStatic = false;

        /* Unroll centroid radial velocity based on target state */
        rvIn = inst->uCenter.vector.doppler;
        gtrack_velocityStateHandling(inst, &inst->uCenter.vector);
        /* Compute measurments centroid in cartesian space */
        gtrack_sph2cart(&inst->uCenter.vector, &inst->uPos);

        /* Compute mean measurment variance, if availbale */
        if(var != 0) {
            gtrack_vectorScalarMul(GTRACK_MEASUREMENT_VECTOR_SIZE, uvar_sum.array, 1.0f/(float)myPointNum, uvar_mean.array);
        }
    }

    /* Update measurement spread if we have 2 or more good points */
    if(myGoodPointNum > 1) {
        for(m = 0; m < MSIZE; m++) {            
            spread = u_max[m] - u_min[m];
            /* Unbiased spread estimation */
            spread = spread*(myGoodPointNum+1)/(myGoodPointNum-1);
            /* Spread can't be larger than 2x of configured limits */
            if(spread > 2*inst->H_limits.array[m])
                spread = 2*inst->H_limits.array[m];
            /* Spread can't be smaller than configured limits */
            if(spread < spreadMin[m])
                spread = spreadMin[m];
/*            if(spread < 0.75f*inst->H_limits.array[m]) */
/*                spread = 0.75f*inst->H_limits.array[m]; */

            if(spread > inst->estSpread.array[m])
                inst->estSpread.array[m] = spread;
            else {
                inst->estSpread.array[m] = (1.0f-inst->estSpreadAlpha)*inst->estSpread.array[m] + inst->estSpreadAlpha*spread;
            }
        }
        gtrack_calcDim(inst->estSpread.array, inst->uCenter.vector.range, inst->estDim);
    }


    if (inst->isTargetStatic == false) {
        /* Handle potential transitioning from dynamic to static state */
        /* We can only transition to static when either no points or only static points available */
        if((myPointNum == 0) || (myDynamicPointNum == 0)) {
            /* Compute cartesian velocity */
            for(n=0; n<inst->stateVectorDimNum; n++) {
                if(inst->isAssociationHeightIgnore && n==1)
                    continue;
                vel = inst->S_apriori_hat[GTRACK_VEL_DIMENSION*inst->stateVectorDimNum + n];
                cartVelocity += vel*vel;
            }
            cartVelocity = sqrtf(cartVelocity);
        }

        if(myPointNum == 0) {
            /* Erasures handling: no measurements available */
            if(cartVelocity < inst->minStaticVelocityOne) {
                /* Force zero velocity/zero acceleration */
                for(n=0; n<inst->stateVectorDimNum; n++) {
                    inst->S_apriori_hat[GTRACK_VEL_DIMENSION*inst->stateVectorDimNum + n] = 0.f;
                    inst->S_apriori_hat[GTRACK_ACC_DIMENSION*inst->stateVectorDimNum + n] = 0.f;
                }
                inst->isTargetStatic = true;
            }
            else {
                /* Target is moving => force constant velocity model */
                /* Force zero acceleration and slow down */
                for(n=0; n<inst->stateVectorDimNum; n++) {
                    inst->S_apriori_hat[GTRACK_VEL_DIMENSION*inst->stateVectorDimNum + n] *= 0.5f;
                    inst->S_apriori_hat[GTRACK_ACC_DIMENSION*inst->stateVectorDimNum + n] = 0.f;
                }
            }
        }
        else if(myDynamicPointNum == 0) { 
            /* Handling static only points */
            if(cartVelocity < inst->minStaticVelocityTwo) {
                /* slow enough => complete stop */
                for(n=0; n<inst->stateVectorDimNum; n++) {
                    inst->S_apriori_hat[GTRACK_VEL_DIMENSION*inst->stateVectorDimNum + n] = 0.f;
                    inst->S_apriori_hat[GTRACK_ACC_DIMENSION*inst->stateVectorDimNum + n] = 0.f;
                }
                if(myStaticPointNum > 3) {
                    /* confirmed with many static points => set it to static */
                    inst->isTargetStatic = true;
                }
            }
            else {
                /* Slow down, keep dynamic */
                for(n=0; n<inst->stateVectorDimNum; n++) {
                    inst->S_apriori_hat[GTRACK_VEL_DIMENSION*inst->stateVectorDimNum + n] *= 0.5f;
                    inst->S_apriori_hat[GTRACK_ACC_DIMENSION*inst->stateVectorDimNum + n] = 0.f;
                }
            }
        }
    }
    else {
        /* Handle potential transitioning from static to dynamic state */
        if(myDynamicPointNum > 0)
            inst->isTargetStatic = false;
    }

    /* Update target confidence level */ 
    if(myGoodPointNum) {
        if(inst->numAssosiatedPoints)
            confidenceUpdate = (float)(myGoodPointNum + (myDynamicPointNum-myGoodPointNum)/2)/inst->numAssosiatedPoints;
        else
            confidenceUpdate = 0;

        inst->confidenceLevel = (1.0f-GTRACK_CONFIDENCE_ALPHA)*inst->confidenceLevel + GTRACK_CONFIDENCE_ALPHA*confidenceUpdate;
    }

    if(myGoodPointNum) {
        /* Compute Rm, Measurement Noise covariance matrix */
        if(var == 0) {    
            for(m = 0; m < MSIZE; m++) {   
                /* spread is covered by 2 sigmas */
                sigma = inst->estSpread.array[m]/2;
                uvar_mean.array[m] = sigma*sigma;
            }
        }
        gtrack_matrixSetDiag(GTRACK_MEASUREMENT_VECTOR_SIZE, uvar_mean.array, Rm);
    }

    /* Update Group Dispersion gD matrix */
    if(myGoodPointNum) {
        /* D is the new dispersion matrix, MSIZExMSIZE */
        for(n = 0; n < num; n++) {
            if(pInd[n] == GTRACK_ID_RESERVED_GOOD_POINT) {
                pInd[n] = inst->uid;
                /* Accumulate covariance from all associated points */                  
                gtrack_matrixCovAcc(GTRACK_MEASUREMENT_VECTOR_SIZE, D, point[n].array, inst->uCenter.array);
            }
        }
        /* Normalize it */
        gtrack_matrixCovNormalize(GTRACK_MEASUREMENT_VECTOR_SIZE, D, myGoodPointNum);

        if(myGoodPointNum > GTRACK_MIN_POINTS_TO_UPDATE_DISPERSION) {
            /* Update persistant group dispersion based on instantaneous D */
            /* The factor alpha goes from maximum (1.f) at the first allocation down to minimum of 0.1f once the target is observed for the long time */ 
            alpha = ((float)myGoodPointNum)/inst->estNumOfPoints;

            /*  inst->gD = (1-alpha)*inst->gD + alpha*D */
            gtrack_matrixCovFilter(GTRACK_MEASUREMENT_VECTOR_SIZE, inst->gD, D, alpha);
        }
    }

    if(myGoodPointNum) {
        /* Compute state vector partial derivatives (Jacobian matrix) */
        gtrack_computeJacobian(inst->currentStateVectorType, inst->S_apriori_hat, J);
        /* Compute JPJ' = J(:,1:mSize) * obj.P_apriori(1:mSize,1:mSize) * J(:,1:mSize)' */
        gtrack_matrixComputePJT(inst->P_apriori_hat, J, PJ);
        gtrack_matrixMultiply(MSIZE, SSIZE, MSIZE, J, PJ, JPJ);

        /* Compute centroid measurement noise covariance matrix Rc used for Kalman updates */
        /* First term represents the error in measuring the centroid and decreased with the number of measurments */
        /* Second term represents the centroid unsertanty due to the fact that not all the memebers observed */
        /* See page 327, 11A.2 of S.Blackman */
        /* Rc = Rm/num + alpha*unit.gD*eye(mSize); */

        /* alpha is weighting factor that is the function of the number of observed poionts versus total number of reflections in a group */
        /* alpha = (unit.maxAssociatedPoints-num)/((unit.maxAssociatedPoints-1)*num); */
        /* See page 327, 11A.3 of S.Blackman */

        alpha = ((float)(inst->estNumOfPoints-myGoodPointNum))/((inst->estNumOfPoints-1)*myGoodPointNum);

        //  Rc.e11 = Rm.e11/myPointNum + alpha*inst->gD[0]; 
        //	Rc.e22 = Rm.e22/myPointNum + alpha*inst->gD[4]; 
        //	Rc.e33 = Rm.e33/myPointNum + alpha*inst->gD[8]; 

        gtrack_matrixGetDiag(GTRACK_MEASUREMENT_VECTOR_SIZE, inst->gD, rm_diag.array);
        for(n=0; n<GTRACK_MEASUREMENT_VECTOR_SIZE; n++)
            rm_diag.array[n] = uvar_mean.array[n]/myGoodPointNum + rm_diag.array[n]*alpha;
        gtrack_matrixSetDiag(GTRACK_MEASUREMENT_VECTOR_SIZE, rm_diag.array, Rc);

#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_MATRIX_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: spread\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(1, MSIZE, inst->estSpread.array);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Rm\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, Rm);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: D\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, D);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: gD\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, inst->gD);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Rc\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, Rc);
        }
#endif

#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_MATRIX_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: S-apriori\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(1,SSIZE, inst->S_apriori_hat);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: J\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, SSIZE, J);
        }
#endif

        /* Compute innovation */
        gtrack_matrixSub(MSIZE, 1, inst->uCenter.array, inst->H_s.array, u_tilda);

#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_MATRIX_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: u_tilda\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, 1, u_tilda);
        }
#endif

        /* Compute centroid covariance cC = [3x6]x[6x6]x[6x3]+[3x3] */
        /* cC = J(:,1:mSize) * obj.P_apriori(1:mSize,1:mSize) * J(:,1:mSize)' + Rc */
        gtrack_matrixAdd(MSIZE, MSIZE, JPJ, Rc, cC);

        /* Compute inverse of cC */
        gtrack_matrixInv(cC, temp1, cC_inv);

#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_MATRIX_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: P\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(SSIZE, SSIZE, inst->P_apriori_hat);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: cC\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, cC);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: cC_inv\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, cC_inv);
        }
#endif

        /* Compute Kalman Gain K[6x3] = P[6x6]xJ[3x6]'xIC_inv[3x3]=[6x3] */
        /* K = obj.P_apriori(1:mSize,1:mSize) * J(:,1:mSize)' * iC_inv */
        gtrack_matrixMultiply(SSIZE, MSIZE, MSIZE, PJ, cC_inv, K);

#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_MATRIX_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: K\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(SSIZE, MSIZE, K);
        }
#endif

        /* State estimation */
        /* obj.S_hat(1:mSize) = obj.S_apriori_hat(1:mSize) + K * u_tilda */
        gtrack_matrixMultiply(SSIZE, MSIZE, 1, K, u_tilda, temp1);
        gtrack_matrixAdd(SSIZE,1, inst->S_apriori_hat, temp1, inst->S_hat);

        /* Covariance estimation */
        /* obj.P(1:mSize,1:mSize) = obj.P_apriori(1:mSize,1:mSize) - K * J(:,1:mSize) * obj.P_apriori(1:mSize,1:mSize) */
        gtrack_matrixTransposeMultiply(SSIZE, MSIZE, SSIZE, K, PJ, temp1);
        gtrack_matrixSub(SSIZE, SSIZE, inst->P_apriori_hat, temp1, inst->P_hat);


        /* Compute groupCovariance gC (will be used in gating) */
        /* We will use ellipsoidal gating, that acounts for the dispersion of the group, target maneuver, and measurement noise */
        /* gC = gD + JPJ + Rm */
        gtrack_matrixAdd(MSIZE, MSIZE, JPJ, Rm, temp1);	
        gtrack_matrixAdd(MSIZE, MSIZE, temp1, inst->gD, inst->gC);

        /* Compute inverse of group innovation */
        gtrack_matrixInv(inst->gC, &inst->gC_det, inst->gC_inv);

#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_MATRIX_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: gC\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, inst->gC);
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: gC_inv\n", inst->heartBeatCount, inst->uid);
            gtrack_matrixPrint(MSIZE, MSIZE, inst->gC_inv);
        }
#endif

    }
    else
    {
        /* Handling of erasures */	
        if(myDynamicPointNum == 0) {
            inst->skipPrediction = true;
        }
        memcpy(inst->S_hat, inst->S_apriori_hat, sizeof(inst->S_hat));	
        memcpy(&inst->uPos, inst->S_apriori_hat, sizeof(inst->uPos));	
        memcpy(inst->P_hat, inst->P_apriori_hat, sizeof(inst->P_hat));
    }

#ifdef GTRACK_LOG_ENABLED
    if(inst->verbose & VERBOSE_MATRIX_INFO) {
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: S-hat\n", inst->heartBeatCount, inst->uid);
        gtrack_matrixPrint(SSIZE, 1, inst->S_hat);
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: P-hat\n", inst->heartBeatCount, inst->uid);
        gtrack_matrixPrint(SSIZE, SSIZE, inst->P_hat);
    }
#endif
#ifdef GTRACK_LOG_ENABLED   
    if(inst->verbose & VERBOSE_DEBUG_INFO) {

        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: ", inst->heartBeatCount, inst->uid);
        if(inst->isTargetStatic) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "(ST, %.2f), ",inst->confidenceLevel);
        }
        else {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "(DYN, %.2f), ", inst->confidenceLevel);
        }

        if(myPointNum) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "Update, %d(%d,%d,%d) points, ", myPointNum, myGoodPointNum, myStaticPointNum, inst->numAssosiatedPoints);

            if(inst->state == TRACK_STATE_DETECTION) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, ", DET %d, ",inst->detect2activeCount);
            }
            if(myGoodPointNum) {
                gtrack_log(GTRACK_VERBOSE_DEBUG,"XYZD={");
                f_ptr = (float *)&inst->uPos;
                for(m=0; m<MSIZE-1; m++) { 
                    gtrack_log(GTRACK_VERBOSE_DEBUG,"%3.2f,", *f_ptr++);
                }
                gtrack_log(GTRACK_VERBOSE_DEBUG,"%3.2f=>%3.2f}, ", rvIn, inst->uCenter.vector.doppler);
            }
        }
        else {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "Miss, ");
            if(inst->isTargetStatic) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "Static, ");
            }
            else {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "Moving, ");
            }    
            if(inst->state == TRACK_STATE_DETECTION) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "DET %d, ",inst->detect2freeCount);
            }
            else {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "ACT %d, ",inst->active2freeCount);
            }
        }

        gtrack_log(GTRACK_VERBOSE_DEBUG, "dim={");
        for(m=0; m<MSIZE-1; m++) {    
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%4.3f,", inst->estDim[m]);
        }
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%4.3f}, ", inst->estDim[MSIZE-1]);

        gtrack_log(GTRACK_VERBOSE_DEBUG, "S={");
        for(n=0; n<SSIZE-1; n++) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%.3f,", inst->S_hat[n]);
        }
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%.3f}\n", inst->S_hat[SSIZE-1]);
    }
#endif
    gtrack_unitEvent(inst, myPointNum, myGoodPointNum);
    return(inst->state);
}

void gtrack_velocityStateHandling(void *handle, GTRACK_measurement_vector *uVec)
{
    GtrackUnitInstance *inst;
    float instanteneousRangeRate;
    float rrError;
    float rvError;
    float rvIn;

    inst = (GtrackUnitInstance *)handle;
    rvIn = uVec->doppler;

    switch(inst->velocityHandling) {

    case VELOCITY_INIT:
        uVec->doppler = inst->rangeRate;
        inst->velocityHandling = VELOCITY_RATE_FILTER;
#ifdef GTRACK_LOG_ENABLED
        if(inst->verbose & VERBOSE_UNROLL_INFO) {
            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Update vState VINIT=>VFILT, %3.2f=>%3.2f\n", inst->heartBeatCount, inst->uid, rvIn, uVec->doppler);
        }
#endif
        break;

    case VELOCITY_RATE_FILTER:
        /* In this state we are using filtered Rate Range to unroll radial velocity, stabilizing Range rate */
        instanteneousRangeRate = (uVec->range - inst->allocationRange)/((inst->heartBeatCount-inst->allocationTime)*inst->dt);

        inst->rangeRate = inst->unrollingParams->alpha * inst->rangeRate + (1-inst->unrollingParams->alpha) * instanteneousRangeRate;
        uVec->doppler = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->rangeRate, rvIn);

        rrError = (instanteneousRangeRate - inst->rangeRate)/inst->rangeRate;

        if(fabsf(rrError) < inst->unrollingParams->confidence) {
            inst->velocityHandling = VELOCITY_TRACKING;
#ifdef GTRACK_LOG_ENABLED
            if(inst->verbose & VERBOSE_UNROLL_INFO) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Update vState VFILT=>VTRACK, Unrolling with RangeRate=%3.2f: %3.2f=>%3.2f\n", inst->heartBeatCount, inst->uid, inst->rangeRate, rvIn, uVec->doppler);
            }
#endif
        }
        else {	
#ifdef GTRACK_LOG_ENABLED
            if(inst->verbose & VERBOSE_UNROLL_INFO) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Update vState VFILT, RangeRate=%3.2f, H-s=%3.2f, rvIn=%3.2f=>%3.2f\n", inst->heartBeatCount, inst->uid, inst->rangeRate, inst->H_s.vector.doppler, rvIn, uVec->doppler);
            }
#endif
        }
        break;

    case VELOCITY_TRACKING:
        /* In this state we are using filtered Rate Range to unroll radial velocity and monitoring Hs error */
        instanteneousRangeRate = (uVec->range - inst->allocationRange)/((inst->heartBeatCount-inst->allocationTime)*inst->dt);

        inst->rangeRate = inst->unrollingParams->alpha * inst->rangeRate + (1-inst->unrollingParams->alpha) * instanteneousRangeRate;
        uVec->doppler = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->rangeRate, rvIn);

        rvError = (inst->H_s.vector.doppler - uVec->doppler)/uVec->doppler;
        if(fabsf(rvError) < 0.1f) {
            inst->velocityHandling = VELOCITY_LOCKED;

#ifdef GTRACK_LOG_ENABLED
            if(inst->verbose & VERBOSE_UNROLL_INFO) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Update vState VTRACK=>VLOCK, Unrolling with RangeRate=%3.2f, H-s=%3.2f: %3.2f=>%3.2f\n", inst->heartBeatCount, inst->uid, inst->rangeRate, inst->H_s.vector.doppler, rvIn, uVec->doppler);
            }
#endif
        }
        else {

#ifdef GTRACK_LOG_ENABLED
            if(inst->verbose & VERBOSE_UNROLL_INFO) {
                gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Update vState VTRACK, Unrolling with RangeRate=%3.2f, H-s=%3.2f: %3.2f=>%3.2f\n", inst->heartBeatCount, inst->uid, inst->rangeRate, inst->H_s.vector.doppler, rvIn, uVec->doppler);
            }
#endif
        }
        break;

    case VELOCITY_LOCKED:
        uVec->doppler = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->H_s.vector.doppler, uVec->doppler);
        break;
    }
}
