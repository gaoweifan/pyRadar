/**
 *   @file  gtrack_utilities_3d.c
 *
 *   @brief
 *      This is a set of 2d utilities functions used by GTRACK Algorithm
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

#ifdef GTRACK_3D

#define PI 3.14159265358979323846f
#define RAD2DEG (180.f/PI)

#define GTRACK_NOMINAL_RANGE_SPREAD                             (0.5f)          /* Default value to initialize the range spread */
#define GTRACK_NOMINAL_ANGULAR_SPREAD                           (2*PI/180.f)    /* Default value to initialize the angular spread */
#define GTRACK_NOMINAL_DOPPLER_SPREAD                           (1.0f)          /* Default value to initialize the doppler spread */

#define GTRACK_BORESIGHT_STATIC_ANGLE                           (6*PI/180.f)
#define GTRACK_BORESIGHT_STATIC_RANGE                           (2.0f)


const float pInit[] = {0.f, 0.f, 0.f, 0.5f, 0.5f, 0.5f, 1.f, 1.f, 1.f};
const float spreadMin[] = {1.0f, 10*PI/180.f, 10*PI/180.f, 0.5f};

/**
*  @b Description
*  @n
*		This function is used to convert a vector from sherical to cartesian
*
*  @param[in]  v
*		Pointer to measurements (spheriacl form) vector
*  @param[out]  c
*		Pointer to state vector (cartesian form)
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void gtrack_sph2cart(GTRACK_measurement_vector *v, GTRACK_cartesian_position *c)
{
    float sinAzim, cosAzim;
    float sinElev, cosElev;

    gtrack_sincosd(v->azimuth*RAD2DEG,&sinAzim, &cosAzim);
    gtrack_sincosd(v->elev*RAD2DEG,&sinElev, &cosElev);

    c->posX = v->range*cosElev*sinAzim;
    c->posY = v->range*cosElev*cosAzim;
    c->posZ = v->range*sinElev;
}
/**
*  @b Description
*  @n
*		This function is used to convert a vector from cartesian to spherical
*
*  @param[in]  c
*		Pointer to state vector (cartesian form)
*  @param[out]  v
*		Pointer to measurements (spheriacl form) vector
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void gtrack_cart2sph(GTRACK_cartesian_position *c, GTRACK_measurement_vector *v)
{
    v->range = sqrtf(c->posX*c->posX + c->posY*c->posY + c->posZ*c->posZ); 
    v->azimuth = atanf(c->posX/c->posY);
    v->elev = atanf(c->posZ/sqrtf(c->posX*c->posX + c->posY*c->posY));
}

/**
*  @b Description
*  @n
*		This function is used to transorm cartesian coordinates from sensor-centric to world space
*
*  @param[in]  c_in
*		Pointer to cartesian coordinate before transformation
*  @param[in]  wt
*		parameters for transformation to world coordinate
*  @param[out]  c_out
*		Pointer to cartesian coordinate after transformation
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void gtrack_censor2world(GTRACK_cartesian_position *c_in, GTRACK_worldTransformParams *wt, GTRACK_cartesian_position *c_out)
{
    /* Rotation around X axes */
    c_out->posX =  c_in->posX;
    c_out->posY =  c_in->posY*wt->rotX.cosTheta + c_in->posZ*wt->rotX.sinTheta;
    c_out->posZ = -c_in->posY*wt->rotX.sinTheta + c_in->posZ*wt->rotX.cosTheta;
    /* Move along Z axis */
    c_out->posZ += wt->offsetZ;
}


/**
*  @b Description
*  @n
*		This function computes measurement spread based on current target position and configured target dimensions
*
*  @param[in]  range
*		target range
*  @param[in]  gate_limits
*		target dimentions limits
*  @param[out]  estSpread
*		Pointer to measurement spread
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void gtrack_calcMeasurementSpread(float range, GTRACK_gateLimits *gate_limits, GTRACK_measurement_vector *estSpread)
{
	if(gate_limits->depth <= FLT_MIN)
		estSpread->range = GTRACK_NOMINAL_RANGE_SPREAD;
	else
		estSpread->range = gate_limits->depth;

	if(gate_limits->width <= FLT_MIN)
        estSpread->azimuth = GTRACK_NOMINAL_ANGULAR_SPREAD;
	else
		estSpread->azimuth = 2*atanf((gate_limits->width/2)/range);

	if(gate_limits->height <= FLT_MIN)
        estSpread->elev = GTRACK_NOMINAL_ANGULAR_SPREAD;
	else
		estSpread->elev = 2*atanf((gate_limits->width/2)/range);

    /* Initial value of doppler spread */
    estSpread->doppler = GTRACK_NOMINAL_DOPPLER_SPREAD;
}
/**
*  @b Description
*  @n
*		This function computes measurment error limits based on current target position and configured target dimensions
*
*  @param[in]  range
*		target range
*  @param[in]  gate_limits
*		target dimentions limits
*  @param[out]  limits
*		Pointer to measurment error limits
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void gtrack_calcMeasurementLimits(float range, GTRACK_gateLimits *gate_limits, GTRACK_measurement_vector *limits)
{

	if(gate_limits->depth <= FLT_MIN)
		limits->range = FLT_MAX;
	else
		limits->range = gate_limits->depth/2;

	if(gate_limits->width <= FLT_MIN)
		limits->azimuth = FLT_MAX;
	else
		limits->azimuth = atanf((gate_limits->width/2)/range);

	if(gate_limits->height <= FLT_MIN)
		limits->elev = FLT_MAX;
	else
		limits->elev = atanf((gate_limits->height/2)/range);

	if(gate_limits->vel <= FLT_MIN)
		limits->doppler = FLT_MAX;
	else
		limits->doppler = gate_limits->vel/2;
}


/**
*  @b Description
*  @n
*		This function computes target dimension estimations based on estimated measurement spread and centroid range
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  mSpread
*		Vector S
*  @param[in]  R
*		scalar Range
*  @param[out]  tDim
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_calcDim(float *mSpread, float R, float *tDim)
{
    tDim[0] = mSpread[0];
    tDim[1] = 2*R*tanf(mSpread[1]/2);
    tDim[2] = 2*R*tanf(mSpread[2]/2);
    tDim[3] = mSpread[3];
}

/**
*  @b Description
*  @n
*		This function is used to calculate a distance between two points defined in measurement coordinates
*
*  @param[in]  p1
*		Pointer to measurements (spheriacl form) vector
*  @param[in]  p2
*		Pointer to measurements (spheriacl form) vector
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      distance, m
*/
float gtrack_calcDistance(GTRACK_measurement_vector *p1, GTRACK_measurement_vector *p2)
{
    //  d = u1(1)^2 + u2(1)^2 - 2*u1(1)*u2(1)*(cos(u1(3))*cos(u2(3))*cos(u1(2)-u2(2))+sin(u1(3))*sin(u2(3)));
    float sinAzim, cosAzim;
    float sinElev1, cosElev1;
    float sinElev2, cosElev2;

    gtrack_sincosd((p1->azimuth-p2->azimuth)*RAD2DEG,&sinAzim, &cosAzim);
    gtrack_sincosd(p1->elev*RAD2DEG,&sinElev1, &cosElev1);
    gtrack_sincosd(p2->elev*RAD2DEG,&sinElev2, &cosElev2);

    return p1->range*p1->range + p2->range*p2->range - 2*p1->range*p2->range*(cosElev1*cosElev2*cosAzim+sinElev1*sinElev2);
}

/**
*  @b Description
*  @n
*		This function computes Mahanalobis distance between vector v and distribution D.
*		Vector is of length 4. Vector represents the delta between distribution centroid and measurment vector.
*		Distribution is 4x4 matrix. Distribution represents the inverse of error covariance matrix.
*		Vector is real, single precision floating point array.
*		Matrix is real, single precision floating point array
*		Matrix is in row-major order
*
*  @param[in]  v
*		Vector v
*  @param[in]  D
*		Matrix D
*  @param[out]  md
*		md is computed 4 dimensional mahanalobis distance
*		md = v*D*v';
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_computeMahalanobis(float *v, float *D, float *md)
{
	*md = 
		v[0]*(v[0]*D[0]+v[1]*D[4]+v[2]*D[8]+v[3]*D[12])+
		v[1]*(v[0]*D[1]+v[1]*D[5]+v[2]*D[9]+v[3]*D[13])+
		v[2]*(v[0]*D[2]+v[1]*D[6]+v[2]*D[10]+v[3]*D[14])+
		v[3]*(v[0]*D[3]+v[1]*D[7]+v[2]*D[11]+v[3]*D[15]);
}
/**
*  @b Description
*  @n
*		This function computes partial Mahanalobis distance between vector v and distribution D.
*		Vector is of length 4. Vector represents the delta between distribution centroid and measurment vector.
*		The last dimension of vector is ignored (v[3] = 0).
*		Distribution is 4x4 matrix. Distribution represents the inverse of error covariance matrix.
*		Vector is real, single precision floating point array.
*		Matrix is real, single precision floating point array
*		Matrix is in row-major order
*
*  @param[in]  v
*		Vector v
*  @param[in]  D
*		Matrix D
*  @param[out]  mdp
*		mdp is computed 3 dimensional mahanalobis distance (ignoring forth dimension, i.e v[3] = 0)
*		v[3]=0; mdp = v*D*v';
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_computeMahalanobisPartial(float *v, float *D, float *mdp)
{
	*mdp = 
		v[0]*(v[0]*D[0]+v[1]*D[4]+v[2]*D[8])+
		v[1]*(v[0]*D[1]+v[1]*D[5]+v[2]*D[9])+
		v[2]*(v[0]*D[2]+v[1]*D[6]+v[2]*D[10]);
}
/**
*  @b Description
*  @n
*		This function is used to multiply two matrices.
*		First matrix P is of size 9x9, the second one is of the size 4x9.
*		The second matrix is being transposed first.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  P
*		Matrix P
*  @param[in]  J
*		Matrix J
*  @param[out]  PJ
*		Matrix PJ = P(9,9) X J(4,9)T
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void gtrack_matrixComputePJT(float *P, float *J, float *PJ)
{
	/* We compute PJ' first because it is re-used in Kalman Gain calculations */
	uint16_t i,j, k, n;
	/* PJ[9x4] = P[9x9] x J[4x9]' */
	for (i = 0U, k = 0U; i < 36U; i+= 4U, k+=9U)
	{
		for (j = 0U, n = 0U; j < 4U; j++, n+=9U)
		{
            PJ[i + j] = (P[k + 0U] * J[n + 0U]) +
						(P[k + 1U] * J[n + 1U]) +
                        (P[k + 2U] * J[n + 2U]) +
                        (P[k + 3U] * J[n + 3U]) +
                        (P[k + 4U] * J[n + 4U]) +
                        (P[k + 5U] * J[n + 5U]) +
                        (P[k + 6U] * J[n + 6U]) +
                        (P[k + 7U] * J[n + 7U]) +
                        (P[k + 8U] * J[n + 8U]);	
		}
	}
}
/**
*  @b Description
*  @n
*		This function computes the inverse of 4x4 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[out]  det_out
*		det_out = determinant;
*  @param[out]  B
*		B = inverse(A);
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void gtrack_matrixInv(const float *A, float *det_out, float *B)
{
    float inv[16];
    float det;
    float invdet;
    int i;

    inv[0] = A[5]  * A[10] * A[15] - 
             A[5]  * A[11] * A[14] - 
             A[9]  * A[6]  * A[15] + 
             A[9]  * A[7]  * A[14] +
             A[13] * A[6]  * A[11] - 
             A[13] * A[7]  * A[10];

    inv[4] = -A[4]  * A[10] * A[15] + 
              A[4]  * A[11] * A[14] + 
              A[8]  * A[6]  * A[15] - 
              A[8]  * A[7]  * A[14] - 
              A[12] * A[6]  * A[11] + 
              A[12] * A[7]  * A[10];

    inv[8] = A[4]  * A[9] * A[15] - 
             A[4]  * A[11] * A[13] - 
             A[8]  * A[5] * A[15] + 
             A[8]  * A[7] * A[13] + 
             A[12] * A[5] * A[11] - 
             A[12] * A[7] * A[9];

    inv[12] = -A[4]  * A[9] * A[14] + 
               A[4]  * A[10] * A[13] +
               A[8]  * A[5] * A[14] - 
               A[8]  * A[6] * A[13] - 
               A[12] * A[5] * A[10] + 
               A[12] * A[6] * A[9];

    inv[1] = -A[1]  * A[10] * A[15] + 
              A[1]  * A[11] * A[14] + 
              A[9]  * A[2] * A[15] - 
              A[9]  * A[3] * A[14] - 
              A[13] * A[2] * A[11] + 
              A[13] * A[3] * A[10];

    inv[5] = A[0]  * A[10] * A[15] - 
             A[0]  * A[11] * A[14] - 
             A[8]  * A[2] * A[15] + 
             A[8]  * A[3] * A[14] + 
             A[12] * A[2] * A[11] - 
             A[12] * A[3] * A[10];

    inv[9] = -A[0]  * A[9] * A[15] + 
              A[0]  * A[11] * A[13] + 
              A[8]  * A[1] * A[15] - 
              A[8]  * A[3] * A[13] - 
              A[12] * A[1] * A[11] + 
              A[12] * A[3] * A[9];

    inv[13] = A[0]  * A[9] * A[14] - 
              A[0]  * A[10] * A[13] - 
              A[8]  * A[1] * A[14] + 
              A[8]  * A[2] * A[13] + 
              A[12] * A[1] * A[10] - 
              A[12] * A[2] * A[9];

    inv[2] = A[1]  * A[6] * A[15] - 
             A[1]  * A[7] * A[14] - 
             A[5]  * A[2] * A[15] + 
             A[5]  * A[3] * A[14] + 
             A[13] * A[2] * A[7] - 
             A[13] * A[3] * A[6];

    inv[6] = -A[0]  * A[6] * A[15] + 
              A[0]  * A[7] * A[14] + 
              A[4]  * A[2] * A[15] - 
              A[4]  * A[3] * A[14] - 
              A[12] * A[2] * A[7] + 
              A[12] * A[3] * A[6];

    inv[10] = A[0]  * A[5] * A[15] - 
              A[0]  * A[7] * A[13] - 
              A[4]  * A[1] * A[15] + 
              A[4]  * A[3] * A[13] + 
              A[12] * A[1] * A[7] - 
              A[12] * A[3] * A[5];

    inv[14] = -A[0]  * A[5] * A[14] + 
               A[0]  * A[6] * A[13] + 
               A[4]  * A[1] * A[14] - 
               A[4]  * A[2] * A[13] - 
               A[12] * A[1] * A[6] + 
               A[12] * A[2] * A[5];

    inv[3] = -A[1] * A[6] * A[11] + 
              A[1] * A[7] * A[10] + 
              A[5] * A[2] * A[11] - 
              A[5] * A[3] * A[10] - 
              A[9] * A[2] * A[7] + 
              A[9] * A[3] * A[6];

    inv[7] = A[0] * A[6] * A[11] - 
             A[0] * A[7] * A[10] - 
             A[4] * A[2] * A[11] + 
             A[4] * A[3] * A[10] + 
             A[8] * A[2] * A[7] - 
             A[8] * A[3] * A[6];

    inv[11] = -A[0] * A[5] * A[11] + 
               A[0] * A[7] * A[9] + 
               A[4] * A[1] * A[11] - 
               A[4] * A[3] * A[9] - 
               A[8] * A[1] * A[7] + 
               A[8] * A[3] * A[5];

    inv[15] = A[0] * A[5] * A[10] - 
              A[0] * A[6] * A[9] - 
              A[4] * A[1] * A[10] + 
              A[4] * A[2] * A[9] + 
              A[8] * A[1] * A[6] - 
              A[8] * A[2] * A[5];

    det = A[0] * inv[0] + A[1] * inv[4] + A[2] * inv[8] + A[3] * inv[12];

    invdet = 1.0f / det;

    for (i = 0; i < 16; i++)
        B[i] = inv[i] * invdet;
    
    *det_out = det;
}

/**
*  @b Description
*  @n
*		This function computes the determinant of 4x4 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[out]  det
*		det = det(A);
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixDet(float *A, float *det)
{
    float inv[4];

    inv[0] = A[5]  * A[10] * A[15] - 
             A[5]  * A[11] * A[14] - 
             A[9]  * A[6]  * A[15] + 
             A[9]  * A[7]  * A[14] +
             A[13] * A[6]  * A[11] - 
             A[13] * A[7]  * A[10];

    inv[1] = -A[4]  * A[10] * A[15] + 
              A[4]  * A[11] * A[14] + 
              A[8]  * A[6]  * A[15] - 
              A[8]  * A[7]  * A[14] - 
              A[12] * A[6]  * A[11] + 
              A[12] * A[7]  * A[10];

    inv[2] = A[4]  * A[9] * A[15] - 
             A[4]  * A[11] * A[13] - 
             A[8]  * A[5] * A[15] + 
             A[8]  * A[7] * A[13] + 
             A[12] * A[5] * A[11] - 
             A[12] * A[7] * A[9];

    inv[3] = -A[4]  * A[9] * A[14] + 
               A[4]  * A[10] * A[13] +
               A[8]  * A[5] * A[14] - 
               A[8]  * A[6] * A[13] - 
               A[12] * A[5] * A[10] + 
               A[12] * A[6] * A[9];

    *det = A[0] * inv[0] + A[1] * inv[1] + A[2] * inv[2] + A[3] * inv[3];
}

/**
*  @b Description
*  @n
*		This function checks whether measurement point P is geometrically behind the target
*
*  @param[in]  p
*		Pointer to measurements (spheriacl form) vector
*  @param[in]  uC
*		Pointer to measurements (spheriacl form) vector, representing target centroid
*  @param[in]  spread
*		Pointer to measurements (spheriacl form) vector, representing estimation of target spread
*  @retval
*      1 if behind, 0 is otherwise
*/
uint8_t gtrack_isPointBehindTarget(GTRACK_measurement_vector *p, GTRACK_measurement_vector *uC, GTRACK_measurement_vector *spread)
{
    if( (fabsf(p->azimuth - uC->azimuth) < spread->azimuth/2) && 
        (fabsf(p->elev - uC->elev) < spread->elev/2) && 
        (fabsf(p->doppler - uC->doppler) < 2*spread->doppler) && 
        (p->range > uC->range))        
        return 1U;
    else
        return 0;
}

/**
*  @b Description
*  @n
*		This function checks whether angular error is less than angular spread
*
*  @param[in]  aError
*		Pointer to measurements (spheriacl form) vector
*  @param[in]  spread
*		Pointer to measurements (spheriacl form) vector, representing estimation of target spread
*  @retval
*      1 if behind, 0 is otherwise
*/
uint8_t gtrack_isInsideSolidAngle(GTRACK_measurement_vector *aError, GTRACK_measurement_vector *spread)
{
    if((fabs(aError->azimuth) < spread->azimuth/2) &&
       (fabs(aError->elev) < spread->elev/2))
        return 1U;
    else
        return 0;
}

/**
*  @b Description
*  @n
*		This function checks whether the point is inside the box boundary or not
*
*  @param[in]  c
*		pointer to a position in cartesian space
*  @param[in] box
*		pointer to a 3D box object
*
*  \ingroup GTRACK_ALG_UTILITY_FUNCTION
*
*  @retval
*      1 if inside, 0 otherwise
*/
uint8_t gtrack_isPointInsideBox(GTRACK_cartesian_position *c, GTRACK_boundaryBox *box)
{
    if( (c->posX > box->x1) && (c->posX < box->x2) && 
        (c->posY > box->y1) && (c->posY < box->y2) && 
        (c->posZ > box->z1) && (c->posZ < box->z2) )
        return 1U;
    else
        return 0;
}
/**
*  @b Description
*  @n
*		This function checks whether measurement point is within the boresigh static zone
*
*  @param[in]  v
*		Pointer to measurements (spheriacl form) vector
*  @retval
*      1 if inside, 0 is otherwise
*/
uint8_t gtrack_isInsideBoresightStaticZone(GTRACK_measurement_vector *v)
{
    if( (v->range > GTRACK_BORESIGHT_STATIC_RANGE) &&
        (fabs(v->azimuth) < GTRACK_BORESIGHT_STATIC_ANGLE) &&
        (fabs(v->elev) < GTRACK_BORESIGHT_STATIC_ANGLE)) 
        return 1U;
    else
        return 0;
}

#endif
