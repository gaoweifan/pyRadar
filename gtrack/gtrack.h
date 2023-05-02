/**
 *   @file  gtrack.h
 *
 *   @brief
 *      This is the header file for the GTRACK Algorithm
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

/** @mainpage GTRACK Algorithm
 *	This code is an implementation of Group TRACKing algorithm.<br>
 *	The algorithm is designed to track multiple targets, where each target is represented by a set of measurement points.<br> 
 *	Each measurement point carries detection informations, for example, range, azimuth, elevation (for 3D option), and radial velocity.<br> 
 *	Instead of tracking individual reflections, the algorithm predicts and updates the location and dispersion properties of the group.<br>
 *	The group is defined as the set of measurements (typically, few tens; sometimes few hundreds) associated with a real life target.<br>
 *	Algorithm supports tracking targets in two or three dimensional spaces as a build time option:<br>
 * 	- When built with 2D option, algorithm inputs range/azimuth/doppler information and tracks targets in 2D cartesian space.
 * 	- When built with 3D option, algorithm inputs range/azimuth/elevation/doppler information and tracks targets in 3D cartesian space.
 *
 *  ### Input/output
 *	- Algorithm inputs the Point Cloud. For example, few hundreds of individual measurements (reflection points).<br>
 *	- Algorithm outputs a Target List. For example, an array of few tens of target descriptors. Each descriptor carries a set of properties for a given target.<br>
 *	- Algorithm optionally outputs a Target Index. If requested, this is an array of target IDs associated with each measurement.<br>
 *
 *  ### Features
 *	- Algorithm uses extended Kalman Filter to model target motion in Cartesian coordinates.<br>
 *	- Algorithm supports constant velocity and constant acceleartion models.<br>
 *	- Algorithm uses 3D/4D Mahalanobis distances as gating function and Max-likelihood criterias as scoring function to associate points with an existing track.<br>
 *
 *  ## External API
 *  Application includes the following algorithm header
 *  @code
    #include <ti/alg/gtrack.h>
 *  @endcode
 *	All resources are allocated at create time during the \ref gtrack_create call. <br>
 *	All resources are freed at delete time time during the \ref gtrack_delete call. <br>
 *  Application is expected to implement the design pattern as described in the pseudo code bellow:
 *  @code
    h = gtrack_create(params);                      // Creates an instance of the algorithm with a desired configuration
    while(running) {
        gtrack_step(h, pointCloud, &targetList);    // Runs a single step of the given alrorithm instance with input point cloud data
    }
    gtrack_delete(h);                               // Delete the algorithm instance
 *  @endcode
 *  ### Dependencies
 *  Library is platform independent. To port the library, platform shall implement few functions that Library abstracts, see \ref GTRACK_Dependencies
 *
 *  ## Internal Architecture of the GTRACK Algorithm
 *  Algorithm is implemented with two internal software sublayers: Module and Units(s). <br>
 *  Application can create multiple Module instances with different configuration parameters (for example, to track different classes of targets).<br>
 *  Each Module instance creates amd manages multiple units.<br>
 *  Each Unit represents a single tracking object.<br> 
 *  Units inherit configuration parameters from the parent Module.<br>
 *  ### Module level
 *  The imlementation of \ref gtrack_create function creates a module instance and pre-allocates resources for a maximum number of units.<br> 
 *  The imlementation of \ref gtrack_step function calls one single round of module functions as illustrated in the pseudo code below:
 *  @code
    gtrack_step(h, pointCloud, &targetList) {
        gtrack_modulePredict(h,...);
        gtrack_moduleAssociate(h,...);
        gtrack_moduleAllocate(h,...);
        gtrack_moduleUpdate(h,...);
        gtrack_modulePresence(h,...);
        gtrack_moduleReport(h,...);
    }
 *  @endcode
 *  The imlementation of \ref gtrack_delete function returns all the resources back to the system
 *  ### Unit level
 *  Units are created during \ref gtrack_moduleAllocate calls.<br>
 *  The resources pre-allocated at module create time are assigned to a new unit.<br>
 *  All active units are called during each parent module function step calls.<br>
 *  As an example, the pseudo code below illustrates the implementation of the predict function:
 *  @code
    gtrack_modulePredict(h,...) {
        for(each active unit) {
            gtrack_unit_predict(unit, ...);
        }
    }
 *  @endcode
 *  Units are deleted during \ref gtrack_moduleUpdate calls. The resources are returned back to the parent module.
 */
#ifndef GTRACK_H
#define GTRACK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


#if !defined (GTRACK_2D) && !defined (GTRACK_3D)
#define GTRACK_3D
#endif

/** @defgroup GTRACK_ALG      GTRACK Algorithm Library
@brief
*	Library implementation of tracking algorithm
*/

/** @defgroup GTRACK_External	Externals
@ingroup GTRACK_ALG
*/

/** @defgroup GTRACK_Dependencies	Dependencies
@ingroup GTRACK_ALG
@brief
*	Functions used by GTRACK modules. Shall be implemented by porting platform layers 
*/

/** @defgroup GTRACK_Internal	Internals
@ingroup GTRACK_ALG
*/

/**
@defgroup GTRACK_ALG_EXTERNAL_FUNCTION            External Functions
@ingroup GTRACK_External
@brief
*	External functional API, called by Application
*/

/**
@defgroup GTRACK_ALG_EXTERNAL_DATA_STRUCTURE      External Data Structures
@ingroup GTRACK_External
@brief
*   Data structures exposed to Application
*/

/**
@defgroup GTRACK_ALG_MODULE_FUNCTION    Module Level Functions
@ingroup GTRACK_Internal
@brief
*   Internal MODULE level API, called by External Functions
*/

/**
@defgroup GTRACK_ALG_UNIT_FUNCTION    Unit Level Functions
@ingroup GTRACK_Internal
@brief
*   Internal UNIT level API, called by MODULE
*/
/**
@defgroup GTRACK_ALG_UTILITY_FUNCTION	Utility Functions
@ingroup GTRACK_Internal
@brief
*   Utility functions used internally
*/

/**
@defgroup GTRACK_ALG_MATH_FUNCTION	Math Functions
@ingroup GTRACK_Internal
@brief
*   Math functions used internally
*/

/**
@defgroup GTRACK_ALG_INTERNAL_DATA_STRUCTURE	Internal Data Structures
@ingroup GTRACK_Internal
@brief
*   Internal data structures
*/

/**
@defgroup GTRACK_ERROR_CODE            Algorithm Error codes
@ingroup GTRACK_External
@brief
*   Error codes reported to Application
@{ */

/**
 * @brief   Base error code for GTRACK algorithm
 */
#define GTRACK_ERRNO_BASE		(-8000)

/**
 * @brief   Error code: No errors
 */
#define GTRACK_EOK				(0)

/**
 * @brief   Error Code: Invalid argument
 */
#define GTRACK_EINVAL			(GTRACK_ERRNO_BASE-1)

/**
 * @brief   Error Code: Operation cannot be implemented because a previous
 * operation is still not complete.
 */
#define GTRACK_EINUSE			(GTRACK_ERRNO_BASE-2)

/**
 * @brief   Error Code: Operation is not implemented.
 */
#define GTRACK_ENOTIMPL			(GTRACK_ERRNO_BASE-3)

/**
 * @brief   Error Code: Out of memory
 */
#define GTRACK_ENOMEM			(GTRACK_ERRNO_BASE-4)

/** @}*/

/** 
@addtogroup GTRACK_ALG_EXTERNAL_DATA_STRUCTURE
@{ */

/**
 * @name Maximum supported configurations
 * @{ */
#define GTRACK_NUM_POINTS_MAX			(1000U) /**< @brief Defines maximum possible number of measurments point the algorithm will accept at configuration time */
#define GTRACK_NUM_TRACKS_MAX			(200U)	/**< @brief Defines maximum possible number of tracking target the algorithm will accept at configuration time */
/** @} */

/**
 * @name Target ID definitions
 * @{ 
 * @details
 *  Target IDs are uint8_t, with valid IDs ranging from 0 to 249. Other values as defined below
 */
#define GTRACK_ID_GHOST_POINT_LIKELY        (250U)   /**< @brief Point is associated to the ghost right behind the target as single multipath reflection */
#define GTRACK_ID_GHOST_POINT_BEHIND        (251U)   /**< @brief Point is associated to the ghost behind the target further away via multiple reflection paths */
#define GTRACK_ID_RESERVED_GOOD_POINT       (252U)   /**< @brief Point definition is reserved */
#define GTRACK_ID_POINT_TOO_WEAK            (253U)   /**< @brief Point is not associated, is too weak */
#define GTRACK_ID_POINT_BEHIND_THE_WALL     (254U)   /**< @brief Point is not associated, is behind the wall */
#define GTRACK_ID_POINT_NOT_ASSOCIATED      (255U)   /**< @brief Point is not associated, noise */
/** @} */

/**
 * @name Benchmarking results
 * @{  
 * @details
 *  During runtime execution, tracking step function can optionally return cycle counts for the sub-functions defined below
 *  Each count is 32bit unsigned integer value representing a timestamp of free runing clock 
 */
#define GTRACK_BENCHMARK_SETUP              (0U)    /**< @brief Cycle count at step setup */
#define GTRACK_BENCHMARK_PREDICT            (1U)    /**< @brief Cycle count after predict function */
#define GTRACK_BENCHMARK_ASSOCIATE          (2U)    /**< @brief Cycle count after associate function */
#define GTRACK_BENCHMARK_ALLOCATE           (3U)    /**< @brief Cycle count after allocate function */
#define GTRACK_BENCHMARK_UPDATE             (4U)    /**< @brief Cycle count after update function */
#define GTRACK_BENCHMARK_PRESENCE           (5U)    /**< @brief Cycle count after presence detection function */
#define GTRACK_BENCHMARK_REPORT             (6U)    /**< @brief Cycle count after report function */

#define GTRACK_BENCHMARK_SIZE               (GTRACK_BENCHMARK_REPORT)    /**< @brief Size of benchmarking array */
/** @} */

#define GTRACK_MIN(x,y) ((x) < (y) ? (x) : (y))
#define GTRACK_MAX(x,y) ((x) > (y) ? (x) : (y))

#ifdef GTRACK_3D
#include "include/gtrack_3d.h"
#else
#include "include/gtrack_2d.h"
#endif
/**
 * @name Sensor position
 * @{  
 * @details
 *  Application can configure tracker with sensor position. Position is in cartesian space relative to the [3-dimentional] world.
 */
/**
 * @brief
 *  Sensor Position Structure
 *
 * @details
 *  The structure defines the sensor position 
 */
typedef struct {
    /**  @brief   X dimension (left-right), m */
	float x;
    /**  @brief   Y dimension (near-far), m */
	float y;
    /**  @brief   Z dimension (height), m */
	float z;
} GTRACK_sensorPosition; 
/** @} */

/**
 * @name Sensor orientation
 * @{  
 * @details
 *  Application can configure tracker with sensor orientation. Orientation is defined as boresight angular tilts.
 */
/**
 * @brief
 *  Sensor Orientation Structure
 *
 * @details
 *  The structure defines the sensor orientation 
 */
typedef struct {
    /**  @brief   Boresight azumuthal tilt, negative left, positive right, in degrees */
	float azimTilt;
    /**  @brief   Boresight elevation tilt, negative up, positive down, in degrees. The value of 90 +/-20 degrees will indicate the sensor is ceiling mount */
	float elevTilt;
} GTRACK_sensorOrientation; 
/** @} */

/**
 * @name Boundary boxes
 * @{  
 * @details
 *  Application can configure tracker with scene boundries. Boundaries are defined as a boxes.
 */
#define GTRACK_MAX_BOUNDARY_BOXES           (2U)    /**< @brief Maximum number of boundary boxes. Points outside of boundary boxes are ignored */
#define GTRACK_MAX_STATIC_BOXES             (2U)    /**< @brief Maximum number of static boxes. Targets inside the static box can persist longer */
#define GTRACK_MAX_OCCUPANCY_BOXES          (2U)    /**< @brief Maximum number of occupancy boxes. Presence detection algorithm will determine whether box is occupied */
/** @} */

/**
 * @brief
 *  GTRACK Box Structure
 *
 * @details
 *  The structure defines the box element used to describe the scenery 
 */
typedef struct {
    /**  @brief   Left boundary, m */
	float x1;
    /**  @brief   Right boundary, m */
	float x2;
    /**  @brief   Near boundary, m */
	float y1;
    /**  @brief   Far boundary, m */
	float y2;
    /**  @brief   Bottom boundary, m */
	float z1;
    /**  @brief   Top boundary, m */
	float z2;
} GTRACK_boundaryBox; 
/**
 * @brief
 *  GTRACK Gate Limits
 *
 * @details
 *  The structure describes the limits the gating function will expand
 */
typedef struct {
    /**  @brief   Depth limit, m */
	float depth;
    /**  @brief   Width limit, m */
	float width;
    /**  @brief   Heigth limit, m */
	float height;
    /**  @brief   Radial velocity limit, m/s */
	float vel;
} GTRACK_gateLimits; 

/**
 * @brief 
 *  GTRACK Scenery Parameters
 *
 * @details
 *  Scenery uses 3-dimensional Cartesian coordinate system, defined as W in the picture below <br>
 *  It is expected that the Z=0 plane corresponds to the scene floor <br> 
 *  The X coordinates are left (negative)-right; the Y ccordinates are near-far. <br>
 *  Origin (O) is typically colocated with senor projection to Z=0 plane <br>
 *
 *  - Sensor Position is 3 dimentioanl coordinate of the sensor <br>
 *    + For example, (0,0,2) will indicate that sensor is directly above the origin at the height of 2m <br>
 *  - Sensor Orientation is sensor's boresight rotation: down tilt (thetta) and azimuthal tilt (not supported) <br>
 *
 *  User can define up to \ref GTRACK_MAX_BOUNDARY_BOXES boundary boxes, and up to \ref GTRACK_MAX_STATIC_BOXES static boxes. <br>
 *  - Boundary Boxes are used to define area of interest. All reflection points outside the boundary area are ignored. <br>
 *    + For example, boundary box can be used to ignore the targets in the hallway outside the room, or to ignore potential ghost-behind-the-wall reflections <br>
 *  - Static Boxes defines the zone where targets are expected to become static for a long time. Typically, that area is a smaller (0.5-1.5m) then boundary <br>
 *    + When not directly configured by customer, the application makes box 0.5m smaller from each side <br>
 *    + When targets are within the static zone, and miss detection event occurs, the [larger] static threshold will apply for de-allocation. When outside the area, the [smaller] exit threshold will be applied <br>
 *    + Static reflection points outside the static area are ignored. <br>
 *
 *  _Scensor geometry and transformations_ \image html Geometry3D.jpg
 */

typedef struct {
    /**  @brief Sensor position, set to (0.f, 0.f) for 2D, set to (0.f, 0.f, H) for 3D. Where H is sensor height, in m */
    GTRACK_sensorPosition       sensorPosition;
    /**  @brief Sensor orientation, set to (0.f, 0.f) for 2D, (AzimTilt, ElevTilt) for 3D. Where AzimTilt and ElevTilt are rotations along Z and X axes correspondily */
    GTRACK_sensorOrientation    sensorOrientation;
    /**  @brief Number of scene boundary boxes. If defined (numBoundaryBoxes > 0), only points within the boundary box(s) can be associated with tracks */
    uint8_t             numBoundaryBoxes;
    /**  @brief Scene boundary boxes */
    GTRACK_boundaryBox  boundaryBox[GTRACK_MAX_BOUNDARY_BOXES];
    /**  @brief Number of scene static boxes. If defined (numStaticBoxes > 0), only targets within the static box(s) can persist as static */
    uint8_t             numStaticBoxes;
    /**  @brief Scene static boxes */
    GTRACK_boundaryBox  staticBox[GTRACK_MAX_STATIC_BOXES];
} GTRACK_sceneryParams;

/**
 * @brief 
 *  GTRACK Presence Detection Parameters
 *
 * @details
 *  This set of parameters describes the presence detection function <br>
 *  Presence is computed over the combined shape of occupancy boxes. Each box is described using 3-dimensional Cartesian coordinate system. It is expected that the Z=0 plane corresponds to the scene floor. The X coordinates are left (negative)-right; the Y ccordinates are near-far. Origin is typically colocated with senor projection to Z=0 plane.
 *  User can define up to \ref GTRACK_MAX_OCCUPANCY_BOXES occupancy boxes. <br>
 *  If any target exists with the occupancy area, the alrgorithm returns 1. Othewise, it returns 0. <br>
 *  The algorithm combines "raw detection" and "known target in the area" indications. <br>
 *  For "raw detection" indication re-uses the candidate set created by allocation process. It checks the occupancy thresholds: number of points in the set against the pointsThre and set's velocity against the velocityThre. <br>
 *  For "known target in the area" the algorithm checks whether known target measurment centroid is within the occupancy boxes. <br>
 */

typedef struct {
    /**  @brief  occupancy threshold, number of points. Setting pointsThre to 0 disables presence detection */
	uint16_t            pointsThre;
    /**  @brief  occupancy threshold, approaching velocity */
    float               velocityThre;
    /**  @brief  occupancy on to off threshold */
	uint16_t            on2offThre;
    /**  @brief Number of occulancy boxes. Presence detection algorithm will determine whether the combined shape is occupied. Setting numOccupancyBoxes to 0 disables presence detection */
    uint8_t             numOccupancyBoxes;
    /**  @brief Scene occupancy boxes */
    GTRACK_boundaryBox  occupancyBox[GTRACK_MAX_OCCUPANCY_BOXES];
} GTRACK_presenceParams;


/**
 * @brief 
 *  GTRACK Gating Function Parameters
 *
 * @details
 *  The structure describes gating function parameters
 */
typedef struct {
    /**  @brief   Gain of the gating function. <br>
     * It is set based on expected tracking errors and uncertainties of detection layer */
	float		gain;

    union {
    /**  @brief   Gating function limits. <br> 
     * It is based on physical dimensions and agility of the targets. <br> 
     * Setting it too small will result in allocating multiple tracks for the single object, setting it too big will cause allocating single track for multiple objects */
	    GTRACK_gateLimits	limits;
        float limitsArray[4];
    };
} GTRACK_gatingParams;

/**
 * @brief 
 *  GTRACK Tracking Management Function Parameters
 *
 * @details
 *  The structure describes the thresholds for state changing counters
 */
typedef struct {
    /**  @brief  DETECTION => ACTIVE threshold. This is a threshold for the number of continuous HITS to transition from DETECT to ACTIVE state */
	uint16_t det2actThre;
    /**  @brief  DETECTION => FREE threshold. This is a threshold for the number of continuous misses to transition from DETECT to FREE state */
    uint16_t det2freeThre;

    /**  @brief  ACTIVE => FREE threshold. This is a generic threshold for continuous misses in ACTIVE state when no special conditions (static2free/exit2free) apply. The corresponding counter is reset with either static or dynamic points associated */
    uint16_t active2freeThre;
    /**  @brief  ACTIVE & STATIC & STATIC_ZONE => FREE threshold. The threshold is for continuous misses for static target in static zone */
    uint16_t static2freeThre;
    /**  @brief  ACTIVE & !STATIC_ZONE => FREE threshold. This is a threshold for continuous misses for target outside of static zone.*/
	uint16_t exit2freeThre;
    /**  @brief  ACTIVE & STATIC & STATIC_ZONE => FREE threshold. This is a maximum time target can be STATIC. There is separate counter that is reset only with dynamic point associated. */
	uint16_t sleep2freeThre;

} GTRACK_stateParams;


/**
 * @brief 
 *  GTRACK Allocation Function Parameters
 *
 * @details
 *  The structure describes the thresholds used in Allocation function
 */
typedef struct {
    /**  @brief  Minimum total SNR of the allocation set-candidate. The threshold shall be roughly equal to the expected linear SNR value of the detection point at 6m range multiplied by pointsThre */
    float snrThre;
    /**  @brief  Minimum total SNR when behind another target. Set it larger then snrThre to require stronger SNR for the obscured allocations */
    float snrThreObscured;
    /**  @brief  Minimum initial velocity, m/s */
    float velocityThre;
    /**  @brief  Minimum number of points in a set */
	uint16_t pointsThre;
    /**  @brief  Maximum squared distance between points in a set */
    float	maxDistanceThre;
    /**  @brief  Maximum velocity delta between points in a set */
    float	maxVelThre;
} GTRACK_allocationParams;

/**
 * @brief 
 *  GTRACK Advanced Parameters
 *
 * @details
 *  The structure describes advanced configuration parameters
 */
typedef struct {
	/**  @brief  Pointer to gating parameters */
	GTRACK_gatingParams *gatingParams;
    /**  @brief  Pointer to allocation parameters */
	GTRACK_allocationParams *allocationParams;
    /**  @brief  Pointer to tracking state parameters */
	GTRACK_stateParams *stateParams;
    /**  @brief  Pointer to scenery parameters */
	GTRACK_sceneryParams *sceneryParams;
    /**  @brief  Pointer to presence detection parameters */
    GTRACK_presenceParams *presenceParams;
} GTRACK_advancedParameters;


/**
 * @brief 
 *  GTRACK State Vector
 *
 * @details
 *  Defines State vector options 
 *		2DV - Not supported		
 *		2DA - Supported
 *		3DV - Not supported,
 *      3DA - Supported
 */
typedef enum
{
    /**  @brief   2D motion model with constant velocity. State vector has four variables S={X,Y, Vx,Vy} */
	GTRACK_STATE_VECTORS_2DV = 0,
    /**  @brief   2D motion model with constant acceleration. State vector has six variables S={X,Y, Vx,Vy, Ax,Ay} */
	GTRACK_STATE_VECTORS_2DA,
    /**  @brief   3D motion model with constant velocity. State vector has six variables S={X,Y,Z, Vx,Vy,Vz} */
	GTRACK_STATE_VECTORS_3DV,
    /**  @brief   3D motion model with constant acceleration. State vector has nine variables S={X,Y,Z, Vx,Vy,Vz, Ax,Ay,Az} */
	GTRACK_STATE_VECTORS_3DA
} GTRACK_STATE_VECTOR_TYPE;


/**
 * @brief 
 *  GTRACK Verbose Level
 *
 * @details
 *  Defines Algorithm verboseness level 
 */
typedef enum
{
    /**  @brief   NONE */
	GTRACK_VERBOSE_NONE = 0,
    /**  @brief   ERROR Level, only errors are reported */
	GTRACK_VERBOSE_ERROR,
    /**  @brief   WARNING Level, errors and warnings are reported */
	GTRACK_VERBOSE_WARNING,
    /**  @brief   DEBUG Level, errors, warnings, and state transitions are reported */
	GTRACK_VERBOSE_DEBUG,
    /**  @brief   MATRIX Level, previous level plus all intermediate computation results are reported */
	GTRACK_VERBOSE_MATRIX,
    /**  @brief   MAXIMUM Level, maximum amount of details are reported */
	GTRACK_VERBOSE_MAXIMUM
} GTRACK_VERBOSE_TYPE;


/**
 * @brief 
 *  GTRACK Configuration
 *
 * @details
 *  The structure describes the GTRACK algorithm configuration options. 
 */
typedef struct
{
    /**  @brief   State Vector Type, Supported Types are 2DA, S={X,Y, Vx,Vy, Ax,Ay} and 3DA, S={X,Y,Z, Vx,Vy,Vz, Ax,Ay,Az} */
    GTRACK_STATE_VECTOR_TYPE stateVectorType;
    /**  @brief   Verboseness Level. <br>
     *            A bit mask representing levels of verbosity: NONE | WARNING | DEBUG | ASSOCIATION DEBUG | GATE_DEBUG | MATRIX DEBUG <br>
     *            Once event level is lower then requested verbosity level, Library generates a log by calling gtrack_log function */
    GTRACK_VERBOSE_TYPE verbose;
    /**  @brief   Maximum Number of Measurement Points per frame. Up to \ref GTRACK_NUM_POINTS_MAX supported <br>
     *            The library will allocate memories based on this parameter */
	uint16_t maxNumPoints;
    /**  @brief   Maximum Number of Tracking Objects. Up to \ref GTRACK_NUM_TRACKS_MAX supported <br>
     *            The library will allocate memories based on this parameter */
	uint16_t maxNumTracks;

    /**  @brief   Expected target radial velocity at the moment of detection, m/s */
    float initialRadialVelocity;
    /**  @brief   Maximum radial velocity reported by sensor +/- m/s */
	float maxRadialVelocity;
    /**  @brief   Radial Velocity resolution, m/s */
	float radialVelocityResolution;
    /**  @brief   Maximum expected target acceleration in lateral (X), longitudinal (Y), and vertical (Z) directions, m/s2 <br> 
     *            Used to compute processing noise matrix. For 2D options, the vertical component is ignored */
    float maxAcceleration[3];
    /**  @brief   Frame rate, ms */
    float deltaT;

    /**  @brief   Advanced parameters, set to NULL for defaults */
    GTRACK_advancedParameters *advParams;

} GTRACK_moduleConfig;

#define GTRACK_STATE_VECTOR_SIZE sizeof(GTRACK_state_vector_pos_vel_acc)/sizeof(float)
#define GTRACK_MEASUREMENT_VECTOR_SIZE sizeof(GTRACK_measurement_vector)/sizeof(float)

/**
 * @brief 
 *  GTRACK Measurement point
 *
 * @details
 *  The structure describes measurement point format
 */
typedef struct
{
    union {
    	/**  @brief   Measurement vector */
        GTRACK_measurement_vector vector;
        float array[GTRACK_MEASUREMENT_VECTOR_SIZE];
    };
	/**  @brief   Range detection SNR, linear */
    float snr;
} GTRACK_measurementPoint;

typedef union {
    /**  @brief   Measurement vector */
    GTRACK_measurement_vector vector;
    float array[GTRACK_MEASUREMENT_VECTOR_SIZE];
} GTRACK_measurementUnion;


/**
 * @brief 
 *  GTRACK target descriptor
 *
 * @details
 *  The structure describes target descriptorformat
 */
typedef struct
{
	/**  @brief   Tracking Unit Identifier */
	uint8_t uid;
	/**  @brief   Target Identifier */
	uint32_t tid;
	/**  @brief   State vector */
	float S[GTRACK_STATE_VECTOR_SIZE];
	/**  @brief   Group covariance matrix */
	float EC[GTRACK_MEASUREMENT_VECTOR_SIZE*GTRACK_MEASUREMENT_VECTOR_SIZE];
	/**  @brief   Gain factor */
	float G;
	/**  @brief   Estimated target dimensions: depth, width, [height], doppler */
	float dim[GTRACK_MEASUREMENT_VECTOR_SIZE];
	/**  @brief   Measurement Centroid range/angle/doppler */
	float uCenter[GTRACK_MEASUREMENT_VECTOR_SIZE];
	/**  @brief   Target confidence level */
    float confidenceLevel;

} GTRACK_targetDesc;

extern void *gtrack_create(GTRACK_moduleConfig *config, int32_t *errCode);
extern void gtrack_step(void *handle, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t mNum, GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint8_t *uIndex, uint8_t *presence, uint32_t *bench);
extern void gtrack_delete(void *handle);

/**
@} */

/* External dependencies */
/** @addtogroup GTRACK_Dependencies
 @{ */

/**
* @brief 
*  GTRACK calls this function to allocate memory. Expects the void pointer if allocation is sucessful, and NULL otherwise
*
*  @param[in]  numElements
*      Number of elements to allocate
*  @param[in]  sizeInBytes
*      Size of each element in bytes to allocate
*/
extern void *gtrack_alloc(uint32_t numElements, uint32_t sizeInBytes);

/**
* @brief 
*  GTRACK calls this function to free memory
*
*  @param[in]  pFree
*      Pointer to a memmory to free
*  @param[in]  sizeInBytes
*      Size of memory in bytes to free
*/
extern void gtrack_free(void *pFree, uint32_t sizeInBytes);

/* For Matlab MEX environment, redefine gtrack_log with mexPrintf */
#ifdef _MEX_
	#define gtrack_log(level, format, ...) mexPrintf(format, ##__VA_ARGS__);
	extern int mexPrintf(const char *format, ...);
#else
/**
* @brief 
*  GTRACK calls this function to log the events
*
*  @param[in]  level
*      Level is the event importance
*  @param[in]  format
*      Format is the variable size formated output
*/
	extern void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...);
#endif
/**@}*/

/* This is windows API include (ex. for memcpy) */
#ifdef _WIN32
#include <windows.h>
#endif

/* This is inline implementation of gtrack_assert */
#if defined (_WIN32) || defined (__linux__)
#include <assert.h>
#define gtrack_assert(expression) assert(expression)
#endif

#if defined (SUBSYS_MSS) || defined (SUBSYS_DSS)
//#include <ti/osal/DebugP.h>
#include <kernel/dpl/DebugP.h>
#define gtrack_assert(expression) DebugP_assert(expression)
#endif

/* This is inline implementation of getCycleCount */
#if defined (_WIN32) || defined (__linux__)
#ifdef _WIN32
#include <intrin.h>
#endif
static __inline uint32_t gtrack_getCycleCount(void){
  #ifdef __aarch64__
  uint64_t tsc;
  asm volatile("mrs %0, cntvct_el0" : "=r" (tsc));
  return (uint32_t)tsc;
  #else
  return (uint32_t)__rdtsc();
  #endif
}
#endif

/* This defines boolean type for VS2012 and below */
#if defined _WIN32
#ifndef __bool_true_false_are_defined
#define	__bool_true_false_are_defined	1

#ifndef false
#define	false	0
#endif

#ifndef true
#define	true	1
#endif

// #define	bool	_Bool
// typedef unsigned char _Bool;

#endif
#endif

#ifdef SUBSYS_MSS
#define	far		/* nothing */
#define	near	/* nothing */
// #if defined (__GNUC__) && !defined(__ti__)
static inline uint32_t gtrack_getCycleCount (void)
{
    uint32_t value;
    // Read CCNT Register
    asm volatile ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(value));
    return value;
}
// #else
// #define gtrack_getCycleCount() __MRC(15, 0, 9, 13, 0)
// #endif
#endif

#ifdef SUBSYS_DSS
#include <c6x.h>
#define  gtrack_getCycleCount() TSCL
#endif

#ifdef __cplusplus
}
#endif

#endif /* GTRACK_H */
