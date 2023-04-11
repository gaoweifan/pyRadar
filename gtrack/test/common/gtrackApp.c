/*
 *   @file  gtrackApp.c
 *
 *   @brief
 *      Gtrack Application Unit Test code
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* mmWave SDK/PDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/utils/testlogger/logger.h>

#include <kernel/dpl/DebugP.h>
#include "FreeRTOS.h"
#include "task.h"
#ifdef SUBSYS_MSS
#include <ti/alg/gtrack/test/usecases/mss/mssgenerated/ti_drivers_open_close.h>
#include <ti/alg/gtrack/test/usecases/mss/mssgenerated/ti_board_open_close.h>
#else
#include <ti/alg/gtrack/test/usecases/dss/dssgenerated/ti_drivers_open_close.h>
#include <ti/alg/gtrack/test/usecases/dss/dssgenerated/ti_board_open_close.h>
#endif

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <ti/alg/gtrack/gtrack.h>
#include "math.h"
#include "float.h"
#include "testlimits.h"

extern far void* _gtrackLibStart;
extern far void* _gtrackLibEnd;


/* Heap memory, recommend to align to HeapP_BYTE_ALIGNMENT */
#define HEAP_MEM_SIZE  (32*1024u)
uint8_t gHeapMem[HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/* Heap handle */
HeapP_Object gHeapObj;

/* This test application wants to modify default parameters */
GTRACK_sceneryParams appSceneryParams = 
{
    /* sensor Position is (0,0,2) */
    .sensorPosition.x = 0.f,
    .sensorPosition.y = 0.f,
    .sensorPosition.z = 2.0f,

    /* Sensor orientation is (15 degrees down, 0 degrees azimuthal tilt */
    .sensorOrientation.azimTilt = 15.0f,
    .sensorOrientation.elevTilt = 0.f,

    .numBoundaryBoxes = 1,

    /* one boundary box {x1,x2,y1,y2,z1,z2} */
    .boundaryBox[0].x1 = -4.0f,
    .boundaryBox[0].x2 = 4.0f,
    .boundaryBox[0].y1 = 0.5f,
    .boundaryBox[0].y2 = 7.5f,
    .boundaryBox[0].z1 = 0.f,
    .boundaryBox[0].z2 = 3.0f,

    .boundaryBox[1].x1 = 0.f,
    .boundaryBox[1].x2 = 0.f,
    .boundaryBox[1].y1 = 0.f,
    .boundaryBox[1].y2 = 0.f,
    .boundaryBox[1].z1 = 0.f,
    .boundaryBox[1].z2 = 0.f,

    .numStaticBoxes = 1,

    /* one static box {x1,x2,y1,y2,z1,z2} */
    .staticBox[0].x1 = -3.0f,
    .staticBox[0].x2 = 3.0f,
    .staticBox[0].y1 = 2.0f,
    .staticBox[0].y2 = 6.0f,
    .staticBox[0].z1 = 0.5f,
    .staticBox[0].z2 = 2.5f,

    .staticBox[1].x1 = 0.f,
    .staticBox[1].x2 = 0.f,
    .staticBox[1].y1 = 0.f,
    .staticBox[1].y2 = 0.f,
    .staticBox[1].z1 = 0.f,
    .staticBox[1].z2 = 0.f,
};

/* Gating Gain 8x, Limits are set to 2m in depth, width, 2m (if applicable) in height and no limits in doppler */
GTRACK_gatingParams appGatingParams = 
{
    .gain = 3.f,

    .limitsArray[0] = 1.5f,
    .limitsArray[1] = 1.5f,
    .limitsArray[2] = 2.f,
    .limitsArray[3] = 10.f,
};

GTRACK_stateParams appStateParams = {
     10U, 5U, 50U, 100U, 5U              /* det2act, det2free, act2free, stat2free, exit2free */
};

GTRACK_allocationParams appAllocationParams = {
     0.f, 200.f, 0.1f, 6U, 1.5f, 2.f           /* 60 in clear, 200 obscured SNRs, 0.1m/s minimal velocity, 5 points, 1.5m in distance, 2m/s in velocity */
};
GTRACK_presenceParams appPresenceDetectionParams = {
    3U, 0.5f, 10U, 1,{{-3.0f,3.0f,2.0f,6.0f,0.5f,2.5f}, {0.f,0.f,0.f,0.f,0.f,0.f}}
};

typedef struct {
    uint32_t    frameNum;
    uint32_t    numTLVs;

} binFileHeader;

typedef enum {
    POINT_CLOUD_TYPE = 6,
    TARGET_LIST_TYPE
} binFileElements;

typedef struct {
    uint32_t type;
    uint32_t length;
} binFileTLV;

typedef struct {
    uint32_t tid;
    float S[GTRACK_STATE_VECTOR_SIZE];
} checkTargetDescr;


GTRACK_measurementPoint pointCloud[GTRACK_NUM_POINTS_MAX];
GTRACK_targetDesc targetDescr[GTRACK_NUM_TRACKS_MAX];

extern far uint32_t memoryBytesUsed;

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void Test_initTask(void* args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Debug Message: */
    test_print ("******************************************\n");
    test_print ("Debug: Launching the GTRACK Test Application\n");
    test_print ("*****************************************\n");

    HeapP_construct(&gHeapObj, gHeapMem, HEAP_MEM_SIZE);

	GTRACK_moduleConfig config;
    GTRACK_advancedParameters advParams;


    checkTargetDescr checkDescr[20];

    void *hTrackModule;
    bool testResult = true;
    bool performanceTestResultPass = false;
    bool frameError = false;

    uint32_t gtrackStartTime;
    uint32_t benchmarkCycles;
    uint32_t benchmarks[GTRACK_BENCHMARK_SIZE+1];

    int32_t errCode;

    binFileHeader frameHeader;
    uint32_t fileNumber;
    size_t result;
	
	uint32_t gtick;	
    uint16_t mNum;
    uint16_t tNum;
    uint8_t  presence;
    uint16_t tCheckNum;

	uint32_t n, k;

	FILE *fCloud;
	char fileName[120];
    binFileTLV tlv;
    float distX,distY;
    bool tidFound;
    GTRACK_boundaryBox *box;

    uint32_t programBytesUsed;

    uint32_t benchmarkPerTrack;
    uint64_t benchmarkPerTrackTotal = 0;
    uint32_t benchmarkPerTrackCount = 0;
    uint32_t benchmarkPerTrackAve = 0;
    uint32_t benchmarkPerTrackMax = 0;
    uint32_t benchmarkPerTrackMin = 0xFFFFFFFF;

    uint32_t cyclesPerSecond;
    uint32_t cyclesPerCCNT;
    float uSecondsPerCycle;

#if 1
    uint32_t start,end;
    start = (uint32_t)&_gtrackLibStart;
	end = (uint32_t)&_gtrackLibEnd;
    programBytesUsed = end - start;
#endif

	MCPI_Initialize();

	memset((void *)&config, 0, sizeof(GTRACK_moduleConfig));
#ifdef GTRACK_3D
    test_print("Gtrack is configured for 3D\n");
	config.stateVectorType = GTRACK_STATE_VECTORS_3DA; // Track three dimensions with acceleration 
#else
    test_print("Gtrack is configured for 2D\n");
	config.stateVectorType = GTRACK_STATE_VECTORS_2DA; // Track two dimensions with acceleration 
#endif
	config.verbose = GTRACK_VERBOSE_NONE;
	config.deltaT = 0.05f; // 50ms frames
	config.maxRadialVelocity = 5.29f; // Radial velocity from sensor is limited to +/- maxURV (in m/s)
	config.radialVelocityResolution = 0.083f; // Radial velocity resolution (in m/s)
	config.maxAcceleration[0] = 0.1f; // Target acceleration is not excedding 0.5 m/s2 in lateral direction
	config.maxAcceleration[1] = 0.1f; // Target acceleration is not excedding 0.5 m/s2 in longitudinal direction
	config.maxAcceleration[2] = 0.1f; // Target acceleration is not excedding 0.5 m/s2 in vertical direction
	config.maxNumPoints = 250;
	config.maxNumTracks = 20;
	config.initialRadialVelocity = 0; // Expected target radial velocity at the moment of detection, m/s
	
	
    /* Here, we want to set allocation, gating, and threshold parameters, leaving the rest to default */
	memset((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
	advParams.allocationParams = &appAllocationParams;
	advParams.gatingParams = &appGatingParams;
	advParams.stateParams = &appStateParams;
    advParams.sceneryParams = &appSceneryParams;
    advParams.presenceParams = &appPresenceDetectionParams;

	config.advParams = &advParams;

    test_print("Tracker Configuration\n");
    test_print("\tstateVectorType: %d\n", config.stateVectorType);
    test_print("\tmaxNumPoints: %d\n", config.maxNumPoints);
    test_print("\tmaxNumTracks: %d\n", config.maxNumTracks);
    test_print("\tmaxRadialVelocity: %f\n", config.maxRadialVelocity);
    test_print("\tradialVelocityResolution: %f\n", config.radialVelocityResolution);
    test_print("\tdeltaT: %f\n", config.deltaT);
	
    test_print("\tinitialRadialVelocity: %f\n", config.initialRadialVelocity);
    test_print("\tmaxAcceleration: [%f, %f, %f]\n", config.maxAcceleration[0], config.maxAcceleration[1], config.maxAcceleration[2]);
	
    test_print("\tscenery:\n");
    for (n = 0; n < config.advParams->sceneryParams->numBoundaryBoxes; n++) {
        box = &config.advParams->sceneryParams->boundaryBox[n];
        test_print("\t\t BoundaryBox %d: [%f, %f, %f, %f, %f, %f]\n", n, box->x1, box->x2, box->y1, box->y2,box->z1, box->z2);
    }
    for (n = 0; n < config.advParams->sceneryParams->numBoundaryBoxes; n++) {
        box = &config.advParams->sceneryParams->staticBox[n];
        test_print("\t\t StaticBox %d: [%f, %f, %f, %f, %f, %f]\n", n, box->x1, box->x2, box->y1, box->y2,box->z1, box->z2);
    }
    
    printf("\tpresence: [%d, %f, %d]\n", config.advParams->presenceParams->pointsThre, config.advParams->presenceParams->velocityThre, config.advParams->presenceParams->on2offThre);
    for (n = 0; n < config.advParams->sceneryParams->numBoundaryBoxes; n++) {
        box = &config.advParams->presenceParams->occupancyBox[n];
        printf("\t\t OccupancyBox %d: [%f, %f, %f, %f, %f, %f]\n", n, box->x1, box->x2, box->y1, box->y2,box->z1, box->z2);
    }
    
    test_print("\tallocation: [%f, %f, %f, %d, %f, %f]\n", config.advParams->allocationParams->snrThre, config.advParams->allocationParams->snrThreObscured, config.advParams->allocationParams->velocityThre, config.advParams->allocationParams->pointsThre, config.advParams->allocationParams->maxDistanceThre, config.advParams->allocationParams->maxVelThre);
    test_print("\tgating: [%f, %f, %f, %f, %f]\n", config.advParams->gatingParams->gain, config.advParams->gatingParams->limitsArray[0], config.advParams->gatingParams->limitsArray[1], config.advParams->gatingParams->limitsArray[2], config.advParams->gatingParams->limitsArray[3]);
    test_print("\tthresholds: [%d, %d, %d, %d, %d]\n", config.advParams->stateParams->det2actThre, config.advParams->stateParams->det2freeThre, config.advParams->stateParams->active2freeThre, config.advParams->stateParams->static2freeThre, config.advParams->stateParams->exit2freeThre);
	
	hTrackModule = gtrack_create(&config, &errCode);

	// Cycleprofiler_init();
    CycleCounterP_reset();
	gtrackStartTime = gtrack_getCycleCount();
	for(n=0; n<1000; n++) {
        /* one milisecond */
        ClockP_usleep(1 *1000);
	}
	cyclesPerSecond = gtrack_getCycleCount() - gtrackStartTime;
    test_print("Calibration: 1 seconds is %u Cycless\n", cyclesPerSecond);
    uSecondsPerCycle = 1000000.f/cyclesPerSecond;

    gtrackStartTime = gtrack_getCycleCount();
    cyclesPerCCNT = gtrack_getCycleCount() - gtrackStartTime;
    test_print("Calibration: CCNT read is %u Cycles\n", cyclesPerCCNT);


    fileNumber = 1;
	
    while (1) {
#ifdef GTRACK_3D
		sprintf(fileName, "../../../test/vectors/usecases/people_counting/fHistScene4_3d_%04u.bin", fileNumber);
#else
        sprintf(fileName, "../../../test/vectors/usecases/people_counting/fHistScene4_2d_%04u.bin", fileNumber);
#endif
		fCloud = fopen(fileName, "rb");
        if(fCloud == 0)
            break;

		if(fileNumber == 1) {
            performanceTestResultPass = true;
        }

        while(1) {
            // Read frame Header
            result = fread(&frameHeader, sizeof(frameHeader), 1, fCloud);
            if(result != 1)
                break;

            frameError = false;
            mNum = 0;
            tCheckNum = 0;

            for(n = 0; n < frameHeader.numTLVs; n++) {
                result = fread(&tlv, 8, 1, fCloud);
                if(result != 1)
                    break;

                switch(tlv.type) {
                    case 6:
                        result = fread(pointCloud, tlv.length, 1, fCloud);
                        if(result != 1)
                            break;

                        mNum = (uint16_t)tlv.length/sizeof(GTRACK_measurementPoint);
                        break;

                    case 7:
                        result = fread(checkDescr, tlv.length, 1, fCloud);
                        if(result != 1)
                            break;
                        tCheckNum = (uint16_t)tlv.length/sizeof(checkTargetDescr);
                        break;

                    default:
                        break;
                }
            }

            gtick = frameHeader.frameNum;

            // Limit the points
            if(mNum > config.maxNumPoints)
                mNum = config.maxNumPoints;

            gtrackStartTime = gtrack_getCycleCount();
            if(hTrackModule != NULL)
            {
                gtrack_step(hTrackModule, pointCloud, 0, mNum, targetDescr, &tNum, 0, 0, &presence, benchmarks);
            }

            if(tCheckNum != tNum) {
                test_print("Error, Frame #%u: missmatched number of targets\n", gtick);
                performanceTestResultPass = false;
                frameError = true;
            }
            for(n=0; n < tNum; n++) {
                tidFound = false;
                for(k=0; k < tNum; k++) {
                    if(targetDescr[n].uid == checkDescr[k].tid) {
                        tidFound = true;
                        break;
                    }
                }

                if(tidFound == false) {
                    test_print("Error, Frame #%u: tid not found\n", gtick);
                    performanceTestResultPass = false;
                    frameError = true;
                    break;
                }

                distX = targetDescr[n].S[0]-checkDescr[k].S[0];
                distY = targetDescr[n].S[1]-checkDescr[k].S[1];
                if(sqrt(distX*distX+distY*distY) > 2) {
                    test_print("Error, Frame #%u: missmatched distance\n", gtick);
                    performanceTestResultPass = false;
                    frameError = true;
                    break;
                }
            }

            benchmarkCycles = gtrack_getCycleCount() - gtrackStartTime;

            if(tNum) {
                benchmarkPerTrack = (uint32_t)benchmarkCycles/tNum;
                benchmarkPerTrackTotal += benchmarkPerTrack;
                benchmarkPerTrackCount += 1;

                if(benchmarkPerTrack < benchmarkPerTrackMin)
                    benchmarkPerTrackMin = benchmarkPerTrack;
                if(benchmarkPerTrack > benchmarkPerTrackMax)
                    benchmarkPerTrackMax = benchmarkPerTrack;
            }
            test_print("Frame #%u, %u Targets, %u Points, %u Tracks, %u Cycles = (", gtick, tCheckNum, mNum, tNum, benchmarkCycles);
            for(n=0; n<GTRACK_BENCHMARK_SIZE; n++) {
                test_print("%u, ", benchmarks[n] - gtrackStartTime);
                gtrackStartTime = benchmarks[n];

            }
            test_print("%u)", benchmarks[n] - gtrackStartTime);

            if(frameError)
                test_print("... ERROR\n");
            else
                test_print("... OK\n");
        }
        fileNumber ++;
        fclose(fCloud);
    }

    if(fileNumber == 1) {
        test_print("ERROR: No valid files found\n");
        testResult = false;
    }

    gtrack_delete(hTrackModule);

    test_print("**************************************************************************************************************\n");
#ifdef GTRACK_3D        
    test_print("Tracking in 3D space\n");
#else
    test_print("Tracking in 2D space\n");
#endif

    test_print("Configured for %d maxPoints and %d maxTracks\n", config.maxNumPoints, config.maxNumTracks);
    test_print("Gtrack Library is using %d bytes of program memory\n", programBytesUsed);

    if (benchmarkPerTrackCount>0) 
    {
        benchmarkPerTrackAve = (uint32_t)(benchmarkPerTrackTotal/benchmarkPerTrackCount);
        test_print("Cycles, per target {mean, min, max} = %u, %u, %u\n", benchmarkPerTrackAve, benchmarkPerTrackMin, benchmarkPerTrackMax);
        test_print("uSec, per target {mean, min, max} = %u, %u, %u\n", (uint32_t)(benchmarkPerTrackAve*uSecondsPerCycle), (uint32_t)(benchmarkPerTrackMin*uSecondsPerCycle), (uint32_t)(benchmarkPerTrackMax*uSecondsPerCycle));
    }
    else
    {
        test_print("ERROR: No benchmark results produced\n");
        testResult = false;
    }

	/* Test Results */
    /* Performance Test */
	if(performanceTestResultPass != true) {
        test_print("ERROR: Tracking performance does not match with reference\n");
        testResult = false;
	}
	else
        test_print("PERFORMANCE TEST PASSED\n");

	/* Program memory check */
    if(programBytesUsed > LIMIT_PROG_MEMORY_SIZE)
        test_print("Warning: Program memory size (%d) check exceeds configured limit\n", programBytesUsed);
    else
        test_print("PROGRAM MEMORY SIZE TEST PASSED\n");

#if 0
	/* Run time benchmark check */
	if((benchmarkPerTrackAve > LIMIT_USEC_PER_TRACK_AVE/uSecondsPerCycle) || (benchmarkPerTrackMax > LIMIT_USEC_PER_TRACK_MAX/uSecondsPerCycle))
        test_print("Warning: Run time benchmark check exceeds configured limits\n");
    else
        test_print("BENCHMARK TEST PASSED\n");
#endif

	if(testResult == false) {
        test_print("TEST FAILED\n");
    	MCPI_setFeatureTestResult("Group Tracker", MCPI_TestResult_FAIL);	
	}
	else {
        test_print("ALL TEST PASSED\n");
    	MCPI_setFeatureTestResult("Group Tracker", MCPI_TestResult_PASS);	
	}
	test_print("**************************************************************************************************************\n");
    
	MCPI_setTestResult();

    Board_driversClose();

    vTaskDelete(NULL);
    return;
}
