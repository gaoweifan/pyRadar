/*
 *   @file  main_mss.c
 *
 *   @brief
 *      Unit Test code for the GTRACK
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

/* MCU+SDK Include Files. */
#include <kernel/dpl/DebugP.h>
#include <ti/alg/gtrack/test/usecases/mss/mssgenerated/ti_drivers_config.h>
#include <ti/alg/gtrack/test/usecases/mss/mssgenerated/ti_board_config.h>
#include "FreeRTOS.h"
#include "task.h"

/* mmWave SDK/PDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/utils/testlogger/logger.h>

#include <ti/alg/gtrack/gtrack.h>

/* FreeRTOS Task declarations. */
#define APP_TASK_PRI         (5U)
#define APP_TASK_STACK_SIZE  (12*1024U)

TaskHandle_t    gAppTask;
StaticTask_t    gAppTaskObj;

StackType_t gAppTskStackMain[APP_TASK_STACK_SIZE] __attribute__((aligned(32)));
/**************************************************************************
 ************************** External Definitions **************************
 **************************************************************************/
extern void Test_initTask(void* args);

extern void asm_matrixMultiply66(float *A, float *B, float*C);
/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Initialize the MCPI Log Message Buffer
 */
MCPI_LOGBUF_INIT(9216);
#if 0
/* external sleep function when in idle (used in .cfg file) */
void wfi_sleep(void);

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction. 
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void wfi_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}
#endif

/**
 *  @b Description
 *  @n
 *      This is the entry point into the unit test code
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    /* init SOC specific modules */
    System_init();
    Board_init();

#if 0
    float A[36] = {
       0.1f, 1.2f, 2.3f, 3.4f, 4.5f, 5.5f,
       0.1f, 1.2f, 2.3f, 3.4f, 4.5f, 5.5f,
       0.1f, 1.2f, 2.3f, 3.4f, 4.5f, 5.5f,
       0.1f, 1.2f, 2.3f, 3.4f, 4.5f, 5.5f,
       0.1f, 1.2f, 2.3f, 3.4f, 4.5f, 5.5f,
       0.1f, 1.2f, 2.3f, 3.4f, 4.5f, 5.5f,
    };
	float B[36] = {
	   5.4f, 4.3f, 3.2f, 2.1f, 1.0f, 0.9f,
       5.4f, 4.3f, 3.2f, 2.1f, 1.0f, 0.9f,
       5.4f, 4.3f, 3.2f, 2.1f, 1.0f, 0.9f,
       5.4f, 4.3f, 3.2f, 2.1f, 1.0f, 0.9f,
       5.4f, 4.3f, 3.2f, 2.1f, 1.0f, 0.9f,
       5.4f, 4.3f, 3.2f, 2.1f, 1.0f, 0.9f,
	};
	float C1[36] = {0};
	float C2[36] = {0};
	
	uint32_t start_bench, asm_bench, c_bench;
#endif


    /* Debug Message: */
	test_print ("******************************************\n");
	test_print ("Debug: Launching the GTRACK Test Application\n");
	test_print ("******************************************\n");
#if 0
	gtrack_matrixPrint(6, 6, A);
    gtrack_matrixPrint(6, 6, B);

    start_bench = gtrack_getCycleCount();
    asm_matrixMultiply66(A,B,C1);
    asm_bench = gtrack_getCycleCount();
    gtrack_matrixMultiply66M(A,B,C2);
    c_bench = gtrack_getCycleCount();

	gtrack_matrixPrint(6, 6, C1);
    gtrack_matrixPrint(6, 6, C2);

    /* Debug Message: */
    test_print ("Benchmarking [6x6] x [6x6] multiplication, cycles: %u (asm), %u (c)\n", asm_bench- start_bench, c_bench - asm_bench);
#endif

    /* This task is created at highest priority, it should create more tasks and then delete itself */
    gAppTask = xTaskCreateStatic( Test_initTask,   /* Pointer to the function that implements the task. */
                                  "test_task_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  APP_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,              /* We are not using the task parameter. */
                                  APP_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gAppTskStackMain,  /* pointer to stack base */
                                  &gAppTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gAppTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough FreeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);

    return 0;
}
