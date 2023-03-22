/*
 * rls_osi.h - mmWaveLink OS Callback Implementation on Windows
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/
/****************************************************************************************
 * FILE INCLUSION PROTECTION
 ****************************************************************************************
 */
#ifndef MMWL_OSI_WIN_H
#define MMWL_OSI_WIN_H

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN /* prevents windows including winsock1.h */
    #include <windows.h>
#elif __linux__
    #include "pevents.h"
    #define __declspec(dllexport)
    #define UINT32 uint32_t
#endif
#include "stdlib.h"

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */
// #ifdef  __cplusplus
// extern "C" {
// #endif

/****************************************************************************************
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
#define OSI_WAIT_FOREVER               (0xFFFFFFFF)
#define OSI_NO_WAIT                    (0)

/******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */
typedef enum osiReturnVal
{
    OSI_OK,
    OSI_OPERATION_FAILED,
    OSI_ABORTED,
    OSI_INVALID_PARAMS,
    OSI_MEMORY_ALLOCATION_FAILURE,
    OSI_TIMEOUT
}osiReturnVal_t;

/* Define Critical Section */
#define ENTER_CRITICAL_SECTION
#define EXIT_CRITICAL_SECTION

/*! 
*    \brief Type definition for a time value

*    \note On each porting or platform the type could be whatever is needed - integer, 
 pointer to structure etc.
*/
typedef unsigned int osiTime_t;

/*!
    \brief type definition for a sync object container
    
    Sync object is object used to synchronize between two threads or thread and interrupt handler.
    One thread is waiting on the object and the other thread send a signal, which then
    release the waiting thread.
    The signal must be able to be sent from interrupt context.
    This object is generally implemented by binary semaphore or events.
        
    \note On each porting or platform the type could be whatever is needed - integer, structure etc.
*/
#ifdef _WIN32
typedef HANDLE osiSyncObj_t;
#elif __linux__
typedef neosmart::neosmart_event_t osiSyncObj_t;
#endif


/*!
    \brief type definition for a locking object container
    
    Locking object are used to protect a resource from mutual accesses of two or more threads. 
    The locking object should suppurt reentrant locks by a signal thread.
    This object is generally implemented by mutex semaphore
    
    \note On each porting or platform the type could be whatever is needed - integer, structure etc.
*/
#ifdef _WIN32
typedef HANDLE osiLockObj_t;
#elif __linux__
typedef pthread_mutex_t *osiLockObj_t;
#endif


/*!
    \brief type definition for a spawn entry callback

    the spawn mechanism enable to run a function on different context. 
    This mechanism allow to transfer the execution context from interrupt context to thread context
    or changing teh context from an unknown user thread to general context.
    The implementation of the spawn mechanism depends on the user's system reqeuirements and could varies 
    from implementation of serialized execution using signle thread to creating thread per call
    
    \note The stack size of the execution thread must be at least of TBD bytes!
*/
typedef void (*rlsSpawnEntryFunc_t)(const void *);

/*!
    \brief type definition for a spawn Task Function

    the spawn mechanism enable to run a function on different context(Task). 
    This mechanism allow to transfer the execution context from interrupt context to thread context
    or changing teh context from an unknown user thread to general context.
    The implementation of the spawn mechanism depends on the user's system reqeuirements and could varies 
    from implementation of serialized execution using signle thread to creating thread per call
    
*/
typedef void (*P_OSI_SPAWN)(void *);


/******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************
 */

/** @fn int osiSleep(UINT32 Duration)
*
*   @brief This is OS Sleep Function
*   @param[in] Duration - Delay in milliseconds
*
*   @return int Success - 0, Failure - Error Code 
*
*   This is OS Sleep Function
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiSleep(UINT32 Duration);


/** @fn int osiSyncObjCreate(osiSyncObj_t* pSyncObj,char* pName)
*
*   @brief This function creates a sync object
*   @param[in] pSyncObj - pointer to the sync object control block
*   @param[in] pName - Name of sync object
*
*   @return int Success - 0, Failure - Error Code 
*
*   The sync object is used for synchronization between diffrent thread or ISR and 
*   a thread.
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiSyncObjCreate(osiSyncObj_t* pSyncObj,char* pName);


/** @fn int osiSyncObjDelete(osiSyncObj_t* pSyncObj)
*
*   @brief This function deletes a sync object
*   @param[in] pSyncObj - pointer to the sync object control block
*
*   @return int Success - 0, Failure - Error Code 
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiSyncObjDelete(osiSyncObj_t* pSyncObj);


/** @fn int osiSyncObjSignal(osiSyncObj_t* pSyncObj)
*
*   @brief This function generates a sync signal for the object. 
*   @param[in] pSyncObj - pointer to the sync object control block
*
*   @return int Success - 0, Failure - Error Code 
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiSyncObjSignal(osiSyncObj_t* pSyncObj);

/** @fn int osiSyncObjWait(osiSyncObj_t* pSyncObj, osiTime_t Timeout)
*
*   @brief This function waits for a sync signal of the specific sync object
*   @param[in] pSyncObj - pointer to the sync object control block
*   @param[in] Timeout - numeric value specifies the maximum number of mSec to 
                            stay suspended while waiting for the sync signal
                            Currently, the mmWave link driver uses only two values:
                                - OSI_WAIT_FOREVER
                                - OSI_NO_WAIT
*
*   @return int Success - 0, Failure - Error Code 
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiSyncObjWait(osiSyncObj_t* pSyncObj, osiTime_t Timeout);


/** @fn int osiSyncObjClear(osiSyncObj_t* pSyncObj)
*
*   @brief This function clears a sync object
*   @param[in] pSyncObj - pointer to the sync object control block
*
*   @return int Success - 0, Failure - Error Code 
*
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiSyncObjClear(osiSyncObj_t* pSyncObj);


/** @fn int osiLockObjCreate(osiLockObj_t* pLockObj, char* pName)
*
*   @brief This function creates a locking object
*   @param[in] pSyncObj - pointer to the locking object control block
*   @param[in] pName - Name of locking object
*
*   @return int Success - 0, Failure - Error Code 
*
*   The locking object is used for protecting a shared resources between different 
*   threads.
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiLockObjCreate(osiLockObj_t* pLockObj, char* pName);


/** @fn int osiLockObjDelete(osiLockObj_t* pLockObj)
*
*   @brief This function deletes a locking object
*   @param[in] pSyncObj - pointer to the locking object control block
*
*   @return int Success - 0, Failure - Error Code 
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiLockObjDelete(osiLockObj_t* pLockObj);

/** @fn int osiLockObjLock(osiLockObj_t* pLockObj , osiTime_t Timeout)
*
*   @brief This function locks a locking object
*   @param[in] pSyncObj - pointer to the locking object control block
*   @param[in] Timeout - numeric value specifies the maximum number of mSec to 
                            stay suspended while waiting for the locking object
                            Currently, the mmWave link driver uses only two values:
                                - OSI_WAIT_FOREVER
                                - OSI_NO_WAIT
*
*   @return int Success - 0, Failure - Error Code 
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiLockObjLock(osiLockObj_t* pLockObj , osiTime_t Timeout);

/** @fn int osiLockObjUnlock(osiLockObj_t* pLockObj)
*
*   @brief This function unlock a locking object
*   @param[in] pSyncObj - pointer to the locking object control block
*
*   @return int Success - 0, Failure - Error Code 
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) int osiLockObjUnlock(osiLockObj_t* pLockObj);


/** @fn int osiSpawn(rlsSpawnEntryFunc_t pEntry , void* pValue , unsigned int flags)
*
*   @brief This function call the pEntry callback from a different context
*   @param[in] pSyncObj - pointer to the locking object control block
*   @param[in] pValue - pointer to any type of memory structure that would be
                            passed to pEntry callback from the execution thread.
*   @param[in] flags - execution flags - reserved for future usage
*
*   @return int Success - 0, Failure - Error Code 
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport)  int osiSpawn(rlsSpawnEntryFunc_t pEntry , const void* pValue , unsigned int flags);


/** @fn int osiExecute(rlsSpawnEntryFunc_t pEntry , void* pValue , unsigned long flags)
*
*   @brief This function call the pEntry callback from a different context
*   @param[in] pSyncObj - pointer to the locking object control block
*   @param[in] pValue - pointer to any type of memory structure that would be
                            passed to pEntry callback from the execution thread.
*   @param[in] flags - execution flags - reserved for future usage
*
*   @return int Success - 0, Failure - Error Code 
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport)  int osiExecute(rlsSpawnEntryFunc_t pEntry , const void* pValue , unsigned int flags);


/** @fn unsigned long osiGetTime(void)
*
*   @brief This function returns the values of the system clock in mSec.
                    The system clock is set to 0 during initialization.
*
*   @return system clock in mSec
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
__declspec(dllexport) unsigned long osiGetTime(void);


// #ifdef  __cplusplus
// }
// #endif 

#endif
/*
 * END OF MMWL_OSI_WIN_H FILE
 */

