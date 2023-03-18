/*
 * nonos.h - CC31xx/CC32xx Host Driver Implementation
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

#ifndef RL_NONOS_H
#define RL_NONOS_H

#include <stdio.h>

#ifdef    __cplusplus
extern "C" {
#endif

/* This function call the user defined function, if defined, from the sync wait loop  */
/* The use case of this function is to allow nonos system to call a user function to put the device into sleep */
/* The wake up should be activated after getting an interrupt from the device to Host */
/* The user function must return without blocking to prevent a delay on the event handling */
/*
#define rlSyncWaitLoopCallback  UserSleepFunction
*/



#define NONOS_WAIT_FOREVER                               0x7FFF
#define NONOS_NO_WAIT                                    0x00

#define NONOS_RET_OK                            (0)
#define NONOS_RET_ERR                           (0xFF)
#define OSI_OK  NONOS_RET_OK

#define NON_OS_SYNC_OBJ_CLEAR_VALUE                0x11
#define NON_OS_SYNC_OBJ_SIGNAL_VALUE                0x22
#define NON_OS_LOCK_OBJ_UNLOCK_VALUE                0x33
#define NON_OS_LOCK_OBJ_LOCK_VALUE                0x44

/*!
    \brief type definition for the return values of this adaptation layer
*/
typedef rlInt8_t rlNonOsRetVal_t;

/*!
    \brief type definition for a time value
*/
typedef rlInt32_t rlNonOsTime_t;

/*!
    \brief     type definition for a sync object container

    Sync object is object used to synchronize between two threads or thread and interrupt handler.
    One thread is waiting on the object and the other thread send a signal, which then
    release the waiting thread.
    The signal must be able to be sent from interrupt context.
    This object is generally implemented by binary semaphore or events.
*/
typedef rlInt8_t rlNonOsSemObj_t;
typedef void (*rlSpawnEntryFunc_t)(void* pValue);


#define rlTime_t       rlNonOsTime_t

#define rlSyncObj_t    rlNonOsSemObj_t

#define rlLockObj_t    rlNonOsSemObj_t

#define RL_OS_WAIT_FOREVER     NONOS_WAIT_FOREVER

#define RL_OS_RET_CODE_OK       NONOS_RET_OK

#define RL_OS_NO_WAIT           NONOS_NO_WAIT


/*!
    \brief     This function clears a sync object

    \param    pSyncObj    -    pointer to the sync object control block

    \return upon successful clearing the function should return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
#define rlNonOsSyncObjClear(pSyncObj)            rlNonOsSemSet(pSyncObj,NON_OS_SYNC_OBJ_CLEAR_VALUE)



/*!
    \brief     This function call the pEntry callback from a different context

    \param    pEntry        -    pointer to the entry callback function

    \param    pValue        -     pointer to any type of memory structure that would be
                            passed to pEntry callback from the execution thread.

    \param    flags        -     execution flags - reserved for future usage

    \return upon successful registration of the spawn the function return 0
            (the function is not blocked till the end of the execution of the function
            and could be returned before the execution is actually completed)
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlNonOsRetVal_t rlNonOsSpawn(rlSpawnEntryFunc_t pEntry , void* pValue , rlUInt32_t flags);

/*!
    \brief     This function must be called from the main loop in non-os paltforms

    \param    None

    \return 0 - No more activities
            1 - Activity still in progress
    \note
    \warning
*/
rlNonOsRetVal_t rlNonOsMainLoopTask(void);

rlInt32_t rlLockObjCreate(rlOsiMutexHdl_t* mutexHdl, rlInt8_t* name);
rlInt32_t rlLockObjDelete(rlOsiMutexHdl_t* mutexHdl);
rlInt32_t rlLockObjLock(rlOsiMutexHdl_t* mutexHdl, rlOsiTime_t timeout);
rlInt32_t rlLockObjUnlock(rlOsiMutexHdl_t* mutexHdl);
rlInt32_t rlSyncObjCreate(rlOsiSemHdl_t* semHdl, rlInt8_t* name);
rlInt32_t rlSyncObjDelete(rlOsiSemHdl_t* semHdl);
rlInt32_t rlSyncObjSignal(rlOsiSemHdl_t* semHdl);
rlInt32_t rlSyncObjWait(rlOsiSemHdl_t* semHdl, rlOsiTime_t timeout);
rlInt32_t rlSpawn(rlSpawnEntryFunc_t pEntry, void* pValue, rlUInt32_t flags);
rlInt32_t rlSleep(rlUInt32_t delay);


extern rlNonOsRetVal_t rlNonOsSemGet(rlNonOsSemObj_t* pSyncObj, rlNonOsSemObj_t WaitValue, rlNonOsSemObj_t SetValue, rlNonOsTime_t Timeout);
extern rlNonOsRetVal_t rlNonOsSemSet(rlNonOsSemObj_t* pSemObj , rlNonOsSemObj_t Value);
extern rlNonOsRetVal_t rlNonOsSpawn(rlSpawnEntryFunc_t pEntry , void* pValue , rlUInt32_t flags);

#if (defined(rlSyncWaitLoopCallback))
extern void rlSyncWaitLoopCallback(void);
#endif

extern rlInt32_t rlAppSleep(rlUInt32_t delay);

/*****************************************************************************

    Overwrite SimpleLink driver OS adaptation functions


 *****************************************************************************/


#undef rlTaskEntry
#define rlTaskEntry                                rlNonOsMainLoopTask

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif
/*
 * END OF RL_NONOS_H
 */

