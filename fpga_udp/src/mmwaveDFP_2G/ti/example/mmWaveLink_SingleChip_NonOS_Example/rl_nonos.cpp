/*
* nonos.c - CC31xx/CC32xx Host Driver Implementation
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



/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <mmwavelink.h>
#include "rl_nonos.h"
#ifdef _WIN32
    #include <windows.h>
#elif __linux__
    #include <thread>
#endif
#define NONOS_MAX_SPAWN_ENTRIES     5

typedef struct
{
    rlSpawnEntryFunc_t         pEntry;
    void*                       pValue;
}rlNonOsSpawnEntry_t;

typedef struct
{
    rlNonOsSpawnEntry_t    SpawnEntries[NONOS_MAX_SPAWN_ENTRIES];
}rlNonOsCB_t;

rlNonOsCB_t rl_nonOsCB;

/*!
    \brief     This function creates a locking object.

    The locking object is used for protecting a shared resources between different
    threads.

    \param    pLockObj    -    pointer to the locking object control block

    \return upon successful creation the function should return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlLockObjCreate(rlOsiMutexHdl_t* mutexHdl, rlInt8_t* name)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemSet((rlNonOsSemObj_t*)mutexHdl, NON_OS_LOCK_OBJ_UNLOCK_VALUE);
    return ((rlInt32_t)retVal);
}

/*!
    \brief     This function deletes a locking object.

    \param    pLockObj    -    pointer to the locking object control block

    \return upon successful deletion the function should return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlLockObjDelete(rlOsiMutexHdl_t* mutexHdl)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemSet((rlNonOsSemObj_t*)mutexHdl, 0);
    return ((rlInt32_t)retVal);
}

/*!
    \brief     This function locks a locking object.

    All other threads that call this function before this thread calls
    the rlNonOsLockObjUnlock would be suspended

    \param    pLockObj    -    pointer to the locking object control block
    \param    Timeout        -    numeric value specifies the maximum number of mSec to
                            stay suspended while waiting for the locking object
                            Currently, the simple link driver uses only two values:
                                - NONOS_WAIT_FOREVER
                                - NONOS_NO_WAIT


    \return upon successful reception of the locking object the function should return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlLockObjLock(rlOsiMutexHdl_t* mutexHdl, rlOsiTime_t timeout)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemGet((rlNonOsSemObj_t*)mutexHdl, NON_OS_LOCK_OBJ_UNLOCK_VALUE, NON_OS_LOCK_OBJ_LOCK_VALUE, timeout);
    return ((rlInt32_t)retVal);
}

/*!
    \brief     This function unlock a locking object.

    \param    pLockObj    -    pointer to the locking object control block

    \return upon successful unlocking the function should return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlLockObjUnlock(rlOsiMutexHdl_t* mutexHdl)
{
    rlNonOsRetVal_t retVal;
    retVal =rlNonOsSemSet((rlNonOsSemObj_t*)mutexHdl, NON_OS_LOCK_OBJ_UNLOCK_VALUE);
    return ((rlInt32_t)retVal);
}

/*!
    \brief     This function creates a sync object

    The sync object is used for synchronization between different thread or ISR and
    a thread.

    \param    pSyncObj    -    pointer to the sync object control block

    \return upon successful creation the function return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlSyncObjCreate(rlOsiSemHdl_t* semHdl, rlInt8_t* name)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemSet((rlNonOsSemObj_t*) semHdl, NON_OS_SYNC_OBJ_CLEAR_VALUE);
    return ((rlInt32_t)retVal);
}

/*!
    \brief     This function deletes a sync object

    \param    pSyncObj    -    pointer to the sync object control block

    \return upon successful deletion the function should return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlSyncObjDelete(rlOsiSemHdl_t* semHdl)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemSet((rlNonOsSemObj_t*) semHdl, 0);
    return ((rlInt32_t)retVal);
}

/*!
    \brief         This function generates a sync signal for the object.

    All suspended threads waiting on this sync object are resumed

    \param        pSyncObj    -    pointer to the sync object control block

    \return     upon successful signaling the function should return 0
                Otherwise, a negative value indicating the error code shall be returned
    \note        the function could be called from ISR context
    \warning
*/
rlInt32_t rlSyncObjSignal(rlOsiSemHdl_t* semHdl)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemSet((rlNonOsSemObj_t*) semHdl, NON_OS_SYNC_OBJ_SIGNAL_VALUE);
    return ((rlInt32_t)retVal);
}

/*!
    \brief     This function waits for a sync signal of the specific sync object

    \param    pSyncObj    -    pointer to the sync object control block
    \param    Timeout        -    numeric value specifies the maximum number of mSec to
                            stay suspended while waiting for the sync signal
                            Currently, the simple link driver uses only two values:
                                - NONOS_WAIT_FOREVER
                                - NONOS_NO_WAIT

    \return upon successful reception of the signal within the timeout window return 0
            Otherwise, a negative value indicating the error code shall be returned
    \note
    \warning
*/
rlInt32_t rlSyncObjWait(rlOsiSemHdl_t* semHdl, rlOsiTime_t timeout)
{
    rlNonOsRetVal_t retVal;
    retVal = rlNonOsSemGet((rlNonOsSemObj_t*) semHdl, NON_OS_SYNC_OBJ_SIGNAL_VALUE, NON_OS_SYNC_OBJ_CLEAR_VALUE, timeout);
    return ((rlInt32_t)retVal);
}

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
rlInt32_t rlSpawn(rlSpawnEntryFunc_t pEntry, void* pValue, rlUInt32_t flags)
{
    return rlNonOsSpawn(pEntry, pValue, flags);
}


rlNonOsRetVal_t rlNonOsSemSet(rlNonOsSemObj_t* pSemObj , rlNonOsSemObj_t Value)
{
    *pSemObj = Value;
    return NONOS_RET_OK;
}

rlNonOsRetVal_t rlNonOsSemGet(rlNonOsSemObj_t* pSyncObj, rlNonOsSemObj_t WaitValue, rlNonOsSemObj_t SetValue, rlNonOsTime_t Timeout)
{
#ifdef rlSyncWaitLoopCallback
    rlNonOsTime_t timeOutRequest = Timeout;
#endif
    volatile rlNonOsSemObj_t *tempVal = pSyncObj;

    while (Timeout>0)
    {
        if (WaitValue == *tempVal)
        {
            *pSyncObj = SetValue;
            break;
        }
        if (Timeout != NONOS_WAIT_FOREVER)
        {
            Timeout--;
        }
#ifdef rlSyncWaitLoopCallback
        if( (NON_OS_SYNC_OBJ_SIGNAL_VALUE == WaitValue) && (timeOutRequest != NONOS_NO_WAIT) )
        {
            if (WaitValue == *pSyncObj)
            {
                *pSyncObj = SetValue;
                break;
            }
            rlSyncWaitLoopCallback();
        }
#endif
    }

    if (0 == Timeout)
    {
        return NONOS_RET_ERR;
    }
    else
    {
        return NONOS_RET_OK;
    }
}


rlNonOsRetVal_t rlNonOsSpawn(rlSpawnEntryFunc_t pEntry , void* pValue , rlUInt32_t flags)
{
     rlInt8_t i = 0;

#ifndef SL_TINY_EXT
    for (i=0 ; i<NONOS_MAX_SPAWN_ENTRIES ; i++)
#endif
    {
        rlNonOsSpawnEntry_t* pE = &rl_nonOsCB.SpawnEntries[i];

        if (NULL == pE->pEntry)
        {
            pE->pValue = pValue;
            pE->pEntry = pEntry;
#ifndef SL_TINY_EXT
            break;
#endif
        }
    }
    return NONOS_RET_OK;
}


rlNonOsRetVal_t rlNonOsMainLoopTask(void)
{
    rlInt8_t i=0;


    for (i=0 ; i<NONOS_MAX_SPAWN_ENTRIES ; i++)
    {
        rlNonOsSpawnEntry_t* pE = &rl_nonOsCB.SpawnEntries[i];
        rlSpawnEntryFunc_t         pF = pE->pEntry;

        if (NULL != pF)
        {

            pF(pE->pValue); /* (pValue) */

            pE->pEntry = NULL;
            pE->pValue = NULL;
        }
    }

    return NONOS_RET_OK;
}


rlInt32_t rlAppSleep(rlUInt32_t delay)
{
    #ifdef _WIN32
        Sleep(delay/5);
    #elif __linux__
        std::this_thread::sleep_for(std::chrono::milliseconds(delay/5));
    #else
        rlUInt32_t i;
        for(i=0; i< delay*10000; i++);
        return 0;
    #endif
    return 0;
}