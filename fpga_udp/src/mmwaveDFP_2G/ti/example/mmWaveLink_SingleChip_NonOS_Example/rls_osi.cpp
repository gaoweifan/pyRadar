/*
 * rls_osi.c - mmWaveLink OS Callback Implementation on Windows
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

#include "rls_osi.h"
#ifdef _WIN32
	#include <process.h>
	#include <windows.h>
#elif __linux__
	#include "pevents.h"
	#include <pthread.h>
	#include <thread>
	#include <cassert>
	#include <time.h>
	#include <errno.h>
	#define FALSE 0
	typedef int BOOL;
#endif
#include <atomic>
#include <stdio.h>
#include <string.h>


typedef void (*P_OS_CALLBACK_FUNCTION)(UINT32);
std::atomic<long>       rls_globalLockMutexCounter(0);
osiLockObj_t* 			rls_pGloblaLockObj = NULL;

int osiSleep(UINT32 Duration)
{
	#ifdef _WIN32
        Sleep(Duration);
    #elif __linux__
        std::this_thread::sleep_for(std::chrono::milliseconds(Duration));
    #endif
	return OSI_OK;
}

/*******************************************************************************

	SYNC OBJECT

********************************************************************************/

int osiSyncObjCreate(osiSyncObj_t* pSyncObj, char* pName)
{
	if (NULL == pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}

	#ifdef _WIN32
		*pSyncObj = CreateEvent( 
			NULL,               // default security attributes
			FALSE,                 // auto-reset event
			FALSE                  // initial state is nonsignaled
			#ifdef UNICODE
			,(LPCWSTR)pName     // object name
			#else
			,(LPCSTR)pName      // object name
			#endif
        ); 
	#elif __linux__
        *pSyncObj = neosmart::CreateEvent(FALSE,FALSE);
    #endif

    if (*pSyncObj == NULL) 
    {
        return OSI_OPERATION_FAILED;
    }

	return OSI_OK;
}


int osiSyncObjDelete(osiSyncObj_t* pSyncObj)
{
	BOOL RetVal;

	if (NULL == pSyncObj || NULL == *pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}

	#ifdef _WIN32
        RetVal = CloseHandle(*pSyncObj);
    #elif __linux__
        RetVal = neosmart::DestroyEvent(*pSyncObj)==0;
    #endif

	if (FALSE != RetVal)
	{
		return OSI_OK;
	}
	else
	{
		return OSI_OPERATION_FAILED;
	}
}


int osiSyncObjSignal(osiSyncObj_t* pSyncObj)
{
	BOOL RetVal;
	
	if (NULL == pSyncObj || NULL == *pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}

	#ifdef _WIN32
		RetVal = SetEvent(*pSyncObj);
	#elif __linux__
        RetVal = neosmart::SetEvent(*pSyncObj)==0;
    #endif

	if (FALSE != RetVal)
	{
		return OSI_OK;
	}
	else
	{
		return OSI_OPERATION_FAILED;
	}
}


int osiSyncObjWait(osiSyncObj_t* pSyncObj , osiTime_t Timeout)
{
	UINT32 RetVal;
	
	if (NULL == pSyncObj || NULL == *pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}

	#ifdef _WIN32
		RetVal = WaitForSingleObject(*pSyncObj, Timeout);
	#elif __linux__
        RetVal = neosmart::WaitForEvent(*pSyncObj, Timeout);
    #endif
	
	#ifdef _WIN32
	if (WAIT_OBJECT_0 == RetVal)
	#elif __linux__
	if (0 == RetVal)
	#endif
	{
		return OSI_OK;
	}
	else
	{
		return OSI_OPERATION_FAILED;
	}
}

int osiSyncObjClear(osiSyncObj_t* pSyncObj)
{
	BOOL RetVal;
	
	if (NULL == pSyncObj || NULL == *pSyncObj)
	{
		return OSI_INVALID_PARAMS;
	}
	
	#ifdef _WIN32
		RetVal = ResetEvent(*pSyncObj);
	#elif __linux__
        RetVal = neosmart::ResetEvent(*pSyncObj)==0;
    #endif

	if (FALSE != RetVal)
	{
		return OSI_OK;
	}
	else
	{
		return OSI_OPERATION_FAILED;
	}
}


/*******************************************************************************

		LOCKING OBJECT

********************************************************************************/

int osiLockObjCreate(osiLockObj_t* 		pLockObj, char* pName)
{	
	if (NULL == pLockObj)
	{
		return OSI_INVALID_PARAMS;
	}
	#ifdef _WIN32
		*pLockObj = CreateMutex( 
			NULL,              // default security attributes
			FALSE,             // initially not owned
			NULL);             // unnamed mutex
		if (*pLockObj == NULL) 
		{
			return OSI_OPERATION_FAILED;
		}
	#elif __linux__
		pthread_mutex_t *mutex = new pthread_mutex_t;
        int result = pthread_mutex_init(mutex, NULL);
		if (result != 0) 
		{
			return OSI_OPERATION_FAILED;
		}
		*pLockObj = mutex;
	#endif

	if (strcmp(pName, "GlobalLockObj") == 0)
	{
		/* save reference to the global lock pointer */
		rls_pGloblaLockObj = pLockObj;

		/* reset the counter */
		rls_globalLockMutexCounter = 0;
	}
	
	return OSI_OK;
}
								 
int osiLockObjDelete(osiLockObj_t* pLockObj)
{
	BOOL RetVal;

	if (NULL == pLockObj || NULL == *pLockObj)
	{
		return OSI_INVALID_PARAMS;
	}
	
	/* if we are going to delete the "GlobalLock" then wait till all threads
	waiting on this mutex are released */
	if (rls_pGloblaLockObj == pLockObj)
	{	
		//add sleep 1
		while(rls_globalLockMutexCounter)
		{
			osiSleep(1);
		}

		rls_pGloblaLockObj = NULL;

	}

	#ifdef _WIN32
		RetVal = CloseHandle(*pLockObj);
	#elif __linux__
        RetVal = pthread_mutex_destroy(*pLockObj)==0;
	#endif

	if (FALSE != RetVal)
	{
		return OSI_OK;
	}
	else
	{
		return OSI_OPERATION_FAILED;
	}
}

int osiLockObjLock(osiLockObj_t* pLockObj , osiTime_t Timeout)
{
	UINT32 RetVal;	

	if (NULL == pLockObj || NULL == *pLockObj)
	{
		return OSI_INVALID_PARAMS;
	}
	
	/* Increment the global lock counter  */
	if (rls_pGloblaLockObj == pLockObj)
	{
		rls_globalLockMutexCounter++;
	}

	#ifdef _WIN32
		RetVal = WaitForSingleObject(*pLockObj,Timeout);
	#elif __linux__
		struct timespec ts;
		struct timespec now;
		clock_gettime(CLOCK_REALTIME, &now);
		ts.tv_sec = now.tv_sec + Timeout/1000;
		ts.tv_nsec = now.tv_nsec + ((Timeout%1000) * 1000000);
		if (ts.tv_nsec >= 1000000000) {
			ts.tv_sec++;
			ts.tv_nsec -= 1000000000;
		}
        RetVal = pthread_mutex_timedlock(*pLockObj, &ts);
	#endif

	/* Decrement the global lock counter  */
	if (rls_pGloblaLockObj == pLockObj)
	{
		rls_globalLockMutexCounter--;
	}

	#ifdef _WIN32
	if (WAIT_OBJECT_0 == RetVal)
	#elif __linux__
	if (0 == RetVal)
	#endif
	{
		return OSI_OK;
	}
	#ifdef _WIN32
	else if (WAIT_TIMEOUT == RetVal) 
	#elif __linux__
	else if (ETIMEDOUT == RetVal)
	#endif
	{
		return OSI_TIMEOUT;
	}
    else
    {
        return OSI_OPERATION_FAILED;
    }
}

int osiLockObjUnlock(osiLockObj_t* pLockObj)
{
	BOOL RetVal;
	
	if (NULL == pLockObj || NULL == *pLockObj)
	{
		return OSI_INVALID_PARAMS;
	}

	#ifdef _WIN32
		RetVal = ReleaseMutex(*pLockObj);
	#elif __linux__
		RetVal = pthread_mutex_unlock(*pLockObj)==0;
	#endif
	
	if (FALSE != RetVal)
	{
		return OSI_OK;
	}
	else
	{
		return OSI_OPERATION_FAILED;
	}
}


/*******************************************************************************

		SPWAN and CONTEXTS

********************************************************************************/



typedef struct spawnThreadEntry
{
	rlsSpawnEntryFunc_t entryFunc;
	const void*         pParam;
}spawnThreadEntry_t;

#ifdef _WIN32
typedef struct spawnCB
{
    HANDLE  ThreadHdl;
    DWORD   ThreadID;
}spawnCB_t;

spawnCB_t* rls_pSpawnCB=NULL;

#define SPAWN_MESSAGE           (WM_APP + 328) // custom message for thread
#endif

int osiSpawn(rlsSpawnEntryFunc_t pEntry , const void* pValue , unsigned int flags)
{
    pEntry(pValue);
    return 0;
}

#ifdef _WIN32
DWORD WINAPI ExecuteEntryFunc(LPVOID lpParam) 
#elif __linux__
void *ExecuteEntryFunc(void *lpParam) 
#endif
{
    spawnThreadEntry_t* pTh = (spawnThreadEntry_t*)(lpParam);
	pTh->entryFunc(pTh->pParam);
    free(pTh);
    return 0;
}

int osiExecute(rlsSpawnEntryFunc_t pEntry , const void* pValue , unsigned int flags)
{
	spawnThreadEntry_t * te = (spawnThreadEntry_t*)malloc(sizeof(spawnThreadEntry_t));
	te->entryFunc = pEntry;
	te->pParam = pValue;
	
	#ifdef _WIN32
		HANDLE  ThreadHdl;
		DWORD   ThreadID;

		ThreadHdl = CreateThread(NULL,0,ExecuteEntryFunc,te,0,&ThreadID);
		Sleep(1);
		return 0;
	#elif __linux__
		pthread_t ThreadHdl;
		return pthread_create(&ThreadHdl, NULL, ExecuteEntryFunc,(void*)te);
	#endif
}

unsigned long osiGetTime(void)
{
	#ifdef _WIN32
		return GetTickCount();
	#elif __linux__
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		return (now.tv_sec*1000) + (now.tv_nsec/1000000);
	#endif
}
