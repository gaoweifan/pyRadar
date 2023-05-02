/******************************************************************************************
* FileName     : mmw_example_nonos.c
*
* Description  : This file implements mmwave link example application for non-OS environment.
*
****************************************************************************************
* (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
*---------------------------------------------------------------------------------------
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided that the following conditions are met:
*
*    Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
*  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/


// #define NON_OS_ENVIRONMENT

/******************************************************************************
* INCLUDE FILES
******************************************************************************
*/
#ifdef _WIN32
    #include <windows.h>
    #include <share.h>
#elif __linux__
    #include <sys/timeb.h>
    #include <time.h>
    #include <stdarg.h>
    #define TRUE 1
    #define FALSE 0
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mmw_example_nonos.h"
#include "mmw_config.h"
#include <mmwavelink.h>
#include <math.h>
#include "mmwl_port_ftdi.h"
#ifdef NON_OS_ENVIRONMENT
#include "rl_nonos.h"
#else
#include "rls_osi.h"
#endif
/* AWR2243 meta image file */
#include "xwr22xx_metaImage.h"

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/
#define MMWL_FW_FIRST_CHUNK_SIZE (224U)
#define MMWL_FW_CHUNK_SIZE (232U)
#define MMWL_META_IMG_FILE_SIZE (sizeof(metaImage))

#define GET_BIT_VALUE(data, noOfBits, location)    ((((rlUInt32_t)(data)) >> (location)) &\
                                               (((rlUInt32_t)((rlUInt32_t)1U << (noOfBits))) - (rlUInt32_t)1U))
/* Async Event Timeouts */
#define MMWL_API_INIT_TIMEOUT                (2000) /* 2 Sec*/
#define MMWL_API_START_TIMEOUT               (1000) /* 1 Sec*/
#define MMWL_API_RF_INIT_TIMEOUT             (1000) /* 1 Sec*/

/* MAX unique chirp AWR2243 supports */
#define MAX_UNIQUE_CHIRP_INDEX                (512 -1)

/* MAX index to read back chirp config  */
#define MAX_GET_CHIRP_CONFIG_IDX              14

/* To enable TX2 */
// #define ENABLE_TX2                             1

/* LUT Buffer size for Advanced chirp 
   Max size = 12KB (12*1024) */
#define LUT_ADVCHIRP_TABLE_SIZE                5*1024

#define AWR2243_ES1_0	   0
#define AWR2243_ES1_1      1

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/
typedef int (*RL_P_OS_SPAWN_FUNC_PTR)(RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, unsigned int flags);
typedef int (*RL_P_OS_DELAY_FUNC_PTR)(unsigned int delay);

/* Global Variable for Device Status */
static unsigned char mmwl_bInitComp = 0U;
static unsigned char mmwl_bMssBootErrStatus = 0U;
static unsigned char mmwl_bStartComp = 0U;
static unsigned char mmwl_bRfInitComp = 0U;
static unsigned char mmwl_bSensorStarted = 0U;
static unsigned char mmwl_bGpadcDataRcv = 0U;
static unsigned char mmwl_bMssCpuFault = 0U;
static unsigned char mmwl_bMssEsmFault = 0U;

unsigned char gAwr2243CrcType = RL_CRC_TYPE_32BIT;

rlUInt16_t lutOffsetInNBytes = 0;
unsigned char gMmwaveSensorEs1_1 = 1;
/* Global variable configurations from config file */
rlDevGlobalCfg_t rlDevGlobalCfgArgs = { 0 };

/* store frame periodicity */
unsigned int framePeriodicity = 0;
/* store frame count */
unsigned int frameCount = 0;

/* SPI Communication handle to AWR2243 device*/
rlComIfHdl_t mmwl_devHdl = NULL;

/* structure parameters of two profile confing and cont mode config are same */
rlProfileCfg_t profileCfgArgs[2] = { 0 };

/* structure parameters of four profile confing of Adv chirp */
rlProfileCfg_t ProfileCfgArgs_AdvChirp[4] = { 0 };

/* strcture to store dynamic chirp configuration */
rlDynChirpCfg_t dynChirpCfgArgs[3] = { 0 };

/* Strcture to store async event config */
rlRfDevCfg_t rfDevCfg = { 0x0 };

/* Structure to store GPADC measurement data sent by device */
rlRecvdGpAdcData_t rcvGpAdcData = {0};

/* calibData is the calibration data sent by the device which needs to store to
   sFlash and will be used for factory calibration or embedded in the application itself */
rlCalibrationData_t calibData = { 0 };
rlPhShiftCalibrationData_t phShiftCalibData = { 0 };

/* File Handle for Calibration Data */
FILE *CalibrationDataPtr = NULL;
FILE *PhShiftCalibrationDataPtr = NULL;

/* Advanced Chirp LUT data */
/* Max size of the LUT is 12KB.
   Maximum of 212 bytes per chunk can be present per SPI message. */
/* This array is created to store the LUT RAM values from the user programmed parameters or config file.
   The populated array is sent over SPI to populate the RadarSS LUT RAM.
   This array is also saved into a file "AdvChirpLUTData.txt" for debug purposes */
/* The chirp paramters start address offset should be 4 byte aligned */
rlInt8_t AdvChirpLUTData[LUT_ADVCHIRP_TABLE_SIZE] = { 0 };

/* File Handle for Advanced Chirp LUT Data */
FILE *AdvChirpLUTDataPtr = NULL;

/* This will be used to alternatively switch the Adv chirp LUT offset 
   between the PING and the PONG buffer */
unsigned char gDynAdvChirpLUTBufferDir = 0;

extern "C" {
uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);
}

/* Function to compare dynamically configured chirp data */
int MMWL_chirpParamCompare(rlChirpCfg_t * chirpData);

#define USE_SYSTEM_TIME
static void rlsGetTimeStamp(char *Tsbuffer)
{
#ifdef USE_SYSTEM_TIME
    #ifdef _WIN32
        SYSTEMTIME SystemTime;
        GetLocalTime(&SystemTime);
        sprintf(Tsbuffer, "[%02d:%02d:%02d:%03d]: ", SystemTime.wHour, SystemTime.wMinute, SystemTime.wSecond, SystemTime.wMilliseconds);
    #elif __linux__
        struct timeb tTimeB;
        ftime(&tTimeB);//get time in sec and ms
        struct tm *tTM = localtime(&tTimeB.time);
        sprintf(Tsbuffer, "[%02d:%02d:%02d:%03d]: ", tTM->tm_hour, tTM->tm_min, tTM->tm_sec, tTimeB.millitm);
    #endif
#else
	__int64 tickPerSecond;
	__int64 tick;
	__int64 sec;
	__int64 usec;

	/* Get accuracy */
	QueryPerformanceFrequency((LARGE_INTEGER*)&tickPerSecond);

	/* Get tick */
	QueryPerformanceCounter((LARGE_INTEGER*)&tick);
	sec = (__int64)(tick / tickPerSecond);
	usec = (__int64)((tick - (sec * tickPerSecond)) * 1000000.0 / tickPerSecond);
	sprintf(Tsbuffer, "%07lld.%06lld: ", sec, usec);
#endif
}
#define _CAPTURE_TO_FILE_
#define DEBUG_EN
FILE* rls_traceFp = NULL;
FILE* rls_traceMmwlFp = NULL;
#ifdef DEBUG_EN
void DEBUG_PRINT(char *fmt, ...)
{
	char cBuffer[1000];
	if (TRUE)
	{
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), fmt, ap);
#ifdef _CAPTURE_TO_FILE_
		if (rls_traceFp != NULL)
		{
			char tsTime[30] = { 0 };
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceFp);
			fflush(rls_traceFp);
		}
		else
		{
			char tsTime[30] = { 0 };
            #ifdef _WIN32
                rls_traceFp = _fsopen("trace.txt", "wt", _SH_DENYWR);
            #elif __linux__
                rls_traceFp = fopen("trace.txt", "at+");
            #endif
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceFp);
			fflush(rls_traceFp);
		}
#endif
		va_end(ap);
	}
}
rlInt32_t MMWAVELINK_LOGGING(const rlInt8_t *fmt, ...)
{
	char cBuffer[1000];
	if (TRUE)
	{
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), fmt, ap);
#ifdef _CAPTURE_TO_FILE_
		if (rls_traceMmwlFp != NULL)
		{
			char tsTime[30] = { 0 };
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceMmwlFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceMmwlFp);
			fflush(rls_traceMmwlFp);
		}
		else
		{
			char tsTime[30] = { 0 };
			#ifdef _WIN32
                rls_traceMmwlFp = _fsopen("mmwavelink_log.txt", "wt", _SH_DENYWR);
            #elif __linux__
                rls_traceMmwlFp = fopen("mmwavelink_log.txt", "at+");
            #endif
            rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceMmwlFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceMmwlFp);
			fflush(rls_traceMmwlFp);
		}
#endif
		va_end(ap);
	}
	return 0;
}
#else
#define DEBUG_PRINT
#endif


/******************************************************************************
* all function definations starts here
*******************************************************************************
*/
/** @fn int MMWL_SwapResetAndPowerOn(rlUInt8_t deviceMap)
*
*   @brief Swap ROM with RAM, resets the core and waits for power-on completion
*   @param[in] deviceMap - device map
*
*   @return int Success - 0, Failure - Error Code 
*	@Note: This API is being used only with AWR2243 ES1.0 sample
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_SwapResetAndPowerOn(rlUInt8_t deviceMap)
{
    int timeOutCnt = 0;
    int retVal = RL_RET_CODE_OK;
	
	/* Wait for the below async events only when firmware download over SPI is done.
	   When booting from sFlash, it is not required to wait for below async events */
	if (rlDevGlobalCfgArgs.EnableFwDownload)
	{
		/* Wait for MSS ESM fault */
		/* Wait for MSS Boot error status if flash is not connected */
		while ((mmwl_bMssEsmFault == 0U) || (mmwl_bMssBootErrStatus == 0U))
		{
            #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(5);
            #else
            osiSleep(1); /*Sleep 1 msec*/
            #endif
			
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_START_TIMEOUT)
			{
				break;
			}
			/* If flash is connected, then no need to wait for MSS boot error status */
			if (rlDevGlobalCfgArgs.IsFlashConnected)
			{
				mmwl_bMssBootErrStatus = 1U;
			}
		}
	}
	mmwl_bMssEsmFault = 0U;
	mmwl_bMssBootErrStatus = 0U;

	/* Disable MSS Watchdog */
	retVal = rlDeviceSetInternalConf(deviceMap, 0xFFFFFF0C, 0x000000AD);

    /* Swap RAM memory map with ROM memory map  */
    retVal = rlDeviceSetInternalConf(deviceMap, 0xFFFFFF20, 0x00ADAD00);

    /* Disable the Ack*/
    rlDeviceConfigureAckTimeout(0);

    /* reset the core */
    retVal = rlDeviceSetInternalConf(deviceMap, 0xFFFFFF04, 0x000000AD);

    /* Enable the Ack*/
    rlDeviceConfigureAckTimeout(1000);

    /* Wait for Power ON complete */
    if (0 == retVal)
    {
        timeOutCnt = 0;
        while (mmwl_bInitComp == 0)
        {
			#ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(5);
            #else
            osiSleep(1); /*Sleep 1 msec*/
            #endif
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_INIT_TIMEOUT)
            {
                if ((deviceMap & 0x1) == 1)
                {
                    rlDevicePowerOff();
                }
                else
                {
                    rlDeviceRemoveDevices(deviceMap);
                }
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
    }
    mmwl_bInitComp = 0U;
    return retVal;
}

/** @fn void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
*    rlUInt16_t sbLen, rlUInt8_t *payload)
*
*   @brief Radar Async Event Handler callback
*   @param[in] msgId - Message Id
*   @param[in] sbId - SubBlock Id
*   @param[in] sbLen - SubBlock Length
*   @param[in] payload - Sub Block Payload
*
*   @return None
*
*   Radar Async Event Handler callback
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
    rlUInt16_t sbLen, rlUInt8_t *payload)
{
    unsigned int deviceMap = 0;
    rlUInt16_t msgId = sbId / RL_MAX_SB_IN_MSG;
    rlUInt16_t asyncSB = RL_GET_SBID_FROM_MSG(sbId, msgId);

    /* Host can receive Async Event from RADARSS/MSS */
    switch (msgId)
    {
        /* Async Event from RADARSS */
        case RL_RF_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
            case RL_RF_AE_INITCALIBSTATUS_SB:
            {
                mmwl_bRfInitComp = 1U;
                printf("Async event: RF-init calibration status \n\n");
            }
            break;
            case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
            {
                mmwl_bSensorStarted = 1U;
                printf("Async event: Frame trigger \n\n");
            }
            break;
            case RL_RF_AE_FRAME_END_SB:
            {
                mmwl_bSensorStarted = 0U;
                printf("Async event: Frame stopped \n\n");
            }
            break;
            case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
            {
                printf("Async event: Run time Calibration Report [0x%x]\n\n", ((rlRfRunTimeCalibReport_t*)payload)->calibErrorFlag);
            }
            break;
            case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
            {
                printf("Async event: Monitoring Timing Failed Report\n\n");
                break;
            }
			case RL_RF_AE_DIG_LATENTFAULT_REPORT_SB:
			{
				rlDigLatentFaultReportData_t *data = (rlDigLatentFaultReportData_t*)payload;
				printf("Dig Latent Fault report [0x%x] Async event\n\n", data->digMonLatentFault);
			}
			break;
			case RL_RF_AE_MON_DIG_PERIODIC_REPORT_SB:
			{
				rlDigPeriodicReportData_t *data = (rlDigPeriodicReportData_t*)payload;
				printf("Dig periodic report [0x%x] Async event\n\n", data->digMonPeriodicStatus);
			}
			break;
            case RL_RF_AE_GPADC_MEAS_DATA_SB:
            {
                mmwl_bGpadcDataRcv = 1U;
                /* store the GPAdc Measurement data which AWR2243 will read from the analog test pins
                    where user has fed the input signal */
                memcpy(&rcvGpAdcData, payload, sizeof(rlRecvdGpAdcData_t));
                break;
            }
            case RL_RF_AE_CPUFAULT_SB:
            {
                printf("BSS CPU fault \n\n");
                while(1);
            }
            case RL_RF_AE_ESMFAULT_SB:
            {
                printf("BSS ESM fault \n\n");
                break;
            }
            default:
                break;
            }

        }
        break;

        /* Async Event from MSS */
        case RL_DEV_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_DEV_AE_MSSPOWERUPDONE_SB:
                {
                    mmwl_bInitComp = 1U;
                }
                break;
                case RL_DEV_AE_MSS_BOOTERRSTATUS_SB:
                {
					mmwl_bMssBootErrStatus = 1U;
                }
                break;
                case RL_DEV_AE_RFPOWERUPDONE_SB:
                {
                    mmwl_bStartComp = 1U;
                }
                break;
                case RL_DEV_AE_MSS_ESMFAULT_SB:
                {
                    mmwl_bMssEsmFault = 1U;
                    printf("MSS ESM Error \n\n");
                }
                break;
                case RL_DEV_AE_MSS_CPUFAULT_SB:
                {
                    mmwl_bMssCpuFault = 1U;
                    printf("MSS CPU Fault\n\n");
                }
                break;
				case RL_DEV_AE_MSS_LATENTFLT_TEST_REPORT_SB:
				{
					rlMssLatentFaultReport_t *data = (rlMssLatentFaultReport_t*)payload;
					printf("MSS Latent fault [0x%x] [0x%x] Async event\n\n", data->testStatusFlg1, data->testStatusFlg2);
				}
				break;
				case RL_DEV_AE_MSS_PERIODIC_TEST_STATUS_SB:
				{
					rlMssPeriodicTestStatus_t *data = (rlMssPeriodicTestStatus_t*)payload;
					printf("MSS periodic test [0x%x] Async event\n\n", data->testStatusFlg);
				}
				break;
				case RL_DEV_AE_MSS_RF_ERROR_STATUS_SB:
				{
					printf("MSS RF Error \n\n");
				}
				break;
                default:
                {
                    printf("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
                    break;
                }
            }
        }
        break;

        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
			switch (asyncSB)
			{
				case RL_MMWL_AE_MISMATCH_REPORT:
				{
					int errTemp = *(int32_t*)payload;
					/* CRC mismatched in the received Async-Event msg */
					if (errTemp == RL_RET_CODE_CRC_FAILED)
					{
						printf("CRC mismatched in the received Async-Event msg \n\n");
					}
					/* Checksum mismatched in the received msg */
					else if (errTemp == RL_RET_CODE_CHKSUM_FAILED)
					{
						printf("Checksum mismatched in the received msg \n\n");
					}
					/* Polling to HostIRQ is timed out,
					   i.e. Device didn't respond to CNYS from the Host */
					else if (errTemp == RL_RET_CODE_HOSTIRQ_TIMEOUT)
					{
						printf("HostIRQ polling timed out \n\n");
					}
					/* If any of OSI call-back function returns non-zero value */
					else if (errTemp == RL_RET_CODE_RADAR_OSIF_ERROR)
					{
						printf("mmWaveLink OS_IF error \n\n");
					}
					break;
				}
			}
            break;
        }
        default:
        {
            printf("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
            break;
        }
    }
}

/** @fn int MMWL_enableDevice(unsigned char deviceIndex)
*
*   @brief Performs SOP and enables the device.
*
*   @param[in] deviceIndex
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Slave device API.
*/
int MMWL_enableDevice(unsigned char deviceIndex)
{
    int retVal = RL_RET_CODE_OK;
    /* Enable device in Functional Mode (SOP-4) */
    printf("rlDeviceEnable Callback is called by mmWaveLink for Device Index [%d]\n\n", deviceIndex);
    return rlsEnableDevice(deviceIndex);
}

/** @fn int MMWL_disableDevice(unsigned char deviceIndex)
*
*   @brief disables the device.
*
*   @param[in] deviceIndex
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Slave device API.
*/
int MMWL_disableDevice(unsigned char deviceIndex)
{
    printf("rlDeviceDisable Callback is called by mmWaveLink for Device Index [%d]\n\n", deviceIndex);
    return rlsDisableDevice(deviceIndex);
}

/** @fn int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
*
*   @brief Compute the CRC of given data
*
*   @param[in] data - message data buffer pointer
*    @param[in] dataLen - length of data buffer
*    @param[in] crcLen - length of crc 2/4/8 bytes
*    @param[out] outCrc - computed CRC data
*
*   @return int Success - 0, Failure - Error Code
*
*   Compute the CRC of given data
*/
int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
{
    uint64_t crcResult = computeCRC(data, dataLen, (16 << crcLen));
    memcpy(outCrc, &crcResult, (2 << crcLen));
    return 0;
}
extern unsigned char i2cAddr[RLS_NUM_CONNECTED_DEVICES_MAX];
/** @fn int MMWL_powerOnMaster(deviceMap)
*
*   @brief Power on Master API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Master API.
*/
int MMWL_powerOnMaster(unsigned char deviceMap, bool downloadFwMode)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    /*
     \subsection     porting_step1   Step 1 - Define mmWaveLink client callback structure
    The mmWaveLink framework is ported to different platforms using mmWaveLink client callbacks. These
    callbacks are grouped as different structures such as OS callbacks, Communication Interface
    callbacks and others. Application needs to define these callbacks and initialize the mmWaveLink
    framework with the structure.

     Refer to \ref rlClientCbs_t for more details
     */
    rlClientCbs_t clientCtx = { 0 };

    if (downloadFwMode){
        clientCtx.crcType=1;
        clientCtx.ackTimeout=50000;
    }else{
        /*Read all the parameters from config file*/
        MMWL_readPowerOnMaster(&clientCtx);
    }

    /* store CRC Type which has been read from mmwaveconfig.txt file */
    gAwr2243CrcType = clientCtx.crcType;

    /*
    \subsection     porting_step2   Step 2 - Implement Communication Interface Callbacks
    The mmWaveLink device support several standard communication protocol among SPI and MailBox
    Depending on device variant, one need to choose the communication channel. For e.g
    xWR1443/xWR1642 requires Mailbox interface and AWR2243 supports SPI interface.
    The interface for this communication channel should include 4 simple access functions:
    -# rlComIfOpen
    -# rlComIfClose
    -# rlComIfRead
    -# rlComIfWrite

    Refer to \ref rlComIfCbs_t for interface details
    */
	if (rlDevGlobalCfgArgs.TransferMode == 0)
	{
		clientCtx.comIfCb.rlComIfOpen = rlsCommOpen;
		clientCtx.comIfCb.rlComIfClose = rlsCommClose;
		clientCtx.comIfCb.rlComIfRead = rlsSpiRead;
		clientCtx.comIfCb.rlComIfWrite = rlsSpiWrite;
	}
	else
	{
		clientCtx.comIfCb.rlComIfOpen = rlsCommOpen;
		clientCtx.comIfCb.rlComIfClose = rlsCommClose;
		clientCtx.comIfCb.rlComIfRead = rlsI2cRead;
		clientCtx.comIfCb.rlComIfWrite = rlsI2cWrite;
		i2cAddr[0] = 0x28;
	}

    /*   \subsection     porting_step3   Step 3 - Implement Device Control Interface
    The mmWaveLink driver internally powers on/off the mmWave device. The exact implementation of
    these interface is platform dependent, hence you need to implement below functions:
    -# rlDeviceEnable
    -# rlDeviceDisable
    -# rlRegisterInterruptHandler

    Refer to \ref rlDeviceCtrlCbs_t for interface details
    */
    clientCtx.devCtrlCb.rlDeviceDisable = MMWL_disableDevice;
    clientCtx.devCtrlCb.rlDeviceEnable = MMWL_enableDevice;
    clientCtx.devCtrlCb.rlDeviceMaskHostIrq = rlsCommIRQMask;
    clientCtx.devCtrlCb.rlDeviceUnMaskHostIrq = rlsCommIRQUnMask;
    clientCtx.devCtrlCb.rlRegisterInterruptHandler = rlsRegisterInterruptHandler;
    clientCtx.devCtrlCb.rlDeviceWaitIrqStatus = rlsDeviceWaitIrqStatus;

    /*  \subsection     porting_step4     Step 4 - Implement Event Handlers
    The mmWaveLink driver reports asynchronous event indicating mmWave device status, exceptions
    etc. Application can register this callback to receive these notification and take appropriate
    actions

    Refer to \ref rlEventCbs_t for interface details*/
    clientCtx.eventCb.rlAsyncEvent = MMWL_asyncEventHandler;

    /*  \subsection     porting_step5     Step 5 - Implement OS Interface
    The mmWaveLink driver can work in both OS and NonOS environment. If Application prefers to use
    operating system, it needs to implement basic OS routines such as tasks, mutex and Semaphore
    And in case of non-OS environment it needs to implement equivalent tasks, mutex and sempaphore
    for non-OS.

    Refer to \ref rlOsiCbs_t for interface details
    */
    #ifdef NON_OS_ENVIRONMENT
    /* Mutex */
    clientCtx.osiCb.mutex.rlOsiMutexCreate = rlLockObjCreate;
    clientCtx.osiCb.mutex.rlOsiMutexLock = rlLockObjLock;
    clientCtx.osiCb.mutex.rlOsiMutexUnLock = rlLockObjUnlock;
    clientCtx.osiCb.mutex.rlOsiMutexDelete = rlLockObjDelete;

    /* Semaphore */
    clientCtx.osiCb.sem.rlOsiSemCreate = rlSyncObjCreate;
    clientCtx.osiCb.sem.rlOsiSemWait = rlSyncObjWait;
    clientCtx.osiCb.sem.rlOsiSemSignal = rlSyncObjSignal;
    clientCtx.osiCb.sem.rlOsiSemDelete = rlSyncObjDelete;

    /* Spawn Task */
    clientCtx.osiCb.queue.rlOsiSpawn = (RL_P_OS_SPAWN_FUNC_PTR)rlSpawn;
    
    /* Sleep/Delay Callback*/
    clientCtx.timerCb.rlDelay = (RL_P_OS_DELAY_FUNC_PTR)rlAppSleep;
    #else
    /* Mutex */
    clientCtx.osiCb.mutex.rlOsiMutexCreate = (rlInt32_t (*)(void**, rlInt8_t*))osiLockObjCreate;
    clientCtx.osiCb.mutex.rlOsiMutexLock = (rlInt32_t (*)(void**, rlOsiTime_t))osiLockObjLock;
    clientCtx.osiCb.mutex.rlOsiMutexUnLock = (rlInt32_t (*)(void**))osiLockObjUnlock;
    clientCtx.osiCb.mutex.rlOsiMutexDelete = (rlInt32_t (*)(void**))osiLockObjDelete;

    /* Semaphore */
    clientCtx.osiCb.sem.rlOsiSemCreate = (rlInt32_t (*)(void**, rlInt8_t*))osiSyncObjCreate;
    clientCtx.osiCb.sem.rlOsiSemWait = (rlInt32_t (*)(void**, rlOsiTime_t))osiSyncObjWait;
    clientCtx.osiCb.sem.rlOsiSemSignal = (rlInt32_t (*)(void**))osiSyncObjSignal;
    clientCtx.osiCb.sem.rlOsiSemDelete = (rlInt32_t (*)(void**))osiSyncObjDelete;

    /* Spawn Task */
    clientCtx.osiCb.queue.rlOsiSpawn = (RL_P_OS_SPAWN_FUNC_PTR)osiExecute;

    /* Sleep/Delay Callback*/
    clientCtx.timerCb.rlDelay = (RL_P_OS_DELAY_FUNC_PTR)osiSleep;
    #endif
    
	/* Logging in mmWavelink*/
	if (rlDevGlobalCfgArgs.EnableMmwlLogging == 1)
	{	
		clientCtx.dbgCb.dbgLevel = RL_DBG_LEVEL_DATABYTE;
		clientCtx.dbgCb.rlPrint = MMWAVELINK_LOGGING;
	}

    /*  \subsection     porting_step6     Step 6 - Implement CRC Interface
    The mmWaveLink driver uses CRC for message integrity. If Application prefers to use
    CRC, it needs to implement CRC routine.

    Refer to \ref rlCrcCbs_t for interface details
    */
    clientCtx.crcCb.rlComputeCRC = MMWL_computeCRC;

    /*  \subsection     porting_step7     Step 7 - Define Platform
    The mmWaveLink driver can be configured to run on different platform by
    passing appropriate platform and device type
    */
    clientCtx.platform = RL_PLATFORM_HOST;
    clientCtx.arDevType = RL_AR_DEVICETYPE_22XX;

    /*clear all the interupts flag*/
    mmwl_bInitComp = 0;
    mmwl_bStartComp = 0U;
    mmwl_bRfInitComp = 0U;

    /*  \subsection     porting_step8     step 8 - Call Power ON API and pass client context
    The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
    initializes buffers, register interrupts, bring mmWave front end out of reset.
    */
    retVal = rlDevicePowerOn(deviceMap, clientCtx);

    /*  \subsection     porting_step9     step 9 - Test if porting is successful
    Once configuration is complete and mmWave device is powered On, mmWaveLink driver receives
    asynchronous event from mmWave device and notifies application using
    asynchronous event callback.
    Refer to \ref MMWL_asyncEventHandler for event details
	@Note: In case of ES1.0 sample application needs to wait for MSS CPU fault as well with some timeout.
    */
    while (mmwl_bInitComp == 0U)
    {
        #ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
        rlAppSleep(5);
        #else
        osiSleep(1); /*Sleep 1 msec*/
        #endif
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bInitComp = 0U;
    return retVal;
}

int MMWL_fileWrite(unsigned char deviceMap,
                unsigned short remChunks,
                unsigned short chunkLen,
                unsigned char *chunk)
{
    int ret_val = -1;

    rlFileData_t fileChunk = { 0 };
    fileChunk.chunkLen = chunkLen;
    memcpy(fileChunk.fData, chunk, chunkLen);

    ret_val = rlDeviceFileDownload(deviceMap, &fileChunk, remChunks);
    return ret_val;
}

/** @fn int MMWL_fileDownload((unsigned char deviceMap,
                  mmwlFileType_t fileType,
                  unsigned int fileLen)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] fileType - firmware/file type
*    @param[in] fileLen - firmware/file length
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_fileDownload(unsigned char deviceMap,
                  unsigned int fileLen)
{
	/* By default xwr22xx_metaImage.h will not contain the CRC which is present at
	   the end of corresponding bin file.
	   Hence, the 4 bytes of CRC is to be added while downloading the image to sFlash */
    unsigned int imgLen = fileLen + 4; 
    int ret_val = -1;
    int mmwl_iRemChunks = 0;
    unsigned short usChunkLen = 0U;
    unsigned int iNumChunks = 0U;
    unsigned short usLastChunkLen = 0;
    unsigned short usFirstChunkLen = 0;
    unsigned short usProgress = 0;

    /*First Chunk*/
    unsigned char firstChunk[MMWL_FW_CHUNK_SIZE];
    unsigned char* pmmwl_imgBuffer = NULL;
	unsigned char CRCBuffer[4] = { 0 };
	unsigned char* pCRC_Buffer = NULL;

	if (gMmwaveSensorEs1_1 == AWR2243_ES1_1)
	{
		/* Patch file header file included for ES1.1 device */
		pmmwl_imgBuffer = (unsigned char*)&metaImage[0];
	}
	else
	{
		/* select the ES1.0 RAM metaimage header file content here.
		Include xwr22xx_metaImage.h from different DFP location
		*/
		pmmwl_imgBuffer = (unsigned char*)&metaImage[0];
	}
	pCRC_Buffer = &CRCBuffer[0];

	//Calculating 32-bit CRC for the metaimage
	MMWL_computeCRC(pmmwl_imgBuffer, fileLen, 1, pCRC_Buffer);

	/* Attaching 4 bytes of CRC at the end of the last chunk */
	*(pmmwl_imgBuffer + MMWL_META_IMG_FILE_SIZE) = CRCBuffer[0];
	*(pmmwl_imgBuffer + MMWL_META_IMG_FILE_SIZE + 1) = CRCBuffer[1];
	*(pmmwl_imgBuffer + MMWL_META_IMG_FILE_SIZE + 2) = CRCBuffer[2];
	*(pmmwl_imgBuffer + MMWL_META_IMG_FILE_SIZE + 3) = CRCBuffer[3];

    if(pmmwl_imgBuffer == NULL)
    {
        printf("MMWL_fileDwld Fail : File Buffer is NULL \n\n\r");
        return -1;
    }

    /*Download to Device*/
    usChunkLen = MMWL_FW_CHUNK_SIZE;
    iNumChunks = (imgLen + 8) / usChunkLen;
    mmwl_iRemChunks = iNumChunks;

    if (mmwl_iRemChunks > 0)
    {
        usLastChunkLen = (imgLen + 8) % usChunkLen;
        usFirstChunkLen = MMWL_FW_CHUNK_SIZE;
		mmwl_iRemChunks += 1;
    }
    else
    {
        usFirstChunkLen = imgLen + 8;
    }

	/* SPI to sFlash file download - Filetype must be set to 4 */
    *((unsigned int*)&firstChunk[0]) = (unsigned int)MMWL_FILETYPE_META_IMG;
    *((unsigned int*)&firstChunk[4]) = (unsigned int)imgLen;
    memcpy((char*)&firstChunk[8], (char*)pmmwl_imgBuffer,
                usFirstChunkLen - 8);

	/* Set mmWaveLink ACK timeout to 3-4 sec since the device will erase the sFlash
	   and send ACK for the first chunk SPI command */
	rlDeviceConfigureAckTimeout(10000);

    ret_val = MMWL_fileWrite(deviceMap, (mmwl_iRemChunks-1), usFirstChunkLen,
                              firstChunk);
    if (ret_val < 0)
    {
        printf("MMWL_fileDwld Fail : Ftype: %d\n\n\r", MMWL_FILETYPE_META_IMG);
        return ret_val;
    }
    pmmwl_imgBuffer += MMWL_FW_FIRST_CHUNK_SIZE;
    mmwl_iRemChunks--;

    if(mmwl_iRemChunks > 0)
    {
        printf("Download in Progress: ");
    }
    /*Remaining Chunk*/
    while (mmwl_iRemChunks > 0)
    {
        usProgress = (((iNumChunks - mmwl_iRemChunks) * 100) / iNumChunks);
        printf("%d%%..", usProgress);

		/* Last chunk */
		if ((mmwl_iRemChunks == 1) && (usLastChunkLen > 0))
		{
			ret_val = MMWL_fileWrite(deviceMap, 0, usLastChunkLen,
				pmmwl_imgBuffer);
			if (ret_val < 0)
			{
				printf("MMWL_fileDwld last chunk Fail : Ftype: %d\n\n\r",
					MMWL_FILETYPE_META_IMG);
				return ret_val;
			}
		}
		else
		{
			ret_val = MMWL_fileWrite(deviceMap, (mmwl_iRemChunks - 1),
				MMWL_FW_CHUNK_SIZE, pmmwl_imgBuffer);

			if (ret_val < 0)
			{
				printf("\n\n\r MMWL_fileDwld rem chunk Fail : Ftype: %d\n\n\r",
					MMWL_FILETYPE_META_IMG);
				return ret_val;
			}
			pmmwl_imgBuffer += MMWL_FW_CHUNK_SIZE;
		}

        mmwl_iRemChunks--;
    }
	/* Revert back to the original ACK timeout for the remaining chunks */
	rlDeviceConfigureAckTimeout(1000);
     printf("Done!\n\n");
    return ret_val;
}

/** @fn int MMWL_firmwareDownload(deviceMap)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_firmwareDownload(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;

    /* Meta Image download */
    printf("Meta Image download started for deviceMap %u\n\n",
        deviceMap);
    retVal = MMWL_fileDownload(deviceMap, MMWL_META_IMG_FILE_SIZE);
    printf("Meta Image download complete ret = %d\n\n", retVal);

    return retVal;
}

/** @fn int MMWL_rfEnable(deviceMap)
*
*   @brief RFenable API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFenable API.
*/
int MMWL_rfEnable(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    retVal = rlDeviceRfStart(deviceMap);
    while (mmwl_bStartComp == 0U)
    {
		#ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
        rlAppSleep(5);
        #else
        osiSleep(1); /*Sleep 1 msec*/
        #endif
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_START_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bStartComp = 0;

    if(retVal == RL_RET_CODE_OK)
    {
        rlVersion_t verArgs = {0};
		rlRfDieIdCfg_t dieId = { 0 };
        retVal = rlDeviceGetVersion(deviceMap,&verArgs);

        printf("RF Version [%2d.%2d.%2d.%2d] \nMSS version [%2d.%2d.%2d.%2d] \nmmWaveLink version [%2d.%2d.%2d.%2d]\n\n",
            verArgs.rf.fwMajor, verArgs.rf.fwMinor, verArgs.rf.fwBuild, verArgs.rf.fwDebug,
            verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
            verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
        printf("RF Patch Version [%2d.%2d.%2d.%2d] \nMSS Patch version [%2d.%2d.%2d.%2d]\n\n",
            verArgs.rf.patchMajor, verArgs.rf.patchMinor, ((verArgs.rf.patchBuildDebug & 0xF0) >> 4), (verArgs.rf.patchBuildDebug & 0x0F),
            verArgs.master.patchMajor, verArgs.master.patchMinor, ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F));
        if(verArgs.rf.patchMajor==0 || verArgs.master.patchMajor==0)
            printf("[Warning!!!!!]: No patch detected, please download firmware first!\n\n");
		retVal = rlGetRfDieId(deviceMap, &dieId);
    }
    return retVal;
}

/** @fn int MMWL_dataFmtConfig(unsigned char deviceMap)
*
*   @brief Data Format Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Data Format Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_dataFmtConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataFmtCfg_t dataFmtCfgArgs = { 0 };

    /*dataFmtCfgArgs from config file*/
    MMWL_readDataFmtConfig(&dataFmtCfgArgs);

    retVal = rlDeviceSetDataFmtConfig(deviceMap, &dataFmtCfgArgs);
    return retVal;
}

/** @fn int MMWL_ldoBypassConfig(unsigned char deviceMap)
*
*   @brief LDO Bypass Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LDO Bypass Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_ldoBypassConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfLdoBypassCfg_t rfLdoBypassCfgArgs = { 0 };

    printf("Calling rlRfSetLdoBypassConfig With Bypass [%d] \n\n",
        rfLdoBypassCfgArgs.ldoBypassEnable);

    retVal = rlRfSetLdoBypassConfig(deviceMap, &rfLdoBypassCfgArgs);
    return retVal;
}

/** @fn int MMWL_adcOutConfig(unsigned char deviceMap)
*
*   @brief ADC Configuration API
*
*   @return Success - 0, Failure - Error Code
*
*   ADC Configuration API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_adcOutConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    rlAdcOutCfg_t adcOutCfgArgs = { 0 };

    /*read adcOutCfgArgs from config file*/
    MMWL_readAdcOutConfig(&adcOutCfgArgs);


    printf("Calling rlSetAdcOutConfig With [%d]ADC Bits and [%d]ADC Format \n\n",
        adcOutCfgArgs.fmt.b2AdcBits, adcOutCfgArgs.fmt.b2AdcOutFmt);

    retVal = rlSetAdcOutConfig(deviceMap, &adcOutCfgArgs);
    return retVal;
}

/** @fn int MMWL_channelConfig(unsigned char deviceMap,
                               unsigned short cascading)
*
*   @brief Channel Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Channel Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_channelConfig(unsigned char deviceMap,
                       unsigned short cascade)
{
    int retVal = RL_RET_CODE_OK;
    /* TBD - Read GUI Values */
    rlChanCfg_t rfChanCfgArgs = { 0 };

    /*read arguments from config file*/
    MMWL_readChannelConfig(&rfChanCfgArgs, cascade);

// #if (ENABLE_TX2)
//     rfChanCfgArgs.txChannelEn |= (1 << 2); // Enable TX2
// #endif

    printf("Calling rlSetChannelConfig With [%d]Rx and [%d]Tx Channel Enabled \n\n",
           rfChanCfgArgs.rxChannelEn, rfChanCfgArgs.txChannelEn);

    retVal = rlSetChannelConfig(deviceMap, &rfChanCfgArgs);
    return retVal;
}

/** @fn int MMWL_setAsyncEventDir(unsigned char deviceMap)
*
*   @brief Update async event message direction and CRC type of Async event
*           from AWR2243 radarSS to Host
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Update async event message direction and CRC type of Async event
*   from AWR2243 radarSS to Host
*/
int MMWL_setAsyncEventDir(unsigned char  deviceMap)
{
    int32_t         retVal;
    /* set global and monitoring async event direction to Host */
    rfDevCfg.aeDirection = 0x05;
    /* Set the CRC type of Async event received from radarSS */
    rfDevCfg.aeCrcConfig = gAwr2243CrcType;
    retVal = rlRfSetDeviceCfg(deviceMap, &rfDevCfg);
    return retVal;
}

/** @fn int MMWL_setMiscConfig(unsigned char deviceMap)
*
*   @brief Sets misc feature such as per chirp phase shifter and Advance chirp
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Sets misc feature such as per chirp phase shifter and Advance chirp
*/
int MMWL_setMiscConfig(unsigned char deviceMap)
{
	int32_t         retVal;
	rlRfMiscConf_t MiscCfg = { 0 };
	/* Enable Adv chirp feature 
		b0 PERCHIRP_PHASESHIFTER_EN
		b1 ADVANCE_CHIRP_CONFIG_EN  */
	MiscCfg.miscCtl = 0x3;
	retVal = rlRfSetMiscConfig(deviceMap, &MiscCfg);
	return retVal;
}

/** @fn int MMWL_setDeviceCrcType(unsigned char deviceMap)
*
*   @brief Set CRC type of async event from AWR2243 MasterSS
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Set CRC type of async event from AWR2243 MasterSS
*/
int MMWL_setDeviceCrcType(unsigned char deviceMap)
{
    int32_t         retVal;
    rlDevMiscCfg_t devMiscCfg = {0};
    /* Set the CRC Type for Async Event from MSS */
    devMiscCfg.aeCrcConfig = gAwr2243CrcType;
    retVal = rlDeviceSetMiscConfig(deviceMap, &devMiscCfg);
    return retVal;
}

/** @fn int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade)
*
*   @brief Channel, ADC,Data format configuration API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] unsigned int cascade
*
*   @return int Success - 0, Failure - Error Code
*
*   Channel, ADC,Data format configuration API.
*/
int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade)
{
    int retVal = RL_RET_CODE_OK;

    /* Set which Rx and Tx channels will be enable of the device */
    retVal = MMWL_channelConfig(deviceMap, cascade);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Channel Config failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Channel Configuration success for deviceMap %u\n\n", deviceMap);
    }
    /* ADC out data format configuration */
    retVal = MMWL_adcOutConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AdcOut Config failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("AdcOut Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* LDO bypass configuration */
    retVal = MMWL_ldoBypassConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LDO Bypass Config failed for deviceMap %u with error code %d\n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LDO Bypass Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* Data format configuration */
    retVal = MMWL_dataFmtConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data format Configuration failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Data format Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* low power configuration */
    retVal = MMWL_lowPowerConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Low Power Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Low Power Configuration success for deviceMap %u \n\n", deviceMap);
    }
    
    /* APLL Synth BW configuration */
    retVal = MMWL_ApllSynthBwConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("APLL Synth BW Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("APLL Synth BW Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /* Async event direction and control configuration for RadarSS */
    retVal = MMWL_setAsyncEventDir(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AsyncEvent Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("AsyncEvent Configuration success for deviceMap %u \n\n", deviceMap);
    }

	if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE)
	{
		/* Misc control configuration for RadarSS */
		/* This API enables the Advanced chirp and per chirp phase shifter features */
		retVal = MMWL_setMiscConfig(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Misc control configuration failed for deviceMap %u with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Misc control configuration success for deviceMap %u \n\n", deviceMap);
		}
	}
	
    return retVal;
}

/** @fn int MMWL_rfInit(unsigned char deviceMap)
*
*   @brief RFinit API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFinit API.
*/
int MMWL_rfInit(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    mmwl_bRfInitComp = 0;

	if (rlDevGlobalCfgArgs.CalibEnable == TRUE)
	{
		rlRfInitCalConf_t rfCalibCfgArgs = { 0 };

		/* Calibration store */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 1)
		{
			/* Enable only required boot-time calibrations, by default all are enabled in the device */
			rfCalibCfgArgs.calibEnMask = 0x1FF0;
		}
		/* Calibration restore */
		else
		{
			/* Disable all the boot-time calibrations, by default all are enabled in the device */
			rfCalibCfgArgs.calibEnMask = 0x0;
		}
		/* RF Init Calibration Configuration */
		retVal = rlRfInitCalibConfig(deviceMap, &rfCalibCfgArgs);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("RF Init Calibration Configuration failed for deviceMap %u with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("RF Init Calibration Configuration success for deviceMap %u \n\n", deviceMap);
		}

		/* Calibration restore */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 0)
		{
			/* Load Phase shifter Calibration Data from a file */
			retVal = MMWL_LoadPhShiftCalibDataFromFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Load Phase shifter Calibration Data from a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Load Phase shifter Calibration Data from a file success for deviceMap %u \n\n", deviceMap);
			}

			/* Phase shifter Calibration Data Restore Configuration */
			retVal = rlRfPhShiftCalibDataRestore(deviceMap, &phShiftCalibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Phase shifter Calibration Data Restore Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Phase shifter Calibration Data Restore Configuration success for deviceMap %u \n\n", deviceMap);
			}
            
			/* Load Calibration Data from a file */
			retVal = MMWL_LoadCalibDataFromFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Load Calibration Data from a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Load Calibration Data from a file success for deviceMap %u \n\n", deviceMap);
			}

			/* Calibration Data Restore Configuration */
			retVal = rlRfCalibDataRestore(deviceMap, &calibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Calibration Data Restore Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Calibration Data Restore Configuration success for deviceMap %u \n\n", deviceMap);
				while (mmwl_bRfInitComp == 0U)
				{
					#ifdef NON_OS_ENVIRONMENT
                    rlNonOsMainLoopTask();
                    rlAppSleep(5);
                    #else
                    osiSleep(1); /*Sleep 1 msec*/
                    #endif
					timeOutCnt++;
					if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
					{
						retVal = RL_RET_CODE_RESP_TIMEOUT;
						break;
					}
				}
				mmwl_bRfInitComp = 0;
			}
		}
	}
    retVal = rlRfInit(deviceMap);
    while (mmwl_bRfInitComp == 0U)
    {
		#ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
        rlAppSleep(5);
        #else
        osiSleep(1); /*Sleep 1 msec*/
        #endif
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bRfInitComp = 0;
	if (rlDevGlobalCfgArgs.CalibEnable == TRUE)
	{
		/* Calibration Store */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 1)
		{
			/* If all the calibration is done successfully as per above Async-event status,
			   now get the calibration data from the device */
			   /* Calibration Data Store Configuration */
			retVal = rlRfCalibDataStore(deviceMap, &calibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Calibration Data Store Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Calibration Data Store Configuration success for deviceMap %u \n\n", deviceMap);
			}
            
            /* Phase shifter Calibration Data Store Configuration */
			retVal = rlRfPhShiftCalibDataStore(deviceMap, &phShiftCalibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Phase shifter Calibration Data Store Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Phase shifter Calibration Data Store Configuration success for deviceMap %u \n\n", deviceMap);
			}

			/* Save Calibration Data to a file */
			retVal = MMWL_saveCalibDataToFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Save Calibration Data to a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Save Calibration Data to a file success for deviceMap %u \n\n", deviceMap);
			}
            
			/* Save Phase shifter Calibration Data to a file */
			retVal = MMWL_savePhShiftCalibDataToFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Save Phase shifter Calibration Data to a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Save Phase shifter Calibration Data to a file success for deviceMap %u \n\n", deviceMap);
			}
		}
	}
    return retVal;
}

/** @fn int MMWL_saveCalibDataToFile(unsigned char deviceMap)
*
*   @brief Save Calibration Data to a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Save Calibration Data to a file.
*/
int MMWL_saveCalibDataToFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int i,j;
	int index = 0;
	char CalibdataBuff[2500] = { 0 };
    #ifdef _WIN32
        CalibrationDataPtr = _fsopen("CalibrationData.txt", "wt", _SH_DENYWR);
    #elif __linux__
        CalibrationDataPtr = fopen("CalibrationData.txt", "at+");
    #endif
	

	/* Copy data from all the 3 chunks */
	for (i = 0; i < 3; i++)
	{
		sprintf(CalibdataBuff + strlen(CalibdataBuff), "0x%04x\n", calibData.calibChunk[i].numOfChunk);
		sprintf(CalibdataBuff + strlen(CalibdataBuff), "0x%04x\n", calibData.calibChunk[i].chunkId);
		/* Store 224 bytes of data in each chunk in terms of 2 bytes per line */
		for (j = 0; j < 224; j+=2)
		{
			sprintf(CalibdataBuff + strlen(CalibdataBuff), "0x%02x%02x\n", calibData.calibChunk[i].calData[j+1], calibData.calibChunk[i].calData[j]);
		}
	}

	fwrite(CalibdataBuff, sizeof(char), strlen(CalibdataBuff), CalibrationDataPtr);
	fflush(CalibrationDataPtr);

	if (CalibrationDataPtr != NULL)
		fclose(CalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_savePhShiftCalibDataToFile(unsigned char deviceMap)
*
*   @brief Save Phase shifter Calibration Data to a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Save Phase shifter Calibration Data to a file.
*/
int MMWL_savePhShiftCalibDataToFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int i,j;
	int index = 0;
	char PhShiftCalibdataBuff[2500] = { 0 };
    #ifdef _WIN32
        PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData.txt", "wt", _SH_DENYWR);
    #elif __linux__
        PhShiftCalibrationDataPtr = fopen("PhShiftCalibrationData.txt", "at+");
    #endif
	

	/* Copy data from all the 3 chunks */
	for (i = 0; i < 3; i++)
	{
		sprintf(PhShiftCalibdataBuff + strlen(PhShiftCalibdataBuff), "0x%02x\n", \
                phShiftCalibData.PhShiftcalibChunk[i].txIndex);
		sprintf(PhShiftCalibdataBuff + strlen(PhShiftCalibdataBuff), "0x%02x\n", \
                phShiftCalibData.PhShiftcalibChunk[i].calibApply);
		/* Store 128 bytes of data in each chunk in terms of 1 byte per line */
		for (j = 0; j < 128; j++)
		{
			sprintf(PhShiftCalibdataBuff + strlen(PhShiftCalibdataBuff), "0x%02x\n", \
                    phShiftCalibData.PhShiftcalibChunk[i].observedPhShiftData[j]);
		}
	}

	fwrite(PhShiftCalibdataBuff, sizeof(char), strlen(PhShiftCalibdataBuff), PhShiftCalibrationDataPtr);
	fflush(PhShiftCalibrationDataPtr);

	if (PhShiftCalibrationDataPtr != NULL)
		fclose(PhShiftCalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_LoadCalibDataFromFile(unsigned char deviceMap)
*
*   @brief Load Calibration Data from a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Load Calibration Data from a file.
*/
int MMWL_LoadCalibDataFromFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int index = 0;
	char CalibdataBuff[2500] = { 0 };
	char *s, buff[8], val[100];
	int i = 0;
	char readNumChunks = 0, readChunkId = 0;
	
    #ifdef _WIN32
        CalibrationDataPtr = _fsopen("CalibrationData.txt", "rt", _SH_DENYRD);
    #elif __linux__
        CalibrationDataPtr = fopen("CalibrationData.txt", "at+");
    #endif

	if (CalibrationDataPtr == NULL)
	{
		printf("CalibrationData.txt does not exist or Error opening the file\n\n");
		return -1;
	}

	/*seek the pointer to starting of the file */
	fseek(CalibrationDataPtr, 0, SEEK_SET);

	/*parse the parameters by reading each line of the calib data file*/
	while ((readNumChunks != 3) && (readChunkId != 3))
	{
		unsigned char readDataChunks = 0;
		if ((s = fgets(buff, sizeof buff, CalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				calibData.calibChunk[i].numOfChunk = (rlUInt16_t)strtol(val, NULL, 0);
				readNumChunks++;
			}
		}
		if ((s = fgets(buff, sizeof buff, CalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				calibData.calibChunk[i].chunkId = (rlUInt16_t)strtol(val, NULL, 0);
				readChunkId++;
			}
		}
		while ((readDataChunks != 224) && ((s = fgets(buff, sizeof buff, CalibrationDataPtr)) != NULL))
		{
			/* Parse value from line */
			const char* temp = &buff[0];
			char byte1[3];
			char byte2[3];

			strncpy(byte1, temp +4, 2);
			byte1[2] = '\0';
			if (byte1 == NULL)
			{
				continue;
			}
			else
			{
				calibData.calibChunk[i].calData[readDataChunks] = (rlUInt8_t)strtol(byte1, NULL, 16);
				readDataChunks++;
			}

			strncpy(byte2, temp + 2, 2);
			byte2[2] = '\0';
			if (byte2 == NULL)
			{
				continue;
			}
			else
			{
				calibData.calibChunk[i].calData[readDataChunks] = (rlUInt8_t)strtol(byte2, NULL, 16);
				readDataChunks++;
			}
		}
		i++;
	}

	fflush(CalibrationDataPtr);

	if (CalibrationDataPtr != NULL)
		fclose(CalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_LoadPhShiftCalibDataFromFile(unsigned char deviceMap)
*
*   @brief Load Phase shifter Calibration Data from a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Load Phase shifter Calibration Data from a file.
*/
int MMWL_LoadPhShiftCalibDataFromFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int index = 0;
	char PhShiftCalibdataBuff[2500] = { 0 };
	char *s, buff[8], val[100];
	int i = 0;
	char readNumChunks = 0, readChunkId = 0;
    unsigned char readDataChunks;
	#ifdef _WIN32
        PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData.txt", "rt", _SH_DENYRD);
    #elif __linux__
        PhShiftCalibrationDataPtr = fopen("PhShiftCalibrationData.txt", "at+");
    #endif
	

	if (PhShiftCalibrationDataPtr == NULL)
	{
		printf("PhShiftCalibrationData.txt does not exist or Error opening the file\n\n");
		return -1;
	}

	/*seek the pointer to starting of the file */
	fseek(PhShiftCalibrationDataPtr, 0, SEEK_SET);

	/*parse the parameters by reading each line of the phase shift calib data file*/
	while ((readNumChunks != 3) && (readChunkId != 3))
	{
		readDataChunks = 0;
		if ((s = fgets(buff, sizeof buff, PhShiftCalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				phShiftCalibData.PhShiftcalibChunk[i].txIndex = (rlUInt8_t)strtol(val, NULL, 0);
				readNumChunks++;
			}
		}
		if ((s = fgets(buff, sizeof buff, PhShiftCalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				phShiftCalibData.PhShiftcalibChunk[i].calibApply = (rlUInt8_t)strtol(val, NULL, 0);
				readChunkId++;
			}
		}
		while ((readDataChunks != 128) && ((s = fgets(buff, sizeof buff, PhShiftCalibrationDataPtr)) != NULL))
		{
			/* Parse value from line */
			const char* temp = &buff[0];
			char byte1[5];

			strncpy(byte1, temp, 4);
			byte1[4] = '\0';
			if (byte1 == NULL)
			{
				continue;
			}
			else
			{
				phShiftCalibData.PhShiftcalibChunk[i].observedPhShiftData[readDataChunks] = (rlUInt8_t)strtol(byte1, NULL, 16);
				readDataChunks++;
			}
		}
        phShiftCalibData.PhShiftcalibChunk[i].reserved = 0U;
		i++;
	}

	fflush(PhShiftCalibrationDataPtr);

	if (PhShiftCalibrationDataPtr != NULL)
		fclose(PhShiftCalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_progFiltConfig(unsigned char deviceMap)
*
*   @brief Programmable filter configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Programmable filter configuration API.
*/
int MMWL_progFiltConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfProgFiltConf_t progFiltCnfgArgs = { 0 };

    /*read profileCfgArgs from config file*/
    MMWL_readProgFiltConfig(&progFiltCnfgArgs);

    printf("Calling rlRfSetProgFiltConfig with \ncoeffStartIdx[%d]\nprogFiltLen[%d] GHz\nprogFiltFreqShift[%d] MHz/uS \n\n",
        progFiltCnfgArgs.coeffStartIdx, progFiltCnfgArgs.progFiltLen, progFiltCnfgArgs.progFiltFreqShift);
    retVal = rlRfSetProgFiltConfig(deviceMap, &progFiltCnfgArgs);
    return retVal;
}

/** @fn int MMWL_progFiltCoeffRam(unsigned char deviceMap)
*
*   @brief Programmable Filter coefficient RAM configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Programmable Filter coefficient RAM configuration API.
*/
int MMWL_progFiltCoeffRam(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfProgFiltCoeff_t progFiltCoeffCnfgArgs = { 0 };
    progFiltCoeffCnfgArgs.coeffArray[0] = -876,
    progFiltCoeffCnfgArgs.coeffArray[1] = -272,
    progFiltCoeffCnfgArgs.coeffArray[2] = 1826,
    progFiltCoeffCnfgArgs.coeffArray[3] = -395,
    progFiltCoeffCnfgArgs.coeffArray[4] = -3672,
    progFiltCoeffCnfgArgs.coeffArray[5] = 3336,
    progFiltCoeffCnfgArgs.coeffArray[6] = 15976,
    progFiltCoeffCnfgArgs.coeffArray[7] = 15976,
    progFiltCoeffCnfgArgs.coeffArray[8] = 3336,
    progFiltCoeffCnfgArgs.coeffArray[9] = -3672,
    progFiltCoeffCnfgArgs.coeffArray[10] = -395,
    progFiltCoeffCnfgArgs.coeffArray[11] = 1826,
    progFiltCoeffCnfgArgs.coeffArray[12] = -272,
    progFiltCoeffCnfgArgs.coeffArray[13] = -876,

    printf("Calling rlRfSetProgFiltCoeffRam with \ncoeffArray0[%d]\ncoeffArray1[%d] GHz\ncoeffArray2[%d] MHz/uS \n\n",
    progFiltCoeffCnfgArgs.coeffArray[0], progFiltCoeffCnfgArgs.coeffArray[1], progFiltCoeffCnfgArgs.coeffArray[2]);
    retVal = rlRfSetProgFiltCoeffRam(deviceMap, &progFiltCoeffCnfgArgs);
    return retVal;
}

/** @fn int MMWL_profileConfig(unsigned char deviceMap)
*
*   @brief Profile configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Profile configuration API.
*/
int MMWL_profileConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, i;
	
	if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE)
	{
		/*read profileCfgArgs from config file*/
		MMWL_readProfileConfig(&profileCfgArgs[0], 1U);

		printf("Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n\n",
			profileCfgArgs[0].profileId, (float)((profileCfgArgs[0].startFreqConst * 53.6441803) / (1000 * 1000 * 1000)),
			(float)(profileCfgArgs[0].freqSlopeConst * 48.2797623) / 1000.0);
		/* with this API we can configure 2 profiles (max 4 profiles) at a time */
		retVal = rlSetProfileConfig(deviceMap, 1U, &profileCfgArgs[0U]);
	}
	else
	{
		/*read ProfileCfgArgs_AdvChirp from config file*/
		MMWL_readProfileConfig(&ProfileCfgArgs_AdvChirp[0], 4U);

		for (i = 0; i < 4; i++)
		{
			printf("Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n\n",
				ProfileCfgArgs_AdvChirp[i].profileId, (float)((ProfileCfgArgs_AdvChirp[i].startFreqConst * 53.6441803) / (1000 * 1000 * 1000)),
				(float)(ProfileCfgArgs_AdvChirp[i].freqSlopeConst * 48.2797623) / 1000.0);
		}

		/* configuring 4 profiles at a time */
		retVal = rlSetProfileConfig(deviceMap, 4U, &ProfileCfgArgs_AdvChirp[0U]);
	}
    return retVal;
}

/** @fn int MMWL_chirpConfig(unsigned char deviceMap)
*
*   @brief Chirp configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Chirp configuration API.
*/
int MMWL_chirpConfig(unsigned char deviceMap)
{
    int i, cnt, retVal = RL_RET_CODE_OK;
    rlChirpCfg_t setChirpCfgArgs[512] = {0};

    /*read chirpCfgArgs from config file*/
    cnt=MMWL_readChirpConfig(&setChirpCfgArgs[0], 512);

    for(i=0;i<cnt;i++){
        printf("Chirp cfg %d calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] \n\n", i+1,
                setChirpCfgArgs[0].profileId, setChirpCfgArgs[0].chirpStartIdx,
                setChirpCfgArgs[0].chirpEndIdx);
    }

    /* With this API we can configure max 512 chirp in one call */
    retVal = rlSetChirpConfig(deviceMap, cnt, &setChirpCfgArgs[0U]);

    /* read back Chirp config, to verify that setChirpConfig actually set to Device
      @Note - This examples read back (10+1) num of chirp config for demonstration,
               which user can raise to match with their requirement */
    rlChirpCfg_t* getChirpCfgArgs = new rlChirpCfg_t[cnt]();// assume each chirp cfg contains only one chirp
    retVal = rlGetChirpConfig(deviceMap, 0, cnt-1, &getChirpCfgArgs[0]);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("GetChirp Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
    }
    else
    {
        for (i=0; i < cnt; i++)
        {
            /* @Note- This check assumes that all chirp configs are configured by single setChirpCfgArgs[0] */
            /* compare each chirpConfig parameters to lastly configured via rlDynChirpConfig API */
            if ((getChirpCfgArgs[i].profileId != setChirpCfgArgs[i].profileId) || \
                (getChirpCfgArgs[i].freqSlopeVar != setChirpCfgArgs[i].freqSlopeVar) || \
                (getChirpCfgArgs[i].txEnable != setChirpCfgArgs[i].txEnable) || \
                (getChirpCfgArgs[i].startFreqVar != setChirpCfgArgs[i].startFreqVar) || \
                (getChirpCfgArgs[i].idleTimeVar != setChirpCfgArgs[i].idleTimeVar) || \
                (getChirpCfgArgs[i].adcStartTimeVar != setChirpCfgArgs[i].adcStartTimeVar))
            {
                    printf("*** Failed - Parameters are mismatched GetChirpConfig compare to rlSetChirpConfig *** \n");
                    printf("note: current validation assume each chirp cfg contains only one chirp\n");
                    break;
            }

        }

        if (i == cnt)
        {
            printf("Get chirp configurations are matching with parameters configured during rlSetChirpConfig \n\n");
        }
    }

    return retVal;
}

int MMWL_chirpParamCompare(rlChirpCfg_t * chirpData)
{
    int retVal = RL_RET_CODE_OK, i = 0,j = 0;
    /* compare each chirpConfig parameters to lastly configured via rlDynChirpConfig API */
    while (i <= MAX_GET_CHIRP_CONFIG_IDX)
    {
        if (dynChirpCfgArgs[0].chirpRowSelect == 0x00)
        {
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 3, 16)) || \
                (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 23, 0)) || \
                (chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
        }
        else if (dynChirpCfgArgs[0].chirpRowSelect == 0x10)
        {
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 3, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 3, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 3, 16)))
            {
                break;
            }
            i++;
            chirpData++;
        }
        else if (dynChirpCfgArgs[0].chirpRowSelect == 0x20)
        {
            if (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 23, 0))
            {
                break;
            }
            i++;
            chirpData++;
            if (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 23, 0))
            {
                break;
            }
            i++;
            chirpData++;
            if (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 23, 0))
            {
                break;
            }
            i++;
            chirpData++;
        }
        else if (dynChirpCfgArgs[0].chirpRowSelect == 0x30)
        {
            if ((chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
        }
        j++;
    }
    if (i <= MAX_GET_CHIRP_CONFIG_IDX)
    {
        retVal = -1;
    }
    return retVal;
}
int MMWL_getDynChirpConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK,i = 0, j= 0, chirpNotMatch = 0;
    unsigned short chirpStartIdx;
    rlChirpCfg_t chirpCfgArgs[MAX_GET_CHIRP_CONFIG_IDX+1] = {0};
    if (dynChirpCfgArgs[0].chirpRowSelect == 0x00)
    {
        chirpStartIdx = (dynChirpCfgArgs[0].chirpSegSel * 16);
    }
    else
    {
        chirpStartIdx = (dynChirpCfgArgs[0].chirpSegSel * 48);
    }
    /* get the chirp config for (10+1) chirps for which it's being updated by dynChirpConfig API
       @Note - This examples read back (10+1) num of chirp config for demonstration,
               which user can raise to match with their requirement */
    retVal = rlGetChirpConfig(deviceMap, chirpStartIdx, chirpStartIdx + MAX_GET_CHIRP_CONFIG_IDX, &chirpCfgArgs[0]);

    if (retVal != RL_RET_CODE_OK)
    {
        printf("*** Failed - rlGetChirpConfig failed with %d*** \n\n",retVal);
    }

    retVal = MMWL_chirpParamCompare(&chirpCfgArgs[0]);

    if (retVal != RL_RET_CODE_OK)
    {
        printf("*** Failed - Parameters are mismatched GetChirpConfig compare to dynChirpConfig *** \n\n");
    }
    else
    {
        printf("Get chirp configurations are matching with parameters configured via dynChirpConfig \n\n");
    }

    return retVal;
}

/* Ping LUT buffer offset (starts at 0 bytes and ends at 52 bytes) */
rlAdvChirpDynLUTAddrOffCfg_t advChirpDynLUTOffsetCfg1;
/* Pong LUT buffer offset (starts at 100 bytes and ends at 152 bytes) */
rlAdvChirpDynLUTAddrOffCfg_t advChirpDynLUTOffsetCfg2;

/** @fn int MMWL_advChirpConfigAll(unsigned char deviceMap)
*
*   @brief Advanced chirp configuration API.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Advanced chirp configuration API.
*/
int MMWL_advChirpConfigAll(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int bufIdx = 0;

	rlAdvChirpCfg_t AdvChirpCfgArgs = { 0 };
	rlFillLUTParams_t rlFillLUTParamsArgs = { 0 };
	rlAdvChirpLUTProfileCfg_t AdvChirpLUTProfileCfgArgs = { 0 };
	/* profile ID LUT dither local buffer */
	rlUInt8_t ProfileCfgData[4] = { 0 };
	rlAdvChirpLUTStartFreqCfg_t AdvChirpLUTStartFreqCfgArgs = { 0 };
	/* Frequency start LUT dither local buffer */
	rlInt16_t StartFreqData[4] = { 0 }; /* Change this based on the chirp param size*/
	rlAdvChirpLUTFreqSlopeCfg_t AdvChirpLUTFreqSlopeCfgArgs = { 0 };
	/* Frequency slope LUT dither local buffer */
	rlInt8_t FreqSlopeData[4] = { 0 };
	rlAdvChirpLUTIdleTimeCfg_t AdvChirpLUTIdleTimeCfgArgs = { 0 };
	/* Idle Time LUT dither local buffer */
	rlInt16_t IdleTimeData[4] = { 0 }; /* Change this based on the chirp param size*/
	rlAdvChirpLUTADCTimeCfg_t AdvChirpLUTADCTimeCfgArgs = { 0 };
	/* ADC start Time LUT dither local buffer */
	rlInt16_t ADCStartTimeData[4] = { 0 }; /* Change this based on the chirp param size*/
	rlAdvChirpLUTTxEnCfg_t AdvChirpLUTTxEnCfgArgs = { 0 };
	/* Tx Enable LUT dither local buffer */
	rlUInt8_t TxEnCfgData[4] = { 0 };
	rlAdvChirpLUTBpmEnCfg_t AdvChirpLUTBpmEnCfgArgs = { 0 };
	/* BPM Enable LUT dither local buffer */
	rlUInt8_t BpmEnCfgData[4] = { 0 };
	rlAdvChirpLUTTx0PhShiftCfg_t AdvChirpLUTTx0PhShiftCfgArgs = { 0 };
	/* Tx0 Phase shifter LUT dither local buffer */
	rlInt8_t Tx0PhShiftData[4] = { 0 };
	rlAdvChirpLUTTx1PhShiftCfg_t AdvChirpLUTTx1PhShiftCfgArgs = { 0 };
	/* Tx1 Phase shifter LUT dither local buffer */
	rlInt8_t Tx1PhShiftData[4] = { 0 };
	rlAdvChirpLUTTx2PhShiftCfg_t AdvChirpLUTTx2PhShiftCfgArgs = { 0 };
	/* Tx2 Phase shifter LUT dither local buffer */
	rlInt8_t Tx2PhShiftData[4] = { 0 };
	rlAdvChirpLUTCfg_t rlAdvChirpLUTCfgArgs = { 0 };
	
	/* Configuring Profile (Param Index = 0) */
	/* Fixed delta dither is not supported for profile parameter */
	/* Configuring 4 unique profile index (0,1,2,3) in the generic SW LUT - LUT Reset period (4) */
	/* The new profile index is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Profile   
	       0         0
		   1         1
		   2         2
		   3         3
		   4         0     and so on */
	/* LUT start address offset for the profile chirp parameter is made 0 */
	/* AdvChirpLUTData[0] is the start address offset (Offset = 0), Each data parameter is 4 bits */
	/* Number of unique LUT dither parameters (4) */
	AdvChirpCfgArgs.chirpParamIdx = 0;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1; /* Make it zero if LUT index zero (PF0) to be applied for all the chirps */
	AdvChirpCfgArgs.lutPatternAddressOffset = 0;
	AdvChirpCfgArgs.numOfPatterns = 4; /* Uses 4 different profiles. Make it one, if it requires only a single profile */

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpProfileConfig(&AdvChirpLUTProfileCfgArgs);
	printf("Saving advChirpLUTProfileConfig with \nLUTAddrOff[%d]\nProfileCfgData[0]=[%d]\nProfileCfgData[1]=[%d]\nProfileCfgData[2]=[%d]\nProfileCfgData[3]=[%d] \n\n",
		AdvChirpLUTProfileCfgArgs.LUTAddrOff, AdvChirpLUTProfileCfgArgs.ProfileCfgData[0], AdvChirpLUTProfileCfgArgs.ProfileCfgData[1],
		AdvChirpLUTProfileCfgArgs.ProfileCfgData[2], AdvChirpLUTProfileCfgArgs.ProfileCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	memcpy(&ProfileCfgData[0], &AdvChirpLUTProfileCfgArgs.ProfileCfgData[0], 4 * sizeof(rlUInt8_t));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_PROFILE_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&ProfileCfgData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Start Frequency (Param Index = 1) */
	/* Fixed start frequency delta dither (-20000) LSB's = (-20000 * 3.6 GHz) / 2^26 = -0.0010728 GHz, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 8 chirps */
	/* Configuring 4 unique start frequency LUT dither (-0.000001,0.000000,0.000001,-0.000001) GHz from mmwaveconfig.txt */
	/* The new start frequency LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Start Freq (from Profile) + LUT dither + Fixed delta dither
		   0         77 GHz + -0.000001 GHz + 0
		   1         77 GHz + 0.000000 GHz + -0.0010728 GHz
		   2         77 GHz + 0.000001 GHz + -0.0010728*2 GHz
		   3         77 GHz + -0.000001 GHz + -0.0010728*3 GHz
		   4         77 GHz + -0.000001 GHz + -0.0010728*4 GHz  (LUT reset period = 4)
		   5         77 GHz + 0.000000 GHz + -0.0010728*5 GHz
		   6         77 GHz + 0.000001 GHz + -0.0010728*6 GHz
		   7         77 GHz + -0.000001 GHz + -0.0010728*7 GHz
		   8         77 GHz + -0.000001 GHz + 0                 (Delta dither reset period = 8) and so on */
	/* LUT start address offset for the start frequency chirp parameter is made 4 */
	/* AdvChirpLUTData[4] is the start address offset (Offset = 4), Each data parameter is 2 bytes (used lutChirpParamSize = 1 and lutChirpParamScale = 0) */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 1;
	AdvChirpCfgArgs.resetMode = 0; /* reset at the end of frame */
	AdvChirpCfgArgs.deltaResetPeriod = 8;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1; /* Update start frequency for each chirp */
	AdvChirpCfgArgs.sf0ChirpParamDelta = -20000; /* This value corresponds to (-20000 * 3.6 GHz) / 2^26 */
	AdvChirpCfgArgs.sf1ChirpParamDelta = 0; /* In legacy frame - SF1, SF2 and SF3 are not used */
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 4;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.lutChirpParamSize = 1;
	AdvChirpCfgArgs.lutChirpParamScale = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpStartFreqConfig(&AdvChirpLUTStartFreqCfgArgs);
	printf("Saving advChirpLUTStartFreqConfig with \nLUTAddrOff[%d]\nParamSize[%d]\nParamScale[%d]\nStartFreqCfgData[0]=[%.6f] GHz\nStartFreqCfgData[1]=[%.6f] GHz\nStartFreqCfgData[2]=[%.6f] GHz\nStartFreqCfgData[3]=[%.6f] GHz \n\n",
		AdvChirpLUTStartFreqCfgArgs.LUTAddrOff, AdvChirpLUTStartFreqCfgArgs.ParamSize, AdvChirpLUTStartFreqCfgArgs.ParamScale, AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[0], AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[1],
		AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[2], AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = ((3.6 * 10^9)/2^26) * 2^Scale Hz = (3.6 * 2^Scale)/2^26 GHz */
	StartFreqData[0] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[0] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));
	StartFreqData[1] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[1] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));
	StartFreqData[2] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[2] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));
	StartFreqData[3] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[3] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_FREQ_START_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&StartFreqData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Frequency Slope (Param Index = 2) */
	/* Fixed slope delta dither (-10) LSB's = (-10 * 3.6 * 900 GHz) / 2^26 = -0.482797 MHz/us, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique slope LUT dither (-0.050,0.000,-0.050,0.050) MHz/us from mmwaveconfig.txt */
	/* The new slope LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Slope (from Profile) + LUT dither + Fixed delta dither
		   0         29.982 MHz/us + -0.050 MHz/us + 0
		   1         29.982 MHz/us + 0.000 MHz/us + -0.482797 MHz/us
		   2         29.982 MHz/us + -0.050 MHz/us + -0.482797*2 MHz/us
		   3         29.982 MHz/us + 0.050 MHz/us + -0.482797*3 MHz/us
		   4         29.982 MHz/us + -0.050 MHz/us + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the slope chirp parameter is made 12 */
	/* AdvChirpLUTData[12] is the start address offset (Offset = 12), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 2;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = -10;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 12;
	AdvChirpCfgArgs.numOfPatterns = 4;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpFreqSlopeConfig(&AdvChirpLUTFreqSlopeCfgArgs);
	printf("Saving advChirpLUTFreqSlopeConfig with \nLUTAddrOff[%d]\nFreqSlopeCfgData[0]=[%.3f] MHz/us\nFreqSlopeCfgData[1]=[%.3f] MHz/us\nFreqSlopeCfgData[2]=[%.3f] MHz/us\nFreqSlopeCfgData[3]=[%.3f] MHz/us \n\n",
		AdvChirpLUTFreqSlopeCfgArgs.LUTAddrOff, AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[0], AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[1],
		AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[2], AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
    /* Dividing the input param data by 1 LSB . 1 LSB = ((3.6 * 10^9)* 900 /2^26) Hz = 48.279 KHz = 0.048279 MHz*/
	FreqSlopeData[0] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[0] * (67108864.0 / 3240000.0));
	FreqSlopeData[1] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[1] * (67108864.0 / 3240000.0));
	FreqSlopeData[2] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[2] * (67108864.0 / 3240000.0));
	FreqSlopeData[3] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[3] * (67108864.0 / 3240000.0));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_FREQ_SLOPE_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &FreqSlopeData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Idle time (Param Index = 3) */
	/* Fixed idle time delta dither (2) LSB's = (2 * 10 ns) = 0.02 us, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique idle time LUT dither (0.01,0.02,0.00,0.01) us from mmwaveconfig.txt */
	/* The new idle time LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Idle time (from Profile) + LUT dither + Fixed delta dither
		   0         100 us + 0.01 us + 0
		   1         100 us + 0.02 us + 0.02 us
		   2         100 us + 0.00 us + 0.02*2 us
		   3         100 us + 0.01 us + 0.02*3 us
		   4         100 us + 0.01 us + 0    (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the idle time chirp parameter is made 16 */
	/* AdvChirpLUTData[16] is the start address offset (Offset = 16), Each data parameter is 2 bytes (used lutChirpParamSize = 0 and lutChirpParamScale = 0) */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 3;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 2;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 16;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.lutChirpParamScale = 0;
	AdvChirpCfgArgs.lutChirpParamSize = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpIdleTimeConfig(&AdvChirpLUTIdleTimeCfgArgs);
	printf("Saving advChirpLUTIdleTimeConfig with \nLUTAddrOff[%d]\nParamSize[%d]\nParamScale[%d]\nIdleTimeCfgData[0]=[%.2f] us\nIdleTimeCfgData[1]=[%.2f] us\nIdleTimeCfgData[2]=[%.2f] us\nIdleTimeCfgData[3]=[%.2f] us \n\n",
		AdvChirpLUTIdleTimeCfgArgs.LUTAddrOff, AdvChirpLUTIdleTimeCfgArgs.ParamSize, AdvChirpLUTIdleTimeCfgArgs.ParamScale, AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[0], AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[1],
		AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[2], AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 10ns * 2^scale  = 0.01 us * 2^scale */
	IdleTimeData[0] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[0] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));
	IdleTimeData[1] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[1] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));
	IdleTimeData[2] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[2] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));
	IdleTimeData[3] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[3] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_IDLE_TIME_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&IdleTimeData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* ADC start time (Param Index = 4) */
	/* Fixed ADC start time delta dither (3) LSB's = (3 * 10 ns) = 0.03 us, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique idle time LUT dither (0.02,0.01,0.00,0.01) us from mmwaveconfig.txt */
	/* The new idle time LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    ADC start time (from Profile) + LUT dither + Fixed delta dither
		   0         6 us + 0.02 us + 0
		   1         6 us + 0.01 us + 0.03 us
		   2         6 us + 0.00 us + 0.03*2 us
		   3         6 us + 0.01 us + 0.03*3 us
		   4         6 us + 0.02 us + 0    (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the idle time chirp parameter is made 24 */
	/* AdvChirpLUTData[24] is the start address offset (Offset = 24), Each data parameter is 2 bytes (used lutChirpParamSize = 0 and lutChirpParamScale = 0) */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 4;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 3;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 24;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.lutChirpParamScale = 0;
	AdvChirpCfgArgs.lutChirpParamSize = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpADCTimeConfig(&AdvChirpLUTADCTimeCfgArgs);
	printf("Saving advChirpLUTADCTimeConfig with \nLUTAddrOff[%d]\nParamSize[%d]\nParamScale[%d]\nADCTimeCfgData[0]=[%.2f] us\nADCTimeCfgData[1]=[%.2f] us\nADCTimeCfgData[2]=[%.2f] us\nADCTimeCfgData[3]=[%.2f] us \n\n",
		AdvChirpLUTADCTimeCfgArgs.LUTAddrOff, AdvChirpLUTADCTimeCfgArgs.ParamSize, AdvChirpLUTADCTimeCfgArgs.ParamScale, AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[0], AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[1],
		AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[2], AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 10ns * 2^scale  = 0.01 us * 2^scale */
	ADCStartTimeData[0] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[0] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));
	ADCStartTimeData[1] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[1] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));
	ADCStartTimeData[2] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[2] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));
	ADCStartTimeData[3] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[3] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_ADC_START_TIME_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&ADCStartTimeData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Tx Enable (Param Index = 5) */
	/* Fixed delta dither is not supported for Tx Enable parameter */
	/* Configuring 4 unique Tx enable mask (7,3,1,2) in the generic SW LUT - LUT Reset period (4) */
	/* The new Tx enable mask is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Tx enable mask
		   0         7
		   1         3
		   2         1
		   3         2
		   4         7     and so on */
    /* LUT start address offset for the Tx enable chirp parameter is made 32 */
    /* AdvChirpLUTData[32] is the start address offset (Offset = 32), Each data parameter is 4 bits */
    /* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 5;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 32;
	AdvChirpCfgArgs.numOfPatterns = 4;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTxEnConfig(&AdvChirpLUTTxEnCfgArgs);
	printf("Saving advChirpLUTTxEnConfig with \nLUTAddrOff[%d]\nTxEnCfgData[0]=[%d]\nTxEnCfgData[1]=[%d]\nTxEnCfgData[2]=[%d]\nTxEnCfgData[3]=[%d] \n\n",
		AdvChirpLUTTxEnCfgArgs.LUTAddrOff, AdvChirpLUTTxEnCfgArgs.TxEnCfgData[0], AdvChirpLUTTxEnCfgArgs.TxEnCfgData[1],
		AdvChirpLUTTxEnCfgArgs.TxEnCfgData[2], AdvChirpLUTTxEnCfgArgs.TxEnCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	memcpy(&TxEnCfgData[0], &AdvChirpLUTTxEnCfgArgs.TxEnCfgData[0], 4 * sizeof(rlUInt8_t));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_TX_EN_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&TxEnCfgData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* BPM Enable (Param Index = 6) */
	/* Fixed delta dither is not supported for BPM Enable parameter */
	/* Configuring 4 unique BPM enable mask (7,3,1,2) in the generic SW LUT - LUT Reset period (4) */
	/* The new BPM enable mask is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    BPM enable mask
		   0         7
		   1         3
		   2         1
		   3         2
		   4         7     and so on */
	/* LUT start address offset for the BPM enable chirp parameter is made 36 */
	/* AdvChirpLUTData[36] is the start address offset (Offset = 36), Each data parameter is 4 bits */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 6;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 36;
	AdvChirpCfgArgs.numOfPatterns = 4;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpBpmEnConfig(&AdvChirpLUTBpmEnCfgArgs);
	printf("Saving advChirpLUTBpmEnConfig with \nLUTAddrOff[%d]\nBpmEnCfgData[0]=[%d]\nBpmEnCfgData[1]=[%d]\nBpmEnCfgData[2]=[%d]\nBpmEnCfgData[3]=[%d] \n\n",
		AdvChirpLUTBpmEnCfgArgs.LUTAddrOff, AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[0], AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[1],
		AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[2], AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	memcpy(&BpmEnCfgData[0], &AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[0], 4 * sizeof(rlUInt8_t));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_BPM_VAL_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, (rlInt8_t *)&BpmEnCfgData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* TX0 Phase shifter (Param Index = 7) */
	/* Fixed TX0 phase shifter delta dither (512) LSB's = (512 * 360) degrees / 2^16 = 2.8125 degrees, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique TX0 phase shifter LUT dither (5.625,11.250,16.875,16.875) degrees from mmwaveconfig.txt */
	/* The new TX0 phase shifter LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    TX0 PS (from Profile) + LUT dither + Fixed delta dither
			0         0 deg + 5.625 deg + 0
			1         0 deg + 11.250 deg + 2.8125 deg
			2         0 deg + 16.875 deg + 2.8125*2 deg
			3         0 deg + 16.875 deg + 2.8125*3 deg
			4         0 deg + 5.625 deg + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the TX0 PS chirp parameter is made 40 */
	/* AdvChirpLUTData[40] is the start address offset (Offset = 40), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 7;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 512;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 40;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.maxTxPhShiftIntDither = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTx0PhShiftConfig(&AdvChirpLUTTx0PhShiftCfgArgs);
	printf("Saving advChirpLUTTx0PhShiftConfig with \nLUTAddrOff[%d]\nTx0PhShiftCfgData[0]=[%.3f] deg\nTx0PhShiftCfgData[1]=[%.3f] deg\nTx0PhShiftCfgData[2]=[%.3f] deg\nTx0PhShiftCfgData[3]=[%.3f] deg \n\n",
		AdvChirpLUTTx0PhShiftCfgArgs.LUTAddrOff, AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[0], AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[1],
		AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[2], AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 360 / 2^6 degrees */
	Tx0PhShiftData[0] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[0] * 64) / 360);
	Tx0PhShiftData[1] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[1] * 64) / 360);
	Tx0PhShiftData[2] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[2] * 64) / 360);
	Tx0PhShiftData[3] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[3] * 64) / 360);

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX0_PHASE_SHIFT_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &Tx0PhShiftData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* TX1 Phase shifter (Param Index = 8) */
	/* Fixed TX1 phase shifter delta dither (1024) LSB's = (1024 * 360) degrees / 2^16 = 5.625 degrees, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique TX1 phase shifter LUT dither (0.000, 5.625,0.000, 5.625) degrees from mmwaveconfig.txt */
	/* The new TX1 phase shifter LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    TX1 PS (from Profile) + LUT dither + Fixed delta dither
		   0         0 deg + 0.000 deg + 0
		   1         0 deg + 5.625 deg + 5.625 deg
		   2         0 deg + 0.000 deg + 5.625*2 deg
		   3         0 deg + 5.625 deg + 5.625*3 deg
		   4         0 deg + 0.000 deg + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the TX1 PS chirp parameter is made 44 */
	/* AdvChirpLUTData[44] is the start address offset (Offset = 44), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 8;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 1024;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 44;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.maxTxPhShiftIntDither = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTx1PhShiftConfig(&AdvChirpLUTTx1PhShiftCfgArgs);
	printf("Saving advChirpLUTTx1PhShiftConfig with \nLUTAddrOff[%d]\nTx1PhShiftCfgData[0]=[%.3f] deg\nTx1PhShiftCfgData[1]=[%.3f] deg\nTx1PhShiftCfgData[2]=[%.3f] deg\nTx1PhShiftCfgData[3]=[%.3f] deg \n\n",
		AdvChirpLUTTx1PhShiftCfgArgs.LUTAddrOff, AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[0], AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[1],
		AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[2], AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 360 / 2^6 degrees */
	Tx1PhShiftData[0] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[0] * 64) / 360);
	Tx1PhShiftData[1] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[1] * 64) / 360);
	Tx1PhShiftData[2] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[2] * 64) / 360);
	Tx1PhShiftData[3] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[3] * 64) / 360);

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX1_PHASE_SHIFT_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &Tx1PhShiftData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* TX2 Phase shifter (Param Index = 9) */
	/* Fixed TX2 phase shifter delta dither (2048) LSB's = (2048 * 360) degrees / 2^16 = 11.25 degrees, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique TX2 phase shifter LUT dither (5.625,0.000, 5.625, 0.000) degrees from mmwaveconfig.txt */
	/* The new TX2 phase shifter LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    TX2 PS (from Profile) + LUT dither + Fixed delta dither
		   0         0 deg + 5.625 deg + 0
		   1         0 deg + 0.000 deg + 11.25 deg
		   2         0 deg + 5.625 deg + 11.25*2 deg
		   3         0 deg + 0.000 deg + 11.25*3 deg
		   4         0 deg + 5.625 deg + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the TX2 PS chirp parameter is made 48 */
	/* AdvChirpLUTData[48] is the start address offset (Offset = 48), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 9;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 2048;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 48;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.maxTxPhShiftIntDither = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_advChirpConfig failed with error code %d*** \n\n", retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTx2PhShiftConfig(&AdvChirpLUTTx2PhShiftCfgArgs);
	printf("Saving advChirpLUTTx2PhShiftConfig with \nLUTAddrOff[%d]\nTx2PhShiftCfgData[0]=[%.3f] deg\nTx2PhShiftCfgData[1]=[%.3f] deg\nTx2PhShiftCfgData[2]=[%.3f] deg\nTx2PhShiftCfgData[3]=[%.3f] deg \n\n",
		AdvChirpLUTTx2PhShiftCfgArgs.LUTAddrOff, AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[0], AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[1],
		AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[2], AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 360 / 2^6 degrees */
	Tx2PhShiftData[0] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[0] * 64) / 360);
	Tx2PhShiftData[1] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[1] * 64) / 360);
	Tx2PhShiftData[2] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[2] * 64) / 360);
	Tx2PhShiftData[3] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[3] * 64) / 360);

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX2_PHASE_SHIFT_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &Tx2PhShiftData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Save the entire locally programmed LUT data to a file for debug purposes
	   This locally programmed LUT data will be the same as RadarSS LUT at the device end */
	retVal = MMWL_saveAdvChirpLUTDataToFile(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("*** Failed - MMWL_saveAdvChirpLUTDataToFile failed with error code %d*** \n\n", retVal);
		return retVal;
	}
	else
	{
		printf("MMWL_saveAdvChirpLUTDataToFile success for deviceMap %u \n\n", deviceMap);
	}
    /* PING BUFFER - Starts from offset 0 and ends at 52 bytes
       PONG BUFFER - Starts from offset 100 and ends at 152 bytes */
    
    for (bufIdx = 0; bufIdx < 2; bufIdx++)
    {
        /* Send the locally programmed LUT data to the device */
        rlAdvChirpLUTCfgArgs.lutAddressOffset = 100*bufIdx;
        rlAdvChirpLUTCfgArgs.numBytes = lutOffsetInNBytes;
        
        retVal = rlSetMultiAdvChirpLUTConfig(deviceMap, &rlAdvChirpLUTCfgArgs, &AdvChirpLUTData[0]);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("rlSetMultiAdvChirpLUTConfig for deviceMap %u failed with error code %d \n\n", deviceMap, retVal);
            return retVal;
        }
        else
        {
            printf("rlSetMultiAdvChirpLUTConfig success for deviceMap %u with lutAddressOffset = %d and numBytes = %d\n\n", deviceMap, rlAdvChirpLUTCfgArgs.lutAddressOffset, rlAdvChirpLUTCfgArgs.numBytes);
        }
    }

	return retVal;
}

/** @fn int MMWL_advChirpConfig(unsigned char deviceMap)
*
*   @brief Advanced chirp configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Advanced chirp configuration API.
*/
int MMWL_advChirpConfig(unsigned char deviceMap, rlAdvChirpCfg_t* AdvChirpCfgArgs)
{
	int retVal = RL_RET_CODE_OK;

	printf("Calling rlSetAdvChirpConfig with \nchirpParamIdx[%d]\nresetMode[%d]\ndeltaResetPeriod[%d]\ndeltaParamUpdatePeriod[%d]\nsf0ChirpParamDelta[%d]\nsf1ChirpParamDelta[%d]\nsf2ChirpParamDelta[%d]\nsf3ChirpParamDelta[%d]\n",
		AdvChirpCfgArgs->chirpParamIdx, AdvChirpCfgArgs->resetMode, AdvChirpCfgArgs->deltaResetPeriod,
		AdvChirpCfgArgs->deltaParamUpdatePeriod, AdvChirpCfgArgs->sf0ChirpParamDelta, AdvChirpCfgArgs->sf1ChirpParamDelta,
		AdvChirpCfgArgs->sf2ChirpParamDelta, AdvChirpCfgArgs->sf3ChirpParamDelta);

	printf("lutResetPeriod[%d]\nlutParamUpdatePeriod[%d]\nlutPatternAddressOffset[%d]\nnumOfPatterns[%d]\nlutBurstIndexOffset[%d]\nlutSfIndexOffset[%d]\nlutChirpParamSize[%d]\nlutChirpParamScale[%d]\nmaxTxPhShiftIntDither[%d] \n\n",
		AdvChirpCfgArgs->lutResetPeriod, AdvChirpCfgArgs->lutParamUpdatePeriod, AdvChirpCfgArgs->lutPatternAddressOffset,
		AdvChirpCfgArgs->numOfPatterns, AdvChirpCfgArgs->lutBurstIndexOffset, AdvChirpCfgArgs->lutSfIndexOffset,
		AdvChirpCfgArgs->lutChirpParamSize, AdvChirpCfgArgs->lutChirpParamScale, AdvChirpCfgArgs->maxTxPhShiftIntDither);

	retVal = rlSetAdvChirpConfig(deviceMap, AdvChirpCfgArgs);
	return retVal;
}

/** @fn int MMWL_saveAdvChirpLUTDataToFile(unsigned char deviceMap)
*
*   @brief Save Advanced Chirp LUT Data to a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Save Advanced Chirp LUT Data to a file.
*/
int MMWL_saveAdvChirpLUTDataToFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int index = 0, j;
	char AdvChirpLUTDataBuff[LUT_ADVCHIRP_TABLE_SIZE*8] = { 0 };
    #ifdef _WIN32
        AdvChirpLUTDataPtr = _fsopen("AdvChirpLUTData.txt", "wt", _SH_DENYWR);
    #elif __linux__
        AdvChirpLUTDataPtr = fopen("AdvChirpLUTData.txt", "at+");
    #endif
	

	/* Store the entire LUT data in terms of 1 byte per line */
	for (j = 0; j < LUT_ADVCHIRP_TABLE_SIZE; j++)
	{
		sprintf(AdvChirpLUTDataBuff + strlen(AdvChirpLUTDataBuff), "%d\n", AdvChirpLUTData[j]);
	}

	fwrite(AdvChirpLUTDataBuff, sizeof(char), strlen(AdvChirpLUTDataBuff), AdvChirpLUTDataPtr);
	fflush(AdvChirpLUTDataPtr);

	if (AdvChirpLUTDataPtr != NULL)
		fclose(AdvChirpLUTDataPtr);

	return retVal;
}

/** @fn int MMWL_frameConfig(unsigned char deviceMap, rlUInt16_t numFrames)
*
*   @brief Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_frameConfig(unsigned char deviceMap, rlUInt16_t numFrames)
{
    int retVal = RL_RET_CODE_OK;
    rlFrameCfg_t frameCfgArgs = { 0 };

    /*read frameCfgArgs from config file*/
    MMWL_readFrameConfig(&frameCfgArgs);

    framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000);
    frameCfgArgs.numFrames = numFrames;
    frameCount = frameCfgArgs.numFrames;


	/* In Adv chirp context, frame start and frame end index is not used, the nuber of chirps is taken from number of loops */
	if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE)
	{
		frameCfgArgs.numLoops = frameCfgArgs.numLoops * (frameCfgArgs.chirpEndIdx - frameCfgArgs.chirpStartIdx + 1);
		frameCfgArgs.chirpEndIdx = 0;
		frameCfgArgs.chirpStartIdx = 0;
	}

    printf("Calling rlSetFrameConfig with \nStart Idx[%d]\nEnd Idx[%d]\nLoops[%d]\nPeriodicity[%d]ms \n\n",
        frameCfgArgs.chirpStartIdx, frameCfgArgs.chirpEndIdx,
        frameCfgArgs.numLoops, (frameCfgArgs.framePeriodicity * 5)/(1000*1000));

    retVal = rlSetFrameConfig(deviceMap, &frameCfgArgs);
    return retVal;
}

/** @fn int MMWL_advFrameConfig(unsigned char deviceMap, rlUInt16_t numFrames)
*
*   @brief Advance Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_advFrameConfig(unsigned char deviceMap, rlUInt16_t numFrames)
{
    int i, retVal = RL_RET_CODE_OK;
    rlAdvFrameCfg_t AdvframeCfgArgs = { 0 };
    rlAdvFrameCfg_t GetAdvFrameCfgArgs = { 0 };
    /* reset frame periodicity to zero */
    framePeriodicity = 0;

    /*read frameCfgArgs from config file*/
    MMWL_readAdvFrameConfig(&AdvframeCfgArgs);

    /* Add all subframes periodicity to get whole frame periodicity */
    for (i=0; i < AdvframeCfgArgs.frameSeq.numOfSubFrames; i++)
        framePeriodicity += AdvframeCfgArgs.frameSeq.subFrameCfg[i].subFramePeriodicity;

    framePeriodicity = (framePeriodicity * 5)/(1000*1000);
    /* store total number of frames configured */
    AdvframeCfgArgs.frameSeq.numFrames = numFrames;
    frameCount = AdvframeCfgArgs.frameSeq.numFrames;

    printf("Calling rlSetAdvFrameConfig with \nnumOfSubFrames[%d]\nforceProfile[%d]\nnumFrames[%d]\ntriggerSelect[%d]ms \n\n",
        AdvframeCfgArgs.frameSeq.numOfSubFrames, AdvframeCfgArgs.frameSeq.forceProfile,
        AdvframeCfgArgs.frameSeq.numFrames, AdvframeCfgArgs.frameSeq.triggerSelect);

    retVal = rlSetAdvFrameConfig(deviceMap, &AdvframeCfgArgs);
    if (retVal == 0)
    {
        retVal = rlGetAdvFrameConfig(deviceMap, &GetAdvFrameCfgArgs);
        if ((AdvframeCfgArgs.frameSeq.forceProfile != GetAdvFrameCfgArgs.frameSeq.forceProfile) || \
            (AdvframeCfgArgs.frameSeq.frameTrigDelay != GetAdvFrameCfgArgs.frameSeq.frameTrigDelay) || \
            (AdvframeCfgArgs.frameSeq.numFrames != GetAdvFrameCfgArgs.frameSeq.numFrames) || \
            (AdvframeCfgArgs.frameSeq.numOfSubFrames != GetAdvFrameCfgArgs.frameSeq.numOfSubFrames) || \
            (AdvframeCfgArgs.frameSeq.triggerSelect != GetAdvFrameCfgArgs.frameSeq.triggerSelect))
        {
            printf("MMWL_readAdvFrameConfig failed...\n\n");
            return retVal;
        }
    }
    return retVal;
}

/**
 *******************************************************************************
 *
 * \brief   Local function to enable the dummy input of objects from AWR143
 *
 * \param   None
 * /return  retVal   BSP_SOK if the test source is set correctly.
 *
 *******************************************************************************
*/
#if defined (ENABLE_TEST_SOURCE)
int MMWL_testSourceConfig(unsigned char deviceMap)
{
    rlTestSource_t tsArgs = {0};
    rlTestSourceEnable_t tsEnableArgs = {0};
    int retVal = RL_RET_CODE_OK;

    tsArgs.testObj[0].posX = 0;

    tsArgs.testObj[0].posY = 500;
    tsArgs.testObj[0].posZ = 0;

    tsArgs.testObj[0].velX = 0;
    tsArgs.testObj[0].velY = 0;
    tsArgs.testObj[0].velZ = 0;

    tsArgs.testObj[0].posXMin = -32700;
    tsArgs.testObj[0].posYMin = 0;
    tsArgs.testObj[0].posZMin = -32700;

    tsArgs.testObj[0].posXMax = 32700;
    tsArgs.testObj[0].posYMax = 32700;
    tsArgs.testObj[0].posZMax = 32700;

    tsArgs.testObj[0].sigLvl = 150;

    tsArgs.testObj[1].posX = 0;
    tsArgs.testObj[1].posY = 32700;
    tsArgs.testObj[1].posZ = 0;

    tsArgs.testObj[1].velX = 0;
    tsArgs.testObj[1].velY = 0;
    tsArgs.testObj[1].velZ = 0;

    tsArgs.testObj[1].posXMin = -32700;
    tsArgs.testObj[1].posYMin = 0;
    tsArgs.testObj[1].posZMin = -32700;

    tsArgs.testObj[1].posXMax = 32700;
    tsArgs.testObj[1].posYMax = 32700;
    tsArgs.testObj[1].posZMax = 32700;

    tsArgs.testObj[1].sigLvl = 948;

    tsArgs.rxAntPos[0].antPosX = 0;
    tsArgs.rxAntPos[0].antPosZ = 0;
    tsArgs.rxAntPos[1].antPosX = 32;
    tsArgs.rxAntPos[1].antPosZ = 0;
    tsArgs.rxAntPos[2].antPosX = 64;
    tsArgs.rxAntPos[2].antPosZ = 0;
    tsArgs.rxAntPos[3].antPosX = 96;
    tsArgs.rxAntPos[3].antPosZ = 0;

    printf("Calling rlSetTestSourceConfig with Simulated Object at X[%d]cm, Y[%d]cm, Z[%d]cm \n\n",
            tsArgs.testObj[0].posX, tsArgs.testObj[0].posY, tsArgs.testObj[0].posZ);

    retVal = rlSetTestSourceConfig(deviceMap, &tsArgs);

    tsEnableArgs.tsEnable = 1U;
    retVal = rlTestSourceEnable(deviceMap, &tsEnableArgs);

    return retVal;
}
#endif

/** @fn int MMWL_dataPathConfig(unsigned char deviceMap)
*
*   @brief Data path configuration API. Configures CQ data size on the
*           lanes and number of samples of CQ[0-2] to br transferred.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Data path configuration API. Configures CQ data size on the
*   lanes and number of samples of CQ[0-2] to br transferred.
*/
int MMWL_dataPathConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataPathCfg_t dataPathCfgArgs = { 0 };

    /* read dataPathCfgArgs from config file */
    MMWL_readDataPathConfig(&dataPathCfgArgs);

    printf("Calling rlDeviceSetDataPathConfig with HSI Interface[%d] Selected \n\n",
            dataPathCfgArgs.intfSel);

    /* same API is used to configure CQ data size on the
     * lanes and number of samples of CQ[0-2] to br transferred.
     */
    retVal = rlDeviceSetDataPathConfig(deviceMap, &dataPathCfgArgs);
    return retVal;
}

/** @fn int MMWL_lvdsLaneConfig(unsigned char deviceMap)
*
*   @brief Lane Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_lvdsLaneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevLvdsLaneCfg_t lvdsLaneCfgArgs = { 0 };

    /*read lvdsLaneCfgArgs from config file*/
    MMWL_readLvdsLaneConfig(&lvdsLaneCfgArgs);

    retVal = rlDeviceSetLvdsLaneConfig(deviceMap, &lvdsLaneCfgArgs);
    return retVal;
}

/** @fn int MMWL_laneConfig(unsigned char deviceMap)
*
*   @brief Lane Enable API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Enable API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_laneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevLaneEnable_t laneEnCfgArgs = { 0 };

    /*read laneEnCfgArgs from config file*/
    MMWL_readLaneConfig(&laneEnCfgArgs);

    retVal = rlDeviceSetLaneConfig(deviceMap, &laneEnCfgArgs);
    return retVal;
}

/** @fn int MMWL_hsiLaneConfig(unsigned char deviceMap)
*
*   @brief LVDS lane configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   LVDS lane configuration API.
*/
int MMWL_hsiLaneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /*lane configuration*/
    retVal = MMWL_laneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LaneConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LaneConfig success for deviceMap %u\n\n", deviceMap);
    }
    /*LVDS lane configuration*/
    retVal = MMWL_lvdsLaneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LvdsLaneConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LvdsLaneConfig success for deviceMap %u\n\n", deviceMap);
    }
    return retVal;
}

/** @fn int MMWL_setHsiClock(unsigned char deviceMap)
*
*   @brief High Speed Interface Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   HSI Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_setHsiClock(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevHsiClk_t hsiClkgs = { 0 };

    /*read hsiClkgs from config file*/
    MMWL_readSetHsiClock(&hsiClkgs);

    printf("Calling rlDeviceSetHsiClk with HSI Clock[%d] \n\n",
            hsiClkgs.hsiClk);

    retVal = rlDeviceSetHsiClk(deviceMap, &hsiClkgs);
    return retVal;
}

/** @fn int MMWL_hsiDataRateConfig(unsigned char deviceMap)
*
*   @brief LVDS/CSI2 Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LVDS/CSI2 Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_hsiDataRateConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataPathClkCfg_t dataPathClkCfgArgs = { 0 };

    /*read lvdsClkCfgArgs from config file*/
    MMWL_readLvdsClkConfig(&dataPathClkCfgArgs);

    printf("Calling rlDeviceSetDataPathClkConfig with HSI Data Rate[%d] Selected \n\n",
            dataPathClkCfgArgs.dataRate);

    retVal = rlDeviceSetDataPathClkConfig(deviceMap, &dataPathClkCfgArgs);
    return retVal;
}

/** @fn int MMWL_hsiClockConfig(unsigned char deviceMap)
*
*   @brief Clock configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Clock configuration API.
*/
int MMWL_hsiClockConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, readAllParams = 0;

    /*LVDS clock configuration*/
    retVal = MMWL_hsiDataRateConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LvdsClkConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_hsiDataRateConfig success for deviceMap %u\n\n", deviceMap);
    }

    /*set high speed clock configuration*/
    retVal = MMWL_setHsiClock(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setHsiClock failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setHsiClock success for deviceMap %u\n\n", deviceMap);
    }

    return retVal;
}

/** @fn int MMWL_gpadcMeasConfig(unsigned char deviceMap)
*
*   @brief API to set GPADC configuration.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code.
*
*   API to set GPADC Configuration. And device will    send GPADC
*    measurement data in form of Asynchronous event over SPI to
*    Host. User needs to feed input signal on the device pins where
*    they want to read the measurement data inside the device.
*/
int MMWL_gpadcMeasConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    int timeOutCnt = 0;
    rlGpAdcCfg_t gpadcCfg = {0};

    /* enable all the sensors [0-6] to read gpADC measurement data */
    gpadcCfg.enable = 0x3F;
    /* set the number of samples device needs to collect to do the measurement */
    gpadcCfg.numOfSamples[0].sampleCnt = 32;
    gpadcCfg.numOfSamples[1].sampleCnt = 32;
    gpadcCfg.numOfSamples[2].sampleCnt = 32;
    gpadcCfg.numOfSamples[3].sampleCnt = 32;
    gpadcCfg.numOfSamples[4].sampleCnt = 32;
    gpadcCfg.numOfSamples[5].sampleCnt = 32;
    gpadcCfg.numOfSamples[6].sampleCnt = 32;

    retVal = rlSetGpAdcConfig(deviceMap, &gpadcCfg);

    if(retVal == RL_RET_CODE_OK)
    {
        while (mmwl_bGpadcDataRcv == 0U)
        {
            #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(5);
            #else
            osiSleep(1); /*Sleep 1 msec*/
            #endif
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
            {
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
    }

    return retVal;
}

/** @fn int MMWL_sensorStart(unsigned char deviceMap)
*
*   @brief API to Start sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Start sensor.
*/
int MMWL_sensorStart(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    int timeOutCnt = 0;
	rlFrameTrigger_t data = { 0 };
	/* Start the frame */
	data.startStop = 0x1;
    mmwl_bSensorStarted = 0U;
	retVal = rlFrameStartStop(deviceMap, &data);
    while (mmwl_bSensorStarted == 0U)
    {
        #ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
        rlAppSleep(5);
        #else
        osiSleep(1); /*Sleep 1 msec*/
        #endif
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    return retVal;
}

/** @fn int MMWL_sensorStop(unsigned char deviceMap)
*
*   @brief API to Stop sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Stop Sensor.
*/
int MMWL_sensorStop(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt =0;
    if (mmwl_bSensorStarted == 1U)
    {
        rlFrameTrigger_t data = { 0 };
        /* Stop the frame after the current frame is over */
        data.startStop = 0;
        retVal = rlFrameStartStop(deviceMap, &data);
        if (retVal == RL_RET_CODE_OK)
        {
            while (mmwl_bSensorStarted == 1U)
            {
                #ifdef NON_OS_ENVIRONMENT
                rlNonOsMainLoopTask();
                rlAppSleep(8);
                #else
                osiSleep(1); /*Sleep 1 msec*/
                #endif
                timeOutCnt++;
                if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
                {
                    retVal = RL_RET_CODE_RESP_TIMEOUT;
                    break;
                }
            }
        }
    }
    return retVal;
}

/** @fn int MMWL_setContMode(unsigned char deviceMap)
*
*   @brief API to set continuous mode.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to set continuous mode.
*/
int MMWL_setContMode(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
	rlContModeCfg_t contModeCfgArgs = { 0 };
	contModeCfgArgs.digOutSampleRate = 9000;
	contModeCfgArgs.hpfCornerFreq1 = 0;
	contModeCfgArgs.hpfCornerFreq2 = 0;
	contModeCfgArgs.startFreqConst = 1435388860; /* 77GHz */
	contModeCfgArgs.txOutPowerBackoffCode = 0;
	contModeCfgArgs.txPhaseShifter = 0;

    /*read contModeCfgArgs from config file*/
    MMWL_readContModeConfig(&contModeCfgArgs);

    printf("Calling setContMode with\n digOutSampleRate[%d]\nstartFreqConst[%d]\ntxOutPowerBackoffCode[%d]\nRXGain[%d]\n\n", \
        contModeCfgArgs.digOutSampleRate, contModeCfgArgs.startFreqConst, contModeCfgArgs.txOutPowerBackoffCode, \
        contModeCfgArgs.rxGain);
    retVal = rlSetContModeConfig(deviceMap, &contModeCfgArgs);
    return retVal;
}

/** @fn int MMWL_dynChirpEnable(unsigned char deviceMap)
*
*   @brief API to enable Dynamic chirp feature.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to enable Dynamic chirp feature.
*/
int MMWL_dynChirpEnable(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDynChirpEnCfg_t dynChirpEnCfgArgs = { 0 };

    retVal = rlSetDynChirpEn(deviceMap, &dynChirpEnCfgArgs);
    return retVal;
}

/** @fn int MMWL_dynChirpConfig(unsigned char deviceMap)
*
*   @brief API to config chirp dynamically.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to config chirp dynamically.
*/
int  MMWL_setDynChirpConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    unsigned int cnt;
    rlDynChirpCfg_t * dataDynChirp[3U] = { &dynChirpCfgArgs[0], &dynChirpCfgArgs[1], &dynChirpCfgArgs[2]};

    dynChirpCfgArgs[0].programMode = 0;

    /* Configure NR1 for 48 chirps */
    dynChirpCfgArgs[0].chirpRowSelect = 0x10;
    dynChirpCfgArgs[0].chirpSegSel = 0;
    /* Copy this dynamic chirp config to other config and update chirp segment number */
    memcpy(&dynChirpCfgArgs[1], &dynChirpCfgArgs[0], sizeof(rlDynChirpCfg_t));
    memcpy(&dynChirpCfgArgs[2], &dynChirpCfgArgs[0], sizeof(rlDynChirpCfg_t));
    /* Configure NR2 for 48 chirps */
    dynChirpCfgArgs[1].chirpRowSelect = 0x20;
    dynChirpCfgArgs[1].chirpSegSel = 1;
    /* Configure NR3 for 48 chirps */
    dynChirpCfgArgs[2].chirpRowSelect = 0x30;
    dynChirpCfgArgs[2].chirpSegSel = 2;

    for (cnt = 0; cnt < 16; cnt++)
    {
        /* Reconfiguring frequency slope for 48 chirps */
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR1 |= (((3*cnt) & 0x3FU) << 8);
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR2 |= (((3*cnt + 1) & 0x3FU) << 8);
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR3 |= (((3*cnt + 2) & 0x3FU) << 8);
        /* Reconfiguring start frequency for 48 chirps */
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR1 |= 3*cnt;
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR2 |= 3*cnt + 1;
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR3 |= 3*cnt + 2;
        /* Reconfiguring ideal time for 48 chirps */
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR1 |= 3 * cnt;
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR2 |= 3 * cnt + 1;
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR3 |= 3 * cnt + 2;
    }

    printf("Calling DynChirpCfg with chirpSegSel[%d]\nchirpNR1[%d]\n\n", \
        dynChirpCfgArgs[0].chirpSegSel, dynChirpCfgArgs[0].chirpRow[0].chirpNR1);
    retVal = rlSetDynChirpCfg(deviceMap, 2U, &dataDynChirp[0]);
    return retVal;
}

/** @fn int  MMWL_setDynAdvChirpOffsetConfig(unsigned char deviceMap, rlAdvChirpDynLUTAddrOffCfg_t *data)
*
*   @brief API to config advnace chirp LUT offset dynamically.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to config advnace chirp LUT offset dynamically.
*/
int  MMWL_setDynAdvChirpOffsetConfig(unsigned char deviceMap, rlAdvChirpDynLUTAddrOffCfg_t *data)
{
    int retVal = RL_RET_CODE_OK;
    printf("Calling rlSetAdvChirpDynLUTAddrOffConfig with profile index LUT offset = %d\n\n", \
            data->lutAddressOffset[0U]);
	retVal = rlSetAdvChirpDynLUTAddrOffConfig(deviceMap, data);
    return retVal;
}

int MMWL_DynAdvConfig(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

    if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE)
	{
		/* wait for few frames to elapse before invoking Dynamic Advance chirp offset config API 
           to update new chirp config to come in effect for next frames */

		   /* wait for few frames worth of time */
        #ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
		rlAppSleep(15 * framePeriodicity);
        #else
        osiSleep(3 * framePeriodicity);
        #endif

        /* Ping LUT buffer offset (starts at 0 bytes and ends at 52 bytes) */
        advChirpDynLUTOffsetCfg1.addrMaskEn = 0x3FF; /* enable for all LUT parameters */
        advChirpDynLUTOffsetCfg1.lutAddressOffset[0U] = 0;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[1U] = 4;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[2U] = 12;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[3U] = 16;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[4U] = 24;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[5U] = 32;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[6U] = 36;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[7U] = 40;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[8U] = 44;
        advChirpDynLUTOffsetCfg1.lutAddressOffset[9U] = 48;
        /* Pong LUT buffer offset (starts at 100 bytes and ends at 152 bytes) */
        advChirpDynLUTOffsetCfg2.addrMaskEn = 0x3FF; /* enable for all LUT parameters */
        advChirpDynLUTOffsetCfg2.lutAddressOffset[0U] = 100;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[1U] = 104;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[2U] = 112;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[3U] = 116;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[4U] = 124;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[5U] = 132;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[6U] = 136;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[7U] = 140;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[8U] = 144;
        advChirpDynLUTOffsetCfg2.lutAddressOffset[9U] = 148;

        if (gDynAdvChirpLUTBufferDir == 0) /* PING Buffer */
        {
            retVal = MMWL_setDynAdvChirpOffsetConfig(deviceMap, &advChirpDynLUTOffsetCfg1);
            gDynAdvChirpLUTBufferDir = 1; /* update the buffer direction to point to the PONG buffer next time */
        }
        else /* PONG Buffer */
        {
            retVal = MMWL_setDynAdvChirpOffsetConfig(deviceMap, &advChirpDynLUTOffsetCfg2);
            gDynAdvChirpLUTBufferDir = 0; /* update the buffer direction to point to the PING buffer next time */ 
        }

		if (retVal != RL_RET_CODE_OK)
		{
			printf("Dynamic Advance Chirp offset config failed with error code %d \n\n",
				retVal);
			return -1;
		}
		else
		{
			printf("Dynamic Advance Chirp offset config successful\n\n");
		}
		printf("======================================================================\n\n");

		retVal = MMWL_dynChirpEnable(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Dynamic Chirp Enable failed with error code %d \n\n",
				retVal);
			return -1;
		}
		else
		{
			printf("Dynamic Chirp Enable successful\n\n");
		}
		printf("======================================================================\n\n");

		/* wait for another few mSec so that dynamic advance chirp offset come in effect,
		 If above API reached to BSS at the end of frame then new chirp config will come in effect
		 during next frame only */
        #ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
		rlAppSleep(10 * framePeriodicity);
        #else
        osiSleep(2 * framePeriodicity);
        #endif
	}

	return retVal;
}

int MMWL_ContModeEnable(unsigned int deviceMap, unsigned short contModeEn)
{
	int retVal = RL_RET_CODE_OK;

	rlDevContStreamingModeCfg_t contStreamingModeCfgArgs = { 0 };
	rlContModeEn_t contModeEnArgs = { 0 };

	/* MSS API */
	contStreamingModeCfgArgs.contStreamModeEn = contModeEn;
	retVal = rlDeviceSetContStreamingModeConfig(deviceMap, &contStreamingModeCfgArgs);

	/* BSS API */
	contModeEnArgs.contModeEn = contModeEn;
	retVal = rlEnableContMode(deviceMap, &contModeEnArgs);

	if (retVal != RL_RET_CODE_OK)
	{
		printf("Continuous streaming start/stop failed with error code %d \n\n",
			retVal);
		return -1;
	}
	else
	{
		printf("Continuous streaming start/stop successful \n\n");
	}

	return retVal;
}

/** @fn int MMWL_powerOff(unsigned char deviceMap)
*
*   @brief API to poweroff device.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to poweroff device.
*/
int MMWL_powerOff(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    retVal = rlDevicePowerOff();
    mmwl_bInitComp = 0;
    mmwl_bStartComp = 0U;
    mmwl_bRfInitComp = 0U;
    mmwl_devHdl = NULL;

    return retVal;
}

/** @fn int MMWL_lowPowerConfig(deviceMap)
*
*   @brief LowPower configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   LowPower configuration API.
*/
int MMWL_lowPowerConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /* TBD - Read GUI Values */
    rlLowPowerModeCfg_t rfLpModeCfgArgs = { 0 };

    /*read rfLpModeCfgArgs from config file*/
    MMWL_readLowPowerConfig(&rfLpModeCfgArgs);

    retVal = rlSetLowPowerModeConfig(deviceMap, &rfLpModeCfgArgs);
    return retVal;
}

/** @fn int MMWL_ApllSynthBwConfig(deviceMap)
*
*   @brief APLL Synth BW configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   APLL Synth BW configuration API.
*/
int MMWL_ApllSynthBwConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    rlRfApllSynthBwControl_t rfApllSynthBwCfgArgs = { 0 };
    rfApllSynthBwCfgArgs.synthIcpTrim = 3;
    rfApllSynthBwCfgArgs.synthRzTrim = 8;
    rfApllSynthBwCfgArgs.apllIcpTrim = 38;
    rfApllSynthBwCfgArgs.apllRzTrimLpf = 9;
	rfApllSynthBwCfgArgs.apllRzTrimVco = 0;
    
    retVal = rlRfApllSynthBwCtlConfig(deviceMap, &rfApllSynthBwCfgArgs);
    return retVal;
}

/** @fn int MMWL_SOPControl(unsigned char deviceMap, int SOPmode)
*
*   @brief SOP mode configuration API.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   SOP mode configuration API.
*/
int MMWL_SOPControl(unsigned char deviceMap, int SOPmode)
{
	int retVal = RL_RET_CODE_OK;
	rlsDevHandle_t rlImpl_devHdl = NULL;

	rlImpl_devHdl = rlsGetDeviceCtx(0);
	if (rlImpl_devHdl != NULL)
	{
		retVal = rlsOpenGenericGpioIf(rlImpl_devHdl);
		retVal = rlsSOPControl(rlImpl_devHdl, SOPmode);
		retVal = rlsOpenBoardControlIf(rlImpl_devHdl);
	}
	else
	{
		retVal = RL_RET_CODE_INVALID_STATE_ERROR;
	}
	return retVal;
}

/** @fn int MMWL_ResetDevice(unsigned char deviceMap)
*
*   @brief Device Reset configuration API.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Device Reset configuration API.
*/
int MMWL_ResetDevice(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	/* Reset the devices */
	rlsDevHandle_t rlImpl_devHdl = NULL;

	rlImpl_devHdl = rlsGetDeviceCtx(0);
	if (rlImpl_devHdl != NULL)
	{
		retVal = rlsFullReset(rlImpl_devHdl, 0);
		retVal = rlsFullReset(rlImpl_devHdl, 1);
		retVal = rlsCloseBoardControlIf(rlImpl_devHdl);
		retVal = rlsCloseGenericGpioIf(rlImpl_devHdl);
	}
	else
	{
		retVal = RL_RET_CODE_INVALID_STATE_ERROR;
	}
	return retVal;
}

/** @fn int MMWL_App_firmwareDownload(unsigned char deviceMap)
*
*   @brief mmWaveLink Application firmware download. 
*          You only need to download once, there's no need to do it again after power cycle.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application initialization.
*/
int MMWL_App_firmwareDownload(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlVersion_t verArgs = {0};

	retVal = MMWL_SOPControl(deviceMap, 4);/* Set SOP 4 mode for SPI */
    if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : SOP 4 mode failed with error %d\n\n", deviceMap, retVal);
		return retVal;
	}
	else
	{
		printf("Device map %u : SOP 4 mode successful\n\n", deviceMap);
	}

	retVal = MMWL_ResetDevice(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Device reset failed with error %d \n\n", deviceMap,
			retVal);
		return retVal;
	}
	else
	{
		printf("Device map %u : Device reset successful\n\n", deviceMap);
	}

    /*  \subsection     api_sequence1     Seq 1 - Call Power ON API
    The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
    initializes buffers, register interrupts, bring mmWave front end out of reset.
    */
    retVal = MMWL_powerOnMaster(deviceMap, true);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("mmWave Device Power on failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("mmWave Device Power on success for deviceMap %u \n\n",
                deviceMap);
    }

	retVal = rlDeviceGetMssVersion(deviceMap, &verArgs.master);
    retVal += rlDeviceGetMmWaveLinkVersion(&verArgs.mmWaveLink);
    printf("MSS version [%2d.%2d.%2d.%2d] \nmmWaveLink version [%2d.%2d.%2d.%2d]\n\n",
        verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
        verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
    printf("MSS Patch version [%2d.%2d.%2d.%2d]\n\n",
        verArgs.master.patchMajor, verArgs.master.patchMinor, ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F));
	/* For AWR2243 ES1.0 MSS FW version '2.2.0.3' and ES1.1: '2.2.1.7' */
	if ((verArgs.master.fwBuild == 1) && (verArgs.master.fwDebug == 7))
	{
		printf("AWR2243 ES1.1\n\n");
	}
	else
	{
		printf("AWR2243 ES1.0\n\n");
	}

    /*  \subsection     api_sequence2     Seq 2 - Download Firmware/patch
    The mmWave device firmware is ROMed and also can be stored in External serial Flash.
    */

    printf("========================== Firmware Download ==========================\n\n");
    retVal = MMWL_firmwareDownload(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Firmware update failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("Firmware update successful for deviceMap %u \n\n",
            deviceMap);
    }
    printf("=====================================================================\n\n");

    retVal = rlDeviceGetMssVersion(deviceMap, &verArgs.master);
    retVal += rlDeviceGetMmWaveLinkVersion(&verArgs.mmWaveLink);
    printf("MSS version [%2d.%2d.%2d.%2d] \nmmWaveLink version [%2d.%2d.%2d.%2d]\n\n",
        verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
        verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
    printf("MSS Patch version [%2d.%2d.%2d.%2d]\n\n",
        verArgs.master.patchMajor, verArgs.master.patchMinor, ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F));

    /* Switch off the device */
    retVal = MMWL_powerOff(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device power off failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
    }
    else
    {
        printf("Device power off success for deviceMap %u \n\n", deviceMap);
    }

    return retVal;
}

/** @fn int MMWL_App_init(unsigned char deviceMap, const char *configFilename)
*
*   @brief mmWaveLink Application init.
*
*   @param[in] deviceMap - Device Index
*   @param[in] configFilename - configuration file path
*   @param[in] downloadFw - download firmware if true
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application initialization.
*/
int MMWL_App_init(unsigned char deviceMap, const char *configFilename, bool downloadFw)
{
    int retVal = RL_RET_CODE_OK;
    int SOPmode = 0;
	rlFwVersionParam_t mssFwVer = {0};

    retVal = MMWL_openConfigFile(configFilename);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("failed to Open configuration file\n\n");
        return retVal;
    }

	/* Read all global variable configurations from config file */
	MMWL_getGlobalConfigStatus(&rlDevGlobalCfgArgs);

	if (rlDevGlobalCfgArgs.TransferMode == 0)
	{
		printf("====================== SPI Mode of Operation ======================\n\n");
		SOPmode = 4; /* Set SOP 4 mode for SPI */
	}
	else if (rlDevGlobalCfgArgs.TransferMode == 1)
	{
		printf("====================== I2C Mode of Operation ======================\n\n");
		SOPmode = 7; /* Set SOP 7 mode for I2C */
	}
	else
	{
		printf("Invalid Transport Mode detected with transportMode %d\n\n", rlDevGlobalCfgArgs.TransferMode);
		return -1;
	}

	retVal = MMWL_SOPControl(deviceMap, SOPmode);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : SOP %d mode failed with error %d\n\n", deviceMap, SOPmode, retVal);
		return retVal;
	}
	else
	{
		printf("Device map %u : SOP %d mode successful\n\n", deviceMap, SOPmode);
	}

	retVal = MMWL_ResetDevice(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Device reset failed with error %d \n\n", deviceMap,
			retVal);
		return retVal;
	}
	else
	{
		printf("Device map %u : Device reset successful\n\n", deviceMap);
	}

    /*  \subsection     api_sequence1     Seq 1 - Call Power ON API
    The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
    initializes buffers, register interrupts, bring mmWave front end out of reset.
    */
    retVal = MMWL_powerOnMaster(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("mmWave Device Power on failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("mmWave Device Power on success for deviceMap %u \n\n",
                deviceMap);
    }

	retVal = rlDeviceGetMssVersion(deviceMap, &mssFwVer);
	/* For AWR2243 ES1.0 MSS FW version '2.2.0.3' and ES1.1: '2.2.1.7' */
	if ((mssFwVer.fwBuild == 1) && (mssFwVer.fwDebug == 7))
	{
		gMmwaveSensorEs1_1 = AWR2243_ES1_1;
	}
	else
	{
		gMmwaveSensorEs1_1 = AWR2243_ES1_0;
	}

    /*  \subsection     api_sequence2     Seq 2 - Download Firmware/patch (Optional)
    The mmWave device firmware is ROMed and also can be stored in External Flash. This
    step is necessary if firmware needs to be patched and patch is not stored in serial
    Flash
    */
    if (downloadFw){
        if (rlDevGlobalCfgArgs.EnableFwDownload)
        {
            printf("========================== Firmware Download ==========================\n\n");
            retVal = MMWL_firmwareDownload(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Firmware update failed for deviceMap %u with error %d \n\n",
                    deviceMap, retVal);
                return retVal;
            }
            else
            {
                printf("Firmware update successful for deviceMap %u \n\n",
                    deviceMap);
            }
            printf("=====================================================================\n\n");
        }

        /* for AWR2243 ES1.0 sample only */
        if (gMmwaveSensorEs1_1 == AWR2243_ES1_0) 
        {
            /* Swap reset and power on the device */
            retVal = MMWL_SwapResetAndPowerOn(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Could not restart the device\n\n");
                return retVal;
            }
        }
    }
    /* Change CRC Type of Async Event generated by MSS to what is being requested by user in mmwaveconfig.txt */
    retVal = MMWL_setDeviceCrcType(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CRC Type set for MasterSS failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("CRC Type set for MasterSS success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence3     Seq 3 - Enable the mmWave Front end (Radar/RF subsystem)
    The mmWave Front end once enabled runs boot time routines and upon completion sends asynchronous event
    to notify the result
    */
    retVal = MMWL_rfEnable(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Radar/RF subsystem Power up failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("Radar/RF subsystem Power up successful for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence4     Seq 4 - Basic/Static Configuration
    The mmWave Front end needs to be configured for mmWave Radar operations. basic
    configuration includes Rx/Tx channel configuration, ADC configuration etc
    */
    printf("======================Basic/Static Configuration======================\n\n");
    retVal = MMWL_basicConfiguration(deviceMap, 0);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Basic/Static configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("Basic/Static configuration success for deviceMap %u \n\n",
                deviceMap);
    }

    /*  \subsection     api_sequence5     Seq 5 - Initializes the mmWave Front end
    The mmWave Front end once configured needs to be initialized. During initialization
    mmWave Front end performs calibration and once calibration is complete, it
    notifies the application using asynchronous event
    */
    retVal = MMWL_rfInit(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("RF Initialization/Calibration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("RF Initialization/Calibration successful for deviceMap %u \n\n", deviceMap);
    }

	if (rlDevGlobalCfgArgs.LinkContModeTest == FALSE)
	{
        /*  \subsection     api_sequence6     Seq 6 - Configures the programmable filter */
        printf("==================Programmable Filter Configuration==================\n\n");
        retVal = MMWL_progFiltConfig(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Programmable Filter Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return retVal;
        }
        else
        {
            printf("Programmable Filter Configuration success for deviceMap %u \n\n", deviceMap);
        }
        
        /*  \subsection     api_sequence7     Seq 7 - Configures the programmable filter RAM coefficients */
        retVal = MMWL_progFiltCoeffRam(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Programmable Filter coefficient RAM Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return retVal;
        }
        else
        {
            printf("Programmable Filter coefficient RAM Configuration success for deviceMap %u \n\n", deviceMap);
        }
        
        /*  \subsection     api_sequence8     Seq 8 - FMCW profile configuration
        TI mmWave devices supports Frequency Modulated Continuous Wave(FMCW) Radar. User
        Need to define characteristics of FMCW signal using profile configuration. A profile
        contains information about FMCW signal such as Start Frequency (76 - 81 GHz), Ramp
        Slope (e.g 30MHz/uS). Idle Time etc. It also configures ADC samples, Sampling rate,
        Receiver gain, Filter configuration parameters
        
        \ Note - User can define upto 4 different profiles
        */
        printf("======================FMCW Configuration======================\n\n");
        retVal = MMWL_profileConfig(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Profile Configuration failed for deviceMap %u with error code %d \n\n",
                    deviceMap, retVal);
            return retVal;
        }
        else
        {
            printf("Profile Configuration success for deviceMap %u \n\n", deviceMap);
        }
        
        if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE)
        {
            /*  \subsection     api_sequence9     Seq 9 - FMCW chirp configuration
            A chirp is always associated with FMCW profile from which it inherits coarse information
            about FMCW signal. Using chirp configuration user can further define fine
            variations to coarse parameters such as Start Frequency variation(0 - ~400 MHz), Ramp
            Slope variation (0 - ~3 MHz/uS), Idle Time variation etc. It also configures which transmit channels to be used
            for transmitting FMCW signal.
        
            \ Note - User can define upto 512 unique chirps
            */
            retVal = MMWL_chirpConfig(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Chirp Configuration failed for deviceMap %u with error %d \n\n",
                    deviceMap, retVal);
                return retVal;
            }
            else
            {
                printf("Chirp Configuration success for deviceMap %u \n\n", deviceMap);
            }
        }
        else
        {
            retVal = MMWL_advChirpConfigAll(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Advanced Chirp Configuration failed for deviceMap %u with error %d \n\n",
                    deviceMap, retVal);
                return retVal;
            }
            else
            {
                printf("Advanced Chirp Configuration success for deviceMap %u \n\n", deviceMap);
            }
        }
    }

    /*  \subsection     api_sequence10     Seq 10 - Data Path (CSI2/LVDS) Configuration
    TI mmWave device supports CSI2 or LVDS interface for sending RAW ADC data. mmWave device
    can also send Chirp Profile and Chirp Quality data over LVDS/CSI2. User need to select
    the high speed interface and also what data it expects to receive.

    \ Note - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host
    */
    printf("==================Data Path(LVDS/CSI2) Configuration==================\n\n");
    retVal = MMWL_dataPathConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data Path Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("Data Path Configuration successful for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence11     Seq 11 - CSI2/LVDS CLock and Data Rate Configuration
    User need to configure what data rate is required to send the data on high speed interface. For
    e.g 150mbps, 300mbps etc.
    \ Note - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host
    */
    retVal = MMWL_hsiClockConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CSI2/LVDS Clock Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("CSI2/LVDS Clock Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence12     Seq 12 - CSI2/LVDS Lane Configuration
    User need to configure how many LVDS/CSI2 lane needs to be enabled
    \ Note - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host
    */
    retVal = MMWL_hsiLaneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CSI2/LVDS Lane Config failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("CSI2/LVDS Lane Configuration success for deviceMap %u \n\n",
                deviceMap);
    }
    printf("======================================================================\n\n");

#ifdef ENABLE_TEST_SOURCE
    retVal = MMWL_testSourceConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Test Source Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Test source Configuration success for deviceMap %u \n\n", deviceMap);
    }
#endif
    return retVal;
}

/** @fn int MMWL_App_setFrameCfg(unsigned char deviceMap, rlUInt16_t numFrames)
*
*   @brief mmWaveLink Application set Frame Configuration.
*
*   @param[in] deviceMap - Device Index
*   @param[in] numFrames - Number of frame to transmit Valid Range 0 to 65535 (0 for infinite frames)
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application set Frame Configuration.
*/
int MMWL_App_setFrameCfg(unsigned char deviceMap, rlUInt16_t numFrames)
{
    int retVal = RL_RET_CODE_OK;

    /* Check for If Advance Frame Test is enabled */
    if(rlDevGlobalCfgArgs.LinkAdvanceFrameTest == FALSE)
    {
        /*  \subsection     api_sequence13     Seq 13 - FMCW frame configuration
        A frame defines sequence of chirps and how this sequence needs to be repeated over time.
        */
        retVal = MMWL_frameConfig(deviceMap, numFrames);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Frame Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        }
        else
        {
            printf("Frame Configuration success for deviceMap %u \n\n", deviceMap);
        }
    }
    else
    {
        /*  \subsection     api_sequence14     Seq 14 - FMCW Advance frame configuration
        A frame defines sequence of chirps and how this sequence needs to be repeated over time.
        */
        retVal = MMWL_advFrameConfig(deviceMap, numFrames);

        if (retVal != RL_RET_CODE_OK)
        {
            printf("Adv Frame Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        }
        else
        {
            printf("Adv Frame Configuration success for deviceMap %u \n\n", deviceMap);
        }
    }
    printf("======================================================================\n\n");
    return retVal;
}

/** @fn int MMWL_App_startCont(unsigned char deviceMap)
*
*   @brief This function starts the FMCW radar in continous mode.
*          In continuous mode, the signal is not frequency modulated but has the same frequency
*          over time.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   @note The continuous streaming mode configuration APIs are supported only for debug purpose.
*         Please refer latest DFP release note for more info.
*/
int MMWL_App_startCont(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    retVal = MMWL_setContMode(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Continuous mode Config failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return retVal;
    }
    else
    {
        printf("Continuous mode Config successful for deviceMap %u \n\n", deviceMap);
    }
    
    #ifdef NON_OS_ENVIRONMENT
    rlNonOsMainLoopTask();
    rlAppSleep(5000);
    #else
    osiSleep(1000);
    #endif
    
    /* Start continuous streaming for the device */
    retVal = MMWL_ContModeEnable(deviceMap, 1);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Continuous mode enable failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
    }
    else
    {
        printf("Continuous mode enable successful for deviceMap %u \n\n", deviceMap);
    }
    return retVal;
}

/** @fn int MMWL_App_stopCont(unsigned char deviceMap)
*
*   @brief This function stops the FMCW radar in continous mode.
*          In continuous mode, the signal is not frequency modulated but has the same frequency
*          over time.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   @note The continuous streaming mode configuration APIs are supported only for debug purpose.
*         Please refer latest DFP release note for more info.
*/
int MMWL_App_stopCont(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    retVal = MMWL_ContModeEnable(deviceMap, 0);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Continuous mode disable failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
    }
    else
    {
        printf("Continuous mode disable successful for deviceMap %u \n\n", deviceMap);
    }
    return retVal;
}

/** @fn int MMWL_App_startSensor(unsigned char deviceMap)
*
*   @brief mmWaveLink Application API to start radar sensor.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application API to start radar sensor.
*/
int MMWL_App_startSensor(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /*  \subsection     api_sequence15     Seq 15 - Start mmWave Radar Sensor
    This will trigger the mmWave Front to start transmitting FMCW signal. Raw ADC samples
    would be received from Digital front end. For AWR2243, if high speed interface is
    configured, RAW ADC data would be transmitted over CSI2/LVDS. On xWR1443/xWR1642, it can
    be processed using HW accelerator or DSP
    */
    retVal = MMWL_sensorStart(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Sensor Start failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
    }
    else
    {
        printf("Sensor Start successful for deviceMap %u \n\n", deviceMap);
    }
    printf("======================================================================\n\n");
    return retVal;
}

/** @fn int MMWL_App_waitSensorStop(unsigned char deviceMap)
*
*   @brief mmWaveLink Application API to Wait for the frame end async event from the device
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application API to wait radar sensor stop.
*/
int MMWL_App_waitSensorStop(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /* Wait for the frame end async event from the device */
    while (mmwl_bSensorStarted != 0x0)
    {
        #ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
        rlAppSleep(5);
        #else
        osiSleep(1);
        #endif
    }
    return retVal;
}

/** @fn int MMWL_App_isSensorStarted()
*
*   @brief mmWaveLink Application API to test if sensor is started
*
*   @return uint8_t started - 1, stopped - 0
*/
unsigned char MMWL_App_isSensorStarted()
{
    return mmwl_bSensorStarted;
}

/** @fn int MMWL_App_stopSensor(unsigned char deviceMap)
*
*   @brief mmWaveLink Application API to stop radar sensor.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application API to stop radar sensor.
*/
int MMWL_App_stopSensor(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    retVal = MMWL_sensorStop(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Sensor Stop failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
    }
    else
    {
        printf("Sensor Stop successful for deviceMap %u \n\n", deviceMap);
    }
    printf("======================================================================\n\n");
    return retVal;
}

/** @fn int MMWL_App_poweroff(unsigned char deviceMap)
*
*   @brief mmWaveLink Application API to switch off radar sensor and Close Configuraiton file.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Application API to turn radar sensor power off and close config file.
*/
int MMWL_App_poweroff(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    
    /* Switch off the device */
    retVal = MMWL_powerOff(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device power off failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
    }
    else
    {
        printf("Device power off success for deviceMap %u \n\n", deviceMap);
    }

    /* Close Configuraiton file */
    MMWL_closeConfigFile();
    return retVal;
}

/** @fn int MMWL_App()
*
*   @brief mmWaveLink Example Application.
*
*   @param[in] configFilename - configuration file path
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Example Application.
*/
int MMWL_App(const char *configFilename)
{
    int retVal = RL_RET_CODE_OK;
    unsigned char deviceMap = RL_DEVICE_MAP_CASCADED_1;
	
    retVal = MMWL_App_init(deviceMap, configFilename, true);
    if (retVal != RL_RET_CODE_OK)
        return -1;
    
    if (rlDevGlobalCfgArgs.LinkContModeTest == FALSE)
    {
        retVal = MMWL_App_setFrameCfg(deviceMap, 50);
        if (retVal != RL_RET_CODE_OK)
        return -1;
    }

    if (rlDevGlobalCfgArgs.LinkContModeTest == TRUE)
    {
        /* Start continuous streaming for the device */
        MMWL_App_startCont(deviceMap);
        
        /* Let the device stream for some more time */
        #ifdef NON_OS_ENVIRONMENT
        rlNonOsMainLoopTask();
        rlAppSleep(25000);
        #else
        osiSleep(5000);
        #endif

        /* Stop continuous streaming for the device */
        MMWL_App_stopCont(deviceMap);
    }
    else
    {
        /* Start streaming for the device */
        MMWL_App_startSensor(deviceMap);
    
        if (rlDevGlobalCfgArgs.LinkDynProfileTest == TRUE)
        {
            /* Host can update profile configurations dynamically while frame is ongoing.
            This test has been added in this example to demostrate dynamic profile update feature
            of mmWave sensor device, developer must check the validity of parameters at the system
            level before implementing the application. */
    
            /* wait for few frames worth of time before updating profile config */
            #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(30 * framePeriodicity);
            #else
            osiSleep(3*framePeriodicity);
            #endif
    
            /* update few of existing profile parameters */
            profileCfgArgs[0].rxGain = 158; /* 30 dB gain and 36 dB Gain target */
            profileCfgArgs[0].pfCalLutUpdate = 0x1; /* bit0: 1, bit1: 0 */
            profileCfgArgs[0].hpfCornerFreq1 = 1;
            profileCfgArgs[0].hpfCornerFreq2 = 1;
            profileCfgArgs[0].txStartTime = 2;
            profileCfgArgs[0].rampEndTime = 7000;
    
            /* Dynamically configure 1 profile (max 4 profiles) while frame is ongoing */
            retVal = rlSetProfileConfig(deviceMap, 1U, &profileCfgArgs[0U]);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Dynamic Profile Configuration failed for deviceMap %u with error code %d \n\n",
                        deviceMap, retVal);
                return -1;
            }
            else
            {
                printf("Dynamic Profile Configuration success for deviceMap %u \n\n", deviceMap);
            }
    
            /* wait for few frames worth of time before reading profile config.
            Dynamic profile configuration will come in effect during next frame, so wait for that time
            before reading back profile config */
		    #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(20 * framePeriodicity);
            #else
            osiSleep(2*framePeriodicity);
            #endif
    
            /* To verify that profile configuration parameters are applied to device while frame is ongoing,
            read back profile configurationn from device */
            retVal = rlGetProfileConfig(deviceMap, 0, &profileCfgArgs[1]);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Dynamic Get Profile Configuration failed for deviceMap %u with error code %d \n\n",
                        deviceMap, retVal);
                return -1;
            }
            else
            {
                printf("Dynamic Get Profile Configuration success for deviceMap %u \n\n", deviceMap);
                /* compare the read back profile configuration parameters to lastly configured parameters */
                if ((profileCfgArgs[0].rxGain != profileCfgArgs[1].rxGain) || \
                    (profileCfgArgs[0].hpfCornerFreq1 != profileCfgArgs[1].hpfCornerFreq1) || \
                    (profileCfgArgs[0].hpfCornerFreq2 != profileCfgArgs[1].hpfCornerFreq2) || \
                    (profileCfgArgs[0].txStartTime != profileCfgArgs[1].txStartTime) || \
                    (profileCfgArgs[0].rampEndTime != profileCfgArgs[1].rampEndTime))
                    printf("Dynamic Profile Config mismatched !!! \n\n");
                else
                    printf("Dynamic profile cfg matched \n\n");
            }
        }
    
        if (rlDevGlobalCfgArgs.LinkDynChirpTest == TRUE)
        {
            /* wait for few frames to elapse before invoking Dynamic chirp config API to update
            new chirp config to come in effect for next frames */
    
            /* wait for few frames worth of time */
		    #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(30 * framePeriodicity);
            #else
            osiSleep(3*framePeriodicity);
            #endif
    
            retVal = MMWL_setDynChirpConfig(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Dynamic Chirp config failed for deviceMap %u with error code %d \n\n",
                    deviceMap, retVal);
                return -1;
            }
            else
            {
                printf("Dynamic Chirp config successful for deviceMap %u \n\n", deviceMap);
            }
    
            retVal = MMWL_dynChirpEnable(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("Dynamic Chirp Enable failed for deviceMap %u with error code %d \n\n",
                    deviceMap, retVal);
                return -1;
            }
            else
            {
                printf("Dynamic Chirp Enable successful for deviceMap %u \n\n", deviceMap);
            }
            printf("======================================================================\n\n");
    
            /* wait for another few mSec so that dynamic chirp come in effect,
            If above API reached to BSS at the end of frame then new chirp config will come in effect
            during next frame only */
		    #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(50 * framePeriodicity);
            #else
            osiSleep(2*framePeriodicity);
            #endif
    
            /* read back Chirp config, which should be same as configured in dynChirpConfig for same segment */
            retVal = MMWL_getDynChirpConfig(deviceMap);
            if (retVal != RL_RET_CODE_OK)
            {
                printf("GetChirp Configuration failed for deviceMap %u with error %d \n\n",
                        deviceMap, retVal);
                return -1;
            }
            else
            {
                printf("GetChirp Configuration success for deviceMap %u \n\n", deviceMap);
            }
        }

		/* Wait for the frame end async event from the  device */
		while (mmwl_bSensorStarted != 0x0)
		{
			retVal = MMWL_DynAdvConfig(deviceMap);
			if (retVal != 0)
			{
				printf("Dynamic Advance chirp LUT offset update failed with error code %d", retVal);
				break;
			}
            #ifdef NON_OS_ENVIRONMENT
            rlNonOsMainLoopTask();
            rlAppSleep(5);
            #else
            osiSleep(1);
            #endif
		}
    }
    /* Note- Before Calling this API user must feed in input signal to device's pins,
    else device will return garbage data in GPAdc measurement over Async event.
    Measurement data is stored in 'rcvGpAdcData' structure after this API call. */
    retVal = MMWL_gpadcMeasConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("GPAdc measurement API failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("GPAdc measurement API success for deviceMap %u \n\n", deviceMap);
    }

    /* Switch off the device */
    /* Close Configuraiton file */
    MMWL_App_poweroff(deviceMap);

    return 0;
}

/** @fn int main()
*
*   @brief Main function.
*
*   @return none
*
*   Main function.
*/
// void main(void)
// {
//     int retVal;

//     printf("================= mmWaveLink Example Application ====================\n\n");
//     retVal = MMWL_App();
//     if(retVal == RL_RET_CODE_OK)
//     {
//         printf("=========== mmWaveLink Example Application execution Successful =========== \n\n");
//     }
//     else
//     {
//         printf("=========== mmWaveLink Example Application execution Failed =========== \n\n");
//     }

//     /* Wait for Enter click */
//     getchar();
//     printf("=========== mmWaveLink Example Application: Exit =========== \n\n");
// }
