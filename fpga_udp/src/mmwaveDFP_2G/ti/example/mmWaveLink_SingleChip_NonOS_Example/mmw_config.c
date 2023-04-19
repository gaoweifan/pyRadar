/****************************************************************************************
* FileName     : mmw_config.c
*
* Description  : This file reads the mmwave configuration from config file.
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

/******************************************************************************
* INCLUDE FILES
******************************************************************************
*/
#ifdef _WIN32
    #include <windows.h>
    #include <share.h>
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mmw_config.h"

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/

/* File pointer for config file*/
FILE *mmwl_configfPtr = NULL;

/******************************************************************************
* Function Definitions
*******************************************************************************
*/

/** @fn char *MMWL_trim(char * s)
*
*   @brief get rid of trailing and leading whitespace along with "\n"
*
*   @param[in] s - String pointer which needs to be trimed
*
*   @return int Success - 0, Failure - Error Code
*
*   get rid of trailing and leading whitespace along with "\n"
*/
char *MMWL_trim(char * s)
{
    /* Initialize start, end pointers */
    char *s1 = s, *s2 = &s[strlen(s) - 1];

    /* Trim and delimit right side */
    while ((isspace(*s2)) && (s2 >= s1))
        s2--;
    *(s2 + 1) = '\0';

    /* Trim left side */
    while ((isspace(*s1)) && (s1 < s2))
        s1++;

    /* Copy finished string */
    strcpy(s, s1);
    return s;
}

/** @fn void MMWL_getGlobalConfigStatus(rlDevGlobalCfg_t *rlDevGlobalCfgArgs)
*
*   @brief Read all global variable configurations from config file.
*
*   @param[in] rlDevGlobalCfg_t *rlDevGlobalCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*/
void MMWL_getGlobalConfigStatus(rlDevGlobalCfg_t *rlDevGlobalCfgArgs)
{
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	int retVal = RL_RET_CODE_OK;
	unsigned int readAllParams = 0;
	/*seek the pointer to starting of the file so that
			we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "LinkAdvanceFrameTest") == 0)
			rlDevGlobalCfgArgs->LinkAdvanceFrameTest = atoi(value);

		if (strcmp(name, "LinkContModeTest") == 0)
			rlDevGlobalCfgArgs->LinkContModeTest = atoi(value);

		if (strcmp(name, "LinkDynChirpTest") == 0)
			rlDevGlobalCfgArgs->LinkDynChirpTest = atoi(value);

		if (strcmp(name, "LinkDynProfileTest") == 0)
			rlDevGlobalCfgArgs->LinkDynProfileTest = atoi(value);

		if (strcmp(name, "LinkAdvChirpTest") == 0)
			rlDevGlobalCfgArgs->LinkAdvChirpTest = atoi(value);

		if (strcmp(name, "EnableFwDownload") == 0)
			rlDevGlobalCfgArgs->EnableFwDownload = atoi(value);

		if (strcmp(name, "EnableMmwlLogging") == 0)
			rlDevGlobalCfgArgs->EnableMmwlLogging = atoi(value);

		if (strcmp(name, "CalibEnable") == 0)
			rlDevGlobalCfgArgs->CalibEnable = atoi(value);

		if (strcmp(name, "CalibStoreRestore") == 0)
			rlDevGlobalCfgArgs->CalibStoreRestore = atoi(value);

		if (strcmp(name, "TransferMode") == 0)
			rlDevGlobalCfgArgs->TransferMode = atoi(value);

		if (strcmp(name, "IsFlashConnected") == 0)
		{
			rlDevGlobalCfgArgs->IsFlashConnected = atoi(value);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx)
*
*   @brief Read rlClientCbs_t params from config file.
*
*   @param[in] rlClientCbs_t *clientCtx
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read rlDevicePowerOn configuration params
*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "crcType") == 0)
        {
            clientCtx->crcType = (rlCrcType_t)atoi(value);
        }
        
        if (strcmp(name, "ackTimeout") == 0)
        {
            clientCtx->ackTimeout = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
*                            unsigned short cascade)
*
*   @brief Read rlChanCfg_t params from config file.
*
*   @param[in] rlChanCfg_t *rfChanCfgArgs
*    @param[in] unsigned short cascade
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read channel configuration params
*/
void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
                            unsigned short cascade)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "channelTx") == 0)
            rfChanCfgArgs->txChannelEn = atoi(value);

        if (strcmp(name, "channelRx") == 0)
            rfChanCfgArgs->rxChannelEn = atoi(value);

        if (strcmp(name, "cascading") == 0)
        {
            rfChanCfgArgs->cascading = atoi(value);
            readAllParams = 1;
        }
    }
    rfChanCfgArgs->cascading = cascade;
}

/** @fn void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs)
*
*   @brief Read rlAdcOutCfg_t params from config file.
*
*   @param[in] rlAdcOutCfg_t *adcOutCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read ADC configuration params
*/
void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "adcBits") == 0)
            adcOutCfgArgs->fmt.b2AdcBits = atoi(value);

        if (strcmp(name, "adcFormat") == 0)
        {
            adcOutCfgArgs->fmt.b2AdcOutFmt = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs)
*
*   @brief Read rlDevDataFmtCfg_t params from config file.
*
*   @param[in] rlDevDataFmtCfg_t *dataFmtCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data format configuration params
*/
void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "rxChanEn") == 0)
            dataFmtCfgArgs->rxChannelEn = atoi(value);

        if (strcmp(name, "adcBitsD") == 0)
            dataFmtCfgArgs->adcBits = atoi(value);

        if (strcmp(name, "adcFmt") == 0)
            dataFmtCfgArgs->adcFmt = atoi(value);

        if (strcmp(name, "iqSwapSel") == 0)
            dataFmtCfgArgs->iqSwapSel = atoi(value);

        if (strcmp(name, "chInterleave") == 0)
        {
            dataFmtCfgArgs->chInterleave = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs)
*
*   @brief Read rlLowPowerModeCfg_t params from config file.
*
*   @param[in] rlLowPowerModeCfg_t *rfLpModeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read low power configuration params
*/
void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "lpAdcMode") == 0)
        {
            rfLpModeCfgArgs->lpAdcMode = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs)
*
*   @brief Read rlDevDataPathCfg_t params from config file.
*
*   @param[in] rlDevDataPathCfg_t *dataPathCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data path configuration params
*/
void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "intfSel") == 0)
            dataPathCfgArgs->intfSel = atoi(value);

        if (strcmp(name, "transferFmtPkt0") == 0)
            dataPathCfgArgs->transferFmtPkt0 = atoi(value);

        if (strcmp(name, "transferFmtPkt1") == 0)
            dataPathCfgArgs->transferFmtPkt1 = atoi(value);

        if (strcmp(name, "cqConfig") == 0)
            dataPathCfgArgs->cqConfig = atoi(value);
        
        if (strcmp(name, "cq0TransSize") == 0)
            dataPathCfgArgs->cq0TransSize = atoi(value);
        
        if (strcmp(name, "cq1TransSize") == 0)
            dataPathCfgArgs->cq1TransSize = atoi(value);
        
        if (strcmp(name, "cq2TransSize") == 0)
            dataPathCfgArgs->cq2TransSize = atoi(value);
    }
}

/** @fn void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs)
*
*   @brief Read rlDevDataPathClkCfg_t params from config file.
*
*   @param[in] rlDevDataPathClkCfg_t *lvdsClkCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS clock configuration params
*/
void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneClk") == 0)
            lvdsClkCfgArgs->laneClkCfg = atoi(value);

        if (strcmp(name, "dataRate") == 0)
        {
            lvdsClkCfgArgs->dataRate = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs)
*
*   @brief Read rlDevHsiClk_t params from config file.
*
*   @param[in] rlDevHsiClk_t *hsiClkgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data path clock configuration params
*/
void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "hsiClk") == 0)
        {
            hsiClkgs->hsiClk = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs)
*
*   @brief Read rlDevLaneEnable_t params from config file.
*
*   @param[in] rlDevLaneEnable_t *laneEnCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS/CSI2 lane configuration params
*/
void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneEn") == 0)
        {
            laneEnCfgArgs->laneEn = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs)
*
*   @brief Read rlDevLvdsLaneCfg_t params from config file.
*
*   @param[in] rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS specific configuration params
*/
void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneFmtMap") == 0)
            lvdsLaneCfgArgs->laneFmtMap = atoi(value);

        if (strcmp(name, "laneParamCfg") == 0)
        {
            lvdsLaneCfgArgs->laneParamCfg = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs, int profileCfgCnt)
*
*   @brief Read rlProfileCfg_t params from config file.
*
*   @param[in] rlProfileCfg_t *profileCfgArgs
*   @param[in] int profileCfgCnt
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read profile configuration
*/
void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs, int profileCfgCnt)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "profileId") == 0)
            profileCfgArgs->profileId = atoi(value);

        if (strcmp(name, "pfVcoSelect") == 0)
            profileCfgArgs->pfVcoSelect = atoi(value);

        if (strcmp(name, "startFreqConst") == 0)
            profileCfgArgs->startFreqConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "idleTimeConst") == 0)
            profileCfgArgs->idleTimeConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "adcStartTimeConst") == 0)
            profileCfgArgs->adcStartTimeConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "rampEndTime") == 0)
            profileCfgArgs->rampEndTime = strtoul(value, &ptr, 10);

        if (strcmp(name, "txOutPowerBackoffCode") == 0)
            profileCfgArgs->txOutPowerBackoffCode = strtoul(value, &ptr, 10);

        if (strcmp(name, "txPhaseShifter") == 0)
            profileCfgArgs->txPhaseShifter = strtoul(value, &ptr, 10);

        if (strcmp(name, "freqSlopeConst") == 0)
            profileCfgArgs->freqSlopeConst = atoi(value);

        if (strcmp(name, "txStartTime") == 0)
            profileCfgArgs->txStartTime = atoi(value);

        if (strcmp(name, "numAdcSamples") == 0)
            profileCfgArgs->numAdcSamples = atoi(value);

        if (strcmp(name, "digOutSampleRate") == 0)
            profileCfgArgs->digOutSampleRate = atoi(value);

        if (strcmp(name, "hpfCornerFreq1") == 0)
            profileCfgArgs->hpfCornerFreq1 = atoi(value);

        if (strcmp(name, "hpfCornerFreq2") == 0)
            profileCfgArgs->hpfCornerFreq2 = atoi(value);

        if (strcmp(name, "rxGain") == 0)
        {
            profileCfgArgs->rxGain = atoi(value);
			profileCfgCnt--;
			if (profileCfgCnt == 0)
			{
				readAllParams = 1;
			}
			else
			{
				// Jump to next profileCfgArgs pointer
				profileCfgArgs++;
			}
        }
    }
}

/** @fn void MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs, int chirpCfgCnt)
*
*   @brief Read rlChirpCfg_t params from config file.
*

*   @param[in] rlChirpCfg_t *chirpCfgArgs
*   @param[in] int chirpCfgCnt
*
*   @return int Chirp Config count
*
*   API to read chirp configuration params
*/

int MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs, int chirpCfgCnt)
{
    int readAllParams = 0,cnt=0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpStartIdx") == 0)
            chirpCfgArgs->chirpStartIdx = atoi(value);

        if (strcmp(name, "chirpEndIdx") == 0)
            chirpCfgArgs->chirpEndIdx = atoi(value);

        if (strcmp(name, "profileIdCPCFG") == 0)
            chirpCfgArgs->profileId = atoi(value);

        if (strcmp(name, "startFreqVar") == 0)
            chirpCfgArgs->startFreqVar = strtoul(value, &ptr, 10);

        if (strcmp(name, "freqSlopeVar") == 0)
            chirpCfgArgs->freqSlopeVar = atoi(value);

        if (strcmp(name, "idleTimeVar") == 0)
            chirpCfgArgs->idleTimeVar = atoi(value);

        if (strcmp(name, "adcStartTimeVar") == 0)
            chirpCfgArgs->adcStartTimeVar = atoi(value);

        if (strcmp(name, "txEnable") == 0)
        {
            chirpCfgArgs->txEnable = atoi(value);
			chirpCfgCnt--;
			if (chirpCfgCnt == 0)
			{
				readAllParams = 1;
			}
			else
			{
				// Jump to next chirpCfgArgs pointer
				chirpCfgArgs++;
                cnt++;
			}
        }
    }
    return cnt;
}

/** @fn void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs)
*
*   @brief Read rlFrameCfg_t params from config file.
*
*   @param[in] rlFrameCfg_t *frameCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read frame configuration params
*/
void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpStartIdxFCF") == 0)
            frameCfgArgs->chirpStartIdx = atoi(value);

        if (strcmp(name, "chirpEndIdxFCF") == 0)
            frameCfgArgs->chirpEndIdx = atoi(value);

        if (strcmp(name, "frameCount") == 0)
            frameCfgArgs->numFrames = atoi(value);

        if (strcmp(name, "loopCount") == 0)
            frameCfgArgs->numLoops = atoi(value);

        if (strcmp(name, "periodicity") == 0)
            frameCfgArgs->framePeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "triggerDelay") == 0)
            frameCfgArgs->frameTriggerDelay = strtoul(value, &ptr, 10);

        if (strcmp(name, "numAdcSamples") == 0)
            frameCfgArgs->numAdcSamples = atoi(value);

        if (strcmp(name, "triggerSelect") == 0)
        {
            frameCfgArgs->triggerSelect = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs)
*
*   @brief Read rlAdvFrameCfg_t params from config file.
*
*   @param[in] rlAdvFrameCfg_t *rlAdvFrameCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read frame configuration params
*/
void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    unsigned char subFrameCfgCnt = 0, numsubframe = 0, advframe_flag = 0;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "numOfSubFrames") == 0){
            rlAdvFrameCfgArgs->frameSeq.numOfSubFrames = atoi(value);
            subFrameCfgCnt = rlAdvFrameCfgArgs->frameSeq.numOfSubFrames;

            rlAdvFrameCfgArgs->frameData.numSubFrames = atoi(value);
            numsubframe = rlAdvFrameCfgArgs->frameData.numSubFrames;

            advframe_flag = 1;
        }

        if (strcmp(name, "forceProfile") == 0)
            rlAdvFrameCfgArgs->frameSeq.forceProfile = atoi(value);

        if (strcmp(name, "numFrames") == 0)
            rlAdvFrameCfgArgs->frameSeq.numFrames = atoi(value);

        if (strcmp(name, "loopBackCfg") == 0)
            rlAdvFrameCfgArgs->frameSeq.loopBackCfg = atoi(value);

        if (strcmp(name, "triggerSelect") == 0)
            rlAdvFrameCfgArgs->frameSeq.triggerSelect = atoi(value);

        if (strcmp(name, "frameTrigDelay") == 0)
            rlAdvFrameCfgArgs->frameSeq.frameTrigDelay = strtoul(value, &ptr, 10);

        if (strcmp(name, "forceProfileIdx") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[--subFrameCfgCnt].forceProfileIdx = atoi(value);

        if (strcmp(name, "chirpStartIdxAF") == 0)
            if (advframe_flag == 1){
                advframe_flag = 0;
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].chirpStartIdxOffset = atoi(value);
            }

        if (strcmp(name, "numOfChirps") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfChirps = atoi(value);

        if (strcmp(name, "numLoops") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numLoops = atoi(value);

        if (strcmp(name, "burstPeriodicity") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].burstPeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "chirpStartIdxOffset") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].chirpStartIdxOffset = atoi(value);

        if (strcmp(name, "numOfBurst") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurst = atoi(value);

        if (strcmp(name, "numOfBurstLoops") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurstLoops = atoi(value);

        if (strcmp(name, "subFramePeriodicity") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].subFramePeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "numAdcSamplesAF") == 0)
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[--numsubframe].numAdcSamples = atoi(value);

        if (strcmp(name, "numChirpsInDataPacket") == 0)
        {
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[numsubframe].numChirpsInDataPacket = atoi(value);

            /* Total number of chirps in one subframe */
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[numsubframe].totalChirps =
                (rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfChirps *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numLoops *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurst *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurstLoops);

            if (numsubframe == 0)
                readAllParams = 1;
        }

    }
}

/** @fn void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs)
*
*   @brief Read rlContModeCfg_t params from config file.
*
*   @param[in] rlContModeCfg_t * rlContModeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read continuous mode configuration params
*/
void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "vcoSelect") == 0)
            rlContModeCfgArgs->vcoSelect= atoi(value);

        if (strcmp(name, "contModeRxGain") == 0)
            rlContModeCfgArgs->rxGain = atoi(value);
    }

}

/** @fn void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs)
*
*   @brief Read rlDynChirpCfg_t params from config file.
*
*   @param[in] rlDynChirpCfg_t* rldynChirpCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read dynamic chirp configuration params
*/
void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs)
{
    int readAllParams = 0;
    int chirpRowCnt = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpRowSel") == 0)
            rldynChirpCfgArgs->chirpRowSelect = atoi(value);

        if (strcmp(name, "chirpSegSel") == 0)
            rldynChirpCfgArgs->chirpSegSel = atoi(value);

        if (strcmp(name, "chirpNR1") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR1 = atoi(value);
        }
        if (strcmp(name, "chirpNR2") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR2 = atoi(value);
        }
        if (strcmp(name, "chirpNR3") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR3 = atoi(value);
        }
    }

}

/** @fn void MMWL_readProgFiltConfig(rlRfProgFiltConf_t* rlProgFiltCnfgArgs)
*
*   @brief Read rlRfProgFiltConf_t params from config file.
*
*   @param[in] rlRfProgFiltConf_t* rlProgFiltCnfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read programmabe filter configuration params
*/
void MMWL_readProgFiltConfig(rlRfProgFiltConf_t* rlProgFiltCnfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "profileId") == 0)
            rlProgFiltCnfgArgs->profileId = atoi(value);

        if (strcmp(name, "coeffStartIdx") == 0)
            rlProgFiltCnfgArgs->coeffStartIdx = atoi(value);

        if (strcmp(name, "progFiltLen") == 0)
            rlProgFiltCnfgArgs->progFiltLen = atoi(value);

        if (strcmp(name, "progFiltFreqShift") == 0)
        {
            rlProgFiltCnfgArgs->progFiltFreqShift = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readAdvChirpConfig(rlAdvChirpCfg_t *AdvChirpCfgArgs)
*
*   @brief Read Advanced chirp config params from config file
*
*   @param[in] rlAdvChirpCfg_t *AdvChirpCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp config params from config file
*/
void MMWL_readAdvChirpConfig(rlAdvChirpCfg_t *AdvChirpCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirp_chirpParamIdx") == 0)
			AdvChirpCfgArgs->chirpParamIdx = atoi(value);

		if (strcmp(name, "AdvChirp_resetMode") == 0)
			AdvChirpCfgArgs->resetMode = atoi(value);

		if (strcmp(name, "AdvChirp_deltaResetPeriod") == 0)
			AdvChirpCfgArgs->deltaResetPeriod = atoi(value);

		if (strcmp(name, "AdvChirp_deltaParamUpdatePeriod") == 0)
			AdvChirpCfgArgs->deltaParamUpdatePeriod = atoi(value);

		if (strcmp(name, "AdvChirp_sf0ChirpParamDelta") == 0)
			AdvChirpCfgArgs->sf0ChirpParamDelta = atoi(value);

		if (strcmp(name, "AdvChirp_sf1ChirpParamDelta") == 0)
			AdvChirpCfgArgs->sf1ChirpParamDelta = atoi(value);

		if (strcmp(name, "AdvChirp_sf2ChirpParamDelta") == 0)
			AdvChirpCfgArgs->sf2ChirpParamDelta = atoi(value);

		if (strcmp(name, "AdvChirp_sf3ChirpParamDelta") == 0)
			AdvChirpCfgArgs->sf3ChirpParamDelta = atoi(value);

		if (strcmp(name, "AdvChirp_lutResetPeriod") == 0)
			AdvChirpCfgArgs->lutResetPeriod = atoi(value);

		if (strcmp(name, "AdvChirp_lutParamUpdatePeriod") == 0)
			AdvChirpCfgArgs->lutParamUpdatePeriod = atoi(value);

		if (strcmp(name, "AdvChirp_lutPatternAddressOffset") == 0)
			AdvChirpCfgArgs->lutPatternAddressOffset = atoi(value);

		if (strcmp(name, "AdvChirp_numPatterns") == 0)
			AdvChirpCfgArgs->numOfPatterns = atoi(value);

		if (strcmp(name, "AdvChirp_lutBurstIndexOffset") == 0)
			AdvChirpCfgArgs->lutBurstIndexOffset = atoi(value);

		if (strcmp(name, "AdvChirp_lutSfIndexOffset") == 0)
			AdvChirpCfgArgs->lutSfIndexOffset = atoi(value);

		if (strcmp(name, "AdvChirp_lutChirpParamSize") == 0)
			AdvChirpCfgArgs->lutChirpParamSize = atoi(value);

		if (strcmp(name, "AdvChirp_lutChirpParamScale") == 0)
			AdvChirpCfgArgs->lutChirpParamScale = atoi(value);

		if (strcmp(name, "AdvChirp_maxTxPhShifIntDither") == 0)
		{
			AdvChirpCfgArgs->maxTxPhShiftIntDither = atoi(value);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpProfileConfig(rlAdvChirpLUTProfileCfg_t* rlAdvChirpLUTProfileCfgArgs)
*
*   @brief Read Advanced chirp Profile config params from config file
*
*   @param[in] rlAdvChirpLUTProfileCfg_t* rlAdvChirpLUTProfileCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp Profile config params from config file
*/
void MMWL_readAdvChirpProfileConfig(rlAdvChirpLUTProfileCfg_t* rlAdvChirpLUTProfileCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_ProfileConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTProfileCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ProfileConfig_Data1") == 0)
			rlAdvChirpLUTProfileCfgArgs->ProfileCfgData[0] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ProfileConfig_Data2") == 0)
			rlAdvChirpLUTProfileCfgArgs->ProfileCfgData[1] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ProfileConfig_Data3") == 0)
			rlAdvChirpLUTProfileCfgArgs->ProfileCfgData[2] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ProfileConfig_Data4") == 0)
		{
			rlAdvChirpLUTProfileCfgArgs->ProfileCfgData[3] = atoi(value);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpStartFreqConfig(rlAdvChirpLUTStartFreqCfg_t* rlAdvChirpLUTStartFreqCfgArgs)
*
*   @brief Read Advanced chirp Start Freq config params from config file
*
*   @param[in] rlAdvChirpLUTStartFreqCfg_t* rlAdvChirpLUTStartFreqCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp Start Freq config params from config file
*/
void MMWL_readAdvChirpStartFreqConfig(rlAdvChirpLUTStartFreqCfg_t* rlAdvChirpLUTStartFreqCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTStartFreqCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_ParamSize") == 0)
			rlAdvChirpLUTStartFreqCfgArgs->ParamSize = atoi(value);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_ParamScale") == 0)
			rlAdvChirpLUTStartFreqCfgArgs->ParamScale = atoi(value);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_Data1") == 0)
			rlAdvChirpLUTStartFreqCfgArgs->StartFreqCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_Data2") == 0)
			rlAdvChirpLUTStartFreqCfgArgs->StartFreqCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_Data3") == 0)
			rlAdvChirpLUTStartFreqCfgArgs->StartFreqCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_StartFreqConfig_Data4") == 0)
		{
			rlAdvChirpLUTStartFreqCfgArgs->StartFreqCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpFreqSlopeConfig(rlAdvChirpLUTFreqSlopeCfg_t* rlAdvChirpLUTFreqSlopeCfgArgs)
*
*   @brief Read Advanced chirp Freq Slope config params from config file
*
*   @param[in] rlAdvChirpLUTFreqSlopeCfg_t* rlAdvChirpLUTFreqSlopeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp Freq Slope config params from config file
*/
void MMWL_readAdvChirpFreqSlopeConfig(rlAdvChirpLUTFreqSlopeCfg_t* rlAdvChirpLUTFreqSlopeCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_FreqSlopeConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTFreqSlopeCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_FreqSlopeConfig_Data1") == 0)
			rlAdvChirpLUTFreqSlopeCfgArgs->FreqSlopeCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_FreqSlopeConfig_Data2") == 0)
			rlAdvChirpLUTFreqSlopeCfgArgs->FreqSlopeCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_FreqSlopeConfig_Data3") == 0)
			rlAdvChirpLUTFreqSlopeCfgArgs->FreqSlopeCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_FreqSlopeConfig_Data4") == 0)
		{
			rlAdvChirpLUTFreqSlopeCfgArgs->FreqSlopeCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpIdleTimeConfig(rlAdvChirpLUTIdleTimeCfg_t* rlAdvChirpLUTIdleTimeCfgArgs)
*
*   @brief Read Advanced chirp Idle time config params from config file
*
*   @param[in] rlAdvChirpLUTIdleTimeCfg_t* rlAdvChirpLUTIdleTimeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp Idle time config params from config file
*/
void MMWL_readAdvChirpIdleTimeConfig(rlAdvChirpLUTIdleTimeCfg_t* rlAdvChirpLUTIdleTimeCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTIdleTimeCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_ParamSize") == 0)
			rlAdvChirpLUTIdleTimeCfgArgs->ParamSize = atoi(value);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_ParamScale") == 0)
			rlAdvChirpLUTIdleTimeCfgArgs->ParamScale = atoi(value);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_Data1") == 0)
			rlAdvChirpLUTIdleTimeCfgArgs->IdleTimeCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_Data2") == 0)
			rlAdvChirpLUTIdleTimeCfgArgs->IdleTimeCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_Data3") == 0)
			rlAdvChirpLUTIdleTimeCfgArgs->IdleTimeCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_IdleTimeConfig_Data4") == 0)
		{
			rlAdvChirpLUTIdleTimeCfgArgs->IdleTimeCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpADCTimeConfig(rlAdvChirpLUTADCTimeCfg_t* rlAdvChirpLUTADCTimeCfgArgs)
*
*   @brief Read Advanced chirp ADC time config params from config file
*
*   @param[in] rlAdvChirpLUTADCTimeCfg_t* rlAdvChirpLUTADCTimeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp ADC time config params from config file
*/
void MMWL_readAdvChirpADCTimeConfig(rlAdvChirpLUTADCTimeCfg_t* rlAdvChirpLUTADCTimeCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTADCTimeCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_ParamSize") == 0)
			rlAdvChirpLUTADCTimeCfgArgs->ParamSize = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_ParamScale") == 0)
			rlAdvChirpLUTADCTimeCfgArgs->ParamScale = atoi(value);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_Data1") == 0)
			rlAdvChirpLUTADCTimeCfgArgs->ADCTimeCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_Data2") == 0)
			rlAdvChirpLUTADCTimeCfgArgs->ADCTimeCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_Data3") == 0)
			rlAdvChirpLUTADCTimeCfgArgs->ADCTimeCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_ADCTimeConfig_Data4") == 0)
		{
			rlAdvChirpLUTADCTimeCfgArgs->ADCTimeCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpTxEnConfig(rlAdvChirpLUTTxEnCfg_t* rlAdvChirpLUTTxEnCfgArgs)
*
*   @brief Read Advanced chirp Tx Enable config params from config file
*
*   @param[in] rlAdvChirpLUTTxEnCfg_t* rlAdvChirpLUTTxEnCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp Tx Enable config params from config file
*/
void MMWL_readAdvChirpTxEnConfig(rlAdvChirpLUTTxEnCfg_t* rlAdvChirpLUTTxEnCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_TxEnConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTTxEnCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_TxEnConfig_Data1") == 0)
			rlAdvChirpLUTTxEnCfgArgs->TxEnCfgData[0] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_TxEnConfig_Data2") == 0)
			rlAdvChirpLUTTxEnCfgArgs->TxEnCfgData[1] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_TxEnConfig_Data3") == 0)
			rlAdvChirpLUTTxEnCfgArgs->TxEnCfgData[2] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_TxEnConfig_Data4") == 0)
		{
			rlAdvChirpLUTTxEnCfgArgs->TxEnCfgData[3] = atoi(value);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpBpmEnConfig(rlAdvChirpLUTBpmEnCfg_t* rlAdvChirpLUTBpmEnCfgArgs)
*
*   @brief Read Advanced chirp BPM Enable config params from config file
*
*   @param[in] rlAdvChirpLUTBpmEnCfg_t* rlAdvChirpLUTBpmEnCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp BPM Enable config params from config file
*/
void MMWL_readAdvChirpBpmEnConfig(rlAdvChirpLUTBpmEnCfg_t* rlAdvChirpLUTBpmEnCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_BpmEnConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTBpmEnCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_BpmEnConfig_Data1") == 0)
			rlAdvChirpLUTBpmEnCfgArgs->BpmEnCfgData[0] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_BpmEnConfig_Data2") == 0)
			rlAdvChirpLUTBpmEnCfgArgs->BpmEnCfgData[1] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_BpmEnConfig_Data3") == 0)
			rlAdvChirpLUTBpmEnCfgArgs->BpmEnCfgData[2] = atoi(value);

		if (strcmp(name, "AdvChirpLUT_BpmEnConfig_Data4") == 0)
		{
			rlAdvChirpLUTBpmEnCfgArgs->BpmEnCfgData[3] = atoi(value);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpTx0PhShiftConfig(rlAdvChirpLUTTx0PhShiftCfg_t* rlAdvChirpLUTTx0PhShiftCfgArgs)
*
*   @brief Read Advanced chirp TX0 Phase shift config params from config file
*
*   @param[in] rlAdvChirpLUTTx0PhShiftCfg_t* rlAdvChirpLUTTx0PhShiftCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp TX0 Phase shift config params from config file
*/
void MMWL_readAdvChirpTx0PhShiftConfig(rlAdvChirpLUTTx0PhShiftCfg_t* rlAdvChirpLUTTx0PhShiftCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_Tx0PhShiftConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTTx0PhShiftCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_Tx0PhShiftConfig_Data1") == 0)
			rlAdvChirpLUTTx0PhShiftCfgArgs->Tx0PhShiftCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx0PhShiftConfig_Data2") == 0)
			rlAdvChirpLUTTx0PhShiftCfgArgs->Tx0PhShiftCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx0PhShiftConfig_Data3") == 0)
			rlAdvChirpLUTTx0PhShiftCfgArgs->Tx0PhShiftCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx0PhShiftConfig_Data4") == 0)
		{
			rlAdvChirpLUTTx0PhShiftCfgArgs->Tx0PhShiftCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpTx1PhShiftConfig(rlAdvChirpLUTTx1PhShiftCfg_t* rlAdvChirpLUTTx1PhShiftCfgArgs)
*
*   @brief Read Advanced chirp TX1 Phase shift config params from config file
*
*   @param[in] rlAdvChirpLUTTx1PhShiftCfg_t* rlAdvChirpLUTTx1PhShiftCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp TX1 Phase shift config params from config file
*/
void MMWL_readAdvChirpTx1PhShiftConfig(rlAdvChirpLUTTx1PhShiftCfg_t* rlAdvChirpLUTTx1PhShiftCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_Tx1PhShiftConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTTx1PhShiftCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_Tx1PhShiftConfig_Data1") == 0)
			rlAdvChirpLUTTx1PhShiftCfgArgs->Tx1PhShiftCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx1PhShiftConfig_Data2") == 0)
			rlAdvChirpLUTTx1PhShiftCfgArgs->Tx1PhShiftCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx1PhShiftConfig_Data3") == 0)
			rlAdvChirpLUTTx1PhShiftCfgArgs->Tx1PhShiftCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx1PhShiftConfig_Data4") == 0)
		{
			rlAdvChirpLUTTx1PhShiftCfgArgs->Tx1PhShiftCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readAdvChirpTx2PhShiftConfig(rlAdvChirpLUTTx2PhShiftCfg_t* rlAdvChirpLUTTx2PhShiftCfgArgs)
*
*   @brief Read Advanced chirp TX2 Phase shift config params from config file
*
*   @param[in] rlAdvChirpLUTTx2PhShiftCfg_t* rlAdvChirpLUTTx2PhShiftCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Read Advanced chirp TX2 Phase shift config params from config file
*/
void MMWL_readAdvChirpTx2PhShiftConfig(rlAdvChirpLUTTx2PhShiftCfg_t* rlAdvChirpLUTTx2PhShiftCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AdvChirpLUT_Tx2PhShiftConfig_LUTAddrOff") == 0)
			rlAdvChirpLUTTx2PhShiftCfgArgs->LUTAddrOff = atoi(value);

		if (strcmp(name, "AdvChirpLUT_Tx2PhShiftConfig_Data1") == 0)
			rlAdvChirpLUTTx2PhShiftCfgArgs->Tx2PhShiftCfgData[0] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx2PhShiftConfig_Data2") == 0)
			rlAdvChirpLUTTx2PhShiftCfgArgs->Tx2PhShiftCfgData[1] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx2PhShiftConfig_Data3") == 0)
			rlAdvChirpLUTTx2PhShiftCfgArgs->Tx2PhShiftCfgData[2] = strtod(value, NULL);

		if (strcmp(name, "AdvChirpLUT_Tx2PhShiftConfig_Data4") == 0)
		{
			rlAdvChirpLUTTx2PhShiftCfgArgs->Tx2PhShiftCfgData[3] = strtod(value, NULL);
			readAllParams = 1;
		}
	}
}

/** @fn int MMWL_openConfigFile()
*
*   @brief Opens MMWave config file
*
*   @return int Success - 0, Failure - Error Code
*
*   Opens MMWave config file
*/
int MMWL_openConfigFile(const char *filename)
{
    /*open config file to read parameters*/
    if (mmwl_configfPtr == NULL)
    {
        mmwl_configfPtr = fopen(filename, "r");
        if (mmwl_configfPtr == NULL)
        {
            printf("failed to open config file\n");
            return -1;
        }
    }
    return 0;
}

/** @fn void MMWL_closeConfigFile()
*
*   @brief Close MMWave config file
*
*   Close MMWave config file
*/
void MMWL_closeConfigFile()
{
    /* Close config file */
    if(mmwl_configfPtr != NULL){
        fclose(mmwl_configfPtr);
        mmwl_configfPtr = NULL;
    }
}

