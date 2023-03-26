/****************************************************************************************
* FileName     : mmw_config.h
*
* Description  : This file implements mmwave link example application.
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
*******************************************************************************
*/
#include <mmwavelink.h>
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* MACROS
*******************************************************************************
*/
/*string length for reading from config file*/
#define STRINGLEN 100

/******************************************************************************
* ADVANCED CHIRP PARAMS STRUCTURES
******************************************************************************
*/
/*! \brief
* Advanced Chirp LUT Profile Configuration Structure
*/
typedef struct rlAdvChirpLUTProfileCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  Profile Cfg Data
	 */
	/* This is a reference code that shows how to populate and use 4 unique parameters
	   in adv chirp generic LUT. 
	   Any number of unique parameters can be populated */
	rlUInt8_t ProfileCfgData[4];
} rlAdvChirpLUTProfileCfg_t;

/*! \brief
* Advanced Chirp LUT Start Freq Configuration Structure
*/
typedef struct rlAdvChirpLUTStartFreqCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	* @brief  Param size
	*/
	rlUInt8_t ParamSize;
	/**
	 * @brief  Param scale
	 */
	rlUInt8_t ParamScale;
	/**
	 * @brief  Start Freq Cfg Data
	 */
	/* This is a reference code that shows how to populate and use 4 unique parameters
	   in adv chirp generic LUT.
	   Any number of unique parameters can be populated */
	double StartFreqCfgData[4];
} rlAdvChirpLUTStartFreqCfg_t;

/*! \brief
* Advanced Chirp LUT Freq Slope Configuration Structure
*/
typedef struct rlAdvChirpLUTFreqSlopeCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  Freq Slope Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	double FreqSlopeCfgData[4];
} rlAdvChirpLUTFreqSlopeCfg_t;

/*! \brief
* Advanced Chirp LUT Idle time Configuration Structure
*/
typedef struct rlAdvChirpLUTIdleTimeCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	* @brief  Param size
	*/
	rlUInt8_t ParamSize;
	/**
	 * @brief  Param scale
	 */
	rlUInt8_t ParamScale;
	/**
	 * @brief  Idle time Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	double IdleTimeCfgData[4];
} rlAdvChirpLUTIdleTimeCfg_t;

/*! \brief
* Advanced Chirp LUT ADC start time Configuration Structure
*/
typedef struct rlAdvChirpLUTADCTimeCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	* @brief  Param size
	*/
	rlUInt8_t ParamSize;
	/**
	 * @brief  Param scale
	 */
	rlUInt8_t ParamScale;
	/**
	 * @brief  ADC start time Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	double ADCTimeCfgData[4];
} rlAdvChirpLUTADCTimeCfg_t;

/*! \brief
* Advanced Chirp LUT Tx Enable Configuration Structure
*/
typedef struct rlAdvChirpLUTTxEnCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  Tx Enable Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	rlUInt8_t TxEnCfgData[4];
} rlAdvChirpLUTTxEnCfg_t;

/*! \brief
* Advanced Chirp LUT BPM Enable Configuration Structure
*/
typedef struct rlAdvChirpLUTBpmEnCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  BPM Enable Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	rlUInt8_t BpmEnCfgData[4];
} rlAdvChirpLUTBpmEnCfg_t;

/*! \brief
* Advanced Chirp LUT TX0 Phase shifter Configuration Structure
*/
typedef struct rlAdvChirpLUTTx0PhShiftCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  TX0 Phase shifter Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	double Tx0PhShiftCfgData[4];
} rlAdvChirpLUTTx0PhShiftCfg_t;

/*! \brief
* Advanced Chirp LUT TX1 Phase shifter Configuration Structure
*/
typedef struct rlAdvChirpLUTTx1PhShiftCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  TX1 Phase shifter Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	double Tx1PhShiftCfgData[4];
} rlAdvChirpLUTTx1PhShiftCfg_t;

/*! \brief
* Advanced Chirp LUT TX2 Phase shifter Configuration Structure
*/
typedef struct rlAdvChirpLUTTx2PhShiftCfg
{
	/**
	 * @brief  LUT Address Offset
	 */
	rlUInt16_t LUTAddrOff;
	/**
	 * @brief  TX2 Phase shifter Cfg Data
	 */
	 /* This is a reference code that shows how to populate and use 4 unique parameters
		in adv chirp generic LUT.
		Any number of unique parameters can be populated */
	double Tx2PhShiftCfgData[4];
} rlAdvChirpLUTTx2PhShiftCfg_t;

/*! \brief
* Global Configuration Structure
*/
typedef struct rlDevGlobalCfg
{
	/**
	 * @brief  Advanced frame test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkAdvanceFrameTest;
	/**
	 * @brief  Continuous mode test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkContModeTest;
	/**
	 * @brief  Dynamic chirp test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkDynChirpTest;
	/**
	 * @brief  Dynamic profile test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkDynProfileTest;
	/**
	 * @brief  Advanced chirp test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkAdvChirpTest;
	/**
	 * @brief  Firmware download enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char EnableFwDownload;
	/**
	 * @brief  mmWaveLink logging enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char EnableMmwlLogging;
	/**
	 * @brief  Calibration enable/disable
	 *         To perform calibration store/restore
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char CalibEnable;
	/**
	 * @brief  Calibration Store/Restore
	 *         If CalibEnable = 1, then whether to store/restore
	 *         1 - Store; 0 - Restore
	 */
	unsigned char CalibStoreRestore;
	/**
	 * @brief  Transport mode
	 *         1 - I2C; 0 - SPI
	 */
	unsigned char TransferMode;

	unsigned char IsFlashConnected;

} rlDevGlobalCfg_t;

/******************************************************************************
* PARSE FUNCTION DECLARATION
******************************************************************************
*/

/*read PowerOn configurations*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx);

/*read frame configurations*/
void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs);

/*read Advance frame configurations*/
void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs);

/*read chirp configurations*/
int MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs, int chirpCfgCnt);

/*read profile configurations*/
void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs, int profileCfgCnt);

/*read lvds lane configurations*/
void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs);

/*read lane configurations*/
void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs);

/*read high speed clock configurations*/
void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs);

/*read LVDS clock configuration*/
void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs);

/*read data path configurations*/
void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs);

/*read low power configurations*/
void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs);

/*read data format configuration*/
void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs);

/*read AdcOut configuration*/
void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs);

/*read channel config parameters*/
void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
                            unsigned short cascade);

/*read the parameters for power on master*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx);

/*Read all global variable configurations from config file*/
void MMWL_getGlobalConfigStatus(rlDevGlobalCfg_t *rlDevGlobalCfgArgs);

/*read continuous mode config args*/
void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs);

/*read Dynamic Chirp config args*/
void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs);

/*read Programmable filter config args*/
void MMWL_readProgFiltConfig(rlRfProgFiltConf_t* rlProgFiltCnfgArgs);

/*read Adv chirp config args*/
void MMWL_readAdvChirpConfig(rlAdvChirpCfg_t *AdvChirpCfgArgs);

/*read Adv Chirp LUT config args*/
void MMWL_readAdvChirpProfileConfig(rlAdvChirpLUTProfileCfg_t* rlAdvChirpLUTProfileCfgArgs);
void MMWL_readAdvChirpStartFreqConfig(rlAdvChirpLUTStartFreqCfg_t* rlAdvChirpLUTStartFreqCfgArgs);
void MMWL_readAdvChirpFreqSlopeConfig(rlAdvChirpLUTFreqSlopeCfg_t* rlAdvChirpLUTFreqSlopeCfgArgs);
void MMWL_readAdvChirpIdleTimeConfig(rlAdvChirpLUTIdleTimeCfg_t* rlAdvChirpLUTIdleTimeCfgArgs);
void MMWL_readAdvChirpADCTimeConfig(rlAdvChirpLUTADCTimeCfg_t* rlAdvChirpLUTADCTimeCfgArgs);
void MMWL_readAdvChirpTxEnConfig(rlAdvChirpLUTTxEnCfg_t* rlAdvChirpLUTTxEnCfgArgs);
void MMWL_readAdvChirpBpmEnConfig(rlAdvChirpLUTBpmEnCfg_t* rlAdvChirpLUTBpmEnCfgArgs);
void MMWL_readAdvChirpTx0PhShiftConfig(rlAdvChirpLUTTx0PhShiftCfg_t* rlAdvChirpLUTTx0PhShiftCfgArgs);
void MMWL_readAdvChirpTx1PhShiftConfig(rlAdvChirpLUTTx1PhShiftCfg_t* rlAdvChirpLUTTx1PhShiftCfgArgs);
void MMWL_readAdvChirpTx2PhShiftConfig(rlAdvChirpLUTTx2PhShiftCfg_t* rlAdvChirpLUTTx2PhShiftCfgArgs);

/*get rid of trailing and leading whitespace along with "\n"*/
char *MMWL_trim(char * s);

/* Open Configuration file in read mode */
int MMWL_openConfigFile(const char *filename);

/* Close Configuration file */
void MMWL_closeConfigFile();

/*Trim the string trailing and leading whitespace*/
char *MMWL_trim(char * s);

#ifdef __cplusplus
}
#endif /* __cplusplus */