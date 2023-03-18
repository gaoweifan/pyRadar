/****************************************************************************************
 * FileName     : mmwl_port_ftdi.h
 *
 * Description  : This file defines the functions to configure mmwave radar device through
 * 				  the FTDI interface
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
 */

/****************************************************************************************
* FILE INCLUSION PROTECTION
****************************************************************************************
*/
#ifndef MMWL_PORT_FTDI_H
#define MMWL_PORT_FTDI_H

 /*!
 \mainpage mmWaveLink FTDI Framework

 \section intro_sec Introduction

*  As mentioned in the User Guide, the mmWaveStudio uses underlying C DLLs to communicate 
*  with the mmWave Devices.These DLL's contains a generic component which is referred 
*  to as Mmwavelink. \n
*  Mmwavelink handles the communication protocol between the Host (in this case a PC) 
*  and the mmWave devices. Refer the mmWave Interface Control Document (ICD) to understand 
*  the protocol in detail. \n
*  - Although Mmwavelink handles the protocol, it still needs to send and receive data over 
*  MicroUSB cable to the FTDI Chip (on the Devpack or the DCA1000).  \n
*  - An FTDI port layer is responsible for sending the message over MicroUSB cable to 
*  the FTDI chip. \n
*  - The FTDI chip then converts this data into SPI signals to communicate 
*  with the device. \n
*  For more detailed information, refer \ref mmwl_port_ftdi

*  The following diagram explains the whole flow and the place of the port layer in the 
*  whole sequence flow.
*
*  @image html portLayerFTDI.jpg

 @note 1: The source code for mmWavelink is shared as part of the DFP release. \n
 */

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*! \brief
*  Port Description
*/
#define PORT_A          (0)
#define PORT_B          (1)
#define PORT_C          (2)
#define PORT_D          (3)

/*! \brief
*  RL STUDIO Version
*/
#define RL_STUDIO_VER               "1.1.5.4.22.11.18"

/*! \brief
*  UART Flags
*/
#define UART_IF_OPEN_FLAG_NONE       (0)
#define UART_IF_OPEN_FLAG_RE_OPEN    (1)

/*! \brief
*  Board Error Definition
*/
#define RLS_ERR_NERR                    (1)
#define RLS_ERR_PIN_MONITOR             (1<<1)
#define RLS_ERR_TSWCONN_MONITOR         (1<<2)

#define RLS_NUM_CONNECTED_DEVICES_MAX   (1U)
#define RLS_DEVICE_NAME_SIZE_MAX        (32U)

/*! \brief
*  SOP Modes
*/
#define RLS_SOP_MODE_SCAN_APTG          (1)
#define RLS_SOP_MODE_DEVELOPMENT        (2)
#define RLS_SOP_MODE_THB                (3)
#define RLS_SOP_MODE_FUNCTIONAL         (4)
#define RLS_SOP_MODE_FLASH_DOWNLOAD     (5)
#define RLS_SOP_MODE_FUNCTIONAL_CPHA1   (6)
#define RLS_SOP_MODE_FUNCTIONAL_I2C     (7)

/*! \brief
*  RadarLink Studio return Codes
*/
#define RLS_RET_CODE_OK                 (0)
#define RLS_RET_CODE_COMM_OPEN_ERROR    (-1)
#define RLS_RET_CODE_COMM_WR_ERROR      (-2)
#define RLS_RET_CODE_COMM_TIMEOUT       (-3)
#define RLS_RET_CODE_COMM_RD_ERROR      (-4)
#define RLS_RET_CODE_DEVICE_NOT_FOUND   (-5)
#define RLS_RET_CODE_MALLOC_FAILED      (-6)
#define RLS_RET_CODE_NULL_PTR_ERROR     (-7)
#define RLS_RET_CODE_INVALID_INPUT      (-8)


/******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */

/*! \brief
*  RadarLink Event Handler
*/
typedef void (*RLS_P_EVENT_HANDLER)(unsigned char deviceIndex, void* pValue);

/*! \brief
*  RadarLink Studio Device handle
*/
typedef void* rlsDevHandle_t;

/**
*  @defgroup mmwl_port_ftdi mmwl_port_ftdi
*  @brief mmWaveLink FTDI Library 
*  
*    Related Files
*   - mmwl_port_ftdi.c
*  @addtogroup mmwl_port_ftdi
*  @{
*/

/******************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************
 */

int rlsDisableDevice(unsigned char deviceIndex);

int rlsEnableDevice(unsigned char deviceIndex);

int rlsRegisterInterruptHandler(unsigned char deviceIndex, 
                                RLS_P_EVENT_HANDLER fpInterruptHdl, 
                                void* pValue);

void rlsCommIRQMask(rlsDevHandle_t hdl);

void rlsCommIRQUnMask(rlsDevHandle_t hdl);

int rlsDeviceWaitIrqStatus(rlsDevHandle_t hdl, unsigned char level);

rlsDevHandle_t rlsCommOpen(unsigned char deviceIndex, unsigned int flags);

int rlsCommClose(rlsDevHandle_t hdl);

int rlsSpiRead(rlsDevHandle_t hdl, unsigned char *preadbuffer, unsigned short ReadLength);

int rlsSpiWrite(rlsDevHandle_t hdl, unsigned char *pwriteBuffer, unsigned short WriteLength);

int rlsI2cRead(rlsDevHandle_t hdl, unsigned char *preadbuffer, unsigned short ReadLength);

int rlsI2cWrite(rlsDevHandle_t hdl, unsigned char *pwriteBuffer, unsigned short WriteLength);

/********************** Other functions ******************************/

/* Get Handle API */
rlsDevHandle_t rlsGetDeviceCtx(unsigned char ucDevId);

/*get number of devices connected */
#ifdef _WIN32
    __declspec(dllexport) int rlsGetNumofDevices(int *numofdevices);
#elif __linux__
	int rlsGetNumofDevices(int *numofdevices);
#endif
/********************** Board Control functions ******************************/

int rlsWarmReset(rlsDevHandle_t hdl, int status);

int rlsSOPControl(rlsDevHandle_t hdl, int SOPmode);

int rlsFullReset(rlsDevHandle_t hdl, int status);

int rlsOpenBoardControlIf(rlsDevHandle_t hdl);

int rlsCloseBoardControlIf(rlsDevHandle_t hdl);

int rlsOpenGenericGpioIf(rlsDevHandle_t hdl);

int rlsCloseGenericGpioIf(rlsDevHandle_t hdl);

int rlsOpenI2cIrqIf(rlsDevHandle_t  hdl);

int rlsCloseI2cIrqIf(rlsDevHandle_t  hdl);

int rlsI2CWrite(rlsDevHandle_t hdl, unsigned char slaveAddress,
	unsigned char regAddress, unsigned char msbData,
	unsigned char lsbData, int datasize);

int rlsI2CRead(rlsDevHandle_t hdl, unsigned char slaveAddress,
	unsigned char regAddress, unsigned char *msbData,
	unsigned char *lsbData, int datasize);

/*!
 Close the Doxygen group.
 @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
/*
 * END OF MMWL_PORT_FTDI_H FILE
 */
