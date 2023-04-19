/****************************************************************************************
 * FileName     : mmwl_port_ftdi.c
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

 /*
 ****************************************************************************************
 * Revision History   :
 *---------------------------------------------------------------------------------------
 * Version  Date        Author             Defect No               Description
 *---------------------------------------------------------------------------------------
 * 0.1.0    22Mar2019   Rahul Goyal           -           Initial Version
 *
 * 0.2.0    25Jul2019   Rahul Goyal           -           Added Doxgyen support
 ****************************************************************************************
 */

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */

#include "mmwl_port_ftdi.h"
#include "ftd2xx.h"
#include "stdio.h"
#include <iostream>
#include <time.h>
#ifdef _WIN32
    // #pragma comment(lib,"ftd2xx.lib")
    #include <share.h>
#elif __linux__
	#include <mutex>
    #include <thread>
    #include <cstring>
#endif

/*******************************************************************************
* MACRO DEFINITIONS
********************************************************************************
*/

/*! \brief
*  NRESET, SOP, BOARD CONTROL Ports and Pins
*/
#define RLS_ENABLE_NRESET_MASK                  (0x01)
#define RLS_ENABLE_SOP_MASK                     (0x02)

/*! \brief
*  FTDI Response Timeout
*/
#define RLS_FTDI_RESPONSE_TIMEOUT               (500)

#define RLS_CLR_BIT_ALL(value)                  (value = 0U)
#define RLS_SET_BIT(value, bit)                 (value |= (1U<<(bit)))
#define RLS_CLR_BIT(value, bit)                 (value &= ~(1U<<(bit)))
#define RLS_GET_BIT(value, bit)                 (0U != (value & (1U<<(bit))))

/*! \brief
*  I2C Constants
*/
#define RLS_MSB_FALLING_EDGE_CLOCK_BYTE_IN      (0x24)
#define RLS_MSB_FALLING_EDGE_CLOCK_BYTE_OUT     (0x11)
#define RLS_MSB_RISING_EDGE_CLOCK_BIT_IN        (0x22)
#define RLS_MSB_FALLING_EDGE_CLOCK_BIT_OUT      (0x13)
/* SCL Freq=60/((1+0x4A)*2)(MHz)=400khz*/
#define RLS_I2C_CLOCK_DIVIDER                   (0x4A)

/*! \brief
*  Port A Pin config,either input (0) & output(1)
*/
#define RLS_PORTA_BIT0_SPICLK1                  (1)
#define RLS_PORTA_BIT1_MOSI1                    (1<<1)
#define RLS_PORTA_BIT2_MISO1                    (0<<2)
#define RLS_PORTA_BIT3_CS1                      (1<<3) 
#define RLS_PORTA_BIT4_TSWCONN_MONITOR_END1     (0<<4)
#define RLS_PORTA_BIT5_120PIN_TMPSNS_ALERT      (0<<5)
#define RLS_PORTA_BIT6_FTDI_PMIC_NRST           (1<<6)
#define RLS_PORTA_BIT7_PIN7_120PIN_NERROR_OUT   (0<<7)                           

/*! \brief
*  Default Port A configuration
*/
#define RLS_PORTA_CONFIG          ( RLS_PORTA_BIT0_SPICLK1 |				\
									RLS_PORTA_BIT1_MOSI1 |				    \
                                    RLS_PORTA_BIT2_MISO1 |				    \
									RLS_PORTA_BIT3_CS1 |					\
                                    RLS_PORTA_BIT4_TSWCONN_MONITOR_END1 |	\
                                    RLS_PORTA_BIT5_120PIN_TMPSNS_ALERT |	\
                                    RLS_PORTA_BIT6_FTDI_PMIC_NRST |		    \
                                    RLS_PORTA_BIT7_PIN7_120PIN_NERROR_OUT   )

/*! \brief
*  Port B Pin config,either input (0) & output(1)
*/
#define RLS_PORTB_BIT0_FTDI_1_I2C_SCL           (1)
#define RLS_PORTB_BIT1_FTDI_1_I2C_SDA_DO        (1<<1)
#define RLS_PORTB_BIT2_FTDI_1_I2C_SDA_D1        (0<<2)
#define RLS_PORTB_BIT3_UNUSED                   (1<<3)
#define RLS_PORTB_BIT4_120PIN_MUX_0             (1<<4)
#define RLS_PORTB_BIT5_12XX_1_HOST_INTR1        (0<<5)
#define RLS_PORTB_BIT6_120PIN_MUX_1             (1<<6)
#define RLS_PORTB_BIT7_12XX_1_HOST_INTR1        (0<<7)

/*! \brief
*  Default Port B configuration
*/
#define RLS_PORTB_CONFIG          ( RLS_PORTB_BIT0_FTDI_1_I2C_SCL |     \
                                    RLS_PORTB_BIT1_FTDI_1_I2C_SDA_DO |  \
                                    RLS_PORTB_BIT2_FTDI_1_I2C_SDA_D1 |  \
                                    RLS_PORTB_BIT3_UNUSED |             \
                                    RLS_PORTB_BIT4_120PIN_MUX_0 |       \
                                    RLS_PORTB_BIT5_12XX_1_HOST_INTR1 |  \
                                    RLS_PORTB_BIT6_120PIN_MUX_1 |       \
                                    RLS_PORTB_BIT7_12XX_1_HOST_INTR1	)

/*! \brief
*  Port C pin numbers
*/
#define RLS_PORTC_PIN0_RSR232_Rx                (0)
#define RLS_PORTC_PIN1_RSR232_Tx                (1)
#define RLS_PORTC_PIN2_PIN_MONITOR              (2) 
#define RLS_PORTC_PIN3_12XX_4_NRST              (3)
#define RLS_PORTC_PIN4_FTDI_1_NERRIN            (4)
#define RLS_PORTC_PIN5_12XX_3_NRST              (5)
#define RLS_PORTC_PIN6_12XX_1_2_NRST            (6) 
#define RLS_PORTC_PIN7_WARMRST                  (7)

/*! \brief
*  Port C Pin config,either input (0) & output(1)
*/
#define RLS_PORTC_BIT0_RSR232_Rx                (1)
#define RLS_PORTC_BIT1_RSR232_Tx                (0<<1)
#define RLS_PORTC_BIT2_PIN_MONITOR              (0<<2)
#define RLS_PORTC_BIT3_12XX_4_NRST              (1<<3)
#define RLS_PORTC_BIT4_FTDI_1_NERRIN            (1<<4)
#define RLS_PORTC_BIT5_12XX_3_NRST              (1<<5)
#define RLS_PORTC_BIT6_12XX_1_2_NRST            (1<<6)
#define RLS_PORTC_BIT7_WARMRST                  (1<<7)

/*! \brief
*  Default Port C configuration
*/
#define RLS_PORTC_CONFIG          ( RLS_PORTC_BIT0_RSR232_Rx |      \
									RLS_PORTC_BIT1_RSR232_Tx |      \
                                    RLS_PORTC_BIT2_PIN_MONITOR |    \
									RLS_PORTC_BIT3_12XX_4_NRST |    \
                                    RLS_PORTC_BIT4_FTDI_1_NERRIN |	\
									RLS_PORTC_BIT5_12XX_3_NRST |	\
                                    RLS_PORTC_BIT6_12XX_1_2_NRST |	\
									RLS_PORTC_BIT7_WARMRST			)

/*! \brief
*  Port D pin numbers
*/
#define RLS_PORTD_PIN0_ENABLE_ONBRD_CLK_BUFFER      (0)
#define RLS_PORTD_PIN1_UART4_TX                     (1)
#define RLS_PORTD_PIN2_PMIC_OUT_SOR1                (2)
#define RLS_PORTD_PIN3_SYNC_OUT_SOR2                (3) 
#define RLS_PORTD_PIN4_TDO_SOR3                     (4)

/*! \brief
*  Port D Pin config,either input (0) & output(1)
*/
#define RLS_PORTD_BIT0_ENABLE_ONBRD_CLK_BUFFER      (1)
#define RLS_PORTD_BIT1_UART4_TX                     (1<<1)
#define RLS_PORTD_BIT2_PMIC_OUT_SOR1                (1<<2)
#define RLS_PORTD_BIT3_SYNC_OUT_SOR2                (1<<3)
#define RLS_PORTD_BIT4_TDO_SOR3                     (1<<4)
#define RLS_PORTD_BIT5_12XX_SLAVE_SOP_PMICCLKOUT    (1<<5)
#define RLS_PORTD_BIT6_12XX_SLAVE_SOP_SYNCOUT       (1<<6)
#define RLS_PORTD_BIT7_12XX_SLAVE_SOP_TDO           (1<<7)

/*! \brief
*  Default Port D configuration
*/
#define RLS_PORTD_CONFIG            ( RLS_PORTD_BIT0_ENABLE_ONBRD_CLK_BUFFER | \
                                    RLS_PORTD_BIT1_UART4_TX |                  \
                                    RLS_PORTD_BIT2_PMIC_OUT_SOR1 |             \
                                    RLS_PORTD_BIT3_SYNC_OUT_SOR2 |             \
                                    RLS_PORTD_BIT4_TDO_SOR3 |                  \
                                    RLS_PORTD_BIT5_12XX_SLAVE_SOP_PMICCLKOUT | \
                                    RLS_PORTD_BIT6_12XX_SLAVE_SOP_SYNCOUT |    \
                                    RLS_PORTD_BIT7_12XX_SLAVE_SOP_TDO          )


#define PMIC_MASK					(1U<<4)   

/*! \brief
*  Avoid Warnings for Unused variables
*/
#define IGNORE_WARNING_UNUSED_VAR(x)            (x==0)

/*! \brief
*  Enable Custom FTDI drivers check
*/
#define CUSTOM_FTDI_NAMING						(1)
/******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */

 /*! \brief
 *  Thread for host Interrupt
 */
typedef struct rlshostIntrThread
{
	/**
     * @brief  thread handle
     */
    #ifdef _WIN32
    HANDLE                  threadHdl;
    #elif __linux__
    int                     threadHdl;
    std::thread             thread;
    #endif
	
	/**
     * @brief  ID of the thread
     */
    #ifdef _WIN32
    DWORD                   threadID;
    #elif __linux__
    std::thread::id         threadID;
    #endif
	/**
     * @brief  callback handler with mmWaveLink
     */
	RLS_P_EVENT_HANDLER     handler;
	/**
     * @brief  pointer to the data buffer/value
     */
	void*                   pValue;
}rlsThreadParam_t;

/*! \brief
*  Device Context
*/
typedef struct rlsDevCtx
{
	/**
     * @brief  device index
     */
    #ifdef _WIN32
    UINT8                   deviceIndex;
    #elif __linux__
    uint8_t                  deviceIndex;
    #endif
	/**
     * @brief  device enable/disable
     */
	BOOLEAN                 deviceEnabled;
	/**
     * @brief  irq mask/unmask
     */
	BOOLEAN                 irqMasked;
	/**
     * @brief  handle to I2C
     */
	FT_HANDLE               irqI2cHandle;
	/**
     * @brief  handle to SPI
     */
	FT_HANDLE               spiHandle;
	/**
     * @brief  handle to Board Control
     */
	FT_HANDLE               boardControlHandle;
	/**
     * @brief  handle to Generic GPIO
     */
	FT_HANDLE               genericGPIOHandle;
	/**
     * @brief  critical section
     */
    #ifdef _WIN32
    CRITICAL_SECTION        cs;
    #elif __linux__
    std::mutex              cs;
    #endif
	/**
     * @brief  handle to host interrupt thread
     */
	rlsThreadParam_t        hostIntrThread;
}rlsDevCtx_t;

/*! \brief
*  identify the FTDI device responsible for the SPI IF in different boards.
*/
const char rls_spiDeviceName[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-MB-EVM-1_FD01 A" };

/*! \brief
*  identify the FTDI device responsible for the Host IRQ in different boards.
*/
const char rls_irqI2CDeviceName[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-MB-EVM-1_FD01 B" }; 

/*! \brief
*  identify the FTDI device responsible for the Board Control in different boards.
*/
const char rls_BoardControlDeviceName[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-MB-EVM-1_FD01 C" };

/*! \brief
*  identify the FTDI device responsible for the Generic GPIO in different boards.
*/
const char rls_GenericGPIODeviceName[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-MB-EVM-1_FD01 D" };

/*! \brief
*  identify the FTDI device responsible for the SPI IF in different boards.
*/
const char rls_spiDeviceNameDevPack[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-DevPack-EVM-012 A" }; 

/*! \brief
*  identify the FTDI device responsible for the Host IRQ in different boards.
*/
const char rls_irqI2CDeviceNameDevPack[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-DevPack-EVM-012 B" }; 

/*! \brief
*  identify the FTDI device responsible for the Board Control in different boards.
*/
const char rls_BoardControlDeviceNameDevPack[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-DevPack-EVM-012 C" }; 

/*! \brief
*  identify the FTDI device responsible for the Generic GPIO in different boards.
*/
const char rls_GenericGPIODeviceNameDevPack[RLS_NUM_CONNECTED_DEVICES_MAX]\
                        [RLS_DEVICE_NAME_SIZE_MAX] = { "AR-DevPack-EVM-012 D" };

/*! \brief
*  identify the mmWave Mother board FTDI devices
*/
const char      rls_mmWaveMotherBoard[] = "AR-MB-EVM";

/*! \brief
*  identify the mmWave Devpack FTDI devices
*/
const char      rls_mmWaveDevPack[] = "AR-DevPack-EVM";

/*! \brief
*  RadarLink Studio Global Data
*/
rlsDevCtx_t     rls_devCtx[ RLS_NUM_CONNECTED_DEVICES_MAX ] = {0};

/*! \brief
*  mutex handle to exclusively use IRQ or I2C
*/
#ifdef _WIN32
HANDLE          rls_gIrqI2cMutex = NULL;
#elif __linux__
std::timed_mutex       rls_gIrqI2cMutex;
#endif

/*! \brief
*  RadarLink Studio Thread status
*/
int             rls_spiPhase = 0;
int             rls_hostIntrThreadLoop = 1;
int             rls_hostIntrExitThread = 0;
int             rls_numOfRunningSPIThreads = 0;

unsigned char i2cAddr[RLS_NUM_CONNECTED_DEVICES_MAX] = { 0 };
void DEBUG_PRINT(const char *fmt, ...)
{
//    std::cout << &fmt << std::endl;
//    printf(fmt);
}
/******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************
 */

#ifdef __linux__
    void Sleep(unsigned long dwMilliseconds){
        std::this_thread::sleep_for(std::chrono::milliseconds(dwMilliseconds));
    }
#endif

static int rlsLockDevice(rlsDevCtx_t*  pDevCtx)
{
    #ifdef _WIN32
    EnterCriticalSection(&pDevCtx->cs);
    #elif __linux__
    pDevCtx->cs.lock();
    #endif
	return RLS_RET_CODE_OK;
}

static int rlsUnlockDevice(rlsDevCtx_t*  pDevCtx)
{
    #ifdef _WIN32
    LeaveCriticalSection(&pDevCtx->cs);
    #elif __linux__
    pDevCtx->cs.unlock();
    #endif
	return RLS_RET_CODE_OK;
}

/** @fn int rlsInitBoardControlPort(rlsDevCtx_t* pDevCtx)
*
*   @brief This function initializes Port C
*   @param[in] hdl - pointer to the structure
*
*   @return int Success - 0, Failure - Error Code
*/
static int rlsInitBoardControlPort( rlsDevCtx_t* pDevCtx )
{
    FT_STATUS       status;

    if( pDevCtx->boardControlHandle != 0 )
    {
        /*Reset the port*/
        status = FT_SetBitMode( pDevCtx->boardControlHandle, 0x0,
                                FT_BITMODE_RESET );

        /* Set Timeout*/
        status |= FT_SetTimeouts( pDevCtx->boardControlHandle, 500, 100 );

        /* Set USB Parameters*/
        status |= FT_SetUSBParameters( pDevCtx->boardControlHandle, 4096,
                                       4096 );

        /* Set Latency TImer*/
        status |= FT_SetLatencyTimer( pDevCtx->boardControlHandle, 1 );
        if (status != FT_OK)
        {
            DEBUG_PRINT("Board Control USB parameters config Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        /* Enable  Asynchronous Bit Bang Controller */
        status = FT_SetBitMode(pDevCtx->boardControlHandle, RLS_PORTC_CONFIG, 
                               FT_BITMODE_ASYNC_BITBANG); 
    
        /* Set Baud Rate */
        status |= FT_SetBaudRate( pDevCtx->boardControlHandle, 115200 );
        if (status != FT_OK)
        {
            DEBUG_PRINT("Board Control Set bit modes Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }
    }
    
    return RLS_RET_CODE_OK;
}

/** @fn int rlsInitGenericGPIOPort(rlsDevCtx_t* pDevCtx)
*
*   @brief This function initializes Port D
*   @param[in] hdl - pointer to the structure
*
*   @return int Success - 0, Failure - Error Code
*/
static int rlsInitGenericGPIOPort( rlsDevCtx_t* pDevCtx )
{
    FT_STATUS       status;

    if( pDevCtx->genericGPIOHandle != 0 )
    {
       /*Reset the port*/
        status = FT_SetBitMode( pDevCtx->genericGPIOHandle, 0x0,
                                FT_BITMODE_RESET );

        /* Set Timeout*/
        status |= FT_SetTimeouts( pDevCtx->genericGPIOHandle, 500, 100 ) ; 

        /* Set USB Parameters*/
        status |= FT_SetUSBParameters( pDevCtx->genericGPIOHandle, 4096, 4096 );

        /* Set Latency TImer*/
        status |= FT_SetLatencyTimer( pDevCtx->genericGPIOHandle, 1 ) ; 
        if (status != FT_OK)
        {
            DEBUG_PRINT("Generic GPIO USB parameters config Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        /* Enable  Asynchronous Bit Bang Controller */
        status = FT_SetBitMode( pDevCtx->genericGPIOHandle, RLS_PORTD_CONFIG,
                                FT_BITMODE_ASYNC_BITBANG );

        /* Set Baud Rate */
        status |= FT_SetBaudRate( pDevCtx->genericGPIOHandle, 115200 ) ; 
        if (status != FT_OK)
        {
            DEBUG_PRINT("Generic GPIO Set bit modes Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }
    }

    return RLS_RET_CODE_OK;
}

/** @fn int rlsInitIrqI2CPort(rlsDevCtx_t* pDevCtx)
*
*   @brief This function initializes Port B
*   @param[in] hdl - pointer to the structure
*
*   @return int Success - 0, Failure - Error Code
*/
static int rlsInitIrqI2CPort(rlsDevCtx_t* pDevCtx)
{
    FT_STATUS   status;
    char        buffer[16];
    char        Buff[4096];
    DWORD       bytesSent;
    DWORD       BytesInQ;
    DWORD       dwCount;
    DWORD       BytesToTransfer;
    DWORD       BytesTransferred;
    int         bCommandEchod = 0;

    if( pDevCtx->irqI2cHandle != 0 )
    {
        /* Set Timeout*/
        status = FT_SetTimeouts( pDevCtx->irqI2cHandle, 500, 100 );

        /*Disable event and error characters*/
        status |= FT_SetChars( pDevCtx->irqI2cHandle, 0, 0, 0, 0 ) ; 

        /* Set USB Parameters*/
        status |= FT_SetUSBParameters( pDevCtx->irqI2cHandle, 4096, 4096 );

        /* Set Latency TImer*/
        status |= FT_SetLatencyTimer( pDevCtx->irqI2cHandle, 1 ) ; 
        if (status != FT_OK)
        {
            DEBUG_PRINT("IRQ USB parameters config Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        /* Enable MPSSE Controller */
        status = FT_SetBitMode( pDevCtx->irqI2cHandle, RLS_PORTB_CONFIG,
                                FT_BITMODE_MPSSE ) ; 
        if (status != FT_OK)
        {
            DEBUG_PRINT("IRQ Set bit modes Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        Sleep(50) ; /* Wait for all the USB stuff to complete and work*/

        /*
          Below codes will synchronize the MPSSE interface by sending bad command 0xAA and checking 
          if the echo command followed by bad command 0xAA can be received, this will make sure the 
          MPSSE interface enabled and synchronized successfully
        */
        BytesToTransfer = 0;
        buffer[BytesToTransfer++] = '\xAB' ;                     /*Add BAD command 0xAB*/
        status = FT_Write( pDevCtx->irqI2cHandle, buffer, BytesToTransfer,
                           &BytesTransferred ) ; 
                                                                /* Send off the BAD commands*/
        BytesToTransfer = 0 ;   /*Clear output buffer*/

        do
        {
            status = FT_GetQueueStatus(pDevCtx->irqI2cHandle, &BytesInQ);
            // Get the number of bytes in the device input buffer
        } while ((BytesInQ == 0) && (status == FT_OK));


        FT_Read(pDevCtx->irqI2cHandle, &buffer[0], (BytesInQ), &BytesTransferred);

        for (dwCount = 0; dwCount < BytesTransferred; dwCount += 2)  /*Check if Bad command and
                                                                echo command received*/
        {
            if ((buffer[dwCount] == '\xFA') && \
                (buffer[dwCount + 1] == '\xAB'))
            {
                bCommandEchod = 1;
                break;
            }
        }

        if (bCommandEchod == 0)
        {
            DEBUG_PRINT("Error - Can't receive echo command , fail to synchronize MPSSE interface\n");
        }

        /* Clear Rx queue */
        do 
        {
            status |= FT_GetQueueStatus( pDevCtx->irqI2cHandle, &BytesInQ );
            if (FT_OK != status) 
            {
                return RLS_RET_CODE_COMM_TIMEOUT;
            }
            FT_Read( pDevCtx->irqI2cHandle, &buffer[0], (BytesInQ % 16),
                     &BytesTransferred ) ; 
            for (dwCount = 0; dwCount < BytesTransferred ; dwCount += 2)  /*Check if Bad command and 
                                                                            echo command received*/
            {
                if ((buffer[dwCount] == '\xFA') && \
                    (buffer[dwCount+1] == '\xAA'))
                {
                    bCommandEchod = 0;
                    break;
                }
            }
        } while (BytesInQ > 0);

        /* Configure the MPSSE settings for I2C communication */
        buffer[BytesToTransfer++] = '\x8A' ;    /*Ensure disable clock divide by 5 for 60Mhz master clock */
        buffer[BytesToTransfer++] = '\x97' ;    /*Ensure turn off adaptive clocking */
        buffer[BytesToTransfer++] = '\x8C' ;    /*Enable 3 phase data clock, used by I2C to allow data 
                                                                                on both clock edges*/
        /* Send off the commands */
        status |= FT_Write( pDevCtx->irqI2cHandle, buffer, BytesToTransfer,
                            &BytesTransferred ) ; 

        BytesToTransfer = 0 ;                   /*Clear output buffer*/
        buffer[BytesToTransfer++] = '\x80' ;    /*Command to set directions of lower 8 pins and 
                                                            force value on bits set as output*/
        buffer[BytesToTransfer++] = (0x03 | PMIC_MASK) ; /*Set SDA, SCL high and Read Modify Write of Mux Control Pin */     //4cascade
        buffer[BytesToTransfer++] = RLS_PORTB_CONFIG ;  /*Set SK,DO pins as output and DI pin as input*/

        /* The SK clock frequency can be worked out by below algorithm with divide by 5 set as off
                                    SK frequency = 60MHz /( (1 + [(0xValueH*256) OR 0xValueL])*2)*/

        buffer[BytesToTransfer++] = '\x86' ;                                  /*Command to set clock divisor*/
        buffer[BytesToTransfer++] = (char)(RLS_I2C_CLOCK_DIVIDER & '\xFF') ;  /*Set 0xValueL of clock divisor*/
        buffer[BytesToTransfer++] = (char)((RLS_I2C_CLOCK_DIVIDER >> 8) & '\xFF') ; /*Set 0xValueH of clock divisor*/

        /* Send off the commands*/
        status |= FT_Write( pDevCtx->irqI2cHandle, buffer, BytesToTransfer,
                            &BytesTransferred ) ; 
        BytesToTransfer = 0 ;           /*Clear output buffer*/
        Sleep(20);                      /*Delay for a while*/

        /*Turn off loop back in case*/
        buffer[BytesToTransfer++] = '\x85' ;    /*Command to turn off loop back of TDI/TDO connection*/

        /* Send off the commands*/
        status |= FT_Write( pDevCtx->irqI2cHandle, buffer, BytesToTransfer,
                            &BytesTransferred ) ; 
        BytesToTransfer = 0;        /*Clear output buffer*/

        if (status != FT_OK)
        {
            DEBUG_PRINT("IRQ and I2C MSPEE Config Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        /* Clear Rx queue */
        do 
        {
            status = FT_GetQueueStatus( pDevCtx->irqI2cHandle, &BytesInQ );
            if (FT_OK != status) 
            {
                return RLS_RET_CODE_COMM_TIMEOUT;
            }

            FT_Read( pDevCtx->irqI2cHandle, &Buff[0], (BytesInQ % 4096),
                     &bytesSent ) ; 
        } while( BytesInQ > 0 );

        if (0 != BytesInQ)
        {
            DEBUG_PRINT("Clear IRQ and I2C Queue Error\n");
            return RLS_RET_CODE_COMM_RD_ERROR;
        }
    }
    return RLS_RET_CODE_OK;
}

static int rlsInitSpiPort(rlsDevCtx_t* pDevCtx)
{
    FT_STATUS       status;
    char            buffer[16];
    DWORD           bytesSent;
    char            Buff[4096];
    DWORD           BytesInQ;

    if(pDevCtx->spiHandle != 0)
    {
        /* Reset SPI device */
        status = FT_SetBitMode( pDevCtx->spiHandle, 0x0, FT_BITMODE_RESET ) ; 

        /* Set Timeout*/
        status = FT_SetTimeouts( pDevCtx->spiHandle, 500, 500 );

        /* Set Usb Parameters*/
        status |= FT_SetUSBParameters( pDevCtx->spiHandle, 4096, 4096 );

        /* Set Latency Timer */
        status |= FT_SetLatencyTimer( pDevCtx->spiHandle, 16 ) ; 
        if (status != FT_OK)
        {
            DEBUG_PRINT("SPI USB parameters config Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        /* Enable SPI Device */
        status |= FT_SetBitMode( pDevCtx->spiHandle, RLS_PORTA_CONFIG,
                                 FT_BITMODE_MPSSE );
        if (status != FT_OK)
        {
            DEBUG_PRINT("MPSSE Init Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }

        buffer[0] = 0x8A ;              /* Use 60MHz master clock (disable divide by 5) */

        buffer[1] = 0x97 ;              /* Turn off adaptive clocking (may be needed for ARM) */

        buffer[2] = 0x8D ;              /* Disable three-phase clocking */

        buffer[3] = 0x86 ;              /* Set divider command */

        buffer[4] = 0x02 ;              /* Divider low value . 60Mhz/(((high+low)+1)*2)) */

        buffer[5] = 0x00 ;              /* Divider high value */

        buffer[6] = 0x80 ;              /* Configure initial state of pins command: */

        buffer[7] = 0xC8 ;              /* initial value: */

        buffer[8] = RLS_PORTA_CONFIG ;  /* Direction: */

        status = FT_Write( pDevCtx->spiHandle, buffer, 9, &bytesSent ) ; 
        if (status != FT_OK)
        {
            DEBUG_PRINT("MSPEE Config Error\n");
            return RLS_RET_CODE_COMM_WR_ERROR;
        }  

        /* Clear Rx queue: */
        do 
        {
            status = FT_GetQueueStatus( pDevCtx->spiHandle, &BytesInQ );
            if (FT_OK != status) 
            {
                return RLS_RET_CODE_COMM_TIMEOUT;
            }

            FT_Read( pDevCtx->spiHandle, &Buff[0], (BytesInQ % 4096),
                     &bytesSent ) ; 
        } while( BytesInQ > 0 );

        if (0 != BytesInQ)
        {
            DEBUG_PRINT("Clear SPI Queue Error\n");
            return RLS_RET_CODE_COMM_RD_ERROR;
        }
    }
    return RLS_RET_CODE_OK;
}


/* 
    return 0 if succeed to find and open SPI and Irq ports 
*/
static int rlsFindAndOpenDevice( rlsDevCtx_t*   pDevCtx )
{
    FT_STATUS                   ftStatus;
#if CUSTOM_FTDI_NAMING
    DWORD                       loopCounter;
#endif
    DWORD                       numOfDevices;
    FT_DEVICE_LIST_INFO_NODE*   pInfoList;
    int                         spiDevNumber = 0;
    int                         irqI2CDevNumber = 1;
#if CUSTOM_FTDI_NAMING
    int                         boardControlDevNumber;
    int                         genericGPIODevNumber;
#endif

    if(pDevCtx == NULL)
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }

    /* If all the ports are opened and handles are updated return */
    if( (pDevCtx->spiHandle != 0) && (pDevCtx->irqI2cHandle != 0)
       && (pDevCtx->boardControlHandle != 0)
       && (pDevCtx->genericGPIOHandle  != 0))
    {
        /* all ports are already open */
        return RLS_RET_CODE_OK;
    }

    /* Create Device Information List */
    #ifdef __linux__
    FT_SetVIDPID(0x0451,0xfd03);
    #endif
    ftStatus = FT_CreateDeviceInfoList(&numOfDevices);
    if (FT_OK != ftStatus)
    {
        DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
        return RLS_RET_CODE_DEVICE_NOT_FOUND;
    }
    
    pInfoList = (FT_DEVICE_LIST_INFO_NODE*) \
                malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numOfDevices) ; 
    if (NULL == pInfoList)
    {
        DEBUG_PRINT("malloc error during read device list\n");
        return RLS_RET_CODE_MALLOC_FAILED;
    }
    
    /* Get Device Information List */
    ftStatus = FT_GetDeviceInfoList( pInfoList, &numOfDevices );
    if (FT_OK != ftStatus)
    {
        DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
        free(pInfoList);
        return RLS_RET_CODE_COMM_RD_ERROR;
    } 
        
    DEBUG_PRINT("rlsFindAndOpenDevice: Got %d devices connected \n",
                numOfDevices);
#if CUSTOM_FTDI_NAMING   
    spiDevNumber = -1;
    irqI2CDevNumber = -1;
    boardControlDevNumber = -1;
    genericGPIODevNumber = -1;
    /* Search for SPI and IRQ device in the List */
    for ( loopCounter=0; loopCounter < numOfDevices; loopCounter++ )
    {                
        DEBUG_PRINT("Device %d: %s , SN: %s\n", loopCounter, \
                    pInfoList[loopCounter].Description, \
                    pInfoList[loopCounter].SerialNumber );
        if (( memcmp( rls_spiDeviceName[pDevCtx->deviceIndex], \
             pInfoList[loopCounter].Description, \
             strlen(rls_spiDeviceName[pDevCtx->deviceIndex] )) == 0 ) || \
            ( memcmp( rls_spiDeviceNameDevPack[pDevCtx->deviceIndex], \
             pInfoList[loopCounter].Description, \
             strlen(rls_spiDeviceNameDevPack[pDevCtx->deviceIndex] )) == 0 ))
        {
            /* Found SPI FTDI Device */
            if(( pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0 )
            {
                spiDevNumber = loopCounter;
            }
        }
    
        if (( memcmp(rls_irqI2CDeviceName[pDevCtx->deviceIndex], \
              pInfoList[loopCounter].Description, \
              strlen(rls_irqI2CDeviceName[pDevCtx->deviceIndex]) ) == 0 ) || \
            ( memcmp(rls_irqI2CDeviceNameDevPack[pDevCtx->deviceIndex], \
              pInfoList[loopCounter].Description, \
              strlen(rls_irqI2CDeviceNameDevPack[pDevCtx->deviceIndex] )) == 0))
        {
            /* Found IRQ-GPIO FTDI Device */
            if((pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0)
            {
                irqI2CDevNumber = loopCounter;
            }
        }
        if (( memcmp(rls_BoardControlDeviceName[pDevCtx->deviceIndex], \
              pInfoList[loopCounter].Description, \
              strlen(rls_BoardControlDeviceName[pDevCtx->deviceIndex]) ) == 0) || \
             ( memcmp(rls_BoardControlDeviceNameDevPack[pDevCtx->deviceIndex], \
               pInfoList[loopCounter].Description, \
               strlen(rls_BoardControlDeviceNameDevPack[pDevCtx->deviceIndex]) ) == 0))
        {
            /* Found Board Control FTDI Device */
            if((pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0)
            {
                boardControlDevNumber = loopCounter;
            }
        }

        if (( memcmp(rls_GenericGPIODeviceName[pDevCtx->deviceIndex], pInfoList[loopCounter].Description, \
                                        strlen(rls_GenericGPIODeviceName[pDevCtx->deviceIndex]) ) == 0) || 
            ( memcmp(rls_GenericGPIODeviceNameDevPack[pDevCtx->deviceIndex], pInfoList[loopCounter].Description, \
                                            strlen(rls_GenericGPIODeviceNameDevPack[pDevCtx->deviceIndex]) ) == 0))
        {
            /* Found Generic GPIO FTDI Device */
            if((pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0)
            {
                genericGPIODevNumber = loopCounter;
            }
        }
    }
#endif
    free(pInfoList);
#if CUSTOM_FTDI_NAMING        
    /* Ignoring the BoardControl port as it is used for RS232 interface for some boards*/
    if((-1 == spiDevNumber) || (-1 == irqI2CDevNumber) ||
       /*(-1 == boardControlDevNumber) || */ (-1 == genericGPIODevNumber))
    {    
        DEBUG_PRINT("Not all device USB ports were found (Spi=%d, Irq & I2C=%d,\
                    Board Control=%d, Generic GPIO=%d)\n", spiDevNumber,\
                    irqI2CDevNumber, boardControlDevNumber, genericGPIODevNumber);
        return RLS_RET_CODE_DEVICE_NOT_FOUND;
    }
    else
    {
#endif
        if(!pDevCtx->spiHandle)
        {
            /* Open SPI Device */
            ftStatus = FT_Open( spiDevNumber, &pDevCtx->spiHandle );
            if (ftStatus != FT_OK)
            {
                DEBUG_PRINT("SPI port open error\n");
                pDevCtx->spiHandle = 0;
                return RLS_RET_CODE_COMM_OPEN_ERROR;
            }
            DEBUG_PRINT("SPI port opened (DeviceNum=%d)\n", spiDevNumber);
        }

        if(!pDevCtx->irqI2cHandle)
        {
            /* Open IRQ & I2C Device */
            ftStatus = FT_Open(irqI2CDevNumber, &pDevCtx->irqI2cHandle);
            if (ftStatus != FT_OK)
            {
                DEBUG_PRINT("IRQ port open error\n");
                pDevCtx->irqI2cHandle = 0;
                return RLS_RET_CODE_COMM_OPEN_ERROR;
            }
            DEBUG_PRINT("IRQ & I2C port opened (DeviceNum=%d)\n",
                        irqI2CDevNumber);
        }
#if 0
        /* Disabled as some boards are using this port for RS232 interface*/
        if(!pDevCtx->boardControlHandle)
        {
            /* Open Board Control Device */
            ftStatus = FT_Open( boardControlDevNumber,
                                &pDevCtx->boardControlHandle );
            if (ftStatus != FT_OK)
            {
                DEBUG_PRINT("Board Control port open error\n");
                pDevCtx->boardControlHandle = 0;
                return RLS_RET_CODE_COMM_OPEN_ERROR;
            }
            DEBUG_PRINT("Board Control port opened (DeviceNum=%d)\n",
                        boardControlDevNumber);
        }

        if(!pDevCtx->genericGPIOHandle)
        {
            /* Open Generic GPIO Device */
            ftStatus = FT_Open( genericGPIODevNumber,
                                &pDevCtx->genericGPIOHandle );
            if (ftStatus != FT_OK)
            {
                DEBUG_PRINT("Generic GPIO port open error\n");
                pDevCtx->genericGPIOHandle = 0;
                return RLS_RET_CODE_COMM_OPEN_ERROR;
            }
            DEBUG_PRINT("Generic GPIO port opened (DeviceNum=%d)\n",
                        genericGPIODevNumber);
        }
#endif
        return RLS_RET_CODE_OK;
#if CUSTOM_FTDI_NAMING
    }
#endif
}

static int rlsCloseDevice( rlsDevCtx_t* pDevCtx )
{
    int             error = RLS_RET_CODE_OK;
    unsigned char   Continue = 0;
    FT_HANDLE       irqI2cHandle_t = 0;
    FT_HANDLE       spiHandle_t = 0;
    FT_HANDLE       boardControlHandle_t = 0;
    FT_HANDLE       genericGPIOHandle_t = 0;

    if(pDevCtx == NULL)
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }
    
    /* Close SPI Device */
    if (0 != pDevCtx->spiHandle)
    {
        FT_Close( pDevCtx->spiHandle );
        pDevCtx->spiHandle = 0;
    }

    /* Close IRQ & I2C Device */
    if (0 != pDevCtx->irqI2cHandle)
    {
        FT_Close( pDevCtx->irqI2cHandle );
        pDevCtx->irqI2cHandle = 0;
    }

    /* Close Board Control Device */
    if (0 != pDevCtx->boardControlHandle)
    {
        FT_Close( pDevCtx->boardControlHandle );
        pDevCtx->boardControlHandle = 0;
    }

    /* Close Generic GPIO Device */
    if (0 != pDevCtx->genericGPIOHandle)
    {
        FT_Close( pDevCtx->genericGPIOHandle );
        pDevCtx->genericGPIOHandle = 0;
    }
    
    return RLS_RET_CODE_OK;
}

static rlsDevCtx_t* rlsInitDeviceCtx( unsigned char deviceIndex )
{
    rlsDevCtx_t* pDevCtx = NULL;
    
    if( deviceIndex < RLS_NUM_CONNECTED_DEVICES_MAX )
    {
        pDevCtx = &(rls_devCtx[deviceIndex]);
        #ifdef _WIN32
		if (NULL == rls_gIrqI2cMutex)
		{
			rls_gIrqI2cMutex = CreateMutex( NULL,              // default security attributes
											FALSE,             // initially not owned
											NULL);             // unnamed mutex
		}
		#endif
        pDevCtx->deviceIndex = deviceIndex ; /*adding right device index for device context*/
        pDevCtx->deviceEnabled = FALSE;
        pDevCtx->irqMasked = FALSE;
        pDevCtx->irqI2cHandle = 0;
        pDevCtx->spiHandle = 0;

        pDevCtx->hostIntrThread.threadHdl = 0;
        pDevCtx->hostIntrThread.handler = 0;
        pDevCtx->hostIntrThread.pValue = 0;
        #ifdef _WIN32
        pDevCtx->hostIntrThread.threadID = 0;
        InitializeCriticalSection( &(pDevCtx->cs) );
        #endif
    }

    return pDevCtx;
}

static int rlsClearDeviceCtx(rlsDevCtx_t* pDevCtx)
{
    int retVal = RLS_RET_CODE_NULL_PTR_ERROR;
    
    if(pDevCtx != NULL)
    {
        /* Delete Critical Section */
        #ifdef _WIN32
        DeleteCriticalSection( &(pDevCtx->cs) );
		CloseHandle(rls_gIrqI2cMutex);
        #endif

        /* Clear device context */
        pDevCtx->deviceEnabled = FALSE;
        retVal = RLS_RET_CODE_OK;
    }

    return retVal;
}

static int rlsCheckForIrq(rlsDevCtx_t* pDevCtx , int level)
{
    unsigned char   inputBuffer[16] = {0};
    unsigned char   outputBuffer[16] = {0};
    FT_STATUS       ftStatus = FT_OK;
    DWORD           BytesTransfer = 0;
    DWORD           BytesTransfered = 0;
    DWORD           BytesToRead = 0;
    DWORD           BytesRead = 0;
    int             hostIntRecv = 0;

    BytesTransfer = 0 ;                                   // command to read its state
    outputBuffer[BytesTransfer++] = 0x81;
    ftStatus = FT_Write( pDevCtx->irqI2cHandle, outputBuffer, BytesTransfer,
                         &BytesTransfered );
    do
    {
        ftStatus = FT_GetQueueStatus( pDevCtx->irqI2cHandle, &BytesToRead );
    } while((BytesToRead == 0) && ftStatus == FT_OK);
    ftStatus |= FT_Read( pDevCtx->irqI2cHandle, inputBuffer, BytesToRead,
                         &BytesRead );
    if ((ftStatus != FT_OK))
    {
        return RLS_RET_CODE_COMM_RD_ERROR;
    }

    if( ((inputBuffer[0] & (1<<5)) != 0 ) && (pDevCtx->deviceEnabled == TRUE ))
    {   
        hostIntRecv = (inputBuffer[0] >> 5) & 0x1;
    }
    
    if (hostIntRecv == level)
    {
		DEBUG_PRINT("Device [%d] Host IRQ High\r\n",
                    pDevCtx->deviceIndex);

		return RLS_RET_CODE_OK;
    }
	else
	{
		return RLS_RET_CODE_COMM_TIMEOUT;
	}
}

static DWORD WINAPI rlsPollingThreadEntrySpi( LPVOID pParam ) 
{
    rlsDevCtx_t* pDevCtx = (rlsDevCtx_t*)pParam;
    int checkIrqRes = 0;

    rls_hostIntrThreadLoop = 1;
    rls_hostIntrExitThread = 1;
    rls_numOfRunningSPIThreads++;

    while( rls_hostIntrThreadLoop )
    {                  
        if ( (!pDevCtx->deviceEnabled) || (pDevCtx->irqMasked))
        {
            Sleep(1);
			/* Added this one for letting WIN7 yield for the next thread.
			In case we don't have it - the do/while loop will not yield and
			we will delay other threads by waiting for this thread */
            #ifdef _WIN32
                SwitchToThread();
            #elif __linux__
                std::this_thread::yield();
            #endif
            continue;
        }

        #ifdef _WIN32
		if (0 == WaitForSingleObject(rls_gIrqI2cMutex, 1000))
        #elif __linux__
        if (1 == rls_gIrqI2cMutex.try_lock_for( std::chrono::milliseconds(1000)))
        #endif
		{
			/* check for Host IRQ */
			checkIrqRes = rlsCheckForIrq(pDevCtx, 1);
            #ifdef _WIN32
                ReleaseMutex(rls_gIrqI2cMutex);
            #elif __linux__
                rls_gIrqI2cMutex.unlock();
            #endif
		}

        /* Check if Device is Enabled, IRQ is received and is not in masked State */
        if ((!pDevCtx->deviceEnabled) || (0 != checkIrqRes) || \
            (pDevCtx->irqMasked))
        {
			Sleep(1);
			/* Added this one for letting WIN7 yield for the next thread.
		    In case we don't have it - the do/while loop will not yield and
		    we will delay other threads by waiting for this thread */
			#ifdef _WIN32
                SwitchToThread();
            #elif __linux__
                std::this_thread::yield();
            #endif
            continue;
        }

        /* Check if Interrupt handler is registered */
        if (pDevCtx->hostIntrThread.handler)
        {
            pDevCtx->hostIntrThread.handler(pDevCtx->deviceIndex,
                                            (rlsDevHandle_t)pDevCtx);
        }
        
        if (rls_numOfRunningSPIThreads > 1)
        {
            rls_numOfRunningSPIThreads--;
            pDevCtx->hostIntrThread.threadHdl = NULL;
            return RLS_RET_CODE_OK;
        }
    }
    rls_hostIntrExitThread = 0;
    rls_numOfRunningSPIThreads--;
    return RLS_RET_CODE_OK;
}

static int rlsStartIrqPollingThread( rlsDevCtx_t* pDevCtx )
{
    int             error = RLS_RET_CODE_OK;
    #ifdef _WIN32
    DWORD           currThread = GetCurrentThreadId();
    #elif __linux__
    const auto      currThread = std::this_thread::get_id();
    #endif
    

    rls_hostIntrThreadLoop = 0;


    if (0 == pDevCtx->hostIntrThread.threadHdl)
    {
        /* Create Host IRQ Polling Thread */
        #ifdef _WIN32
        pDevCtx->hostIntrThread.threadHdl = CreateThread(NULL, 0,
                                        rlsPollingThreadEntrySpi, pDevCtx, 0,
                                        &pDevCtx->hostIntrThread.threadID);
        #elif __linux__
        pDevCtx->hostIntrThread.thread = std::thread(rlsPollingThreadEntrySpi,pDevCtx);
        pDevCtx->hostIntrThread.threadID = pDevCtx->hostIntrThread.thread.get_id();
        pDevCtx->hostIntrThread.threadHdl = 1;
        #endif
        
    }
    else
    {
        rlsLockDevice(pDevCtx);
        if( pDevCtx->hostIntrThread.threadID != currThread )
        {
            #ifdef _WIN32
            TerminateThread( pDevCtx->hostIntrThread.threadHdl,0 );
            #elif __linux__
            pDevCtx->hostIntrThread.thread.join();
            pDevCtx->hostIntrThread.threadHdl = 0;
            #endif
            Sleep(200);
        }
        #ifdef _WIN32
        pDevCtx->hostIntrThread.threadHdl = CreateThread( NULL, 0,
                                        rlsPollingThreadEntrySpi, pDevCtx, 0,
                                        &pDevCtx->hostIntrThread.threadID );
        #elif __linux__
        pDevCtx->hostIntrThread.thread = std::thread(rlsPollingThreadEntrySpi,pDevCtx);
        pDevCtx->hostIntrThread.threadID = pDevCtx->hostIntrThread.thread.get_id();
        pDevCtx->hostIntrThread.threadHdl = 1;
        #endif
        rlsUnlockDevice(pDevCtx);
    }

	/* wait for the thread to start */
    while(rls_hostIntrThreadLoop == 0)
    {
        Sleep(1);
    }
    
    return RLS_RET_CODE_OK;
}

static int rlsStopIrqPollingThread( rlsDevCtx_t* pDevCtx )
{
    int             error = RLS_RET_CODE_OK;
    #ifdef _WIN32
    DWORD           currThread = GetCurrentThreadId();
    #elif __linux__
    const auto      currThread = std::this_thread::get_id();
    #endif
    
    rls_hostIntrThreadLoop = 0;

    if( pDevCtx->hostIntrThread.threadHdl && (pDevCtx->hostIntrThread.threadID \
        != currThread ))
    {
        #ifdef __linux__
            pDevCtx->hostIntrThread.thread.join();
        #endif
        /* Wait for polling thread to terminate */
        while(rls_hostIntrExitThread == 1)
        {
            Sleep(1);
        }
    }
    else
    {
        DEBUG_PRINT("Stopping  RL IRQ polling thread \n");
    }
    pDevCtx->hostIntrThread.threadHdl = NULL;

    return RLS_RET_CODE_OK;
}

/** @fn void rlsHighSpeedSetI2CStart()
*
*   @brief This function creates a start condition for I2C
*   @param[in] pI2cBuffer - pointer to the buffer
*   @param[in] pI2cBytesToTransfer - pointer to the bytes to transfer
*
*   @return void
*
*    Below function will setup the START condition for I2C bus communication. First, set SDA, 
*    SCL high and ensure hold time requirement by device is met. Second, set SDA low, SCL high 
*    and ensure setup time requirement met. Finally, set SDA, SCL low
*/
static void rlsHighSpeedSetI2CStart( BYTE *pI2cBuffer,
                                     DWORD *pI2cBytesToTransfer )
{
    DWORD dwCount;

    for(dwCount=0; dwCount < 4; dwCount++)              /* Repeat commands to ensure the min period of start
                                                                            hold time ie 600ns is achieved*/
    {
        pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;   /*Set directions of lower 8 pins and force
                                                                            value on bits set as output*/
        pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x03 | PMIC_MASK) ;    /*Set SDA, SCL high and Read Modify Write of Mux Control Pin*/      //4cascade
        pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG ;     /*Set SK,DO as o/p and DI as i/p*/
    }

    for(dwCount=0; dwCount < 4; dwCount++)              /* Repeat commands to ensure the min period of start
                                                                            setup time ie 600ns is achieved*/
    {
        pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;   /*Set directions of lower 8 pins and force
                                                                        value on bits set as output*/
        pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x01 | PMIC_MASK) ; /*Set SDA low, SCL high and Read Modify Write of Mux Control Pin*/     //4cascade
        pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG ;  /*Set SK,DO as o/p and DI as i/p*/
    }
    
    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;       /*Set directions of lower 8 pins and force
                                                                    value on bits set as output*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x00 | PMIC_MASK) ; /*Set SDA, SCL low and Read Modify Write of Mux Control Pin*/      //4cascade
    pI2cBuffer[(*pI2cBytesToTransfer)++] =  RLS_PORTB_CONFIG ;     /*Set SK,DO as o/p and DI as i/p */
}

/** @fn void rlsHighSpeedSetI2CStop()
*
*   @brief This function creates a stop condition for I2C
*   @param[in] pI2cBuffer - pointer to the buffer
*   @param[in] pI2cBytesToTransfer - pointer to the bytes to transfer
*   @return void
*
*    Below function will setup the STOP condition for I2C bus communication. First, set SDA low, 
*    SCL high and ensure setup time requirement by device is met. Second, set SDA, SCL high and 
*    ensure hold time requirement met. Finally, set SDA, SCL as input to tristate the I2C bus.
*/
static void rlsHighSpeedSetI2CStop( BYTE *pI2cBuffer,
                                    DWORD *pI2cBytesToTransfer )
{
    DWORD dwCount;
    for(dwCount=0; dwCount<4; dwCount++)                /* Repeat commands to ensure the min period of stop 
                                                                            setup time ie 600ns is achieved*/
    {
        pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;   /*Set directions of lower 8 pins and force
                                                                    value on bits set as output*/
        pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x01| PMIC_MASK) ; /*Set SDA low, SCL high and Read Modify Write of Mux Control Pin*/         //4cascade
        pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG ;      /*Set SK,DO as o/p and DI as i/p*/
    }
 
    for(dwCount=0; dwCount<4; dwCount++)                /* Repeat commands to ensure the min period of stop
                                                                            hold time ie 600ns is achieved*/
    {
        pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;   /*Set directions of lower 8 pins and force
                                                                    value on bits set as output*/
        pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x03 | PMIC_MASK) ; /*Set SDA, SCL high and Read Modify Write of Mux Control Pin*/         //4cascade
        pI2cBuffer[(*pI2cBytesToTransfer)++] =  RLS_PORTB_CONFIG ; /*Set SK,DO as o/p and DI as i/p*/
    }

}

/** @fn int rlsI2CSendByteAndCheckACK(FT_HANDLE ftHandle, BYTE dwDataSend)
*
*   @brief This function send a byte of data
*   @param[in] ftHandle - handle to ftdi port
*   @param[in] pI2cBuffer - pointer to the buffer
*   @param[in] pI2cBytesToTransfer - pointer to the bytes to transfer
*   @param[in] pI2cBytesTransfered - pointer to the bytes transfered
*   @param[in] dwDataSend - Data to be send
*
*   @return int Success - 0, Failure - Error Code
*
*   Below function will send a data byte to I2C slave, then check if the ACK bit sent from 
*   slave device can be received.Return true if data is successfully sent and ACK bit is received.
*   Return false if error during sending data or ACK bit can't be received
*/
static int rlsI2CSendByteAndCheckACK( FT_HANDLE ftHandle, BYTE *pI2cBuffer,
                                      DWORD *pI2cBytesToTransfer, 
                                      DWORD *pI2cBytesTransfered, 
                                      BYTE dwDataSend )
{
    FT_STATUS ftStatus = FT_OK;
    
    /*Clock data byte out on �ve Clock Edge MSB first*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_MSB_FALLING_EDGE_CLOCK_BYTE_OUT;
    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x00';
    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x00' ;       /*Data length of 0x0000 means 1 byte data to clock out*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = dwDataSend ;   /*Add data to be send*/
    
    /* Get Acknowledge bit from I2C slave */
    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;       /*Command to set directions of lower 8 pins and
                                                                    force value on  bits set as output*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x00 | PMIC_MASK) ;        /*Set SCL low and Read Modify Write of Mux Control Pin*/            //4cascade
    pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG & 0xFD ;  /*Set SK as output and DO as input*/

    /*Command to scan in ACK bit , -ve clock Edge MSB first */                         
    pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_MSB_RISING_EDGE_CLOCK_BIT_IN ; 
    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x0'  ;       /*Length of 0x0 means to scan in 1 bit*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x87' ;       /*Send answer back immediate command*/
    
    /*Send off the commands*/
    ftStatus = FT_Write( ftHandle, pI2cBuffer, (*pI2cBytesToTransfer),
                         pI2cBytesTransfered ) ; 
    (*pI2cBytesToTransfer) = 0 ;     /*Clear output buffer*/
    
    /*Check if ACK bit received, may need to read more times to get ACK bit or fail if timeout*/
    ftStatus = FT_Read( ftHandle, pI2cBuffer, 1, pI2cBytesTransfered ) ;  /*Read one byte from device
                                                                                    receive buffer*/
    if( (ftStatus != FT_OK) || (*pI2cBytesTransfered == 0) )
    { 
        return FALSE ;      /*Error, can't get the ACK bit from I2C slave */ 
    }
    else
    if ( (pI2cBuffer[0] & '\x1') != '\x0' )          /*Check ACK bit 0 on data byte read out*/
    { 
        (*pI2cBytesToTransfer) = 0;
        return FALSE ;                              /*Error, can't get the ACK bit from I2C slave */ 
    }

    pI2cBuffer[(*pI2cBytesToTransfer)++] = '\x80' ;   /*Command to set directions of lower 8 pins and 
                                                                force value on bits set as output*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = (0x02 | PMIC_MASK) ; /*Set SDA high, SCL low and Read Modify Write of Mux Control Pin*/         //4cascade
    pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG ;  /*Set SK,DO as output and DI as input*/
    return TRUE;
}

/** @fn int rlsI2CReadByteAndSendNACK(FT_HANDLE ftHandle, BYTE *pI2cBuffer, 
                                    DWORD *pI2cBytesToTransfer, DWORD *pI2cBytesTransfered, 
                                    BYTE *Data)
*
*   @brief This function send a byte of data
*   @param[in] ftHandle - handle to ftdi port
*   @param[in] pI2cBuffer - pointer to the buffer
*   @param[in] pI2cBytesToTransfer - pointer to the bytes to transfer
*   @param[in] pI2cBytesTransfered - pointer to the bytes transfered
*   @param[in] Data - pointer to Data read
*
*   @return int Success - 0, Failure - Error Code
*
*   Below function will read a data byte from I2C slave, then give NACK bit. Return true if data
*   is read successfully and NACK bit is sent. Return false if error during reading data or NACK 
*   bit can't be send.
*/
static int rlsI2CReadByteAndSendNACK( FT_HANDLE ftHandle, BYTE *pI2cBuffer,
                                      DWORD *pI2cBytesToTransfer, 
                                      DWORD *pI2cBytesTransfered, BYTE *Data )
{
    int     ftStatus, readTimeoutCounter;
    DWORD   i2cNumInputBuffer;
    BYTE    inputBuffer[10];

    /* Clock one byte of data in...*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x20 ; /*Command: clock data byte in on clk rising edge*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /* Length*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /* Length 0x0000 means clock ONE byte in*/
    
    /*Now clock out one bit (ACK/NAK). This bit has value '1' to send a NAK to the I2C Slave*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x13 ; /*Command: clock data bits out on clk falling edge*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /*Length of 0x00 means clock out ONE bit*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0xFF ; /*Command will send bit 7 of this byte (= �1�)*/
    
    /* Put I2C line back to idle (during transfer) state... Clock line low, Data line high*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x80 ; /* Command to set ADbus direction/ data*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = (0xFE | PMIC_MASK) ; /* Set the value of the pins and Read Modify Write of Mux Control Pin*/            //4cascade
    pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG & 0xFB ; /* Set pins o/p except bit 2 (DI)*/
    /* B0 (SCL) is output driven low
       B1 (DATA OUT) is output high (open drain)
       B2 (DATA IN) is input (therefore the output value specified is ignored)*/
    
    /* This command then tells the MPSSE to send any results gathered back immediately*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x87 ; /* Send answer back immediate command*/
    
    /* Send off the commands to the FT4232H*/
    ftStatus = FT_Write( ftHandle, pI2cBuffer, *pI2cBytesToTransfer,
                         pI2cBytesTransfered );
    (*pI2cBytesToTransfer) = 0 ; /*Clear output buffer*/

    /* Now wait for the byte which we read to come back to the host PC*/
    i2cNumInputBuffer = 0;
    readTimeoutCounter = 0;
    ftStatus = FT_GetQueueStatus( ftHandle, &i2cNumInputBuffer );
    
    /* Get number of bytes in the input buffer*/
    while((i2cNumInputBuffer < 1) && (ftStatus == FT_OK) && \
           (readTimeoutCounter < 500))
    {
        /* Sit in this loop until
         (1) we receive the one byte expected
         or (2) a hardware error occurs causing the GetQueueStatus to return an error code
         or (3) we have checked 500 times and the expected byte is not coming*/
        ftStatus = FT_GetQueueStatus( ftHandle, &i2cNumInputBuffer ) ; /* Get # bytes in buffer*/
        readTimeoutCounter++;
        Sleep(1) ; // short delay
    }
    /* If loop above exited due to the byte coming back (not an error code and not a timeout)
       then read the byte available and return True to indicate success*/
    if ((ftStatus == FT_OK) && (readTimeoutCounter < 500))
    {
        ftStatus = FT_Read( ftHandle, &inputBuffer, i2cNumInputBuffer,
                            pI2cBytesTransfered);
        *Data = inputBuffer[0] ;    /* store the data read*/
        return TRUE ;               /* Indicate success */
    }
    else
    {
        return FALSE ;  /* Failed to get any data back or got an error code back*/
    }
}

/** @fn int rlsI2CReadByteAndSendACK(FT_HANDLE ftHandle, BYTE *pI2cBuffer, 
                                     DWORD *pI2cBytesToTransfer, DWORD *pI2cBytesTransfered, 
                                     BYTE *Data)
*
*   @brief This function send a byte of data
*   @param[in] ftHandle - handle to ftdi port
*   @param[in] pI2cBuffer - pointer to the buffer
*   @param[in] pI2cBytesToTransfer - pointer to the bytes to transfer
*   @param[in] pI2cBytesTransfered - pointer to the bytes transfered
*   @param[in] Data - pointer to Data read
*
*   @return int Success - 0, Failure - Error Code
*
*   Below function will read a data byte from I2C slave, then give ACK bit. Return true if data is
*   read successfully and ACK bit is sent. Return false if error during reading data or ACK bit 
*   can't be send.
*/
static int rlsI2CReadByteAndSendACK( FT_HANDLE ftHandle, BYTE *pI2cBuffer,
                                     DWORD *pI2cBytesToTransfer,  
                                     DWORD *pI2cBytesTransfered, BYTE *Data )
{
    int     ftStatus, readTimeoutCounter;
    DWORD   i2cNumInputBuffer;
    BYTE    inputBuffer[10];

    /* Clock one byte of data in...*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x20 ; /*Command: clock data byte in on clk rising edge*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /* Length*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /* Length 0x0000 means clock ONE byte in*/
    
    /*Now clock out one bit (ACK/NAK). This bit has value '1' to send a NAK to the I2C Slave*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x13 ; /*Command: clock data bits out on clk falling 
                                                    edge*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /*Length of 0x00 means clock out ONE bit*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x00 ; /*Command will send bit 7 of this byte (= �1�)*/
    
    /*Put I2C line back to idle (during transfer) state... Clock line low, Data line high*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x80 ; /*Command to set ADbus direction/ data*/
    pI2cBuffer[(*pI2cBytesToTransfer)++] = (0xFE | PMIC_MASK) ; /*Set the value of the pins and Read Modify Write of Mux Control Pin*/         //4cascade
    pI2cBuffer[(*pI2cBytesToTransfer)++] = RLS_PORTB_CONFIG & 0xFB ; /*Set pins o/p except bit 2 (DI)*/
    /* B0 (SCL) is output driven low
       B1 (DATA OUT) is output high (open drain)
       B2 (DATA IN) is input (therefore the output value specified is ignored)*/
    
    /* This command then tells the MPSSE to send any results gathered back immediately */
    pI2cBuffer[(*pI2cBytesToTransfer)++] = 0x87 ; /* Send answer back immediate command */
    
    /* Send off the commands to the FT4232H */
    ftStatus = FT_Write( ftHandle, pI2cBuffer, *pI2cBytesToTransfer,
                         pI2cBytesTransfered );
    (*pI2cBytesToTransfer) = 0 ; /*Clear output buffer*/

    /* Now wait for the byte which we read to come back to the host PC*/
    i2cNumInputBuffer = 0;
    readTimeoutCounter = 0;
    ftStatus = FT_GetQueueStatus( ftHandle, &i2cNumInputBuffer );
    
    /* Get number of bytes in the input buffer*/
    while( (i2cNumInputBuffer < 1) && (ftStatus == FT_OK) && \
           (readTimeoutCounter < 500) )
    {
        /* Sit in this loop until
         (1) we receive the one byte expected
          or (2) a hardware error occurs causing the GetQueueStatus to return an error code
          or (3) we have checked 500 times and the expected byte is not coming*/
        ftStatus = FT_GetQueueStatus( ftHandle, &i2cNumInputBuffer ) ; /*Get # bytes in buffer*/
        readTimeoutCounter++;
        Sleep(1) ; /*short delay*/
    }
    
    /* If loop above exited due to the byte coming back (not an error code and not a timeout)
       then read the byte available and return True to indicate success*/
    if( (ftStatus == FT_OK) && (readTimeoutCounter < 500) )
    {
        ftStatus = FT_Read( ftHandle, &inputBuffer, i2cNumInputBuffer,
                            pI2cBytesTransfered );
        *Data = inputBuffer[0] ;    /* store the data read */
        return TRUE ;               /* Indicate success */
    }
    else
    {
        return FALSE ;  /* Failed to get any data back or got an error code back */
    }
}

/** @fn int rlsDisableDevice(unsigned char deviceIndex)
*
*   @brief This function disables the device having the specified device index
*   @param[in] deviceIndex - Index of the device that needs to be disabled
*
*   @return int Success - RLS_RET_CODE_OK,
*               Failure - RLS_RET_CODE_NULL_PTR_ERROR
*/
int rlsDisableDevice( unsigned char deviceIndex )
{
    rlsDevCtx_t*   pDevCtx;

    DEBUG_PRINT("Disable Device\n");
    
#ifndef RLS_SPI_SIMULATION
    pDevCtx = (rlsDevCtx_t *)rlsGetDeviceCtx(deviceIndex);
    if(NULL != pDevCtx)
    {
        pDevCtx->deviceEnabled = FALSE;

        if(pDevCtx->hostIntrThread.threadHdl != NULL)
        {
            rlsStopIrqPollingThread(pDevCtx);
            pDevCtx->hostIntrThread.threadHdl = NULL;
        }
    }
    else
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }
        
#endif

    return RLS_RET_CODE_OK;
}

/** @fn int rlsEnableDevice(unsigned char deviceIndex)
*
*   @brief This function enables the device having the specified device index
*   @param[in] deviceIndex - Index of the device that needs to be enabled
*
*   @return int Success - RLS_RET_CODE_OK,
*               Failure - RLS_RET_CODE_NULL_PTR_ERROR
*/
int rlsEnableDevice( unsigned char deviceIndex )
{
    rlsDevCtx_t*    pDevCtx;
    int             retVal = RLS_RET_CODE_OK;

    DEBUG_PRINT("Enable Device\n");

#ifndef RLS_SPI_SIMULATION
    pDevCtx = (rlsDevCtx_t *)rlsGetDeviceCtx(deviceIndex);

    if(NULL != pDevCtx)
    {
        pDevCtx->deviceEnabled = TRUE;
        /* Starting Interrupt Thread after Power on to avoid Spurious Interrupt */
        if (NULL != pDevCtx->hostIntrThread.handler)
        {
            rlsStartIrqPollingThread(pDevCtx);
        }
    }
    else
    {
        retVal = RLS_RET_CODE_NULL_PTR_ERROR;
    }
#endif
    return retVal;
}

/** @fn int rlsRegisterInterruptHandler(unsigned char deviceIndex, 
										RLS_P_EVENT_HANDLER fpInterruptHdl, 
										void* pValue);
*
*   @brief This function register the interrupt handler with mmwavelink for the
*	       device having the specified device index
*   @param[in] deviceIndex - Index of the device for which the interrupt needs to
*							 be registered
*   @param[in] fpInterruptHdl - Function pointer to be registered as interrupt
*								handler
*	@param[in] pValue - pointer to the value which will be passed as an argument
*						to the registered function
*
*   @return int Success - RLS_RET_CODE_OK,
*               Failure - RLS_RET_CODE_NULL_PTR_ERROR
*/
int rlsRegisterInterruptHandler( unsigned char deviceIndex,
                                 RLS_P_EVENT_HANDLER fpInterruptHdl ,
                                 void* pValue )
{
    rlsDevCtx_t*   pDevCtx;
   
    pDevCtx = (rlsDevCtx_t *)rlsGetDeviceCtx(deviceIndex);
    
    if(NULL != pDevCtx)
    {
        rlsLockDevice(pDevCtx);
        pDevCtx->hostIntrThread.handler = fpInterruptHdl;
        pDevCtx->hostIntrThread.pValue = pValue;
        rlsUnlockDevice(pDevCtx);
        if(pDevCtx->hostIntrThread.threadHdl != NULL)
        {
            rlsStopIrqPollingThread(pDevCtx);
            pDevCtx->hostIntrThread.threadHdl = NULL;
        }
        return RLS_RET_CODE_OK;
    }
    else
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }
}

/** @fn int rlsCommIRQMask(rlsDevHandle_t hdl)
*
*   @brief This function masks the Host IRQ
*   @param[in] hdl - handle for the device for which the IRQ needs to be masked
*
*   @return None
*/
void rlsCommIRQMask( rlsDevHandle_t hdl )
{
    rlsDevCtx_t* pDevCtx = (rlsDevCtx_t *)hdl;

    rlsLockDevice(pDevCtx);
    pDevCtx->irqMasked = TRUE;
    rlsUnlockDevice(pDevCtx);
}


/** @fn int rlsCommIRQUnMask(rlsDevHandle_t hdl)
*
*   @brief This function unmasks the Host IRQ
*   @param[in] hdl - handle for the device for which the IRQ needs to be masked
*
*   @return None
*/
void rlsCommIRQUnMask( rlsDevHandle_t hdl )
{
    rlsDevCtx_t* pDevCtx = (rlsDevCtx_t *)hdl;

    rlsLockDevice(pDevCtx);
    pDevCtx->irqMasked = FALSE;
    rlsUnlockDevice(pDevCtx);
}

/** @fn int rlsDeviceWaitIrqStatus(rlsDevHandle_t hdl, unsigned char level)
*
*   @brief This function waits until the Host IRQ pin level matches the
*          specified level
*   @param[in] hdl - handle for the device for which the specified Host IRQ state
*                    needs to be confirmed
*   @param[in] level - Host IRQ pin Level should match this value for the
*                      function to return
*
*   @return int Success - RLS_RET_CODE_OK,
*               Failure - RLS_RET_CODE_COMM_RD_ERROR
*/
int rlsDeviceWaitIrqStatus( rlsDevHandle_t hdl, unsigned char level )
{
    rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
    unsigned char   inputBuffer[16] = { 0 };
    unsigned char   outputBuffer[16] = { 0 };
    FT_STATUS       ftStatus = FT_OK;
    DWORD           BytesTransfer = 0;
    DWORD           BytesTransfered = 0;
    DWORD           BytesToRead = 0;
    DWORD           BytesRead = 0;
    int             hostIntRecv = 0;
    
    #ifdef _WIN32
        WaitForSingleObject(rls_gIrqI2cMutex, INFINITE);
    #elif __linux__
        rls_gIrqI2cMutex.lock();
    #endif
    do
    {
        BytesTransfer = 0;

        /* Read Board Control Port(C)*/
        outputBuffer[BytesTransfer++] = 0x81;
        ftStatus = FT_Write( pDevCtx->irqI2cHandle, outputBuffer, BytesTransfer,
                             &BytesTransfered );
            
        Sleep(2);
            
        ftStatus |= FT_GetQueueStatus( pDevCtx->irqI2cHandle, &BytesToRead );
            
        ftStatus |= FT_Read( pDevCtx->irqI2cHandle, inputBuffer, BytesToRead,
                             &BytesRead );

        if ((ftStatus != FT_OK))
        {
            break;
        }
        else
        {
            hostIntRecv = inputBuffer[0] & (1 << 5);
        }
        
        if (hostIntRecv == level)
        {
            DEBUG_PRINT("Device [%d] Host IRQ Low\r\n", pDevCtx->deviceIndex);
            #ifdef _WIN32
                ReleaseMutex(rls_gIrqI2cMutex);
            #elif __linux__
                rls_gIrqI2cMutex.unlock();
            #endif
            return RLS_RET_CODE_OK;
        }
    } while( level != hostIntRecv );
    #ifdef _WIN32
        ReleaseMutex(rls_gIrqI2cMutex);
    #elif __linux__
        rls_gIrqI2cMutex.unlock();
    #endif
    return RLS_RET_CODE_COMM_RD_ERROR;
}

/** @fn rlsDevHandle_t rlsCommOpen(unsigned char deviceIndex,
*                                  unsigned int flags)
*
*   @brief This function opens and initialize all the FTDI ports
*   @param[in] deviceIndex - Index of the device for which the FTDI ports needs
*                            to be initialized
*   @param[in] flags - UNUSED
*
*   @return rlsDevHandle_t Success - Device handle for the initialized FTDI
*                                    interface
*                          Failure - NULL
*/
rlsDevHandle_t rlsCommOpen(unsigned char deviceIndex, unsigned int flags )
{
    rlsDevCtx_t*    pDevCtx;
    BOOLEAN         InitOk = TRUE;

    pDevCtx = rlsInitDeviceCtx( deviceIndex );

    if( NULL != pDevCtx )
    {
        IGNORE_WARNING_UNUSED_VAR( flags );        
        DEBUG_PRINT("[VER] mmWaveLink Studio Version: %s\r\n", RL_STUDIO_VER);

        /* Find and Open SPI/IRQ device */
        if (0 != rlsFindAndOpenDevice(pDevCtx))
        {
            InitOk = FALSE;
            DEBUG_PRINT("could not open device\r\n");
        }
        
        /* Initialize SPI device */
        if (0 != rlsInitSpiPort(pDevCtx))
        {
            InitOk = FALSE;
            DEBUG_PRINT("could not open spi port\r\n");
        }
        
        /* Initialize IRQ device */
        if (0 != rlsInitIrqI2CPort(pDevCtx))
        {
            InitOk = FALSE;
            DEBUG_PRINT("could not open irq port\r\n");
        }
        
        /* Initialize Board Control device */
        if (0 != rlsInitBoardControlPort(pDevCtx))
        {
            InitOk = FALSE;
            DEBUG_PRINT("could not open Board Control port\r\n");
        }
        
        /* Initialize Generic GPIO device */
        if (0 != rlsInitGenericGPIOPort(pDevCtx))
        {
            InitOk = FALSE;
            DEBUG_PRINT("could not open GPIO port\r\n");
        }

        /* Check for Errors, Return */
        if(!InitOk)
        {
            rlsClearDeviceCtx(pDevCtx);
            return NULL;
        } 
        else
        {
			DEBUG_PRINT("FTDI Open Successful\n");
            /* FTDI Open Successful, Return Handle */
            return (rlsDevHandle_t) pDevCtx;
        }
    }

	return NULL;
}

/** @fn int rlsCommClose(rlsDevHandle_t hdl)
*
*   @brief This function closes all the FTDI ports
*   @param[in] hdl - device handle for which the ports needs to be closed
*
*   @return int RLS_RET_CODE_OK
*/
int rlsCommClose( rlsDevHandle_t hdl )
{
    rlsDevCtx_t*   pDevCtx = (rlsDevCtx_t*) hdl;

    /* Close FTDI Device */
    rlsCloseDevice(pDevCtx) ;
    return RLS_RET_CODE_OK;
}

/** @fn int rlsSpiRead(rlsDevHandle_t hdl, unsigned char *preadbuffer,
*                      unsigned short ReadLength)
*
*   @brief This function reads the specified number of bytes over SPI interface
*   @param[in] hdl - device handle for which the data needs to be read
*   @param[out] preadbuffer - pointer to the Destination buffer
*   @param[in] ReadLength - number of bytes to be read
*
*   @return int Success - number of bytes read
*               Failure - RLS_RET_CODE_COMM_TIMEOUT, RLS_RET_CODE_COMM_RD_ERROR
*                         RLS_RET_CODE_COMM_WR_ERROR
*/
int rlsSpiRead( rlsDevHandle_t hdl, unsigned char *preadbuffer,
                unsigned short ReadLength )
{
#define COMMAND_LENGTH 10
    rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
    int             retVal = 0;
	int             spiWordSize = 2;
	DWORD           bytesSent = 0;
	int             bytesReceived = 0;
	int             dataAvailable = 0;
	char            buffer[COMMAND_LENGTH];
	DWORD           tempByteRecv = 0;
	FT_STATUS       status = FT_OK;
	unsigned int    i = 0;
	unsigned int    timeoutCnt = 0;
	unsigned char   tempData = 0;
	
	/* Read Remaining Length from device */
	rlsLockDevice(pDevCtx);

	for (i = 0; i < ReadLength; i += spiWordSize)
	{
		buffer[0] = 0x80;              /* Set Data bits LowByte */
		buffer[1] = 0xC2;              /* Set CS Low, MOSI high */
		buffer[2] = RLS_PORTA_CONFIG;  /* Set Pin Direction */

		if (rls_spiPhase == 0)
		{
			buffer[3] = 0x20;          /* Read bytes on +ve (posedge), MSB first, no write */
		}
		else
		{
			buffer[3] = 0x24;          /* Read bytes on -ve (negedge), MSB first, no write */
		}

		buffer[4] = (unsigned char)((spiWordSize - 1) & 0xFF);          /* Set Length LSB */

		buffer[5] = (unsigned char)(((spiWordSize - 1) >> 8) & 0xFF); /* Set Length MSB */

		buffer[6] = 0x80;              /* Set Data bits LowByte  */
		buffer[7] = 0xC8;              /* Set CS High, MOSI Low */
		buffer[8] = RLS_PORTA_CONFIG;  /* Set Pin Direction */
		buffer[9] = 0x87;               /* Send Immediate */

		/* Write to FTDI MPSSE */
		status = FT_Write(pDevCtx->spiHandle, &buffer[0], COMMAND_LENGTH,
			(LPDWORD)&bytesSent);

		if (FT_OK != status)
		{
			DEBUG_PRINT("Read error, couldn't send command to MSPEE \n");
			return RLS_RET_CODE_COMM_WR_ERROR;
		}

		do
		{
			/* Check if data is received */
			status = FT_GetQueueStatus(pDevCtx->spiHandle,
                                       (DWORD *)&dataAvailable);
			timeoutCnt++;
		} while ((FT_OK == status) && (dataAvailable < spiWordSize) &&
			(timeoutCnt < RLS_FTDI_RESPONSE_TIMEOUT));

		if (FT_OK != status)
		{
			DEBUG_PRINT("Wait Data Available Error\n");
			return RLS_RET_CODE_COMM_RD_ERROR;
		}
		/*
		else if (RLS_FTDI_RESPONSE_TIMEOUT == timeoutCnt)
		{
			DEBUG_PRINT("Time Out During SPI Read\n");
			return RLS_RET_CODE_COMM_TIMEOUT;
		}
		*/
		/* Read from FTDI MPSSE */
		status = FT_Read(pDevCtx->spiHandle, (preadbuffer + i),
			(DWORD)spiWordSize, (LPDWORD)&tempByteRecv);
			tempData = *(preadbuffer + i);
			*(preadbuffer + i) = *(preadbuffer + i + 1);
			*(preadbuffer + i + 1) = tempData;

			bytesReceived += 2;
	}
	rlsUnlockDevice(pDevCtx);

	/* Create data buffer and print it into file */
	char tempBuff[2048] = {0};
	int offset = 0;
	for (i = 0; i < ReadLength; i += 2)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X%02X ", preadbuffer[i + 1],
                    preadbuffer[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [RD]%s", pDevCtx->deviceIndex, tempBuff);

	if ((FT_OK != status) || (bytesReceived != ReadLength))
	{
		DEBUG_PRINT("Read transfer error \n");
		return RLS_RET_CODE_COMM_RD_ERROR;
	}

	return (int)bytesReceived;
}

/** @fn int rlsSpiWrite(rlsDevHandle_t hdl, unsigned char *pwriteBuffer,
*                       unsigned short WriteLength)
*
*   @brief This function writes the specified number of bytes over SPI interface
*   @param[in] hdl - device handle for which the data needs to be written
*   @param[out] pwriteBuffer - pointer to the Source buffer
*   @param[in] WriteLength - number of bytes to be written
*
*   @return int Success - number of bytes written
*               Failure - RLS_RET_CODE_COMM_WR_ERROR
*/
int rlsSpiWrite( rlsDevHandle_t hdl, unsigned char *pwriteBuffer,
                 unsigned short WriteLength ) 
{
    rlsDevCtx_t     *pDevCtx = (rlsDevCtx_t*)hdl;
    char*           pbuffer = NULL;
	char            buffer[12];
    DWORD           bytesSent = 0;
	DWORD			tempBytesSent = 0;
    FT_STATUS       status = FT_OK;
    unsigned short   i = 0;

#ifdef RLS_SPI_SIMULATION
#else

    rlsLockDevice(pDevCtx);

    /* Set Data bits LowByte */
	buffer[0] = 0x80;
    /* Set CS low */
	buffer[1] = 0xC0;
    /* Set Pin Direction */
	buffer[2] = RLS_PORTA_CONFIG;
    if(rls_spiPhase == 0)
    {
        /* Read bytes on +ve (posedge), MSB first, no write */
		buffer[3] = 0x11;
    }
    else
    {
        /* Read bytes on -ve (negedge), MSB first, no write */
		buffer[3] = 0x10;
    }
    /*Clock bytes on -ve (negedge), MSB first, no read */
	buffer[3] = 0x11;
    /* Write Length MSB */
	buffer[4] = (unsigned char) ((1) & 0xFF);
    /* Write Length LSB */
	buffer[5] = (unsigned char) (((1)>>8) & 0xFF);
    //memcpy(&pbuffer[6], Bytebuffer+i, 2);
    /* Set Data bits LSB */
	buffer[2+6] = 0x80;
    /* Set CS High */
	buffer[2+7] = 0xC8;
    /* Set Pin Direction */
	buffer[2+8] = RLS_PORTA_CONFIG;
    /* Send Immediate */
	buffer[2+9] = 0x87;

    for(i =0; i< WriteLength; i+=2)
    {
        /* Write Data Bits */
		buffer[6] = *(pwriteBuffer+i+1);
		buffer[7] = *(pwriteBuffer+i);

        /* Write to FTDI MPSSE */
        status = FT_Write(pDevCtx->spiHandle,
						buffer,
                        12, 
                        &tempBytesSent);
		bytesSent += tempBytesSent;
    }
    rlsUnlockDevice(pDevCtx);
#endif
	/* Create data buffer and print it into file */
	char tempBuff[2048] = {0};
	int offset = 0;
	for (i = 0; i < WriteLength; i += 2)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X%02X ", pwriteBuffer[i + 1],
                    pwriteBuffer[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [WR]%s", pDevCtx->deviceIndex, tempBuff);
    
    if (status != FT_OK)
    {
        DEBUG_PRINT("MSPEE Write Command Error\n");
        return RLS_RET_CODE_COMM_WR_ERROR;
    }    
    return (int)(WriteLength);
}

/** @fn int rlsI2cRead(rlsDevHandle_t hdl, unsigned char *preadbuffer,
*                      unsigned short ReadLength)
*
*   @brief This function reads the specified number of bytes over I2C interface
*   @param[in] hdl - device handle for which the data needs to be read
*   @param[out] preadbuffer - pointeR to the Destination buffer
*   @param[in] ReadLength - number of bytes to be read
*
*   @return int Success - number of bytes read
*               Failure - RLS_RET_CODE_COMM_TIMEOUT, RLS_RET_CODE_COMM_RD_ERROR
*                         RLS_RET_CODE_COMM_WR_ERROR
*/
int rlsI2cRead( rlsDevHandle_t hdl, unsigned char *preadbuffer,
                unsigned short ReadLength )
{
	rlsDevCtx_t* pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS ftStatus;
	int status = 0;
	unsigned char i = 0;
	BYTE i2cBuffer[100];
	DWORD i2cBytesToTransfer = 0;
	DWORD i2cBytesTransfered = 0;
	unsigned char i2cSlaveAddress;
	int slaveAddrStatus = 0;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	/* wait for Port B to be free */
    #ifdef _WIN32
        WaitForSingleObject(rls_gIrqI2cMutex, INFINITE);
    #elif __linux__
        rls_gIrqI2cMutex.lock();
    #endif

	/*Purge USB receive buffer first before read operation*/
	/*Get the number of bytes in the device receive buffer*/
	ftStatus = FT_GetQueueStatus(pDevCtx->irqI2cHandle, &i2cBytesToTransfer);
	if ((ftStatus == FT_OK) && (i2cBytesToTransfer > 0))
	{
		/*Read out all the data from receive buffer*/
		FT_Read(pDevCtx->irqI2cHandle, i2cBuffer, i2cBytesToTransfer,
                &i2cBytesTransfered);
	}

	ftStatus = FT_OK;

	/*Set START condition for I2C communication*/
	rlsHighSpeedSetI2CStart(i2cBuffer, &i2cBytesToTransfer);

	/*Setting Bit 0 to one to enable read*/
	i2cSlaveAddress = (i2cAddr[pDevCtx->deviceIndex] << 1) | 0x01;

	/*Send slave address for read and check ACK bit*/
	status = rlsI2CSendByteAndCheckACK(pDevCtx->irqI2cHandle, i2cBuffer,
                                       &i2cBytesToTransfer, &i2cBytesTransfered,
                                       i2cSlaveAddress);

	slaveAddrStatus = status;

	for (i = 0; i < ReadLength - 1; i++)
	{
		/* Reading the data */
		status = rlsI2CReadByteAndSendACK(pDevCtx->irqI2cHandle, i2cBuffer, \
			&i2cBytesToTransfer, &i2cBytesTransfered, &preadbuffer[i]);
	}
	/* Reading Last data */
	status = rlsI2CReadByteAndSendNACK(pDevCtx->irqI2cHandle, i2cBuffer, \
		&i2cBytesToTransfer, &i2cBytesTransfered, &preadbuffer[i]);

	/*Set STOP condition for I2C communication*/
	rlsHighSpeedSetI2CStop(i2cBuffer, &i2cBytesToTransfer);

	/*Send off the commands*/
	ftStatus |= FT_Write(pDevCtx->irqI2cHandle, i2cBuffer, i2cBytesToTransfer, \
		&i2cBytesTransfered);
	i2cBytesToTransfer = 0;    /*Clear output buffer*/

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (i = 0; i < ReadLength; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", preadbuffer[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [RD]%s", pDevCtx->deviceIndex, tempBuff);

	Sleep(1);                 /*Delay for a while to ensure read is completed*/

	/* Signal that the port B can be used*/
	#ifdef _WIN32
        ReleaseMutex(rls_gIrqI2cMutex);
    #elif __linux__
        rls_gIrqI2cMutex.unlock();
    #endif

	if (status == 1)
		status = ReadLength;
	else
		status = 0;

	return status;
}

/** @fn int rlsI2cWrite(rlsDevHandle_t hdl, unsigned char *pwriteBuffer,
*                       unsigned short WriteLength)
*
*   @brief This function writes the specified number of bytes over I2C interface
*   @param[in] hdl - device handle for which the data needs to be written
*   @param[out] pwriteBuffer - pointer to the Source buffer
*   @param[in] WriteLength - number of bytes to be written
*
*   @return int Success - number of bytes written
*               Failure - RLS_RET_CODE_COMM_WR_ERROR
*/
int rlsI2cWrite( rlsDevHandle_t hdl, unsigned char *pwriteBuffer,
                 unsigned short WriteLength )
{
	rlsDevCtx_t* pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS  ftStatus = FT_OK;
	int status = 0;
	unsigned char i = 0;
	BYTE i2cBuffer[100];
	DWORD i2cBytesToTransfer = 0;
	DWORD i2cBytesTransfered = 0;
	unsigned char i2cSlaveAddress;
	int slaveAddrStatus = 0;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	/* wait for Port B to be free */
    #ifdef _WIN32
        WaitForSingleObject(rls_gIrqI2cMutex, INFINITE);
    #elif __linux__
        rls_gIrqI2cMutex.lock();
    #endif
	/*Setting Bit 0 to zero to enable write*/
	i2cSlaveAddress = (i2cAddr[pDevCtx->deviceIndex] << 1) & 0xFE;

	/*Set START condition for I2C communication*/
	rlsHighSpeedSetI2CStart(i2cBuffer, &i2cBytesToTransfer);

	/*Send off the commands*/
	ftStatus |= FT_Write(pDevCtx->irqI2cHandle, i2cBuffer, i2cBytesToTransfer, \
		&i2cBytesTransfered);
	i2cBytesToTransfer = 0;    /*Clear output buffer*/

	/*Address of I2C slave for write*/
	status = rlsI2CSendByteAndCheckACK(pDevCtx->irqI2cHandle, i2cBuffer, \
		&i2cBytesToTransfer, &i2cBytesTransfered, \
		i2cSlaveAddress);

	slaveAddrStatus = status;

	for (i = 0; i < WriteLength; i++)
	{
		status = rlsI2CSendByteAndCheckACK(pDevCtx->irqI2cHandle, i2cBuffer, \
			&i2cBytesToTransfer, &i2cBytesTransfered, pwriteBuffer[i]);
	}

	if (status == 0)
	{
		DEBUG_PRINT("Error in I2C Write");
	}

	/*Set STOP condition for I2C communication*/
	rlsHighSpeedSetI2CStop(i2cBuffer, &i2cBytesToTransfer);

	/*Send off the commands*/
	ftStatus |= FT_Write(pDevCtx->irqI2cHandle, i2cBuffer, i2cBytesToTransfer, \
		&i2cBytesTransfered);
	i2cBytesToTransfer = 0;    /*Clear output buffer*/

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (i = 0; i < WriteLength; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pwriteBuffer[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [WR]%s", pDevCtx->deviceIndex, tempBuff);

	Sleep(1);                 /*Delay for a while to ensure read is completed*/

	/* Signal that the port B can be used*/
	#ifdef _WIN32
        ReleaseMutex(rls_gIrqI2cMutex);
    #elif __linux__
        rls_gIrqI2cMutex.unlock();
    #endif

	if (status == 1)
		status = WriteLength;
	else
		status = 0;

	return status;
}

rlsDevHandle_t rlsGetDeviceCtx( unsigned char deviceIndex )
{
	rlsDevCtx_t* pDevCtx = NULL;

	if (deviceIndex < RLS_NUM_CONNECTED_DEVICES_MAX)
	{
		pDevCtx = &(rls_devCtx[deviceIndex]);
		pDevCtx->deviceIndex = deviceIndex;
	}

	return (rlsDevHandle_t)pDevCtx;
}

int rlsGetNumofDevices( int *numOfDevices )
{
	FT_STATUS                   ftStatus;
#if CUSTOM_FTDI_NAMING
	FT_DEVICE_LIST_INFO_NODE*   pInfoList;
#endif
	DWORD                       numOfFTDIDevices;
#if CUSTOM_FTDI_NAMING
	DWORD                       loopCounter;
#endif
    #ifdef __linux__
    FT_SetVIDPID(0x0451,0xfd03);
    #endif
	ftStatus = FT_CreateDeviceInfoList(&numOfFTDIDevices);
	if (FT_OK != ftStatus)
	{
		DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
		return RLS_RET_CODE_DEVICE_NOT_FOUND;
	}
#if CUSTOM_FTDI_NAMING
	pInfoList = (FT_DEVICE_LIST_INFO_NODE*) \
                malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numOfFTDIDevices);
	if (NULL == pInfoList)
	{
		DEBUG_PRINT("malloc error during read device list\n");
		return RLS_RET_CODE_MALLOC_FAILED;
	}

	/* Get Device Information List */
	ftStatus = FT_GetDeviceInfoList(pInfoList, &numOfFTDIDevices);
	if (FT_OK != ftStatus)
	{
		DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
		free(pInfoList);
		return RLS_RET_CODE_COMM_RD_ERROR;
	}
#endif
	*numOfDevices = 0;
#if CUSTOM_FTDI_NAMING
	/* Search for SPI and IRQ device in the List */
	for (loopCounter = 0; loopCounter < numOfFTDIDevices; loopCounter++)
	{
		if ((memcmp(pInfoList[loopCounter].Description,
            &rls_mmWaveMotherBoard[0], strlen(rls_mmWaveMotherBoard)) == 0) ||
			(memcmp(pInfoList[loopCounter].Description,
            &rls_mmWaveDevPack[0], strlen(rls_mmWaveDevPack)) == 0))
		{
			(*numOfDevices)++;
		}
	}
#endif

#if CUSTOM_FTDI_NAMING
	*numOfDevices = *numOfDevices / 4;
#else
	*numOfDevices = numOfFTDIDevices / 4;
#endif

	return RLS_RET_CODE_OK;
}

/** @fn int rlsWarmReset(rlsDevHandle_t hdl, int state)
*
*   @brief This function enables and disables warm reset
*   @param[in] hdl - pointer to the handle
*   @param[in] state - high or low state of decides to enable or disable C7 pin.
*
*   @return int Success - 0, Failure - Error Code
*/
int rlsWarmReset( rlsDevHandle_t hdl, int state )
{
    rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
    FT_STATUS       ftStatus;
    unsigned char   buffer[16];
    DWORD           BytesToTransfer;
    DWORD           BytesTransfered;
    int             error = RLS_RET_CODE_OK;
    FT_HANDLE       Hdl = 0;

    if(pDevCtx == NULL)
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }

    Hdl = pDevCtx->boardControlHandle ;

    if(Hdl != 0)
    {
        BytesToTransfer = 1;

        /* Read Board Control Port(C)*/
        ftStatus = FT_GetBitMode( Hdl, &buffer[0] );
        if ((ftStatus != FT_OK))
        {
            DEBUG_PRINT("Error - Port C cannot be read\n");
            /* Reset the port*/
            FT_SetBitMode( Hdl, RLS_PORTC_CONFIG, FT_BITMODE_RESET );
            FT_Close( Hdl ) ;                           /* Close the USB port*/
            return RLS_RET_CODE_COMM_WR_ERROR ;         /* Exit with error*/
        }
    
        /* Warm reset value ie C7=1 */
        BytesToTransfer = 0;

        if (state == 1)
        {
            /*Setting C7 to one*/
            buffer[BytesToTransfer++] = \
            RLS_SET_BIT(buffer[0], RLS_PORTC_PIN7_WARMRST);
        }
        else
        {
            /* Setting C7 to 0 */
            buffer[BytesToTransfer++] = \
            RLS_CLR_BIT(buffer[0], RLS_PORTC_PIN7_WARMRST);  
        }
        
        /*Writing value to buffer*/
        ftStatus = FT_Write( Hdl, buffer, BytesToTransfer, &BytesTransfered );
                                                                           
        if (ftStatus != FT_OK)
        {
            DEBUG_PRINT("Error - Warm Reset- C7 pin cannot be written\n");
             /* Reset the port*/
            FT_SetBitMode( Hdl, RLS_PORTC_CONFIG, FT_BITMODE_RESET );
            FT_Close( Hdl ) ;                       /* Close the USB port*/
            return RLS_RET_CODE_COMM_WR_ERROR ;     /*Exit with error*/
        }
    }
    else
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }
   
    return RLS_RET_CODE_OK;
}

/** @fn int rlsSOPControl(rlsDevHandle_t hdl, int SOPmode)
*
*   @brief This function writes SOP mode to the device
*   @param[in] hdl - pointer to the handle
*   @param[in] SOPmode - drives device to specifed SOP mode .
*
*   @return int Success - 0, Failure - Error Code
*/
int rlsSOPControl( rlsDevHandle_t hdl, int SOPmode )
{
    rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
    FT_STATUS       ftStatus;
    unsigned char   buffer[16];
    DWORD           BytesToTransfer;
    DWORD           BytesTransfered;
    FT_HANDLE       Hdl;
    int             sopPin[3];
    int             error = RLS_RET_CODE_OK;

    Hdl = pDevCtx->genericGPIOHandle;
    sopPin[0] = RLS_PORTD_PIN2_PMIC_OUT_SOR1;
    sopPin[1] = RLS_PORTD_PIN3_SYNC_OUT_SOR2;
    sopPin[2] = RLS_PORTD_PIN4_TDO_SOR3;

    rls_spiPhase = 0;

    if(pDevCtx == NULL)
    {
        return RLS_RET_CODE_NULL_PTR_ERROR;
    }

    if(Hdl != 0)
    {
        /*Reading Port D*/
        ftStatus = FT_GetBitMode( Hdl, &buffer[0] );
        if (ftStatus != FT_OK)
        {
            DEBUG_PRINT("Error - Port D cannot be read\n");
            /* Reset the port*/
            FT_SetBitMode( Hdl, RLS_PORTD_CONFIG, FT_BITMODE_RESET );
            FT_Close( Hdl ) ;   /* Close the USB port*/
            return RLS_RET_CODE_COMM_WR_ERROR ;         /* Exit with error*/
        }

        /*Setting the SOP pins depending on SOPmode*/
        BytesToTransfer = 1;
        if( SOPmode == RLS_SOP_MODE_SCAN_APTG )             /*SOP MODE 1 SCAN/ATPG*/
        {
            RLS_CLR_BIT(buffer[0], sopPin[0]) ;             /*Setting D2 to 0*/
            RLS_SET_BIT(buffer[0], sopPin[1]) ;             /*Setting D3 to 1*/
            RLS_CLR_BIT(buffer[0], sopPin[2]) ;             /*Setting D4 to 0*/
        }
        else if( SOPmode == RLS_SOP_MODE_DEVELOPMENT )      /*SOP MODE 2 DEV/FLED/ORBIT*/
        {
            RLS_CLR_BIT(buffer[0], sopPin[0]) ;             /*Setting D2 to 0*/
            RLS_SET_BIT(buffer[0], sopPin[1]) ;             /*Setting D3 to 1*/
            RLS_SET_BIT(buffer[0], sopPin[2]) ;             /*Setting D4 to 1*/
        }
        else if( SOPmode == RLS_SOP_MODE_THB )              /*SOP MODE 3 THB*/
        {
            RLS_CLR_BIT(buffer[0], sopPin[0]) ;             /*Setting D2 to 0*/
            RLS_CLR_BIT(buffer[0], sopPin[1]) ;             /*Setting D3 to 0*/
            RLS_CLR_BIT(buffer[0], sopPin[2]) ;             /*Setting D4 to 0*/
        }
        else if( SOPmode == RLS_SOP_MODE_FUNCTIONAL)        /*SOP MODE 4 Functional Mode*/
        {
            RLS_CLR_BIT(buffer[0], sopPin[0]) ;             /*Setting D2 to 0*/
            RLS_CLR_BIT(buffer[0], sopPin[1]) ;             /*Setting D3 to 0*/
            RLS_SET_BIT(buffer[0], sopPin[2]) ;             /*Setting D4 to 1*/
        }
        else if ( SOPmode == RLS_SOP_MODE_FLASH_DOWNLOAD)   /*SOP MODE 5 Device Management Mode*/
        {
            RLS_SET_BIT(buffer[0], sopPin[0]) ;             /*Setting D2 to 1*/
            RLS_CLR_BIT(buffer[0], sopPin[1]) ;             /*Setting D3 to 0*/
            RLS_SET_BIT(buffer[0], sopPin[2]) ;             /*Setting D4 to 1*/
        }
        else if ( SOPmode == RLS_SOP_MODE_FUNCTIONAL_CPHA1) /*SOP MODE 6 Functional SPI CPHA1 mode*/
        {
            RLS_SET_BIT(buffer[0], sopPin[0]) ;             /*Setting D2 to 1*/
            RLS_SET_BIT(buffer[0], sopPin[1]) ;             /*Setting D3 to 1*/
            RLS_CLR_BIT(buffer[0], sopPin[2]) ;             /*Setting D4 to 0*/

            rls_spiPhase = 1;
        }
		else if (SOPmode == RLS_SOP_MODE_FUNCTIONAL_I2C) /*SOP MODE 7 Functional I2C mode*/
		{
			RLS_SET_BIT(buffer[0], sopPin[0]);             /*Setting D2 to 1*/
			RLS_SET_BIT(buffer[0], sopPin[1]);             /*Setting D3 to 1*/
			RLS_SET_BIT(buffer[0], sopPin[2]);             /*Setting D4 to 1*/
		}
        else
        {
            DEBUG_PRINT("Wrong SOP state is given");
            return -1;
        }
        /*Writing value to buffer*/
        ftStatus = FT_Write( Hdl, buffer, BytesToTransfer, &BytesTransfered );
        if (ftStatus != FT_OK)
        {
            DEBUG_PRINT("Error - Port D cannot be read\n");
            /* Reset the port*/
            FT_SetBitMode( Hdl, RLS_PORTD_CONFIG, FT_BITMODE_RESET );
            FT_Close( Hdl ) ;                       /* Close the USB port*/
            return RLS_RET_CODE_COMM_WR_ERROR ;     /* Exit with error*/
        }
    }
    return RLS_RET_CODE_OK;
}

/** @fn int rlsFullReset(rlsDevHandle_t hdl, int state)
*
*   @brief This function enables and disables full reset
*   @param[in] hdl - pointer to the handle
*   @param[in] state - high or low state of decides to enable or disable C6 pin.
*
*   @return int Success - 0, Failure - Error Code
*/
int rlsFullReset(rlsDevHandle_t hdl, int state )
{
	rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS       ftStatus;
	unsigned char   buffer[16];
	DWORD           BytesToTransfer;
	DWORD           BytesTransfered;
	FT_HANDLE       Hdl;
	unsigned char   portConfig;
	int             pinnum;
	int             error = RLS_RET_CODE_OK;

	Hdl = pDevCtx->boardControlHandle;
	portConfig = RLS_PORTC_CONFIG;
	pinnum = RLS_PORTC_PIN6_12XX_1_2_NRST;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if ((Hdl != 0))
	{
		BytesToTransfer = 1;

		/* Read Board Control Port(C)*/
		ftStatus = FT_GetBitMode(Hdl, &buffer[0]);
		if ((ftStatus != FT_OK))
		{
			DEBUG_PRINT("Error - Port C cannot be read\n");
			/* Reset the port*/
			FT_SetBitMode(Hdl, portConfig, FT_BITMODE_RESET);
			FT_Close(Hdl);   /* Close the USB port*/
			return RLS_RET_CODE_COMM_WR_ERROR;         /* Exit with error */
		}

		/*Full reset value ie C6=1*/
		BytesToTransfer = 0;

		if (state == 1)
		{
			/* Setting C6 to one */
			buffer[BytesToTransfer++] = RLS_SET_BIT(buffer[0], pinnum);
		}
		else
		{
			/*Setting C6 to 0*/
			buffer[BytesToTransfer++] = RLS_CLR_BIT(buffer[0], pinnum);
		}

		/*Writing value to buffer*/
		ftStatus = FT_Write(Hdl, buffer, BytesToTransfer, &BytesTransfered);
		if (ftStatus != FT_OK)
		{
			DEBUG_PRINT("Error - Warm Reset- C6 pin cannot be written\n");
			/* Reset port*/
			FT_SetBitMode(Hdl, portConfig, FT_BITMODE_RESET);
			FT_Close(Hdl);   /* Close port*/
			return RLS_RET_CODE_COMM_WR_ERROR;         /* Exit */
		}
	}

	else
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	/*wait*/
	Sleep(2);

	return RLS_RET_CODE_OK;
}

/** @fn int rlsOpenBoardControlIf(rlsDevHandle_t hdl)
*
*   @brief This function Opens Board Control Port(C)
*   @param[in] hdl - pointer to the handle
*
*   @return int 0- connection Open Successful
*              -ve -Connection Failed
*/
int rlsOpenBoardControlIf(rlsDevHandle_t  hdl )
{
	rlsDevCtx_t*                pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS                   ftStatus;
	DWORD                       numOfDevices;
	FT_DEVICE_LIST_INFO_NODE*   pInfoList;
#if CUSTOM_FTDI_NAMING
	DWORD                       loopCounter;
#endif
	int                         boardControlDevNumber = 2;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if (pDevCtx->boardControlHandle == 0)
	{
		/* Create Device Information List */
        #ifdef __linux__
        FT_SetVIDPID(0x0451,0xfd03);
        #endif
		ftStatus = FT_CreateDeviceInfoList(&numOfDevices);
		if (FT_OK != ftStatus)
		{
			DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
			return RLS_RET_CODE_DEVICE_NOT_FOUND;
		}

		pInfoList = (FT_DEVICE_LIST_INFO_NODE*)
                    malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numOfDevices);
		if (NULL == pInfoList)
		{
			DEBUG_PRINT("malloc error during read device list\n");
			return RLS_RET_CODE_MALLOC_FAILED;
		}

		/* Get Device Information List */
		ftStatus = FT_GetDeviceInfoList(pInfoList, &numOfDevices);
		if (FT_OK != ftStatus)
		{
			DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
			free(pInfoList);
			return RLS_RET_CODE_COMM_RD_ERROR;
		}

		DEBUG_PRINT("rlsOpenBoardControlIf: Got %d devices connected \n",
                    numOfDevices);
#if CUSTOM_FTDI_NAMING
		boardControlDevNumber = -1;

		/* Search for SPI and IRQ device in the List */
		for (loopCounter = 0; loopCounter < numOfDevices; loopCounter++)
		{
            std::cout << pInfoList[loopCounter].Description << std::endl;
			if ((memcmp(rls_BoardControlDeviceName[pDevCtx->deviceIndex], \
                pInfoList[loopCounter].Description, \
				strlen(rls_BoardControlDeviceName[pDevCtx->deviceIndex])) == 0) \
                || \
				(memcmp(rls_BoardControlDeviceNameDevPack[pDevCtx->deviceIndex],
                pInfoList[loopCounter].Description, \
				strlen(rls_BoardControlDeviceNameDevPack[pDevCtx->deviceIndex]))== 0))
			{
				/* Found Board Control FTDI Device */
				if ((pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0)
				{
					boardControlDevNumber = loopCounter;
				}
			}
		}
#endif
		free(pInfoList);
#if CUSTOM_FTDI_NAMING
		if (boardControlDevNumber == -1)
		{
            DEBUG_PRINT("Error while opening the board control\n");
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
#endif
		ftStatus = FT_Open(boardControlDevNumber, &pDevCtx->boardControlHandle);
		if (ftStatus != FT_OK)
		{
			DEBUG_PRINT("Board Control port open error\n");
			pDevCtx->boardControlHandle = 0;
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}

		/* Initialize Board Control device */
		if (0 != rlsInitBoardControlPort(pDevCtx))
		{
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
	}

	return RLS_RET_CODE_OK;
}

/** @fn int rlsCloseBoardControlIf(rlsDevHandle_t hdl)
*
*   @brief This function Closes Board Control Port(D)
*   @param[in] hdl - pointer to the handle
*
*   @return int 0- connection Close Successful
*              -ve -Failure
*/
int rlsCloseBoardControlIf(rlsDevHandle_t  hdl )
{
	FT_STATUS       ftStatus;
	rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
	int             error = RLS_RET_CODE_OK;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if ((pDevCtx->boardControlHandle != 0))
	{
		ftStatus = FT_SetBitMode(pDevCtx->boardControlHandle, 0x0,
                                 FT_BITMODE_RESET);
		ftStatus = FT_ResetPort(pDevCtx->boardControlHandle);
		ftStatus = FT_Close(pDevCtx->boardControlHandle);
		pDevCtx->boardControlHandle = 0;
	}

	return RLS_RET_CODE_OK;
}

/** @fn int rlsOpenGenericGpioIf(rlsDevHandle_t hdl)
*
*   @brief This function Opens GPIO Control Port(D)
*   @param[in] hdl - pointer to the handle
*
*   @return int 0- connection Open Successful
*              -ve -Connection Failed
*/
int rlsOpenGenericGpioIf(rlsDevHandle_t  hdl )
{
	rlsDevCtx_t*                pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS                   ftStatus;
	DWORD                       numOfDevices;
	FT_DEVICE_LIST_INFO_NODE*   pInfoList;
#if CUSTOM_FTDI_NAMING
	DWORD                       loopCounter;
#endif
	int                         genericGPIODevNumber = 3;
	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if ((pDevCtx->genericGPIOHandle == 0))
	{
		/* Create Device Information List */
        #ifdef __linux__
        FT_SetVIDPID(0x0451,0xfd03);
        #endif
		ftStatus = FT_CreateDeviceInfoList(&numOfDevices);
		if (FT_OK != ftStatus)
		{
			DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
			return RLS_RET_CODE_DEVICE_NOT_FOUND;
		}

		pInfoList = (FT_DEVICE_LIST_INFO_NODE*)
                    malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numOfDevices);
		if (NULL == pInfoList)
		{
			DEBUG_PRINT("malloc error during read device list\n");
			return RLS_RET_CODE_MALLOC_FAILED;
		}

		/* Get Device Information List */
		ftStatus = FT_GetDeviceInfoList(pInfoList, &numOfDevices);
		if (FT_OK != ftStatus)
		{
			DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
			free(pInfoList);
			return RLS_RET_CODE_COMM_RD_ERROR;
		}

		DEBUG_PRINT("rlsOpenGenericGpioIf: Got %d devices connected \n",
                    numOfDevices);
#if CUSTOM_FTDI_NAMING
		genericGPIODevNumber = -1;

		/* Search for SPI and IRQ device in the List */
		for (loopCounter = 0; loopCounter < numOfDevices; loopCounter++)
		{
			if ((memcmp(rls_GenericGPIODeviceName[pDevCtx->deviceIndex], \
                pInfoList[loopCounter].Description, \
				strlen(rls_GenericGPIODeviceName[pDevCtx->deviceIndex])) == 0) || \
				(memcmp(rls_GenericGPIODeviceNameDevPack[pDevCtx->deviceIndex],
                pInfoList[loopCounter].Description, \
                strlen(rls_GenericGPIODeviceNameDevPack[pDevCtx->deviceIndex])) == 0))
			{
				/* Found Generic GPIO Control FTDI Device */
				if ((pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0)
				{
					genericGPIODevNumber = loopCounter;
				}
			}
		}
#endif
		free(pInfoList);
#if CUSTOM_FTDI_NAMING
		if (genericGPIODevNumber == -1)
		{
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
#endif
		ftStatus = FT_Open(genericGPIODevNumber, &pDevCtx->genericGPIOHandle);
		if (ftStatus != FT_OK)
		{
			DEBUG_PRINT("GPIO Control port open error\n");
			pDevCtx->genericGPIOHandle = 0;
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
		/* Initialize Generic GPIO device */
		if (0 != rlsInitGenericGPIOPort(pDevCtx))
		{
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
	}

	return RLS_RET_CODE_OK;
}

/** @fn int rlsCloseGenericGpioIf(rlsDevHandle_t hdl)
*
*   @brief This function Closes GPIO Control Port(D)
*   @param[in] hdl - pointer to the handle
*
*   @return int 0- connection Close Successful
*              -ve -Failure
*/
int rlsCloseGenericGpioIf(rlsDevHandle_t  hdl )
{
	FT_STATUS       ftStatus;
	rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
	int             error = RLS_RET_CODE_OK;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if ((pDevCtx->genericGPIOHandle != 0))
	{
		ftStatus = FT_SetBitMode(pDevCtx->genericGPIOHandle, 0x0,
                                 FT_BITMODE_RESET);
		ftStatus = FT_ResetPort(pDevCtx->genericGPIOHandle);
		ftStatus = FT_Close(pDevCtx->genericGPIOHandle);
		pDevCtx->genericGPIOHandle = 0;
	}

	return RLS_RET_CODE_OK;
}

/** @fn int rlsOpenI2cIrqIf(rlsDevHandle_t hdl)
*
*   @brief This function Opens IRQ/I2C Control Port(D)
*   @param[in] hdl - pointer to the handle
*
*   @return int 0- connection Open Successful
*              -ve -Connection Failed
*/
int rlsOpenI2cIrqIf(rlsDevHandle_t  hdl )
{
	rlsDevCtx_t*                pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS                   ftStatus;
	DWORD                       numOfDevices;
	FT_DEVICE_LIST_INFO_NODE*   pInfoList;
#if CUSTOM_FTDI_NAMING
	DWORD                       loopCounter;
#endif
	int                         i2cIrqDevNumber = 1;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if ((pDevCtx->irqI2cHandle == 0))
	{
		/* Create Device Information List */
        #ifdef __linux__
        FT_SetVIDPID(0x0451,0xfd03);
        #endif
		ftStatus = FT_CreateDeviceInfoList(&numOfDevices);
		if (FT_OK != ftStatus)
		{
			DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
			return RLS_RET_CODE_DEVICE_NOT_FOUND;
		}

		pInfoList = (FT_DEVICE_LIST_INFO_NODE*)
                    malloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numOfDevices);
		if (NULL == pInfoList)
		{
			DEBUG_PRINT("malloc error during read device list\n");
			return RLS_RET_CODE_MALLOC_FAILED;
		}

		/* Get Device Information List */
		ftStatus = FT_GetDeviceInfoList(pInfoList, &numOfDevices);
		if (FT_OK != ftStatus)
		{
			DEBUG_PRINT("Error requesting dev list from D2XX driver \n");
			free(pInfoList);
			return RLS_RET_CODE_COMM_RD_ERROR;
		}

		DEBUG_PRINT("rlsOpenI2cIrqIf: Got %d devices connected\n",
                    numOfDevices);
#if CUSTOM_FTDI_NAMING
		i2cIrqDevNumber = -1;

		/* Search for SPI and IRQ device in the List */
		for (loopCounter = 0; loopCounter < numOfDevices; loopCounter++)
		{
			if ((memcmp(rls_irqI2CDeviceName[pDevCtx->deviceIndex], \
                pInfoList[loopCounter].Description, \
				strlen(rls_irqI2CDeviceName[pDevCtx->deviceIndex])) == 0) || \
				(memcmp(rls_irqI2CDeviceNameDevPack[pDevCtx->deviceIndex], \
                pInfoList[loopCounter].Description, \
                strlen(rls_irqI2CDeviceNameDevPack[pDevCtx->deviceIndex])) == 0))
			{
				/* Found Generic GPIO Control FTDI Device */
				if ((pInfoList[loopCounter].Flags & FT_FLAGS_OPENED) == 0)
				{
					i2cIrqDevNumber = loopCounter;
				}
			}
		}
#endif
		free(pInfoList);
#if CUSTOM_FTDI_NAMING
		if (i2cIrqDevNumber == -1)
		{
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
#endif
		ftStatus = FT_Open(i2cIrqDevNumber, &pDevCtx->irqI2cHandle);
		if (ftStatus != FT_OK)
		{
			DEBUG_PRINT("GPIO Control port open error\n");
			pDevCtx->irqI2cHandle = 0;
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
		/* Initialize Generic GPIO device */
		if (0 != rlsInitIrqI2CPort(pDevCtx))
		{
			return RLS_RET_CODE_COMM_OPEN_ERROR;
		}
	}

	return RLS_RET_CODE_OK;
}


/** @fn int rlsCloseI2cIrqIf(rlsDevHandle_t hdl)
*
*   @brief This function Closes I2C/IR Control Port(D)
*   @param[in] hdl - pointer to the handle
*
*   @return int 0- connection Close Successful
*              -ve -Failure
*/
int rlsCloseI2cIrqIf(rlsDevHandle_t  hdl )
{
	FT_STATUS       ftStatus;
	rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
	int             error = RLS_RET_CODE_OK;
	unsigned char   Continue = 0;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	if ((pDevCtx->irqI2cHandle != 0))
	{
		ftStatus = FT_SetBitMode(pDevCtx->irqI2cHandle, 0x0, FT_BITMODE_RESET);
		ftStatus = FT_ResetPort(pDevCtx->irqI2cHandle);
		ftStatus = FT_Close(pDevCtx->irqI2cHandle);
		pDevCtx->irqI2cHandle = 0;
	}

	return RLS_RET_CODE_OK;
}

/** @fn int rlsI2CWrite(rlsDevHandle_t hdl, unsigned char slaveAddress,
*                       unsigned char regAddress, unsigned char msbData,
*                       unsigned char lsbData, int datasize)
*
*   @brief This function writes to the slave through I2C protocol
*   @param[in] hdl - pointer to the handle
*   @param[in] slaveAddress - I2C address of the slave
*   @param[in] regAddress - Register address in the slave
*   @param[in] msbData -    MSB of the data read
*   @param[in] lsbData -    LSB of the data read
*   @param[in] datasize -   Number of bytes of Data to be written
*
*   @return int Success - 0, Failure - Error Code
*/
int rlsI2CWrite( rlsDevHandle_t hdl, unsigned char slaveAddress,
	unsigned char regAddress, unsigned char msbData,
	unsigned char lsbData, int datasize )
{
	rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS       ftStatus = FT_OK;
	int             status = FALSE;
	BYTE            i2cBuffer[100];
	DWORD           i2cBytesToTransfer = 0;
	DWORD           i2cBytesTransfered = 0;
	FT_HANDLE       Hdl = 0;
	int             error = RLS_RET_CODE_OK;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	Hdl = pDevCtx->irqI2cHandle;

	/*Setting Bit 0 to zero to enable write*/
	slaveAddress = (slaveAddress << 1) & 0xFE;

	/* wait for Port B to be free */
	#ifdef _WIN32
        WaitForSingleObject(rls_gIrqI2cMutex, INFINITE);
    #elif __linux__
        rls_gIrqI2cMutex.lock();
    #endif

	/*Set START condition for I2C communication*/
	rlsHighSpeedSetI2CStart(i2cBuffer, &i2cBytesToTransfer);

	if (datasize == 1)
	{
		/*Address of I2C slave for write*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, slaveAddress);
		/*Send address byte and check if ACK bit is received*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, regAddress);
		/*Send MSB byte and check if ACK bit is received*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, msbData);
	}

	if (datasize == 2)
	{
		/*Address of I2C slave for write*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, slaveAddress);
		/*Send address byte and check if ACK bit is received*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, regAddress);
		/*Send MSB byte and check if ACK bit is received*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, msbData);
		/*Send LSB byte and check if ACK  bit is received*/
		status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
                                           &i2cBytesTransfered, lsbData);
	}

	if (status == 1)
	{
		DEBUG_PRINT("Error in I2C Write\n");
	}

	/*Set STOP condition for I2C communication*/
	rlsHighSpeedSetI2CStop(i2cBuffer, &i2cBytesToTransfer);

	/*Send off the commands*/
	ftStatus |= FT_Write(Hdl, i2cBuffer, i2cBytesToTransfer,
                         &i2cBytesTransfered);
	i2cBytesToTransfer = 0;    /*Clear output buffer*/


	Sleep(1);                 /*Delay for a while to ensure write is completed*/

	/* Signal that the port B can be used*/
	#ifdef _WIN32
        ReleaseMutex(rls_gIrqI2cMutex);
    #elif __linux__
        rls_gIrqI2cMutex.unlock();
    #endif

	if (status == TRUE)
		status = 0;
	else
		status = 1;

	return status;
}

/** @fn int rlsI2CRead(rlsDevHandle_t hdl, unsigned char slaveAddress,
*                      unsigned char regAddress, unsigned char *msbData,
*                      unsigned char *lsbData, int datasize)
*
*   @brief This function writes to the salve through I2C protocol
*   @param[in] hdl - pointer to the handle
*   @param[in] slaveAddress - I2C address of the slave
*   @param[in] regAddress - Register address in the slave
*   @param[in] msbData -    MSB of the data written
*   @param[in] lsbData -    LSB of the data written
*   @param[in] datasize -   Number of bytes of Data to be read
*
*   @return int Success - 0, Failure - Error Code
*/
int rlsI2CRead( rlsDevHandle_t hdl, unsigned char slaveAddress,
	unsigned char regAddress, unsigned char *msbData,
	unsigned char *lsbData, int datasize )
{
	rlsDevCtx_t*    pDevCtx = (rlsDevCtx_t*)hdl;
	FT_STATUS       ftStatus;
	int             status;
	BYTE            i2cBuffer[100];
	DWORD           i2cBytesToTransfer = 0;
	DWORD           i2cBytesTransfered = 0;
	unsigned char   i2cSlaveAddress;
	FT_HANDLE       Hdl = 0;
	int             error = RLS_RET_CODE_OK;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_NULL_PTR_ERROR;
	}

	Hdl = pDevCtx->irqI2cHandle;

	/*Setting Bit 0 to zero to enable write*/
	i2cSlaveAddress = (slaveAddress << 1) & 0xFE;

	/* wait for Port B to be free */
	#ifdef _WIN32
        WaitForSingleObject(rls_gIrqI2cMutex, INFINITE);
    #elif __linux__
        rls_gIrqI2cMutex.lock();
    #endif

	/*Purge USB receive buffer first before read operation*/
	/*Get the number of bytes in the device receive buffer*/
	ftStatus = FT_GetQueueStatus(Hdl, &i2cBytesToTransfer);
	if ((ftStatus == FT_OK) && (i2cBytesToTransfer > 0))
	{
		/*Read out all the data from receive buffer*/
		FT_Read(Hdl, i2cBuffer, i2cBytesToTransfer, &i2cBytesTransfered);
	}

	ftStatus = FT_OK;
	/*Set START condition for I2C communication*/
	rlsHighSpeedSetI2CStart(i2cBuffer, &i2cBytesToTransfer);
	/*Send slave address for write and check if ACK bit is received.*/
	status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer, \
		&i2cBytesTransfered, i2cSlaveAddress);
	/*Send Register address byte and check if ACK bit is received*/
	status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer, \
		&i2cBytesTransfered, regAddress);

	/*Set START condition for I2C communication*/
	rlsHighSpeedSetI2CStart(i2cBuffer, &i2cBytesToTransfer);

	/*Setting Bit 0 to one to enable read*/
	i2cSlaveAddress = (slaveAddress << 1) | 0x01;
	/*Send slave address for read and check ACK bit*/
	status = rlsI2CSendByteAndCheckACK(Hdl, i2cBuffer, &i2cBytesToTransfer, \
		&i2cBytesTransfered, i2cSlaveAddress);

	if (datasize == 1)
	{
		status = rlsI2CReadByteAndSendNACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
			&i2cBytesTransfered, msbData);
	}

	if (datasize == 2)
	{
		/* Reading MSB data */
		status = rlsI2CReadByteAndSendACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
			&i2cBytesTransfered, msbData);
		/* Reading LSB data */
		status = rlsI2CReadByteAndSendNACK(Hdl, i2cBuffer, &i2cBytesToTransfer,\
			&i2cBytesTransfered, lsbData);
	}

	/*Set STOP condition for I2C communication*/
	rlsHighSpeedSetI2CStop(i2cBuffer, &i2cBytesToTransfer);

	/*Send off the commands*/
	ftStatus |= FT_Write(Hdl, i2cBuffer, i2cBytesToTransfer,
                         &i2cBytesTransfered);
	i2cBytesToTransfer = 0;    /*Clear output buffer*/

	Sleep(1);                 /*Delay for a while to ensure read is completed*/

	/* Signal that the port B can be used*/
	#ifdef _WIN32
        ReleaseMutex(rls_gIrqI2cMutex);
    #elif __linux__
        rls_gIrqI2cMutex.unlock();
    #endif

	if (status == TRUE)
		status = 0;
	else
		status = 1;

	return status;
}

/*
 * END OF mmwl_port_ftdi.c FILE
 */
