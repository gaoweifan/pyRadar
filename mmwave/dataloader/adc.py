# Copyright 2019 The OpenRadar Authors. All Rights Reserved.
# Copyright 2021 GaoWeifan. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import codecs
import socket
import struct
import json
import math
from enum import Enum

import numpy as np
import fpga_udp

class CMD(Enum):
    RESET_FPGA_CMD_CODE = '0100'
    RESET_AR_DEV_CMD_CODE = '0200'
    CONFIG_FPGA_GEN_CMD_CODE = '0300'
    CONFIG_EEPROM_CMD_CODE = '0400'
    RECORD_START_CMD_CODE = '0500'
    RECORD_STOP_CMD_CODE = '0600'
    PLAYBACK_START_CMD_CODE = '0700'
    PLAYBACK_STOP_CMD_CODE = '0800'
    SYSTEM_CONNECT_CMD_CODE = '0900'
    SYSTEM_ERROR_CMD_CODE = '0a00'
    CONFIG_PACKET_DATA_CMD_CODE = '0b00'
    CONFIG_DATA_MODE_AR_DEV_CMD_CODE = '0c00'
    INIT_FPGA_PLAYBACK_CMD_CODE = '0d00'
    READ_FPGA_VERSION_CMD_CODE = '0e00'

    def __str__(self):
        return str(self.value)


# Global variables hold test configurations
# Set one time per test by read_config() function when read configuration parameters from test profile file
# Used by parser processing 
CFG_PARAMS = {}
 
numOfChirps_buf = []
numLoops_buf = []
freqSlope_buf = []
numAdcSamples_buf = []
adcSampleRate_buf = []
profileIdx_buf = []
SigImgNumSlices_buf = []
RxSatNumSlices_buf = []
chanIdx_buf = []

lvdsCfg_headerEn_buf = []
lvdsCfg_dataFmt_buf = []
lvdsCfg_userBufEn_buf = []

Raw_file_numSubframes = 0
Raw_file_subframeIdx_buf = []
Raw_file_sessionFlag = ""

ADC_file_numSubframes = 0
ADC_file_subframeIdx_buf = []
ADC_file_sessionFlag = ""

CC9_file_numSubframes = 0
CC9_file_subframeIdx_buf = []
CC9_file_sessionFlag = "" 

# Definations for test pass/fail or not_apply/don't care
NOT_APPLY = -1
TC_PASS   =  0
TC_FAIL   =  1

# MESSAGE = codecs.decode(b'5aa509000000aaee', 'hex')
CONFIG_HEADER = '5aa5'
HEADER_Num = 0xa55a
CONFIG_STATUS = '0000'
STATUS_STR = {0:'success',1:'failed'}
CONFIG_FOOTER = 'aaee'
FOOTER_Num = 0xeeaa
ADC_PARAMS = {'chirps': 16,  # 32
              'rx': 4,
              'tx': 3,
              'samples': 256,
              'IQ': 2,
              'bytes': 2}
# STATIC
MAX_PACKET_SIZE = 4096
BYTES_IN_PACKET = 1456  # Data in payload per packet from FPGA
BYTES_OF_PACKET = 1466  # payload size per packet from FPGA
MAX_BYTES_PER_PACKET = 1470  # Maximum bytes in the data packet
FPGA_CLK_CONVERSION_FACTOR = 1000  # Record packet delay clock conversion factor
FPGA_CLK_PERIOD_IN_NANO_SEC = 8  # Record packet delay clock period in ns
VERSION_BITS_DECODE = 0x7F  # Version bits decode
VERSION_NUM_OF_BITS = 7  # Number of bits required for version
PLAYBACK_BIT_DECODE = 0x4000  # Playback FPGA bitfile identifier bit
# DYNAMIC
BYTES_IN_FRAME = (ADC_PARAMS['chirps'] * ADC_PARAMS['rx'] * ADC_PARAMS['tx'] *
                  ADC_PARAMS['IQ'] * ADC_PARAMS['samples'] * ADC_PARAMS['bytes'])
BYTES_IN_FRAME_CLIPPED = (BYTES_IN_FRAME // BYTES_IN_PACKET) * BYTES_IN_PACKET
PACKETS_IN_FRAME = BYTES_IN_FRAME / BYTES_IN_PACKET
PACKETS_IN_FRAME_CLIPPED = BYTES_IN_FRAME // BYTES_IN_PACKET
UINT16_IN_PACKET = BYTES_IN_PACKET // 2
UINT16_IN_FRAME = BYTES_IN_FRAME // 2


class DCA1000:
    """Software interface to the DCA1000 EVM board via ethernet.

    Attributes:
        static_ip (str): IP to receive data from the FPGA
        adc_ip (str): IP to send configuration commands to the FPGA
        data_port (int): Port that the FPGA is using to send data
        config_port (int): Port that the FPGA is using to read configuration commands from


    General steps are as follows:
        1. Power cycle DCA1000 and XWR1xxx sensor
        2. Open mmWaveStudio and setup normally until tab SensorConfig or use lua script
        3. Make sure to connect mmWaveStudio to the board via ethernet
        4. Start streaming data
        5. Read in frames using class

    Examples:
        >>> dca = DCA1000()
        >>> adc_data = dca.read(timeout=.1)
        >>> frame = dca.organize(adc_data, 128, 4, 256)

    """

    def __init__(self, static_ip='192.168.33.30', adc_ip='192.168.33.180',
                 data_port=4098, config_port=4096):
        # Save network data
        self.static_ip = static_ip
        self.adc_ip = adc_ip
        self.data_port = data_port
        self.config_port = config_port

        # Create configuration and data destinations
        self.cfg_dest = (adc_ip, config_port)
        self.cfg_recv = (static_ip, config_port)
        self.data_recv = (static_ip, data_port)

        # Create sockets
        self.config_socket = socket.socket(socket.AF_INET,
                                           socket.SOCK_DGRAM,
                                           socket.IPPROTO_UDP)
        self.data_socket = socket.socket(socket.AF_INET,
                                         socket.SOCK_DGRAM,
                                         socket.IPPROTO_UDP)

        # Bind data socket to fpga
        self.data_socket.bind(self.data_recv)

        # Bind config socket to fpga
        self.config_socket.bind(self.cfg_recv)

        self.data = []
        self.packet_count = []
        self.byte_count = []

        self.frame_buff = []

        self.curr_buff = None
        self.last_frame = None

        self.lost_packets = None

    def refresh_parameter(self):
        global BYTES_IN_FRAME
        global BYTES_IN_FRAME_CLIPPED
        global PACKETS_IN_FRAME
        global PACKETS_IN_FRAME_CLIPPED
        global UINT16_IN_PACKET
        global UINT16_IN_FRAME
        # DYNAMIC
        BYTES_IN_FRAME = (ADC_PARAMS['chirps'] * ADC_PARAMS['rx'] * ADC_PARAMS['tx'] *
                        ADC_PARAMS['IQ'] * ADC_PARAMS['samples'] * ADC_PARAMS['bytes'])
        BYTES_IN_FRAME_CLIPPED = (BYTES_IN_FRAME // BYTES_IN_PACKET) * BYTES_IN_PACKET
        PACKETS_IN_FRAME = BYTES_IN_FRAME / BYTES_IN_PACKET
        PACKETS_IN_FRAME_CLIPPED = BYTES_IN_FRAME // BYTES_IN_PACKET
        UINT16_IN_PACKET = BYTES_IN_PACKET // 2
        UINT16_IN_FRAME = BYTES_IN_FRAME // 2

    @staticmethod
    def AWR2243_read_config (config_file_name):
        global CFG_PARAMS
        global ADC_PARAMS
        config = open(config_file_name,'r')
        curProfileId=0
        chirpStartIdxFCF=0
        chirpEndIdxFCF=0
        loopCount=0
        chirpStartIdx_buf=[]
        chirpEndIdx_buf=[]
        profileIdCPCFG_buf=[]
        txEnable_buf=[]
        for line in config:
            #print("**** line from config file: \n" + line)
            List = line.replace(" ","").split("=")
                            
            if 'channelTx' == List[0]:
                CFG_PARAMS['txAntMask'] = int(List[1].split(";")[0])
                numTxChan = 0
                for chanIdx in range (3):
                    if (CFG_PARAMS['txAntMask'] >> chanIdx) & 1 == 1:
                        numTxChan = numTxChan + 1
                CFG_PARAMS['numTxChan'] = numTxChan
            if 'channelRx' == List[0]:
                CFG_PARAMS['rxAntMask'] = int(List[1].split(";")[0])
                numRxChan = 0
                for chanIdx in range (4):
                    if (CFG_PARAMS['rxAntMask'] >> chanIdx) & 1 == 1:
                        numRxChan = numRxChan + 1
                CFG_PARAMS['numRxChan'] = numRxChan
                ADC_PARAMS['rx'] = numRxChan
            if 'adcBitsD' == List[0]:
                ADC_PARAMS['bytes']=2
            if 'adcFmt' == List[0]:
                ADC_PARAMS['IQ']=2
            if 'dataRate' == List[0]:
                dataRate = int(List[1].split(";")[0])
                if dataRate==0b0001: # 0001b - 600 Mbps (DDR only)
                    CFG_PARAMS['lvdsBW']=600
                if dataRate==0b0010: # 0010b - 450 Mbps (SDR, DDR)
                    CFG_PARAMS['lvdsBW']=450
                if dataRate==0b0011: # 0011b - 400 Mbps (DDR only)
                    CFG_PARAMS['lvdsBW']=400
                if dataRate==0b0100: # 0100b - 300 Mbps (SDR, DDR)
                    CFG_PARAMS['lvdsBW']=300
                if dataRate==0b0101: # 0101b - 225 Mbps (DDR only)
                    CFG_PARAMS['lvdsBW']=225
                if dataRate==0b0110: # 0110b - 150 Mbps (DDR only)
                    CFG_PARAMS['lvdsBW']=150
            if 'laneEn' == List[0]:
                numlaneEn = 0
                for laneIdx in range (4):
                    if (int(List[1].split(";")[0]) >> laneIdx) & 1 == 1:
                        numlaneEn = numlaneEn + 1
                CFG_PARAMS['numlaneEn'] = numlaneEn
            
            # profile cfg
            if 'profileId' == List[0]:
                curProfileId = int(List[1].split(";")[0])
            if 'startFreqConst' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['startFreq'] = int(List[1].split(";")[0]) * (3.6 / (1<<26))
            if 'idleTimeConst' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['idleTime'] = int(List[1].split(";")[0])/100
            if 'adcStartTimeConst' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['adc_valid_start_time'] = int(List[1].split(";")[0])/100
            if 'rampEndTime' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['rampEndTime'] = int(List[1].split(";")[0])/100
            if 'freqSlopeConst' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['freq_slope'] = int(List[1].split(";")[0]) * (3.6e3 * 900 / (1<<26))
            if 'txStartTime' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['txStartTime'] = int(List[1].split(";")[0])/100
            if 'numAdcSamples' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['samples'] = int(List[1].split(";")[0])
            if 'digOutSampleRate' == List[0]:
                if curProfileId==0:#只取第一个profile
                    ADC_PARAMS['sample_rate'] = int(List[1].split(";")[0])
            if 'rxGain' == List[0]:
                curProfileId+=1 #避免后面frameCfg的numAdcSamples被当作profile的
            
            # chirp cfg
            if 'chirpStartIdx' == List[0]:
                chirpStartIdx_buf.append(int(List[1].split(";")[0]))
            if 'chirpEndIdx' == List[0]:
                chirpEndIdx_buf.append(int(List[1].split(";")[0]))
            if 'profileIdCPCFG' == List[0]:
                profileIdCPCFG_buf.append(int(List[1].split(";")[0]))
            if 'txEnable' == List[0]:
                txEnable_buf.append(int(List[1].split(";")[0]))

            # frame cfg
            if 'chirpStartIdxFCF' == List[0]:
                chirpStartIdxFCF = int(List[1].split(";")[0])
            if 'chirpEndIdxFCF' == List[0]:
                chirpEndIdxFCF = int(List[1].split(";")[0])
            if 'loopCount' == List[0]:
                loopCount = int(List[1].split(";")[0])
            if 'periodicity' == List[0]:
                ADC_PARAMS['frame_periodicity'] = int(List[1].split(";")[0])/200000
        
        ADC_PARAMS['tx'] = len(txEnable_buf)
        
        if ADC_PARAMS['tx'] > CFG_PARAMS['numTxChan']:
            raise ValueError("exceed max tx num, check channelTx and chirp cfg")
        tmp_chirpNum = chirpEndIdx_buf[0]-chirpStartIdx_buf[0]+1
        all_chirpNum = tmp_chirpNum
        for i in range(ADC_PARAMS['tx']-1):
            all_chirpNum += chirpEndIdx_buf[i+1]-chirpStartIdx_buf[i+1]+1
            if tmp_chirpNum != chirpEndIdx_buf[i+1]-chirpStartIdx_buf[i+1]+1:
                raise ValueError("AWR2243_read_config does not support different chirp number in different tx ant yet.")
        if all_chirpNum != chirpEndIdxFCF-chirpStartIdxFCF+1:
            raise ValueError("chirp number in frame cfg and all chirp cfgs mismatched")
        ADC_PARAMS['chirps'] = loopCount*tmp_chirpNum
        
        LVDSDataSizePerChirp = ADC_PARAMS['samples']*ADC_PARAMS['rx']*ADC_PARAMS['IQ']*ADC_PARAMS['bytes']+52
        LVDSDataSizePerChirp = math.ceil(LVDSDataSizePerChirp/256)*256
        maxSendBytesPerChirp = (ADC_PARAMS['idleTime']+ADC_PARAMS['rampEndTime'])*CFG_PARAMS['numlaneEn']*CFG_PARAMS['lvdsBW']/8
        return (LVDSDataSizePerChirp,maxSendBytesPerChirp,ADC_PARAMS,CFG_PARAMS)

    @staticmethod
    def read_config (config_file_name):
        """!
        This function read config from test profile file and fills up global variables to contain the configuration

            @param config_file_name : test config profile file name
            @return None
        """
        global CFG_PARAMS
        global ADC_PARAMS

        global numOfChirps_buf
        global numLoops_buf
        global numAdcSamples_buf
        global adcSampleRate_buf
        global freqSlope_buf
        global profileIdx_buf
        global SigImgNumSlices_buf
        global RxSatNumSlices_buf
        global chanIdx_buf

        global lvdsCfg_headerEn_buf
        global lvdsCfg_dataFmt_buf
        global lvdsCfg_userBufEn_buf

        global Raw_file_numSubframes
        global Raw_file_subframeIdx_buf
        global Raw_file_sessionFlag

        global ADC_file_numSubframes
        global ADC_file_subframeIdx_buf
        global ADC_file_sessionFlag

        global CC9_file_numSubframes
        global CC9_file_subframeIdx_buf
        global CC9_file_sessionFlag

        CFG_PARAMS = {}
        ADC_PARAMS = {}
        numOfChirps_buf = []
        numLoops_buf = []
        numAdcSamples_buf = []
        adcSampleRate_buf = []
        freqSlope_buf = []
        profileIdx_buf = []
        SigImgNumSlices_buf = []
        RxSatNumSlices_buf = []
        chanIdx_buf = []
        lvdsCfg_headerEn_buf = []
        lvdsCfg_dataFmt_buf = []
        lvdsCfg_userBufEn_buf = []
        Raw_file_subframeIdx_buf = []
        ADC_file_subframeIdx_buf = []
        CC9_file_subframeIdx_buf = []

        config = open(config_file_name,'r')

        for line in config:
            #print("**** line from config file: \n" + line)
            List = line.split()
                            
            if 'channelCfg' in line:
                CFG_PARAMS['rxAntMask'] = int(List[1])  
                CFG_PARAMS['txAntMask'] = int(List[2])  
            if 'adcCfg' in line:
                CFG_PARAMS['dataSize'] = int(List[1])  
                CFG_PARAMS['dataType'] = int(List[2])  
            if 'adcbufCfg' in line:
                CFG_PARAMS['chirpMode'] = int(List[5])
            if 'Platform' in line:
                if '14' in line:
                    CFG_PARAMS['platfrom'] = '14'
                if '16' in line:
                    CFG_PARAMS['platfrom'] = '16'
                if '18' in line:
                    CFG_PARAMS['platfrom'] = '18'
                if '64' in line:
                    CFG_PARAMS['platfrom'] = '64'
                if '68' in line:
                    CFG_PARAMS['platfrom'] = '68'

            if 'profileCfg' in line:
                profileIdx_buf.append(int(List[1]))
                ADC_PARAMS['startFreq']=int(List[2])
                freqSlope_buf.append(float(List[8])) 
                numAdcSamples_buf.append(int(List[10])) 
                adcSampleRate_buf.append(int(List[11])) 
                idleTime = float(List[3])
                rampEndTime = float(List[5])
                ADC_PARAMS['idleTime']=idleTime
                ADC_PARAMS['adc_valid_start_time']=float(List[4])
                ADC_PARAMS['rampEndTime']=rampEndTime
                ADC_PARAMS['freq_slope']=freqSlope_buf[0]
                ADC_PARAMS['txStartTime']=float(List[9])
                ADC_PARAMS['samples']=numAdcSamples_buf[0]
                ADC_PARAMS['sample_rate']=adcSampleRate_buf[0]
                            
            if 'frameCfg' in line:
                CFG_PARAMS['chirpStartIdx'] = int(List[1]) 
                CFG_PARAMS['chirpEndIdx'] = int(List[2])  
                numOfChirps_buf.append(CFG_PARAMS['chirpEndIdx'] - CFG_PARAMS['chirpStartIdx'] + 1)
                numLoops_buf.append(int(List[3]))
                CFG_PARAMS['numSubframes'] = 1 
                ADC_PARAMS['chirps']=numLoops_buf[0]
                ADC_PARAMS['frame_periodicity'] = float(List[5])
                        
            if 'advFrameCfg' in line:
                CFG_PARAMS['numSubframes'] = int(List[1])  
            if 'subFrameCfg' in line:
                numOfChirps_buf.append(int(List[4]))
                numLoops_buf.append(int(List[5]))
                            
            if 'lvdsStreamCfg' in line: 
                lvdsCfg_headerEn_buf.append(int(List[2]))
                lvdsCfg_dataFmt_buf.append(int(List[3]))
                lvdsCfg_userBufEn_buf.append(int(List[4]))

            if 'CQSigImgMonitor' in line: 
                SigImgNumSlices_buf.append(int(List[2]))

            if 'CQRxSatMonitor' in line: 
                RxSatNumSlices_buf.append(int(List[4]))
                            
        config.close()

        ####################################
        ######## Parser rxAnt config #######
        ####################################
        rxAntMask = CFG_PARAMS['rxAntMask']
                    
        rxChanEn = []
        rxChanEn.append(rxAntMask & 1)
        rxChanEn.append((rxAntMask >> 1) & 1)
        rxChanEn.append((rxAntMask >> 2) & 1)
        rxChanEn.append((rxAntMask >> 3) & 1)
        #print(rxChanEn) 

        txAntMask = CFG_PARAMS['txAntMask']
        # print(rxAntMask,txAntMask)
        txChanEn = []
        txChanEn.append(txAntMask & 1)
        txChanEn.append((txAntMask >> 1) & 1)
        txChanEn.append((txAntMask >> 2) & 1)
        #print(txChanEn) 

        numRxChan = 0
        chanIdx_buf = []
        for chanIdx in range (4):
            if rxChanEn[chanIdx] == 1:
                chanIdx_buf.append(chanIdx)
                numRxChan = numRxChan + 1
        CFG_PARAMS['numRxChan'] = numRxChan
        
        numTxChan = 0
        for chanIdx in range (3):
            if txChanEn[chanIdx] == 1:
                numTxChan = numTxChan + 1
        CFG_PARAMS['numTxChan'] = numTxChan
        ####################################
        ######## Parser lvds config ########
        ####################################
        Raw_file_numSubframes = 0
        Raw_file_subframeIdx_buf = []
        Raw_file_sessionFlag = ""

        ADC_file_numSubframes = 0
        ADC_file_subframeIdx_buf = []
        ADC_file_sessionFlag = ""

        CC9_file_numSubframes = 0
        CC9_file_subframeIdx_buf = []
        CC9_file_sessionFlag = ""

        # Based on the 1st subframe's lvdsStreamCfg CLI (headerEn, dataFmt and userBufEn)
    
        # if the 1st subframe has no header (headerEn = 0): 
        # > in this case, HW session ADC only (dataFmt = 1) and no SW session (userBufEn = 0) is the only valid configuration combination. 
        # > <prefix>_Raw_<x>.bin is generated to record HW session of the 1st subframe.
        # > in advanced subframe case, rest 3 subframes must have same lvdsStreamCfg as the 1st subframe, and record to <prefix>_Raw_<x>.bin as well. 

        # if the 1st subframe has header (headerEn = 1) and HW session is ADC only (dataFmt = 1) or CP+ADC+CQ (dataFmt = 4): 
        # > <prefix>_hdr_0ADC_<x>.bin is generated to record HW session of the 1st subframe. in advanced subframe case if any of rest 3 subfrmes has HW session, will be recorded to <prefix>_hdr_0ADC_<x>.bin as well. 
        # > <prefix>_hdr_0CC9_<x>.bin will be generated to record SW session if any subframes has SW session (userBufEn = 1). 
        
        # if the 1st subframe has header (headerEn = 1) and no HW session (dataFmt = 0): 
        # > in this case, the 1st subframe must have SW session (userBufEn = 1)
        # > <prefix>_hdr_0ADC_<x>.bin is generated to record SW session of the 1st subframe. In advanced subframe case if any of rest 3 subframes has SW session, will be recorded to <prefix>_hdr_0ADC_<x>.bin as well.  
        # > in advanced subframe case <prefix>_hdr_0CC9_<x>.bin will be generated to record HW session if any of rest 3 subframes has HW session (dataFmt = 1 or dataFmt = 4). 
        
        CFG_PARAMS['datacard_dataLoggingMode'] = "multi"
        if lvdsCfg_headerEn_buf[0] == 0:
            CFG_PARAMS['datacard_dataLoggingMode'] = "raw"

        if lvdsCfg_headerEn_buf[0] == 0:
            if lvdsCfg_dataFmt_buf[0] == 1 and lvdsCfg_userBufEn_buf[0] == 0:
                if CFG_PARAMS['datacard_dataLoggingMode'] == "raw":
                    # Raw file
                    Raw_file_numSubframes = Raw_file_numSubframes + 1
                    Raw_file_subframeIdx_buf.append(0)
                    Raw_file_sessionFlag = "HW"
                elif CFG_PARAMS['datacard_dataLoggingMode'] == "multi":
                    returen_value = TC_FAIL
                    print ("Error: no header can not be in multi mode!")
                else:
                    returen_value = TC_FAIL
                    print ("Error: Undefined CFG_PARAMS['datacard_dataLoggingMode']!") 
            else:
                returen_value = TC_FAIL
                print ("Error: Invalid lvdsStreamCfg")
        elif lvdsCfg_headerEn_buf[0] == 1:
            if lvdsCfg_dataFmt_buf[0] == 1 or lvdsCfg_dataFmt_buf[0] == 4: # 1:ADC 4:CP+ADC+CQ
                ADC_file_sessionFlag = "HW"
                CC9_file_sessionFlag = "SW"
                ADC_file_numSubframes = ADC_file_numSubframes + 1
                ADC_file_subframeIdx_buf.append(0) 
                if lvdsCfg_userBufEn_buf[0] == 1:
                    CC9_file_numSubframes = CC9_file_numSubframes + 1
                    CC9_file_subframeIdx_buf.append(0)    
            elif lvdsCfg_dataFmt_buf[0] == 0: #no ADC no HW
                ADC_file_sessionFlag = "SW"
                CC9_file_sessionFlag = "HW"
                if lvdsCfg_userBufEn_buf[0] == 1:
                    ADC_file_numSubframes = ADC_file_numSubframes + 1
                    ADC_file_subframeIdx_buf.append(0) 
                else:
                    returen_value = TC_FAIL
                    print ("Error: subframe 0 has no HW and SW")
            else:
                print ("subframe %d has a invalid dataFmt config" % subframeIdx)
        else:
            returen_value = TC_FAIL
            print ("Error: Invalid lvdsCfg_headerEn_buf[0]")
            
        # Rest of 3 subframes if advanced subframe case 
        for subframeIdx in range (1, CFG_PARAMS['numSubframes']):
            if lvdsCfg_dataFmt_buf[subframeIdx] == 1 or lvdsCfg_dataFmt_buf[subframeIdx] == 4: # 1:ADC 4:CP+ADC+CQ
                if ADC_file_sessionFlag == "HW":
                    ADC_file_numSubframes = ADC_file_numSubframes + 1
                    ADC_file_subframeIdx_buf.append(subframeIdx) 
                if CC9_file_sessionFlag == "HW":
                    CC9_file_numSubframes = CC9_file_numSubframes + 1
                    CC9_file_subframeIdx_buf.append(subframeIdx) 
            if lvdsCfg_userBufEn_buf[subframeIdx] == 1:
                if ADC_file_sessionFlag == "SW": 
                    ADC_file_numSubframes = ADC_file_numSubframes + 1
                    ADC_file_subframeIdx_buf.append(subframeIdx)         
                if CC9_file_sessionFlag == "SW":
                    CC9_file_numSubframes = CC9_file_numSubframes + 1
                    CC9_file_subframeIdx_buf.append(subframeIdx) 
        
        
        ADC_PARAMS['rx']=CFG_PARAMS['numRxChan']
        ADC_PARAMS['tx']=CFG_PARAMS['numTxChan']
        ADC_PARAMS['IQ']=2
        ADC_PARAMS['bytes']=2
        
        
        '''
        For HW data, the inter-chirp duration should be sufficient to stream out the desired amount of data. 
        For example, if the HW data-format is ADC and HSI header is enabled, then the total amount of data generated per chirp is:
        (numAdcSamples * numRxChannels * 4 (size of complex sample) + 52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] )
        rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.
        The chirp time Tc in us = idle time + ramp end time in the profile configuration. 
        For n-lane LVDS with each lane at a maximum of B Mbps,
        maximum number of bytes that can be send per chirp = Tc * n * B / 8 
        which should be greater than the total amount of data generated per chirp i.e
        Tc * n * B / 8 >= round-up(numAdcSamples * numRxChannels * 4 + 52, 256).
        E.g if n = 2, B = 600 Mbps, idle time = 7 us, ramp end time = 44 us, numAdcSamples = 512, numRxChannels = 4, 
        then 7650 >= 8448 is violated so this configuration will not work. 
        If the idle-time is doubled in the above example, then we have 8700 > 8448, so this configuration will work.

        For SW data, the number of bytes to transmit each sub-frame/frame is:
        52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] + sizeof(MmwDemo_LVDSUserDataHeader_t) [=8] +
        number of detected objects (Nd) * { sizeof(DPIF_PointCloudCartesian_t) [=16] + sizeof(DPIF_PointCloudSideInfo_t) [=4] }
        rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.
        or X = round-up(60 + Nd * 20, 256). So the time to transmit this data will be X * 8 / (n*B) us. 
        The maximum number of objects (Ndmax) that can be detected is defined in the DPC (DPC_OBJDET_MAX_NUM_OBJECTS). 
        So if Ndmax = 500, then time to transmit SW data is 68 us. 
        Because we parallelize this transmission with the much slower UART transmission, 
        and because UART transmission is also sending at least the same amount of information as the LVDS, 
        the LVDS transmission time will not add any burdens on the processing budget beyond the overhead of 
        reconfiguring and activating the CBUFF session (this overhead is likely bigger than the time to transmit).

        The total amount of data to be transmitted in a HW or SW packet must be greater than the minimum required by CBUFF, 
        which is 64 bytes or 32 CBUFF Units 
        (this is the definition CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS in the CBUFF driver implementation). 
        If this threshold condition is violated, 
        the CBUFF driver will return an error during configuration 
        and the demo will generate a fatal exception as a result. 
        When HSI header is enabled, the total transfer size is ensured to be at least 256 bytes, which satisfies the minimum. 
        If HSI header is disabled, for the HW session, this means that numAdcSamples * numRxChannels * 4 >= 64. 
        Although mmwavelink allows minimum number of ADC samples to be 2, the demo is supported for numAdcSamples >= 64. 
        So HSI header is not required to be enabled for HW only case. 
        But if SW session is enabled, without the HSI header, the bytes in each packet will be 8 + Nd * 20. 
        So for frames/sub-frames where Nd < 3, the demo will generate exception. 
        Therefore HSI header must be enabled if SW is enabled, this is checked in the CLI command validation.
        '''
        LVDSDataSizePerChirp = numAdcSamples_buf[0]*CFG_PARAMS['numRxChan']*ADC_PARAMS['IQ']*ADC_PARAMS['bytes']+52
        LVDSDataSizePerChirp = math.ceil(LVDSDataSizePerChirp/256)*256
        nlane = 2
        B = 600
        maxSendBytesPerChirp = (idleTime+rampEndTime)*nlane*B/8
        # print(ADC_PARAMS)
        return (LVDSDataSizePerChirp,maxSendBytesPerChirp,ADC_PARAMS,CFG_PARAMS)
        # print (CFG_PARAMS)
        # print (numOfChirps_buf)
        # print (numLoops_buf)
        # print (numAdcSamples_buf)
        # print (profileIdx_buf)
        # print (SigImgNumSlices_buf)
        # print (RxSatNumSlices_buf)
        # print (chanIdx_buf)

        # print (lvdsCfg_headerEn_buf)
        # print (lvdsCfg_dataFmt_buf)
        # print (lvdsCfg_userBufEn_buf)

        # print (Raw_file_numSubframes)
        # print (Raw_file_subframeIdx_buf)
        # print (Raw_file_sessionFlag)

        # print (ADC_file_numSubframes)
        # print (ADC_file_subframeIdx_buf)
        # print (ADC_file_sessionFlag)

        # print (CC9_file_numSubframes)
        # print (CC9_file_subframeIdx_buf)
        # print (CC9_file_sessionFlag)
        
    def __del__(self):
        #print("Stop stream: ",self.stream_stop())
        self.close()
        print("Close socket: success")

    def configure(self,FPGAjsonFilePath,IWRcfgFilePath):
        """Initializes and connects to the FPGA

        Returns:
            None

        """
        LVDSDataSizePerChirp_l,maxSendBytesPerChirp_l,ADC_PARAMS_l,CFG_PARAMS_l=self.read_config(IWRcfgFilePath)
        self.refresh_parameter()
        print(ADC_PARAMS_l)
        print(CFG_PARAMS_l)
        print("LVDSDataSizePerChirp:%d must <= maxSendBytesPerChirp:%d"%(LVDSDataSizePerChirp_l,maxSendBytesPerChirp_l))
        print("System connection check:",self.sys_alive_check())
        print(self.read_fpga_version())
        print("Config fpga:",self.config_fpga(FPGAjsonFilePath))
        print("Config record packet delay:",self.config_record(FPGAjsonFilePath))

    def close(self):
        """Closes the sockets that are used for receiving and sending data

        Returns:
            None

        """
        self.data_socket.close()
        self.config_socket.close()

    def read(self, timeout=1):
        """ Read in a single packet via UDP

        Args:
            timeout (float): Time to wait for packet before moving on

        Returns:
            Full frame as array if successful, else None

        """
        # Configure
        self.data_socket.settimeout(timeout)

        # Frame buffer
        ret_frame = np.zeros(UINT16_IN_FRAME, dtype=np.uint16)
        print(BYTES_IN_FRAME,BYTES_IN_FRAME_CLIPPED,BYTES_IN_PACKET)
        # Wait for start of next frame
        while True:
            packet_num, byte_count, packet_data = self._read_data_packet()
            if byte_count % BYTES_IN_FRAME_CLIPPED == 0:
                packets_read = 1
                ret_frame[0:UINT16_IN_PACKET] = packet_data
                break

        # Read in the rest of the frame            
        while True:
            packet_num, byte_count, packet_data = self._read_data_packet()
            packets_read += 1

            if byte_count % BYTES_IN_FRAME_CLIPPED == 0:
                self.lost_packets = PACKETS_IN_FRAME_CLIPPED - packets_read
                return ret_frame

            curr_idx = ((packet_num - 1) % PACKETS_IN_FRAME_CLIPPED)
            try:
                ret_frame[curr_idx * UINT16_IN_PACKET:(curr_idx + 1) * UINT16_IN_PACKET] = packet_data
            except:
                pass

            if packets_read > PACKETS_IN_FRAME_CLIPPED:
                packets_read = 0

    def postProcPacket(self,recvQueue,maxPacketNum):
        # global firstPacketNum
        # global receivedPacketNum
        data=recvQueue.pop(0)
        packet_data=np.frombuffer(data[10:], dtype=np.uint16)
        firstPacketNum = struct.unpack('<1L', data[:4])[0]
        print(f"First Packet ID - {firstPacketNum}")
        receivedPacketNum=[firstPacketNum]
        receivedData=np.zeros(UINT16_IN_PACKET*maxPacketNum, dtype=np.uint16)
        receivedData[0:UINT16_IN_PACKET] = packet_data
        while(recvQueue):
            data=recvQueue.pop(0)
            packet_data=np.frombuffer(data[10:], dtype=np.uint16)
            packetNum = struct.unpack('<1L', data[:4])[0]
            idx = packetNum-firstPacketNum
            if (idx+1>maxPacketNum or idx<1):
                continue
            receivedPacketNum.append(packetNum)
            receivedData[UINT16_IN_PACKET*idx:UINT16_IN_PACKET*(idx+1)]=packet_data
        print(f"Last Packet ID - {packetNum}")
        return receivedData,firstPacketNum,receivedPacketNum
    
    def fastRead_from0(self,numframes=1):
        packetNum = math.ceil(PACKETS_IN_FRAME*numframes)
        
        recvQueue=[]
        for i in range(packetNum):
            data,addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)
            recvQueue.append(data)
        
        print("all received, post processing packets...")
        receivedData,firstPacketNum,receivedPacketNum=self.postProcPacket(recvQueue,packetNum)
        databuf=receivedData[0:numframes*UINT16_IN_FRAME]
        print("received packet num:%d,expected packet num:%d"%(len(receivedPacketNum),packetNum))

        return databuf

    def fastRead_in_Cpp(self,numframes=1,timeOut=2,sortInC=True):
        packetNum = math.ceil(PACKETS_IN_FRAME*numframes)

        recvData = fpga_udp.read_data_udp(self.data_socket.fileno(),packetNum,BYTES_OF_PACKET,timeOut,sortInC)
        
        if sortInC: # sort packet using C code (True) or python (False)
            receivedPacketNum=fpga_udp.get_receivedPacketNum()
            print("received packet num:%d,expected packet num:%d,loss:%.2f%%"%(receivedPacketNum,packetNum,(packetNum-receivedPacketNum)/packetNum))
            return recvData
        else:
            print("all received, post processing packets...")

            recvData = np.reshape(recvData,(packetNum,BYTES_OF_PACKET))
            recvQueue = list(map(lambda x:bytes(x),recvData))
            
            receivedData,firstPacketNum,receivedPacketNum=self.postProcPacket(recvQueue,packetNum)
            databuf=receivedData[0:numframes*UINT16_IN_FRAME]
            print("received packet num:%d,expected packet num:%d,loss:%.2f%%"%(len(receivedPacketNum),packetNum,(packetNum-len(receivedPacketNum))/packetNum))

            return databuf

    def write_frames_to_file(self,filename="raw_data.bin",numframes=1):
        # data_buf = self.read()
        # for i in range(numframes-1):
        #     print("%d frame received"%(i+2))
        #     data_buf = np.append(data_buf,self.read())

        # data_buf = self.fastRead_from0(numframes)
        data_buf = self.fastRead_in_Cpp(numframes)
        data_buf.tofile(filename)
        print("file saved to",filename)

    def write_udpPakets_to_file(self,filename="upd_data.bin",numPakets=1):
        data, addr = self.data_socket.recvfrom(MAX_PACKET_SIZE)  # MAX_BYTES_PER_PACKET
        data_buf = np.frombuffer(data, dtype=np.uint16)
        for i in range(numPakets-1):
            print("%d udp received"%(i+2))
            data_buf = np.append(data_buf,self.data_socket.recvfrom(MAX_PACKET_SIZE))
        data_buf.tofile(filename)

    def write_payloadPakets_to_file(self,filename="pld_data.bin",numPakets=1):
        import time
        start=time.time()
        #packet_num, byte_count, packet_data = self._read_data_packet()
        #data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)
        #data_buf = packet_data
        #print("1 packets received")
        #print(packet_num, byte_count)
        for i in range(numPakets):
            #packet_num, byte_count, packet_data = self._read_data_packet()
            data,addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)
            #data_buf = np.append(data_buf,packet_data)
            #print("%d packets received"%(i+2))
            #print(packet_num, byte_count,packet_data.shape)
        end=time.time()
        print("%d packets received"%(i+1))
        #data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)  # MAX_PACKET_SIZE
        packet_num = struct.unpack('<1l', data[:4])[0]
        byte_count = struct.unpack('>Q', b'\x00\x00' + data[4:10][::-1])[0]
        print(packet_num, byte_count)
        print('Running time: %s Seconds'%(end-start),((byte_count/(end-start))*8)/(2**20))
        #data_buf.tofile(filename)

    def sys_alive_check(self):
        # SYSTEM_CONNECT_CMD_CODE
        # 5a a5 09 00 00 00 aa ee
        res = self._send_command(CMD.SYSTEM_CONNECT_CMD_CODE)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.SYSTEM_CONNECT_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def read_fpga_version(self):
        # READ_FPGA_VERSION_CMD_CODE
        # 5a a5 0e 00 00 00 aa ee
        res = self._send_command(CMD.READ_FPGA_VERSION_CMD_CODE)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.READ_FPGA_VERSION_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            verNum = res[2]
            MajorVersion = verNum & VERSION_BITS_DECODE
            MinorVersion = (verNum >> VERSION_NUM_OF_BITS) & VERSION_BITS_DECODE
            retStr = 'FPGA Version : {}.{} '.format(MajorVersion,MinorVersion)
            if(verNum & PLAYBACK_BIT_DECODE):
                retStr += '[Playback]'
            else:
                retStr += '[Record]'
            return retStr

    def config_fpga(self, filepath):
        # CONFIG_FPGA_GEN_CMD_CODE
        # 5a a5 03 00 06 00 01 02 01 02 03 1e aa ee
        ConfigLogMode = {'raw': '01', 'multi': '02'}
        ConfigLvdsMode = {1: '01', 2: '02'}
        ConfigTransferMode = {'LVDSCapture': '01', 'playback': '02'}
        ConfigCaptureMode = {'SDCardStorage': '01', 'ethernetStream': '02'}
        ConfigFormatMode = {1: '01', 2: '02', 3: '03'}
        ConfigTimer = '1e' # not support

        with open(filepath,'r',encoding='utf8')as fp:
            json_data = json.load(fp)
            payload = (ConfigLogMode[json_data['DCA1000Config']['dataLoggingMode']]+
                ConfigLvdsMode[json_data['DCA1000Config']['lvdsMode']]+
                ConfigTransferMode[json_data['DCA1000Config']['dataTransferMode']]+
                ConfigCaptureMode[json_data['DCA1000Config']['dataCaptureMode']]+
                ConfigFormatMode[json_data['DCA1000Config']['dataFormatMode']]+
                ConfigTimer)
        packLen=str(codecs.encode(struct.pack('<H',len(codecs.decode(payload,'hex'))),'hex'), encoding = "utf-8")
        res = self._send_command(CMD.CONFIG_FPGA_GEN_CMD_CODE, packLen, payload)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.CONFIG_FPGA_GEN_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def config_record(self, filepath):
        # CONFIG_PACKET_DATA_CMD_CODE 
        # 5a a5 0b 00 06 00 be 05 35 0c 00 00 aa ee
        bytesPerPacket = str(codecs.encode(struct.pack('<H',MAX_BYTES_PER_PACKET),'hex'), encoding = "utf-8")
        reserved = '0000'

        with open(filepath,'r',encoding='utf8')as fp:
            json_data = json.load(fp)
            recordDelay = str(codecs.encode(struct.pack('<H',int(
                            json_data['DCA1000Config']['packetDelay_us'] *
                            FPGA_CLK_CONVERSION_FACTOR /
                            FPGA_CLK_PERIOD_IN_NANO_SEC
                        )),'hex'), encoding = "utf-8")
            payload = (bytesPerPacket + recordDelay + reserved)
        packLen=str(codecs.encode(struct.pack('<H',len(codecs.decode(payload,'hex'))),'hex'), encoding = "utf-8")
        res = self._send_command(CMD.CONFIG_PACKET_DATA_CMD_CODE, packLen, payload)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.CONFIG_PACKET_DATA_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def _send_command(self, cmd, length='0000', body='', timeout=1):
        """Helper function to send a single commmand to the FPGA

        Args:
            cmd (CMD): Command code to send to the FPGA
            length (str): Length of the body of the command (if any)
            body (str): Body information of the command
            timeout (int): Time in seconds to wait for socket data until timeout

        Returns:
            str: Response message

        """
        # Create timeout exception
        self.config_socket.settimeout(timeout)

        # Create and send message
        resp = ''
        msg = codecs.decode(''.join((CONFIG_HEADER, str(cmd), length, body, CONFIG_FOOTER)), 'hex')
        try:
            self.config_socket.sendto(msg, self.cfg_dest)
            resp, addr = self.config_socket.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout as e:
            print(e)
        return resp

    def _read_data_packet(self):
        """Helper function to read in a single ADC packet via UDP

        Returns:
            int: Current packet number, byte count of data that has already been read, raw ADC data in current packet

        """
        data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)  # MAX_PACKET_SIZE
        packet_num = struct.unpack('<1l', data[:4])[0]
        byte_count = struct.unpack('>Q', b'\x00\x00' + data[4:10][::-1])[0]
        packet_data = np.frombuffer(data[10:], dtype=np.uint16)
        # packet_data = data[10:]
        return packet_num, byte_count, packet_data

    def _listen_for_error(self):
        """Helper function to try and read in for an error message from the FPGA

        Returns:
            None

        """
        self.config_socket.settimeout(None)
        msg = self.config_socket.recvfrom(MAX_PACKET_SIZE)
        if msg == b'5aa50a000300aaee':
            print('stopped:', msg)
    
    def reset_fpga(self):
        res = self._send_command(CMD.RESET_FPGA_CMD_CODE)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.RESET_FPGA_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def reset_radar(self):
        res = self._send_command(CMD.RESET_AR_DEV_CMD_CODE)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.RESET_AR_DEV_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def stream_start(self):
        res = self._send_command(CMD.RECORD_START_CMD_CODE)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.RECORD_START_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            print("Start streaming")
            return STATUS_STR[res[2]]

    def stream_stop(self):
        """Helper function to send the stop command to the FPGA

        Returns:
            str: Response Message

        """
        res = self._send_command(CMD.RECORD_STOP_CMD_CODE)
        if res :
            res = struct.unpack("<HHHH",res)
            if( res[0] != HEADER_Num or 
                res[1] != struct.unpack("<H",codecs.decode(str(CMD.RECORD_STOP_CMD_CODE),'hex'))[0] or 
                res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            self.data_socket.setblocking(True)
            print("Stop streaming")
            return STATUS_STR[res[2]]

    @staticmethod
    def organize(raw_frame, num_chirps, num_rx, num_samples, Qfirst=True):
        """Reorganizes raw ADC data into a full frame

        Args:
            raw_frame (ndarray): Data to format
            num_chirps: Number of chirps included in the frame
            num_rx: Number of receivers used in the frame
            num_samples: Number of ADC samples included in each chirp
            Qfirst: order of IQ in data file, True for Q first, False for I first
                    Note: by default, mmwave-sdk is Q first while mmwave-studio is I first.
                    Tips: you can check who is first by plotting time domain curves of both I and Q,
                          and observe their phase, Q is delayed by pi/2 phases from I.

        Returns:
            ndarray: Reformatted frame of raw data of shape (num_chirps, num_rx, num_samples)

        """
        ret = np.zeros(len(raw_frame) // 2, dtype=complex)

        # Separate IQ data
        if(Qfirst):
            ret[0::2] = 1j * raw_frame[0::4] + raw_frame[2::4]
            ret[1::2] = 1j * raw_frame[1::4] + raw_frame[3::4]
        else:
            ret[0::2] = raw_frame[0::4] + 1j * raw_frame[2::4]
            ret[1::2] = raw_frame[1::4] + 1j * raw_frame[3::4]
        return ret.reshape((num_chirps, num_rx, num_samples))
