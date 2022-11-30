# Copyright 2019 The OpenRadar Authors. All Rights Reserved.
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

from tabnanny import verbose
import numpy as np
import serial
import struct
import time
from multiprocessing import Process, Queue, Lock, Event
import fpga_udp
from mmwave.dataloader.parser_mmw_demo import parser_one_mmw_demo_output_packet
import math

MAGIC_WORD_ARRAY = np.array([2, 1, 4, 3, 6, 5, 8, 7])
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
MSG_DETECTED_POINTS = 1
MSG_RANGE_PROFILE = 2
MSG_NOISE_PROFILE = 3
MSG_AZIMUT_STATIC_HEAT_MAP = 4
MSG_POINT_CLOUD_2D = 6


class TI:
    """Software interface to a TI mmWave EVM for reading TLV format. Based on TI's SDKs

    Attributes:
        sdk_version: Version of the TI SDK the radar is using
        cli_port: Serial communication port of the configuration/user port
        data_port: Serial communication port of the data port
        num_rx_ant: Number of RX (receive) antennas being utilized by the radar
        num_tx_ant: Number of TX (transmit) antennas being utilized by the radar
        num_virtual_ant: Number of VX (virtual) antennas being utilized by the radar
        verbose: Optional output messages while parsing data
        connected: Optional attempt to connect to the radar during initialization
        mode: Demo mode to read different TLV formats

    """

    def __init__(self, sdk_version=3.0, cli_loc='/dev/ttyACM0', cli_baud=115200,
                 data_loc='/dev/ttyACM1', data_baud=921600, num_rx=4, num_tx=2,
                 verbose=False, connect=True, mode=0,config_file='./xwr18xx_profile.cfg'):
        super(TI, self).__init__()
        self.connected = False
        self.initial_frame_sent = False
        self.verbose = verbose
        self.mode = mode
        self.data_loc=data_loc
        self.data_baud=data_baud
        if connect:
            self.cli_port = serial.Serial(cli_loc, cli_baud)
            # self.data_port = serial.Serial(data_loc, data_baud,timeout=2)
            self.connected = True
        self.data_buf=[]
        self.data_process_handle=None
        self.sdk_version = sdk_version
        self.num_rx_ant = num_rx
        self.num_tx_ant = num_tx
        self.num_virtual_ant = num_rx * num_tx
        self.byteBuffer = np.zeros(1,dtype = 'uint8')
        self.byteBufferLength = 0
        if mode == 0:
            self._initialize(config_file)

    def __del__(self):
        print("Stop sensor, Close com port")
        self.close()

    def _configure_radar(self, config):
        # Configure IWR1842 by serial port
        for i in config:
            # Skip empty line
            if(i == ''):
                continue
            # Skip comment line
            if(i[0] == '%'):
                continue
            # Stop on sensorStart command
            if (i == 'sensorStart'):
                break
            self.cli_port.write((i+'\n').encode())
            if (self.verbose):
                print('>>> ' + i)
                time.sleep(0.01)
                print(self.cli_port.read(self.cli_port.in_waiting))
            time.sleep(0.01)
        self.initial_frame_sent = False

    def _initialize(self, config_file='./xwr18xx_profile.cfg'):
        self.config = [line.rstrip('\r\n') for line in open(config_file)]
        if self.connected:
            self._configure_radar(self.config)

        self.config_params = {}  # Initialize an empty dictionary to store the configuration parameters

        for i in self.config:

            # Split the line
            split_words = i.split(" ")

            # Get the information about the profile configuration
            if "profileCfg" in split_words[0]:
                start_freq = float(split_words[2])
                idle_time = float(split_words[3])
                ramp_end_time = float(split_words[5])
                freq_slope_const = float(split_words[8])
                self.num_adc_samples = int(split_words[10])
                num_adc_samples_round_to2 = 1

                while self.num_adc_samples > num_adc_samples_round_to2:
                    num_adc_samples_round_to2 = num_adc_samples_round_to2 * 2

                dig_out_sample_rate = int(split_words[11])

            # Get the information about the frame configuration    
            elif "frameCfg" in split_words[0]:

                self.chirp_start_idx = int(split_words[1])
                self.chirp_end_idx = int(split_words[2])
                self.num_loops = int(split_words[3])
                self.num_frames = int(split_words[4])
                self.frame_periodicity = float(split_words[5])
                self.trigger_select = int(split_words[6])
                self.trigger_delay = float(split_words[7])

            elif "channelCfg" in split_words[0]:
                self.num_rx_ant = bin(int(split_words[1])).count("1")
                self.num_tx_ant = bin(int(split_words[2])).count("1")
                self.num_virtual_ant = self.num_rx_ant * self.num_tx_ant
                

        # Combine the read data to obtain the configuration parameters
        num_chirps_per_frame = (self.chirp_end_idx - self.chirp_start_idx + 1) * self.num_loops
        self.config_params["numDopplerBins"] = num_chirps_per_frame // self.num_tx_ant
        self.config_params["numRangeBins"] = num_adc_samples_round_to2
        self.config_params["rangeResolutionMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (
                2 * freq_slope_const * 1e12 * self.num_adc_samples)
        self.config_params["rangeIdxToMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (
                2 * freq_slope_const * 1e12 * self.config_params["numRangeBins"])
        self.config_params["dopplerResolutionMps"] = 3e8 / (
                2 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * self.config_params[
                    "numDopplerBins"] * self.num_tx_ant)
        self.config_params["maxRange"] = (300 * 0.9 * dig_out_sample_rate) / (2 * freq_slope_const * 1e3)
        self.config_params["maxVelocity"] = 3e8 / (
                    4 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * self.num_tx_ant)
        self.config_params["sleepTime"] = 0.001*self.frame_periodicity
        self.config_params["num_rx_ant"] = self.num_rx_ant
        self.config_params["num_tx_ant"] = self.num_tx_ant

    def setFrameCfg(self,num_frames):
        self.num_frames=num_frames
        i=f'frameCfg {self.chirp_start_idx} {self.chirp_end_idx} {self.num_loops} {self.num_frames} {self.frame_periodicity} {self.trigger_select} {self.trigger_delay}'
        self.cli_port.write((i+'\n').encode())
        if (self.verbose):
            print('>>> ' + i)
            time.sleep(0.01)
            print(self.cli_port.read(self.cli_port.in_waiting))

    
    def startSensor(self):
        if self.initial_frame_sent:
            start_cmd = 'sensorStart 0'
        else:
            start_cmd = 'sensorStart'
        self.cli_port.write((start_cmd + '\n').encode())
        if (self.verbose):
            print('>>> ' + start_cmd)
            time.sleep(0.002)
            print(self.cli_port.read(self.cli_port.in_waiting))
        self.initial_frame_sent = True

    def stopSensor(self):
        self.cli_port.write(('sensorStop\n').encode())
        if (self.verbose):
            print('>>> sensorStop')
            time.sleep(0.1)
            print(self.cli_port.read(self.cli_port.in_waiting))

    def close(self):
        """End connection between radar and machine

        Returns:
            None

        """
        #self.cli_port.write('sensorStop\n'.encode())
        self.cli_port.close()
        # self.data_port.close()

    def _read_buffer(self):
        """

        Returns:

        """
        byte_buffer = self.data_port.read(self.data_port.in_waiting)
        return byte_buffer

    def _read_process(self,lock,event,data_buf,data_port,interval):
        with lock:
            while(event.is_set()):
                data_buf.put(data_port.read(data_port.in_waiting))
                time.sleep(interval) # Sampling frequency
            print("process exit event accepted")
            data_buf.put(data_port.read(data_port.in_waiting))


    def create_read_process(self,numframes=0):
        if(self.data_process_handle is None):
            # self.continue_lock=Lock()
            # self.continue_lock.acquire()

            # self.continue_event=Event()
            # self.continue_event.set()

            self.data_buf = Queue()

            # print("interval time(sec) between frames:",self.config_params["sleepTime"])
            # self.data_process_handle = Process(target=self._read_process,args=(
            #     self.continue_lock,
            #     self.continue_event,
            #     self.data_buf,
            #     self.data_port,
            #     self.config_params["sleepTime"]*0.4))
            
            # self.data_process_handle.start()
            BYTES_IN_PACKET = 1456  # Data in payload per packet from FPGA
            BYTES_IN_FRAME = (self.num_loops*self.num_virtual_ant*self.num_adc_samples*4)
            PACKETS_IN_FRAME = BYTES_IN_FRAME / BYTES_IN_PACKET
            packetNum = math.ceil(PACKETS_IN_FRAME*numframes)
            print(packetNum,BYTES_IN_PACKET)
            self.data_process_handle=fpga_udp.radar_start_read_thread(self.data_loc,self.data_baud,packetNum,BYTES_IN_PACKET)
            print("data reading process created")
        else:
            print("error: another data reading process is exist")

    def start_read_process(self):
        # self.continue_lock.release()
        fpga_udp.reset_radar_buf()
        print("data reading process started")

    def stop_read_process(self):
        if(self.data_process_handle is not None):
            # self.continue_event.clear()
            # print("waiting for data reading process to exit")
            # self.data_process_handle.join(2)
            # self.data_process_handle.terminate()
            self.byteBuffer=fpga_udp.get_radar_buf()
            self.byteBufferLength=len(self.byteBuffer)
            print("received serial data bytes:",self.byteBufferLength)
            fpga_udp.radar_stop_read_thread()
        self.data_process_handle = None
        print("data reading process exit")

    # Funtion to parse the incoming data
    def parseData18xx(self, readBuffer):
        # Constants
        OBJ_STRUCT_SIZE_BYTES = 12
        BYTE_VEC_ACC_MAX_SIZE = 2**15
        MMWDEMO_UART_MSG_DETECTED_POINTS = 1
        MMWDEMO_UART_MSG_RANGE_PROFILE   = 2
        MMWDEMO_OUTPUT_MSG_NOISE_PROFILE = 3
        MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP = 4
        MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
        maxBufferSize = 2**15
        tlvHeaderLengthInBytes = 8
        pointLengthInBytes = 16
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
        
        # Initialize variables
        magicOK = 0 # Checks if magic number has been read
        dataOK = 0 # Checks if the data has been read correctly
        frameNumber = 0
        detObj = {}
        

        byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
        byteCount = len(byteVec)
        
        # Check that the buffer is not full, and then add the data to the buffer
        if (self.byteBufferLength + byteCount) < maxBufferSize:
            self.byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
            self.byteBufferLength = self.byteBufferLength + byteCount
            
        # Check that the buffer has some data
        if self.byteBufferLength > 16:
            
            # Check for all possible locations of the magic word
            possibleLocs = np.where(self.byteBuffer == magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = self.byteBuffer[loc:loc+8]
                if np.all(check == magicWord):
                    startIdx.append(loc)
                
            # Check that startIdx is not empty
            if startIdx:
                
                # Remove the data before the first start index
                if startIdx[0] > 0 and startIdx[0] < self.byteBufferLength:
                    self.byteBuffer[:self.byteBufferLength-startIdx[0]] = self.byteBuffer[startIdx[0]:self.byteBufferLength]
                    self.byteBuffer[self.byteBufferLength-startIdx[0]:] = np.zeros(len(self.byteBuffer[self.byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                    self.byteBufferLength = self.byteBufferLength - startIdx[0]
                    
                # Check that there have no errors with the byte buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0
                    
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2**8, 2**16, 2**24]
                
                # Read the total packet length
                totalPacketLen = np.matmul(self.byteBuffer[12:12+4],word)
                
                # Check that all the packet has been read
                if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
                    magicOK = 1
        
        # If magicOK is equal to 1 then process the message
        if magicOK:
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Initialize the pointer index
            idX = 0
            
            # Read the header
            magicNumber = self.byteBuffer[idX:idX+8]
            idX += 8
            version = format(np.matmul(self.byteBuffer[idX:idX+4],word),'x')
            idX += 4
            totalPacketLen = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            platform = format(np.matmul(self.byteBuffer[idX:idX+4],word),'x')
            idX += 4
            frameNumber = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            timeCpuCycles = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numDetectedObj = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numTLVs = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            subFrameNumber = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4

            # Read the TLV messages
            for tlvIdx in range(numTLVs):
                
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2**8, 2**16, 2**24]

                # Check the header of the TLV message
                tlv_type = np.matmul(self.byteBuffer[idX:idX+4],word)
                idX += 4
                tlv_length = np.matmul(self.byteBuffer[idX:idX+4],word)
                idX += 4

                # Read the data depending on the TLV message
                if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                    # Initialize the arrays
                    x = np.zeros(numDetectedObj,dtype=np.float32)
                    y = np.zeros(numDetectedObj,dtype=np.float32)
                    z = np.zeros(numDetectedObj,dtype=np.float32)
                    velocity = np.zeros(numDetectedObj,dtype=np.float32)
                    
                    for objectNum in range(numDetectedObj):
                        
                        # Read the data for each object
                        x[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        y[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        z[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        velocity[objectNum] = self.byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                    
                    # Store the data in the detObj dictionary
                    detObj = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity":velocity}
                    dataOK = 1
                elif tlv_type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:

                    # Get the number of bytes to read
                    numBytes = 2*self.config_params["numRangeBins"]*self.config_params["numDopplerBins"]

                    # Convert the raw data to int16 array
                    payload = self.byteBuffer[idX:idX + numBytes]
                    idX += numBytes
                    rangeDoppler = payload.view(dtype=np.int16)

                    # Some frames have strange values, skip those frames
                    # TO DO: Find why those strange frames happen
                    if np.max(rangeDoppler) > 10000:
                        continue

                    # Convert the range doppler array to a matrix
                    rangeDoppler = np.reshape(rangeDoppler, (self.config_params["numDopplerBins"], self.config_params["numRangeBins"]),'F') #Fortran-like reshape
                    rangeDoppler = np.append(rangeDoppler[int(len(rangeDoppler)/2):], rangeDoppler[:int(len(rangeDoppler)/2)], axis=0)

                    # Generate the range and doppler arrays for the plot
                    rangeArray = np.array(range(self.config_params["numRangeBins"]))*self.config_params["rangeIdxToMeters"]
                    dopplerArray = np.multiply(np.arange(-self.config_params["numDopplerBins"]/2 , self.config_params["numDopplerBins"]/2), self.config_params["dopplerResolutionMps"])
                    
                    # plt.clf()
                    # cs = plt.contourf(rangeArray,dopplerArray,rangeDoppler)
                    # fig.colorbar(cs, shrink=0.9)
                    # fig.canvas.draw()
                    # plt.pause(0.1)

                    
    
            # Remove already processed data
            if idX > 0 and self.byteBufferLength>idX:
                shiftSize = totalPacketLen
                
                    
                self.byteBuffer[:self.byteBufferLength - shiftSize] = self.byteBuffer[shiftSize:self.byteBufferLength]
                self.byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(len(self.byteBuffer[self.byteBufferLength - shiftSize:]),dtype = 'uint8')
                self.byteBufferLength = self.byteBufferLength - shiftSize
                
                # Check that there are no errors with the buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0         

        return dataOK, frameNumber, detObj

    # post process UART data. if UARTbinData is None, self.byteBuffer(where received data stored) will be used.
    def post_process_data_buf(self,UARTbinData=None,verbose=False):
        print("post processing data...")
        # processed_frame=[]
        # while(not self.data_buf.empty()):
        #     dataOk, frameNumber, detObj = self.parseData18xx(self.data_buf.get())
        #     if dataOk:
        #         # Store the current frame into processed_frame
        #         processed_frame.append(detObj)
        #         data = np.array([detObj['x'], detObj['y'], detObj['z'], detObj['velocity']]).transpose(1, 0)
        #         print("frame No.:",frameNumber,"shape of detObj:", np.shape(data))
        
        
        # init local variables
        totalBytesParsed = 0
        numFramesParsed = 0
        if(UARTbinData is None):
            UARTbinData=bytes(self.byteBuffer)
        processed_frame=[]

        # parser_one_mmw_demo_output_packet extracts only one complete frame at a time
        # so call this in a loop till end of file
        while (totalBytesParsed < self.byteBufferLength):
            
            # parser_one_mmw_demo_output_packet function already prints the
            # parsed data to stdio. So showcasing only saving the data to arrays 
            # here for further custom processing
            parser_result, headerStartIndex,platform,frameNumber,timeCpuCycles, totalPacketNumBytes, \
            numDetObj, numTlv, subFrameNumber,  \
            detectedX_array, detectedY_array, detectedZ_array, detectedV_array,  \
            detectedRange_array, detectedAzimuth_array, detectedElevAngle_array,  \
            detectedSNR_array, detectedNoise_array,  \
            rangeProfile_array, noiseFloor_array,  \
            azimuthHeatMap_array,rangeDoppler_array,  \
            stats_array, temperature_stats_array = parser_one_mmw_demo_output_packet(
                UARTbinData[totalBytesParsed::1], 
                self.byteBufferLength-totalBytesParsed,
                self.config_params,
                verbose)
            
            # Check the parser result
            if(verbose):
                print ("Parser result: ", parser_result)
            if (parser_result == 0): 
                totalBytesParsed += (headerStartIndex+totalPacketNumBytes)    
                numFramesParsed+=1
                if(verbose):
                    print("alreadyParsedBytes: ", totalBytesParsed)
                
                retObj={
                        'parser_result':parser_result,
                        'headerStartIndex':headerStartIndex,
                        'platform':platform,
                        'frameNumber':frameNumber,
                        'timeCpuCycles':timeCpuCycles,
                        'totalPacketNumBytes':totalPacketNumBytes,
                        'numDetObj':numDetObj,
                        'numTlv':numTlv,
                        'subFrameNumber':subFrameNumber,
                        'detectedX_array':detectedX_array,
                        'detectedY_array':detectedY_array,
                        'detectedZ_array':detectedZ_array,
                        'detectedV_array':detectedV_array,
                        'detectedRange_array':detectedRange_array,
                        'detectedAzimuth_array':detectedAzimuth_array,
                        'detectedElevAngle_array':detectedElevAngle_array,
                        'detectedSNR_array':detectedSNR_array,
                        'detectedNoise_array':detectedNoise_array,
                        'rangeProfile_array':rangeProfile_array, 
                        'noiseFloor_array':noiseFloor_array,
                        'azimuthHeatMap_array':azimuthHeatMap_array,
                        'rangeDoppler_array':rangeDoppler_array,
                        'stats_array':stats_array, 
                        'temperature_stats_array':temperature_stats_array,
                        'config_params':self.config_params
                }
                processed_frame.append(retObj.copy())
            else: 
                # error in parsing
                if(totalPacketNumBytes==-1): # does not find the magic number i.e output packet header 
                    print(f"error: processing frame failed, abort")
                    break
                totalBytesParsed += (headerStartIndex+totalPacketNumBytes)   
                print(f"warning: processing frame {frameNumber} failed, ignoring")

        # All processing done; Exit
        print("totalBytesParsed: ", totalBytesParsed)
        print("numFramesParsed: ", numFramesParsed)
        return processed_frame
