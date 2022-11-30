# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions are
# *  met:
# *
# *    Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# *  PARTICULAR TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
# *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# *  EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# *  LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING
# *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *

# import the required Python packages
import struct
import math
import binascii
import codecs
import numpy as np

# definations for parser pass/fail
TC_PASS   =  0
TC_FAIL   =  1

# define Message types used in Millimeter Wave Demo
tlvMsgTypeDef={
    'MSG_DETECTED_POINTS':                  1,  # List of detected points.
    'MSG_RANGE_PROFILE':                    2,  # Range profile.
    'MSG_NOISE_PROFILE':                    3,  # Noise floor profile.
    'MSG_AZIMUT_STATIC_HEAT_MAP':           4,  # Samples to calculate static azimuth heatmap.
    'MSG_RANGE_DOPPLER_HEAT_MAP':           5,  # Range/Doppler detection matrix.
    'MSG_STATS':                            6,  # Stats information.
    'MSG_DETECTED_POINTS_SIDE_INFO':        7,  # List of detected points' side info.
    'MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP': 8,  # Samples to calculate static azimuth/elevation heatmap, (all virtual antennas exported) - unused in this demo.
    'MSG_TEMPERATURE_STATS':                9,  # temperature stats from Radar front end
    'MSG_MAX':                              10  # maximum number of tlv msg
}

def getUint32(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer.

        @param data : 1-demension byte array  
        @return     : 32-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getUint16(data):
    """!
       This function coverts 2 bytes to a 16-bit unsigned integer.

        @param data : 1-demension byte array
        @return     : 16-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256)

def getHex(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer in hex.

        @param data : 1-demension byte array
        @return     : 32-bit unsigned integer in hex
    """ 
    return (binascii.hexlify(data[::-1]))

def checkMagicPattern(data):
    """!
       This function check if data arrary contains the magic pattern which is the start of one mmw demo output packet.  

        @param data : 1-demension byte array
        @return     : 1 if magic pattern is found
                      0 if magic pattern is not found 
    """ 
    found = 0
    if (data[0] == 2 and data[1] == 1 and data[2] == 4 and data[3] == 3 and data[4] == 6 and data[5] == 5 and data[6] == 8 and data[7] == 7):
        found = 1
    return (found)
          
def parser_helper(data, readNumBytes, verbose=False):
    """!
       This function is called by parser_one_mmw_demo_output_packet() function or application to read the input buffer, find the magic number, header location, the length of frame, the number of detected object and the number of TLV contained in this mmw demo output packet.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
    """ 

    headerStartIndex = -1

    for index in range (readNumBytes):
        if checkMagicPattern(data[index:index+8:1]) == 1:
            headerStartIndex = index
            break
  
    if headerStartIndex == -1: # does not find the magic number i.e output packet header 
        totalPacketNumBytes = -1
        numDetObj           = -1
        numTlv              = -1
        subFrameNumber      = -1
        platform            = -1
        frameNumber         = -1
        timeCpuCycles       = -1
    else: # find the magic number i.e output packet header 
        totalPacketNumBytes = getUint32(data[headerStartIndex+12:headerStartIndex+16:1])
        platform            = getHex(data[headerStartIndex+16:headerStartIndex+20:1])
        frameNumber         = getUint32(data[headerStartIndex+20:headerStartIndex+24:1])
        timeCpuCycles       = getUint32(data[headerStartIndex+24:headerStartIndex+28:1])
        numDetObj           = getUint32(data[headerStartIndex+28:headerStartIndex+32:1])
        numTlv              = getUint32(data[headerStartIndex+32:headerStartIndex+36:1])
        subFrameNumber      = getUint32(data[headerStartIndex+36:headerStartIndex+40:1])
    if(verbose):
        print("headerStartIndex    = %d" % (headerStartIndex))
        print("totalPacketNumBytes = %d" % (totalPacketNumBytes))
        print("platform            = %s" % (platform)) 
        print("frameNumber         = %d" % (frameNumber)) 
        print("timeCpuCycles       = %d" % (timeCpuCycles))   
        print("numDetObj           = %d" % (numDetObj)) 
        print("subFrameNumber      = %d" % (subFrameNumber))   
        print("numTlv              = %d" % (numTlv))
                              
    return (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber,platform,frameNumber,timeCpuCycles)


def parser_one_mmw_demo_output_packet(data, readNumBytes, config_params, verbose=False):
    """!
       This function is called by application. Firstly it calls parser_helper() function to find the start location of the mmw demo output packet, then extract the contents from the output packet.
       Each invocation of this function handles only one frame at a time and user needs to manage looping around to parse data for multiple frames.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return result                : parser result. 0 pass otherwise fail
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
        @return detectedX_array       : 1-demension array holds each detected target's x of the mmw demo output packet
        @return detectedY_array       : 1-demension array holds each detected target's y of the mmw demo output packet
        @return detectedZ_array       : 1-demension array holds each detected target's z of the mmw demo output packet
        @return detectedV_array       : 1-demension array holds each detected target's v of the mmw demo output packet
        @return detectedRange_array   : 1-demension array holds each detected target's range profile of the mmw demo output packet
        @return detectedAzimuth_array : 1-demension array holds each detected target's azimuth of the mmw demo output packet
        @return detectedElevAngle_array : 1-demension array holds each detected target's elevAngle of the mmw demo output packet
        @return detectedSNR_array     : 1-demension array holds each detected target's snr of the mmw demo output packet
        @return detectedNoise_array   : 1-demension array holds each detected target's noise of the mmw demo output packet
    """

    headerNumBytes = 40   

    PI = 3.14159265

    detectedX_array = []
    detectedY_array = []
    detectedZ_array = []
    detectedV_array = []
    detectedRange_array = []
    detectedAzimuth_array = []
    detectedElevAngle_array = []
    detectedSNR_array = []
    detectedNoise_array = []

    rangeProfile_array = []
    noiseFloor_array = []
    azimuthHeatMap_array = []
    rangeDoppler_array = []

    stats_array=[]
    temperature_stats_array=[]
    

    result = TC_PASS

    # call parser_helper() function to find the output packet header start location and packet size 
    (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber,platform,frameNumber,timeCpuCycles) = parser_helper(data, readNumBytes, verbose=verbose)
                         
    if headerStartIndex == -1:
        result = TC_FAIL
        print("************ Frame Fail, cannot find the magic words *****************")
    else:
        nextHeaderStartIndex = headerStartIndex + totalPacketNumBytes 

        if headerStartIndex + totalPacketNumBytes > readNumBytes:
            result = TC_FAIL
            print("********** Frame Fail, readNumBytes may not long enough ***********")
        elif nextHeaderStartIndex + 8 < readNumBytes and checkMagicPattern(data[nextHeaderStartIndex:nextHeaderStartIndex+8:1]) == 0:
            result = TC_FAIL
            print("********** Frame Fail, incomplete packet **********") 
        elif numDetObj <= 0:
            result = TC_FAIL
            print("************ Frame Fail, numDetObj = %d *****************" % (numDetObj))
        elif subFrameNumber > 3:
            result = TC_FAIL
            print("************ Frame Fail, subFrameNumber = %d *****************" % (subFrameNumber))
        else: 
            # Read the TLV messages
            tlvStart = headerStartIndex + headerNumBytes
            for tlvIdx in range(numTlv):

                # Check the header of the TLV message
                tlvType    = getUint32(data[tlvStart+0:tlvStart+4:1])
                tlvLen     = getUint32(data[tlvStart+4:tlvStart+8:1])      
                offset = 8

                if(verbose):
                    print(f"TLV {tlvIdx}") 
                    print(f"    type {tlvType} - {list(tlvMsgTypeDef.keys())[tlvType-1]}")
                    print(f"    len {tlvLen} bytes")

                # Read the data depending on the TLV message
                if tlvType == tlvMsgTypeDef['MSG_DETECTED_POINTS']:

                    # TLV type 1 contains x, y, z, v values of all detect objects. 
                    # each x, y, z, v are 32-bit float in IEEE 754 single-precision binary floating-point format, so every 16 bytes represent x, y, z, v values of one detect objects.    
                    
                    # for each detect objects, extract/convert float x, y, z, v values and calculate range profile and azimuth                           
                    for obj in range(numDetObj):
                        # convert byte0 to byte3 to float x value
                        x = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset:tlvStart + offset+4:1]),'hex'))[0]

                        # convert byte4 to byte7 to float y value
                        y = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+4:tlvStart + offset+8:1]),'hex'))[0]

                        # convert byte8 to byte11 to float z value
                        z = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+8:tlvStart + offset+12:1]),'hex'))[0]

                        # convert byte12 to byte15 to float v value
                        v = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+12:tlvStart + offset+16:1]),'hex'))[0]

                        # calculate range profile from x, y, z
                        compDetectedRange = math.sqrt((x * x)+(y * y)+(z * z))

                        # calculate azimuth from x, y           
                        if y == 0:
                            if x >= 0:
                                detectedAzimuth = 90
                            else:
                                detectedAzimuth = -90 
                        else:
                            detectedAzimuth = math.atan(x/y) * 180 / PI

                        # calculate elevation angle from x, y, z
                        if x == 0 and y == 0:
                            if z >= 0:
                                detectedElevAngle = 90
                            else: 
                                detectedElevAngle = -90
                        else:
                            detectedElevAngle = math.atan(z/math.sqrt((x * x)+(y * y))) * 180 / PI
                                
                        detectedX_array.append(x)
                        detectedY_array.append(y)
                        detectedZ_array.append(z)
                        detectedV_array.append(v)
                        detectedRange_array.append(compDetectedRange)
                        detectedAzimuth_array.append(detectedAzimuth)
                        detectedElevAngle_array.append(detectedElevAngle)
                                                                    
                        offset = offset + 16
                elif tlvType == tlvMsgTypeDef['MSG_DETECTED_POINTS_SIDE_INFO']:
                    # TLV type 7 contains snr and noise of all detect objects.
                    # each snr and noise are 16-bit integer represented by 2 bytes, so every 4 bytes represent snr and noise of one detect objects.    
                
                    # for each detect objects, extract snr and noise                                            
                    for obj in range(numDetObj):
                        # byte0 and byte1 represent snr. convert 2 bytes to 16-bit integer
                        snr   = getUint16(data[tlvStart + offset + 0:tlvStart + offset + 2:1])
                        # byte2 and byte3 represent noise. convert 2 bytes to 16-bit integer 
                        noise = getUint16(data[tlvStart + offset + 2:tlvStart + offset + 4:1])

                        detectedSNR_array.append(snr)
                        detectedNoise_array.append(noise)
                                                                        
                        offset = offset + 4
                    if(verbose):
                        print("                  x(m)         y(m)         z(m)        v(m/s)    Com0range(m)  azimuth(deg)  elevAngle(deg)  snr(0.1dB)    noise(0.1dB)")
                        for obj in range(numDetObj):
                            print("    obj%3d: %12f %12f %12f %12f %12f %12f %12d %12d %12d" % (obj, detectedX_array[obj], detectedY_array[obj], detectedZ_array[obj], detectedV_array[obj], detectedRange_array[obj], detectedAzimuth_array[obj], detectedElevAngle_array[obj], detectedSNR_array[obj], detectedNoise_array[obj]))
                elif tlvType == tlvMsgTypeDef['MSG_RANGE_PROFILE']:
                    # Array of profile points at 0th Doppler (stationary objects). 
                    # The points represent the sum of log2 magnitudes of received antennas expressed in Q9 format.
                    # Length: (Range FFT size) x (size of uint16_t)
                    numRangeBins=int(tlvLen/2)
                    mag = []
                    for rangeBin in range(numRangeBins):
                        # byte0 and byte1 represent magnitude. convert 2 bytes to 16-bit integer
                        mag.append(getUint16(data[tlvStart + offset + 0:tlvStart + offset + 2:1]))
                        offset = offset + 2
                    mag = np.array(mag,dtype=np.uint16)
                    rangeProfile_array.append(mag)
                    if(verbose):
                        print(f"    RANGE_MAG shape: {mag.shape}")
                elif tlvType == tlvMsgTypeDef['MSG_NOISE_PROFILE']:
                    # This is the same format as range profile but the profile is at the maximum Doppler bin (maximum speed objects). 
                    # In general for stationary scene, there would be no objects or clutter at maximum speed 
                    # so the range profile at such speed represents the receiver noise floor.
                    # Length: (Range FFT size) x (size of uint16_t)
                    numRangeBins=int(tlvLen/2)
                    noiseFloor = []
                    for rangeBin in range(numRangeBins):
                        # byte0 and byte1 represent magnitude. convert 2 bytes to 16-bit integer
                        noiseFloor.append(getUint16(data[tlvStart + offset + 0:tlvStart + offset + 2:1]))
                        offset = offset + 2
                    noiseFloor = np.array(noiseFloor,dtype=np.uint16)
                    noiseFloor_array.append(noiseFloor)
                    if(verbose):
                        print(f"    noiseFloor shape: {noiseFloor.shape}")
                elif tlvType == tlvMsgTypeDef['MSG_AZIMUT_STATIC_HEAT_MAP']:
                    # Array DPU_AoAProcHWA_HW_Resources::azimuthStaticHeatMap. 
                    # The antenna data are complex symbols, with imaginary(int16_t) first and real(int16_t) second in the following order:

                    # Imag(ant 0, range 0  ), Real(ant 0, range 0  ),...,Imag(ant N-1, range 0  ),Real(ant N-1, range 0  )
                    # ...
                    # Imag(ant 0, range R-1), Real(ant 0, range R-1),...,Imag(ant N-1, range R-1),Real(ant N-1, range R-1)

                    # Based on this data the static azimuth heat map is constructed by the GUI running on the host.
                    # Length: (Range FFT size) x (Number of virtual antennas) (size of cmplx16ImRe_t_(2 x int16_t))
                    azimuthHeatMap = np.frombuffer(data[tlvStart + offset:tlvStart + offset + tlvLen:1],dtype=np.int16)
                    azimuthHeatMap = np.reshape(azimuthHeatMap,(config_params["numRangeBins"],8,2))  # rangeBin*VirAnt*IQ
                    azimuthHeatMap = (1j * azimuthHeatMap[:,:,0] + azimuthHeatMap[:,:,1]).astype(np.complex64)
                    azimuthHeatMap_array.append(azimuthHeatMap)
                    if(verbose):
                        print(f"    azimuthHeatMap shape: {azimuthHeatMap.shape}")
                elif tlvType == tlvMsgTypeDef['MSG_RANGE_DOPPLER_HEAT_MAP']:
                    # Detection matrix DPIF_DetMatrix::data. The order is :

                    # X(range bin 0  , Doppler bin 0),...,X(range bin 0  , Doppler bin D-1),
                    # ...
                    # X(range bin R-1, Doppler bin 0),...,X(range bin R-1, Doppler bin D-1)

                    # Length: (Range FFT size) x (Doppler FFT size) (size of uint16_t)

                    # Convert the raw data to uint16 array
                    rangeDoppler = np.frombuffer(data[tlvStart + offset:tlvStart + offset + tlvLen:1],dtype=np.uint16)

                    # Convert the range doppler array to a matrix
                    rangeDoppler = np.reshape(rangeDoppler, (config_params["numRangeBins"], config_params["numDopplerBins"]))
                    # rangeDoppler = np.append(rangeDoppler[int(len(rangeDoppler)/2):], rangeDoppler[:int(len(rangeDoppler)/2)], axis=0)
                    rangeDoppler_array.append(rangeDoppler)
                    if(verbose):
                        print(f"    rangeDopplerHeatMap shape: {rangeDoppler.shape}")

                    # Generate the range and doppler arrays for the plot
                    # rangeArray = np.array(range(self.config_params["numRangeBins"]))*self.config_params["rangeIdxToMeters"]
                    # dopplerArray = np.multiply(np.arange(-self.config_params["numDopplerBins"]/2 , self.config_params["numDopplerBins"]/2), self.config_params["dopplerResolutionMps"])
                    
                    # plt.clf()
                    # cs = plt.contourf(rangeArray,dopplerArray,rangeDoppler)
                    # fig.colorbar(cs, shrink=0.9)
                    # fig.canvas.draw()
                    # plt.pause(0.1)
                    pass
                elif tlvType == tlvMsgTypeDef['MSG_STATS']:
                    # Timing information as per MmwDemo_output_message_stats_t.
                    # Note:
                    #     1.The interChirpProcessingMargin is not computed (it is always set to 0). 
                    #       This is because there is no CPU involvement in the 1D processing (only HWA and EDMA are involved), 
                    #       and it is not possible to know how much margin is there in chirp processing without CPU being notified 
                    #       at every chirp when processing begins (chirp event) and when the HWA-EDMA computation ends. 
                    #       The CPU is intentionally kept free during 1D processing because a real application may use this time 
                    #       for doing some post-processing algorithm execution.
                    #     2.While the interFrameProcessingTime reported will be of the current sub-frame/frame, 
                    #       the interFrameProcessingMargin and transmitOutputTime will be of the previous sub-frame 
                    #       (of the same MmwDemo_output_message_header_t::subFrameNumber as that of the current sub-frame) 
                    #       or of the previous frame.
                    #     3.The interFrameProcessingMargin excludes the UART transmission time (available as transmitOutputTime). 
                    #       This is done intentionally to inform the user of a genuine inter-frame processing margin 
                    #       without being influenced by a slow transport like UART, this transport time can be significantly longer 
                    #       for example when streaming out debug information like heat maps. Also, in a real product deployment, 
                    #       higher speed interfaces (e.g LVDS) are likely to be used instead of UART. 
                    #       User can calculate the margin that includes transport overhead 
                    #       (say to determine the max frame rate that a particular demo configuration will allow) 
                    #       using the stats because they also contain the UART transmission time.
                    stats = {
                        'interFrameProcessingTime':-1,
                        'transmitOutputTime':-1,
                        'interFrameProcessingMargin':-1,
                        'interChirpProcessingMargin':-1,
                        'activeFrameCPULoad':-1,
                        'interFrameCPULoad':-1
                    }
                    # byte0 to byte3 represent interFrameProcessingTime. Interframe processing time in usec.
                    stats['interFrameProcessingTime'] = getUint32(data[tlvStart + offset + 0:tlvStart + offset + 4:1])
                    # byte4 to byte7 represent transmitOutputTime. Transmission time of output detection information in usec.
                    stats['transmitOutputTime'] = getUint32(data[tlvStart + offset + 4:tlvStart + offset + 8:1])
                    # byte8 to byte11 represent interFrameProcessingMargin. Interframe processing margin in usec.
                    stats['interFrameProcessingMargin'] = getUint32(data[tlvStart + offset + 8:tlvStart + offset + 12:1])
                    # byte12 to byte15 represent interChirpProcessingMargin. Interchirp processing margin in usec.
                    stats['interChirpProcessingMargin'] = getUint32(data[tlvStart + offset + 12:tlvStart + offset + 16:1])
                    # byte16 to byte19 represent activeFrameCPULoad. CPU Load (%) during active frame duration.
                    stats['activeFrameCPULoad'] = getUint32(data[tlvStart + offset + 16:tlvStart + offset + 20:1])
                    # byte20 to byte23 represent interFrameCPULoad. CPU Load (%) during inter frame duration. 
                    stats['interFrameCPULoad'] = getUint32(data[tlvStart + offset + 20:tlvStart + offset + 24:1])
                    stats_array.append(stats.copy())
                    if(verbose):
                        print(f"    interFrameProcessingTime {stats['interFrameProcessingTime']} us")
                        print(f"    transmitOutputTime {stats['transmitOutputTime']} us")
                        print(f"    interFrameProcessingMargin {stats['interFrameProcessingMargin']} us")
                        print(f"    interChirpProcessingMargin {stats['interChirpProcessingMargin']} us")
                        print(f"    activeFrameCPULoad {stats['activeFrameCPULoad']} %")
                        print(f"    interFrameCPULoad {stats['interFrameCPULoad']} %")
                elif tlvType == tlvMsgTypeDef['MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP']:
                    # Samples to calculate static azimuth/elevation heatmap, (all virtual antennas exported)
                    # unused in this demo.
                    if(verbose):
                        print(f"    azimuthElevationHeatMap -unused in this demo")
                elif tlvType == tlvMsgTypeDef['MSG_TEMPERATURE_STATS']:
                    # Structure of detailed temperature report as obtained from Radar front end. 
                    # tempReportValid is set to return value of rlRfGetTemperatureReport. 
                    # If tempReportValid is 0, values in temperatureReport are valid else they should be ignored. 
                    # This TLV is sent along with Stats TLV described in Stats information
                    temperature_stats={
                        'tempReportValid':-1,
                        
                    }
                    # byte0 to byte3 represent tempReportValid. Interframe processing time in usec.
                    temperature_stats['tempReportValid'] = struct.unpack('<i', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 4:1]),'hex'))[0]
                    offset+=4
                    
                    # radarSS local Time from device powerup. 1 LSB = 1 ms
                    temperature_stats['time'] = struct.unpack('<I', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 4:1]),'hex'))[0]
                    offset+=4
                    
                    # RX0 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpRx0Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # RX1 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpRx1Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # RX2 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpRx2Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # RX3 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpRx3Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # TX0 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpTx0Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # TX1 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpTx1Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # TX2 temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpTx2Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # PM temperature sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpPmSens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # Digital temp sensor reading (signed value). 1 LSB = 1 deg C
                    temperature_stats['tmpDig0Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]
                    offset+=2
                    
                    # Second digital temp sensor reading (signed value). 1 LSB = 1 deg C
                    # ( applicable only in xWR1642/xWR6843/xWR1843.) 
                    temperature_stats['tmpDig1Sens'] = struct.unpack('<h', codecs.decode(binascii.hexlify(data[tlvStart + offset + 0:tlvStart + offset + 2:1]),'hex'))[0]

                    temperature_stats_array.append(temperature_stats.copy())

                    if(verbose):
                        print(f"    tempReportValid {temperature_stats['tempReportValid']} (0 for valid)")
                        print(f"    tmpRx[0~3]Sens {temperature_stats['tmpRx0Sens']} {temperature_stats['tmpRx1Sens']} {temperature_stats['tmpRx2Sens']} {temperature_stats['tmpRx3Sens']}")
                        print(f"    tmpTx[0~2]Sens {temperature_stats['tmpTx0Sens']} {temperature_stats['tmpTx1Sens']} {temperature_stats['tmpTx2Sens']}")
                        print(f"    tmpPmSens {temperature_stats['tmpPmSens']}")
                        print(f"    tmpDig[0~1]Sens {temperature_stats['tmpDig0Sens']} {temperature_stats['tmpDig1Sens']}")
                
                
                tlvStart += tlvLen+8
            
    return (result, headerStartIndex,platform,frameNumber,timeCpuCycles, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, detectedX_array, detectedY_array, detectedZ_array, detectedV_array, detectedRange_array, detectedAzimuth_array, detectedElevAngle_array, detectedSNR_array, detectedNoise_array,rangeProfile_array,noiseFloor_array,azimuthHeatMap_array,rangeDoppler_array,stats_array,temperature_stats_array)




    









