import traceback
import time
from mmwave.dataloader import DCA1000
import fpga_udp as radar
import numpy as np
import datetime
'''
# AWR2243采集原始数据的一般流程
1. 重置雷达与DCA1000(reset_radar、reset_fpga)
2. 通过SPI初始化雷达并配置相应参数(AWR2243_init、AWR2243_setFrameCfg)(linux下需要root权限)
3. 通过网口udp发送配置fpga指令(config_fpga)
4. 通过网口udp发送配置record数据包指令(config_record)
5. 通过网口udp发送开始采集指令(stream_start)
6. 通过SPI启动雷达(AWR2243_sensorStart)
7. UDP接收数据包+解析出原始数据+数据实时处理(fastRead_in_Cpp、postProc)
8.1. (optional, 若numFrame==0则必须有)通过SPI停止雷达(AWR2243_sensorStop)
8.2. (optional, 若numFrame==0则不能有)等待雷达采集结束(AWR2243_waitSensorStop)
9. (optional, 若numFrame==0则必须有)通过网口udp发送停止采集指令(stream_stop)
10. 通过SPI关闭雷达电源与配置文件(AWR2243_poweroff)

# "mmwaveconfig.txt"毫米波雷达配置文件要求
TBD

# "cf.json"数据采集卡配置文件要求
具体信息请查阅TI_DCA1000EVM_CLI_Software_UserGuide.pdf
lvds Mode:
LVDS mode specifies the lane config for LVDS. This field is valid only when dataTransferMode is "LVDSCapture".
The valid options are
• 1 (4lane)
• 2 (2lane)
packet delay:
In default conditions, Ethernet throughput varies up to 325 Mbps speed in a 25-µs Ethernet packet delay. 
The user can change the Ethernet packet delay from 5 µs to 500 µs to achieve different throughputs.
"packetDelay_us":  5 (us)   ~   706 (Mbps)
"packetDelay_us": 10 (us)   ~   545 (Mbps)
"packetDelay_us": 25 (us)   ~   325 (Mbps)
"packetDelay_us": 50 (us)   ~   193 (Mbps)
'''
dca = None

def postProc(adc_data,ADC_PARAMS_l):
    adc_data=np.reshape(adc_data,(-1,ADC_PARAMS_l['chirps'],ADC_PARAMS_l['tx'],ADC_PARAMS_l['samples'],ADC_PARAMS_l['IQ'],ADC_PARAMS_l['rx']))
    adc_data=np.flip(adc_data,axis=3)#对sample轴取倒序
    #100Frames*128Chirps*3TX*256Samples*2IQ*4RX(Lanes)
    # print(adc_data.shape)

    adc_data=np.transpose(adc_data,(0,1,2,5,3,4))
    #100Frames*128Chirps*3TX*4RX(Lanes)*256Samples*2IQ
    # print(adc_data.shape)

    adc_data = (1j * adc_data[...,0] + adc_data[...,1]).astype(np.complex64)
    #100Frames*128Chirps*3TX*4RX*256Samples复数形式
    # print(adc_data.shape)

    range_fft = np.fft.fft(adc_data, axis=4)
    range_fft_avg = np.mean(range_fft,1)#沿chirp计算均值
    range_fft_avg=range_fft-np.tile(np.expand_dims(range_fft_avg,1),(1,ADC_PARAMS_l['chirps'],1,1,1))#与均值相消

    range_fft_comb=np.transpose(np.array([np.real(range_fft_avg[:,:,:,:,0:8]),np.imag(range_fft_avg[:,:,:,:,0:8])],dtype=np.float32),(1,2,5,3,4,0))
    # print(range_fft_comb.shape)#frame*Chirps*Samples_boundary*TX*RX*IQ

    x_min=range_fft_comb.min(axis=(1,2,3,4,5))
    x_min=np.tile(np.expand_dims(x_min,axis=(1,2,3,4,5)),(1,range_fft_comb.shape[1],range_fft_comb.shape[2],range_fft_comb.shape[3],range_fft_comb.shape[4],range_fft_comb.shape[5]))

    x_max=range_fft_comb.max(axis=(1,2,3,4,5))
    x_max=np.tile(np.expand_dims(x_max,axis=(1,2,3,4,5)),(1,range_fft_comb.shape[1],range_fft_comb.shape[2],range_fft_comb.shape[3],range_fft_comb.shape[4],range_fft_comb.shape[5]))

    data_normalized = 2*(range_fft_comb-x_min)/(x_max-x_min)-1

try:
    dca = DCA1000()

    # 1. 重置雷达与DCA1000
    dca.reset_radar()
    dca.reset_fpga()
    print("wait for reset")
    time.sleep(1)
    
    # 2. 通过SPI初始化雷达并配置相应参数
    radar_config_file = "configFiles/AWR2243_mmwaveconfig.txt"  # laneEn=15则LVDS为4 lane模式
    dca_config_file = "configFiles/cf.json"  # 若LVDS设置为4 lane模式，记得将cf.json中的lvdsMode设为1
    radar.AWR2243_init(radar_config_file)
    numLoops=50
    frameNumInBuf=16
    numframes=16 # numframes必须<=frameNumInBuf
    radar.AWR2243_setFrameCfg(0)  # Valid Range 0 to 65535 (0 for infinite frames)
    
    # 检查LVDS参数
    LVDSDataSizePerChirp_l,maxSendBytesPerChirp_l,ADC_PARAMS_l,CFG_PARAMS_l=dca.AWR2243_read_config(radar_config_file)
    dca.refresh_parameter()
    print(ADC_PARAMS_l)
    print(CFG_PARAMS_l)
    print("LVDSDataSizePerChirp:%d must <= maxSendBytesPerChirp:%d"%(LVDSDataSizePerChirp_l,maxSendBytesPerChirp_l))
    # 检查fpga是否连通正常工作
    print("System connection check:",dca.sys_alive_check())
    print(dca.read_fpga_version())
    # 3. 通过网口udp发送配置fpga指令
    print("Config fpga:",dca.config_fpga(dca_config_file))
    # 4. 通过网口udp发送配置record数据包指令
    print("Config record packet delay:",dca.config_record(dca_config_file))

    # 按回车开始采集
    input("press ENTER to start capture...")

    # 5. 通过网口udp发送开始采集指令
    dca.stream_start()
    dca.fastRead_in_Cpp_thread_start(frameNumInBuf) # 启动udp采集线程

    # 6. 通过SPI启动雷达
    radar.AWR2243_sensorStart()

    # 7. UDP接收数据包+解析出原始数据+数据实时处理
    start = time.time()
    for i in range(numLoops):
        print("current loop:",i)
        # start1 = time.time()
        # data_buf = dca.fastRead_in_Cpp(1,sortInC=True)
        # data_buf = dca.fastRead_in_Cpp_noDisp(1)
        data_buf = dca.fastRead_in_Cpp_thread_get(numframes,verbose=True,sortInC=True)
        # end1 = time.time()
        # print("capture time elapsed(s):",end1-start1)
        # print("capture performance: %.2f FPS"%(numframes/(end1-start1)))

        # start2 = time.time()
        postProc(data_buf,ADC_PARAMS_l)
        # end2 = time.time()
        # print("postProc time elapsed(s):",end2-start2)
        # print("postProc performance: %.2f FPS"%(numframes/(end2-start2)))
    end = time.time()
    print("Performance: %.2f Loops per Sec"%(numLoops/(end-start)))

except Exception as e:
    traceback.print_exc()
finally:
    # 8.1 通过SPI停止雷达
    radar.AWR2243_sensorStop()
    # 8.2 等待雷达采集结束
    radar.AWR2243_waitSensorStop()
    if dca is not None:
        # 9. 通过网口udp发送停止采集指令
        dca.fastRead_in_Cpp_thread_stop() # 停止udp采集线程(必须先于stream_stop调用，即UDP接收时不能同时发送)
        dca.stream_stop()  # DCA停止采集
        dca.close()
    # 10. 通过SPI关闭雷达电源与配置文件
    radar.AWR2243_poweroff()