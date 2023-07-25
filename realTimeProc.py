import traceback
import time
from mmwave.dataloader import DCA1000
from mmwave.dataloader.radars import TI
import numpy as np
import datetime
'''
# AWR2243采集原始数据的一般流程
1. 重置雷达与DCA1000(reset_radar、reset_fpga)
2. 通过UART初始化雷达并配置相应参数(TI、setFrameCfg)
3. 通过网口udp发送配置fpga指令(config_fpga)
4. 通过网口udp发送配置record数据包指令(config_record)
5. 通过网口udp发送开始采集指令(stream_start)
6. 通过UART启动雷达（理论上通过FTDI(USB转SPI)也能控制，目前只在AWR2243上实现）(startSensor)
7. UDP循环接收数据包+解析出原始数据+数据实时处理(fastRead_in_Cpp、postProc)
8. 通过UART停止雷达(stopSensor)
9. 通过网口udp发送停止采集指令(fastRead_in_Cpp_thread_stop、stream_stop)

# "*.cfg"毫米波雷达配置文件要求
Default profile in Visualizer disables the LVDS streaming.
To enable it, please export the chosen profile and set the appropriate enable bits.
adcbufCfg需如下设置，lvdsStreamCfg的第三个参数需设置为1，具体参见mmwave_sdk_user_guide.pdf
1. adcbufCfg -1 0 1 1 1
2. lvdsStreamCfg -1 0 1 0 

# "cf.json"数据采集卡配置文件要求
使用xWR1843时需将lvdsMode设为2，xWR1843只支持2路LVDS lanes
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
radar = None

def postProc(adc_data,ADC_PARAMS_l):
    adc_data=np.reshape(adc_data,(-1,ADC_PARAMS_l['chirps'],ADC_PARAMS_l['tx'],ADC_PARAMS_l['rx'],ADC_PARAMS_l['samples']//2,ADC_PARAMS_l['IQ'],2))
    #100Frames*16Chirps*2TX*4RX*32(Samples//2)*2IQ(Lanes)*2Samples
    # print(adc_data.shape)

    adc_data=np.transpose(adc_data,(0,1,2,3,4,6,5))
    adc_data=np.reshape(adc_data,(-1,ADC_PARAMS_l['chirps'],ADC_PARAMS_l['tx'],ADC_PARAMS_l['rx'],ADC_PARAMS_l['samples'],ADC_PARAMS_l['IQ']))
    #100Frames*16Chirps*2TX*4RX*64Samples*2IQ(Lanes)
    # print(adc_data.shape)

    adc_data = (1j * adc_data[...,0] + adc_data[...,1]).astype(np.complex64) # Q first
    #100Frames*16Chirps*2TX*4RX*64Samples复数形式
    # print(adc_data.shape)

    # Range-FFT
    range_fft = np.fft.fft(adc_data, axis=-1)
    
    # Cluster Removal
    range_fft_avg = np.mean(range_fft,1)#沿chirp计算均值
    range_fft_avg=range_fft-np.tile(np.expand_dims(range_fft_avg,1),(1,ADC_PARAMS_l['chirps'],1,1,1))#与均值相消
    
    # Doppler-FFT
    range_doppler = np.fft.fft(range_fft_avg, axis=1)
    range_doppler = np.fft.fftshift(range_doppler, axes=1)

try:
    dca = DCA1000()

    # 1. 重置雷达与DCA1000
    dca.reset_radar()
    dca.reset_fpga()
    print("wait for reset")
    time.sleep(1)
    
    # 2. 通过UART初始化雷达并配置相应参数
    dca_config_file = "configFiles/cf.json" # 记得将cf.json中的lvdsMode设为2，xWR1843只支持2路LVDS lanes
    radar_config_file = "configFiles/xWR1843_profile_3D.cfg" # 记得将lvdsStreamCfg的第三个参数设置为1开启LVDS数据传输
    # 记得改端口号,verbose=True会显示向毫米波雷达板子发送的所有串口指令及响应
    radar = TI(cli_loc='COM4', data_loc='COM5',data_baud=921600,config_file=radar_config_file,verbose=True)
    numLoops=50
    frameNumInBuf=16
    numframes=16 # numframes必须<=frameNumInBuf
    radar.setFrameCfg(0) # 0 for infinite frames
    
    # 3. 通过网口UDP发送配置FPGA指令
    # 4. 通过网口UDP发送配置record数据包指令
    '''
    dca.sys_alive_check()             # 检查FPGA是否连通正常工作
    dca.config_fpga(dca_config_file)  # 配置FPGA参数
    dca.config_record(dca_config_file)# 配置record参数
    '''
    ADC_PARAMS_l,_=dca.configure(dca_config_file,radar_config_file)  # 此函数完成上述所有操作

    # 按回车开始采集
    input("press ENTER to start capture...")

    # 5. 通过网口udp发送开始采集指令
    dca.stream_start()
    dca.fastRead_in_Cpp_thread_start(frameNumInBuf) # 启动udp采集线程，方法一（推荐）

    # 6. 通过UART启动雷达
    radar.startSensor()

    # 7. UDP接收数据包+解析出原始数据+数据实时处理
    start = time.time()
    for i in range(numLoops):
        print("current loop:",i)
        # start1 = time.time()
        data_buf = dca.fastRead_in_Cpp_thread_get(numframes,verbose=True,sortInC=True) # 方法一（推荐）
        # data_buf = dca.fastRead_in_Cpp(1,sortInC=True) # 方法二
        # data_buf = dca.fastRead_in_Cpp_noDisp(1) # 方法三
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
    if radar is not None:
        # 8. 通过UART停止雷达
        radar.stopSensor()
    if dca is not None:
        # 9. 通过网口udp发送停止采集指令
        dca.fastRead_in_Cpp_thread_stop() # 停止udp采集线程(必须先于stream_stop调用，即UDP接收时不能同时发送)
        dca.stream_stop()  # DCA停止采集
        dca.close()