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
6. 启动UDP数据包接收线程(fastRead_in_Cpp_async_start)
7. 通过SPI启动雷达(AWR2243_sensorStart)
8.1. (optional, 若numFrame==0则必须有)通过SPI停止雷达(AWR2243_sensorStop)
8.2. (optional, 若numFrame==0则不能有)等待雷达采集结束(AWR2243_waitSensorStop)
9. (optional, 若numFrame==0则必须有)通过网口udp发送停止采集指令(stream_stop)
10. 等待UDP数据包接收线程结束+解析出原始数据(fastRead_in_Cpp_async_wait)
11. 保存原始数据到文件离线处理(tofile)
12. 通过SPI关闭雷达电源与配置文件(AWR2243_poweroff)

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
    numframes=50
    radar.AWR2243_setFrameCfg(numframes)  # radar设置frame个数后会自动停止，无需向fpga及radar发送停止命令
    
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
    # 6. 启动UDP数据包接收线程
    numframes_out,sortInC_out = dca.fastRead_in_Cpp_async_start(numframes,sortInC=True)

    # 7. 通过SPI启动雷达
    startTime = datetime.datetime.now()
    start = time.time()
    radar.AWR2243_sensorStart()

    # 8.1 通过SPI停止雷达
    # radar.AWR2243_sensorStop()
    # 8.2 等待雷达采集结束
    radar.AWR2243_waitSensorStop()
    end = time.time()
    print("time elapsed(s):",end-start)
    
    # 9. 通过网口udp发送停止采集指令
    # dca.stream_stop()  # DCA停止采集，设置frame个数后会自动停止，无需向fpga发送停止命令

    # 10. 等待UDP数据包接收线程结束+解析出原始数据
    data_buf = dca.fastRead_in_Cpp_async_wait(numframes=numframes_out,sortInC=sortInC_out)
    # 11. 保存原始数据到文件
    filename="raw_data_"+startTime.strftime('%Y-%m-%d-%H-%M-%S')+".bin"
    data_buf.tofile(filename)
    print("file saved to",filename)

except Exception as e:
    traceback.print_exc()
finally:
    if dca is not None:
        dca.close()
    # 12. 通过SPI关闭雷达电源与配置文件
    radar.AWR2243_poweroff()