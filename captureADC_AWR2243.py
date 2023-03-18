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
7. 实时处理数据流或离线采集保存(write_frames_to_file)
8. (optional, 若numFrame==0则不能有)等待雷达采集结束(AWR2243_waitSensorStop)
9. (optional, 若numFrame==0则必须有)通过网口udp发送停止采集指令(stream_stop)
10. (optional, 若numFrame==0则必须有)通过SPI停止雷达(AWR2243_sensorStop)
11. 通过SPI关闭雷达电源与配置文件(AWR2243_poweroff)

# "mmwaveconfig.txt"毫米波雷达配置文件要求
TBD

# "cf.json"数据采集卡配置文件要求
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
    dca_config_file = "configFiles/cf.json"
    radar_config_file = "configFiles/AWR2243_mmwaveconfig.txt"
    radar.AWR2243_init(radar_config_file)
    numframes=100
    radar.AWR2243_setFrameCfg(numframes)  # radar设置frame个数后会自动停止，无需向fpga及radar发送停止命令

    input("press ENTER to start capture...")  # 按回车开始采集
    # 3. 通过网口udp发送配置fpga指令
    # 4. 通过网口udp发送配置record数据包指令
    '''
    dca.sys_alive_check()  # 检查fpga是否连通正常工作
    dca.config_fpga(dca_config_file)  # 3. 配置fpga
    dca.config_record(dca_config_file)  # 4. 配置record
    '''
    dca.configure(dca_config_file,radar_config_file)  # 此函数完成上述所有操作
    # 5. 通过网口udp发送开始采集指令
    dca.stream_start()
    startTime = datetime.datetime.now()
    # 6. 通过SPI启动雷达
    radar.AWR2243_sensorStart()

    # 7. 实时处理数据流或离线采集保存
    dca.write_frames_to_file(filename="raw_data_"+startTime.strftime('%Y-%m-%d-%H-%M-%S')+".bin",numframes=numframes) # 7. 从网口接收ADC原始数据+处理数据+保存到文件
    
    # 8. 等待雷达采集结束
    radar.AWR2243_waitSensorStop()
    # 9. 通过网口udp发送停止采集指令
    # dca.stream_stop()  # DCA停止采集，设置frame个数后会自动停止，无需向fpga发送停止命令
    # 10. 通过SPI停止雷达
    radar.AWR2243_sensorStop()

except Exception as e:
    traceback.print_exc()
finally:
    if dca is not None:
        dca.close()
    # 11. 通过SPI关闭雷达电源与配置文件
    radar.AWR2243_poweroff()