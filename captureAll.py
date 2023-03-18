import traceback
import time
from mmwave.dataloader import DCA1000
from mmwave.dataloader.radars import TI
import numpy as np
import datetime
'''
# 采集原始数据的一般流程
1. 重置雷达与DCA1000(reset_radar、reset_fpga)
2. 通过UART初始化雷达并配置相应参数(TI、setFrameCfg)
3. (optional)创建从串口接收片内DSP处理好的数据的进程(create_read_process)
4. 通过网口udp发送配置fpga指令(config_fpga)
5. 通过网口udp发送配置record数据包指令(config_record)
6. (optional)启动串口接收进程（只进行缓存清零）(start_read_process)
7. 通过网口udp发送开始采集指令(stream_start)
8. 通过串口启动雷达（理论上通过FTDI(USB转SPI)也能控制，目前只在AWR2243上实现）(startSensor)
9. 实时处理数据流或离线采集保存(write_frames_to_file)
10. (optional)通过网口udp发送停止采集指令(stream_stop)
11. 通过串口关闭雷达(stopSensor) 或 通过网口发送重置雷达命令(reset_radar)
12. (optional)停止接收串口数据(stop_read_process)
13. (optional)解析从串口接收的点云等片内DSP处理好的数据(post_process_data_buf)

# "*.cfg"毫米波雷达配置文件要求
Default profile in Visualizer disables the LVDS streaming.
To enable it, please export the chosen profile and set the appropriate enable bits.
adcbufCfg需如下设置，lvdsStreamCfg的第三个参数需设置为1，具体参见mmwave_sdk_user_guide.pdf
1. adcbufCfg -1 0 1 1 1
2. lvdsStreamCfg -1 0 1 0 

# "cf.json"数据采集卡配置文件要求
In default conditions, Ethernet throughput varies up to 325 Mbps speed in a 25-µs Ethernet packet delay. 
The user can change the Ethernet packet delay from 5 µs to 500 µs to achieve different throughputs.
"packetDelay_us":  5 (us)   ~   706 (Mbps)
"packetDelay_us": 10 (us)   ~   545 (Mbps)
"packetDelay_us": 25 (us)   ~   325 (Mbps)
"packetDelay_us": 50 (us)   ~   193 (Mbps)
'''
dca = None
radar = None

try:
    dca = DCA1000()

    # 1. 重置雷达与DCA1000
    dca.reset_radar()
    dca.reset_fpga()
    print("wait for reset")
    time.sleep(1)

    # 2. 通过UART初始化雷达并配置相应参数
    dca_config_file = "configFiles/cf.json"
    radar_config_file = "configFiles/xWR1843_profile_3D.cfg"
    numframes=1400
    # 记得改端口号,verbose=True会显示向毫米波雷达板子发送的所有串口指令及响应
    radar = TI(cli_loc='COM4', data_loc='COM5',data_baud=921600,config_file=radar_config_file,verbose=True)
    # radar设置frame个数后会自动停止，无需向fpga发送停止命令，但仍需向radar发送停止命令
    radar.setFrameCfg(numframes)

    # 3. 创建从串口接收片内DSP处理好的数据的进程
    # radar.create_read_process(numframes)

    # 4. 通过网口udp发送配置fpga指令
    # 5. 通过网口udp发送配置record数据包指令
    '''
    dca.sys_alive_check()             # 检查fpga是否连通正常工作
    dca.config_fpga(dca_config_file)  # 配置fpga
    dca.config_record(dca_config_file)# 配置record
    '''
    dca.configure(dca_config_file,radar_config_file)  # 此函数完成上述所有操作

    # 按回车开始采集
    input("press ENTER to start capture...")

    # 6. 启动串口接收进程（只进行缓存清零，仅当循环多次采集而不运行stop时才需使用）
    # radar.start_read_process()
    # 7. 通过网口udp发送开始采集指令
    dca.stream_start()
    startTime = datetime.datetime.now()
    # 8. 通过串口启动雷达
    radar.startSensor()

    # 9. 从网口接收ADC原始数据+处理数据+保存到文件
    dca.write_frames_to_file(filename="raw_data_"+startTime.strftime('%Y-%m-%d-%H-%M-%S')+".bin",numframes=numframes)
    
    # 10. DCA停止采集，设置frame个数后会自动停止，无需向fpga发送停止命令
    # dca.stream_stop()
    # 11. 通过串口关闭雷达
    radar.stopSensor()
    # 12. 停止接收串口数据
    # radar.stop_read_process()

    # 13. 解析从串口接收的点云等片内DSP处理好的数据,verbose=True会在处理过程中显示每一帧的详细信息
    # DSP_Processed_data=radar.post_process_data_buf(verbose=False)
    # 解析好的点云等数据在DSP_Processed_data这个变量内
    # 未解析的串口原始数据在radar.byteBuffer这个变量内
    # print(radar.byteBuffer)
    # 将解析后的串口数据保存到文件，加载可用np.load('xxx.npy', allow_pickle=True)
    # np.save('DSP_Processed_data.npy', DSP_Processed_data)
    # print('file saved to DSP_Processed_data.npy')

except Exception as e:
    traceback.print_exc()
finally:
    if dca is not None:
        dca.close()
    if radar is not None:
        radar.cli_port.close()
        # radar.data_port.close() # 停止接收串口数据radar.stop_read_process()时自动关闭