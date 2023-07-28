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
8. (optional)启动UDP数据包接收线程(fastRead_in_Cpp_async_start)
9. 通过串口启动雷达（理论上通过FTDI(USB转SPI)也能控制，目前只在AWR2243上实现）(startSensor)
10. 等待UDP数据包接收线程结束+解析出原始数据(fastRead_in_Cpp_async_wait/fastRead_in_Cpp)
11. 保存原始数据到文件离线处理(tofile)
12. (optional)通过网口udp发送停止采集指令(stream_stop)
13. 通过串口关闭雷达(stopSensor) 或 通过网口发送重置雷达命令(reset_radar)
14. (optional)停止接收串口数据(stop_read_process)
15. (optional)解析从串口接收的点云等片内DSP处理好的数据(post_process_data_buf)

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
    numframes=10
    # 记得改端口号,verbose=True会显示向毫米波雷达板子发送的所有串口指令及响应
    radar = TI(cli_loc='COM4', data_loc='COM5',data_baud=921600,config_file=radar_config_file,verbose=True)
    # radar设置frame个数后会自动停止，无需向FPGA发送停止命令，但仍需向radar发送停止命令
    radar.setFrameCfg(numframes)

    # 3. 创建从串口接收片内DSP处理好的数据的进程
    radar.create_read_process(numframes)

    # 4. 通过网口UDP发送配置FPGA指令
    # 5. 通过网口UDP发送配置record数据包指令
    '''
    dca.sys_alive_check()             # 检查FPGA是否连通正常工作
    dca.config_fpga(dca_config_file)  # 配置FPGA参数
    dca.config_record(dca_config_file)# 配置record参数
    '''
    dca.configure(dca_config_file,radar_config_file)  # 此函数完成上述所有操作

    # 按回车开始采集
    input("press ENTER to start capture...")

    # 6. 启动串口接收进程（只进行缓存清零，仅当循环多次采集而不运行stop时才需使用）
    radar.start_read_process()
    # 7. 通过网口UDP发送开始采集指令
    dca.stream_start()
    # 8. 启动UDP数据包接收线程
    # numframes_out,sortInC_out = dca.fastRead_in_Cpp_async_start(numframes,sortInC=True) # 【采集方法一】1、异步调用(需要C++17及以上编译器支持)

    # 9. 通过串口启动雷达
    startTime = datetime.datetime.now()
    start = time.time()
    radar.startSensor()

    # 10. 等待UDP数据包接收线程结束+解析出原始数据
    # data_buf = dca.fastRead_in_Cpp_async_wait(numframes_out,sortInC_out) # 【采集方法一】2、等待异步线程结束
    data_buf = dca.fastRead_in_Cpp(numframes,sortInC=True) # 【采集方法二】同步调用(会丢失开始采集前的包，但兼容性更好)
    end = time.time()
    print("time elapsed(s):",end-start)

    # 11. 保存原始数据到文件
    filename="raw_data_"+startTime.strftime('%Y-%m-%d-%H-%M-%S')+".bin"
    data_buf.tofile(filename)
    print("file saved to",filename)
    
    # 12. DCA停止采集，设置frame个数后会自动停止，无需向FPGA发送停止命令
    # dca.stream_stop()
    # 13. 通过串口关闭雷达
    radar.stopSensor()
    # 14. 停止接收串口数据
    radar.stop_read_process()

    # 15. 解析从串口接收的点云等片内DSP处理好的数据,verbose=True会在处理过程中显示每一帧的详细信息
    DSP_Processed_data=radar.post_process_data_buf(verbose=False)

    # 解析好的点云等数据在DSP_Processed_data这个变量内
    # print(DSP_Processed_data)

    # 未解析的串口原始数据在radar.byteBuffer这个变量内
    # print(radar.byteBuffer)
    
    # 将解析后的串口数据保存到文件，加载可用np.load('xxx.npy', allow_pickle=True)
    dspFileName = "DSP_data_"+startTime.strftime('%Y-%m-%d-%H-%M-%S')
    np.save(dspFileName, DSP_Processed_data)
    print(f'file saved to {dspFileName}.npy')

except Exception as e:
    traceback.print_exc()
finally:
    if dca is not None:
        dca.close()
    if radar is not None:
        radar.cli_port.close()
        # radar.data_port.close() # 停止接收串口数据radar.stop_read_process()时自动关闭