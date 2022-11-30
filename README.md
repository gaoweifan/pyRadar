# ADC/UART data capturing using IWR1843 with DCA1000

capture both raw ADC IQ data and processed UART point cloud data simultaneously in pure python code (with a little C code)


## Introduction

该模块分为两部分，mmwave和fpga_udp。
1.  mmwave修改自[OpenRadar](https://github.com/PreSenseRadar/OpenRadar)，用于配置文件读取、串口数据发送与接收、原始数据解析等。
2.  fpga_udp修改自[pybind11 example](https://github.com/pybind/python_example)，用于通过C语言编写的socket代码从网口接收高速的原始数据。


## Installation

 - clone this repository
 - `pip install ./fpga_udp`


## Example

### ***captureAll.py***
同时采集原始ADC采样的IQ数据及片内DSP预处理好的点云等串口数据的示例代码。
#### 1.采集原始数据的一般流程
 1.  (optional)创建从串口接收片内DSP处理好的数据的进程
 2.  通过串口启动雷达（理论上通过网口也能控制，暂未实现）
 3.  通过网口udp发送配置fpga指令
 4.  通过网口udp发送配置record数据包指令
 5.  (optional)启动串口接收进程（只进行缓存清零）
 6.  通过网口udp发送开始采集指令
 7.  实时数据流处理或离线采集保存
 8.  通过网口udp发送停止采集指令
 9.  通过串口关闭雷达 或 通过网口发送重置雷达命令
 10.  (optional)停止接收串口数据
 11.  (optional)解析从串口接收的点云等片内DSP处理好的数据
#### 2."*.cfg"毫米波雷达配置文件要求
 - Default profile in Visualizer disables the LVDS streaming.
 - To enable it, please export the chosen profile and set the appropriate enable bits.
 - adcbufCfg需如下设置，lvdsStreamCfg的第三个参数需设置为1，具体参见mmwave_sdk_user_guide.pdf
    - adcbufCfg -1 0 1 1 1
    - lvdsStreamCfg -1 0 1 0 
#### 3."cf.json"数据采集卡配置文件要求
 - In default conditions, Ethernet throughput varies up to 325 Mbps speed in a 25-µs Ethernet packet delay. 
 - The user can change the Ethernet packet delay from 5 µs to 500 µs to achieve different throughputs.
    - "packetDelay_us":  5 (us)   ~   706 (Mbps)
    - "packetDelay_us": 10 (us)   ~   545 (Mbps)
    - "packetDelay_us": 25 (us)   ~   325 (Mbps)
    - "packetDelay_us": 50 (us)   ~   193 (Mbps)

### ***testDecode.ipynb***
解析原始ADC采样数据及串口数据的示例代码，需要用Jupyter(推荐VS Code安装Jupyter插件)打开。
#### 1.解析LVDS接收的ADC原始IQ数据
##### 利用numpy对LVDS接收的ADC原始IQ数据进行解析
 - 载入相关库
 - 设置对应参数
 - 载入保存的bin数据并解析
 - 绘制时域IQ波形
 - 计算Range-FFT
 - 计算Doppler-FFT
 - 计算Azimuth-FFT
##### 利用mmwave.dsp提供的函数对LVDS接收的ADC原始IQ数据进行解析
#### 2.解析UART接收的片内DSP处理过的点云、doppler等数据
 - 载入相关库
 - 载入保存的串口解析数据
 - 显示cfg文件设置的数据
 - 显示片内处理时间(可用来判断是否需要调整帧率)
 - 显示各天线温度
 - 显示数据包信息
 - 显示点云数据
 - 计算距离标签及多普勒速度标签
 - 显示range profile及noise floor profile
 - 显示多普勒图Doppler Bins
 - 显示方位角图Azimuth (Angle) Bins

### ***testParam.ipynb***
毫米波雷达配置参数合理性校验，需要用Jupyter(推荐VS Code安装Jupyter插件)打开。
 - 主要校验毫米波雷达需要的cfg文件及DCA采集板需要的cf.json文件的参数配置是否正确。
 - 参数的约束条件来自于IWR1843自身的器件特性，具体请参考IWR1843数据手册、mmwave SDK用户手册、chirp编程手册。
 - 若参数满足约束条件将以青色输出调试信息，若不满足则以紫色或黄色输出。
 - 需要注意的是，本程序约束条件并非完全准确，故特殊情况下即使参数全都满足约束条件，也有概率无法正常运行。

### ***testDecodeADCdata.mlx***
解析原始ADC采样数据的MATLAB示例代码
 - 设置对应参数
 - 载入保存的bin原始ADC数据
 - 根据参数解析重构数据格式
 - 绘制时域IQ波形
 - 计算Range-FFT(一维FFT+静态杂波滤除)
 - 计算Doppler-FFT
 - 1D-CA-CFAR Detector on Range-FFT
 - 计算Azimuth-FFT


## Software Architecture

### TBD.py
TBD


## Environmental Requirements

1.  TBD


## Instructions for Use

1.  先按照要求搭建运行环境，未提及的模块在运行时若报错请自己查询添加
2.  TBD