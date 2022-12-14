#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <thread>
#include <mutex>
#ifdef _WIN32
    #include <Winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #include "WzSerialportPlus/Windows/WzSerialportPlus.h"
#elif __linux__
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include "WzSerialportPlus/Linux/WzSerialportPlus.h"
#endif

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

WzSerialportPlus radar_serial_port;
int memidx=0;
int overflowCnt=0;
int packetNum_g=1000, packetSize_g=1456;
uint8_t *serial_buf_ptr=NULL;
std::mutex serial_buf_mutex;

int add(int i, int j) {
    return i+j;
}

bool radar_start_read_thread(std::string portName, int baudrate, int packetNum, int packetSize){
    if(serial_buf_ptr!=NULL)
        return false;
    serial_buf_ptr = new uint8_t[packetSize*packetNum];//initialize buffer
    packetNum_g=packetNum;
    packetSize_g=packetSize;

    radar_serial_port.setReceiveCalback([&](char* data,int length){//Set the receive callback function and execute it after receiving the data
        serial_buf_mutex.lock();//exclusive buffer
        if(memidx+length>packetSize_g*packetNum_g){//Guaranteed not to exceed the buffer range
            overflowCnt++;
            printf("[fpga_udp]serial buffer overflowed for %d time(s)!memidx=%d,length=%d,packetSize=%d,packetNum=%d\n",overflowCnt,memidx,length,packetSize_g,packetNum_g);
            memidx=0;
        }
        memcpy(serial_buf_ptr+memidx,data,length);//Copy the received data into the buffer
        memidx+=length;
        serial_buf_mutex.unlock();
    });

    #ifdef _WIN32
        if (portName.substr(0,3)=="COM" && stoi(portName.substr(3))>=10)
            portName="\\\\.\\"+portName;//port name has prefix in windows
    #endif
    return radar_serial_port.open(portName,baudrate,1,8,'n');//Open the port to start the thread to receive data
}

py::array_t<uint8_t> get_radar_buf(){
    serial_buf_mutex.lock();//exclusive buffer
    auto result = py::array_t<uint8_t>(memidx);
    py::buffer_info buf = result.request();
    uint8_t *buf_ptr = static_cast<uint8_t *>(buf.ptr);
    memcpy(buf_ptr,serial_buf_ptr,memidx);
    memidx=0;
    serial_buf_mutex.unlock();
    return result;
}

void radar_stop_read_thread(){
    radar_serial_port.close();
    serial_buf_mutex.lock();
    delete[] serial_buf_ptr;
    serial_buf_ptr=NULL;
    memidx=0;
    overflowCnt=0;
    serial_buf_mutex.unlock();
}

py::array_t<uint8_t> read_data_udp(int sock_fd, int packetNum, int packetSize, int timeout_s){
    if(packetSize%2 != 0){
        throw std::runtime_error("[fpga_udp]Number of packetSize must be even");
    }
    /* No pointer is passed, so NumPy will allocate the buffer */
    auto result = py::array_t<uint8_t>(packetSize*packetNum);
    py::buffer_info buf = result.request();
    uint8_t *buf_ptr = static_cast<uint8_t *>(buf.ptr);

    // Time out
    #ifdef __linux__
        int sockfd = sock_fd;
        struct timeval tv;
        tv.tv_sec  = timeout_s;
        tv.tv_usec = 0;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
    #elif _WIN32
        SOCKET sockfd = (SOCKET)sock_fd;
        int TimeOut = timeout_s*1000;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&TimeOut, sizeof(TimeOut));
    #endif

    struct sockaddr_in src;
    socklen_t src_len = sizeof(src);
    memset(&src, 0, sizeof(src));

    for(int i=0;i<packetNum;i++){
        int idx = i*packetSize;
        // if(!(idx<packetSize*packetNum)){
        //     throw std::runtime_error("Array Index Out Of Bounds!");
        // }
        int receivePacketLen=recvfrom(sockfd, (char*)&buf_ptr[idx], packetSize, 0, (sockaddr*)&src, &src_len);
        if (receivePacketLen < 0){      //blocking receive timeout
            printf("[fpga_udp]udp time out\n");
            break;
        }
    }
    

    return result;
}

PYBIND11_MODULE(fpga_udp, m) {
    m.doc() = R"pbdoc(
        FPGA UDP reader plugin
        -----------------------

        .. currentmodule:: fpga_udp

        .. autosummary::
           :toctree: _generate

           add
           subtract
           radar_start_read_thread
           get_radar_buf
           radar_stop_read_thread
           getOverflowCnt
           read_data_udp
    )pbdoc";

    m.def("radar_start_read_thread", &radar_start_read_thread);
    m.def("get_radar_buf", &get_radar_buf);
    m.def("radar_stop_read_thread", &radar_stop_read_thread);
    m.def("getOverflowCnt", []() {
        return overflowCnt;
    });
    m.def("reset_radar_buf", []() {
        serial_buf_mutex.lock();
        memidx=0;
        overflowCnt=0;
        serial_buf_mutex.unlock();
    });

    m.def("read_data_udp", &read_data_udp, R"pbdoc(
        read UDP packet from FPGA as fast as written in C.

        sockfd:\tan open file descriptor of a socket
        packetNum:\tA total of packetNum packets will be read
        packetSize:\tand the size of each packet is packetSize
        timeout_s:\ttime out for udp in sec
    )pbdoc");

    m.def("add", &add, R"pbdoc(
        Add two numbers
    )pbdoc");

    m.def("subtract", [](int i, int j) {
        return i - j;
    });

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
