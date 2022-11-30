# WzSerialportPlus

C++串口库，使用的是回调机制接收数据的实现方式，并解决了WzSerialport在Linux上读者反馈遇到一些数据阶段甚至接收不到的问题。



* Linux已完成

* Windows已完成



源码分别在对应的文件夹下。



#### Linux测试代码：

```cpp
#include <iostream>

#include "WzSerialportPlus.h"

int main(int argc,char** argv)
{
    WzSerialportPlus wzSerialportPlus;
    wzSerialportPlus.setReceiveCalback([&](char* data,int length){
        printf("received: %s\n",data);

        std::string responsePrefix = "received: ";
        std::string response(data,length);
        response = responsePrefix + response;
        
        wzSerialportPlus.send((char*)response.c_str(),response.length());
    });
    if(wzSerialportPlus.open("/dev/ttyS1",9600,1,8,'n'))
    {
        getchar();
        wzSerialportPlus.close();
    }

    return 0;
}
```



#### Windows测试代码：

```cpp
#include <iostream>

#include "WzSerialportPlus.h"

int main(int argc,char** argv)
{
    WzSerialportPlus wzSerialportPlus;
    wzSerialportPlus.setReceiveCalback([&](char* data,int length){
        printf("received: %s\n",data);

        std::string responsePrefix = "received: ";
        std::string response(data,length);
        response = responsePrefix + response;

        wzSerialportPlus.send((char*)response.c_str(),response.length());
    });
    if(wzSerialportPlus.open("\\\\.\\COM11",9600,1,8,'n'))
    {
        getchar();
        wzSerialportPlus.close();
    }

    return 0;
}
```

