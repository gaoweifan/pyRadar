
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