
#include "WzSerialportPlus.h"

#include <stdio.h>
#include <string.h>

WzSerialportPlus::WzSerialportPlus()
    : serialportHandle(nullptr),
        name(""),
        baudrate(9600),
        stopbit(1),
        databit(8),
        paritybit('n'),
        receivable(false),
        receiveMaxlength(92160),
        receiveTimeout(5000),
        receiveCallback(nullptr)
{

}

WzSerialportPlus::WzSerialportPlus(const std::string& name,
                    const int& baudrate,
                    const int& stopbit,
                    const int& databit,
                    const int& paritybit)
		:serialportHandle(nullptr),
            name(name),
            baudrate(baudrate),
            stopbit(stopbit),
            databit(databit),
            paritybit(paritybit),
            receivable(false),
            receiveMaxlength(92160),
            receiveTimeout(5000),
            receiveCallback(nullptr)
{

}

WzSerialportPlus::~WzSerialportPlus()
{
    close();

    while(receivable)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // printf("[WzSerialportPlus::~WzSerialportPlus()]: destructed...\n");
}

bool WzSerialportPlus::open()
{
	serialportHandle = CreateFileA(name.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0, 
		NULL,
		OPEN_EXISTING, 
		0, 
		NULL);

	if (serialportHandle == (HANDLE)-1)
	{
        printf("[WzSerialportPlus::open()]: open failed with serialportHandle is %d , maybe permission denied or this serialport is opened!\n", (int)serialportHandle);
		return false;
	}
	
	if (!SetupComm(serialportHandle, receiveMaxlength, receiveMaxlength))
	{
		printf("[WzSerialportPlus::open()]: open failed with cannot setup io buffer size!\n");
		CloseHandle(serialportHandle);
		return false;
	}

	/* 
	 * parameters config:
	 *		baudrate,databit,paritybit,stopbit 
	 */
	DCB parameters;
	memset(&parameters, 0, sizeof(parameters));
	parameters.DCBlength = sizeof(parameters);
	parameters.BaudRate = baudrate; 
	parameters.ByteSize = databit; 
	switch (paritybit)
	{
	case 'n':
    case 'N':
		parameters.Parity = NOPARITY;
		// parameters.fParity = FALSE;
		break;
	case 'o':
	case 'O':
		parameters.Parity = ODDPARITY;
		break;
	case 'e':
    case 'E': 
		parameters.Parity = EVENPARITY;
		break;
	case 3:
		parameters.Parity = MARKPARITY;
		break;
	case 4:
		parameters.Parity = SPACEPARITY;
		break;
	}
	switch (stopbit) 
	{
	case 1:
		parameters.StopBits = ONESTOPBIT;
		break;
	case 2:
		parameters.StopBits = TWOSTOPBITS;
		break;
	case 3:
		parameters.StopBits = ONE5STOPBITS; 
		break;
	}
	if (!SetCommState(serialportHandle, &parameters))
	{
		printf("[WzSerialportPlus::open()]: open failed with cannot set parameters!\n");
		CloseHandle(serialportHandle);
		return false;
	}

	/* timeouts config */
	COMMTIMEOUTS timeOuts;
	timeOuts.ReadIntervalTimeout = MAXDWORD;
	timeOuts.ReadTotalTimeoutMultiplier = 0;
	timeOuts.ReadTotalTimeoutConstant = 0;
	timeOuts.WriteTotalTimeoutMultiplier = 0;
	timeOuts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(serialportHandle, &timeOuts);

	/* clear all buffers */
	PurgeComm(serialportHandle, PURGE_TXCLEAR | PURGE_RXCLEAR);

    receivable = true;

    std::thread([&]{
        char* receiveData = new char[receiveMaxlength];
		DWORD receivedLength = 0;
		BOOL readState = false;

        while (receivable)
        {
			memset(receiveData, 0, receiveMaxlength);

			readState = ReadFile(serialportHandle,
				receiveData,
				receiveMaxlength,
				&receivedLength,
				NULL);

			if (readState && receivedLength > 0)
			{
				onReceive(receiveData, receivedLength);
				if (nullptr != receiveCallback)
				{
					receiveCallback(receiveData, receivedLength);
				}
			}

            receivedLength = 0;
			readState = false;
			
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        delete[] receiveData;
        receiveData = nullptr; 
    }).detach();

    printf("[WzSerialportPlus::open()]: open success.\n");
    return true;
}

bool WzSerialportPlus::open(const std::string& name,
                            const int& baudrate,
                            const int& stopbit,
                            const int& databit,
                            const int& paritybit)
{
    this->name = name;
    this->baudrate = baudrate;
    this->stopbit = stopbit;
    this->databit = databit;
    this->paritybit = paritybit;
    return open();
}

void WzSerialportPlus::close()
{   
    if(receivable)
    {
        receivable = false;
    }

    if(serialportHandle != nullptr)
    {
        CloseHandle(serialportHandle);
		serialportHandle = nullptr;
    }
}

int WzSerialportPlus::send(char* data,int length)
{
	DWORD lengthSent = -1; 

	BOOL bWriteStat = WriteFile(serialportHandle, 
		data,
		length,
		&lengthSent,
		NULL);
	if (bWriteStat)
	{
		return lengthSent;
	}
	else 
	{
		return 0;
	}
}

void WzSerialportPlus::setReceiveCalback(ReceiveCallback receiveCallback)
{
    this->receiveCallback = receiveCallback;
}

void WzSerialportPlus::onReceive(char* data,int length)
{

}
