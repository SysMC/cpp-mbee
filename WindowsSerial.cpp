/* "Системы модули и компоненты" ("СМК"). 2018. Москва.
Библиотека C++ для модулей MBee .
Распространяется свободно. Надеемся, что программные продукты, созданные
с помощью данной библиотеки будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library.
Copyright © 2018 Systems, modules and components. Moscow. Russia.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Code adapted from  XBee-Arduino library XBee.h. Copyright info below.
* @file       XBee.h
* @author     Andrew Rapp
* @license    This project is released under the GNU License
* @copyright  Copyright (c) 2009 Andrew Rapp. All rights reserved
* @date       2009
* @brief      Interface to the wireless XBee modules
*/
#include <iostream>
using namespace std;
#include <windows.h>

#include "includes\System.h"

/**
Функция для получения константы скорости, определенной в windows.h. 
*/
DWORD getBitrateConstant(unsigned long bitrate);

/*********************************************************************
* Public методы класса Serial.
*********************************************************************/
Serial::Serial(const char* portName) 
{
    _portName = portName;
    SizeBuffer = 4096; //Размер приемного и передающего буфера.
}

int Serial::available()
{
    DWORD temp; 
    COMSTAT ComState;
    ClearCommError(_portHandler, &temp, &ComState);
    return ComState.cbInQue;
} 

uint8_t Serial::read()
{
    DWORD iSize;
    uint8_t byte;
    ReadFile(_portHandler, &byte, 1, &iSize, 0);  //Получаем 1 байт из буфера COM-порта.
    return byte;
}

void Serial::write(uint8_t byte)
{
    DWORD sendedChar;
    WriteFile(_portHandler, &byte, 1, &sendedChar, 0);
}

bool Serial::begin(unsigned long bitrate)
{
    COMMTIMEOUTS CommTimeOuts;
    _portHandler = CreateFile(_portName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if(_portHandler == INVALID_HANDLE_VALUE)
        return false;
    DCB dcb;
    dcb.DCBlength = sizeof(dcb);
    if(!GetCommState(_portHandler, &dcb))
        return false;
    SetupComm(_portHandler, SizeBuffer, SizeBuffer); 
    if(!GetCommState(_portHandler, &dcb))
        return false;
    dcb.BaudRate = getBitrateConstant(bitrate); 
    dcb.fBinary = TRUE; 
    dcb.fOutxCtsFlow = FALSE; 
    dcb.fOutxDsrFlow = FALSE; 
    dcb.fDtrControl = DTR_CONTROL_HANDSHAKE; 
    dcb.fDsrSensitivity = FALSE; 
    dcb.fNull = FALSE; 
    dcb.fRtsControl = RTS_CONTROL_DISABLE; 
    dcb.fAbortOnError = FALSE; 
    dcb.ByteSize = 8; 
    dcb.Parity = NOPARITY; 
    dcb.StopBits = ONESTOPBIT; 
    if(!SetCommState(_portHandler, &dcb))
        return false;
    CommTimeOuts.ReadIntervalTimeout= 10; 
    CommTimeOuts.ReadTotalTimeoutMultiplier = 1; 
    //Значений этих тайм – аутов вполне хватает для уверенного приема даже на скорости 110 бод.
    CommTimeOuts.ReadTotalTimeoutConstant = 100; 
    //Используется в данном случае как время ожидания посылки.
    CommTimeOuts.WriteTotalTimeoutMultiplier = 0; 
    CommTimeOuts.WriteTotalTimeoutConstant = 0; 
    if(!SetCommTimeouts(_portHandler, &CommTimeOuts))
        return false;
    PurgeComm(_portHandler, PURGE_RXCLEAR); //Чистим входной буфер от мусора.
    PurgeComm(_portHandler, PURGE_TXCLEAR); //Чистим выходной буфер от мусора.
    return true;
}

void Serial::end()
{
    CloseHandle(_portHandler);
}

DWORD getBitrateConstant(unsigned long bitrate)
{
    switch(bitrate)
    {
    case 9600:
        return CBR_9600;
    case 19200:
        return CBR_19200;
    case 38400:
        return CBR_38400;
    case 57600:
        return CBR_57600;
    case 115200:
        return CBR_115200;
    case 230400:
        return 230400; //Константа CBR_230400 в windows.h не определена.
    default:
        return CBR_9600;
    }
}

