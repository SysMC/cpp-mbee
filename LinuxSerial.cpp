/* "Системы модули и компоненты" ("СМК"). 2020. Москва.
Библиотека C++ для модулей MBee .
Распространяется свободно. Надеемся, что программные продукты, созданные
с помощью данной библиотеки будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library.
Copyright © 2020 Systems, modules and components. Moscow. Russia.

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

#include <unistd.h> 
#include <fcntl.h> 
#include <termios.h> 
#include <sys/ioctl.h>

#include "includes/System.h"

/**
Функция для получения константы скорости, определенной в termios.h. 
*/
speed_t getBitrateConstant(unsigned long bitrate);

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
    int bytes;
    ioctl(*(int*)_portHandler, FIONREAD, &bytes);
    return bytes;
} 

uint8_t Serial::read()
{
    uint8_t byte;
    ::read(*(int*)_portHandler, &byte, 1);//Получаем 1 байт из буфера COM-порта.
    return byte;
}

void Serial::write(uint8_t byte)
{
    ::write(*(int*)_portHandler, &byte, 1); //Пишем 1 байт в буфер COM-порта.
}

bool Serial::begin(unsigned long bitrate)
{
    struct termios options;
    static int sfd = open(_portName, O_RDWR | O_NOCTTY); 
    if(sfd == -1)
        return false;  
    _portHandler = &sfd;
    tcgetattr(*(int*)_portHandler, &options);
    cfsetspeed(&options, getBitrateConstant(bitrate));
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;
    cfmakeraw(&options);
    tcsetattr(*(int*)_portHandler, TCSANOW, &options);
    tcflush(*(int*)_portHandler, TCIOFLUSH);
    return true;
}

void Serial::end()
{
    ::close(*(int*)_portHandler);
}

speed_t getBitrateConstant(unsigned long bitrate)
{
    switch(bitrate)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    default:
        return B9600;
    }
}

