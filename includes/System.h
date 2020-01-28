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

#ifndef System_h
#define System_h

#include <inttypes.h>

/**
Класс Stream является базовым для всех классов, связанных с обработкой последовательных данных.
все методы класса являются чисто виртуальными и должны быть обязательно определены в классах наследниках.
*/
class Stream
{
public:
    virtual int available() = 0;
    virtual uint8_t read() = 0;
    virtual void write(uint8_t val) = 0;
    //virtual bool flush() = 0;
    //virtual bool find(const char target) = 0;
    //virtual findUntil(const char target, const char terminal) = 0;
    //virtual uint8_t peek() = 0;
    //virtual int readBytes(uint8_t* buffer, int length) = 0;
    //virtual int readBytesUntil(char character, uint8_t* buffer, int length) = 0;
    //virtual char* readString(char terminator) = 0;
    //virtual char* readStringUntil() = 0;
    //virtual long parseInt(char* list) = 0;
    //virtual float parsefloat(char* list) = 0;
    //virtual void setTimeout(long timeout) = 0;
};

/**
Класс, расширяющий базовый класс Stream и предназначенный для работы с последовательным портом.
*/
class Serial : public Stream
{
public:
    /**
    Конструктор, создающий объект для работы с последовательным портом с заданным системным именем.
    */
    Serial(const char* portName);

    /**
    Деструктор.
    */
    ~Serial()
    {
        end();
    }

    /**
    Возвращает число байт, доступных в приемном буфере последовательного порта.
    */
    int available();

    /**
    Возвращает 1 байт из приемного буфера последовательного порта.
    */
    uint8_t read();

    /**
    Записывает 1 байт в выходной буфера последовательного порта, который немедленно передается.
    */
    void write(uint8_t val);

    /**
    Устанавливает скорость последовательного порта. Возвращает true, если порт успешно иниициализирован и требуемая битовая скорость установлена.
    false - порт не существует.
    */
    bool begin(unsigned long bitrate);

    /**
    Закрывает последовательный порт.
    */
    void end();

protected:
    int SizeBuffer;
    const char* _portName;
    void* _portHandler;
};

/**
Возвращает число миллисекунд, прошедшее с момента начала работы программы.
*/
uint32_t millis();

#endif //System_h
