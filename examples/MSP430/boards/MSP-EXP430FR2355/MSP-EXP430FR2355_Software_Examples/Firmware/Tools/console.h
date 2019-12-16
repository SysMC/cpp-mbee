/* "Системы модули и компоненты" ("СМК"). 2019. Москва.
Библиотека C++ для модулей MBee. Пример использования библиотеки cpp-mbee на микроконтроллерах MSP430 производства  Texas Instruments.
Распространяется свободно. Надеемся, что программные продукты, созданные
на основе данного примера будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library Example for MSP430.
Copyright © 2019 Systems, modules and components. Moscow. Russia.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//*****************************************************************************
//
// console.h - заголовочный файл для console.c.
//
//*****************************************************************************
#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include "uart.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * CLASSES
 */

/**
Класс, обеспечивающий вывод на последовательный порт диагностических сообщение, а также прием команд управления.

\author von Boduen.
*/
class Console
{
public:
  /**
  Конструктор "по умолчанию".
  */
  Console();

  /**
  Настраивает модуль eUSCI для работы в качестве UART консоли.
  */
  void begin(uint32_t bitrate);

  /**
  Метод, который должен регулярно вызываться в главном цикле программы.
  Опрашивает последовательный порт и возвращает код поступившей из консоли команды.
  */
  uint8_t run();

  /**
  Метод, который выводит на последовательный порт содержание пакета с данными о состоянии линий ввода/вывода (пакет 0x83).
  */
  void printIoSamplePacket(RxIoSampleResponse& ioSample);

  void print(char const  *buffer_p);
  void println(char const *buffer_p);
  void print(uint32_t parameter, uint8_t radix);
  void println(uint32_t parameter, uint8_t radix);


protected:

private:
  uint8_t* ultoa(uint32_t num, uint8_t *buf, uint8_t radix);
 };

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif //CONSOLE_H
