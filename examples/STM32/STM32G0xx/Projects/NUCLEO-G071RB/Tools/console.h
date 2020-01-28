/* "Системы модули и компоненты" ("СМК"). 2020. Москва.
Библиотека C++ для модулей MBee. Пример использования библиотеки cpp-mbee на микроконтроллерах STM32 производства STMicroelectronics.
Распространяется свободно. Надеемся, что программные продукты, созданные
на основе данного примера будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library Example for STM32.
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
extern UART_HandleTypeDef * consoleUartHandle;

/*********************************************************************
 * CLASSES
 */

/**
Класс, обеспечивающий вывод на последовательный порт диагностических сообщений, а также прием команд управления.

\author von Boduen.
*/
class Console
{
public:
  /**
  Конструктор "по умолчанию".
  */
  Console(const char* portName);

    /**
  Деструктор.
  */
  ~Console()
  {
    end();
  }

  /**
  Настраивает модуль UART для работы в качестве консоли.
  */
  bool begin(uint32_t bitrate);

  /**
  Закрывает последовательный порт.
  */
  void end();

  /**
  Метод, который должен регулярно вызываться в главном цикле программы.
  Опрашивает последовательный порт и возвращает код поступившей из консоли команды.
  */
  uint8_t run();

  /**
  Метод, который выводит на последовательный порт содержание пакета с данными о состоянии линий ввода/вывода (пакет 0x83 или 0x84).
  */
  void printIoSamplePacket(RxIoSampleResponse& ioSample);

protected:
  const char* _portName;
 };

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif //CONSOLE_H
