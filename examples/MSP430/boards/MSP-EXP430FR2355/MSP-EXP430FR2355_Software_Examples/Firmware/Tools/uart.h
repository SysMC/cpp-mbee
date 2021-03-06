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
// uart.h - Заголовочный файл драйвера UART uart.c.
//
//*****************************************************************************
#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include "inc/hw_memmap.h"
#include <stdint.h>
#include <stdlib.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define CONSOLE_UART_TX_BUFFER_SIZE       512
#define CONSOLE_UART_RX_BUFFER_SIZE       512

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */
//*****************************************************************************
//
//! \brief Инициализация модуля eUSCI для работы  в режиме UART.
//!
//! Записывает константы, определяющие режим работы и битовыую скорость в регистры
//! соответствующего модуля eUSCI без использования библиотеки driverlib.
//! При расчете константы для регистров использовалась утилита
//! MSP430 USCI/EUSCI UART Baud Rate Calculation, доступная на сайте Texas Insruments поиском по названию.
//!
//!
//! \return NONE.
//
//*****************************************************************************
void uartInit(uint32_t bitrate);

//*****************************************************************************
//
//! \brief Проверка приемного буфера UART.
//!
//! Возвращает число байт в приемном буфере UART.
//!
//!
//! \return uint16_t - число байт в приемном буфере UART.
//
//*****************************************************************************
uint16_t uartAvailable(void);

//*****************************************************************************
//
//! \brief Очистка приемного буфера UART.
//!
//! \return NONE.
//
//*****************************************************************************
void uartFlush(void);

//*****************************************************************************
//
//! \brief Проверка окончания передачи буфера UART.
//!
//! Ожидает окончания передачи последнего бита на физическом выводе.
//!
//! \return NONE.
//
//*****************************************************************************
void uartWaitTxComplete(void);

//*****************************************************************************
//
//! \brief Передача пакета данных в UART.
//!
//! Осуществляет запись пакета данных в передающий буфер UART и активирует передачу
//! только после записи всего пакета. Если пакет не помещается в буфер, то происходит
//! его обрезание. Передача инициализируется только после записи всего пакета. В
//! результате сокращается время работы функции.
//!
//! \param buf_p const void* - указатель на буфер, содержащий данные для передачи.
//! \param len uint16_t - число байт, которые надо передать.
//!
//! \return bool - статус выполнения.
//
//*****************************************************************************
bool uartTxWait(const void *buf_p, uint16_t len);

//*****************************************************************************
//
//! \brief Чтение данных из буфера UART.
//!
//! Считывает данные, находящиеся в приемном буфере UART.
//!
//! \param buf_p const void* указатель на буфер, куда будут записаны считанные из UART данные.
//! \param len uint16_t - максимальное число считываемых байт.
//!
//! \return uint16_t - число считанных байт.
//
//*****************************************************************************
uint16_t uartRx(const void *buf_p, uint16_t len);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif //UART_H
