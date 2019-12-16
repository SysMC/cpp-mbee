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
// Alarm.h - заголовочный файл для Alarm.cpp.
//
//*****************************************************************************
#ifndef ALARM_H
#define ALARM_H

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
#define ALARM_INPUT 4 //Номер вывода радиомодуля MBee-868-xx, к которому на удаленных узлах подключен цифровой датчик.
#define ACTUATOR_OUTPUT "R4" //Идентификатор вывода радиомодуля MBee-868-xx, который на удаленных узлах используется в качестве цифрового выхода для управления каким-либо исполнительным устройством.
#define SENSOR_ADDRESSES_START 0x0002 //Начальный адрес диапазона адресов, назначенных датчикам. Используется для фильтрации входящих пакетов.
#define SENSOR_ADDRESSES_END   0x000A //Конечный адрес диапазона адресов, назначенных датчикам. Используется для фильтрации входящих пакетов.

#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ   8000 //Чтобы не вводить такты ожидания при обращении к FRAM, тактовую тактовую частоту MCLK устанавливаем равной 8 МГц.
#define CS_MCLK_FLLREF_RATIO ((uint16_t)(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ / 32.768))

#define MBEE_UART_BITRATE 9600 //Выбираем скорость последовательного порта, с которой осуществляется обмен с радиомодулем MBee.

#define RED   1 //Код красного светодиода.
#define GREEN 2 //Код зеленого светодиода.
#define LED_ON_TIME 50 //Время, на которое включается светодиод при индикации какого-либо события. Миллисекунды.

//Константа, определяющая максимальное время, когда программа может находится внутри безопасной секции. Должна быть аккуратно подобрана так, чтобы не пропустить два последовательных прерывания от одного и
//того же источника прерываний.
#define MAX_DELAY_US  20 //Микросекунд.

#define CS_XT1_TIMEOUT 50000 //Таймаут ожидания запуска кварцевого резонатора 32768 Гц. Значение взято из примеров к driverlib.

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif //ALARM_H
