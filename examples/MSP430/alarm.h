/* --COPYRIGHT--
 *
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// Alarm.h - заголовочный файл для alarm.cpp.
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
#define ENABLE_CONSOLE //Закомменировать, если консоль не используется. В этим случае контроль работы осуществляется только с помощью светодиодов.

#define ALARM_INPUT 4 //Номер вывода радиомодуля MBee-868-xx, к которому на удаленных узлах подключен цифровой датчик.
#define SENSOR_ADDRESSES_START 0x0002 //Начальный адрес диапазона адресов, назначенных датчикам. Используется для фильтрации входящих пакетов.
#define SENSOR_ADDRESSES_END   0x000A //Конечный адрес диапазона адресов, назначенных датчикам. Используется для фильтрации входящих пакетов.

#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ   8000 //Чтобы не вводить такты ожидания при обращении к FRAM, тактовую тактовую частоту MCLK устанавливаем равной 8 МГц.
#define CS_MCLK_FLLREF_RATIO ((uint16_t)(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ / 32.768))

#define MBEE_UART_BITRATE 9600 //Выбираем скорость последовательного порта, с которой осуществляется обмен с радиомодулем MBee.

#define LOCAL_RESPONSE_TIMEOUT 60 //Время ожидания локального статуса отправки пакета.
#define ACKNOWLEDGE_TIMEOUT 100 //Время ожидания подтверждения получения пакета от удаленного модема.

#define RED   1 //Код красного светодиода.
#define GREEN 2 //Код зеленого светодиода.
#define LED_ON_TIME 50 //Время, на которое включается светодиод при индикации какого-либо события. Миллисекунды.

#if defined(ENABLE_CONSOLE)
  #define CONSOLE_UART_BITRATE 115200
#endif

//Константа, определяющая максимальное время, когда программа может находится внутри безопасной секции. Должна быть аккуратно подобрана так, чтобы не пропустить два последовательных прерывания от одного и
//того же источника прерываний.
#define MAX_DELAY_US  20 //Микросекунд.

#if MSP_MBEE_UART_MODULE == EUSCI_A0_BASE
  #define MBEE_UART_PORT_NUM   GPIO_PORT_UCA0TXD
  #define MBEE_UART_RX_BIT_NUM GPIO_PIN_UCA0RXD
  #define MBEE_UART_TX_BIT_NUM GPIO_PIN_UCA0TXD
  #if defined(ENABLE_CONSOLE)
    #define MSP_CONSOLE_UART_MODULE EUSCI_A1_BASE
    #define GPIO_FUNCTION_CONSOLE_UCAxRXD GPIO_PRIMARY_MODULE_FUNCTION
    #define GPIO_FUNCTION_CONSOLE_UCAxTXD GPIO_PRIMARY_MODULE_FUNCTION
    #define CONSOLE_UART_PORT_NUM   GPIO_PORT_P4
    #define CONSOLE_UART_RX_BIT_NUM GPIO_PIN2
    #define CONSOLE_UART_TX_BIT_NUM GPIO_PIN3
  #endif
#elif MSP_MBEE_UART_MODULE == EUSCI_A1_BASE
  #define MBEE_UART_PORT_NUM   GPIO_PORT_P4
  #define MBEE_UART_RX_BIT_NUM GPIO_PIN2
  #define MBEE_UART_TX_BIT_NUM GPIO_PIN3
  #if defined(ENABLE_CONSOLE)
    #define MSP_CONSOLE_UART_MODULE EUSCI_A0_BASE
    #define GPIO_FUNCTION_CONSOLE_UCAxRXD GPIO_PRIMARY_MODULE_FUNCTION
    #define GPIO_FUNCTION_CONSOLE_UCAxTXD GPIO_PRIMARY_MODULE_FUNCTION
    #define CONSOLE_UART_PORT_NUM   GPIO_PORT_UCA0TXD
    #define CONSOLE_UART_RX_BIT_NUM GPIO_PIN_UCA0RXD
    #define CONSOLE_UART_TX_BIT_NUM GPIO_PIN_UCA0TXD
  #endif
#else
  #error UART module must be defined!
#endif

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
