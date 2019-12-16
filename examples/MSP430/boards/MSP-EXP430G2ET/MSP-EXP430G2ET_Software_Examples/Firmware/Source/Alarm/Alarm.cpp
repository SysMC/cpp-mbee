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
/*********************************************************************
 * INCLUDES
 */
#include "msp430.h"
#include "BOOSTXL-MBEE868-1.1.h"
#include "MBee.h" //Подключаем С++ библиотеку для работы с радиомодулями.
#include "Alarm.h"
#include "macros.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//Значения кодов событий. Представляют собой номера битов переменной events. Число событий не должно превышать 16.
#define BUTTON1_PRESSED_EVENT           0
#define DIP1_CHANGE_EVENT               1
#define IO_SAMPLE_PACKET_RECEIVED_EVENT 2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static SerialStarWithCallbacks MBee; //Объект для взаимодействия с радиомодулем с помощью callback-функций.
static RemoteAtCommandRequest remoteCommand; //Объект для организации отправки команд удаленным узлам.
static Serial serial("UART"); //Объект для работы с COM-портом. Имя может быть произвольное.
static RxIoSampleResponse ioPacket; //Объект для принятого пакета с данными о состоянии линий ввода/вывода удаленного радиомодуля.
static uint16_t remoteModuleAddress; //Адрес удаленного радиомодуля от которого получен пакет.
static volatile  uint16_t events; //Битовое поле с флагами событий.

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static void switchOffAlarm(void);
static void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter); //Callback-функция, вызываемая библиотекой cpp-mbee при приеме пакета 0x83.
static void parseIoSamplePacket(void);
static void sendCommandToRemoteModule(void);
static void hardwareInit(void);
static void ledBlinking(uint8_t led, uint8_t count);
static void delayUs(uint16_t us);
static void delayMs(uint16_t ms);

/*********************************************************************
 * LOCAL FUNCTIONS
 *********************************************************************/
/**
<b>Пример использования библиотеки cpp-mbee.</b>
\n
Сигнализатор осуществляет прием пакетов (пакет 0x83) с данными о состоянии линий ввода/вывода от датчиков и фильтрует их на принадлежность заданному диапазону адресов.
Если в пакете содержится информация о состоянии вывода радиомодя, определяемого константой ALARM_INPUT и если данный вывод используется как цифровой вход, то сигнализатор
управляет своим P2.4 в зависимости от состояния входа ALARM_INPUT.
Тревожный сигнал на выходе сигнализатора может быть временно (до прихода очередного пакета) выключен кнопкой BUTTON1.
В примере демонстрируется также возможность отправки сообщений в направлении от сигнализатора к удаленному датчику, находящемуся в спящем режиме. Для этого используется первый контакт DIP-переключателя. При переводе
его в положение ON на удаленный модуль отправляется команда перевода цифрового выхода ACTUATOR_OUTPUT в высокий уровень. Для отключения выхода ACTUATOR_OUTPUT необходимо перевести первый контакт
DIP-переключателя в положение OFF.
*/
int main( void )
{
  WDTCTL = WDTPW + WDTHOLD; //Отключаем встроенный watchdog-таймер во избежание его таймаута во время инициализации.
  hardwareInit();
  __enable_interrupt(); //Разрешаем прерывания, необходимые для работы модулей UART.
  serial.begin(MBEE_UART_BITRATE);
  MBee.begin(serial);
  MBee.onRxIoSampleResponse(ioSamplePacketReceived); //Регистрация callback-функции для приема пакетов 0x83 с состоянием линий ввода-вывода.
  //Устанавливаем неизменяемые в процессе работы поля в объекте, предназначенном для отправки команд удаленному модулю.
  remoteCommand.setDefault(); //Приводим все поля команды для передачи к значениям "по умолчанию" для демонстрации наличия этой функции. Таким образом упрощается повторное использование объекта для отправки последующих команд.
  remoteCommand.setAcknowledge(false); //Команды удаленному модулю будут отправляться без подтверждения.
  remoteCommand.setApplyChanges(true); //Команды будет выполняться сразу после приема.
  remoteCommand.setSaveChanges(false); //Значение параметра, переданное в команде не будет сохраняться в энергонезависимой памяти удаленного радиомодуля.
  remoteCommand.setCca(false); //Для того, чтобы команда была гарантировано передана в интервал времени, в течение которого удаленный модуль находится в состоянии приема, отключаем функцию CCA (Clear Channel Assessment).
  remoteCommand.setEncryption(false); //Отключаем функцию шифрования. Вызов здесь этой функции включен только для демонстрации, поскольку по-умолчанию шифрование отключено.
  remoteCommand.setSleepingDevice(true); //Команда предназначается спящему удаленному узлу, поэтому включаем буферизацию.
  remoteCommand.setCommand((uint8_t*)ACTUATOR_OUTPUT); //Установка кода команды удаленному модулю.
  remoteCommand.setCommandValueLength(1); //Длина команды управления режимом вывода равна 1 байт.
  remoteCommand.setFrameId(0); //Отключаем локальное подтверждение отправки команды в эфир, устанавливая FrameId равным 0.
main_loop:
  MBee.run();
  if(TEST_EVENT(BUTTON1_PRESSED_EVENT))
  {
    RESET_EVENT(BUTTON1_PRESSED_EVENT);
    switchOffAlarm();
  }
  if(TEST_EVENT(DIP1_CHANGE_EVENT))
  {
    RESET_EVENT(DIP1_CHANGE_EVENT);
    sendCommandToRemoteModule();
  }
  if(TEST_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT))
  {
    RESET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
    parseIoSamplePacket();
  }
  goto main_loop;
}

//*****************************************************************************
//
//! \brief Выключение цифрового выхода "Тревога".
//!
//! \return NONE
//
//*****************************************************************************
void switchOffAlarm(void)
{
  P2OUT &= ~BIT4; //Выключаем цифровой выход "Тревога".
}

//*****************************************************************************
//
//! \brief Callback-функция, вызываемая при приеме пакета 0x83.
//! Так как callback-функция должна быть максимально короткая и не содержать задержек, то
//! в ней просто выставляем событие и копируем принятый пакет.
//!
//! \return NONE
//
//*****************************************************************************
void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter)
{
  ioPacket = ioSample;
  SET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
}

//*****************************************************************************
//
//! \brief Разбор принятого пакета 0x83.
//!
//! \return NONE
//
//*****************************************************************************
void parseIoSamplePacket(void)
{
  uint8_t sampleNumber;
  ledBlinking(GREEN, 1);  //Прием каждого пакета индицируем светодиодом.
  remoteModuleAddress = ioPacket.getRemoteAddress(); //Сохраняем адрес модуля от которого приняли пакет.
  if((remoteModuleAddress >= SENSOR_ADDRESSES_START) && (remoteModuleAddress <= SENSOR_ADDRESSES_END)) //Фильтруем входящий пакет по диапазону адресов.
  {
    if(ioPacket.getSampleSize()) //Содержится ли в принятом пакете информация о состоянии линий ввода/вывода удаленного модема?
    {
      sampleNumber = ioPacket.getSampleNumber(ALARM_INPUT);
      if(sampleNumber) //Есть ли в принятом пакете информация о состоянии вывода радиомодуля, к которому подключен цифровой датчик и какой номер выборки он имеет?
      {
        if((ioPacket.getMode(sampleNumber) & 0x7F) == IO_DIGITAL_INPUT) //Настроен ли данный вывод как цифровой вход?
        {
          if(ioPacket.getDigital(sampleNumber) == LOW)
          {
            P2OUT |= BIT4; //Вход на удаленном модуле замкнут. Включаем цифровой выход "Тревога" (светодиод LAMP1).
          }
          else
          {
            P2OUT &= ~BIT4; //Вход на удаленном модуле разомкнут. Выключаем цифровой выход "Тревога" (светодиод LAMP1).
          }
        }
      }
    }
  }
}

//*****************************************************************************
//
//! \brief Отправка команды удаленному радиомодулю.
//!
//! \return NONE
//
//*****************************************************************************
void sendCommandToRemoteModule(void)
{
  uint8_t parameter;
  remoteCommand.setRemoteAddress(remoteModuleAddress); //Устанавливаем адрес получателя совпадающим с адресом отправителя.
  if(P2IN & BIT7) //В зависимости от состояния DIP-переключателя, устанавливаем высокий или низкий уровень на цифровом выходе удаленного модуля.
  {
    parameter = IO_DIGITAL_OUTPUT_LO;
  }
  else
  {
    parameter = IO_DIGITAL_OUTPUT_HI;
  }
  remoteCommand.setCommandValue(&parameter);
  MBee.send(remoteCommand);
}

//*****************************************************************************
//
//! \brief Инициализация всех линий ввода/вывода и периферийных модулей.
//!
//! \return NONE
//
//*****************************************************************************
void hardwareInit(void)
{
  BCSCTL1 = CALBC1_8MHZ; //Задаем частоту встроенного генератора равной 8 МГц.
  DCOCTL = CALDCO_8MHZ;
  P1SEL = BIT1 + BIT2; //Настройка выводов, используемых модулем UART для соединения с радиомодулем MBee.
  P1SEL2 = BIT1 + BIT2;
  P1OUT = BIT6; //Подключаем подтягивающий резистор кнопки BUTTON1 к Ucc.
  P1REN = BIT6; //Включаем подтягивающий резистор кнопки BUTTON1.
  P1IES = 0x00; //Прерывания по линии BUTTON1 по отрицательному фронту.
  P1IE = BIT6; //Разрешаем прерывания по входу BUTTON1.
  P1IFG = 0x00; //Сбрасываем флаги прерывания.
  P2SEL = 0x00; //Отключаем кварцевый резонатор.
  P2OUT = BIT7; //Настройка вывода, управляющего первым светодиодом и вторым светодиодом, а также подключаем резистор DIP-переключателя к Ucc.
  P2DIR = BIT4 + BIT6; //Линии LAMP1 и LAMP2 включаем на вывод.
  P2REN = BIT7; //Подключаем подтягивающий резистор к выводу DIP-переключателя.
  P2IES = 0x00; //Прерывания по линии DIP-переключателя по отрицательному фронту.
  P2IE = BIT7; //Разрешаем прерывания по входу DIP-переключателя.
  P2IFG = 0x00; //Сбрасываем флаги прерывания.
  //Настройка таймера Timer1_A3, используемого для формирования задержек. Настройка осуществляется только частично так как старт таймера и часть настроек будет выполняться непосредственно в функциях задержки.
  TA1CTL = TASSEL_2 + ID_3; //Тактовую частоту Timer1_A3 устанавливаем равной 1 МГц. Источник тактового сигнала - SMCLK, делитель на 8.
}

//*****************************************************************************
//
//! \brief Включает требуемый светодиод заданное число раз.
//!
//! \param Код цвета светодиода.
//! \param Число включений светодиода.
//!
//! \return NONE
//
//*****************************************************************************
void ledBlinking(uint8_t led, uint8_t count)
{
  while(count--)
  {
    switch(led)
    {
      case RED:
        P2OUT |= BIT4;
        delayMs(LED_ON_TIME);
        P2OUT &= ~BIT4;
        delayMs(LED_ON_TIME);
      break;
      case GREEN:
        P2OUT |= BIT6;
        delayMs(LED_ON_TIME);
        P2OUT &= ~BIT6;
        delayMs(LED_ON_TIME);
      break;
    }
  }
}

//*****************************************************************************
//
//! \brief Задержка микроcекундного диапазона.
//!
//! Обеспечивает thread-безопасную задержку миллисекундного диапазона. Использует
//! таймер Timer1_Ax.
//!
//! \param us Микросекунды.
//!
//! \return NONE
//
//*****************************************************************************
void delayUs(uint16_t us)
{
  uint16_t intState;
  uint16_t count = us/MAX_DELAY_US;
  if(us)
  {
    TA1CTL &= ~(MC_1 | MC_2); //Останавливаем таймер на случай, если он был включен.
    TA1CTL |= TACLR; //Сбрасываем TA1 и делители.
    TA1CCR0 = (MAX_DELAY_US - 1);
    TA1CTL |= MC_1; //Запускаем таймер в UP mode.
    TA1CCTL0 &= ~CCIFG;
    do
    {
      ENTER_CRITICAL_SECTION(intState);
      while(!(TA1CCTL0 & CCIFG));
      TA1CCTL0 &= ~CCIFG;
      EXIT_CRITICAL_SECTION(intState);
    }while(count--);
    TA1CTL &= ~(MC_1 | MC_2);
  }
}

//*****************************************************************************
//
//! \brief Задержка миллиcекундного диапазона.
//!
//! Обеспечивает thread-безопасную задержку миллисекундного диапазона. Использует
//! таймер Timer1_B3.
//!
//! \param ms Миллисекунды.
//!
//! \return NONE
//
//*****************************************************************************
void delayMs(unsigned int ms)
{
  while(ms)
  {
    delayUs(1000);
    ms--;
  }
}

//*****************************************************************************
//
//! \brief Обработка прерываний при изменении уровня на входах порта 1.
//!
//! \return NONE
//
//*****************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT1_VECTOR)))
#endif
void P1_ISR (void)
{
  if(P1IFG & BIT6)
  {
    SET_EVENT(BUTTON1_PRESSED_EVENT);
    P1IFG &= ~BIT6;
  }
}

//*****************************************************************************
//
//! \brief Обработка прерываний при изменении уровня на входах порта 2.
//!
//! \return NONE
//
//*****************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT2_VECTOR)))
#endif
void P2_ISR (void)
{
  if(P2IFG & BIT7)
  {
    SET_EVENT(DIP1_CHANGE_EVENT);
    P2IES ^= BIT7; //Меняем активный фронт прерывания на противоположный.
    P2IFG &= ~BIT7;
  }
}
