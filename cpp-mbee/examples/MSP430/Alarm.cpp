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
#include "driverlib.h"
#include "board.h"
#include "MBee.h" //Подключаем С++ библиотеку для работы с радиомодулями.
#include "Alarm.h"
#include "macros.h"

#if defined(ENABLE_CONSOLE)
  #include "console.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//Значения кодов событий. Представляют собой номера битов переменной events. Число событий не должно превышать 16.
#define BUTTON_PRESSED_EVENT            0
#define IO_SAMPLE_PACKET_RECEIVED_EVENT  1

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
static Serial serial("UART"); //Объект для работы с COM-портом. Имя может быть произвольное.
static RxIoSampleResponse packet; //Объект для принятого пакета с данными о состоянии линий ввхода/вывода удаленного радиомодуля.
static volatile  uint16_t events; //Битовое поле с флагами событий.
#if defined(ENABLE_CONSOLE)
 Console console; //Объект для интерактивного управления радиомодулем и вывода диагностических сообщений.
#endif

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static void switchOffAlarm(void);
static void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter); //Callback-функция, вызываемая библиотекой cpp-mbee при приеме пакета 0x83.
static void parseIoSamplePacket(void);
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
В данном примере микроконтроллер MSP430 работает в качестве автономного сигнализатора беспроводной сети тревожных датчиков.
В качестве беспроводных датчиков могут быть использованы датчики протечки, пожарно-охраннные извещатели, датчики газа, разбития стекла и тому подобные устройства
с цифровым выходом.
Для организации беспроводной сети применяются радиомодули MBee868-xx, работающие под управлением программного обеспечения SerialStar.
Все узлы сети должны иметь совпадающие сетевые настройки и эфирные параметры.
Модули, подключаемые к датчикам, должны иметь собственные адреса (параметр MY), лежащие в диапазоне SENSOR_ADDRESSES_START..SENSOR_ADDRESSES_END. Обычно такие датчики имеют автономное питание и поэтому должны
большую часть времени находится в спящем режиме. Отправка ими пакета, содержащего данные о состоянии линий ввода/вывода может осуществляться по срабатыванию датчика (параметр SM, должен быть установлен, равным 1),
по внутреннему таймеру(SM = 4) или в комбинированном режиме (SM = 5). Описание режимов сна находится в техническом описании программного обеспечения SerialStar.
Радиомодуль, подключаемый к микроконтроллеру MSP430 сигнализатора, должен работать в пакетном режиме (AP = 2). Битовая скорость на нем (устанавливается с помощью программы SysMCBootLoader)
должна совпадать со скоростью, определяемой параметром MBEE_UART_BITRATE.
Сигнализатор осуществляет прием пакетов (пакет 0x83) с данными о состоянии линий ввода/вывода от датчиков и фильтрует их на принадлежность заданному диапазону.
Если в пакете содержится информация о состоянии вывода радиомодя, определяемого константой ALARM_INPUT и если данный вывод используется как цифровой вход, то Сигнализатор
управляет своим выводом, задаваемым параметрами GPIO_PORT_LED1 и GPIO_PIN_LED1 (файл board.h) в зависимости от состояния входа ALARM_INPUT.
Тревожный сигнал на выходе сигнализатора может быть временно (до прихода очередного пакета) выключен кнопкой S1, настройки которой также определяются в файле board.h.
Если проект компилируется с опцией ENABLE_CONSOLE, то второй модуль eUSCI, имеющийся в микроконтроллере MSP430 используется для выдачи диагностических сообщений, а также как
альтернативный управляющий интерфейс. Отправив на UART консоли символ '1' можно временно выключить выходной сигнал также, как это происходит при нажатии кнопки S1.
Настройки параметров консоли осуществляются в файле alarm.h.
*/
int main( void )
{
  WDT_A_hold(WDT_A_BASE); //Отключаем встроенный watchdog-таймер во избежание его таймаута во время инициализации.
  hardwareInit();
  __enable_interrupt(); //Разрешаем прерывания, необходимые для работы модулей UART.
  serial.begin(MBEE_UART_BITRATE);
  MBee.begin(serial);
  #if defined(ENABLE_CONSOLE)
    console.begin(CONSOLE_UART_BITRATE);
  #endif
  MBee.onRxIoSampleResponse(ioSamplePacketReceived); //Регистрация callback-функции для приема пакетов 0x83 с состоянием линий ввода-вывода.
main_loop:
  MBee.run();
  #if defined(ENABLE_CONSOLE)
    switch(console.run())
    {
      case '1':
        switchOffAlarm();
        break;
      default:
        break;
    }
  #endif
  if(TEST_EVENT(BUTTON_PRESSED_EVENT))
  {
    RESET_EVENT(BUTTON_PRESSED_EVENT);
    switchOffAlarm();
  }
  if(TEST_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT))
  {
    RESET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
    parseIoSamplePacket();
  }
  goto main_loop;
}

void switchOffAlarm(void)
{
  GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1); //Выключаем цифровой выход "Тревога".
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
  packet = ioSample;
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
  //Прием каждого пакета индицируем светодиодом.
  ledBlinking(GREEN, 1);
  if((packet.getRemoteAddress() >= SENSOR_ADDRESSES_START) && (packet.getRemoteAddress() <= SENSOR_ADDRESSES_END)) //Фильтруем входящий пакет по диапазону адресов.
  {
    if(packet.getSampleSize()) //Содержится ли в принятом пакете информация о состоянии линий ввода/вывода удаленного модема?
    {
      sampleNumber = packet.getSampleNumber(ALARM_INPUT);
      if(sampleNumber) //Есть ли в принятом пакете информация о состоянии вывода радиомодуля, к которому подключен цифровой датчик и какой номер выборки он имеет?
      {
        if((packet.getMode(sampleNumber) & 0x7F) == IO_DIGITAL_INPUT) //Настроен ли данный вывод как цифровой вход?
        {
          if(packet.getDigital(sampleNumber) == LOW)
          {
            GPIO_setOutputHighOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1); //Вход на удаленном модуле замкнут. Включаем цифровой выход "Тревога".
          }
          else
          {
            GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1); //Вход на удаленном модуле разомкнут. Выключаем цифровой выход "Тревога".
          }
        }
      }
    }
  }
  #if defined(ENABLE_CONSOLE)
    console.printIoSamplePacket(packet);
  #endif
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
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_XIN, GPIO_PIN_XIN + GPIO_PIN_XOUT, GPIO_FUNCTION_XIN); //Настраиваем выводы, к которым подключен кварцевый резонатор 32768 Гц.
  GPIO_setAsPeripheralModuleFunctionInputPin(MBEE_UART_PORT_NUM, MBEE_UART_RX_BIT_NUM, GPIO_FUNCTION_UCA0RXD); //Настройка выводов, используемых модулем UART для соединения с радиомодулем MBee.
  GPIO_setAsPeripheralModuleFunctionOutputPin(MBEE_UART_PORT_NUM, MBEE_UART_TX_BIT_NUM, GPIO_FUNCTION_UCA0TXD);
  #if defined(ENABLE_CONSOLE)
    GPIO_setAsPeripheralModuleFunctionInputPin(CONSOLE_UART_PORT_NUM, CONSOLE_UART_RX_BIT_NUM, GPIO_FUNCTION_CONSOLE_UCAxRXD); //Настройка выводов, используемых модулем UART для организации консоли.
    GPIO_setAsPeripheralModuleFunctionOutputPin(CONSOLE_UART_PORT_NUM, CONSOLE_UART_TX_BIT_NUM, GPIO_FUNCTION_CONSOLE_UCAxTXD);
  #endif
  GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1); //Настройка вывода, управляющего первым светодиодом.
  GPIO_setAsOutputPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
  GPIO_setOutputLowOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2); //Настройка вывода, управляющего первым светодиодом.
  GPIO_setAsOutputPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_S1, GPIO_PIN_S1); //Настраиваем, вывод, к которому подключена первая кнопка.
  GPIO_enableInterrupt(GPIO_PORT_S1, GPIO_PIN_S1);
  GPIO_selectInterruptEdge(GPIO_PORT_S1, GPIO_PIN_S1, GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_clearInterrupt(GPIO_PORT_S1, GPIO_PIN_S1);

  PMM_unlockLPM5(); //Активируем настройки портов.
  CS_turnOnXT1LFWithTimeout(CS_XT1_DRIVE_3 , CS_XT1_TIMEOUT); //Включение кварцевого генератора с самым высоким усилением и, соответственно, с самой высокой помехоустойчивостью и более высоким потреблением.
  CS_initClockSignal(CS_FLLREF, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1 ); //В качестве источника тактового сигнала DCO выбираем кварцевый генератор 32768 Гц без входного деления частоты.
  CS_initFLL(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ, CS_MCLK_FLLREF_RATIO); //Устанавливаем требуемую частоту сигнала DCO. Эта же функция выбирает DCO как источник и MCLK и SMCLK. Для работы библиотеки cpp-mbee требуется,чтобы тактовая частота SMCLK была равной 8 МГц.
  CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1); //Для примера для ACLK выбираем кварцевый генератор 32768 Гц.

  //Настройка таймера Timer1_B3, используемого для формирования задержек. Настройка осуществляется только частично так как старт таймера и часть настроек будет выполняться непосредственно в функциях задержки.
  Timer_B_initUpModeParam timerbConfig = {0};
  timerbConfig.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
  timerbConfig.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_8; //Тактовую частоту Timer1_B3 устанавливаем равной 1 МГц.
  Timer_B_initUpMode(TIMER_B1_BASE, &timerbConfig);
}

//*****************************************************************************
//
//! \brief Включает требуемый светодиод заданное число раз.
//!
//! Код цвета светодиода.
//! Число включений светодиода.
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
        GPIO_setOutputHighOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
        delayMs(LED_ON_TIME);
        GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
        delayMs(LED_ON_TIME);
      break;
      case GREEN:
        GPIO_setOutputHighOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
        delayMs(LED_ON_TIME);
        GPIO_setOutputLowOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
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
//! таймер Timer1_B3.
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
    TB1CTL &= ~(MC_1 | MC_2); //Останавливаем таймер на случай, если он был включен.
    TB1CTL |= TBCLR; //Сбрасываем TB0 и делители.
    TB1CCR0 = (MAX_DELAY_US - 1);
    TB1CTL |= MC__UP; //Запускаем таймер в UP mode.
    TB1CCTL0 &= ~CCIFG;
    do
    {
      ENTER_CRITICAL_SECTION(intState);
      while(!(TB1CCTL0 & CCIFG));
      TB1CCTL0 &= ~CCIFG;
      EXIT_CRITICAL_SECTION(intState);
    }while(count--);
    TB1CTL &= ~(MC_1 | MC_2);
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
//! \brief Обработка прерывания от кнопки S1.
//!
//! \return NONE
//
//*****************************************************************************
#if GPIO_PORT_S1 == GPIO_PORT_P1
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=PORT1_VECTOR
  __interrupt
  #elif defined(__GNUC__)
  __attribute__((interrupt(PORT1_VECTOR)))
  #endif
  void P1_ISR (void)
#elif GPIO_PORT_S1 == GPIO_PORT_P2
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=PORT2_VECTOR
  __interrupt
  #elif defined(__GNUC__)
  __attribute__((interrupt(PORT2_VECTOR)))
  #endif
  void P2_ISR (void)
#elif GPIO_PORT_S1 == GPIO_PORT_P3
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=PORT3_VECTOR
  __interrupt
  #elif defined(__GNUC__)
  __attribute__((interrupt(PORT3_VECTOR)))
  #endif
  void P3_ISR (void)
#elif GPIO_PORT_S1 == GPIO_PORT_P4
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=PORT4_VECTOR
  __interrupt
  #elif defined(__GNUC__)
  __attribute__((interrupt(PORT4_VECTOR)))
  #endif
  void P4_ISR (void)
#elif GPIO_PORT_S1 == GPIO_PORT_P5
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=PORT5_VECTOR
  __interrupt
  #elif defined(__GNUC__)
  __attribute__((interrupt(PORT5_VECTOR)))
  #endif
  void P5_ISR (void)
#elif GPIO_PORT_S1 == GPIO_PORT_P6
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=PORT6_VECTOR
  __interrupt
  #elif defined(__GNUC__)
  __attribute__((interrupt(PORT6_VECTOR)))
  #endif
  void P6_ISR (void)
#endif // #if GPIO_PORT_S1
  {
    SET_EVENT(BUTTON_PRESSED_EVENT);
    GPIO_clearInterrupt(GPIO_PORT_S1, GPIO_PIN_S1);
  }
