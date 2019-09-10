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
#include "Control.h"
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
#define BUTTON_PRESSED_EVENT  0

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

#if defined(ENABLE_CONSOLE)
 Console console; //Объект для интерактивного управления радиомодулем и вывода диагностических сообщений.
#endif
static TxRequest tx; //Объект для пакета с данными для отправки удаленному модему. Для экономии памяти рекомендуется использовать один и тот же объект для отправки всех пакетов.
static RemoteAtCommandRequest remoteAtRequest; //Объект для пакета с AT-командой удаленному модулю.
static TxStatusResponse txStatus; //Объект для пакета с локальным ответом на AT-команду удаленному модулю.
static RemoteAtCommandResponse remoteAtResponse; //Объект для пакета с ответом от удаленного модуля.
static volatile  uint16_t events; //Битовое поле с флагами событий.
static uint8_t buttonMode; //Переменная, в которой хранится режим кнопки, оределяющий реакцию на ее нажатие.
static uint8_t cmdParam[2]; //Параметр команды. 2- максимальная длина параметра команды, которая будет использоваться в этом примере.
#if defined(ENABLE_CONSOLE)
  static int16_t brigthness; //Текущее значение яркости светодиода удаленного модуля.
#endif

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static void targetSetup(void);
static void targetDigitalOutSetHigh(void);
static void targetDigitalOutSetLow(void);
static void targetPWMOutSetDutyCycle(uint16_t dutyCycle);
static void sendCommand(void);
static void hardwareInit(void);
static void ledBlinking(uint8_t led, uint8_t count);
static void delayUs(uint16_t us);
static void delayMs(uint16_t ms);

/*********************************************************************
 * LOCAL FUNCTIONS
 *********************************************************************/
/*********************************************************************
 * LOCAL FUNCTIONS
 *********************************************************************/
/**
<b>Пример использования библиотеки cpp-mbee.</b>
\n
В данном примере микроконтроллер MSP430 выполняет роль пульта беспроводной системы управления освещением.
Для организации беспроводной сети применяются радиомодули MBee868-xx, работающие под управлением программного обеспечения SerialStar.
Все узлы сети должны иметь совпадающие сетевые настройки и эфирные параметры.
Радиомодуль, подключаемый к микроконтроллеру MSP430 пульта, должен работать в пакетном режиме (AP = 2). Битовая скорость на нем (устанавливается с помощью программы SysMCBootLoader)
должна совпадать со скоростью, определяемой параметром MBEE_UART_BITRATE.
Пульт осуществляет отправку AT-команд удаленному радиомодулю (target), адрес которого определяется константой TARGET_ADDRESS. Передача команд осуществляется кнопкой S1. Команды отправляются
циклически. Смысл каждой команды поясняется соответствующим комментарием.
Если проект компилируется с опцией ENABLE_CONSOLE, то второй модуль eUSCI, имеющийся в микроконтроллере MSP430 используется для выдачи диагностических сообщений, а также как
альтернативный управляющий интерфейс. Настройки параметров консоли осуществляются в файле control.h.
*/
int main( void )
{
  WDT_A_hold(WDT_A_BASE); //Отключаем встроенный watchdog-таймер во избежание его таймаута во время инициализации.
  hardwareInit();
  __enable_interrupt(); //Разрешаем прерывания, необходимые для работы модулей eUSCI в режиме UART.
  serial.begin(MBEE_UART_BITRATE);
  MBee.begin(serial);
  #if defined(ENABLE_CONSOLE)
    console.begin(CONSOLE_UART_BITRATE);
  #endif
main_loop:
  MBee.run();
  #if defined(ENABLE_CONSOLE)
    switch(console.run())
    {
      case '1':
        targetSetup(); //Осуществляем начальную настройку неизменяющихся полей команды. Должна передана в начале работы перед!
        break;
      case '2':
        targetDigitalOutSetHigh(); //Включаем дискретный цифровой выход на удаленном модуле.
        break;
      case '3':
        targetDigitalOutSetLow(); //Выключаем дискретный цифровой выход на удаленном модуле.
        break;
      case '+':
        brigthness += PWM_DUTY_CYCLE_MAX / BRIGTHNESS_NUMBER_OF_STEPS; //Увеличиваем яркость светодиода (рабочий цикл ШИМ) на одну ступень.
        if(brigthness >= PWM_DUTY_CYCLE_MAX)
          brigthness = PWM_DUTY_CYCLE_MAX;
        targetPWMOutSetDutyCycle(brigthness);
        break;
      case '-':
        brigthness -= PWM_DUTY_CYCLE_MAX / BRIGTHNESS_NUMBER_OF_STEPS; //Уменьшаем яркость светодиода (рабочий цикл ШИМ) на одну ступень.
        if(brigthness <= 0)
          brigthness = 0;
        targetPWMOutSetDutyCycle(brigthness);
        break;
      default:
        break;
    }
  #endif
  if(TEST_EVENT(BUTTON_PRESSED_EVENT))
  {
    RESET_EVENT(BUTTON_PRESSED_EVENT);
    switch(buttonMode)
    {
      case 0:
        targetSetup(); //Осуществляем начальную на тройку неизменяющихся полей команды.
        buttonMode = 1;
        break;
      case 1:
        targetDigitalOutSetHigh(); //Включаем дискретный цифровой выход на удаленном модуле.
        buttonMode = 2;
        break;
      case 2:
        targetDigitalOutSetLow(); //Выключаем дискретный цифровой выход на удаленном модуле.
        buttonMode = 3;
        break;
      case 3:
      targetPWMOutSetDutyCycle(PWM_DUTY_CYCLE_MAX / 4); //Включаем ШИМ на 25%.
        buttonMode = 4;
        break;
      case 4:
        targetPWMOutSetDutyCycle(PWM_DUTY_CYCLE_MAX / 2); //Включаем ШИМ на 50%.
        buttonMode = 5;
        break;
      case 5:
        targetPWMOutSetDutyCycle(3 * PWM_DUTY_CYCLE_MAX / 4); //Включаем ШИМ на 75%.
        buttonMode = 6;
        break;
      case 6:
        targetPWMOutSetDutyCycle(PWM_DUTY_CYCLE_MAX); //Включаем ШИМ на 100%.
        buttonMode = 7;
        break;
      case 7:
        targetPWMOutSetDutyCycle(0); //Выключаем ШИМ.
        buttonMode = 0;
        break;
      default:
        break;
    }
  }
  goto main_loop;
}

//*****************************************************************************
//
//! \brief Установка неизменяющихся полей пакета с командой удаленному модулю.
//!
//! \return NONE
//
//*****************************************************************************
void targetSetup(void)
{
  remoteAtRequest.setDefault(); //Приводим все поля команды для передачи к значениям "по умолчанию" для демонстрации наличия этой функции. Таким образом упрощается повоторное использование объекта для отправки последующих команд.
  remoteAtRequest.setRemoteAddress(TARGET_ADDRESS); //Устанавливаем адрес удаленного модуля, которому будет передаваться команда.
  remoteAtRequest.setCommand((uint8_t*)"R4"); //Устанавливаем код команды - вывод R4 (Сигнал DO2 на плате MB-Tag-1.1) переводим в режим первого ШИМ-выхода.
  cmdParam[0] = IO_PWM1; //Устанавливаем значения параметра команды.
  remoteAtRequest.setCommandValue(&cmdParam[0]); //Устанавливаем указатель на переменную с параметром команды.
  remoteAtRequest.setCommandValueLength(1); //Длина поля параметра команды 1 байт.
  sendCommand();
}

//*****************************************************************************
//
//! \brief Отправка удаленному модулю команды на включение цифрового выхода.
//!
//! \return NONE
//
//*****************************************************************************
void targetDigitalOutSetHigh(void)
{
  remoteAtRequest.setCommand((uint8_t*)"L5"); //Устанавливаем код команды - вывод L5 (Сигнал DO3 на плате MB-Tag-1.1) переводим в режим цифрового выхода.
  cmdParam[0] = IO_DIGITAL_OUTPUT_HI; //Устанавливаем значения параметра команды.
  remoteAtRequest.setCommandValue(&cmdParam[0]); //Устанавливаем указатель на переменную с параметром команды.
  remoteAtRequest.setCommandValueLength(1); //Длина поля параметра команды 1 байт.
  sendCommand();
}

//*****************************************************************************
//
//! \brief Отправка удаленному модулю команды на выключение цифрового выхода.
//!
//! \return NONE
//
//*****************************************************************************
void targetDigitalOutSetLow(void)
{
  remoteAtRequest.setCommand((uint8_t*)"L5"); //Устанавливаем код команды - вывод L5 (Сигнал DO3 на плате MB-Tag-1.1) переводим в режим цифрового выхода.
  cmdParam[0] = IO_DIGITAL_OUTPUT_LO; //Устанавливаем значения параметра команды.
  remoteAtRequest.setCommandValue(&cmdParam[0]); //Устанавливаем указатель на переменную с параметром команды.
  remoteAtRequest.setCommandValueLength(1); //Длина поля параметра команды 1 байт.
  sendCommand();
}

//*****************************************************************************
//
//! \brief Отправка удаленному модулю команды установки на ШИМ-выходе требуемого значения
//! рабочего цикла.
//!
//! \return NONE
//
//*****************************************************************************
void targetPWMOutSetDutyCycle(uint16_t dutyCycle)
{
  uint8_t *temp = cmdParam;
  remoteAtRequest.setCommand((uint8_t*)"M1"); //Устанавливаем код команды M1 - установка значения рабочего цикла первого канала ШИМ.
  UINT16_TO_BUF_BIG_ENDIAN(temp, dutyCycle); //Устанавливаем значения параметра команды.
  remoteAtRequest.setCommandValue(&cmdParam[0]); //Устанавливаем указатель на переменную с параметром команды.
  remoteAtRequest.setCommandValueLength(2); //Длина поля параметра команды 2 байт.
  sendCommand();
}

//*****************************************************************************
//
//! \brief Передача команды удаленному модулю.
//!
//! \return NONE
//
//*****************************************************************************
void sendCommand(void)
{
  //Далее приводятся возможные опции отправки (см. описание опций в документации к библиотеке cpp-mbee.).
  remoteAtRequest.setSaveChanges(false); //Изменения будут сразу применены без сохранения.
  //remoteAtRequest.setCca(false); //Отключает режим CCA (Clear Channel Assessment) перед отправкой команды.
  //remoteAtRequest.setAcknowledge(false); //Для повышения пропускной способности можно отключить подтверждение получения команды от удаленным модемом.
  //remoteAtRequest.setApplyChanges(false); //Измененный параметр не будет применен и сохранен сразу после выполнения команды, а помещен в очередь.
  //remoteAtRequest.setSleepingDevice(true); //Команда предназначается спящему модему.
  MBee.send(remoteAtRequest); //Отправляем команду на удаленный модуль.
  if(remoteAtRequest.getFrameId()) //Если frameID = 0, то локальный ответ не формируется и поэтому его не ждем.
  {
    if(MBee.readPacket(LOCAL_RESPONSE_TIMEOUT)) //Ждем локального ответа со статусом команды.
    {
      if(MBee.getResponse().getApiId() == TRANSMIT_STATUS_API_FRAME)//Проверяем, является ли принятый пакет локальным ответом на AT-команду удаленному модему.
      {
        MBee.getResponse().getTxStatusResponse(txStatus); //Получаем объект с локальным статусом отправки.
        #if defined(ENABLE_CONSOLE)
          if(txStatus.isSuccess())
          {
            if(remoteAtRequest.getSleepingDevice())
            {
              console.print("Command is successful buffered for sleeping target with address: ");
              console.println(TARGET_ADDRESS, 16);
            }
            else
            {
              console.print("Command is successful transmitted to the target with address: ");
              console.println(TARGET_ADDRESS, 16);
            }
          }
          else if(txStatus.getStatus() == ERROR_COMMAND_STATUS)
          {
            console.println("Error! Unsufficient memory for packet.");
          }
          else if(txStatus.getStatus() == INVALID_CODE_COMMAND_STATUS)
          {
            console.println("Error! Invalid command code.");
          }
          else if(txStatus.getStatus() == INVALID_PARAMETER_COMMAND_STATUS)
          {
            console.println("Error! Invalid parameter value.");
          }
          else if(txStatus.getStatus() == TX_FAILURE_COMMAND_STATUS)
          {
            console.println("Warning! Command is not transmitted due to CCA failure.");
          }
      #endif
      }
    }
  }
  if((remoteAtRequest.getFrameId() == 0) || (txStatus.isSuccess() && remoteAtRequest.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема только если локальный ответ выключен с помощью frameId = 0, команда отправлена(буферизирована) и не предназначается спящему модему.
  {
    if(MBee.readPacket(ACKNOWLEDGE_TIMEOUT)) //Ждем ответа от удаленного модуля со статусом команды.
    {
      if(MBee.getResponse().getApiId() == REMOTE_AT_COMMAND_RESPONSE_API_FRAME)  //Проверяем, не является ли полученный фрейм ответом на команду удаленным модемом.
      {
        MBee.getResponse().getRemoteAtCommandResponse(remoteAtResponse);
        if(remoteAtResponse.isOk())
        {
          ledBlinking(GREEN, 2); //Команда успешно доставлена на удаленый модем.
          #if defined(ENABLE_CONSOLE)
          console.print("Command is successful delivered to the target with address: "); //Получение команды подтверждено удаленным модулем.
            console.println(TARGET_ADDRESS, 16);
          #endif
        }
      }
    }
    else
    {
      ledBlinking(RED, 3); //Нет ответа на команду от удаленного модема.
      #if defined(ENABLE_CONSOLE)
        console.println("No response from the target with address: "); //Нет ответа от удаленного модуля.
      #endif
    }
  }
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
//! \param led Код цвета светодиода.
//! \param count Число включений светодиода.
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

