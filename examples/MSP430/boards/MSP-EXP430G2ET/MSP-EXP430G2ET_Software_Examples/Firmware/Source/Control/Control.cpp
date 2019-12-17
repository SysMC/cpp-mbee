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
#include "Control.h"
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
static Serial serial("UART"); //Объект для работы с COM-портом. Имя может быть произвольное.
static RemoteAtCommandRequest remoteAtRequest; //Объект для пакета с AT-командой удаленному модулю.
static TxStatusResponse txStatus; //Объект для пакета с локальным ответом на AT-команду удаленному модулю.
static RemoteAtCommandResponse remoteAtResponse; //Объект для пакета с ответом от удаленного модуля.
static RxIoSampleResponse ioPacket; //Объект для принятого пакета с данными о состоянии линий ввода/вывода удаленного радиомодуля.
static volatile  uint16_t events; //Битовое поле с флагами событий.
static uint8_t buttonMode; //Переменная, в которой хранится режим кнопки, оределяющий реакцию на ее нажатие.
static uint8_t cmdParam[2]; //Параметр команды. 2- максимальная длина параметра команды, которая будет использоваться в этом примере.
static uint16_t potentiometerBrigthnessSetting; //Текущее значение яркости, установленное с помощью потенциометра.
static uint32_t currentTime; //Время, прошедшее с момента включения данного узла.
static uint32_t timer; //Переменная для измерения интервалов времени.

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter); //Callback-функция, вызываемая библиотекой cpp-mbee при приеме пакета 0x83.
static void parseIoSamplePacket(void);
static void targetSetup(void);
static void setTargetAddress(void);
static void targetDigitalOutSet(uint8_t level);
static void targetPWMOutSetDutyCycle(uint16_t dutyCycle);
static void sendRequest(void);
static void sendCommand(void);
static void hardwareInit(void);
static void ledBlinking(uint8_t led, uint8_t count);
static bool potentiometerPoll(void);
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
Пульт дистанционно управляет одним дискретным и одним аналоговым каналом. Кроме этого он осуществляет контроль одного аналогового канала. В реальных системах а качестве объекта
управления может быть использован, например, драйвер светодиодного светильника с релейным управлением или с возможностью плавной регулировки с помощью ШИМ. К аналоговому входу
может быть подключен датчик тока драйвера. Мониторинг тока позволяет получать данные об исправности светильника и о величине потребляемой им энергии.
Первый контакт DIP-переключателя управляет режимом адресации пульта. Если переключатель находится в положении OFF, команды отправляются в широковещательном режиме и будут исполняться
всеми радиомодулями, имеющими совпадающие сетевые и радиочастотные настройки. В широковещательном режиме контроль доставки команд не осуществляется.
Если установить первый контакт DIP-переключателя в положение ON, то пульт переходит в режим адресного управления. Все команды будут отправляться модулю с сетевым адресом, определяемом
константой TARGET_ADDRESS в файле Control.h. При этом осуществляется контроль получения команды удаленным радиомодулем. Если какая-либо команда им не получена, то пульт выдает
предупреждение - три коротких звуковых сигнала и/или (зависит от положения перемычки BUZZER) трех включений красного светодиода.
При нажатии на кнопку BUTTON1 на плате BOOSTXL-MBEE868-1.1, пульт отправляет команду включения или выключения цифрового выхода на удаленном модуле. Если модуль, установленный на
плате MB-Tag-1.2 получит команду включения, на ней должен загореться светодиод, подключенный к выходу DO3.
Управление ШИМ-выходом осуществляется с помощью потенциометра пульта. Если при вращении ручки потенциометра его сопротивление изменится более, чем на заданную величину, то
пульт отправит на удаленный модуль команду установки нового значения рабочего цикла ШИМ. Чувствительность потенциометра определяется параметром POTENTIOMETER_SENSITIVITY в файле
Control.h. На плате MB-Tag-1.2 это должно приводить к изменению яркости светодиода на выходе DO2.
Для контроля аналогового канала на удаленном модуле, пульт периодически передает команду запроса состояния линий ввода/вывода. Интервал задается параметром
REMOTE_MODULE_POLL_PERIOD в файле Control.h. Прием ответа на команду подтверждается кратковременным включением зеленого светодиода.
Если измеренное значение напряжения на аналоговом входе AI1 на плате MB-Tag-1.2 превышает порог, определяемый константой CURRENT_MAX, то на пульте включается постоянный звуковой
сигнал и загорается красный светодиод. Для проверки контроля напряжения на аналоговом входе его можно подключить с помощью перемычки к выходу DO2. ВНИМАНИЕ! Поскольку на выходе DO2
формируется ШИМ-сигнал, то измеренное значение напряжения будет зависеть от момента проведения измерения, что может приводить к "ложным" превышениям порога CURRENT_MAX даже при низких
средних уровнях напряжения. Для исключения этого можно увеличить емкость конденсатора, подключенного ко входу AI1 на плате MBee-Tag-1.2.
*/
int main( void )
{
  WDTCTL = WDTPW + WDTHOLD; //Отключаем встроенный watchdog-таймер во избежание его таймаута во время инициализации.
  hardwareInit();
  __enable_interrupt(); //Разрешаем прерывания, необходимые для работы модулей eUSCI в режиме UART.
  serial.begin(MBEE_UART_BITRATE);
  MBee.begin(serial);
  targetSetup(); //Осуществляем начальную настройку неизменяющихся полей объекта с командой.
  MBee.onRxIoSampleResponse(ioSamplePacketReceived); //Регистрация callback-функции для приема пакетов 0x83 с состоянием линий ввода-вывода.
  events = 0;
  potentiometerPoll();
  timer = millis(); //Иницализируем таймер.
main_loop:
  MBee.run();
  if(TEST_EVENT(BUTTON1_PRESSED_EVENT))
  {
    RESET_EVENT(BUTTON1_PRESSED_EVENT);
    switch(buttonMode)
    {
      case 0:
        targetDigitalOutSet(IO_DIGITAL_OUTPUT_HI); //Включаем дискретный цифровой выход на удаленном модуле.
        buttonMode = 1;
        break;
      case 1:
        targetDigitalOutSet(IO_DIGITAL_OUTPUT_LO); //Выключаем дискретный цифровой выход на удаленном модуле.
        buttonMode = 0;
        break;
      default:
        break;
    }
  }
  if(TEST_EVENT(DIP1_CHANGE_EVENT))
  {
    RESET_EVENT(DIP1_CHANGE_EVENT);
    setTargetAddress();
  }
  if(TEST_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT))
  {
    RESET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
    parseIoSamplePacket();
  }
  currentTime = millis();
  if(currentTime - timer > REMOTE_MODULE_POLL_PERIOD) //Проверяем, не пора ли опросить удаленный модуль.
  {
    sendRequest();
    timer = currentTime;
  }
  if(potentiometerPoll()) //Если положение потенциометра изменилось более чем на заданную величину (определяется POTENTIOMETER_SENSITIVITY), то отправляем команду на удаленный модуль.
  {
    targetPWMOutSetDutyCycle(potentiometerBrigthnessSetting);
  }
  goto main_loop;
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
  if(ioPacket.getRemoteAddress() == remoteAtRequest.getRemoteAddress() || remoteAtRequest.getRemoteAddress() == BROADCAST_ADDRESS) //Проверяем совпадает ли адрес отправителя пакета с адресом модуля, которому был отправлен запрос.
  {
    if(ioPacket.getSampleSize()) //Содержится ли в принятом пакете информация о состоянии линий ввода/вывода удаленного модема?
    {
      sampleNumber = ioPacket.getSampleNumber(CURRENT_SENSOR_INPUT);
      if(sampleNumber) //Есть ли в принятом пакете информация о состоянии вывода радиомодуля, к которому подключен датчик тока и какой номер выборки он имеет?
      {
        if((ioPacket.getMode(sampleNumber)) == IO_ADC) //Настроен ли данный вывод как аналоговый вход?
        {
          if(ioPacket.getAnalog(sampleNumber) > CURRENT_MAX)
          {
            P2OUT |= BIT4; //Напряжение на аналоговом входе больше заданного порога. Включаем цифровой выход "Тревога" (светодиод LAMP1).
          }
          else
          {
            P2OUT &= ~BIT4; //Напряжение на аналоговом входе меньше заданного порога. Выключаем цифровой выход "Тревога" (светодиод LAMP1).
          }
        }
      }
    }
  }
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
  remoteAtRequest.setDefault(); //Приводим все поля команды для передачи к значениям "по умолчанию" для демонстрации наличия этой функции. С помощью этой функции упрощается повторное использование объекта для отправки последующих команд.
  remoteAtRequest.setCommandValue(&cmdParam[0]); //Устанавливаем указатель на переменную с параметром команды.
  setTargetAddress();
}

//*****************************************************************************
//
//! \brief Установка адреса удаленного модуля в зависимости от положения DIP-переключателя.
//!
//! \return NONE
//
//*****************************************************************************
void setTargetAddress(void)
{
  uint16_t address = TARGET_ADDRESS;
  if(P2IN & BIT7) //DIP-переключатель разомкнут - устанавливаем широковещательный режим передачи.
  {
    address = BROADCAST_ADDRESS;
  }
  remoteAtRequest.setRemoteAddress(address);
}

//*****************************************************************************
//
//! \brief Отправка удаленному модулю команды на включение/выключение цифрового выхода.
//!
//! \param level Уровень, который требуется установить на данном цифровом выходе.
//!
//! \return NONE
//
//*****************************************************************************
void targetDigitalOutSet(uint8_t level)
{
  remoteAtRequest.setCommand((uint8_t*)"L5"); //Устанавливаем код команды - вывод L5 (Сигнал DO3 на плате MB-Tag-1.1) переводим в режим цифрового выхода.
  cmdParam[0] = level; //Устанавливаем значения параметра команды.
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
  remoteAtRequest.setCommandValueLength(2); //Длина поля параметра команды 2 байт.
  sendCommand();
}

//*****************************************************************************
//
//! \brief Передача запроса о состоянии линий ввода/вывода удаленному модулю.
//!
//! \return NONE
//
//*****************************************************************************
void sendRequest(void)
{
  bool acknowledge;
  acknowledge = remoteAtRequest.getAcknowledge(); //Сохраняем текущие установки удаленного подтверждения.
  remoteAtRequest.setAcknowledge(false); //Команды запроса передаются без подтверждения получения, поскольку подтверждением будет служить сам ответ на запрос.
  remoteAtRequest.setCommand((uint8_t*)"IO");
  remoteAtRequest.setCommandValueLength(0); //Поскольку эта команда запроса, то поле параметра имеет нулевую длину.
  sendCommand();
  remoteAtRequest.setAcknowledge(acknowledge); //Восстанавливаем прежний режим удаленного подтверждения.
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
  bool acknowledge;
  //Далее приводятся возможные опции отправки (см. описание опций в документации к библиотеке cpp-mbee.).
  remoteAtRequest.setSaveChanges(false); //Изменения будут сразу применены без сохранения.
  //remoteAtRequest.setCca(false); //Отключает режим CCA (Clear Channel Assessment) перед отправкой команды.
  //remoteAtRequest.setAcknowledge(false); //Для повышения пропускной способности можно отключить подтверждение получения команды от удаленным модемом.
  //remoteAtRequest.setApplyChanges(false); //Измененный параметр не будет применен и сохранен сразу после выполнения команды, а помещен в очередь.
  //remoteAtRequest.setSleepingDevice(true); //Команда предназначается спящему модему.
  acknowledge = remoteAtRequest.getAcknowledge(); //Сохраняем текущие установки удаленного подтверждения.
  if(remoteAtRequest.getRemoteAddress() == BROADCAST_ADDRESS)
  {
    remoteAtRequest.setAcknowledge(false); //Чтобы не вызвать шквал пакетов, отключаем удаленное подтверждение доставки, если адрес получателя широковещательный.
  }
  MBee.send(remoteAtRequest); //Отправляем команду на удаленный модуль.
  if(remoteAtRequest.getFrameId()) //Если frameID = 0, то локальный ответ не формируется и поэтому его не ждем.
  {
    if(MBee.readPacket(LOCAL_RESPONSE_TIMEOUT)) //Ждем локального ответа со статусом команды.
    {
      if(MBee.getResponse().getApiId() == TRANSMIT_STATUS_API_FRAME)//Проверяем, является ли принятый пакет локальным ответом на AT-команду удаленному модему.
      {
        MBee.getResponse().getTxStatusResponse(txStatus); //Получаем объект с локальным статусом отправки.
      }
    }
  }
  //Ждем ответного пакета от удаленного модема если выполняются следующие условия:
  //1. Локальное подтверждение отправки выключено с помощью frameId = 0.
  //2. Команда отправлена в эфир и при этом не предназначается спящему модему.
  //3. Команда не широковещательная.
  //4. Удаленное подтверждение включено.
  if(remoteAtRequest.getFrameId() == 0 || (txStatus.isSuccess() && remoteAtRequest.getSleepingDevice() == false && remoteAtRequest.getRemoteAddress() != BROADCAST_ADDRESS && remoteAtRequest.getAcknowledge()))
  {
    if(MBee.readPacket(ACKNOWLEDGE_TIMEOUT)) //Ждем ответа от удаленного модуля со статусом команды.
    {
      if(MBee.getResponse().getApiId() == REMOTE_AT_COMMAND_RESPONSE_API_FRAME)  //Проверяем, не является ли полученный фрейм ответом на команду удаленным модемам.
      {
        MBee.getResponse().getRemoteAtCommandResponse(remoteAtResponse);
        if(remoteAtResponse.isOk())
        {
          ledBlinking(GREEN, 2); //Команда успешно доставлена на удаленый модем.
        }
      }
    }
    else
    {
      ledBlinking(RED, 3); //Нет ответа на команду от удаленного модема.
    }
  }
  remoteAtRequest.setAcknowledge(acknowledge); //Восстанавливаем прежний режим удаленного подтверждения.
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

  //Настройка АЦП.
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON;
  ADC10CTL1 = INCH_0;
  ADC10AE0 |= 0x01;

//  ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
//  ADC_enable(ADC_BASE);
//  ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);
//  ADC_setResolution(ADC_BASE, ADC_RESOLUTION);
//  ADC_configureMemory(ADC_BASE, ADC_INPUT_A9, ADC_VREFPOS_INT, ADC_VREFNEG_AVSS);
//  ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
//  PMM_enableInternalReference();
//  while(PMM_REFGEN_NOTREADY == PMM_getVariableReferenceVoltageStatus());
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
//! \brief Установка рабочего цикла ШИМ в зависимости от положения потенциометра.
//!
//! \return true, если текущее значение изменилось более, чем на заданную величину.
//
//*****************************************************************************
bool potentiometerPoll(void)
{
  bool status = false;
  int16_t delta;
  uint16_t currentPotentiometerBrigthnessSetting;
  ADC10CTL0 |= ENC + ADC10SC;
  while (!(ADC10IFG & ADC10CTL0));
//  ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
//  while(!ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG));
//  currentPotentiometerBrigthnessSetting = ((long)ADC_getResults(ADC_BASE) * PWM_DUTY_CYCLE_MAX)/((0x0FFF >> (4 - (ADC_RESOLUTION >> 3))) - 1);
  currentPotentiometerBrigthnessSetting = ((long)ADC10MEM * PWM_DUTY_CYCLE_MAX)/1023;
  delta = potentiometerBrigthnessSetting - currentPotentiometerBrigthnessSetting;
  if(delta < 0)
  {
    delta = - delta;
  }
  if(delta >= (PWM_DUTY_CYCLE_MAX / 100) * POTENTIOMETER_SENSITIVITY) //Проверяем, превышает ли разность между установленным уровенем яркости и текущим порог чувствительности.
  {
    potentiometerBrigthnessSetting = currentPotentiometerBrigthnessSetting;
    status = true;
  }
  return status;
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

