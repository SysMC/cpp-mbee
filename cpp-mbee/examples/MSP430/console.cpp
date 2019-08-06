/* --COPYRIGHT--
 *
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// console.cpp - .
//
//*****************************************************************************

/*********************************************************************
 * INCLUDES
 */
#if defined(ALARM_EXAMPLE)
  #include "Alarm.h"
#elif defined(CONTROL_EXAMPLE)
  #include "Control.h"
#endif
#if defined(ENABLE_CONSOLE)
#include <string.h>
#include "MBee.h"
#include "Console.h"
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
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
/*********************************************************************
* Public методы класса Switch.
*********************************************************************/
Console::Console()
{

}

void Console::begin(uint32_t bitrate)
{
  uartInit(bitrate);
}

uint8_t Console::run()
{
  uint8_t command = 0;
  if(uartAvailable())
  {
    uartRx(&command, 1); //Считываем первый байт, поступивший в буфер UART.
  }
  return command;
}

void Console::printIoSamplePacket(RxIoSampleResponse& ioSample)
{
  println("");
  print("Received packet with I/O samples from remote node with address: "); //Печатаем адрес отправителя.
  println(ioSample.getRemoteAddress(), 16);
  print("Chip temperature is "); //Выводим температуру чипа удаленного модуля по показаниям встроенного датчика.
  if(ioSample.getTemperature() < 128) //Переводим число из дополнительного кода в прямой.
  {
    print(ioSample.getTemperature(), 10);
  }
  else
  {
    print("-"); //Температура отрицательная.
    print(256 - ioSample.getTemperature(), 10);
  }
  println("C.");
  print("Supply voltage is ");
  print((uint32_t)(((float)(ioSample.getVbatt()) / 51) * 1000), 10);
  println(" mV.");

  for(uint8_t i = 1; i <= ioSample.getSampleSize(); i++)// Выводим данные о текущем состоянии каждой линии ввода/вывода содержащейся а пакете.
  {
    print("Pin #");
    print(ioSample.getPin(i), 10);
    print(" is "); //Выводим идентификатор режима, на работу в котором настроена данная линия ввода/вывода.
    if((ioSample.getMode(i) & 0x7F) == IO_ADC) //Информация о текущем состоянии цифрового входа/выхода передается в старшем бите кода режима. Поэтому сбрасываем его, чтобы получить код режима.
    {
      print("analog input. Voltage: ");
      print((uint32_t)(((float)(ioSample.getAnalog(i))* 2500) /4096), 10);
      print(" mV.");

    }
    else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_INPUT)
    {
      print("digital input. State: ");
      print(ioSample.getDigital(i), 10); //Пример получения информации о текущем состоянии цифровой линии с помощью специальной функции.
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_OUTPUT_LO)
    {
      print("digital output with default LOW state. State: ");
      print(ioSample.getMode(i) & 0x80, 10); //Пример получения информации о текущем состоянии цифровой линии с помощью старшего бита кода режима. Такой способ работает намного быстрее, чем вызов функции getDigital().
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_OUTPUT_HI)
    {
      print("digital output with default HIGH state. State: ");
      print(ioSample.getMode(i) & 0x80, 10);
    }

    else if((ioSample.getMode(i) & 0x7F) == IO_COUNTER_INPUT1)
    {
      print("count input 1 with pullup. Pulse count: ");
      print(ioSample.getCounter(i), 10);
      print(".");
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_COUNTER_INPUT2)
    {
      print("count input 2. Pulse count: ");
      print(ioSample.getCounter(i), 10);
      print(".");
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_WAKEUP_INPUT_FALLING_EDGE)
    {
      print("wakeup(alarm) input with HIGH to LOW active front. State: ");
      print(ioSample.getMode(i) & 0x80, 10);
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_WAKEUP_INPUT_RISING_EDGE)
    {
      print("wakeup(alarm) input with LOW to HIGH active front. State: ");
      print(ioSample.getMode(i) & 0x80, 10);
    }
    println("");
  }
}

void Console::print(char const *buffer_p)
{
  uartTxWait(buffer_p, strlen(buffer_p));
}

void Console::println(char const *buffer_p)
{
  uartTxWait(buffer_p, strlen(buffer_p));
  print("\r\n");
}

void Console::print(uint32_t parameter, uint8_t radix)
{
  uint8_t buffer[10 + 1]; //Для передачи 4 байт в 10-тичном виде требуется 10 символов +  1 терминирующий символ 0x00.
  switch(radix)
  {
    case 10:
    ultoa(parameter, buffer, radix);
    break;
    case 16:
    *buffer = '0';
    *(buffer + 1) = 'x';
    ultoa(parameter, buffer + 2, radix);
    break;
  }
  uartTxWait(buffer, strlen((char*)buffer));
}

void Console::println(uint32_t parameter, uint8_t radix)
{
  print(parameter, radix);
  print("\r\n");
}

/*********************************************************************
* Private методы класса RailroadSwitch.
*********************************************************************/
/*********************************************************************
 * @fn      ultoa
 *
 * @brief   convert a 32 bit number to ASCII
 *
 * @param   num -
 *          buf -
 *          radix -
 *
 * @return  *buf
 *
 *********************************************************************/
uint8_t* Console::ultoa(uint32_t num, uint8_t *buf, uint8_t radix)
{
  uint8_t temp[33];
  int8_t index = 0;
  uint8_t digit;
  uint8_t stringIndex = 0;
  do
  {
    digit = num % radix;
    if(digit < 10)
      temp[index++] = digit + '0';
    else
      temp[index++] = digit - 10 + 'A';
    num = (num) / radix;
  }while(num > 0);
  index--;
  while (index >=0)
    buf[stringIndex++] = temp[index--];
  buf[stringIndex] = 0;
  return((uint8_t*)buf);
}
#endif //ENABLE_CONSOLE.
