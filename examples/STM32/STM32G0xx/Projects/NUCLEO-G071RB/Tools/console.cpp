/* "Системы модули и компоненты" ("СМК"). 2019. Москва.
Библиотека C++ для модулей MBee. Пример использования библиотеки cpp-mbee на микроконтроллерах STM32 производства STMicroelectronics.
Распространяется свободно. Надеемся, что программные продукты, созданные
на основе данного примера будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library Example for STM32.
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
// console.cpp - определения методов для класса console.
//
//*****************************************************************************

/*********************************************************************
 * INCLUDES
 */
#include "usart.h"

unsigned long consoleUartBitrate;

#if defined(ENABLE_CONSOLE)
#warning Console is enabled.
#include "MBee.h"
#include "Console.h"

/*********************************************************************
 * MACROS
 */
#define st(x) do{x} while(__LINE__ == -1)
#define ENTER_CRITICAL_SECTION(x, y)  st((x) =  NVIC_GetEnableIRQ(y);\
                                            HAL_NVIC_DisableIRQ(y);)
#define EXIT_CRITICAL_SECTION(x, y) st(if(x)\
                                       {HAL_NVIC_EnableIRQ(y);}\
                                       else\
                                       {HAL_NVIC_DisableIRQ(y);})

/*********************************************************************
 * CONSTANTS
 */
//Переназначаем стандартные потоки ввода/вывода на UART консоли.
#if defined(__IAR_SYSTEMS_ICC__)
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE * f)
#elif defined(__TI_COMPILER_VERSION__)
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE * f)
#elif defined(__GNUC__)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(void)
#endif

/*********************************************************************
 * TYPEDEFS
 */
typedef uint32_t interruptState_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern UART_HandleTypeDef * consoleUartHandle;

/*********************************************************************
 * LOCAL VARIABLES
 */
#if defined(__IAR_SYSTEMS_ICC__)
  __no_init static uint8_t consoleTxBuffer[CONSOLE_UART_TX_BUFFER_SIZE]; //Буфера не инициализируем для сокращения времени рестарта.
  __no_init static uint8_t consoleRxBuffer[CONSOLE_UART_RX_BUFFER_SIZE];
#elif defined(__TI_COMPILER_VERSION__)
  #pragma NOINIT
  static uint8_t consoleTxBuffer[CONSOLE_UART_TX_BUFFER_SIZE];
  #pragma NOINIT
  static uint8_t consoleRxBuffer[CONSOLE_UART_RX_BUFFER_SIZE];
#elif defined(__GNUC__)
  static uint8_t consoleTxBuffer[CONSOLE_UART_TX_BUFFER_SIZE] __attribute__((section(".noinit")));
  static uint8_t consoleRxBuffer[CONSOLE_UART_RX_BUFFER_SIZE] __attribute__((section(".noinit")));
#endif

static uint8_t consoleRxByte;
static volatile uint32_t consoleTxBufferPointer, consoleTxBufferCounter; //Инициализируем 0-ми указатели и счетчики буферов.
static volatile uint32_t consoleRxBufferPointer, consoleRxBufferCounter;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
#ifdef __cplusplus
 extern "C" {
#endif
 int __io_putchar(int ch);
 int __io_getchar(void);
#ifdef __cplusplus
}
#endif

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
* Public методы класса Console.
*********************************************************************/
Console::Console(const char* portName)
{
  _portName = portName;
}

bool Console::begin(uint32_t bitrate)
{
  consoleUartBitrate = bitrate;
  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart2, &consoleRxByte, 1);
  return true;
}

void Console::end()
{
  HAL_UART_DeInit(&huart2);
}

uint8_t Console::run()
{
  uint8_t command;
  scanf("%c", &command);
  return command;
}

void Console::printIoSamplePacket(RxIoSampleResponse& ioSample)
{
  printf("\nReceived packet with I/O samples from remote node with address: %x.\n\r", ioSample.getRemoteAddress()); //Печатаем адрес отправителя.
  if(ioSample.getTemperature() < 128) //Выводим температуру чипа удаленного модуля по показаниям встроенного датчика.
  {
    printf("Chip temperature is %dC.\r\n", ioSample.getTemperature());
  }
  else
  {
    printf("Chip temperature is %dC.\r\n", ioSample.getTemperature() - 256);
  }
  printf("Supply voltage is %.2fV.\r\n", ((float)(ioSample.getVbatt()) / 51)); //Печатаем напряжение источника питания.
  for(uint32_t i = 1; i <= ioSample.getSampleSize(); i++)// Выводим данные о текущем состоянии каждой линии ввода/вывода содержащейся а пакете.
  {
    printf("Pin #%d is ", ioSample.getPin(i)); //Выводим идентификатор режима, на работу в котором настроена данная линия ввода/вывода.
    if((ioSample.getMode(i) & 0x7F) == IO_ADC) //Информация о текущем состоянии цифрового входа/выхода передается в старшем бите кода режима. Поэтому сбрасываем его, чтобы получить код режима.
    {
      printf("analog input. Voltage: %.2fmV.",((float)(ioSample.getAnalog(i))* 2500) /4096);
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_INPUT)
    {
      printf("digital input. State: %d", ioSample.getDigital(i)); //Пример получения информации о текущем состоянии цифровой линии с помощью специальной функции.
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_OUTPUT_LO)
    {
      printf("digital output with default LOW state. State: %d", (ioSample.getMode(i) & 0x80) >> 7); //Пример получения информации о текущем состоянии цифровой линии с помощью старшего бита кода режима. Такой способ работает намного быстрее, чем вызов функции getDigital().
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_OUTPUT_HI)
    {
      printf("digital output with default HIGH state. State: %d", (ioSample.getMode(i) & 0x80) >> 7);
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_COUNTER_INPUT1)
    {
      printf("count input 1 with pullup. Pulse count: %d.", (int)ioSample.getCounter(i));
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_COUNTER_INPUT2)
    {
      printf("count input 2 with pullup. Pulse count: %d.", (int)ioSample.getCounter(i));
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_WAKEUP_INPUT_FALLING_EDGE)
    {
      printf("wakeup(alarm) input with HIGH to LOW active front. State: %d", (ioSample.getMode(i) & 0x80) >> 7);
    }
    else if((ioSample.getMode(i) & 0x7F) == IO_WAKEUP_INPUT_RISING_EDGE)
    {
      printf("wakeup(alarm) input with LOW to HIGH active front. State: %d.", (ioSample.getMode(i) & 0x80) >> 7);
    }
    printf("\r\n");
  }
}

PUTCHAR_PROTOTYPE
{
  interruptState_t  intState;
  ENTER_CRITICAL_SECTION(intState, USART2_IRQn);
		uint32_t temp = consoleTxBufferPointer;
		uint32_t offset = temp + consoleTxBufferCounter++;
		if(offset >= CONSOLE_UART_TX_BUFFER_SIZE)
		{
			offset -= CONSOLE_UART_TX_BUFFER_SIZE;
		}
		consoleTxBuffer[offset++] = ch;
		EXIT_CRITICAL_SECTION(intState, USART2_IRQn);
		if(offset == CONSOLE_UART_TX_BUFFER_SIZE)
		{
			offset = 0u;
		}
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)&ch, 1);
		return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch;
  interruptState_t intState;
  ENTER_CRITICAL_SECTION(intState, USART2_IRQn);
  if(consoleRxBufferCounter)
  {
    ch = consoleRxBuffer[consoleRxBufferPointer++];
    if(consoleRxBufferPointer >= CONSOLE_UART_RX_BUFFER_SIZE)
    {
      consoleRxBufferPointer -= CONSOLE_UART_RX_BUFFER_SIZE;
    }
    consoleRxBufferCounter--;
  }
  else
  {
    ch = '\n';
  }
  EXIT_CRITICAL_SECTION(intState, USART2_IRQn);
  return ch;
}

void CONSOLE_HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  consoleTxBufferCounter--;
  if(consoleTxBufferCounter)
  {
    if(HAL_UART_Transmit_IT(UartHandle, &consoleTxBuffer[++consoleTxBufferPointer], 1) != HAL_OK)
    {
      Error_Handler();
    }
    if(consoleTxBufferPointer == CONSOLE_UART_TX_BUFFER_SIZE)
    {
      consoleTxBufferPointer = 0u;
    }
  }
}

void CONSOLE_HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  uint32_t temp;
  uint32_t offset;
  temp = consoleRxBufferPointer;
  offset = temp + consoleRxBufferCounter;
  if(offset >= CONSOLE_UART_RX_BUFFER_SIZE)
  {
    offset -= CONSOLE_UART_RX_BUFFER_SIZE;
  }
  consoleRxBuffer[offset] = consoleRxByte;
  if(consoleRxBufferCounter == CONSOLE_UART_RX_BUFFER_SIZE)
  {
    consoleRxBufferPointer++;
    if(consoleRxBufferPointer == CONSOLE_UART_RX_BUFFER_SIZE)
    {
      consoleRxBufferPointer = 0u;
    }
  }
  else
  {
    consoleRxBufferCounter++;
  }
  HAL_UART_Receive_IT(UartHandle, &consoleRxByte, 1);
}
#endif //ENABLE_CONSOLE.
