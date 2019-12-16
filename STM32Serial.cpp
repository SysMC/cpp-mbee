/* "Системы модули и компоненты" ("СМК"). 2019. Москва.
Библиотека C++ для модулей MBee.
Распространяется свободно. Надеемся, что программные продукты, созданные
с помощью данной библиотеки будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library.
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

Code adapted from  XBee-Arduino library XBee.h. Copyright info below.
* @file       XBee.h
* @author     Andrew Rapp
* @license    This project is released under the GNU License
* @copyright  Copyright (c) 2009 Andrew Rapp. All rights reserved
* @date       2009
* @brief      Interface to the wireless XBee modules
*/

/*********************************************************************
 * INCLUDES
 */
#include "System.h"
#include "usart.h"

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

/*********************************************************************
 * DIAGNOSTICS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef uint32_t interruptState_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint32_t millisCorrector = 1; //Переменная используется для коррекции счетчика миллисекунд, если в приложении, использующем библиотеку есть функции, блокирующие прерывания от таймера SysTick.
unsigned long mbeeUartBitrate;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32_t volatile uwTick;

/*********************************************************************
 * LOCAL VARIABLES
 */
#if defined(__IAR_SYSTEMS_ICC__)
  __no_init static uint8_t txBuffer[UART_TX_BUFFER_SIZE]; //Буфера не инициализируем для сокращения времени рестарта.
  __no_init static uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
#elif defined(__TI_COMPILER_VERSION__)
  #pragma NOINIT
  static uint8_t txBuffer[UART_TX_BUFFER_SIZE];
  #pragma NOINIT
  static uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
#elif defined(__GNUC__)
  static uint8_t txBuffer[UART_TX_BUFFER_SIZE] __attribute__((section(".noinit")));
  static uint8_t rxBuffer[UART_RX_BUFFER_SIZE] __attribute__((section(".noinit")));
#endif
static uint8_t RxByte;
static volatile uint32_t txBufferPointer, txBufferCounter; //Инициализируем 0-ми указатели и счетчики буферов.
static volatile uint32_t rxBufferPointer, rxBufferCounter;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * FUNCTIONS - API
 *********************************************************************/
//Определения публичных методов класса Serial
Serial::Serial(const char* portName)
{
   _portName = portName;
}

int Serial::available()
{
  return rxBufferCounter;
}

uint8_t Serial::read() //Поскольку в библиотеке чтение всегда осуществляется по одному байту и только после вызова функции available, то дополнительные проверки на пустоту буфера проводить не надо.
{
  uint8_t byte;
  interruptState_t intState;
  ENTER_CRITICAL_SECTION(intState, USART1_IRQn);
  byte = rxBuffer[rxBufferPointer++];
  if(rxBufferPointer >= UART_RX_BUFFER_SIZE)
  {
    rxBufferPointer -= UART_RX_BUFFER_SIZE;
  }
  rxBufferCounter--;
  EXIT_CRITICAL_SECTION(intState, USART1_IRQn);
  return byte;
}

void Serial::write(uint8_t byte)
{
  interruptState_t  intState;
  ENTER_CRITICAL_SECTION(intState, USART1_IRQn);
  uint32_t temp = txBufferPointer;
  uint32_t offset = temp + txBufferCounter++;
  if(offset >= UART_TX_BUFFER_SIZE)
  {
    offset -= UART_TX_BUFFER_SIZE;
  }
  txBuffer[offset++] = byte;
  EXIT_CRITICAL_SECTION(intState, USART1_IRQn);
  if(offset == UART_TX_BUFFER_SIZE)
  {
    offset = 0u;
  }
  HAL_UART_Transmit_IT(&huart1, &byte, 1);
}

bool Serial::begin(unsigned long bitrate)
{
  mbeeUartBitrate = bitrate;
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1, &RxByte, 1);
  return true;
}

void Serial::end()
{
  HAL_UART_DeInit(&huart1);
}

uint32_t millis()
{
  return uwTick;
}

void MBEE_HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  txBufferCounter--;
  if(txBufferCounter)
  {
    if(HAL_UART_Transmit_IT(UartHandle, &txBuffer[++txBufferPointer], 1) != HAL_OK)
    {
      Error_Handler();
    }
    if(txBufferPointer == UART_TX_BUFFER_SIZE)
    {
      txBufferPointer = 0u;
    }
  }
}

void MBEE_HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  uint32_t temp;
  uint32_t offset;
  temp = rxBufferPointer;
  offset = temp + rxBufferCounter;
  if(offset >= UART_RX_BUFFER_SIZE)
  {
    offset -= UART_RX_BUFFER_SIZE;
  }
  rxBuffer[offset] = RxByte;
  if(rxBufferCounter == UART_RX_BUFFER_SIZE)
  {
    rxBufferPointer++;
    if(rxBufferPointer == UART_RX_BUFFER_SIZE)
    {
      rxBufferPointer = 0u;
    }
  }
  else
  {
    rxBufferCounter++;
  }
  HAL_UART_Receive_IT(UartHandle, &RxByte, 1);
}




