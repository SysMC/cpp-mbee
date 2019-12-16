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
//uart.c - Высокоуровневый API для модуля eUSCI в режиме UART. Создан на основе драйвера fr2433_uartcli-master с GitHub.
//Оптимизация выполнялась с целью увеличения скорости работы и уменьшения размера кода. Частота SMCLK должна быть установлена равной 8 МГц.
//Константы CLOCK_PRESCALAR, FIRST_MODULATION_STAGE и SECOND_MODULATION_STAGE рассчитывались с помощью on-line утилиты "MSP430 USCI/EUSCI UART Baud Rate Calculation",
//имеющейся на сайте Texas Instruments.
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
#include "macros.h"
#include "uart.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//Выбор модуля UART для работы с консолью.
#ifndef MSP_CONSOLE_UART_MODULE
  #define MSP_CONSOLE_UART_MODULE  EUSCI_A0_BASE
  //#define MSP_CONSOLE_UART_MODULE  EUSCI_A1_BASE
#endif

#if (MSP_CONSOLE_UART_MODULE == EUSCI_A0_BASE)
  #warning Module eUSCI_A0 is selected as console UART for example application. Set appropriate pins for peripheral function!
  #define UCAxIE    UCA0IE
  #define UCAxIFG   UCA0IFG
  #define UCAxBRW   UCA0BRW
  #define UCAxMCTLW UCA0MCTLW
  #define UCAxCTLW0 UCA0CTLW0
  #define UCAxRXBUF UCA0RXBUF
  #define UCAxTXBUF UCA0TXBUF
  #define UCAxIV    UCA0IV
  #define USCI_Ax_VECTOR USCI_A0_VECTOR
  #define USCI_Ax_ISR    USCI_A0_ISR
#elif (MSP_CONSOLE_UART_MODULE == EUSCI_A1_BASE)
  #warning Module eUSCI_A1 is selected as console UART for example application. Set appropriate pins for peripheral function!
  #define UCAxIE    UCA1IE
  #define UCAxIFG   UCA1IFG
  #define UCAxBRW   UCA1BRW
  #define UCAxMCTLW UCA1MCTLW
  #define UCAxCTLW0 UCA1CTLW0
  #define UCAxRXBUF UCA1RXBUF
  #define UCAxTXBUF UCA1TXBUF
  #define UCAxIV    UCA1IV
  #define USCI_Ax_VECTOR  USCI_A1_VECTOR
  #define USCI_Ax_ISR     USCI_A1_ISR
#else
  #error Module eUSCI for console UART must be selected!
#endif

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
#if defined(__IAR_SYSTEMS_ICC__)
  __no_init static uint8_t txBuffer[CONSOLE_UART_TX_BUFFER_SIZE]; //Буфера не инициализируются для сокращения времени рестарта.
  __no_init static uint8_t rxBuffer[CONSOLE_UART_RX_BUFFER_SIZE];
#elif defined(__TI_COMPILER_VERSION__)
  #pragma NOINIT(txBuffer)
  static uint8_t txBuffer[CONSOLE_UART_TX_BUFFER_SIZE];
  #pragma NOINIT(rxBuffer)
  static uint8_t rxBuffer[CONSOLE_UART_RX_BUFFER_SIZE];
#elif defined(__GNUC__)
  static uint8_t txBuffer[CONSOLE_UART_TX_BUFFER_SIZE] __attribute__((noinit));
  static uint8_t rxBuffer[CONSOLE_UART_RX_BUFFER_SIZE] __attribute__((noinit));
#endif
static volatile uint16_t txBufferPointer, txBufferCounter; //Инициализируем нулями указатели и счетчики буферов.
static volatile uint16_t rxBufferPointer, rxBufferCounter;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
uint16_t uartGetTxCounter(void);
void uartTx(const void *, const uint16_t);

/*********************************************************************
 * FUNCTIONS - API
 *********************************************************************/
void uartInit(uint32_t bitrate)
{
  uint16_t ucaxbrw;
  uint16_t ucaxmctlw;
  switch(bitrate)
  {
    case 9600:
      ucaxbrw = 0x0034;
      ucaxmctlw = 0x4911;
      break;
    case 19200:
      ucaxbrw = 0x001A;
      ucaxmctlw = 0xD601;
      break;
    case 38400:
      ucaxbrw = 0x000D;
      ucaxmctlw = 0x4501;
      break;
    case 57600:
      ucaxbrw = 0x0008;
      ucaxmctlw = 0xF7A1;
      break;
    case 115200:
      ucaxbrw = 0x0004;
      ucaxmctlw = 0x5551;
      break;
    case 230400:
      ucaxbrw = 0x0002;
      ucaxmctlw = 0xBB21;
      break;
    default:
      ucaxbrw = 0x3004; //По умолчанию скорость 9600.
      ucaxmctlw = 0x4911;
    break;
  }
  UCAxBRW = ucaxbrw;
  UCAxMCTLW = ucaxmctlw;
  UCAxCTLW0 = 0x0080;
  UCAxIE = 0x0001; //Прерывания разрешаются только после записи UCAxCTLW0 (видимо сброса бита UCSWRST).
}

uint16_t uartAvailable(void)
{
  uint16_t counter;
  CRITICAL_STATEMENT(counter = rxBufferCounter);
  return counter;
}

void uartFlush(void)
{
  uint8_t dummy;
  while(uartAvailable())
    uartRx(&dummy, 1);
}

void uartWaitTxComplete(void)
{
  while(uartGetTxCounter());
  while(!(UCAxIFG & UCTXCPTIE));
}

bool uartTxWait(const void *buf_p, uint16_t len)
{
  bool status = false;
  uint16_t blockLen;
  if(len > 0 && buf_p != NULL)
  {
    while(len > 0)
    {
      blockLen = CONSOLE_UART_TX_BUFFER_SIZE - uartGetTxCounter();
      if(blockLen > 0)
      {
        if(blockLen > len)
          blockLen = len;
        uartTx(buf_p, blockLen);
        buf_p = (unsigned char*)buf_p + blockLen;
        len -= blockLen;
      }
    }
    status = true;
  }
  return status;
}

uint16_t uartRx(const void *buf_p, uint16_t len)
{
  uint16_t intState;
  uint16_t index = 0;
  ENTER_CRITICAL_SECTION(intState);
  if(len && rxBufferCounter)
  {
    if(len > rxBufferCounter) //Ограничиваем число считываемых байт.
      len = rxBufferCounter;
    while(index < len)
    {
      ((uint8_t *)buf_p)[index++] = rxBuffer[rxBufferPointer++];
      if(rxBufferPointer >= CONSOLE_UART_RX_BUFFER_SIZE)
        rxBufferPointer -= CONSOLE_UART_RX_BUFFER_SIZE;
    }
    rxBufferCounter -= len;
  }
  else
    len = 0; //Обнуляем для случая равенства нулю rxBufferCounter.
  EXIT_CRITICAL_SECTION(intState);
  return len;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 *********************************************************************/
uint16_t uartGetTxCounter(void)
{
  uint16_t counter;
  CRITICAL_STATEMENT(counter = txBufferCounter);
  return counter;
}

void uartTx(const void *buf_p, const uint16_t len)
{
  uint16_t intState;
  uint16_t index = 0;
  uint16_t pointer;
  uint16_t offset;
  ENTER_CRITICAL_SECTION(intState);
  pointer = txBufferPointer;
  offset = pointer + txBufferCounter;
  if(offset >= CONSOLE_UART_TX_BUFFER_SIZE)
    offset -= CONSOLE_UART_TX_BUFFER_SIZE;
  while(index < len) //Переписываем данные в выходной буфер.
  {
    txBuffer[offset++] = ((const uint8_t *)buf_p)[index++];
    txBufferCounter++;
    if(offset == CONSOLE_UART_TX_BUFFER_SIZE)
      offset = 0;
  }
  if(index > 0 && !(UCAxIE & UCTXIE))
  {
    UCAxIFG |= UCTXIFG;
    UCAxIE |= UCTXIE | UCTXCPTIE;
  }
  EXIT_CRITICAL_SECTION(intState);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=USCI_Ax_VECTOR
  __interrupt void USCI_Ax_ISR(void)
#elif defined(__GNUC__)
  void __attribute__ ((interrupt(USCI_Ax_VECTOR))) USCI_Ax_ISR (void)
#else
  #error Compiler not supported!
#endif
{
  uint16_t temp;
  uint16_t offset;
  switch(__even_in_range(UCAxIV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE:
      break;
    case USCI_UART_UCRXIFG:
      //if(rxBufferCounter ==CONSOLE_UART_RX_BUFFER_SIZE) //Раскомментировать если ролловер буфера не желателен.
      //break;
      temp = rxBufferPointer; //Если не скопировать в локальную переменную, то будет предупреждение, потому что txBufferHead объявлена как volatile.
      offset = temp + rxBufferCounter;
      if(offset >= CONSOLE_UART_RX_BUFFER_SIZE)
      {
        offset -= CONSOLE_UART_RX_BUFFER_SIZE;
      }
      rxBuffer[offset] = UCAxRXBUF;
      if(rxBufferCounter == CONSOLE_UART_RX_BUFFER_SIZE)
      {
        rxBufferPointer++;
        if(rxBufferPointer == CONSOLE_UART_RX_BUFFER_SIZE)
        {
          rxBufferPointer = 0;
        }
      }
      else
      {
        rxBufferCounter++;
      }
      break;
    case USCI_UART_UCTXIFG:
      if(txBufferCounter)
      {
        txBufferCounter--;
        UCAxTXBUF = txBuffer[txBufferPointer++];
        if(txBufferPointer == CONSOLE_UART_TX_BUFFER_SIZE)
        {
          txBufferPointer = 0;
        }
      }
      else
      {
        UCAxIE &= ~(UCTXIE | UCTXCPTIE); //Если передали последний байт в буфере, то запрещаем соответствующие прерывания.
      }
        break;
    case USCI_UART_UCSTTIFG:
      break;
    case USCI_UART_UCTXCPTIFG:
      break;
    default:
      break;
  }
}
#endif //ENABLE_CONSOLE.
