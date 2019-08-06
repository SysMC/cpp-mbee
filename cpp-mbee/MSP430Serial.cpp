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
#include <msp430.h>
#include "System.h"

/*********************************************************************
 * MACROS
 */
#define st(x) do{x} while(__LINE__ == -1)
#define ENTER_CRITICAL_SECTION(x)       st((x) = __get_SR_register() & GIE; /*Атомарно сохраняем регистр STATUS.*/\
                                        __disable_interrupt();)
#define EXIT_CRITICAL_SECTION(x)        st(__bis_SR_register (x);)
#define CRITICAL_STATEMENT(x)           st(interruptState_t s;\
                                            ENTER_CRITICAL_SECTION(s);\
                                            (x);\
                                            EXIT_CRITICAL_SECTION(s);)

/*********************************************************************
 * CONSTANTS
 */
//Выбор модуля UART для работы с радиомодулем.
#ifndef MSP_MBEE_UART_MODULE
  #define MSP_MBEE_UART_MODULE  EUSCI_A0_BASE
  //#define MSP_MBEE_UART_MODULE  EUSCI_A1_BASE
#endif

//Выбор модуля таймера.
#ifndef MSP_MBEE_TIMER_MODULE
  //#define MSP_MBEE_TIMER_MODULE TIMER_A0_BASE
  //#define MSP_MBEE_TIMER_MODULE TIMER_A1_BASE
  #define MSP_MBEE_TIMER_MODULE TIMER_B0_BASE
#endif

#define UART_TX_BUFFER_SIZE 128u
#define UART_RX_BUFFER_SIZE 128u

/*********************************************************************
 * DIAGNOSTICS
 */
#if (MSP_MBEE_UART_MODULE == EUSCI_A0_BASE)
  #warning Module eUSCI_A0 is selected as UART for MBee library. Set appropriate pins for peripheral function!
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
#elif (MSP_MBEE_UART_MODULE == EUSCI_A1_BASE)
  #warning Module eUSCI_A1 is selected as UART for MBee library. Set appropriate pins for peripheral function!
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
  #error Module for UART must be selected!
#endif

#if (MSP_MBEE_TIMER_MODULE == TIMER_A0_BASE)
  #warning Module Timer_A0 is selected as MBee library timer.
  #define TxxCTL    TA0CTL
  #define TxxEX0    TA0EX0
  #define TxxCCR0   TA0CCR0
  #define TxxCCTL0  TA0CCTL0
  #define TIMERx_x0_VECTOR  TIMER0_A0_VECTOR
  #define TIMERx_x0_ISR     TIMER0_A0_ISR
#elif (MSP_MBEE_TIMER_MODULE == TIMER_A1_BASE)
  #warning Module Timer_A1 is selected as MBee library timer.
  #define TxxCTL    TA1CTL
  #define TxxEX0    TA1EX0
  #define TxxCCR0   TA1CCR0
  #define TxxCCTL0  TA1CCTL0
  #define TIMERx_x0_VECTOR  TIMER1_A0_VECTOR
  #define TIMERx_x0_ISR     TIMER1_A0_ISR
#elif (MSP_MBEE_TIMER_MODULE == TIMER_B0_BASE)
  #warning Module Timer_B0 is selected as MBee library timer.
  #define TxxCTL    TB0CTL
  #define TxxEX0    TB0EX0
  #define TxxCCR0   TB0CCR0
  #define TxxCCTL0  TB0CCTL0
  #define TIMERx_x0_VECTOR  TIMER0_B0_VECTOR
  #define TIMERx_x0_ISR     TIMER0_B0_ISR
#else
  #error Module for timer must be selected!
#endif

/*********************************************************************
 * TYPEDEFS
 */
typedef uint16_t interruptState_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
uint16_t millisCorrector = 1;

/*********************************************************************
 * LOCAL VARIABLES
 */

#if defined(__IAR_SYSTEMS_ICC__)
  __no_init static uint8_t txBuffer[UART_TX_BUFFER_SIZE]; //Буфера не инициализируем для сокращения времени рестарта.
  __no_init static uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
#elif defined(__TI_COMPILER_VERSION__)
  #pragma NOINIT
  static uint8_t txBuffer[UART_TX_BUFFER_SIZE]; //Буфера не инициализируем для сокращения времени рестарта.
  #pragma NOINIT
  static uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
#elif defined(__GNUC__)
  static uint8_t txBuffer[UART_TX_BUFFER_SIZE] __attribute__((noinit));
  static uint8_t rxBuffer[UART_RX_BUFFER_SIZE] __attribute__((noinit));
#endif

static volatile uint16_t txBufferPointer, txBufferCounter; //Инициализируем 0-ми указатели и счетчики буферов.
static volatile uint16_t rxBufferPointer, rxBufferCounter;
static uint64_t sysTickCounter;

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
   //SizeBuffer = UART_RX_BUFFER_SIZE; //Этот член класса не используется так как в версии библиотеки для MSP430 применяются раздельные статические буфера на прием и на передачу.
}

int Serial::available()
{
  uint16_t counter;
  interruptState_t intState;
  ENTER_CRITICAL_SECTION(intState);
  counter = rxBufferCounter;
  EXIT_CRITICAL_SECTION(intState);
  return counter;
}

uint8_t Serial::read() //Поскольку в библиотеке чтение всегда осуществляется по одному байту и только после вызова функции available, то дополнительные проверки на пустоту буфера проводить не надо.
{
  uint8_t byte;
  interruptState_t intState;
  ENTER_CRITICAL_SECTION(intState);
  byte = rxBuffer[rxBufferPointer++];
  if(rxBufferPointer >= UART_RX_BUFFER_SIZE)
  {
    rxBufferPointer -= UART_RX_BUFFER_SIZE;
  }
  rxBufferCounter--;
  EXIT_CRITICAL_SECTION(intState);
  return byte;
}

void Serial::write(uint8_t byte)
{
  interruptState_t intState;
  ENTER_CRITICAL_SECTION(intState);
  uint16_t temp = txBufferPointer;
  uint16_t offset = temp + txBufferCounter;
  if(offset >= UART_TX_BUFFER_SIZE)
  {
    offset -= UART_TX_BUFFER_SIZE;
  }
  txBuffer[offset++] = byte;
  txBufferCounter++;
  if(offset == UART_TX_BUFFER_SIZE)
  {
    offset = 0u;
  }
  if(!(UCAxIE & UCTXIE))
  {
    UCAxIFG |= UCTXIFG;
    UCAxIE |= UCTXIE | UCTXCPTIE;
  }
  EXIT_CRITICAL_SECTION(intState);
}

bool Serial::begin(unsigned long bitrate)
{
  //Константы для регистров рассчитаы с помощью утилиты MSP430 USCI/EUSCI UART Baud Rate Calculation, доступной на сайте Texas Insruments поиском по названию.
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
   //Source - SMCLK, Clock - 125 KHz, UP-mode, Period - 1 ms.
  TxxCTL = 0x02D0;
  TxxEX0 = 0x0007;
  TxxCCR0 = 125;
  TxxCCTL0 = 0x0010;
  return true;
}

void Serial::end()
{

}

uint64_t millis()
{
  uint64_t time;
  CRITICAL_STATEMENT(time = sysTickCounter);
  return time;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 *********************************************************************/
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
      //if(rxBufferCounter ==UART_RX_BUFFER_SIZE) //Раскомментировать если ролловер буфера не желателен.
      //break;
      temp = rxBufferPointer; //Если не скопировать в локальную переменную, то будет предупреждение, потому что txBufferHead объявлена как volatile.
      offset = temp + rxBufferCounter;
      if(offset >= UART_RX_BUFFER_SIZE)
      {
        offset -= UART_RX_BUFFER_SIZE;
      }
      rxBuffer[offset] = UCAxRXBUF;
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
      break;
    case USCI_UART_UCTXIFG:
      if(txBufferCounter)
      {
        txBufferCounter--;
        UCAxTXBUF = txBuffer[txBufferPointer++];
        if(txBufferPointer == UART_TX_BUFFER_SIZE)
        {
          txBufferPointer = 0u;
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

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector=TIMERx_x0_VECTOR
  __interrupt void TIMERx_x0_ISR(void)
#elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMERx_x0_VECTOR))) TIMERx_x0_ISR (void)
#else
  #error Compiler not supported!
#endif
{
  sysTickCounter += millisCorrector;
  millisCorrector = 1;
}


