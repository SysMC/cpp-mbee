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
// BOOSTXL-MBEE868-1.1.H - Файл, переопределяющий стандартные линии вывода LaunchPad
// в случае использования платы BOOSTXL-MBEE868-1.1.
//
//*****************************************************************************
#ifndef BOOSTXL_MBEE868_1_1_H
#define BOOSTXL_MBEE868_1_1_H

#ifdef __MSP430FR4133__
  #define GPIO_PORT_LAMP1                 GPIO_PORT_P1
  #define GPIO_PIN_LAMP1                  GPIO_PIN4

  #define GPIO_PORT_LAMP2                 GPIO_PORT_P1
  #define GPIO_PIN_LAMP2                  GPIO_PIN7

  #define GPIO_PORT_BUTTON1               GPIO_PORT_P5
  #define GPIO_PIN_BUTTON1                GPIO_PIN3

  #define GPIO_PORT_DIP1                  GPIO_PORT_P1
  #define GPIO_PIN_DIP1                   GPIO_PIN6

  #define GPIO_PORT_POTENTIOMETER         GPIO_PORT_P8
  #define GPIO_PIN_POTENTIOMETER          GPIO_PIN1

#endif // __MSP430FR4133__

#ifdef __MSP430FR2311__

#endif // __MSP430FR2311__

#ifdef __MSP430FR2433__

#endif // __MSP430FR2433__

#ifdef __MSP430FR2355__
  #define GPIO_PORT_LAMP1                 GPIO_PORT_P2
  #define GPIO_PIN_LAMP1                  GPIO_PIN5

  #define GPIO_PORT_LAMP2                 GPIO_PORT_P2
  #define GPIO_PIN_LAMP2                  GPIO_PIN0

  #define GPIO_PORT_LAMP3                 GPIO_PORT_P6
  #define GPIO_PIN_LAMP3                  GPIO_PIN0

  #define GPIO_PORT_LAMP4                 GPIO_PORT_P6
  #define GPIO_PIN_LAMP4                  GPIO_PIN1

  #define GPIO_PORT_BUTTON1               GPIO_PORT_P4
  #define GPIO_PIN_BUTTON1                GPIO_PIN7

  #define GPIO_PORT_BUTTON2               GPIO_PORT_P3
  #define GPIO_PIN_BUTTON2                GPIO_PIN1

  #define GPIO_PORT_BUTTON3               GPIO_PORT_P2
  #define GPIO_PIN_BUTTON3                GPIO_PIN4

  #define GPIO_PORT_BUTTON4               GPIO_PORT_P3
  #define GPIO_PIN_BUTTON4                GPIO_PIN3

  #define GPIO_PORT_DIP1                  GPIO_PORT_P2
  #define GPIO_PIN_DIP1                   GPIO_PIN2

  #define GPIO_PORT_DIP2                  GPIO_PORT_P3
  #define GPIO_PIN_DIP2                   GPIO_PIN5

  #define GPIO_PORT_BUZZER                GPIO_PORT_P2
  #define GPIO_PIN_BUZZER                 GPIO_PIN1

  #define GPIO_PORT_POTENTIOMETER         GPIO_PORT_P1
  #define GPIO_PIN_POTENTIOMETER          GPIO_PIN5

  #define GPIO_PORT_MODULE_RESET          GPIO_PORT_P6
  #define GPIO_PIN_MODULE_RESET           GPIO_PIN3

  #define GPIO_PORT_MODULE_SYSTEM_BUTTON  GPIO_PORT_P6
  #define GPIO_PIN__MODULE_SYSTEM_BUTTON  GPIO_PIN4

  #define GPIO_PORT_MODULE_SLEEP_REQUEST  GPIO_PORT_P5
  #define GPIO_PIN_MODULE_SLEEP_REQUEST   GPIO_PIN4

  #define GPIO_PORT_MODULE_SLEEP_STATUS   GPIO_PORT_P1
  #define GPIO_PIN__MODULE_SLEEP_STATUS   GPIO_PIN1

#endif // __MSP430FR2355__

#endif // BOOSTXL_MBEE868_1_1_H

