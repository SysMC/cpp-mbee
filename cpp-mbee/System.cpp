/* "Системы модули и компоненты" ("СМК"). 2018. Москва.
Библиотека C++ для модулей MBee .
Распространяется свободно. Надеемся, что программные продукты, созданные
с помощью данной библиотеки будут полезными, однако никакие гарантии, явные или
подразумеваемые не предоставляются.

The MIT License(MIT)

MBee C++ Library.
Copyright © 2018 Systems, modules and components. Moscow. Russia.

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

#include "includes/System.h"

#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
  #warning Library cpp-mbee is compiling for microcontrollers. Allocation for hardware timer is required. See readme.txt for details.
#else
  #include <time.h>
  uint64_t millis()
  {
    return(1000 * clock() / CLOCKS_PER_SEC);
  }
#endif

