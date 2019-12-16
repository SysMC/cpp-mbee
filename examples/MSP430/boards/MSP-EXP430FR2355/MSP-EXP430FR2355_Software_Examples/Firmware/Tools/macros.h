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
// macros.h - файл с полезными макроопределениями.
//
//*****************************************************************************
#define BV(n) (1 << (n))
#define st(x) do{x} while(__LINE__ == -1)
#define ENTER_CRITICAL_SECTION(x)       st((x) = __get_SR_register() & GIE; /*Атомарно сохраняем регистр STATUS.*/\
                                        __disable_interrupt();)
#define EXIT_CRITICAL_SECTION(x)        st(__bis_SR_register (x);)
#define CRITICAL_STATEMENT(x)           st(uint16_t s;\
                                            ENTER_CRITICAL_SECTION(s);\
                                            (x);\
                                            EXIT_CRITICAL_SECTION(s);)

#define SET_EVENT(evt)    (events |= BV(evt))
#define RESET_EVENT(evt)  (events &= ~BV( evt))
#define TEST_EVENT(evt)   (events & BV(evt))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((unsigned long)((unsigned long)((Byte0) & 0x00FF) \
          + ((unsigned long)((Byte1) & 0x00FF) << 8) \
          + ((unsigned long)((Byte2) & 0x00FF) << 16) \
          + ((unsigned long)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((unsigned short)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
          ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

//Write the 16 bit value of 'val' in little endian format to the buffer pointed to by pBuf, and increment pBuf by 2
#define UINT16_TO_BUF_LITTLE_ENDIAN(pBuf,val) \
	do { \
		*(pBuf)++ = ((((unsigned short)(val)) >>  0) & 0xFF); \
		*(pBuf)++ = ((((unsigned short)(val)) >>  8) & 0xFF); \
	}while (0)

#define UINT16_TO_BUF_BIG_ENDIAN(pBuf,val) \
	do { \
		*(pBuf)++ = ((((unsigned short)(val)) >> 8) & 0xFF); \
		*(pBuf)++ = ((((unsigned short)(val)) >> 0) & 0xFF); \
	}while (0)

//Write the 32bit value of 'val' in little endian format to the buffer pointed to by pBuf, and increment pBuf by 4
#define UINT32_TO_BUF_LITTLE_ENDIAN(pBuf,val) \
	do { \
		*(pBuf)++ = ((((unsigned long)(val)) >>  0) & 0xFF); \
		*(pBuf)++ = ((((unsigned long)(val)) >>  8) & 0xFF); \
		*(pBuf)++ = ((((unsigned long)(val)) >> 16) & 0xFF); \
		*(pBuf)++ = ((((unsigned long)(val)) >> 24) & 0xFF); \
	}while (0)

#define UINT32_TO_BUF_BIG_ENDIAN(pBuf,val) \
	do { \
		*(pBuf)++ = ((((unsigned long)(val)) >> 24) & 0xFF); \
		*(pBuf)++ = ((((unsigned long)(val)) >> 16) & 0xFF); \
		*(pBuf)++ = ((((unsigned long)(val)) >> 8) & 0xFF); \
		*(pBuf)++ = ((((unsigned long)(val)) >> 0) & 0xFF); \
	}while (0)
