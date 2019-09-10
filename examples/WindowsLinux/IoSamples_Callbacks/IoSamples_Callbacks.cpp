/**
  * Этот файл является частью библиотеки MBee C++.
  *
  * MBee C++ является бесплатным программным обеспечением.
  * Подробная информация о лицензиях находится в файле mbee.h.
  *
  * \author </i> von Boduen. Special thanx to Andrew Rapp.
  */
#include "../../../includes/MBee.h" //Подключаем библиотеку MBee C++.

/** Пример предназначен для демонстрации работы с библиотекой с помощью callback функций. 
  * Такой способ организации кода позволяет избавиться от необходимости "вручную"
  * в функции main() заниматься проверкой наличия поступивших от радиомодуля данных. Также 
  * автоматизируются многие действия, связанные с передачей пакетов.
  * Пример предназначен для демонстрации приема и обработки пакета с данными о состоянии 
  * цифровых и аналоговых датчиков на удаленном модеме. Принимающий и передающий модули MBee-868-x.0 
  * работают под управлением ПО Serial Star.
  * Действия, производимые в примере подробно описаны в комментариях к соответствующим строкам.
  * Потребуются 2 модуля MBee-868-x.0. Первый модуль соедининяется с компьютером c помощью 
  * MB-USBridge или любого другого совместимого USB-UART преобразователя. 
  * На обоих модулях, после загрузки программного обеспечения SerialStar, должен быть произведен возврат 
  * к заводским настройкам одним из двух способов:
  * 1. Быстрое 4-х кратное нажатие "SYSTEM BUTTON" (замыкание вывода №36 модуля на землю).
  * 2. С помощью командного режима: +++, AT RE<CR>, AT CN<CR>.
  * 
  * Первый модуль должен быть предварительно настроен для работы в пакетном режиме с escape-
  * символами AP=2.
  * Последовательность для настройки: 
  * 1. +++
  * 2. AT AP2<CR>
  * 3. AT CN<CR>
  * 
  * Второй модуль должен быть установлен на плату MB-Tag, или любую другую, обеспечивающую 
  * питание 3,3В. На модуле необходимо включить режим периодического сна с помощью следующей  
  * последовательности команд:
  * 1. +++
  * 2. AT SM4<CR>
  * 3. AT CN<CR>
  * Все прочие настройки можно оставить "по умолчанию".
  * 
  * Диагностические сообщения и результаты выводятся на экран компьютера.
  */
const char* portName = "\\\\.\\COM4"; //Назначаем имя COM-порта. ВНИМАНИЕ! Наличие префикса в имени COM-порта \\\\.\\ для Windows обязательно!
//const char* portName = "/dev/ttyAMA0"; //Назначаем имя порта для Raspberry.

const long portBitrate = 9600; //Определяем скорость COM-порта.

#define REQUESTED_PIN 29 //Номер вывода, используемый в примере запроса наличия в принятом пакете данных о его текущем состонии.

void rxIoSampleResponseCallback(RxIoSampleResponse& ioSample, uintptr_t optionalParameter); //Объявляем callback-функцию для обработки принятого пакета с данными о состоянии датчиков удаленного модема.
void printDigitalPinState(uint8_t state); //Объявляем вспомогательную функцию, используемую для печати уровней на цифровых линиях.

Serial serial(portName); //Создаем объект для работы с COM-портом.
SerialStarWithCallbacks mbee; //Создаем объект для взаимодействия с радиомодулем.

int main()
{
    if(serial.begin(portBitrate))
        printf("Port successful opened.");
    else
        printf("Error port opening. Port not exist, or occupied?");
    mbee.begin(serial);

    //Регистрируем типы пакетов, которые требуется принимать.
    mbee.onRxIoSampleResponse(rxIoSampleResponseCallback); //Регистрируем функцию, которая будет автоматически вызываться каждый раз после приема пакета с данными о состоянии цифровых и аналоговых датчиков удаленного узла.
    while(true)
        mbee.run();
}

//Определение зарегистрированной функции обработки пакета с данными о состоянии цифровых и аналоговых датчиков на удаленном модеме.
void rxIoSampleResponseCallback(RxIoSampleResponse& ioSample, uintptr_t optionalParameter)
{

    printf("\nReceived packet with I/O samples from remote modem with address: %#x", ioSample.getRemoteAddress()); //Печатаем адрес модуля от которого получен пакет.
    /**********************************************************************************************************/
    int8_t temp =  ioSample.getTemperature();
    if(temp > 128) //Переводим число из дополнительного кода в прямой.
        temp = -temp;//Температура отрицательная.
    printf("\nTemperature of the module is %dC.", temp); //Выводим температуру удаленного модема по показаниям встроенного датчика.
    /**********************************************************************************************************/
    if(ioSample.getSampleSize()) //Содержится ли в принятом пакете информация о состоянии линий ввода/вывода удаленного модема?
        printf("\nThere are(is) %d I/0 sample(s) in the packet.", ioSample.getSampleSize());

    /**********************************************************************************************************/
    if(ioSample.isAvailable(REQUESTED_PIN)) //Пример запроса о наличии в принятом пакете данных о конкретном выводе модуля.
        printf("\nPin #%d is sampled in the received packet.", REQUESTED_PIN);
    else
        printf("\nPin #%d is NOT sampled in the received packet..", REQUESTED_PIN);

    for(uint8_t i = 1; i <= ioSample.getSampleSize(); i++)// Выводим данные о текущем состоянии каждой линии ввода/вывода содержащейся а пакете.
    {
        printf("\nPin #%d is ", ioSample.getPin(i)); //Выводим идентификатор режима, на работу в котором настроена данная линия ввода/вывода.
        if((ioSample.getMode(i) & 0x7F) == IO_ADC) //Информация о текущем состоянии цифрового входа/выхода передается в старшем бите кода режима. Поэтому сбрасываем его, чтобы получить код режима.
            printf("analog input. Voltage: %.2fV.", float(ioSample.getAnalog(i))* 2.5 /4096); //2.5 - опорное напряжение АЦП модуля, 4096 - число уровней дискретизации (разрядность АЦП равна 12).
        else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_INPUT)
        {
            printf("digital input. State: ");
            printDigitalPinState(ioSample.getDigital(i)); //Пример получения информации о текущем состоянии цифровой линии с помощью специальной функции.
        }
        else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_OUTPUT_LO)
        {
            printf("digital output with default LOW state. State: ");
            printDigitalPinState(ioSample.getMode(i) & 0x80); //Пример получения информации о текущем состоянии цифровой линии с помощью старшего бита кода режима. Такой способ работает намного быстрее, чем вызов функции getDigital().
        }
        else if((ioSample.getMode(i) & 0x7F) == IO_DIGITAL_OUTPUT_HI)
        {
            printf("digital output with default HIGH state. State: ");
            printDigitalPinState(ioSample.getMode(i) & 0x80);
        }
        else if((ioSample.getMode(i) & 0x7F) == IO_COUNTER_INPUT1)
            printf("count input 1 with pullup. Pulse count: %d.", ioSample.getCounter(i));
        else if((ioSample.getMode(i) & 0x7F) == IO_COUNTER_INPUT2)
            printf("count input 2 with pullup. Pulse count: %d.", ioSample.getCounter(i));
        else if((ioSample.getMode(i) & 0x7F) == IO_WAKEUP_INPUT_FALLING_EDGE)
        {
            printf("wakeup(alarm) input with HIGH to LOW active front. State: ");
            printDigitalPinState(ioSample.getMode(i) & 0x80);
        }
        else if((ioSample.getMode(i) & 0x7F) == IO_WAKEUP_INPUT_RISING_EDGE)
        {
            printf("wakeup(alarm) input with LOW to HIGH active front. State: ");
            printDigitalPinState(ioSample.getMode(i) & 0x80);
        }
    }
}

//Вспомогательная функция, используемая для печати уровней на цифровых линиях.
void printDigitalPinState(uint8_t state)
{
    if(state)
        printf("HIGH.");
    else
        printf("LOW.");
}