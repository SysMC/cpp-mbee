/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Alarm/Src/main.c
  * @author  von Boduen
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics and Systems, modules and components.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST and SMC under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MBee.h" //Подключаем С++ библиотеку для работы с радиомодулями.
#if defined(ENABLE_CONSOLE)
  #include "console.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Значения кодов событий. Представляют собой номера битов переменной events. Число событий не должно превышать 32.
#define BUTTON1_PRESSED_EVENT           0
#define IO_SAMPLE_PACKET_RECEIVED_EVENT 1
#define ADC_CONVERSION_COMPLETE_EVENT   2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SET_EVENT(evt)    (events |= BV(evt))
#define RESET_EVENT(evt)  (events &= ~BV( evt))
#define TEST_EVENT(evt)   (events & BV(evt))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static SerialStarWithCallbacks MBee; //Объект для взаимодействия с радиомодулем с помощью callback-функций.
static Serial serial("USART1"); //Объект для работы с COM-портом. Название может быть произвольным так как конкретный модуль USART определяется в CubeMX.
static RxIoSampleResponse ioPacket; //Объект для принятого пакета с данными о состоянии линий ввода/вывода удаленного радиомодуля.
static RemoteAtCommandRequest remoteAtRequest; //Объект для пакета с AT-командой удаленному модулю.
static TxStatusResponse txStatus; //Объект для пакета с локальным ответом на AT-команду удаленному модулю.
static RemoteAtCommandResponse remoteAtResponse; //Объект для пакета с ответом от удаленного модуля.
static uint8_t buttonMode; //Переменная, в которой хранится режим кнопки, оределяющий реакцию на ее нажатие.
static uint8_t cmdParam[2]; //Параметр команды. 2- максимальная длина параметра команды, которая будет использоваться в этом примере.
static uint32_t potentiometerBrigthnessSetting; //Текущее значение яркости, установленное с помощью потенциометра.
static uint32_t currentTime; //Время, прошедшее с момента включения данного узла.
static uint32_t timer; //Переменная для измерения интервалов времени.
static volatile  uint16_t events; //Битовое поле с флагами событий.
static uint32_t currentPotentiometerBrigthnessSetting;
#if defined(ENABLE_CONSOLE)
  static Console console("USART2"); //Объект для интерактивного управления радиомодулем и вывода диагностических сообщений через дополнительный UART. Название может быть произвольным так как конкретный модуль USART определяется в CubeMX.
  static int32_t brigthness; //Текущее значение яркости светодиода удаленного модуля.
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter); //Callback-функция, вызываемая библиотекой cpp-mbee при приеме пакета 0x83.
static void parseIoSamplePacket(void);
static void targetSetup(void);
static void targetDigitalOutSet(uint32_t level);
static void targetPWMOutSetDutyCycle(uint32_t dutyCycle);
static void sendRequest(void);
static void sendCommand(void);
static bool potentiometerPoll(void);
static void signal(uint32_t count);
extern void MBEE_HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);
extern void MBEE_HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
#if defined(ENABLE_CONSOLE)
  extern void CONSOLE_HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);
  extern void CONSOLE_HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  serial.begin(MBEE_UART_BITRATE);
  MBee.begin(serial);
  targetSetup(); //Осуществляем начальную настройку неизменяющихся полей объекта с командой.
  #if defined(ENABLE_CONSOLE)
    console.begin(CONSOLE_UART_BITRATE);
  #endif
  MBee.onRxIoSampleResponse(ioSamplePacketReceived); //Регистрация callback-функции для приема пакетов 0x83 с состоянием линий ввода-вывода.
  timer = millis(); //Записываем начальное значение в таймер.
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MBee.run();
    #if defined(ENABLE_CONSOLE)
      switch(console.run())
      {
        case '1':
          targetDigitalOutSet(IO_DIGITAL_OUTPUT_HI); //Включаем дискретный цифровой выход на удаленном модуле.
          break;
        case '2':
          targetDigitalOutSet(IO_DIGITAL_OUTPUT_LO); //Выключаем дискретный цифровой выход на удаленном модуле.
          break;
        case '+':
          brigthness += PWM_DUTY_CYCLE_MAX / BRIGTHNESS_NUMBER_OF_STEPS; //Увеличиваем яркость светодиода (рабочий цикл PWM) на одну ступень.
          if(brigthness >= (int)PWM_DUTY_CYCLE_MAX)
            brigthness = PWM_DUTY_CYCLE_MAX;
          targetPWMOutSetDutyCycle(brigthness);
          break;
        case '-':
          brigthness -= PWM_DUTY_CYCLE_MAX / BRIGTHNESS_NUMBER_OF_STEPS; //Уменьшаем яркость светодиода (рабочий цикл PWM) на одну ступень.
          if(brigthness <= 0)
            brigthness = 0;
          targetPWMOutSetDutyCycle(brigthness);
          break;
        default:
          break;
      }
    #endif
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
    if(TEST_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT))
    {
      RESET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
      parseIoSamplePacket();
    }
    if(TEST_EVENT(ADC_CONVERSION_COMPLETE_EVENT))
    {
      RESET_EVENT(ADC_CONVERSION_COMPLETE_EVENT);
      if(potentiometerPoll()) //Если положение потенциометра изменилось более чем на заданную величину (определяется POTENTIOMETER_SENSITIVITY), то отправляем команду на удаленный модуль.
      {
        targetPWMOutSetDutyCycle(potentiometerBrigthnessSetting);
      }
      if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) //Запускаем очередное преобразование.
      {
        Error_Handler();
      }
    }
    currentTime = millis();
    if(currentTime - timer > REMOTE_MODULE_POLL_PERIOD) //Проверяем, не пора ли опросить удаленный модуль.
    {
      sendRequest();
      timer = currentTime;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV5;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Callback-функция, вызываемая при приеме пакета 0x83.
  *        Так как callback-функция должна быть максимально короткая и не содержать задержек, то в ней просто выставляем событие и копируем принятый пакет.
  * @param  ioSample: ссылка на объект с принятым пакетом 0x83.
  * @param  uintptr_t: опциональный параметр. См. документацию к библиотеке cpp-mbee.
  * @retval None
  */
void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter)
{
  ioPacket = ioSample;
  SET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
}

/**
  * @brief Разбор принятого пакета 0x83
  * @retval None
  */
void parseIoSamplePacket(void)
{
  uint8_t sampleNumber;
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
            HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_SET); //Напряжение на аналоговом входе больше заданного порога. Включаем светодиоды и звуковой сигнал.
            HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
            if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
            {
              Error_Handler();
            }
          }
          else
          {
            HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_RESET); //Напряжение на аналоговом входе меньше заданного порога. Выключаем светодиоды и звуковой сигнал.
            HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
            if(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK)
            {
              Error_Handler();
            }
          }
        }
      }
    }
  }
  #if defined(ENABLE_CONSOLE)
    HAL_GPIO_TogglePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin); //Для отладки. С целью измерения времени передачи данных в буфер и дополнительной индикации приема пакета.
    HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
    console.printIoSamplePacket(ioPacket);
    HAL_GPIO_TogglePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin); //Для отладки. С целью измерения времени передачи данных в буфер и дополнительной индикации приема пакета.
    HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
  #endif
}

/**
  * @brief  Установка неизменяющихся полей пакета с командой удаленному модулю.
  * @retval None
  */
void targetSetup(void)
{
  remoteAtRequest.setDefault(); //Приводим все поля команды для передачи к значениям "по умолчанию" для демонстрации наличия этой функции. С помощью этой функции упрощается повторное использование объекта для отправки последующих команд.
  remoteAtRequest.setCommandValue(&cmdParam[0]); //Устанавливаем указатель на переменную с параметром команды.
}

/**
  * @brief  Отправка удаленному модулю команды на включение/выключение цифрового выхода.
  * @param  level: Уровень, который требуется установить на данном цифровом выходе.
  * @retval None
  */
void targetDigitalOutSet(uint32_t level)
{
  remoteAtRequest.setCommand((uint8_t*)"L5"); //Устанавливаем код команды - вывод L5 (Сигнал DO3 на плате MB-Tag-1.1) переводим в режим цифрового выхода.
  cmdParam[0] = level; //Устанавливаем значения параметра команды.
  remoteAtRequest.setCommandValueLength(1); //Длина поля параметра команды 1 байт.
  sendCommand();
}

/**
  * @brief  Отправка удаленному модулю команды установки на PWM-выходе требуемого значения
  * @retval None
  */
void targetPWMOutSetDutyCycle(uint32_t dutyCycle)
{
  remoteAtRequest.setCommand((uint8_t*)"M1"); //Устанавливаем код команды M1 - установка значения рабочего цикла первого канала PWM.
  cmdParam[0] = dutyCycle >> 8; //Устанавливаем значения параметра команды, преобразуя значение в big endian.
  cmdParam[1] = dutyCycle;
  remoteAtRequest.setCommandValueLength(2); //Длина поля параметра команды 2 байт.
  sendCommand();
}

/**
  * @brief  Передача запроса о состоянии линий ввода/вывода удаленному модулю.
  * @retval None
  */
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

/**
  * @brief  Передача команды удаленному модулю.
  * @retval None
  */
void sendCommand(void)
{
  bool acknowledge;
  //Далее приводятся возможные опции отправки (см. описание опций в документации к библиотеке cpp-mbee.).
  remoteAtRequest.setSaveChanges(false); //Значение параметра, переданное в команде не будет сохраняться в энергонезависимой памяти удаленного радиомодуля.
  //remoteAtRequest.setCca(false); //Отключает режим CCA (Clear Channel Assessment) перед отправкой команды.
  //remoteAtRequest.setAcknowledge(false); //Для повышения пропускной способности можно отключить подтверждение получения команды от удаленным модемом.
  //remoteAtRequest.setApplyChanges(false); //Команды будет выполняться сразу после приема.
  //remoteAtRequest.setSleepingDevice(true); //Команда предназначается спящему модему.
  acknowledge = remoteAtRequest.getAcknowledge(); //Сохраняем текущие установки удаленного подтверждения.
  //Устанавливаем режим адресации в зависимости от положении входа ADDRESS_MODE_SWITCH.
  if(HAL_GPIO_ReadPin(ADDRESS_MODE_SWITCH_GPIO_Port, ADDRESS_MODE_SWITCH_Pin) == GPIO_PIN_SET) //Вход, управляющий режимом адресации разомкнут (в высоком уровне) - устанавливаем широковещательный режим передачи.
  {
    remoteAtRequest.setRemoteAddress(BROADCAST_ADDRESS);
    remoteAtRequest.setAcknowledge(false); //Чтобы не вызвать шквал пакетов, отключаем удаленное подтверждение доставки, если адрес получателя широковещательный.
  }
  else
  {
    remoteAtRequest.setRemoteAddress(TARGET_ADDRESS);
  }
  MBee.send(remoteAtRequest); //Отправляем команду на удаленный модуль.
  if(remoteAtRequest.getFrameId()) //Если frameID = 0, то локальный ответ не формируется и поэтому его не ждем.
  {
    if(MBee.readPacket(LOCAL_RESPONSE_TIMEOUT)) //Ждем локального ответа со статусом команды.
    {
      if(MBee.getResponse().getApiId() == TRANSMIT_STATUS_API_FRAME)//Проверяем, является ли принятый пакет локальным ответом на AT-команду удаленному модему.
      {
        MBee.getResponse().getTxStatusResponse(txStatus); //Получаем объект с локальным статусом отправки.
        #if defined(ENABLE_CONSOLE)
          if(txStatus.isSuccess())
          {
            if(remoteAtRequest.getSleepingDevice())
            {
              printf("Command is successful buffered for sleeping target with address: %x.\n\r", TARGET_ADDRESS);
            }
            else
            {
              printf("Command is successful transmitted to the target with address: %x.\n\r", TARGET_ADDRESS);
            }
          }
          else if(txStatus.getStatus() == ERROR_COMMAND_STATUS)
          {
            printf("Error! Unsufficient memory for ioPacket.\n\r");
          }
          else if(txStatus.getStatus() == INVALID_CODE_COMMAND_STATUS)
          {
            printf("Error! Invalid command code.\n\r");
          }
          else if(txStatus.getStatus() == INVALID_PARAMETER_COMMAND_STATUS)
          {
            printf("Error! Invalid parameter value.\n\r");
          }
          else if(txStatus.getStatus() == TX_FAILURE_COMMAND_STATUS)
          {
            printf("Warning! Command is not transmitted due to CCA failure.\n\r");
          }
      #endif
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
          signal(1); //Команда успешно доставлена на удаленый модем.
          #if defined(ENABLE_CONSOLE)
            printf("Command is successful delivered to the target with address: %x.\n\r", TARGET_ADDRESS); //Получение команды подтверждено удаленным модулем.
          #endif
        }
      }
    }
    else
    {
      signal(3); //Нет ответа на команду от удаленного модема.
      #if defined(ENABLE_CONSOLE)
        printf("No response from the target with address: %x.\n\r", TARGET_ADDRESS); //Нет ответа от удаленного модуля.
      #endif
    }
  }
  remoteAtRequest.setAcknowledge(acknowledge); //Восстанавливаем прежний режим удаленного подтверждения.
}

/**
  * @brief  Установка рабочего цикла PWM в зависимости от положения потенциометра.
  * @retval status: true если текущее значение изменилось более, чем на заданную величину.
  */
bool potentiometerPoll(void)
{
  bool status = false;
  int32_t delta;
  currentPotentiometerBrigthnessSetting = (HAL_ADC_GetValue(&hadc1) * PWM_DUTY_CYCLE_MAX) / 4095;
  delta = potentiometerBrigthnessSetting - currentPotentiometerBrigthnessSetting;
  if(delta < 0)
  {
    delta = - delta;
  }
  if(delta >= (int)((PWM_DUTY_CYCLE_MAX / 100) * POTENTIOMETER_SENSITIVITY)) //Проверяем, превышает ли разность между установленным уровенем яркости и текущим порог чувствительности.
  {
    potentiometerBrigthnessSetting = currentPotentiometerBrigthnessSetting;
    status = true;
  }
  return status;
}

/**
  * @brief Включает светодиод и формирует звуковой сигнал заданное число раз.
  * @param  count: Число включений.
  * @retval None
  */
void signal(uint32_t count)
{
  while(count--)
  {
    HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_SET); //Включаем светодиод, подключенный к плате NUCLEO.
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET); //Включаем пользовательский светодиод.
    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) //Включаем звуковой сигнал.
    {
      Error_Handler();
    }
    HAL_Delay(LED_ON_TIME);
    HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_RESET); //Выключаем светодиод, подключенный к плате NUCLEO.
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET); //Выключаем пользовательский светодиод.
    if(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK) //Выключаем звуковой сигнал.
    {
      Error_Handler();
    }
    HAL_Delay(LED_ON_TIME);
  }
}

/**
  * @brief Callback-функция, вызываемая при передаче каждого байта по UART.
  * @param  UartHandle: указатель на handler UART, который закончил передачу байта.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if(UartHandle == &huart1)
  {
    MBEE_HAL_UART_TxCpltCallback(UartHandle);
  }
  #if defined(ENABLE_CONSOLE)
    else if(UartHandle ==  &huart2)
    {
      CONSOLE_HAL_UART_TxCpltCallback(UartHandle);
    }
  #endif
}

/**
  * @brief Callback-функция, вызываемая при приеме каждого байта по UART.
  * @param  UartHandle: указатель на handler UART, на который был принят байт.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if(UartHandle == &huart1)
  {
    MBEE_HAL_UART_RxCpltCallback(UartHandle);
  }
  #if defined(ENABLE_CONSOLE)
    else if(UartHandle == &huart2)
    {
      CONSOLE_HAL_UART_RxCpltCallback(UartHandle);
    }
  #endif
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == NUCLEO_BUTTON_Pin) || (GPIO_Pin == USER_BUTTON1_Pin))
  {
    SET_EVENT(BUTTON1_PRESSED_EVENT);
    HAL_Delay(50); //Антидребезг.
  }
}

/**
  * @brief Callback-функция, вызываемая при окончании выполнения каждого аналого-цифрового преобразования.
  * @param  UartHandle: указатель на handler АЦП.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  SET_EVENT(ADC_CONVERSION_COMPLETE_EVENT);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
