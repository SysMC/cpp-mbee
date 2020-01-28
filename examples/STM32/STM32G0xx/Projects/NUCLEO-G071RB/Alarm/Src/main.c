/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Alarm/Src/main.c
  * @author  von Boduen
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics and Systems, modules and components.
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
static RemoteAtCommandRequest remoteCommand; //Объект для организации отправки команд удаленным узлам.
static Serial serial("USART1"); //Объект для работы с COM-портом. Название может быть произвольным так как конкретный модуль USART определяется в CubeMX.
static RxIoSampleResponse ioPacket; //Объект для принятого пакета с данными о состоянии линий ввода/вывода удаленного радиомодуля.
static uint16_t remoteModuleAddress; //Адрес удаленного радиомодуля от которого получен пакет.
static GPIO_PinState remoteLedState; //Переменная, в которой хранится состояние светодиода на удаленном модуле.
static volatile  uint32_t events; //Битовое поле с флагами событий.
#if defined(ENABLE_CONSOLE)
  static Console console("USART2"); //Объект для интерактивного управления радиомодулем и вывода диагностических сообщений через дополнительный UART. Название может быть произвольным так как конкретный модуль USART определяется в CubeMX.
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void switchOffAlarm(void);
static void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter); //Callback-функция, вызываемая библиотекой cpp-mbee при приеме пакета 0x83 или 0x84.
static void parseIoSamplePacket(void);
static void sendCommandToRemoteModule(void);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  serial.begin(MBEE_UART_BITRATE);
  MBee.begin(serial);
  #if defined(ENABLE_CONSOLE)
    console.begin(CONSOLE_UART_BITRATE);
  #endif
  MBee.onRxIoSampleResponse(ioSamplePacketReceived); //Регистрация callback-функции для приема пакетов 0x83 или 0x84 с состоянием линий ввода-вывода.
  //Устанавливаем неизменяемые в процессе работы поля в объекте, предназначенном для отправки команд удаленному модулю.
  remoteCommand.setDefault(); //Приводим все поля команды для передачи к значениям "по умолчанию" для демонстрации наличия этой функции. С помощью этой функции упрощается повторное использование объекта для отправки последующих команд.
  remoteCommand.setAcknowledge(false); //Команды удаленному модулю будут отправляться без подтверждения их приема удаленным модулем.
  remoteCommand.setApplyChanges(true); //Команды будет выполняться сразу после приема.
  remoteCommand.setSaveChanges(false); //Значение параметра, переданное в команде не будет сохраняться в энергонезависимой памяти удаленного радиомодуля.
  remoteCommand.setCca(false); //Для того, чтобы команда была гарантировано передана в интервал времени, в течение которого удаленный модуль находится в состоянии приема, отключаем функцию CCA (Clear Channel Assessment).
  remoteCommand.setEncryption(false); //Отключаем функцию шифрования. Вызов здесь этой функции включен только для демонстрации, поскольку по-умолчанию шифрование отключено.
  remoteCommand.setSleepingDevice(true); //Команда предназначается спящему удаленному узлу, поэтому включаем буферизацию.
  remoteCommand.setCommand((uint8_t*)ACTUATOR_OUTPUT); //Установка кода команды удаленному модулю.
  remoteCommand.setCommandValueLength(1); //Длина команды управления режимом выводам равна 1 байт.
  remoteCommand.setFrameId(0); //Отключаем локальное подтверждение отправки команды в эфир, устанавливая FrameId равным 0.
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
          switchOffAlarm();
          break;
        case '2':
          sendCommandToRemoteModule();
          break;
        default:
          break;
      }
    #endif
    if(TEST_EVENT(BUTTON1_PRESSED_EVENT))
    {
      RESET_EVENT(BUTTON1_PRESSED_EVENT);
      switchOffAlarm();
      sendCommandToRemoteModule();
    }
    if(TEST_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT))
    {
      RESET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
      parseIoSamplePacket();
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Выключение сигналов "Тревога".
  * @retval None
  */
void switchOffAlarm(void)
{
  HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_RESET); //Выключаем светодиод, подключенный к плате NUCLEO.
  HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET); //Выключаем пользовательский светодиод.
  if(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK) //Выключаем звуковой сигнал.
  {
    Error_Handler();
  }
}

/**
  * @brief Callback-функция, вызываемая при приеме пакета 0x83 или 0x84.
  *        Так как callback-функция должна быть максимально короткая и не содержать задержек, то в ней просто выставляем событие и копируем принятый пакет.
  * @param  ioSample: ссылка на объект с принятым пакетом 0x83 или 0x84.
  * @param  uintptr_t: опциональный параметр. См. документацию к библиотеке cpp-mbee.
  * @retval None
  */
void ioSamplePacketReceived(RxIoSampleResponse& ioSample, uintptr_t optionalParameter)
{
  ioPacket = ioSample;
  SET_EVENT(IO_SAMPLE_PACKET_RECEIVED_EVENT);
}

/**
  * @brief Разбор принятого пакета 0x83 или 0x84.
  * @retval None
  */
void parseIoSamplePacket(void)
{
  uint8_t sampleNumber;
  remoteModuleAddress = ioPacket.getRemoteAddress(); //Сохраняем адрес модуля от которого приняли пакет.
  if((remoteModuleAddress >= SENSOR_ADDRESSES_START) && (remoteModuleAddress <= SENSOR_ADDRESSES_END)) //Фильтруем входящий пакет по диапазону адресов.
  {
    if(ioPacket.getSampleSize()) //Содержится ли в принятом пакете информация о состоянии линий ввода/вывода удаленного модема?
    {
      sampleNumber = ioPacket.getSampleNumber(ALARM_INPUT);
      if(sampleNumber) //Есть ли в принятом пакете информация о состоянии вывода радиомодуля, к которому подключен цифровой датчик и какой номер выборки он имеет?
      {
        if((ioPacket.getMode(sampleNumber) & 0x7F) == IO_DIGITAL_INPUT) //Настроен ли данный вывод как цифровой вход?
        {
          if(ioPacket.getDigital(sampleNumber) == LOW)
          {
            HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_SET); //Вход на удаленном модуле замкнут. Включаем светодиоды и звуковой сигнал.
            HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
            if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
            {
              Error_Handler();
            }
          }
          else
          {
            HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_RESET); //Вход на удаленном модуле разомкнут. Выключаем светодиоды и звуковой сигнал.
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
  * @brief Отправка команды удаленному радиомодулю.
  * @retval None
  */
void sendCommandToRemoteModule(void)
{
  uint8_t parameter;
  remoteCommand.setRemoteAddress(remoteModuleAddress); //Устанавливаем адрес получателя совпадающим с адресом отправителя.
  if(remoteLedState == GPIO_PIN_RESET) //В зависимости от текущего состояния цифрового выхода удаленного модуля устанавливаем на нем противоположный уровень.
  {
    parameter = IO_DIGITAL_OUTPUT_HI;
    remoteLedState = GPIO_PIN_SET;
  }
   else
  {
    parameter = IO_DIGITAL_OUTPUT_LO;
    remoteLedState = GPIO_PIN_RESET;
  }
  remoteCommand.setCommandValue(&parameter);
  MBee.send(remoteCommand);
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
    HAL_Delay(20); //Антидребезг.
  }
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
  while(1)
  {
  }
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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
