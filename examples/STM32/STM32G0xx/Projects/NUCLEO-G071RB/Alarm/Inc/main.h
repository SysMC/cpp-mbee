/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NUCLEO_BUTTON_Pin GPIO_PIN_13
#define NUCLEO_BUTTON_GPIO_Port GPIOC
#define NUCLEO_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define CONSOLE_UART_TX_Pin GPIO_PIN_2
#define CONSOLE_UART_TX_GPIO_Port GPIOA
#define CONSOLE_UART_TXA3_Pin GPIO_PIN_3
#define CONSOLE_UART_TXA3_GPIO_Port GPIOA
#define NUCLEO_LED_Pin GPIO_PIN_5
#define NUCLEO_LED_GPIO_Port GPIOA
#define USER_BUTTON1_Pin GPIO_PIN_7
#define USER_BUTTON1_GPIO_Port GPIOA
#define USER_BUTTON1_EXTI_IRQn EXTI4_15_IRQn
#define MBEE_UART_TX_Pin GPIO_PIN_4
#define MBEE_UART_TX_GPIO_Port GPIOC
#define MBEE_UART_TXC5_Pin GPIO_PIN_5
#define MBEE_UART_TXC5_GPIO_Port GPIOC
#define USER_LED1_Pin GPIO_PIN_1
#define USER_LED1_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define ALARM_INPUT 4 //Номер вывода радиомодуля MBee-868-xx, к которому на удаленных узлах подключен цифровой датчик.
#define ACTUATOR_OUTPUT "R4" //Обозначение вывода радиомодуля MBee-868-xx, который на удаленных узлах используется в качестве цифрового выхода для управления каким-либо исполнительным устройством.
#define SENSOR_ADDRESSES_START 0x0002 //Начальный адрес диапазона адресов, назначенных датчикам. Применяется для фильтрации входящих пакетов.
#define SENSOR_ADDRESSES_END   0x000A //Конечный адрес диапазона адресов, назначенных датчикам. Применяется для фильтрации входящих пакетов.

//Определяем размеры буферов и скорость последовательного порта для взаимодействия с радиомодулем.
#define UART_TX_BUFFER_SIZE 512
#define UART_RX_BUFFER_SIZE 512
#define MBEE_UART_BITRATE 9600

#define ENABLE_CONSOLE //Закомментировать, если консоль не используется. В этим случае контроль работы и управление осуществляется только с помощью светодиодов и кнопок.
//Настраиваем параметры консоли.
#if defined(ENABLE_CONSOLE)
  #define CONSOLE_UART_TX_BUFFER_SIZE 1024
  #define CONSOLE_UART_RX_BUFFER_SIZE 128
  #define CONSOLE_UART_BITRATE 115200
#endif

#define BUZZER_FREQUENCY  855 //Частота сигнала, генерируемого PWM, Гц. Для 16 битных таймеров не должна быть меньше SystemCoreClock/65535.
#define BUZZER_PWM_PRESCALER_VALUE  (uint32_t)((SystemCoreClock >= 56000000)?((SystemCoreClock / 56000000) - 1): 0)
#define BUZZER_PWM_PERIOD_VALUE     (uint32_t)(SystemCoreClock/BUZZER_FREQUENCY - 1) //Период PWM.
#define BUZZER_PWM_PULSE_VALUE      (uint32_t)(BUZZER_PWM_PERIOD_VALUE / 2) //Рабочий цикл PWM.

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
