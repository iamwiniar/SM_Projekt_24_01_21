/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "bmp280.h"
#include "bmp280_add.h"
#include "string.h"
#include "arm_math.h"
#include "defs.h"
#include "lcd_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Zmienne przechowujące pomiar
	double temp = 0.0;
	int32_t temp32 = 0;
	float32_t temp_f = 0.0;

//Buffory do wyświetlania temperatury
	char dane[20];
	char num[12];
	char rx_buffer[20];
	char tempzad[20];

//Instacja filtru PID i uchyb
	arm_pid_instance_f32 pid;
	float32_t error = 0.0;

//Wypełnienie sygnałów PWM rezystora i wentylatora
	int wypelnienie_heater = 0;
	int wypelnienie_cooler = 0;

//Sygnał sterujący i wartość zadana
	float32_t u = 0;
	float32_t x_zad = 25.0;
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if(htim == &htim2)
		{
			BMP280_read(&temp, &temp32);
			temp_f = (float32_t)temp;
			error = x_zad - temp_f;
			u = arm_pid_f32(&pid,error)*10.0;
			if(u>=0){
			wypelnienie_heater = (int)u;
			wypelnienie_cooler = 0;
		};

		if(u<0)
		{
			wypelnienie_heater = 0;
		};

		if(u<-500)
		{
			wypelnienie_cooler = -(int)u;
		};
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wypelnienie_heater);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, wypelnienie_cooler);
		sprintf(num,"%d", temp32);
		sprintf(dane,"%c%c.%c%c\r", num[0], num[1], num[2], num[3]);

		HAL_UART_Transmit(&huart3, dane, strlen(dane), 5);

// Zadawanie temperatury enkoderem ze stopniem 0.1

		static uint16_t pozycja = 0;
		int zmiana = htim1.Instance->CNT - pozycja;
		if(zmiana >= 4 || zmiana <= -4)
		{
			zmiana /= 4;
			if(zmiana == -1) x_zad -= 0.1;
			if(zmiana == 1) x_zad += 0.1;
			if(x_zad > 45.0) x_zad = 45.0;
			if(x_zad < 25.0) x_zad = 25.0;
			pozycja = htim1.Instance->CNT;
			gcvt(x_zad, 4, tempzad);
		}

// Wyswietlanie temperatury na LCD

		LCD_SetCursor(&hlcd1, 0, 0);
		LCD_printf(&hlcd1, "TEMP: %c%c.%c%c", num[0], num[1], num[2], num[3]);
		LCD_SetCursor(&hlcd1, 1, 0);
		gcvt(x_zad, 4, tempzad);
		if(x_zad - (int)x_zad != 0)
			LCD_printf(&hlcd1, "TEMP ZAD: %s0", tempzad);
		else
			LCD_printf(&hlcd1, "TEMP ZAD: %s.00", tempzad);
		}
	}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3)
	{
		char* pend;
		x_zad = strtof(rx_buffer,&pend);
		gcvt(x_zad, 4, tempzad);
		HAL_UART_Receive_IT(&huart3, rx_buffer, 4);
	}
}


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
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//Inicjalizacja czujnika
  int8_t rslt = BMP280_init();

//Wystartowanie TIM2 z okresem próbkowania Ts = 62.5 ms
  uint32_t okres = OKRES_PROBKOWANIA;
  HAL_TIM_Base_Start_IT(&htim2);
  __HAL_TIM_SET_AUTORELOAD(&htim2, okres);

//Wystartowanie kanałów PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wypelnienie_heater);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, wypelnienie_cooler);

//Wystartowanie UART3
  HAL_UART_Receive_IT(&huart3, rx_buffer, 4);

//Inicjalizacja regulatora PID
  pid.Kp = PID_KP;
  pid.Ki = PID_KI;
  pid.Kd = PID_KD;
  arm_pid_init_f32(&pid,1);

//Inicjalizacja LCD
  HAL_TIM_Base_Start_IT(&htim4);
  __HAL_TIM_SET_AUTORELOAD(&htim4, okres);
  LCD_Init(&hlcd1);

//Inicjalizacja enkodera
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  htim1.Instance -> CNT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
