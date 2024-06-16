/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "NRF24L01.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

  	  /* nRF24L01 */

		uint8_t Rxaddr[5] = {'I','T','C','0','1'};
		uint8_t TxData[8] = {0};


  	  /* READ ADC */

		int ADC_Read[4] = {0};

		/* ADC JOYSTICK 1 */

			int Button1 = 0;
			int Joystick_x = 0;
			int Joystick_y = 0;
			int Omega = 0;
			int State_button = 0;
			int Switch = 0;
			long last_time = 0, current_time = 0;;

		/* ADC JOYSTICK 2 */

			int Joystick_Vx = 0;
			int Joystick_Vy = 0;
			int Omega2 = 0;
			int State_button2 = 0;
			int Switch2 = 0;
			long last_time2 = 0, current_time2 = 0;;

		/* BUTTON BEBOUNCE */

			#define BUTTON1_PIN GPIO_PIN_6
			#define BUTTON1_PORT GPIOA
			#define BUTTON2_PIN GPIO_PIN_7
			#define BUTTON2_PORT GPIOA
			#define BUTTON3_PIN GPIO_PIN_2
			#define BUTTON3_PORT GPIOB


			#define DEBOUNCE_DELAY 200

			uint32_t previousMillis = 0;
			uint32_t currentMillis = 0;
			volatile uint32_t count[3] = {0};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* MAP JOYSTICK */

	double map(double x, double in_min, double in_max, double out_min, double out_max) {
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void nRF24l01_Master(){
		  TxData[0] = Joystick_x;
		  TxData[1] = Joystick_y;
		  TxData[2] = Joystick_Vx;
		  TxData[3] = Joystick_Vy;
		  TxData[4] = count[0];
		  TxData[5] = count[1];
		  TxData[6] = count[2];
		  TxData[7] = 9;
			NRF24_Transmit(TxData, 8);//NRF24_Transmit(data, size)
			HAL_Delay(5);
	}


/* MAP JOYSTICK 1 */

	void Joystick() {
		  if (ADC_Read[1] >= 0 && ADC_Read[1] <= 120){
			  Joystick_x = map(ADC_Read[1], 0, 120, 0, 128);
		  }
		  if (ADC_Read[1] >= 132 && ADC_Read[0] <= 255){
			  Joystick_x = map(ADC_Read[1], 132, 255, 128, 255);
		  }
		  if (ADC_Read[1] > 120 && ADC_Read[1] < 132){
			  Joystick_x = 128;
		  }

		  if (ADC_Read[0] >= 0 && ADC_Read[0] <= 120){
			  Joystick_y = map(ADC_Read[0], 0, 120, 0, 128);
		  }
		  if (ADC_Read[0] >= 132 && ADC_Read[0] <= 255){
			  Joystick_y = map(ADC_Read[0], 132, 255, 128, 255);
		  }
		  if (ADC_Read[0] > 120 && ADC_Read[0] < 132){
			  Joystick_y = 128;
		  }

	}

/* MAP JOYSTICK 2 */

	void Joystick_1() {
		 if (ADC_Read[2] >= 0 && ADC_Read[2] <= 120){
				Joystick_Vx = map(ADC_Read[2], 0, 120, 0, 128);
		 }
		 if (ADC_Read[2] >= 135 && ADC_Read[2] <= 255){
				Joystick_Vx = map(ADC_Read[2], 135, 255, 128, 255);
		 }
		 if (ADC_Read[2] > 120 && ADC_Read[2] < 135){
				Joystick_Vx = 128;
		 }
		 if (ADC_Read[3] >= 0 && ADC_Read[3] <= 120){
				Joystick_Vy = map(ADC_Read[3], 0, 120, 0, 128);
		 }
		 if (ADC_Read[3] >= 130 && ADC_Read[3] <= 255){
				Joystick_Vy = map(ADC_Read[3], 130, 255, 128, 255);
		 }
		 if (ADC_Read[3] > 120 && ADC_Read[3] < 130){
				Joystick_Vy = 128;
		 }

	}


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  /* nRF24L01 */

	  NRF24_Init();
	  NRF24_TxMode(Rxaddr, 112, 8);
	  HAL_ADC_Start_DMA(&hadc1, &ADC_Read, 4);
	  HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* JOYSTICK */

		  Joystick();
		  Joystick_1();

	  /* DATA MASTER */

	      nRF24l01_Master();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

	/* BUTTON BEBOUNCE */

		void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		  currentMillis = HAL_GetTick();

		  if (GPIO_Pin == BUTTON1_PIN && (currentMillis - previousMillis > DEBOUNCE_DELAY)) {
			count[0]++;
			if (count[0] > 3){
				count[0] = 0;
			}
			previousMillis = currentMillis;
		  }

			if (GPIO_Pin == BUTTON2_PIN
					&& (currentMillis - previousMillis > DEBOUNCE_DELAY)) {
				count[1]++;
				if (count[1] > 3) {
					count[1] = 0;
				}
				previousMillis = currentMillis;
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
  __disable_irq();
  while (1)
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
