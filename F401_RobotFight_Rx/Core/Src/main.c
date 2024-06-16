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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "NRF24L01.h"
	#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

	uint8_t Rxaddr[5] = {'I','T','C','0','1'};
	uint8_t RxData[8];
	uint32_t lastTime = 0;

	float M1, M2, V;
	float Vx, Vy, VX, VY, Omega;
	uint8_t speed1;
	uint8_t speed2;
	int speed;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



double map(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void reset_data(void) {
	RxData[0] = 128; //x1
	RxData[1] = 128; //y1
	RxData[2] = 128; //x2
	RxData[3] = 0; //y2
	RxData[4] = 128; //y2
	RxData[5] = 128; //y2
	RxData[6] = 0; //y2
	RxData[7] = 0; //y2
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

		NRF24_Init();
		NRF24_RxMode(Rxaddr, 112, 8); //NRF24_TxMode(Address, channel, payload_size)
		RxData[0] = 128; //x1
		RxData[1] = 128; //y1
		RxData[2] = 128; //x2
		RxData[3] = 0; //y2
		RxData[4] = 128; //y2
		RxData[5] = 128; //y2
		RxData[6] = 0; //y2
		RxData[7] = 0; //y2

	/* TIMER WHEEL */

		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	/* TIMER SHORTER */

		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	/* TIMER BLDC */

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (DataReady(1) > 0) {
			NRF24_Receive(RxData, 8);
			lastTime = HAL_GetTick();
		}
		if ((HAL_GetTick() - lastTime > 10)) {
			reset_data();
		}

		/* MAPPING */

			Vx 	  = map(RxData[0], 0, 256, -999, 999);
			Omega = map(RxData[1], 0, 256, 999, -999);
			VX    = map(RxData[3], 0, 256, 1000, 2000);
			VY    = map(RxData[2], 0, 256, -999, 999);

		/* SPEED MOTOR */

			M1 = (Vx + Omega);
			M2 = (Vx - Omega);

			if (M1 > 999){
				M1 = 999;
			}
			if (M1 < -999){
				M1 = -999;
			}
			if (M1 < 999 && M1 > -999){
				M1 = M1;
			}

			if (M2 > 999){
				M2 = 999;
			}
			if (M2 < -999){
				M2 = -999;
			}
			if (M2 < 999 && M2 > -999){
				M2 = M2;
			}

		/* MOTOR1 */

			if (M1 > 0 && M1 <= 999){
				TIM2->CCR1 = M1;
				TIM2->CCR2 = 0;
			}
			if (M1 == 0){
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
			}
			if (M1 < 0 && M1 >= -999){
				TIM2->CCR1 = 0;
				TIM2->CCR2 = (-1) *M1;
			}

		/* MOTOR2 */

			if (M2 > 0 && M2 <= 999){
				TIM2->CCR3 = M2;
				TIM2->CCR4 = 0;
			}
			if (M2 == 0){
				TIM2->CCR3 = 0;
				TIM2->CCR4 = 0;
			}
			if (M2 < 0 && M2 >= -999){
				TIM2->CCR3 = 0;
				TIM2->CCR4 = (-1) *M2;
			}

		/* SHORTER */

			speed1 = RxData[4];
			speed2 = RxData[5];
			speed = speed1 - speed2;
			if (speed > 0 )
			{
				TIM3->CCR1 = speed*300;
				TIM3->CCR2 = 0;
			}
			else if (speed < 0)
			{

				TIM3->CCR1 = 0;
				TIM3->CCR2 = -1*speed*300;
			}
			else if (speed == 0)
			{

				TIM3->CCR1 = 0;
				TIM3->CCR2 = 0;
			}

		/* BLDC */

			TIM1->CCR1 = VX;



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
	while (1) {
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
