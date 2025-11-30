/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // flag

uint32_t r = 0;
uint32_t g = 0;
uint32_t b = 0;
//uint32_t clear = 0 ;
uint32_t mau = -1;
int color[4][6] = {0};
int goc = -1;
//uint32_t n_r = 0;
//uint32_t n_g = 0;
//uint32_t n_b = 0;


void Servo_Init(){
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_2);
}

void initColor(){
	
////////////////////////mau do
  color[0][0] =  90; //Min R
  color[0][1] =  115; //Max R

  color[0][2] =  150;//Min G
  color[0][3] =  200; //Max G

  color[0][4] =   105;//Min B
  color[0][5] =   140;//Max B
////////////////////////////////// mau xanh
  color[1][0] =  160; //Min R
  color[1][1] =  240; //Max R

  color[1][2] =  100;//Min G
  color[1][3] =  150; //Max G

  color[1][4] =   55;//Min B
  color[1][5] =   90;//Max B
//////////////////////////////////// xanh la
  color[2][0] =  150; //Min R
  color[2][1] =  180; //Max R

  color[2][2] =  115;//Min G
  color[2][3] =  140; //Max G

  color[2][4] =   115;//Min B
  color[2][5] =   140;//Max B
/////////////////////////////////////// mau vang
	color[3][0] =  55; //Min R
  color[3][1] =  90; //Max R

  color[3][2] =  70;//Min G
  color[3][3] =  100; //Max G

  color[3][4] =   95;//Min B
  color[3][5] =   120;//Max B
}

void Servo1_Write(uint8_t goc){
	 // 0 -> 180 = 1000 -> 2000
		uint16_t _ccr1 = (1500 * goc) / 180 + 800;
		htim1.Instance->CCR1 = _ccr1;
}
void Servo2_Write(uint8_t goc){
	 // 0 -> 180 = 1000 -> 2000
		uint16_t _ccr2 = (1500 * goc) / 180 + 800;
		htim1.Instance->CCR2 = _ccr2;
}

void Sensor_Init(){
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin( s0_GPIO_Port, s0_Pin, 1); // chon chia tan so 20%
	HAL_GPIO_WritePin( s1_GPIO_Port, s1_Pin, 0); //0
}
	


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if(Is_First_Captured == 0)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
        }
        else
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            if(IC_Val2 > IC_Val1)
                Difference = IC_Val2 - IC_Val1;
            else
                Difference = (0xFFFF - IC_Val1 + IC_Val2);

            Is_First_Captured = 0;
        }
    }
}
uint32_t getFrequency(void)
{
    Is_First_Captured = 0;
    Difference = 0;

    HAL_Delay(10);  // doi xung on dinh

    return Difference;  // gi� tr? chu k?, c�ng nh? = t?n s? c�ng cao
}

int readColor()
{
    //////////////////////
    //  READ RED (S2=0,S3=0)
    //////////////////////
		int i = 30;
		r = 0; g = 0; b = 0;
		while(i--){
			HAL_GPIO_WritePin(s2_GPIO_Port, s2_Pin, 0);
			HAL_GPIO_WritePin(s3_GPIO_Port, s3_Pin, 0);
			HAL_Delay(30);
			r += getFrequency();
    //////////////////////
    //  READ GREEN (S2=1,S3=1)
    //////////////////////
			HAL_GPIO_WritePin(s2_GPIO_Port, s2_Pin, 1);
			HAL_GPIO_WritePin(s3_GPIO_Port, s3_Pin, 1);
			HAL_Delay(30);
			g += getFrequency();
    //////////////////////
    //  READ BLUE (S2=0,S3=1)
    //////////////////////
			HAL_GPIO_WritePin(s2_GPIO_Port, s2_Pin, 0);
			HAL_GPIO_WritePin(s3_GPIO_Port, s3_Pin, 1);
			HAL_Delay(30);
			b += getFrequency();	
		}

		r /= 30; g /= 30; b /= 30;
		
		for (int i = 0; i < 4; i++){
			if(r >= color[i][0] && r <= color[i][1] && g >= color[i][2] && g <= color[i][3] && b >= color[i][4] && b <= color[i][5])
				return i;
		}
		return -1;
}

//void normalizeColor(void)
//{
//    if (clear == 0) clear = 1;  // tr�nh chia 0

//    n_r = (r * 100) / clear;
//    n_g = (g * 100) / clear;
//    n_b = (b * 100) / clear;
//}
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	Sensor_Init();
	Servo_Init();
	initColor();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		Servo1_Write(170);
//		HAL_Delay(1000);
		Servo1_Write(85);
	//	HAL_Delay(1000);
		mau = readColor();
		HAL_Delay(500);
		
//		Servo1_Write(15);
//		HAL_Delay(1000);
//		
		
	//	HAL_Delay(200);
		switch(mau){
			case 0:
				Servo2_Write(0);
				HAL_Delay(200);
				Servo1_Write(15);
				HAL_Delay(1000);
				break;
			case 1:
				Servo2_Write(36);
				HAL_Delay(200);
				Servo1_Write(15);
				HAL_Delay(1000);
				break;
			case 2:
				Servo2_Write(72);
				HAL_Delay(200);
				Servo1_Write(15);
				HAL_Delay(1000);
				break;
			case 3:
				Servo2_Write(105);
				HAL_Delay(200);
				Servo1_Write(15);
				HAL_Delay(1000);
			default:
				Servo2_Write(144);
				HAL_Delay(200);
				Servo1_Write(15);
				HAL_Delay(1000);
				break;
		}
		Servo1_Write(170);
		HAL_Delay(1000);


//		for (int i = 0; i <= 144; i += 36){
//			Servo2_Write(i);
//			HAL_Delay(500);
//		}

			
//		for (int i = 0; i <= 180; i++){
//			Servo2_Write(i);
//			HAL_Delay(20);
//		}
//	
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, s0_Pin|s1_Pin|s2_Pin|s3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : s0_Pin s1_Pin s2_Pin s3_Pin */
  GPIO_InitStruct.Pin = s0_Pin|s1_Pin|s2_Pin|s3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
