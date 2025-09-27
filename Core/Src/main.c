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
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "i2c_lcd.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define PH_IDEAL 7.4f//valor ideal de ph
#define SENSIBILIDAD_BAJAR_PH 1.0f //ph/ml
#define SENSIBILIDAD_SUBIR_PH 3.6363f//ph/ml
#define VOLUMEN_PILETA 5000.0f// ml
#define TC 5.0f  //segundos
#define KP 1.7f
#define DUTY_MIN 0.275f
#define DUTY_MAX 1.0f
#define CAUDAL_MAXIMO 1.33f


//funcion para calcular el error de ph
float calcularErrorPH(float ph_actual){
	return PH_IDEAL - ph_actual;
}

//determino sensibilidad segun error
float obtenerSensibilidad(float error){
	return (error < 0.0f) ? SENSIBILIDAD_BAJAR_PH : SENSIBILIDAD_SUBIR_PH;
}
 //calculo volumen
float calcularVolumen (float error, float sensibilidad){
	return error/sensibilidad;
}

//Calculo caudal
float calcularCaudal(float volumen){
	return volumen/TC;
}

//calculo duty final limitado

float calcularDutyFinal(float caudal){
	float duty_crudo = caudal / CAUDAL_MAXIMO;
	float duty_con_ganancia = duty_crudo * KP;

	if (duty_con_ganancia > DUTY_MAX) return DUTY_MAX;
	if (duty_con_ganancia < DUTY_MIN) return DUTY_MIN;
	return duty_con_ganancia;
}

void setBombaPWM(float duty, int subir) {
    uint32_t pulse = (uint32_t)(duty * htim2.Init.Period);
    if (subir) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // Bomba subir pH
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);      // Apaga bomba bajar
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);      // Apaga bomba subir
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);  // Bomba bajar pH
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

//A0 ES LA BOMBA PARA SUBIR PH Y A1 PARA BAJAR
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();

  //arrancar los canales PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Bomba subir pH
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Bomba bajar pH

  float ph_actual = 0.5f;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  float error = calcularErrorPH(ph_actual);

	  if (fabs(error) == 0.0f){
		  lcd_clear();
		  lcd_put_cur(0,0);
		  lcd_send_string("pH ideal");
		  setBombaPWM(0, 1);
		  HAL_Delay(2000); // espera 2 segundos

		   lcd_clear();
		   lcd_put_cur(0,0);
		   lcd_send_string("Bombas off");
		   HAL_Delay(2000);

	  } else {

	  float sensibilidad = obtenerSensibilidad(error);
	  float volumen = calcularVolumen(error, sensibilidad);
	  float caudal = calcularCaudal(volumen);
	  float duty = calcularDutyFinal(fabs(caudal));

	    lcd_clear();
	    lcd_put_cur(0,0);
	    char ph_line[17];
	    if(error < 0.0f){
	    snprintf(ph_line, sizeof(ph_line), "Bajar: %.2f pH", fabs(error));
	    setBombaPWM(duty, 0);
	  } else{
	     snprintf(ph_line, sizeof(ph_line), "Subir: %.2f pH", fabs(error));
	     setBombaPWM(duty,1);
	  }

	    lcd_send_string(ph_line);       // ✅ mostrar el texto
	    HAL_Delay(2000);

	    HAL_Delay(2000);

	 // Secuencia: Duty de la bomba
	     lcd_clear();
	     lcd_put_cur(0,0);
	     char duty_line[17];
	     snprintf(duty_line, sizeof(duty_line), "Duty: %.2f", duty);
	     lcd_send_string(duty_line);
	      HAL_Delay(2000);

	  }

  }
}


  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
