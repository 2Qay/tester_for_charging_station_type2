/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
//#include "math.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_status_A(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}
void set_status_B(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}
void set_status_C(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}
void set_status_D(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}
void set_status_E(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

uint8_t str_tx[100];
void hello_world(void)
{
	  set_status_A();
	  lcd_initialization();
	  HAL_Delay(200);

	  lcd_set_cursor(0, 0);
	  sprintf(str_tx, "Лямбда электроника");
	  lcd_draw_text(str_tx);
	  lcd_set_cursor(0, 1);
	  sprintf(str_tx, "при поддержке");
	  lcd_draw_text(str_tx);
	  lcd_set_cursor(0, 2);
	  sprintf(str_tx, "Фонда содействия");
	  lcd_draw_text(str_tx);
	  lcd_set_cursor(0, 3);
	  sprintf(str_tx, "инновациям");
	  lcd_draw_text(str_tx);
	  HAL_Delay(1000);
}

uint32_t PP_adc;
float PP_value;
uint32_t cable_current;
void read_PP(void)
{
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3,100);
		PP_adc = (uint32_t) HAL_ADC_GetValue(&hadc3);
		HAL_ADC_Stop(&hadc3);
		PP_value = (float)PP_adc*3.3/4096.0;
		/*
		 * 50 - 150 om = 70A
		 * 150 - 330 om = 32A
		 * 330 - 1000 om = 20A
		 * 1000 - 2700 om = 13A
		 */

		if(     PP_value > 0.07 && PP_value < 0.21){
			cable_current = 70;}
		else if(PP_value > 0.21 && PP_value < 0.43){
			cable_current = 32;}
		else if(PP_value > 0.43 && PP_value < 1.0){
			cable_current = 20;}
		else if(PP_value > 1.0 && PP_value < 1.81){
			cable_current = 13;}
		else if(PP_value <= 0.07 || PP_value >= 1.81){
			cable_current = 0;}
}

const uint32_t CP_adc_arr = 500;
uint32_t CP_adc[500];
float CP_val_MAX = 0;

void read_CP(void)
{
 	 HAL_ADC_ConfigChannel(&hadc2, ADC_CHANNEL_3);
 	 for(uint32_t i = 0; i < CP_adc_arr; i++)
 	 {
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2,100);
		CP_adc[i] = (uint32_t) HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);
 	 }
 	 HAL_ADC_ConfigChannel(&hadc1, ADC_CHANNEL_4);
 	 CP_val_MAX = 0;
 	 for(uint32_t i = 0; i < CP_adc_arr; i++)
 	 {
 		 if(CP_adc[i] > CP_val_MAX)
 		 {
 			CP_val_MAX = (float)CP_adc[i];
 		 }
 	 }
 	CP_val_MAX = CP_val_MAX*0.004914 + 0.15;
}

const uint32_t adc_arr = 2000;
uint32_t adc_a[2000];
uint32_t adc_b[2000];
uint32_t adc_c[2000];
uint32_t adc_max_a;
uint32_t adc_max_b;
uint32_t adc_max_c;
float adc_result_a;
float adc_result_b;
float adc_result_c;

void read_phase_A(void)
{
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);   // main channel
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);


	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_ConfigChannel(&hadc1, ADC_CHANNEL_4);
	for(uint32_t i = 0; i < adc_arr; i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_a[i] = (uint32_t) HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	adc_max_a = 0;
	for(uint32_t i = 0; i < adc_arr; i++)
	{
		 if(adc_max_a < adc_a[i])
		 {
			 adc_max_a = adc_a[i];
		 }
	}
	adc_result_a = ((float)adc_max_a*3.3/4096.0-1.22662)*1358.88/sqrt(2);
	if(adc_result_a < 12.0)
	{
		adc_result_a = 0.0;
	}
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // main channel
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}

void read_phase_B(void)
{

	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);   // main channel
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);


	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	//HAL_ADC_ConfigChannel(&hadc1, ADC_CHANNEL_5);
	for(uint32_t i = 0; i < adc_arr; i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_b[i] = (uint32_t) HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	adc_max_b = 0;
	for(uint32_t i = 0; i < adc_arr; i++)
	{
		 if((float)adc_b[i] > adc_max_b)
		 {
			 adc_max_b = adc_b[i];
		 }
	}
	adc_result_b = ((float)adc_max_b*3.3/4096.0-1.22662)*1358.88/sqrt(2);
	if(adc_result_b < 12.0)
	{
		adc_result_b = 0.0;
	}
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // main channel
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}

void read_phase_C(void)
{
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);   // main channel
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}


	//HAL_ADC_ConfigChannel(&hadc1, ADC_CHANNEL_6);
	for(uint32_t i = 0; i < adc_arr; i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adc_c[i] = (uint32_t) HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	adc_max_c = 0;
	for(uint32_t i = 0; i < adc_arr; i++)
	{
		 if((float)adc_c[i] > adc_max_c)
		 {
			 adc_max_c = adc_c[i];
		 }
	}
	adc_result_c = ((float)adc_max_c*3.3/4096.0-1.22662)*1358.88/sqrt(2);
	if(adc_result_c < 12.0)
	{
		adc_result_c = 0.0;
	}
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // main channel
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
}



uint32_t ICValue;
float Duty;
float Frequency;
uint32_t CP_Voltage = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // If the interrupt is triggered by channel 1
	{
		// Read the IC value
		ICValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (ICValue != 0)
		{
			// calculate the Duty Cycle
			Duty = 100.0 - ((float)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *100.0)/(float)ICValue;
			Frequency = 84000000.0/(float)ICValue;
		}
	}
}


uint8_t menu = 0;
uint8_t menu_hor = 0;
uint8_t menu_ver = 0;
char status = 'A';

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if(GPIO_Pin == GPIO_PIN_2)
   {

   }
//	gpio_1		gpio_7
//        gpio_4
//  gpio_3		gpio_6
//   	  gpio_5
//
   //__BUTTONS_____________________________________________
   if(GPIO_Pin == GPIO_PIN_1)
   {
//	   set_status_A();
//	   menu = 1;
//	   Frequency = 0;
//	   Duty = 0;
   }
   if(GPIO_Pin == GPIO_PIN_3)
   {
	  switch (menu_hor) {
		case 0:
			menu_hor = 1;
			break;
		case 1:
			menu_hor = 2;
			break;
		case 2:
			menu_hor = 0;
			break;
		default:
			break;
	  }
   }

   if(GPIO_Pin == GPIO_PIN_6)
   {
	  switch (menu_hor) {
		case 0:
			menu_hor = 2;
			break;
		case 1:
			menu_hor = 0;
			break;
		case 2:
			menu_hor = 1;
			break;
		default:
			break;
	  }
   }

   if(GPIO_Pin == GPIO_PIN_5)
   {
	  switch (menu_ver) {
		case 0:
			menu_ver = 1;
			break;
		case 1:
			menu_ver = 2;
			break;
		case 2:
			menu_ver = 3;
			break;
		case 3:
			menu_ver = 4;
			break;
		case 4:
			menu_ver = 0;
			break;
		default:
			break;
	  }
   }

   if(GPIO_Pin == GPIO_PIN_4)
   {
	  switch (menu_ver) {
		case 0:
			menu_ver = 4;
			break;
		case 1:
			menu_ver = 0;
			break;
		case 2:
			menu_ver = 1;
			break;
		case 3:
			menu_ver = 2;
			break;
		case 4:
			menu_ver = 3;
			break;
		default:
			break;
	  }
   }
   switch (menu_ver) {
  		case 0:
  			status = 'A';
  			set_status_A();
  			break;
  		case 1:
  			status = 'B';
  			set_status_B();
  			break;
  		case 2:
  			status = 'C';
  			set_status_C();
  			break;
  		case 3:
  			status = 'D';
  			set_status_D();
  			break;
  		case 4:
  			status = 'E';
  			set_status_E();
  			break;
  		default:
  			break;
  	  }
   if(GPIO_Pin == GPIO_PIN_7)
   {
//	   HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);   // main channel
//	   HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);
//	   menu = 7;
   }

   //______________________________________________________
}

uint32_t station_current;
void duty_read()
{
	if(     Duty < 3)
		station_current = 0;
	else if(Duty >=3 && Duty < 7)
		station_current = 999;
	else if(Duty >=7 && Duty < 8)
		station_current = 0;
	else if(Duty >=8 && Duty < 10)
		station_current = 6;
	else if(Duty >=10 && Duty < 85)
		station_current = Duty * 0.6;
	else if(Duty >=85 && Duty < 96)
		station_current = (Duty-64)*2.5;
	else if(Duty >=96 && Duty < 97)
		station_current = 80;
	else if(Duty >=97 && Duty < 100)
		station_current = 0;
}

void lcd_print_info()
{
	if(menu_hor == 0)
	{
		Frequency = 0;
		Duty = 0;
		read_PP();
		read_CP();
		read_phase_A();
		read_phase_B();
		read_phase_C();
		duty_read();
		lcd_clear_display();
		lcd_set_cursor(0, 0);
		sprintf(str_tx, "статус = %c ", status);
		lcd_draw_text(str_tx);
		lcd_set_cursor(0, 1);
		if(cable_current != 0)
			sprintf(str_tx, "мощ.каб. = %u А   ", cable_current);
		else
			sprintf(str_tx, "мощ.каб. = ошибка", cable_current);
		lcd_draw_text(str_tx);
		lcd_set_cursor(0, 2);
		if(station_current == 0 && cable_current == 0)
			sprintf(str_tx, "ошибка станции     ", Duty);
		else if(station_current == 999)
			sprintf(str_tx, "ст. быстр. зар.!   ");
		else
			sprintf(str_tx, "мощ.ст. = %u А", station_current);
		lcd_draw_text(str_tx);
		lcd_set_cursor(0, 3);
		if((adc_result_a > adc_result_b - 44) && (adc_result_a < adc_result_b + 44) && (adc_result_a > adc_result_c - 44) && (adc_result_a < adc_result_c + 44) && (adc_result_b > adc_result_c - 44) && (adc_result_b < adc_result_c + 44))
			sprintf(str_tx, "напр. фаз = ОК    ");
		else
			sprintf(str_tx, "напр. фаз = ОШИБКА");
		lcd_draw_text(str_tx);
	}

	if(menu_hor == 1)
	{
		 Frequency = 0;
		 Duty = 0;
		 read_CP();
	  	 lcd_clear_display();
		 lcd_set_cursor(0, 0);
		 sprintf(str_tx, "статус = %c ", status);
		 lcd_draw_text(str_tx);
		 lcd_set_cursor(0, 1);
		 sprintf(str_tx, "Ст. = %.2f Гц  ", Frequency);
		 lcd_draw_text(str_tx);
		 lcd_set_cursor(0, 2);
		 sprintf(str_tx, "Св. = %.2f проц.   ", Duty);
		 lcd_draw_text(str_tx);
		 lcd_set_cursor(0, 3);
		 sprintf(str_tx, "Ам. = %.2f В   ", CP_val_MAX);
		 lcd_draw_text(str_tx);
	}
	if(menu_hor == 2)
	{
		 read_phase_A();
		 read_phase_B();
		 read_phase_C();
		 HAL_Delay(100);
	  	 lcd_clear_display();
		 lcd_set_cursor(0, 0);
		 sprintf(str_tx, "статус = %c", status);
		 lcd_draw_text(str_tx);
		 lcd_set_cursor(0, 1);
		 //*(3.3/4096.0-1.22662)*1358.88
		 sprintf(str_tx, "фаза A = %.1f В   ", adc_result_a);
		 lcd_draw_text(str_tx);
		 lcd_set_cursor(0, 2);
		 sprintf(str_tx, "фаза B = %.1f В   ", adc_result_b);
		 lcd_draw_text(str_tx);
		 lcd_set_cursor(0, 3);
		 sprintf(str_tx, "фаза C = %.1f В   ", adc_result_c);
		 lcd_draw_text(str_tx);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // main channel
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

  //РёРЅРёС†РёР°Р»РёР·Р°С†РёСЏ СЌРєСЂР°РЅР°

//  uint32_t pretick = 0;
//  uint32_t posttick = 0;
  hello_world();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_print_info();
	  HAL_Delay(200);
//	  uint32_t ICValue;
//	  float Duty;
//	  float Frequency;



//		 lcd_draw_text(str_tx);
	    sprintf(str_tx, "_____________START____________________ \r\n");
	    CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));

		for(uint16_t i = 0; i < adc_arr; i++)
		{
			sprintf(str_tx, "%.1f \t %.1f \t %.1f  \r\n", ((float)adc_a[i]*3.3/4096.0-1.22662)*1358.88, ((float)adc_b[i]*3.3/4096.0-1.22662)*1358.88, ((float)adc_c[i]*3.3/4096.0-1.22662)*1358.88);
			CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
		}
		sprintf(str_tx, " \r\n");
		CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));

//		sprintf(str_tx, "%.1f \r\n", ((float)adc_max_a*3.3/4096.0-1.22662)*1358.88);
//		CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
//		sprintf(str_tx, "_____________STOP______________________\r\n\r\n");
//		CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
//		HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//	  HAL_Delay(2000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_B2_Pin|LCD_B3_Pin|LCD_B4_Pin|LCD_B5_Pin
                          |LCD_B6_Pin|LCD_RST_Pin|LCD_PCB_Pin|LCD_RS_Pin
                          |HUI_Pin|LCD_B0_Pin|LCD_B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_B7_GPIO_Port, LCD_B7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CP_ERR_Pin|CP_D_Pin|CP_C_Pin|CP_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PP_13A_Pin|PP_20A_Pin|PP_32A_Pin|PP_70A_Pin
                          |PP_ERR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_A0_Pin|LCD_RW_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_B2_Pin LCD_B3_Pin LCD_B4_Pin LCD_B5_Pin
                           LCD_B6_Pin LCD_RST_Pin LCD_PCB_Pin LCD_RS_Pin
                           LCD_B0_Pin LCD_B1_Pin */
  GPIO_InitStruct.Pin = LCD_B2_Pin|LCD_B3_Pin|LCD_B4_Pin|LCD_B5_Pin
                          |LCD_B6_Pin|LCD_RST_Pin|LCD_PCB_Pin|LCD_RS_Pin
                          |LCD_B0_Pin|LCD_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_B7_Pin */
  GPIO_InitStruct.Pin = LCD_B7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_B7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HUI_Pin */
  GPIO_InitStruct.Pin = HUI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(HUI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CP_ERR_Pin CP_D_Pin CP_C_Pin CP_B_Pin */
  GPIO_InitStruct.Pin = CP_ERR_Pin|CP_D_Pin|CP_C_Pin|CP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PP_13A_Pin PP_20A_Pin PP_32A_Pin PP_70A_Pin
                           PP_ERR_Pin */
  GPIO_InitStruct.Pin = PP_13A_Pin|PP_20A_Pin|PP_32A_Pin|PP_70A_Pin
                          |PP_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT2_EXTI1_Pin BUT1_EXTI3_Pin BUT3_EXTI4_Pin BUT4_EXTI5_Pin
                           BUT6_EXTI6_Pin BUT5_EXTI7_Pin */
  GPIO_InitStruct.Pin = BUT2_EXTI1_Pin|BUT1_EXTI3_Pin|BUT3_EXTI4_Pin|BUT4_EXTI5_Pin
                          |BUT6_EXTI6_Pin|BUT5_EXTI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_A0_Pin LCD_RW_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_A0_Pin|LCD_RW_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
