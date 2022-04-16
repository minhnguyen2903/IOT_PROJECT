/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266.h"
#include "LCD1602.h"
#include <string.h>
#include <stdlib.h>
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

#define MAX_BUFFER_SIZE 50

char Rx_data[MAX_BUFFER_SIZE];
int waitForDataBack = 0;
int timeout = 0;
int requestId = 0;
int request_ok = 0;
int request_fail = 0;
int request_reconnect = 0;
int adcValue = 0;
uint16_t adc[2];

//void Send_String(char *data)
//{
//  //		memset(Rx_data, 0, MAX_BUFFER_SIZE);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, strlen(data));
//  HAL_Delay(50);
//  waitForDataBack = 1;
//  timeout = 0;
//}

//void Send_Request(char *data)
//{
//  memset(Rx_data, 0, MAX_BUFFER_SIZE);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, strlen(data));
//  HAL_Delay(50);
//  waitForDataBack = 1;
//  timeout = 0;
//}

//int Wait_For(char *nstr)
//{
//  HAL_Delay(1);
//  timeout++;
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
//  int len = strlen(nstr);
//  int so_far = 0;
//  int i = 0;
//  while (i < MAX_BUFFER_SIZE)
//  {
//    int temp = i;
//    if (Rx_data[i] == nstr[so_far])
//    {
//      while (so_far < len)
//      {
//        if (Rx_data[temp % MAX_BUFFER_SIZE] == nstr[so_far])
//        {
//          temp++;
//          so_far++;
//          if (so_far == len)
//          {
//            timeout = 0;
//            return i;
//          }
//        }
//        else
//        {
//          so_far = 0;
//          break;
//        }
//      }
//    }
//    i++;
//  }
//  return 0;
//}

//void ESP_init()
//{
//espResetInit:
//  lcd_clear();
//  lcd_send_string("RESETING");

//  Send_String("+++");
//  HAL_Delay(200);
//  Send_String("+++AT+RST\r\n");
//  for (int i = 0; i < 5; i++)
//  {
//    HAL_Delay(2000);
//    lcd_send_string(".");
//  }

//  Send_String("AT+CWMODE=1\r\n"); // thiet lap che do lam viec station
//  while (!Wait_For("AT+CWMODE=1\r\r\n\r\nOK"))
//  {
//    if (timeout > 5000)
//    {
//      timeout = 0;
//      goto espResetInit;
//    }
//  }
//  lcd_clear();
//  lcd_send_string("CONNECTING");
//  lcd_put_cur(1, 0);
//  lcd_send_string("TO WIFI");
//  HAL_Delay(100);
//  Send_String("AT+CIPMUX=0\r\n"); // thiet lap che do truyen don kenh
//  while (!Wait_For("AT+CIPMUX=0\r\r\n\r\nOK"))
//  {
//    if (timeout > 5000)
//    {
//      timeout = 0;
//      goto espResetInit;
//    }
//  };
//  HAL_Delay(100);
//  Send_String("AT+CWJAP=\"hehe\",\"00000000\"\r\n"); // ket noi wifi
//  while (!Wait_For("WIFI GOT IP\r\n\r\nOK"))
//  {
//    if (timeout > 15000)
//    {
//      timeout = 0;
//      goto espResetInit;
//    }
//  };
//  lcd_clear();
//  lcd_put_cur(0, 0);
//  lcd_send_string("WIFI CONNECTED");
//  HAL_Delay(1000);
//  Send_String("AT+CIPSTART=\"TCP\",\"192.168.48.102\",8163\r\n"); // ket noi server
//  while (!Wait_For("CONNECT\r\n\r\nOK"))
//  {
//    if (timeout > 5000)
//    {
//      timeout = 0;
//      goto espResetInit;
//    }
//  };
//  lcd_clear();
//  lcd_send_string("SERVER CONNECTED");
//  Send_String("AT+CIPMODE=1\r\n"); // thiet lap che do lam viec station
//  while (!Wait_For("AT+CIPMODE=1\r\r\n\r\nOK"))
//  {
//    if (timeout > 5000)
//    {
//      timeout = 0;
//      goto espResetInit;
//    }
//  }
//  Send_String("AT+CIPSEND\r\n"); // thiet lap che do lam viec station
//  HAL_Delay(200);
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, (uint8_t *)Rx_data, MAX_BUFFER_SIZE);
  }
}

void getADC(ADC_HandleTypeDef *hadc, int *adcValue)
{
  HAL_ADC_Start(hadc);
  HAL_Delay(50);
  *adcValue = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
}

#define MAX_LED 23
#define USE_BRIGHTNESS 0

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4]; // for brightness

int datasentflag = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
  datasentflag = 1;
}

void Set_LED(int LEDnum, int Red, int Green, int Blue)
{
  LED_Data[LEDnum][0] = LEDnum;
  LED_Data[LEDnum][1] = Green;
  LED_Data[LEDnum][2] = Red;
  LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness(int brightness) // 0-45
{
#if USE_BRIGHTNESS

  if (brightness > 45)
    brightness = 45;
  for (int i = 0; i < MAX_LED; i++)
  {
    LED_Mod[i][0] = LED_Data[i][0];
    for (int j = 1; j < 4; j++)
    {
      float angle = 90 - brightness; // in degrees
      angle = angle * PI / 180;      // in rad
      LED_Mod[i][j] = (LED_Data[i][j]) / (tan(angle));
    }
  }

#endif
}

uint16_t pwmData[(24 * MAX_LED) + 50];

void WS2812_Send(void)
{
  uint32_t indx = 0;
  uint32_t color;

  for (int i = 0; i < MAX_LED; i++)
  {
#if USE_BRIGHTNESS
    color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));
#else
    color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8) | (LED_Data[i][3]));
#endif
    for (int i = 23; i >= 0; i--)
    {
      if (color & (1 << i))
      {
        pwmData[indx] = 60; // 2/3 of 90
      }

      else
        pwmData[indx] = 30; // 1/3 of 90

      indx++;
    }
  }

  for (int i = 0; i < 50; i++)
  {
    pwmData[indx] = 0;
    indx++;
  }

  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
}

void Reset_LED(void)
{
  for (int i = 0; i < MAX_LED; i++)
  {
    LED_Data[i][0] = i;
    LED_Data[i][1] = 0;
    LED_Data[i][2] = 0;
    LED_Data[i][3] = 0;
  }
}

// ported from the arduino code for 8 LEDs located at ->>>>  https://adrianotiger.github.io/Neopixel-Effect-Generator/

uint16_t effStep = 0;

uint8_t rainbow_effect_left()
{
  // Strip ID: 0 - Effect: Rainbow - LEDS: 8
  // Steps: 13 - Delay: 54
  // Colors: 3 (255.0.0, 0.255.0, 0.0.255)
  // Options: rainbowlen=8, toLeft=true,
  //  if(millis() - strip_0.effStart < 54 * (strip_0.effStep)) return 0x00;

  float factor1, factor2;
  uint16_t ind;
  for (uint16_t j = 0; j < 23; j++)
  {
    ind = effStep + j * 1.625;
    switch ((int)((ind % 13) / 4.333333333333333))
    {
    case 0:
      factor1 = 1.0 - ((float)(ind % 13 - 0 * 4.333333333333333) / 4.333333333333333);
      factor2 = (float)((int)(ind - 0) % 13) / 4.333333333333333;
      /************ chnaged here *********/
      Set_LED(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
      WS2812_Send();
      break;
    case 1:
      factor1 = 1.0 - ((float)(ind % 13 - 1 * 4.333333333333333) / 4.333333333333333);
      factor2 = (float)((int)(ind - 4.333333333333333) % 13) / 4.333333333333333;
      Set_LED(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
      WS2812_Send();
      break;
    case 2:
      factor1 = 1.0 - ((float)(ind % 13 - 2 * 4.333333333333333) / 4.333333333333333);
      factor2 = (float)((int)(ind - 8.666666666666666) % 13) / 4.333333333333333;
      Set_LED(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
      WS2812_Send();
      break;
    }
  }
  if (effStep >= 13)
  {
    effStep = 0;
    return 0x03;
  }
  else
    effStep++;
  return 0x01;
}

uint8_t rainbow_effect_right()
{
  // Strip ID: 0 - Effect: Rainbow - LEDS: 8
  // Steps: 14 - Delay: 30
  // Colors: 3 (255.0.0, 0.255.0, 0.0.255)
  // Options: rainbowlen=8, toLeft=false,
  //  if(millis() - strip_0.effStart < 30 * (strip_0.effStep)) return 0x00;
  float factor1, factor2;
  uint16_t ind;
  for (uint16_t j = 0; j < 23; j++)
  {
    ind = 14 - (int16_t)(effStep - j * 1.75) % 14;
    switch ((int)((ind % 14) / 4.666666666666667))
    {
    case 0:
      factor1 = 1.0 - ((float)(ind % 14 - 0 * 4.666666666666667) / 4.666666666666667);
      factor2 = (float)((int)(ind - 0) % 14) / 4.666666666666667;
      Set_LED(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
      WS2812_Send();
      break;
    case 1:
      factor1 = 1.0 - ((float)(ind % 14 - 1 * 4.666666666666667) / 4.666666666666667);
      factor2 = (float)((int)(ind - 4.666666666666667) % 14) / 4.666666666666667;
      Set_LED(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
      WS2812_Send();
      break;
    case 2:
      factor1 = 1.0 - ((float)(ind % 14 - 2 * 4.666666666666667) / 4.666666666666667);
      factor2 = (float)((int)(ind - 9.333333333333334) % 14) / 4.666666666666667;
      Set_LED(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
      WS2812_Send();
      break;
    }
  }
  if (effStep >= 14)
  {
    effStep = 0;
    return 0x03;
  }
  else
    effStep++;
  return 0x01;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int row = 0;
int col = 0;
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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

  lcd_init();
  lcd_put_cur(0, 0);
  lcd_send_string("HELLO ");
  lcd_send_string("WORLD ");
  lcd_send_string("FROM");

  lcd_put_cur(1, 0);
  lcd_send_string("CONTROLLERS TECH");
  HAL_Delay(3000);
  lcd_put_cur(0, 0);
  int count = 0;
  HAL_UART_Receive_IT(&huart1, (uint8_t *)Rx_data, MAX_BUFFER_SIZE);
  ESP_init();
  waitForDataBack = 0;
  int swLed = 0;
  int delayRequest = 29;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  again:
    getADC(&hadc1, &adcValue);
    delayRequest++;
    if (delayRequest > 30)
    {
      delayRequest = 0;
      if (Wait_For("OKKK"))
      {
        waitForDataBack = 0;
        request_fail = 0;
        request_reconnect = 0;
        request_ok++;
      }
      else
      {
        request_fail++;
      }
      if (Wait_For("CLOSED") || Wait_For("ERROR") || Wait_For("busy"))
      {
        ESP_init();
        goto again;
      }
      char str[50];
      if (!waitForDataBack)
      {
        requestId++;
        sprintf(str, "GET /api/data?requestId=%d&value=%d&led1=1\r\n\r\n", requestId, adcValue);
        Send_Request(str);
      }
      if (request_fail > 3)
      {
        lcd_clear();
        lcd_send_string("ERROR");
        Send_Request(str);
        request_reconnect++;
        request_fail = 0;
      }
      if (request_reconnect > 5)
      {
        ESP_init();
        goto again;
      }
      char adc[16];
      sprintf(adc, "GAS SENSOR: %d%%", (adcValue * 100) / 4096);
      lcd_clear();
      lcd_send_string(adc);
    }
    if (Wait_For("case1"))
    {
      swLed = 0;
    }
    if (Wait_For("case2"))
    {
      swLed = 1;
    }
    if (swLed == 1)
    {
      for (int i = 0; i < 24; i++)
      {
        Set_LED(i, 255, 0, 0);
      }
      WS2812_Send();
    }
    else
    {
      rainbow_effect_left();
    }

    // rainbow_effect_right();
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

  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff - 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90 - 1;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
