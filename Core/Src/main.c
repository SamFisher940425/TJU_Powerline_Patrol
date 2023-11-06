/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RS485_READ = 0,
  RS485_WRITE = 1
} RS485_Status;

typedef enum
{
  RS485_CH1 = 0,
  RS485_CH2 = 1
} RS485_Channel;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS232_RX_DATA_LENGTH 35
#define PELCOD_CMD_LENGTH 7
#define PELCOD_CMD_TYPE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask_WDog */
osThreadId_t myTask_WDogHandle;
const osThreadAttr_t myTask_WDog_attributes = {
  .name = "myTask_WDog",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTask_232Rx */
osThreadId_t myTask_232RxHandle;
const osThreadAttr_t myTask_232Rx_attributes = {
  .name = "myTask_232Rx",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask_485C1Tx */
osThreadId_t myTask_485C1TxHandle;
const osThreadAttr_t myTask_485C1Tx_attributes = {
  .name = "myTask_485C1Tx",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTask_WorkFlow */
osThreadId_t myTask_WorkFlowHandle;
const osThreadAttr_t myTask_WorkFlow_attributes = {
  .name = "myTask_WorkFlow",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint8_t g_rs232_status = 0;
volatile uint8_t g_rs232_rx_buf[RS232_RX_DATA_LENGTH] = {0};
volatile uint16_t g_sbus_ch_val[16] = {0};
volatile uint8_t g_sbus_new_frame_flag = 0;
volatile uint8_t g_rs485_tx_buf[PELCOD_CMD_TYPE][PELCOD_CMD_LENGTH] = {0}; // line_0: move, line_1: zoom, line_2: focal, line_3: aperture, line_4: light, line_5: wiper
volatile uint8_t g_pelcod_status[PELCOD_CMD_TYPE] = {0};
volatile uint8_t g_pelcod_new_pack_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartTask_WDog(void *argument);
void StartTask_232Rx(void *argument);
void StartTask_485C1Tx(void *argument);
void StartTask_WorkFlow(void *argument);

/* USER CODE BEGIN PFP */
void RS232_RxEventCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos);
void RS485_Status_Set(RS485_Channel ch, RS485_Status status);

void SBus_Pelcod_Trans(void);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_RegisterRxEventCallback(&huart1, RS232_RxEventCallBack);
  RS485_Status_Set(RS485_CH1, RS485_WRITE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask_WDog */
  myTask_WDogHandle = osThreadNew(StartTask_WDog, NULL, &myTask_WDog_attributes);

  /* creation of myTask_232Rx */
  myTask_232RxHandle = osThreadNew(StartTask_232Rx, NULL, &myTask_232Rx_attributes);

  /* creation of myTask_485C1Tx */
  myTask_485C1TxHandle = osThreadNew(StartTask_485C1Tx, NULL, &myTask_485C1Tx_attributes);

  /* creation of myTask_WorkFlow */
  myTask_WorkFlowHandle = osThreadNew(StartTask_WorkFlow, NULL, &myTask_WorkFlow_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_2_DE_Pin|RS485_2_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_1_DE_Pin|RS485_1_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_3_Pin|LED_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RS485_2_DE_Pin RS485_2_RE_Pin */
  GPIO_InitStruct.Pin = RS485_2_DE_Pin|RS485_2_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_1_DE_Pin RS485_1_RE_Pin */
  GPIO_InitStruct.Pin = RS485_1_DE_Pin|RS485_1_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_2_Pin SW_3_Pin */
  GPIO_InitStruct.Pin = SW_2_Pin|SW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void RS232_RxEventCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos)
{
  HAL_UART_RxEventTypeTypeDef event_type = 3;
  event_type = HAL_UARTEx_GetRxEventType(huart);
  switch (event_type)
  {
  case HAL_UART_RXEVENT_TC:
    if ((g_rs232_rx_buf[33] & 0x0F) == 0x0C)
    {
      uint8_t rs232_xor_result = 0;
      for (uint8_t i = 1; i < (RS232_RX_DATA_LENGTH - 1); i++)
      {
        rs232_xor_result = rs232_xor_result ^ g_rs232_rx_buf[i];
      }
      if (rs232_xor_result == g_rs232_rx_buf[34])
      {
        if (g_sbus_new_frame_flag == 0)
        {
          for (uint8_t i = 0; i < 16; i++)
          {
            g_sbus_ch_val[i] = ((uint16_t)g_rs232_rx_buf[2 * i + 1] << 8) | (uint16_t)g_rs232_rx_buf[2 * i + 2];
          }
          g_sbus_new_frame_flag = 1;
        }
      }
    }
    g_rs232_status = 0;
    break;
  case HAL_UART_RXEVENT_HT:
    /* code */
    break;
  case HAL_UART_RXEVENT_IDLE:
    g_rs232_status = 0;
    break;

  default:
    break;
  }
}

void RS485_Status_Set(RS485_Channel ch, RS485_Status status)
{
  if (status == RS485_READ)
  {
    switch (ch)
    {
    case RS485_CH1:
      HAL_GPIO_WritePin(RS485_1_DE_GPIO_Port, RS485_1_DE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RS485_1_RE_GPIO_Port, RS485_1_RE_Pin, GPIO_PIN_RESET);
      break;
    case RS485_CH2:
      HAL_GPIO_WritePin(RS485_2_DE_GPIO_Port, RS485_2_DE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RS485_2_RE_GPIO_Port, RS485_2_RE_Pin, GPIO_PIN_RESET);
      break;

    default:
      break;
    }
  }
  else if (status == RS485_WRITE)
  {
    switch (ch)
    {
    case RS485_CH1:
      HAL_GPIO_WritePin(RS485_1_DE_GPIO_Port, RS485_1_DE_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RS485_1_RE_GPIO_Port, RS485_1_RE_Pin, GPIO_PIN_SET);
      break;
    case RS485_CH2:
      HAL_GPIO_WritePin(RS485_2_DE_GPIO_Port, RS485_2_DE_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RS485_2_RE_GPIO_Port, RS485_2_RE_Pin, GPIO_PIN_SET);
      break;

    default:
      break;
    }
  }
  else
  {
    return;
  }
}

void SBus_Pelcod_Trans(void)
{
  uint8_t up_down_flag = 0;     // 0 stop 1 down 2 up
  uint8_t up_down_speed = 0;    // 0-63-255
  uint8_t left_right_flag = 0;  // 0 stop 1 left 2 right
  uint8_t left_right_speed = 0; // 0-63-255
  // uint8_t func_code_1 = 0;
  uint8_t func_code_2 = 0;
  uint16_t check_code = 0;

  for (int i = 0; i < PELCOD_CMD_TYPE; i++) // clear cmd history
  {
    for (int j = 0; j < PELCOD_CMD_LENGTH; j++)
    {
      g_rs485_tx_buf[i][j] = 0;
    }
  }

  if (g_sbus_ch_val[2] >= 1550) // up
  {
    up_down_flag = 2;
    if (g_sbus_ch_val[2] >= 1850) // up turbo
    {
      up_down_speed = 0xFF;
    }
    else
    {
      up_down_speed = (g_sbus_ch_val[2] - 1550) * 63 / 300;
    }
  }
  else if (g_sbus_ch_val[2] <= 1450) // down
  {
    up_down_flag = 1;
    if (g_sbus_ch_val[2] <= 1150) // down turbo
    {
      up_down_speed = 0xFF;
    }
    else
    {
      up_down_speed = (1450 - g_sbus_ch_val[2]) * 63 / 300;
    }
  }
  else
  {
    up_down_flag = 0;
    up_down_speed = 0;
  }

  if (g_sbus_ch_val[3] >= 1550) // right
  {
    left_right_flag = 2;
    if (g_sbus_ch_val[3] >= 1850) // right turbo
    {
      left_right_speed = 0xFF;
    }
    else
    {
      left_right_speed = (g_sbus_ch_val[3] - 1550) * 63 / 300;
    }
  }
  else if (g_sbus_ch_val[3] <= 1450) // left
  {
    left_right_flag = 1;
    if (g_sbus_ch_val[3] <= 1150) // left turbo
    {
      left_right_speed = 0xFF;
    }
    else
    {
      left_right_speed = (1450 - g_sbus_ch_val[3]) * 63 / 300;
    }
  }
  else
  {
    left_right_flag = 0;
    left_right_speed = 0;
  }

  func_code_2 = 0;
  if (up_down_flag == 2)
  {
    func_code_2 |= 0x08;
  }
  else // if (up_down_flag == 1)
  {
    func_code_2 |= 0x10;
  }
  // else
  // {
  //   func_code_2 &= ~(0x18);
  // }
  if (left_right_flag == 2)
  {
    func_code_2 |= 0x02;
  }
  else // if (left_right_flag == 1)
  {
    func_code_2 |= 0x04;
  }
  // else
  // {
  //   func_code_2 &= ~(0x06);
  // }

  g_rs485_tx_buf[0][0] = 0xFF;
  g_rs485_tx_buf[0][1] = 0x01;
  g_rs485_tx_buf[0][2] = 0x00;
  g_rs485_tx_buf[0][3] = func_code_2;
  g_rs485_tx_buf[0][4] = left_right_speed;
  g_rs485_tx_buf[0][5] = up_down_speed;
  check_code = 0;
  for (int i = 1; i < 6; i++)
  {
    check_code += g_rs485_tx_buf[0][i];
  }
  check_code = check_code % 256;
  g_rs485_tx_buf[0][6] = (uint8_t)(check_code & 0x00FF);

  if (g_sbus_ch_val[6] > 1750) // zoom
  {
    if (g_pelcod_status[1] != 2)
    {
      g_rs485_tx_buf[1][0] = 0xFF;
      g_rs485_tx_buf[1][1] = 0x01;
      g_rs485_tx_buf[1][2] = 0x00;
      g_rs485_tx_buf[1][3] = 0x20;
      g_rs485_tx_buf[1][4] = 0x00;
      g_rs485_tx_buf[1][5] = 0x00;
      g_rs485_tx_buf[1][6] = 0x21;
      g_pelcod_status[1] = 2;
    }
    else
    {
      for (int i = 0; i < PELCOD_CMD_LENGTH; i++)
      {
        g_rs485_tx_buf[1][i] = 0;
      }
    }
  }
  else if (g_sbus_ch_val[6] < 1250)
  {
    if (g_pelcod_status[1] != 1)
    {
      g_rs485_tx_buf[1][0] = 0xFF;
      g_rs485_tx_buf[1][1] = 0x01;
      g_rs485_tx_buf[1][2] = 0x00;
      g_rs485_tx_buf[1][3] = 0x40;
      g_rs485_tx_buf[1][4] = 0x00;
      g_rs485_tx_buf[1][5] = 0x00;
      g_rs485_tx_buf[1][6] = 0x41;
      g_pelcod_status[1] = 1;
    }
    else
    {
      for (int i = 0; i < PELCOD_CMD_LENGTH; i++)
      {
        g_rs485_tx_buf[1][i] = 0;
      }
    }
  }
  else
  {
    if (g_pelcod_status[1] != 0)
    {
      g_rs485_tx_buf[1][0] = 0xFF;
      g_rs485_tx_buf[1][1] = 0x01;
      g_rs485_tx_buf[1][2] = 0x00;
      g_rs485_tx_buf[1][3] = 0x00;
      g_rs485_tx_buf[1][4] = 0x00;
      g_rs485_tx_buf[1][5] = 0x00;
      g_rs485_tx_buf[1][6] = 0x01;
      g_pelcod_status[1] = 0;
    }
    else
    {
      for (int i = 0; i < PELCOD_CMD_LENGTH; i++)
      {
        g_rs485_tx_buf[1][i] = 0;
      }
    }
  }

  if (g_sbus_ch_val[7] > 1750) // focal
  {
    if (g_pelcod_status[2] != 2)
    {
      g_rs485_tx_buf[2][0] = 0xFF;
      g_rs485_tx_buf[2][1] = 0x01;
      g_rs485_tx_buf[2][2] = 0x01;
      g_rs485_tx_buf[2][3] = 0x00;
      g_rs485_tx_buf[2][4] = 0x00;
      g_rs485_tx_buf[2][5] = 0x00;
      g_rs485_tx_buf[2][6] = 0x02;
      g_pelcod_status[2] = 2;
    }
    else
    {
      for (int i = 0; i < PELCOD_CMD_LENGTH; i++)
      {
        g_rs485_tx_buf[2][i] = 0;
      }
    }
  }
  else if (g_sbus_ch_val[7] < 1250)
  {
    if (g_pelcod_status[2] != 1)
    {
      g_rs485_tx_buf[2][0] = 0xFF;
      g_rs485_tx_buf[2][1] = 0x01;
      g_rs485_tx_buf[2][2] = 0x00;
      g_rs485_tx_buf[2][3] = 0x80;
      g_rs485_tx_buf[2][4] = 0x00;
      g_rs485_tx_buf[2][5] = 0x00;
      g_rs485_tx_buf[2][6] = 0x81;
      g_pelcod_status[2] = 1;
    }
    else
    {
      for (int i = 0; i < PELCOD_CMD_LENGTH; i++)
      {
        g_rs485_tx_buf[2][i] = 0;
      }
    }
  }
  else
  {
    if (g_pelcod_status[2] != 0)
    {
      g_rs485_tx_buf[2][0] = 0xFF;
      g_rs485_tx_buf[2][1] = 0x01;
      g_rs485_tx_buf[2][2] = 0x00;
      g_rs485_tx_buf[2][3] = 0x00;
      g_rs485_tx_buf[2][4] = 0x00;
      g_rs485_tx_buf[2][5] = 0x00;
      g_rs485_tx_buf[2][6] = 0x01;
      g_pelcod_status[2] = 0;
    }
    else
    {
      for (int i = 0; i < PELCOD_CMD_LENGTH; i++)
      {
        g_rs485_tx_buf[2][i] = 0;
      }
    }
  }

  // if (g_sbus_ch_val[4] > 1750) // light
  // {
  //   g_rs485_tx_buf[4][0] = 0x00;
  //   g_rs485_tx_buf[4][1] = 0x00;
  //   g_rs485_tx_buf[4][2] = 0x00;
  //   g_rs485_tx_buf[4][3] = 0x00;
  //   g_rs485_tx_buf[4][4] = 0x00;
  //   g_rs485_tx_buf[4][5] = 0x00;
  //   g_rs485_tx_buf[4][6] = 0x00;
  // }
  // else if (g_sbus_ch_val[4] < 1250)
  // {
  //   g_rs485_tx_buf[4][0] = 0x00;
  //   g_rs485_tx_buf[4][1] = 0x00;
  //   g_rs485_tx_buf[4][2] = 0x00;
  //   g_rs485_tx_buf[4][3] = 0x00;
  //   g_rs485_tx_buf[4][4] = 0x00;
  //   g_rs485_tx_buf[4][5] = 0x00;
  //   g_rs485_tx_buf[4][6] = 0x00;
  // }
  // else
  // {
  //   g_rs485_tx_buf[4][0] = 0x00;
  //   g_rs485_tx_buf[4][1] = 0x00;
  //   g_rs485_tx_buf[4][2] = 0x00;
  //   g_rs485_tx_buf[4][3] = 0x00;
  //   g_rs485_tx_buf[4][4] = 0x00;
  //   g_rs485_tx_buf[4][5] = 0x00;
  //   g_rs485_tx_buf[4][6] = 0x00;
  // }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(500);
    HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_WDog */
/**
 * @brief Function implementing the myTask_WDog thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_WDog */
void StartTask_WDog(void *argument)
{
  /* USER CODE BEGIN StartTask_WDog */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 300);
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END StartTask_WDog */
}

/* USER CODE BEGIN Header_StartTask_232Rx */
/**
 * @brief Function implementing the myTask_232Rx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_232Rx */
void StartTask_232Rx(void *argument)
{
  /* USER CODE BEGIN StartTask_232Rx */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for (;;)
  {
    xLastWakeTime = osKernelGetTickCount();
    osDelayUntil(xLastWakeTime + 5);
    switch (g_rs232_status)
    {
    case 0:
      if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)g_rs232_rx_buf, RS232_RX_DATA_LENGTH) == HAL_OK)
      {
        g_rs232_status = 1;
      }
      break;

    default:
      break;
    }
  }
  /* USER CODE END StartTask_232Rx */
}

/* USER CODE BEGIN Header_StartTask_485C1Tx */
/**
 * @brief Function implementing the myTask_485C1Tx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_485C1Tx */
void StartTask_485C1Tx(void *argument)
{
  /* USER CODE BEGIN StartTask_485C1Tx */
  static int8_t rs485_tx_line = (PELCOD_CMD_TYPE - 1);
  /* Infinite loop */
  for (;;)
  {
    osDelay(20);
    if (g_pelcod_new_pack_flag == 1) // a new pack needs to be sent
    {
      while (rs485_tx_line >= 0) // find valid frame
      {
        if (g_rs485_tx_buf[rs485_tx_line][0] == 0xFF) // valid frame
        {
          break;
        }
        else // empty frame
        {
          rs485_tx_line -= 1;
        }
      }
      if (rs485_tx_line >= 0)
      {
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&g_rs485_tx_buf[rs485_tx_line][0], PELCOD_CMD_LENGTH);
        rs485_tx_line -= 1;
      }

      if (rs485_tx_line < 0)
      {
        g_pelcod_new_pack_flag = 0;
        rs485_tx_line = (PELCOD_CMD_TYPE - 1);
      }
    }
  }
  /* USER CODE END StartTask_485C1Tx */
}

/* USER CODE BEGIN Header_StartTask_WorkFlow */
/**
 * @brief Function implementing the myTask_WorkFlow thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_WorkFlow */
void StartTask_WorkFlow(void *argument)
{
  /* USER CODE BEGIN StartTask_WorkFlow */
  static uint16_t miss_cnt = 0;
  static uint8_t connect_flag = 0;
  /* Infinite loop */
  for (;;)
  {
    osDelay(5);
    if (g_sbus_new_frame_flag == 1 && g_pelcod_new_pack_flag == 0)
    {
      SBus_Pelcod_Trans();
      g_pelcod_new_pack_flag = 1;
      g_sbus_new_frame_flag = 0;
      connect_flag = 1;
      miss_cnt = 0;
    }
    else
    {
      if (connect_flag == 1)
      {
        miss_cnt += 1;
        if (miss_cnt > 200)
        {
          // fall safe execution
          g_rs485_tx_buf[0][0] = 0xFF;
          g_rs485_tx_buf[0][1] = 0x01;
          g_rs485_tx_buf[0][2] = 0x00;
          g_rs485_tx_buf[0][3] = 0x00;
          g_rs485_tx_buf[0][4] = 0x00;
          g_rs485_tx_buf[0][5] = 0x00;
          g_rs485_tx_buf[0][6] = 0x01;
          for (int i = 1; i < PELCOD_CMD_TYPE; i++)
          {
            for (int j = 0; j < PELCOD_CMD_LENGTH; j++)
            {
              g_rs485_tx_buf[i][j] = 0;
            }
          }
          g_pelcod_new_pack_flag = 1;
          miss_cnt = 0;
          connect_flag = 0;
        }
      }
    }
  }
  /* USER CODE END StartTask_WorkFlow */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
