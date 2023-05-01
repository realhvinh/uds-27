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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define data_length 8
#define tester_ID 	0x712
#define ECU_ID 		0x7A2
#define true 		1
#define false 		0
#define zero		0
#define over_time 	10000
#define LEFT 0xAA
#define MID 0x00
#define RIGHT 0xFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef tester_TxHeader;
CAN_RxHeaderTypeDef tester_RxHeader;
uint8_t tester_TxData[data_length];
uint8_t tester_RxData[data_length];
uint32_t tester_TxMailbox;
CAN_TxHeaderTypeDef ECU_TxHeader;
CAN_RxHeaderTypeDef ECU_RxHeader;
uint8_t ECU_TxData[data_length];
uint8_t ECU_RxData[data_length];
uint32_t ECU_TxMailbox;

uint8_t ECU_store_seed[data_length];	// ECU control
uint8_t joystick;						// ECU control
uint16_t count = zero;					// ECU control
uint8_t is_ECU_unlock = false;			// ECU control
uint8_t is_run_timer = false;			// ECU control
uint8_t is_get_fifo1 = false;			// ECU control
uint8_t is_over_time = false;			// ECU control

uint8_t is_get_fifo0 = false;			// tester control
uint8_t flag_wake_up_key = false;		// tester control
uint8_t flag_user_key = false;			// tester control
uint8_t flag_left_key = false;			// tester control
uint8_t flag_right_key = false;			// tester control
uint8_t flag_mid_key = false;			// tester control

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);	// ECU control
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);		// ECU control
void securityAccessServiceSeedRequestFrame();						// tester control
void securityAccessServiceSeedResponseFrame();						// ECU control
void securityAccessSendKeyRequestFrame(uint8_t Tx_Data[], uint8_t Data[]);	// tester control
void securityAccessSendKeyResponseFrame();							// ECU control
void SecurityAccessNegativeResponseMessage(uint8_t Data[], uint8_t flag_NRC);
void ReadDataByIdentifierRequestFrame(uint8_t Data[]);						// tester control
void ReadDataByIdentifierResponseFrame(uint8_t Data[], uint8_t adc_value);	// ECU control
void WriteDataByIdentifierRequestFrame(uint8_t Data[], uint8_t joystick_position);
void WriteDataByIdentifierResponseFrame(uint8_t Data[]);			// ECU control
void Service22();								// tester control
void Service27();								// tester control
void Service2E(uint8_t joystick_position);		// tester control
void CheckAndHandleService22();					// tester control
void CheckAndHandleService27(); 				// tester control
void CheckAndHandleService2E();					// tester control
void CheckAndHandleTesterCANFIFO0();			// tester control
void CheckAndHandleECUCANFIFO1();				// ECU control
void CheckOverTime();							// ECU control
void HandleSID22();								// ECU control
void HandleSID27();								// ECU control
void HandleSID2E();								// ECU control
void HandleSID62();								// tester control
void HandleSID67();								// tester control
void HandleSID6E();								// tester control
void HandleSID7F();								// tester control
void store_joystick_value(uint8_t Data);		// ECU control
uint8_t is_accept_key(uint8_t Data[],uint8_t  store_seed[]);	// ECU control
uint8_t get_PCI(uint8_t);
uint8_t get_size(uint8_t);
uint8_t get_SID(uint8_t Data[]);
uint16_t get_DID(uint8_t Data[]);
uint8_t get_sub_function(uint8_t Data[]);
uint8_t get_NRC(uint8_t Data[]);
uint8_t generate_seed();
uint8_t cal_key(uint8_t);

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);			// khoi dong module ADC
  HAL_TIM_Base_Start_IT(&htim3);	// khoi dong module timer
  HAL_CAN_Start(&hcan1);			// khởi động module can1
  HAL_CAN_Start(&hcan2);			// khởi động module can2
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // Activate the notification
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  CheckAndHandleService27();
	  CheckAndHandleService22();
	  CheckAndHandleService2E();
	  CheckAndHandleTesterCANFIFO0();
	  CheckAndHandleECUCANFIFO1();
	  CheckOverTime();
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
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
  sConfig.Channel = ADC_CHANNEL_6;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	tester_TxHeader.DLC = data_length;  // data length
	tester_TxHeader.IDE = CAN_ID_STD;
	tester_TxHeader.RTR = CAN_RTR_DATA;
	tester_TxHeader.StdId = tester_ID;  // ID
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
	for (int i = 0; i < 8; i++){
		tester_TxData[i] = 0x55;
	}
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = ECU_ID<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = ECU_ID<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
	ECU_TxHeader.DLC = data_length;  // data length
	ECU_TxHeader.IDE = CAN_ID_STD;
	ECU_TxHeader.RTR = CAN_RTR_DATA;
	ECU_TxHeader.StdId = ECU_ID;  // ID
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
	for (int i = 0; i < 8; i++){
		ECU_TxData[i] = 0x55;
	}
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig.FilterIdHigh = tester_ID<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = tester_ID<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // doesn't matter in single can controllers
  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 839;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MID_Pin LEFT_Pin RIGHT_Pin */
  GPIO_InitStruct.Pin = MID_Pin|LEFT_Pin|RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : WakeUp_Pin USER_Pin */
  GPIO_InitStruct.Pin = WakeUp_Pin|USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void store_joystick_value(uint8_t Data){
	joystick = Data;
}
uint8_t get_PCI(uint8_t Data){
	return Data >> 4;
}
uint8_t get_size(uint8_t Data){
	return Data & 0x0F;
}
uint8_t get_SID(uint8_t Data[]){
	return Data[1];
}
uint16_t get_DID(uint8_t Data[]){
	uint16_t temp = Data[2];
	temp <<= 8;
	temp += Data[3];
	return temp;
}
uint8_t get_sub_function(uint8_t Data[]){
	return Data[2];
}
uint8_t generate_seed(){
	return (uint8_t)(HAL_GetTick());
}
uint8_t cal_key(uint8_t Data){
	return Data + 1;
}
uint8_t is_accept_key(uint8_t Data[],uint8_t  store_seed[]){
	for (int i = 3; i < 8; i++){
		if (Data[i] != cal_key(store_seed[i])) return false;
	}
	return true;
}
uint8_t get_NRC(uint8_t Data[]){
	return Data[2];
}
void ReadDataByIdentifierRequestFrame(uint8_t Data[]){
	Data[0] = 0x03;	// PCi, size
	Data[1] = 0x22;	// SID
	Data[2] = 0xF0;	// DID
	Data[3] = 0x02;	// DID
}
void ReadDataByIdentifierResponseFrame(uint8_t Data[], uint8_t adc_value){
	Data[0] = 0x04;	// PCI
	Data[1] = 0x62;	// PSID
	Data[2] = 0xF0;	// DID
	Data[3] = 0x02; // DID
	Data[4] = adc_value;	// data
}
void WriteDataByIdentifierRequestFrame(uint8_t Data[], uint8_t joystick_position){
	Data[0] = 0x04;	// PCi, size
	Data[1] = 0x2E;	// SID
	Data[2] = 0xF1;	// DID
	Data[3] = 0x12;	// DID
	Data[4] = joystick_position;	// data
}
void WriteDataByIdentifierResponseFrame(uint8_t Data[]){
	Data[0] = 0x03;	// PCi, size
	Data[1] = 0x6E;	// PSID
	Data[2] = 0xF1;	// DID
	Data[3] = 0x12;	// DID
}
void securityAccessServiceSeedRequestFrame(uint8_t Data[]){
	Data[0] = 0x02;	// PCI la 0, size la 2
	Data[1] = 0x27;	// service 27
	Data[2] = 0x01;	// request seed
}
void securityAccessServiceSeedResponseFrame(uint8_t Data[]){
	Data[0] = 0x07;	// PCI la 0, size la 4
	Data[1] = 0x67;	// response $27 nen la $67
	Data[2] = 0x01;	// request seed
	Data[3] = generate_seed();	// Security Seed
	ECU_store_seed[3] = Data[3];
	Data[4] = generate_seed();	// Security Seed
	ECU_store_seed[4] = Data[4];
	Data[5] = generate_seed();	// Security Seed
	ECU_store_seed[5] = Data[5];
	Data[6] = generate_seed();	// Security Seed
	ECU_store_seed[6] = Data[6];
	Data[7] = generate_seed();	// Security Seed
	ECU_store_seed[7] = Data[7];
}
void securityAccessSendKeyRequestFrame(uint8_t Tx_Data[], uint8_t Data[]){
	Tx_Data[0] = 0x07;				// PCI la 0, size la 4
	Tx_Data[1] = 0x27;				// response $27 nen la $67
	Tx_Data[2] = 0x02;				// response seed = request seed + 1
	Tx_Data[3] = cal_key(Data[3]);	// Security key
	Tx_Data[4] = cal_key(Data[4]);	// Security key
	Tx_Data[5] = cal_key(Data[5]);	// Security key
	Tx_Data[6] = cal_key(Data[6]);	// Security key
	Tx_Data[7] = cal_key(Data[7]);	// Security key
}
void securityAccessSendKeyResponseFrame(uint8_t Data[]){
	Data[0] = 0x02;	// PCI la 0, size la 2
	Data[1] = 0x67;	// response $27 nen la $67
	Data[2] = 0x02;	// response seed = request seed + 1
	if (is_ECU_unlock == true) Data[3] = false;	// unlock or lock
	else Data[3] = true;
}
void SecurityAccessNegativeResponseMessage(uint8_t Data[], uint8_t flag_NRC){
	Data[0] = 0x03;	// PCI la 0, size la 3
	Data[1] = 0x7F;	// NRC code
	switch(flag_NRC) {
		case 0x12:
			Data[2] = 0x12;	// sub function not supported
			break;
		case 0x35:
			Data[2] = 0x35;	// invalid key
			break;
		case 0x13:
			Data[2] = 0x13;	// incorrect message length or invalid format
		default:
			Data[2] = 0x33;	//securityAccessDenied
	}
}
void HandleSID22(){
	uint16_t DID = get_DID(ECU_RxData);
	if (DID == 0xF002){
		is_run_timer = true;
		HAL_ADC_Start(&hadc1);
		HAL_Delay(10);
		uint8_t adc_value = HAL_ADC_GetValue(&hadc1);
		ReadDataByIdentifierResponseFrame(ECU_TxData, adc_value);
	}
	else{
		SecurityAccessNegativeResponseMessage(ECU_TxData, 0x12);
	}
	HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
}
void HandleSID27(){
	uint8_t SBF = get_sub_function(ECU_RxData);
	if (SBF == 0x01){
		securityAccessServiceSeedResponseFrame(ECU_TxData);
	}
	else if (SBF == 0x02){
		if (is_accept_key(ECU_RxData, ECU_store_seed) == true){
			if (is_ECU_unlock == true){
				is_ECU_unlock = false;
				securityAccessSendKeyResponseFrame(ECU_TxData);
			}
			else{
				is_ECU_unlock = true;
				is_run_timer = true;
				securityAccessSendKeyResponseFrame(ECU_TxData);
			}
		}
		else {
			SecurityAccessNegativeResponseMessage(ECU_TxData, 0x35);// invalid key
		}
	}
	else SecurityAccessNegativeResponseMessage(ECU_TxData, 0x12);	// invalid sbf
	HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
}
void HandleSID2E(){
	uint16_t DID = get_DID(ECU_RxData);
	if (DID == 0xF112){
		store_joystick_value(ECU_RxData[4]);
		WriteDataByIdentifierResponseFrame(ECU_TxData);
		is_run_timer = true;
	}
	else{
		SecurityAccessNegativeResponseMessage(ECU_TxData, 0x12);
	}
	HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
}
void HandleSID67(){
	uint8_t SBF = get_sub_function(tester_RxData);
	if (SBF == 0x01){
		securityAccessSendKeyRequestFrame(tester_TxData, tester_RxData);
		HAL_CAN_AddTxMessage(&hcan1, &tester_TxHeader, tester_TxData, &tester_TxMailbox);
	}
	else if (SBF == 0x02){
		if (tester_RxData[3] == false){
			printf("Unlocked\n\r");
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		}
		else{
			printf("Locked\n\r");
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		}
	}
}
void HandleSID62(){
	uint16_t DID = get_DID(tester_RxData);
	if (DID == 0xF002){
		uint8_t adc_value = tester_RxData[4];
		printf("ADC value: %x\n\r", adc_value);
	}
}
void HandleSID6E(){
	uint16_t DID = get_DID(tester_RxData);
	if (DID == 0xF112){
		printf("Written joystick position\n\r");
		printf("joystick position: %x\n\r", joystick);
	}
}
void HandleSID7F(){
	uint8_t NRC = get_NRC(tester_RxData);
	if (NRC == 0x12){
		printf("subFunctionNotSupported\n\r");
	}
	else if (NRC == 0x35){
		printf("invalidKey\n\r");
	}
	else if (NRC == 0x13){
		printf("incorrectMessageLengthOrInvalidFormat\n\r");
	}
	else if (NRC == 0x33){
		printf("securityAccessDenied\n\r");
	}
}
void Service22(){
	ReadDataByIdentifierRequestFrame(tester_TxData);
	HAL_CAN_AddTxMessage(&hcan1, &tester_TxHeader, tester_TxData, &tester_TxMailbox);
}
void Service27(){
	securityAccessServiceSeedRequestFrame(tester_TxData);
	HAL_CAN_AddTxMessage(&hcan1, &tester_TxHeader, tester_TxData, &tester_TxMailbox);
}
void Service2E(uint8_t joystick_position){
	WriteDataByIdentifierRequestFrame(tester_TxData, joystick_position);
	HAL_CAN_AddTxMessage(&hcan1, &tester_TxHeader, tester_TxData, &tester_TxMailbox);
}
void CheckAndHandleService22(){
	if (flag_user_key == true){
		flag_user_key = false;
		Service22();
	}
}
void CheckAndHandleService27(){
	if (flag_wake_up_key == true){
		flag_wake_up_key = false;
		Service27();
	}
}
void CheckAndHandleService2E(){
	if (flag_left_key == true){
		flag_left_key = false;
		Service2E(LEFT);
	}
	else if (flag_mid_key == true){
		flag_mid_key = false;
		Service2E(MID);
	}
	else if (flag_right_key == true){
		flag_right_key = false;
		Service2E(RIGHT);
	}
}
void CheckAndHandleTesterCANFIFO0(){
	if (is_get_fifo0 == true){
		is_get_fifo0 = false;
		uint8_t PCI = get_PCI(tester_RxData[0]);
		if (PCI == 0x00){
			uint8_t SID = get_SID(tester_RxData);
			if (SID == 0x62) HandleSID62();
			else if (SID == 0x67) HandleSID67();
			else if (SID == 0x6E) HandleSID6E();
			else if (SID == 0x7F) HandleSID7F();
		}
	}
}
void CheckAndHandleECUCANFIFO1(){
	if (is_get_fifo1 == true){
		is_get_fifo1 = false;
		uint8_t PCI = get_PCI(ECU_RxData[0]);
		uint8_t LEN = get_size(ECU_RxData[0]);
		if (PCI == 0x00){
			uint8_t SID = get_SID(ECU_RxData);
			if (is_ECU_unlock == true){
				if (SID == 0x22){
					if (LEN == 0x03) HandleSID22();
					else {
						SecurityAccessNegativeResponseMessage(ECU_TxData, 0x13);
						HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
					}
				}
				else if (SID == 0x2E) {
					if (LEN == 0x04) HandleSID2E();
					else {
						SecurityAccessNegativeResponseMessage(ECU_TxData, 0x13);
						HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
					}
				}
				else if (SID == 0x27){
					if (LEN == 0x07 || LEN == 0x02) HandleSID27();
				}
			}
			else if (is_ECU_unlock == false){
				if (SID == 0x27){
					if (LEN == 0x07 || LEN == 0x02) HandleSID27();
					else {
						SecurityAccessNegativeResponseMessage(ECU_TxData, 0x13);
						HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
					}
				}
				else {
					SecurityAccessNegativeResponseMessage(ECU_TxData, 0x33);
					HAL_CAN_AddTxMessage(&hcan2, &ECU_TxHeader, ECU_TxData, &ECU_TxMailbox);
				}
			}
		}
	}
}
void CheckOverTime(){
	if (is_over_time == true){
		is_over_time = false;
		is_run_timer = false;
		count = 0;
		is_ECU_unlock = false;
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		printf("System's locked\n\r");
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == WakeUp_Pin){
		printf("Wake up\n\r");
		flag_wake_up_key = true;
		while(HAL_GPIO_ReadPin(WakeUp_GPIO_Port, WakeUp_Pin) == GPIO_PIN_RESET);
	}
	else if (GPIO_Pin == USER_Pin){
		printf("User\n\r");
		flag_user_key = true;
		while(HAL_GPIO_ReadPin(USER_GPIO_Port, USER_Pin) == GPIO_PIN_RESET);
	}
	else if (GPIO_Pin == LEFT_Pin){
		printf("Left\n\r");
		flag_left_key = true;
		while(HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == GPIO_PIN_RESET);
	}
	else if (GPIO_Pin == RIGHT_Pin){
		printf("Right\n\r");
		flag_right_key = true;
		while(HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == GPIO_PIN_RESET);
	}
	else if (GPIO_Pin == MID_Pin){
		printf("Mid\n\r");
		flag_mid_key = true;
		while(HAL_GPIO_ReadPin(MID_GPIO_Port, MID_Pin) == GPIO_PIN_RESET);
	}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tester_RxHeader, tester_RxData);
	if (tester_RxHeader.DLC == data_length){
		is_get_fifo0 = true;
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &ECU_RxHeader, ECU_RxData);
	if (ECU_RxHeader.DLC == data_length){
		is_get_fifo1 = true;
		is_run_timer = false;
		count = 0;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		if (is_run_timer == true){
			count++;
			if (count % 1000 == 0) printf("%d\n\r", count);
			if (count == over_time) {
				is_over_time = true;
				count %= over_time;
			}
		}
		else count = zero;
	}
}
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
  return ch;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n\r", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
