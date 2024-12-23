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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#define V1_SENSOR_MULT	0.000837696335//0.996551//Encontra a tensão da Saída do Sensor x100
#define V1_REAL_MULT 	789.1033381//808.1496160//63.37024//Encontra a tensão de entrada do Sensor  x100

#define C2_SENSOR_MULT	0.000878232758//0.362903//Encontra a tensão da Saída do Sensor x100
#define C2_REAL_MULT	23.10292756//5.4691//Encontra a Corrente de entrada do Sensor  x100

#define F_BUFFER_SIZE	256
#define H_BUFFER_SIZE	128

//UART
#define ESC 0x10
#define ESC_INC 0x20
#define DEVICE_ADDR  0x01

#define RESPONSE_OPCODE_MASK 0x80

#define MAX_PACKAGE_LEN 150

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for adchalfselectQueue */
osMessageQueueId_t adchalfselectQueueHandle;
const osMessageQueueAttr_t adchalfselectQueue_attributes = {
  .name = "adchalfselectQueue"
};
/* Definitions for uartqueue */
osMessageQueueId_t uartqueueHandle;
const osMessageQueueAttr_t uartqueue_attributes = {
  .name = "uartqueue"
};
/* Definitions for uartBinSema */
osSemaphoreId_t uartBinSemaHandle;
const osSemaphoreAttr_t uartBinSema_attributes = {
  .name = "uartBinSema"
};
/* USER CODE BEGIN PV */

uint32_t 	adcBuffer[F_BUFFER_SIZE];
float 		adc1_voltage[H_BUFFER_SIZE];
float 		adc2_current[H_BUFFER_SIZE];

float cc_voltage = 0.0;
float rms_voltage = 0.0;
float cc_current = 0.0;
float rms_current = 0.0;
float pot_aparente = 0.0;
float pot_reativa = 0.0;
float pot_ativa = 0.0;
float pf = 0.0;


uint8_t rx_buffer[256];
uint8_t tx_buffer[256];
uint16_t rx_index = 0;


enum UART_PACKAGE_PARTS
{
  UPP_STX,
  UPP_DEVICE_ADDRESS,
  UPP_OPCODE,
  UPP_DATA_LEN,
  UPP_DATA,
  UPP_CHECKSUM,
  UPP_ETX
};

struct UART_PACKAGE_PROTOCOL
{
  unsigned char uc_Stx;
  unsigned char uc_DeviceAddress;
  unsigned char uc_OpCode;
  unsigned char uc_Datalen;
  unsigned char uc_Data[MAX_PACKAGE_LEN];
  unsigned char uc_Checksum;
  unsigned char uc_Etx;
};

UART_PACKAGE_PARTS  m_udtUartPackageParts;
UART_PACKAGE_PROTOCOL m_udtReceptionPackage;
UART_PACKAGE_PROTOCOL m_udtTransmitionPackage;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
void StartUartTask(void *argument);
void StartAdcTask(void *argument);

/* USER CODE BEGIN PFP */



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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uartBinSema */
  uartBinSemaHandle = osSemaphoreNew(1, 1, &uartBinSema_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of adchalfselectQueue */
  adchalfselectQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &adchalfselectQueue_attributes);

  /* creation of uartqueue */
  uartqueueHandle = osMessageQueueNew (128, sizeof(uint8_t), &uartqueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);

  /* creation of adcTask */
  adcTaskHandle = osThreadNew(StartAdcTask, NULL, &adcTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10416;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	uint8_t bufferside=1;
	xQueueSendToBackFromISR(adchalfselectQueueHandle, &bufferside, &pxHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	uint8_t bufferside=2;
	xQueueSendToBackFromISR(adchalfselectQueueHandle, &bufferside, &pxHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

        if (rx_buffer[rx_index] == ETX) {

        	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(uartQueue, rx_buffer, &pxHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

            rx_index = 0;
        } else {

            rx_index++;
        }
        HAL_UART_Receive_IT(&huart4, &rx_buffer[rx_index], 1);

}


void UART_SendPacket(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len) {
    uint8_t send_buffer[256];
    uint16_t idx = 0;

    send_buffer[idx++] = STX;

    for (int i = 0; i < len; i++) {
        if (data[i] == STX || data[i] == ETX || data[i] == ESCAPE) {
            // Se for um código especial, envie ESCAPE + valor + 0x20
            send_buffer[idx++] = ESCAPE;
            send_buffer[idx++] = data[i] + 0x20;
        } else {
            // Caso contrário, envie o valor normal
            send_buffer[idx++] = data[i];
        }
    }

    send_buffer[idx++] = ETX;  // Fim do pacote (ETX)

    // Envia os dados via UART
    HAL_UART_Transmit(huart, send_buffer, idx, HAL_MAX_DELAY);
}

// Função para receber pacotes com tratamento de ESCAPE CODE
void UART_ReceivePacket(UART_HandleTypeDef *huart, uint8_t *recv_buffer, uint16_t *recv_len) {
    uint8_t byte;
    uint16_t idx = 0;
    HAL_StatusTypeDef status;

    *recv_len = 0;
    while (1) {
        // Recebe um byte via UART
        status = HAL_UART_Receive(huart, &byte, 1, HAL_MAX_DELAY);
        if (status == HAL_OK) {
            if (byte == ESCAPE) {
                // Se o byte for ESCAPE, o próximo byte é um valor especial
                HAL_UART_Receive(huart, &byte, 1, HAL_MAX_DELAY);
                recv_buffer[idx++] = byte - 0x20;  // Subtrai 0x20 para restaurar o valor original
            } else {
                recv_buffer[idx++] = byte;
            }

            // Verifica se recebeu o ETX (fim do pacote)
            if (recv_buffer[idx - 1] == ETX) {
                *recv_len = idx;
                break;
            }
        }
    }
}

// Função para preparar a resposta com base no OPCODE recebido
void PrepareResponse(uint8_t opcode, uint8_t *response_data, uint16_t *response_len) {
    switch (opcode) {
        case 0x10:  // Exemplo de OPCODE 1
            response_data[0] = 0x10;  // Resposta para o OPCODE 1
            response_data[1] = 0x20;
            *response_len = 2;
            break;
        default:
            response_data[0] = 0xFF;
            *response_len = 1;
            break;
    }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartTask */
/**
  * @brief  Function implementing the uartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t packet[256];
    uint16_t recv_len;

  /* Infinite loop */
  while(1)
  {

	if (xQueueReceive(uartQueue, packet, portMAX_DELAY) == pdTRUE) {

		if (packet[0] == STX && packet[recv_len - 1] == ETX) {
			uint8_t opcode = packet[2];
			uint8_t response_buffer[256];
			uint16_t response_len;

			// Prepara a resposta com base no OPCODE
			PrepareResponse(opcode, response_buffer, &response_len);

			// Envia o pacote de resposta
			HAL_UART_Transmit_IT(&huart4, response_buffer, response_len);
		}
	}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAdcTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void *argument)
{
  /* USER CODE BEGIN StartAdcTask */

	uint8_t sidebuffer_choice = 0;
	uint16_t i = 0;

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcBuffer, F_BUFFER_SIZE);
	HAL_TIM_Base_Start(&htim1);

	cc_voltage = 0.0;
	cc_current = 0.0;
	rms_voltage = 0.0;
	rms_current = 0.0;
	pot_ativa = 0.0;
	pot_aparente = 0.0;
	pot_reativa = 0.0;
	pf = 0.0;
  /* Infinite loop */
  while(1)
  {
		xQueueReceive(adchalfselectQueueHandle, &sidebuffer_choice, portMAX_DELAY);

		if (sidebuffer_choice == 1){
			i = 0;
		}
		if (sidebuffer_choice == 2){
			i = H_BUFFER_SIZE;
		}

		for(uint16_t c = i; c < H_BUFFER_SIZE; c++){
				// Extrai os 16 bits menos significativos
				adc1_voltage[c] = (((uint16_t)(adcBuffer[c] & 0x0000FFFF)) * V1_SENSOR_MULT * V1_REAL_MULT);

				cc_voltage += adc1_voltage[c];

				// Extrai os 16 bits mais significativos
				adc2_current[c] = (((uint16_t)((adcBuffer[c] >> 16) & 0x0000FFFF)) * C2_SENSOR_MULT * C2_REAL_MULT);

				cc_current += adc2_current[c];
		}

		cc_voltage /= H_BUFFER_SIZE;
		cc_current /= H_BUFFER_SIZE;

		for(uint16_t c = i; c < H_BUFFER_SIZE; c++){
				rms_voltage += (adc1_voltage[c] - cc_voltage) * (adc1_voltage[c] - cc_voltage);
				rms_current += (adc2_current[c] - cc_current) * (adc2_current[c] - cc_current);
				pot_ativa += ((adc2_current[c] - cc_current) * (adc1_voltage[c] - cc_voltage));
		}

		rms_voltage = sqrtf(rms_voltage/H_BUFFER_SIZE);
		rms_current = sqrtf(rms_current/H_BUFFER_SIZE);

		pot_aparente = (rms_voltage * rms_current);
		pot_reativa = (pot_aparente * pot_aparente)-(pot_ativa * pot_ativa);
		pf = pot_ativa/pot_aparente;

  }
  /* USER CODE END StartAdcTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
