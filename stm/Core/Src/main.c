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
#include "queue.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

 typedef enum UART_MACHINE_STATES
{
  UMS_RECEIVING,
  UMS_PROCESSING_RESPONSE_PACKAGE,
  UMS_SENDING_RESPONSE,
  UMS_TIMEOUT						//sem rotina por enqnt...
}UART_MACHINE_STATES;

 typedef enum UART_PACKAGE_PARTS
{
  UPP_STX,
  UPP_DEVICE_ADDRESS,
  UPP_OPCODE,
  UPP_DATA_LEN,
  UPP_DATA,
  UPP_CHECKSUM,
  UPP_ETX
}UART_PACKAGE_PARTS;


 typedef enum ERRORS_LISTtag
 {
     EL_NO_ERROR = 0x00,
     EL_INVALID_OPCODE = 0x01,
     EL_INVALID_DATA = 0x02,
     EL_MEMORY_WRITE_ERROR = 0x03,

     EL_NUM_ERRORS
 }ERRORS_LIST;


// Pergunta
//    0x10 a 0x7F
// Resposta
//    0x90 a 0xFF

typedef enum UART_OPCODES
{
	// Busca de dados e informações 0x10 a 0x1F
	UO_KEEPALIVE		= 0x10,
	UO_GET_DATA 		= 0x11,


	// Ações de configuração e ajustes 0x20 a 0x3F
	UO_SETCONFIG		= 0x20,

	// Operação ou outro 0x40 a 0x5F

	UO_NUM_OF

}UART_OPCODES;


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



#define STX 0x02
#define ETX 0x03
#define ESC 0x10
#define ESC_INC 0x20
#define DEVICE_ADDR  0x01

#define RESPONSE_OPCODE_MASK 0x80

#define MAX_DATA_LEN 	122
#define MAX_PACKAGE_LEN 128
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
/* Definitions for rxuartqueue */
osMessageQueueId_t rxuartqueueHandle;
const osMessageQueueAttr_t rxuartqueue_attributes = {
  .name = "rxuartqueue"
};
/* Definitions for energyqueue */
osMessageQueueId_t energyqueueHandle;
const osMessageQueueAttr_t energyqueue_attributes = {
  .name = "energyqueue"
};
/* Definitions for uartBinSema */
osSemaphoreId_t uartBinSemaHandle;
const osSemaphoreAttr_t uartBinSema_attributes = {
  .name = "uartBinSema"
};
/* USER CODE BEGIN PV */

typedef struct UART_PACKAGE_PROTOCOL
{
  unsigned char uc_Stx;
  unsigned char uc_DeviceAddress;
  unsigned char uc_OpCode;
  unsigned char uc_Datalen;
  unsigned char uc_Data[MAX_DATA_LEN];
  unsigned char uc_Checksum;
  unsigned char uc_Etx;
}UART_PACKAGE_PROTOCOL;

typedef struct ENERGY_DATA
{
	int32_t rms_current;
	int32_t rms_voltage;
	uint32_t pot_aparente;
	uint32_t pot_reativa;
	int32_t  pot_ativa;
	uint16_t pf;
	uint64_t consumption;
}ENERGY_DATA;

 int32_t pot_ativa1 = 0;
 uint32_t pot_aparente1 = 0;
 uint32_t pot_reativa1 = 0;
 int32_t rms_voltage1 = 0;
 int32_t rms_current1 = 0;
 uint16_t pf1 = 0;
 uint64_t consumption1 = 0;

uint32_t 	adcBuffer[F_BUFFER_SIZE];
uint16_t 	adc1_voltage[H_BUFFER_SIZE];
uint16_t 	adc2_current[H_BUFFER_SIZE];

UART_MACHINE_STATES m_udtUartmachineStates;
UART_PACKAGE_PARTS  m_udtUartPackageParts;
UART_PACKAGE_PROTOCOL m_udtReceptionPackage;
UART_PACKAGE_PROTOCOL m_udtTransmitionPackage;

// Dado que está sendo processado no pacote
unsigned char m_ucCorrentDataPos;

// Dado que está sendo processado no Buffer de Transmissao
unsigned char m_ucTXBufferCorrentDataPos = 0;

// CheckSum Calculado
unsigned char m_ucCalculatedChecksum;

// Indica que está tratando um scape char
uint8_t m_blnProcessingScapeChar;

// Indica que está na hora de responder
uint8_t m_blnReply = 0;

uint8_t rx_buffer;
uint8_t tx_buffer[MAX_PACKAGE_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_UART4_Init(void);
void StartUartTask(void *argument);
void StartAdcTask(void *argument);

/* USER CODE BEGIN PFP */

/**
* void ResetSerial()
*
* Rotina responsável por reinicializar o protocolo serial para aguardar um novo pacote.
*
* @author Vinicius Ludwig
*/
void ResetSerial();

/**
* unsigned char CalculateChecksum(unsigned char* udtpackage, unsigned char ucLen)
*
* Calcula o checksum de um pacote.
*
* @author Vinicius Ludwig
*/
unsigned char CalculateChecksum(unsigned char* udtpackage, unsigned char ucLen);

/**
* bool SendData(unsigned char ucDataTosend, bool blnIsSpecialChar)
*
* Valida e envia um dado via serial
*
* @author Vinicius Ludwig
*/
uint8_t SendData(unsigned char ucDataTosend, uint8_t blnIsSpecialChar);

/**
* void UartMainProcess(unsigned char ucData)
*
* Chamado do processamento da serial na task da uart.
*
* @author Vinicius Ludwig
*/
void UartMainProcess(unsigned char ucData);

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
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_UART4_Init();
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

  /* creation of rxuartqueue */
  rxuartqueueHandle = osMessageQueueNew (128, sizeof(uint8_t), &rxuartqueue_attributes);

  /* creation of energyqueue */
  energyqueueHandle = osMessageQueueNew (1, sizeof(ENERGY_DATA), &energyqueue_attributes);

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


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------  UART --------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
* void ResetSerial()
*
* Rotina responsável por reinicializar o protocolo serial para aguardar um novo pacote.
*
* @author Vinicius Ludwig
*/
void ResetSerial()
{
  // Inicialmente estamos aguardando a recepção dos dados do dispositivo remoto.
  m_udtUartmachineStates = UMS_RECEIVING;

  // Inicialmente estamos aguardando o STX
  m_udtUartPackageParts = UPP_STX;

  // Cria os ponteiros para os pacotes
  unsigned char* pucReceptionpackage = &m_udtReceptionPackage.uc_Stx;
  unsigned char* pucTranmitionpackage = &m_udtTransmitionPackage.uc_Stx;

  unsigned char ucPosition = 0;

  // Prepara a varredura
  while(ucPosition <  sizeof(UART_PACKAGE_PROTOCOL))
  {
      *pucReceptionpackage = 0x00;
      *pucTranmitionpackage = 0x00;
      pucReceptionpackage++;
      pucTranmitionpackage++;

     ucPosition++;
  }

  // Inicializa a posição a ser processada como 0.
  m_ucCorrentDataPos = 0x00;

  // Inicializa o Checksum calculado com 0
  m_ucCalculatedChecksum = 0;

}


/**
* void UartMainProcess()
*
* Chamado do processamento da serial no loop principal.
*
* @author Vinicius Ludwig
*/
void UartMainProcess(unsigned char ucData)
{
  switch (m_udtUartmachineStates)
  {
    case UMS_RECEIVING:
    {

//      // Verifica se existem dados para ler
//      if(Serial2.available() == 0)
//      {
//         // Sem dados para ler.
//         ///////////////////////
//
//         // Cai fora
//         break;
//       }

      // recebe o dado da queue


      /////////////////////////////////////////////////
      // Processamento dos caracteres especiais
      /////////////////////////////////////////////////

      // Verifica se é um inicializador de pacotes
      if( ucData == STX )
      {
          // Inicializador de pacotes.
          //////////////////////////////

          // Reseta a serial;
          ResetSerial();

          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Stx = STX;

          // Vai para o próximo estado
          m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DEVICE_ADDRESS;

          // Cai fora.
          break;
      }

      // Verifica se é um terminador de pacotes
      if( ucData == ETX )
      {
        // Terminador de pacotes.
        //////////////////////////////

        // Verifica se está na hora de receber esse dado.
        if( m_udtUartPackageParts != (UART_PACKAGE_PARTS)UPP_ETX )
        {
          // Dado incorreto.
          //////////////////

          // Reseta a serial;
          ResetSerial();

          // Cai fora.
          break;
        }

        m_udtReceptionPackage.uc_Etx = ETX;

        // Vai para o próximo estado
        m_udtUartmachineStates = UMS_PROCESSING_RESPONSE_PACKAGE;
        m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_STX;

        // Cai fora
        break;
      }

      // Verifica se é um scape char e se esse deve ser tratado
      if(ucData == ESC)
      {
        m_blnProcessingScapeChar = 1;

        break;
      }

      // Verifica se está no dado pós scape char
      if(m_blnProcessingScapeChar == 1)
      {
        // Dado pós scape char
        ///////////////////////

        // Processa o dado
        ucData = ucData & ~ESC_INC;

        // Indica que já tratou
        m_blnProcessingScapeChar = 0;
      }

      switch (m_udtUartPackageParts)
      {
        case UPP_DEVICE_ADDRESS:
        {
          // Verifica se o dado recebido é o correto.
          if( ucData == DEVICE_ADDR )
          {
              // Dado correto.
              //////////////////

              // Guarda o dado recebido
              m_udtReceptionPackage.uc_DeviceAddress = ucData;

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_OPCODE;
          }
        }
        break;
        case UPP_OPCODE:
        {
          // Guarda o dado recebido
          m_udtReceptionPackage.uc_OpCode = ucData;

          // Vai para o próximo estado
          m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA_LEN;

        }
        break;
        case UPP_DATA_LEN:
        {
          // Verifica se o dado recebido é o correto.
          if( ucData >= MAX_DATA_LEN )
          {
              // OpCode inválido.
              //////////////////

              // Reseta a serial;
              ResetSerial();

              // Cai fora.
              break;
          }

          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Datalen = ucData;

          // Verifica se existirão dados
          if (m_udtReceptionPackage.uc_Datalen > 0)
          {
              // Existirão dados.
              // /////////////////

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA;
          }
          else
          {
              // Não existirão dados.
              // /////////////////////

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
          }


        }
        break;
        case UPP_DATA:
        {
          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Data[m_ucCorrentDataPos] = ucData;

          // Incrementa a posição
          m_ucCorrentDataPos++;

          // Verifica se atingiu o número de dados
          if(m_ucCorrentDataPos >= m_udtReceptionPackage.uc_Datalen)
          {
            // Chegou ao fim dos dados
            /////////////////////////////

            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
          }
        }
        break;
        case UPP_CHECKSUM:
        {
          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Checksum = ucData;

          // Calcula o checksum do pacote
          m_ucCalculatedChecksum = CalculateChecksum(&m_udtReceptionPackage.uc_Stx, (1 + 1 + 1 + 1 + m_udtReceptionPackage.uc_Datalen +1));

          // Verifica se o checksum bateu
          if(m_udtReceptionPackage.uc_Checksum != m_ucCalculatedChecksum)
          {
            // Não bateu
            /////////////

            // Reseta a serial;
            ResetSerial();

            // Cai fora
            break;
          }

          // Vai para o próximo estado
          m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_ETX;
        }
        break;
      }
    }
    break;
    case UMS_PROCESSING_RESPONSE_PACKAGE:
    {
      // Prepara os dados fixos do pacote
      m_udtTransmitionPackage.uc_Stx = STX;
      m_udtTransmitionPackage.uc_Etx = ETX;
      m_udtTransmitionPackage.uc_DeviceAddress = 0x01;
      m_udtTransmitionPackage.uc_OpCode = RESPONSE_OPCODE_MASK | m_udtReceptionPackage.uc_OpCode;


      switch( m_udtReceptionPackage.uc_OpCode )
      {
         case UO_KEEPALIVE:
         {
            // Prepara o pacote de resposta
            m_udtTransmitionPackage.uc_Datalen = 0x00;

         }
         break;
         case UO_GET_DATA:
                 {
					ENERGY_DATA m_udtEnergyDataPacket;

					xQueueReceive(energyqueueHandle, &m_udtEnergyDataPacket, portMAX_DELAY);

					// Prepara o pacote de resposta
					m_udtTransmitionPackage.uc_Datalen = 0x1A;


					m_udtTransmitionPackage.uc_Data[0] = (uint8_t)((m_udtEnergyDataPacket.rms_voltage & 0x0000FF00) >> 8);
					m_udtTransmitionPackage.uc_Data[1] = (uint8_t)((m_udtEnergyDataPacket.rms_voltage & 0x000000FF));

					m_udtTransmitionPackage.uc_Data[2] = (uint8_t)((m_udtEnergyDataPacket.rms_current & 0x0000FF00) >> 8);
					m_udtTransmitionPackage.uc_Data[3] = (uint8_t)((m_udtEnergyDataPacket.rms_current & 0x000000FF));

					m_udtTransmitionPackage.uc_Data[4] = (uint8_t)((m_udtEnergyDataPacket.pf & 0xFF00) >> 8);
					m_udtTransmitionPackage.uc_Data[5] = (uint8_t)((m_udtEnergyDataPacket.pf & 0x00FF));

					m_udtTransmitionPackage.uc_Data[6] = (uint8_t)((m_udtEnergyDataPacket.pot_aparente & 0xFF000000) >> 24);
					m_udtTransmitionPackage.uc_Data[7] = (uint8_t)((m_udtEnergyDataPacket.pot_aparente & 0x00FF0000) >> 16);
					m_udtTransmitionPackage.uc_Data[8] = (uint8_t)((m_udtEnergyDataPacket.pot_aparente & 0x0000FF00) >> 8);
					m_udtTransmitionPackage.uc_Data[9] = (uint8_t)((m_udtEnergyDataPacket.pot_aparente & 0x000000FF));

					m_udtTransmitionPackage.uc_Data[10] = (uint8_t)((m_udtEnergyDataPacket.pot_ativa & 0xFF000000) >> 24);
					m_udtTransmitionPackage.uc_Data[11] = (uint8_t)((m_udtEnergyDataPacket.pot_ativa & 0x00FF0000) >> 16);
					m_udtTransmitionPackage.uc_Data[12] = (uint8_t)((m_udtEnergyDataPacket.pot_ativa & 0x0000FF00) >> 8);
					m_udtTransmitionPackage.uc_Data[13] = (uint8_t)((m_udtEnergyDataPacket.pot_ativa & 0x000000FF));

					m_udtTransmitionPackage.uc_Data[14] = (uint8_t)((m_udtEnergyDataPacket.pot_reativa & 0xFF000000) >> 24);
					m_udtTransmitionPackage.uc_Data[15] = (uint8_t)((m_udtEnergyDataPacket.pot_reativa & 0x00FF0000) >> 16);
					m_udtTransmitionPackage.uc_Data[16] = (uint8_t)((m_udtEnergyDataPacket.pot_reativa & 0x0000FF00) >> 8);
					m_udtTransmitionPackage.uc_Data[17] = (uint8_t)((m_udtEnergyDataPacket.pot_reativa & 0x000000FF));

					m_udtTransmitionPackage.uc_Data[18] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0xFF00000000000000) >> 56);
					m_udtTransmitionPackage.uc_Data[19] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x00FF000000000000) >> 48);
					m_udtTransmitionPackage.uc_Data[20] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x0000FF0000000000) >> 40);
					m_udtTransmitionPackage.uc_Data[21] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x000000FF00000000) >> 32);
					m_udtTransmitionPackage.uc_Data[22] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x00000000FF000000) >> 24);
					m_udtTransmitionPackage.uc_Data[23] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x0000000000FF0000) >> 16);
					m_udtTransmitionPackage.uc_Data[24] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x000000000000FF00) >> 8);
					m_udtTransmitionPackage.uc_Data[25] = (uint8_t)((m_udtEnergyDataPacket.consumption & 0x00000000000000FF));

                 }
                 break;
         case UO_SETCONFIG:
         		 {
         			 ERRORS_LIST udtError = EL_NO_ERROR;
         			 uint8_t blnStatus = 0;

         			 // Verifica se o tamanho do pacote condiz com o esperado
         			 if (m_udtReceptionPackage.uc_Datalen == 4)
         			 {
         				 // Era a opcao 1
         				 //////////////////////////////////


         			 }
         			 else if (m_udtReceptionPackage.uc_Datalen == 0)
         			 {
         				 // Ou a opcao 2
         				 ///////////////////////////////

         				// Executa alguma funcao que retorna booleano de validação
         				//blnStatus = SaveParameters(bytLocation);
         			 }
         			 else
         			 {
         				 //Erro, dados inválidos
         				 /////////////////////////

         				 // Indica o erro
         				 udtError = EL_INVALID_DATA;
         			 }

         			 //Verifica se já veio com erro
         			 if (udtError == EL_NO_ERROR)
         			 {
         				 // Chegou sem erros
         				 ////////////////////

         				 // Verifica se deu algum erro de escrita
         				 if (blnStatus == 0)
         				 {
         					 // Deu erro de escrita
         					 /////////////////////////

         					 // Guarda o erro.
         					 udtError = EL_MEMORY_WRITE_ERROR;
         				 }
         			 }


         			// Prepara os dados que serão enviados (resposta de valdiação)
         			m_udtTransmitionPackage.uc_Datalen = 0x01;	//tamanho do pacote de resposta
         			m_udtTransmitionPackage.uc_Data[0] = (uint8_t)udtError & 0xFF;

         		 }
         		 break;



		 default:
		 {
			 // Retorna um erro
			 ///////////////////

			 // Força o OPCODE de erro

			 m_udtTransmitionPackage.uc_OpCode = RESPONSE_OPCODE_MASK | UO_SETCONFIG;
			m_udtTransmitionPackage.uc_Datalen = 0x01;
			m_udtTransmitionPackage.uc_Data[0] = EL_INVALID_OPCODE;
		 }
      }

      // Calcula o Checksum do pacote a ser enviado
      m_udtTransmitionPackage.uc_Checksum = CalculateChecksum(&m_udtTransmitionPackage.uc_Stx, (1 + 1 + 1 + 1 + m_udtTransmitionPackage.uc_Datalen + 1));

      // Vai para o próximo estado
      m_udtUartmachineStates = UMS_SENDING_RESPONSE;
      m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_STX;
    }
    break;
    case UMS_SENDING_RESPONSE:
    {
      //Aqui poderia verificar se a serial está disponivel para responder
//      if( Serial2.availableForWrite() == 0 )
//      {
//        // Não pode transmitir.
//        /////////////////////////
//
//        // Cai fora.
//        break;
//      }

      switch (m_udtUartPackageParts)
      {
        case UPP_STX:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Stx, 1) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DEVICE_ADDRESS;
          }
        }
        break;
        case UPP_DEVICE_ADDRESS:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_DeviceAddress, 0) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_OPCODE;
          }
        }
        break;
        case UPP_OPCODE:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_OpCode, 0) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA_LEN;
          }
        }
        break;
        case UPP_DATA_LEN:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Datalen, 0) == 1)
          {
            // Zera a posição
            m_ucCorrentDataPos = 0x00;

            // Verifica se existirão dados
            if (m_udtTransmitionPackage.uc_Datalen > 0)
            {
                // Existirão dados.
                // /////////////////

                // Vai para o próximo estado
                m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA;
            }
            else
            {
                // Não existirão dados.
                ////////////////////////

                // Vai para o próximo estado
                m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
            }
          }
        }
        break;
        case UPP_DATA:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Data[m_ucCorrentDataPos], 0) == 1)
          {
            // Incrementa a posição
            m_ucCorrentDataPos++;

            // Verifica se atingiu o número de dados
            if(m_ucCorrentDataPos >= m_udtTransmitionPackage.uc_Datalen)
            {
              // Chegou ao fim dos dados
              /////////////////////////////

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
            }
          }

        }
        break;
        case UPP_CHECKSUM:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Checksum, 0) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_ETX;
          }
        }
        break;
        case UPP_ETX:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Etx, 1) == 1)
          {
        	 //Indica para a task que pode enviar a resposta
        	 m_blnReply = 1;

        	 ResetSerial();
          }
        }
        break;
      }
    }
    break;
    case UMS_TIMEOUT:
    {
      // Timeout
      ////////////

      // Reseta a serial;
      ResetSerial();
    }
    break;
  }

}


/**
* bool SendData(unsigned char ucDataTosend, bool blnIsSpecialChar)
*
* Valida e envia um dado via serial
*
* @author Vinicius Ludwig
*/
uint8_t SendData(unsigned char ucDataTosend, uint8_t blnIsSpecialChar)
{

  // Cria a variável de retorno indicando que foi um dado normal
  // false indica um scape char e não deve ir para o próximo
  uint8_t blnReturnValue = 1;

  // Verifica se é um caractere especial.
  if( blnIsSpecialChar == 0 )
  {
    // Não é um caractere especial,
    // devemos tratar
    ///////////////////////////////

    // Verifica se processou um caractere especial na última passada.
    if( m_blnProcessingScapeChar == 1 )
    {
      // Sinalizou um caractere igual a um
      // especial na última passada.
      ///////////////////////////////////////

      // Altera o dado
      ucDataTosend = ucDataTosend | ESC_INC;

      // Indica que já processou.
      m_blnProcessingScapeChar = 0;
    }
    else
    {
      // Não foi um igual a especial na última passada.
      ////////////////////////////////////////////////////

      // Verifica se é item igual a um especial
      if(
          (ucDataTosend == STX)
          ||(ucDataTosend == ETX)
          ||(ucDataTosend == ESC))
      {
        // É um especial
        /////////////////

        // Eviou um scape char, não deve avançar
        blnReturnValue = 0;

        // Indica que enviou um especial
        m_blnProcessingScapeChar = 1;

        // altera o dado
        ucDataTosend = ESC;
      }
    }
  }

  tx_buffer[m_ucTXBufferCorrentDataPos] = ucDataTosend;
  m_ucTXBufferCorrentDataPos++;

  //Serial2.write(ucDataTosend);

  return blnReturnValue;
}


/**
* unsigned char CalculateChecksum(unsigned char* udtpackage, unsigned char ucLen)
*
* Calcula o checksum de um pacote.
*
* @author Vinicius Ludwig
*/
unsigned char CalculateChecksum(unsigned char* udtpackage, unsigned char ucLen)
{
  // Cria a inicializa a variável de retorno.
  unsigned char ucChecksum = 0;

  // Cria a variável do controle de posição
  unsigned char ucPosition = 0;

  // Prepara a varredura
  while(ucPosition < ucLen)
  {
    //Varre os dados.
    //////////////////

    // Soma  o valor da vez.
    ucChecksum = ucChecksum + *udtpackage;

    // Atualiza os indices
    udtpackage++;
    ucPosition++;

  }

  // Retorna a informação
  return ucChecksum;
}





////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------- CALLBACKS ------------------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	xQueueSendFromISR(rxuartqueueHandle, &rx_buffer, &pxHigherPriorityTaskWoken);

	HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_buffer, 1);

	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	for(uint8_t i=0; i < m_ucTXBufferCorrentDataPos; i++)
	{
		tx_buffer[i] = 0x00;
	}

	m_ucTXBufferCorrentDataPos = 0;

	xSemaphoreGiveFromISR(uartBinSemaHandle, &pxHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}


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
	uint8_t receivedByte;
	uint8_t null = 0;

	HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_buffer, 1);

  /* Infinite loop */
  while(1)
  {
	  if(m_blnReply == 0)
	  {
		  if((m_udtUartmachineStates == UMS_RECEIVING)){
			// Se houver dados recebidos na fila
			if (xQueueReceive(rxuartqueueHandle, &receivedByte, portMAX_DELAY)) {
				// Processa o byte recebido
				UartMainProcess(receivedByte);
			}
		  }
		  else if ((m_udtUartmachineStates == UMS_PROCESSING_RESPONSE_PACKAGE))
		  {
			  UartMainProcess(null);
		  }
		  else if ((m_udtUartmachineStates == UMS_SENDING_RESPONSE))
		  {
			UartMainProcess(null);
		  }
	  }
	  else
	  {
		  HAL_UART_Transmit_IT(&huart4, (uint8_t *)&tx_buffer, m_ucTXBufferCorrentDataPos);
		  xSemaphoreTake(uartBinSemaHandle, portMAX_DELAY);

		  m_blnReply = 0;
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
	uint32_t accumulated_active_power = 0;
	uint16_t cycle_count = 0;

	uint32_t cc_voltage = 0;
	uint32_t cc_current = 0;
	uint8_t sidebuffer_choice = 0;
	uint16_t i = 0;
	uint16_t j = 0;

	ENERGY_DATA m_udtEnergyDataCalcs;
	m_udtEnergyDataCalcs.consumption = 0;

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcBuffer, F_BUFFER_SIZE);
	HAL_TIM_Base_Start(&htim1);


  /* Infinite loop */
  while(1)
  {
		xQueueReceive(adchalfselectQueueHandle, &sidebuffer_choice, portMAX_DELAY);

		cc_voltage = 0;
		cc_current = 0;
		j = 0;

		m_udtEnergyDataCalcs.pot_ativa = 0;
		m_udtEnergyDataCalcs.pot_aparente = 0;
		m_udtEnergyDataCalcs.pot_reativa = 0;
		m_udtEnergyDataCalcs.rms_voltage = 0;
		m_udtEnergyDataCalcs.rms_current = 0;
		m_udtEnergyDataCalcs.pf = 0;

		if (sidebuffer_choice == 1){
			i = 0;
		}
		if (sidebuffer_choice == 2){
			i = H_BUFFER_SIZE;
		}

		for(uint16_t c = i; c < (H_BUFFER_SIZE + i); c++){
				// Extrai os 16 bits menos significativos
				adc1_voltage[j] = (((uint32_t)(adcBuffer[c] & 0x0000FFFF)) * V1_SENSOR_MULT * V1_REAL_MULT);

				cc_voltage += adc1_voltage[j];

				// Extrai os 16 bits mais significativos
				adc2_current[j] = (((uint32_t)((adcBuffer[c] >> 16) & 0x0000FFFF)) * C2_SENSOR_MULT * C2_REAL_MULT);

				cc_current += adc2_current[j];

				j++;
		}

		cc_voltage /= H_BUFFER_SIZE;
		cc_current /= H_BUFFER_SIZE;

		for(uint16_t c = i; c < H_BUFFER_SIZE; c++){
			m_udtEnergyDataCalcs.rms_voltage += (int32_t)((adc1_voltage[c] - cc_voltage) * (adc1_voltage[c] - cc_voltage));
			m_udtEnergyDataCalcs.rms_current += (int32_t)((adc2_current[c] - cc_current) * (adc2_current[c] - cc_current));
			m_udtEnergyDataCalcs.pot_ativa += (int32_t)((adc2_current[c] - cc_current) * (adc1_voltage[c] - cc_voltage));
		}

		m_udtEnergyDataCalcs.pot_ativa = (m_udtEnergyDataCalcs.pot_ativa / H_BUFFER_SIZE)*100;

		m_udtEnergyDataCalcs.rms_voltage = (sqrtf((uint32_t)(m_udtEnergyDataCalcs.rms_voltage/H_BUFFER_SIZE)))*10;
		m_udtEnergyDataCalcs.rms_current = (sqrtf((uint32_t)(m_udtEnergyDataCalcs.rms_current/H_BUFFER_SIZE)))*100;

		if ((m_udtEnergyDataCalcs.rms_voltage * m_udtEnergyDataCalcs.rms_current) > 0)
	    {
			m_udtEnergyDataCalcs.pot_aparente = (uint32_t)((m_udtEnergyDataCalcs.rms_voltage * m_udtEnergyDataCalcs.rms_current))/10;
	    }

		if ((m_udtEnergyDataCalcs.pot_aparente > 0) && (m_udtEnergyDataCalcs.pot_ativa > 0))
		{
			m_udtEnergyDataCalcs.pot_ativa = m_udtEnergyDataCalcs.pot_ativa/10;
			m_udtEnergyDataCalcs.pot_aparente = m_udtEnergyDataCalcs.pot_aparente/10;

			if ((m_udtEnergyDataCalcs.pot_aparente > 0) && (m_udtEnergyDataCalcs.pot_ativa > 0))
			{
				m_udtEnergyDataCalcs.pot_reativa = ((m_udtEnergyDataCalcs.pot_aparente * m_udtEnergyDataCalcs.pot_aparente) / 10)-((m_udtEnergyDataCalcs.pot_ativa * m_udtEnergyDataCalcs.pot_ativa) / 10);
			}

		}

		if(m_udtEnergyDataCalcs.pot_reativa > 0)
	    {
			m_udtEnergyDataCalcs.pot_reativa = sqrtf((uint32_t)m_udtEnergyDataCalcs.pot_reativa);
		}

		if ((m_udtEnergyDataCalcs.pot_ativa > 0) && (m_udtEnergyDataCalcs.pot_aparente > 0))
	    {
			m_udtEnergyDataCalcs.pf = (m_udtEnergyDataCalcs.pot_ativa*1000)/m_udtEnergyDataCalcs.pot_aparente;
	    }

		accumulated_active_power += (m_udtEnergyDataCalcs.pot_ativa);
		cycle_count++;

		if (cycle_count >= 60)
		{

			m_udtEnergyDataCalcs.consumption += accumulated_active_power/3600;

			accumulated_active_power = 0;
			cycle_count = 0;
		}

		pot_ativa1 = m_udtEnergyDataCalcs.pot_ativa;
		pot_aparente1 = m_udtEnergyDataCalcs.pot_aparente;
		pot_reativa1 = m_udtEnergyDataCalcs.pot_reativa;
		rms_voltage1 = m_udtEnergyDataCalcs.rms_voltage;
		rms_current1 = m_udtEnergyDataCalcs.rms_current;
		pf1 = m_udtEnergyDataCalcs.pf;
		consumption1 = m_udtEnergyDataCalcs.consumption;




		if (uxQueueMessagesWaiting(energyqueueHandle) == 1) {
			xQueueOverwrite(energyqueueHandle, &m_udtEnergyDataCalcs);
		}
		else
		{
			//alimenta queue
			xQueueSend(energyqueueHandle, &m_udtEnergyDataCalcs, 0);
		}

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
