/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Modbus.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    osMutexId_t Sensor_data;             // Handle for the unique mutex lock
    volatile uint16_t Sensor_value;       // Processed Sensor value
} SensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for ADC_task */
osThreadId_t ADC_taskHandle;
const osThreadAttr_t ADC_task_attributes = {
  .name = "ADC_task",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Calibration_tas */
osThreadId_t Calibration_tasHandle;
const osThreadAttr_t Calibration_tas_attributes = {
  .name = "Calibration_tas",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Watch_dog_task */
osThreadId_t Watch_dog_taskHandle;
const osThreadAttr_t Watch_dog_task_attributes = {
  .name = "Watch_dog_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Config_monitor_ */
osThreadId_t Config_monitor_Handle;
const osThreadAttr_t Config_monitor__attributes = {
  .name = "Config_monitor_",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Display_task */
osThreadId_t Display_taskHandle;
const osThreadAttr_t Display_task_attributes = {
  .name = "Display_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC_Queue */
osMessageQueueId_t ADC_QueueHandle;
const osMessageQueueAttr_t ADC_Queue_attributes = {
  .name = "ADC_Queue"
};
/* Definitions for Sensor_data */
osMutexId_t Sensor_dataHandle;
const osMutexAttr_t Sensor_data_attributes = {
  .name = "Sensor_data"
};
/* USER CODE BEGIN PV */
#define FLASH_MAGIC_KEY   0x5A42
#define HOLDING_REG_COUNT 4
#define INPUT_REG_COUNT   3

typedef struct {
    uint16_t magic;
    uint8_t slave_id;
    uint32_t baud_rate;
    uint32_t parity; 
    uint32_t stop_bits;
} DeviceConfig_t;

/* Ensure the linker script isolates this section to a specific page boundary */
__attribute__((section(".data_flash"), aligned(2048))) DeviceConfig_t Flash_Stored_Config;

/* Modbus Infrastructure */
modbusHandler_t ModbusH;
DeviceConfig_t live_cfg;

/* Separate structures matching clean industrial definitions */
uint16_t HoldingDATA[HOLDING_REG_COUNT]; // RW Configs: Status, Cal, Modbus Addr
uint16_t InputDATA[INPUT_REG_COUNT];     // RO Variables: PPM, RAW ADC

// Holding Register Offsets (FC03 / FC06)
#define REG_STATUS       0 // Address 40001
#define REG_CAL_CMD      1 // Address 40002
#define REG_MB_ADDR      2 // Address 40003

// Input Register Offsets (FC04)
#define REG_PPM_X10      0 // Address 30001
#define REG_ADC_RAW      1 // Address 30002


#define ADC_BLOCK_LEN  50
#define ADC_BUF_LEN    (ADC_BLOCK_LEN * 2) // 100 elements
volatile uint16_t adc_raw_buffer[ADC_BUF_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
void Start_ADC_task(void *argument);
void Start_Calibration_task(void *argument);
void Start_Watch_dog_task(void *argument);
void Start_Config_monitor_task(void *argument);
void Start_Display_task(void *argument);

/* USER CODE BEGIN PFP */
void Read_Device_Configuration(void);
uint16_t Read_Physical_DIP_Switches(void);
void Program_Flash_Config(uint8_t id, uint32_t baud, uint32_t parity);

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
// FIX: Read configuration matrix BEFORE re-initializing the UART peripheral frame
  Read_Device_Configuration();

  /* Re-initialize USART1 with dynamic parameters */
  huart1.Instance        = USART1;
  huart1.Init.BaudRate   = live_cfg.baud_rate;
  huart1.Init.Parity     = live_cfg.parity;
  huart1.Init.WordLength = (live_cfg.parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
  huart1.Init.StopBits   = live_cfg.stop_bits;
  huart1.Init.Mode       = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

/* Bind and allocate Modbus Slave Engine with distinct spaces */
  ModbusH.uModbusType            = MB_SLAVE;
  ModbusH.port                   = &huart1; 
  ModbusH.u8id                   = live_cfg.slave_id;
  ModbusH.u16timeOut             = 1000;
  ModbusH.xTypeHW                = USART_HW_DMA; 
  
  // RS485 Toggling
  ModbusH.EN_Port                = GPIOA;
  ModbusH.EN_Pin                 = GPIO_PIN_8; 
  
  ModbusH.u16regs                = NULL; // Unused when separate areas are active
  ModbusH.u16regsize             = 0;
  
  // Link Read-Write Holding Region (Addresses starting at 0)
  ModbusH.u16holdingRegs         = HoldingDATA;
  ModbusH.u16holdingRegsStartAdd = 0;
  ModbusH.u16holdingRegsNregs    = HOLDING_REG_COUNT;
  
  // Link Read-Only Input Region (Addresses starting at 0)
  ModbusH.u16inputRegs           = InputDATA;
  ModbusH.u16inputRegsStartAdd   = 0;
  ModbusH.u16inputRegsNregs      = INPUT_REG_COUNT;
  
  ModbusInit(&ModbusH);
  ModbusStart(&ModbusH);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Sensor_data */
  Sensor_dataHandle = osMutexNew(&Sensor_data_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ADC_Queue */
  ADC_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &ADC_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ADC_task */
  ADC_taskHandle = osThreadNew(Start_ADC_task, NULL, &ADC_task_attributes);

  /* creation of Calibration_tas */
  Calibration_tasHandle = osThreadNew(Start_Calibration_task, NULL, &Calibration_tas_attributes);

  /* creation of Watch_dog_task */
  Watch_dog_taskHandle = osThreadNew(Start_Watch_dog_task, NULL, &Watch_dog_task_attributes);

  /* creation of Config_monitor_ */
  Config_monitor_Handle = osThreadNew(Start_Config_monitor_task, NULL, &Config_monitor__attributes);

  /* creation of Display_task */
  Display_taskHandle = osThreadNew(Start_Display_task, NULL, &Display_task_attributes);

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
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 781;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4095;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, TM1637_CLK_Pin|TM1637_DIO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : TM1637_CLK_Pin TM1637_DIO_Pin */
  GPIO_InitStruct.Pin = TM1637_CLK_Pin|TM1637_DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Read_Device_Configuration(void)
{
    // Check Pin PA0 (DIP Switch 1 - Configuration Mode)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) 
    {
        /* ─── SOFTWARE MODE (Read Flash) ─── */
        if (Flash_Stored_Config.magic == FLASH_MAGIC_KEY) 
        {
            live_cfg.slave_id  = Flash_Stored_Config.slave_id;
            live_cfg.baud_rate = Flash_Stored_Config.baud_rate;
            live_cfg.parity    = Flash_Stored_Config.parity;
            live_cfg.stop_bits = Flash_Stored_Config.stop_bits;
        } 
        else 
        {
            /* Fallback Safe Defaults */
            live_cfg.slave_id  = 1;
            live_cfg.baud_rate = 9600;
            live_cfg.parity    = UART_PARITY_NONE;
            live_cfg.stop_bits = UART_STOPBITS_2; 
        }
    } 
    else 
    {
        /* ─── HARDWARE MODE (Read 16-Bit Switch Matrix) ─── */
        uint16_t dip_switches = Read_Physical_DIP_Switches();
        live_cfg.slave_id = ((dip_switches >> 1) & 0xFF) + 1;

        uint8_t baud_selection = (dip_switches >> 9) & 0x07;
        switch (baud_selection) {
            case 0x01: live_cfg.baud_rate = 14400;  break;
            case 0x02: live_cfg.baud_rate = 19200;  break;
            case 0x03: live_cfg.baud_rate = 38400;  break;
            case 0x04: live_cfg.baud_rate = 57600;  break;
            case 0x05: live_cfg.baud_rate = 115200; break;
            default:   live_cfg.baud_rate = 9600;   break;
        }

        uint8_t parity_selection = (dip_switches >> 12) & 0x03;
        if (parity_selection == 0x01) {
            live_cfg.parity = UART_PARITY_EVEN;
            live_cfg.stop_bits = UART_STOPBITS_1;
        } else if (parity_selection == 0x02) {
            live_cfg.parity = UART_PARITY_ODD;
            live_cfg.stop_bits = UART_STOPBITS_1;
        } else {
            live_cfg.parity = UART_PARITY_NONE;
            live_cfg.stop_bits = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) ? 
                                 UART_STOPBITS_1 : UART_STOPBITS_2;
        }
    }
}

uint16_t Read_Physical_DIP_Switches(void)
{
    // Realize your shift registers or GPIO read configurations here.
    // Returning a dummy layout matching your mask definitions for testing:
    return 0x0000; 
}

void Program_Flash_Config(uint8_t id, uint32_t baud, uint32_t parity)
{
    DeviceConfig_t working_packet;
    working_packet.magic     = FLASH_MAGIC_KEY;
    working_packet.slave_id  = id;
    working_packet.baud_rate = baud;
    working_packet.parity    = parity;
    
    if (parity == UART_PARITY_NONE) {
        working_packet.stop_bits = (live_cfg.stop_bits == UART_STOPBITS_1) ? UART_STOPBITS_1 : UART_STOPBITS_2;
    } else {
        working_packet.stop_bits = UART_STOPBITS_1;
    }

    // 1. Stop active UART DMA communication pipelines before tearing down peripherals
    HAL_UART_DMAStop(&huart1);
    
    // 2. Lock Modbus semaphore to safely modify shared registers and hardware frames
    if (osSemaphoreAcquire(ModbusH.ModBusSphrHandle, osWaitForever) == osOK) 
    {
        /* Update runtime configuration context */
        live_cfg.slave_id  = working_packet.slave_id;
        live_cfg.baud_rate = working_packet.baud_rate;
        live_cfg.parity    = working_packet.parity;
        live_cfg.stop_bits = working_packet.stop_bits;

        /* Re‑initialize the UART peripheral with new dynamic parameters */
        huart1.Init.BaudRate   = live_cfg.baud_rate;
        huart1.Init.Parity     = live_cfg.parity;
        huart1.Init.WordLength = (live_cfg.parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
        huart1.Init.StopBits   = live_cfg.stop_bits;
        
        if (HAL_UART_Init(&huart1) != HAL_OK) 
        {
            Error_Handler();
        }

        /* Restart DMA receive engine with idle‑line detection (as used by ModbusStart) */
        // MAX_BUFFER matches the library's buffer boundary configuration
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ModbusH.xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK) 
        {
            Error_Handler();
        }
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // Disable half‑transfer interrupt to prevent unexpected splits

        /* Update Modbus engine identification properties */
        ModbusH.u8id = live_cfg.slave_id;

        /* Reset the ring buffer state and internal metrics counters */
        RingClear(&ModbusH.xBufferRX);
        ModbusH.u8lastRec    = 0;
        ModbusH.u8BufferSize = 0;
        ModbusH.u16InCnt     = 0;
        ModbusH.u16OutCnt    = 0;
        ModbusH.u16errCnt    = 0;

        // Release semaphore resource
        osSemaphoreRelease(ModbusH.ModBusSphrHandle);
    }

    // 3. Write the new configuration structure into Non-Volatile Flash memory
    // CRITICAL: Do NOT disable global interrupts here or use osKernelLock(). 
    // Flash programming takes up to 40ms; halting interrupts triggers an IWDG Watchdog reset.
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = (uint32_t)&Flash_Stored_Config;
    erase_init.NbPages     = 1;

    uint32_t page_error = 0;
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK) 
    {
        uint32_t *source_ptr  = (uint32_t*)&working_packet;
        uint32_t *dest_ptr    = (uint32_t*)&Flash_Stored_Config;
        size_t words_to_write = sizeof(DeviceConfig_t) / 4;
        
        for (size_t i = 0; i < words_to_write; i++) 
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dest_ptr + i), *(source_ptr + i));
        }
    }
    HAL_FLASH_Lock();
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    osThreadFlagsSet(ADC_taskHandle, 0x01); 
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    osThreadFlagsSet(ADC_taskHandle, 0x02); 
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_ADC_task */
/**
  * @brief  Function implementing the ADC_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_ADC_task */
void Start_ADC_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  
  // Start continuous ADC acquisition via hardware timer triggers
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(uintptr_t)adc_raw_buffer, ADC_BUF_LEN);
  HAL_TIM_Base_Start(&htim3);
  uint32_t flags;
  /* Infinite loop */
  for(;;)
  {
  flags = osThreadFlagsWait(0x01 | 0x02, osFlagsWaitAny, osWaitForever);

	  // 2. Determine which offset to read based on the flag set in the ISR
	  uint32_t buffer_offset = 0;
  if (flags & 0x02)
    {
      buffer_offset = ADC_BLOCK_LEN; // Point to index 64 (Second Half)
    }
    else if (flags & 0x01)
    {
      buffer_offset = 0;              // Point to index 0 (First Half)
    }

	  // 3. Compute the average safely from the stable half
	  uint32_t sum = 0;
	  for(int i = 0; i < ADC_BLOCK_LEN; i++)
	  {
		sum += adc_raw_buffer[buffer_offset + i];
	  }
	  
	  uint16_t ADC_average = sum / ADC_BLOCK_LEN;

  osStatus_t queue_status = osMessageQueuePut(ADC_QueueHandle, &ADC_average, 0, 0);
    if (queue_status != osOK)
    {
        /* FIX: Protect shared register from Modbus data races */
        if (osMutexAcquire(Sensor_dataHandle, osWaitForever) == osOK)
        {
            HoldingDATA[REG_STATUS] |= (1 << 4); // Set overflow flag safely
            osMutexRelease(Sensor_dataHandle);
        }
    }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Calibration_task */
/**
* @brief Function implementing the Calibration_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Calibration_task */
void Start_Calibration_task(void *argument)
{
  /* USER CODE BEGIN Start_Calibration_task */
  uint16_t received_ADC_average;
  osStatus_t queue_status;
  /* Infinite loop */
for (;;)
{
  queue_status = osMessageQueueGet(ADC_QueueHandle, & received_ADC_average, NULL, osWaitForever);
  if (queue_status == osOK)
  {
    float ppm = (float) received_ADC_average * 0.75 f;
    // FIX: Secure the actual binary semaphore used by the background Modbus engine thread
    if (osSemaphoreAcquire(ModbusH.ModBusSphrHandle, osWaitForever) == osOK)
    {
      InputDATA[REG_ADC_RAW] = received_ADC_average;
      InputDATA[REG_PPM_X10] = (uint16_t)(ppm * 10.0 f);
      HoldingDATA[REG_STATUS] = 0x01;
      osSemaphoreRelease(ModbusH.ModBusSphrHandle);
    }
   }
  }
  /* USER CODE END Start_Calibration_task */
}

/* USER CODE BEGIN Header_Start_Watch_dog_task */
/**
* @brief Function implementing the Watch_dog_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Watch_dog_task */
void Start_Watch_dog_task(void *argument)
{
  /* USER CODE BEGIN Start_Watch_dog_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);                 // wait 1 second
    HAL_IWDG_Refresh(&hiwdg);      // reset the watchdog counter
  }
  /* USER CODE END Start_Watch_dog_task */
}

/* USER CODE BEGIN Header_Start_Config_monitor_task */
/**
* @brief Function implementing the Config_monitor_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Config_monitor_task */
void Start_Config_monitor_task(void *argument)
{
  /* USER CODE BEGIN Start_Config_monitor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Config_monitor_task */
}

/* USER CODE BEGIN Header_Start_Display_task */
/**
* @brief Function implementing the Display_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Display_task */
void Start_Display_task(void *argument)
{
  /* USER CODE BEGIN Start_Display_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Display_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
