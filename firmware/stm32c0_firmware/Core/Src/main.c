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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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

/* USER CODE BEGIN PV */
// --- Global SysTick Counter Reference ---
extern __IO uint32_t uwTick; 

// --- Gas Sensor (USART1) Definitions ---
#define GAS_RX_BUF_SIZE  32
uint8_t gas_rx_buffer[GAS_RX_BUF_SIZE];
volatile uint8_t gas_rx_index = 0;
volatile uint8_t gas_data_ready = 0;
uint16_t current_gas_value = 0;

// --- Modbus RTU Slave (USART2) Definitions ---
#define MODBUS_BUF_SIZE  128
uint8_t modbus_rx_buffer[MODBUS_BUF_SIZE];
volatile uint16_t modbus_rx_index = 0;
volatile uint32_t modbus_last_rx_tick = 0;
volatile uint8_t modbus_frame_complete = 0;

// --- TM1637 State Machine Definitions ---
typedef enum {
  TM_IDLE,
  TM_START,
  TM_SEND_BYTE,
  TM_ACK,
  TM_STOP
} TM1637_State_t;

TM1637_State_t tm_state = TM_IDLE;
uint8_t tm_bit_index = 0;
volatile uint8_t display_update_required = 0;

// The complete transmission chain sequence array buffer
uint8_t tm_tx_pipeline[8]; 
uint8_t tm_pipeline_index = 0;
uint8_t tm_tx_byte = 0;

const uint8_t TM_FONT_NUM[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
const uint8_t TM_SEG_HEAT[] = {0x76, 0x79, 0x77, 0x38}; // H, E, A, t
const uint8_t TM_SEG_ERR[]  = {0x79, 0x50, 0x50, 0x00}; // E, r, r, [Blank]
const uint8_t TM_SEG_GOOD[] = {0x3D, 0x5C, 0x5C, 0x5E}; // G, o, o, d

// Ring buffer containing the sequenced target states to write
uint8_t tm_display_buffer[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void GasSensor_SendInit(void);
void Process_Gas_Data(void);
void Modbus_Process_State_Machine(void);
uint8_t Modbus_Validate_And_Parse(uint8_t *buffer, uint16_t length);
void TM1637_SetSegments(const uint8_t *segments);
void TM1637_ShowNumber(uint16_t value);
void TM1637_Tick_State_Machine(void);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1. Enable RX-Not-Empty Interrupt for both UARTs
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_RXNE(USART2);
  
  // 2. Ensure RS485 MAX485 is set to Receiver Mode (DE pin LOW)
  LL_GPIO_ResetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin);

  // 3. Make sure TM1637 Open-Drain bus starts passive (HIGH via pull-ups)
  LL_GPIO_SetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin);
  LL_GPIO_SetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin);

  // 4. Fire the initialization packet to the Gas Sensor
  GasSensor_SendInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t startup_timestamp = 0;
    static uint8_t boot_phase = 0;
    
    if (startup_timestamp == 0) startup_timestamp = uwTick;

    // Non-blocking Startup Sequence Manager
    switch (boot_phase) {
      case 0: // Display "HEAt" for the first 3 seconds
        TM1637_SetSegments(TM_SEG_HEAT);
        if (uwTick - startup_timestamp >= 3000) {
          boot_phase = 1;
          startup_timestamp = uwTick;
        }
        break;

      case 1: // Run spinning animation for 3 seconds
        {
          uint32_t elapsed_anim = uwTick - startup_timestamp;
          uint8_t frame = (elapsed_anim / 250) % 4; // Advance frame every 250ms
          
          uint8_t anim_buffer[4] = {0};
          if (frame == 0)      { anim_buffer[0] = 0x01; anim_buffer[1] = 0x01; anim_buffer[2] = 0x01; anim_buffer[3] = 0x01; }
          else if (frame == 1) { anim_buffer[3] = 0x06; }
          else if (frame == 2) { anim_buffer[0] = 0x08; anim_buffer[1] = 0x08; anim_buffer[2] = 0x08; anim_buffer[3] = 0x08; }
          else                 { anim_buffer[0] = 0x30; }
          
          TM1637_SetSegments(anim_buffer);
          
          if (elapsed_anim >= 3000) {
            boot_phase = 2;
            startup_timestamp = uwTick;
          }
        }
        break;

      case 2: // Display static "Good" for 3 seconds
        TM1637_SetSegments(TM_SEG_GOOD);
        if (uwTick - startup_timestamp >= 3000) {
          boot_phase = 3; // Warm-up complete
        }
        break;

      case 3: // Operational State
        if (gas_data_ready) {
          Process_Gas_Data();
          gas_data_ready = 0;
          TM1637_ShowNumber(current_gas_value);
        }
        break;
    }

    // Process tasks concurrently 
    Modbus_Process_State_Machine();
    TM1637_Tick_State_Machine();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  /**USART1 GPIO Configuration
  PB7   ------> USART1_RX
  PC14-OSCX_IN (PC14)   ------> USART1_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin);

  /**/
  GPIO_InitStruct.Pin = RS485_DE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TM1637_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(TM1637_SCL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TM1637_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(TM1637_SDA_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- GAS SENSOR IMPLEMENTATION ---
void GasSensor_SendInit(void) {
  uint8_t init_cmd[9] = {0xFF, 0x01, 0x78, 0x03, 0x00, 0x00, 0x00, 0x00, 0x84};
  
  for (uint8_t i = 0; i < 9; i++) {
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, init_cmd[i]);
  }
}

void Process_Gas_Data(void) {
  gas_rx_index = 0; 

  if (gas_rx_buffer[0] != 0xFF || gas_rx_buffer[1] != 0x88) return;

  uint8_t checksum = 0;
  for (uint8_t i = 1; i < 8; i++) {
    checksum += gas_rx_buffer[i];
  }
  checksum = (~checksum) + 1;
  if (checksum != gas_rx_buffer[8]) return; 

  uint16_t raw_concentration = (gas_rx_buffer[2] << 8) | gas_rx_buffer[3];
  uint8_t decimal_places = gas_rx_buffer[5];
  
  float resolution = 1.0f;
  if (decimal_places == 1) resolution = 0.1f;
  else if (decimal_places == 2) resolution = 0.01f;

  float final_concentration = (float)raw_concentration * resolution;
  current_gas_value = (uint16_t)(final_concentration + 0.5f); 

  uint16_t temp_adc = (gas_rx_buffer[6] << 8) | gas_rx_buffer[7];
  float vpd3 = 3.0f * (float)temp_adc / 1024.0f;
  if (vpd3 < 3.0f) {
    float rth = vpd3 * 10000.0f / (3.0f - vpd3);
    float log_rth = logf(rth / 10000.0f);
    float temp_c = 1.0f / (1.0f / (273.15f + 25.0f) + (1.0f / 3380.13f) * log_rth) - 273.15f;
    (void)temp_c; 
  }
}

// --- MODBUS RTU SLAVE IMPLEMENTATION ---

void Modbus_Process_State_Machine(void) {
  if (modbus_rx_index > 0 && (uwTick - modbus_last_rx_tick) > 2) {
    modbus_frame_complete = 1;
  }

  if (modbus_frame_complete) {
    if (Modbus_Validate_And_Parse(modbus_rx_buffer, modbus_rx_index)) {
      
      LL_GPIO_SetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin);
      
      uint8_t response[8] = {0x01, 0x03, 0x02, (uint8_t)(current_gas_value >> 8), (uint8_t)(current_gas_value & 0xFF), 0x00, 0x00, 0x00};
      
      // Calculate CRC16 Modbus (Simplified Example inline)
      uint16_t crc = 0xFFFF;
      for (int pos = 0; pos < 6; pos++) {
        crc ^= (uint16_t)response[pos];
        for (int i = 8; i != 0; i--) {
          if ((crc & 0x0001) != 0) {
            crc >>= 1;
            crc ^= 0xA001;
          } else {
            crc >>= 1;
          }
        }
      }
      response[6] = (uint8_t)(crc & 0xFF);
      response[7] = (uint8_t)(crc >> 8);

      for (uint8_t i = 0; i < 8; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, response[i]);
      }
      
      while (!LL_USART_IsActiveFlag_TC(USART2));
      LL_GPIO_ResetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin);
    }
    modbus_rx_index = 0;
    modbus_frame_complete = 0;
  }
}

uint8_t Modbus_Validate_And_Parse(uint8_t *buffer, uint16_t length) {
  if (length < 4) return 0;
  if (buffer[0] != 0x01) return 0; 
  return 1;
}

// --- TM1637 STATE MACHINE ---
void TM1637_SetSegments(const uint8_t *segments) {
  if (tm_state != TM_IDLE) return; 

  // Byte 0: Data Command 1 (Write data, auto address increment)
  tm_tx_pipeline[0] = 0x40; 
  // Byte 1: Address Command 2 (Start writing at display register location 0)
  tm_tx_pipeline[1] = 0xC0; 
  // Bytes 2-5: The 4 raw structural display frame segment masks
  tm_tx_pipeline[2] = segments[0];
  tm_tx_pipeline[3] = segments[1];
  tm_tx_pipeline[4] = segments[2];
  tm_tx_pipeline[5] = segments[3];
  // Byte 6: Display Brightness Control Command 3 (Max Intensity: 0x88 | 0x07)
  tm_tx_pipeline[6] = 0x8F; 

  tm_pipeline_index = 0;
  display_update_required = 1;
}

void TM1637_ShowNumber(uint16_t value) {
  uint8_t seg_buf[4];
  seg_buf[0] = (value >= 1000) ? TM_FONT_NUM[(value / 1000) % 10] : 0x00;
  seg_buf[1] = (value >= 100)  ? TM_FONT_NUM[(value / 100) % 10]  : 0x00;
  seg_buf[2] = (value >= 10)   ? TM_FONT_NUM[(value / 10) % 10]   : 0x00;
  seg_buf[3] = TM_FONT_NUM[value % 10];
  
  if (value == 0) seg_buf[3] = TM_FONT_NUM[0];
  
  TM1637_SetSegments(seg_buf);
}

void TM1637_Tick_State_Machine(void) {
  static uint32_t last_tm_tick = 0;
  if (uwTick - last_tm_tick < 1) return; // Process at a strict 1ms pace
  last_tm_tick = uwTick;

  switch (tm_state) {
    case TM_IDLE:
      if (display_update_required) {
        display_update_required = 0;
        tm_pipeline_index = 0;
        tm_state = TM_START;
        tm_tx_byte = tm_tx_pipeline[tm_pipeline_index]; // Grab first Command (0x40)
      }
      break;

    case TM_START:
      LL_GPIO_ResetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin); // SDA low
      tm_state = TM_SEND_BYTE;
      tm_bit_index = 0;
      break;

    case TM_SEND_BYTE:
      LL_GPIO_ResetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin); // CLK low
      if (tm_tx_byte & (1 << tm_bit_index)) {
        LL_GPIO_SetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin);
      } else {
        LL_GPIO_ResetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin);
      }
      LL_GPIO_SetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin); // CLK high (Sample)
      
      tm_bit_index++;
      if (tm_bit_index >= 8) {
        tm_state = TM_ACK;
      }
      break;

    case TM_ACK:
      LL_GPIO_ResetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin); // CLK low
      LL_GPIO_SetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin);   // Release SDA for ACK
      LL_GPIO_SetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin);   // CLK high
      tm_state = TM_STOP;
      break;

    case TM_STOP:
      // Check data boundary to determine if we should generate a STOP condition
      // Block 1 (Index 0: Command 1) needs a STOP
      // Block 2 (Index 1-5: Address + 4 Data Bytes) streams continuously, STOP occurs after index 5
      // Block 3 (Index 6: Brightness Command) needs a final STOP
      if (tm_pipeline_index == 0 || tm_pipeline_index == 5 || tm_pipeline_index == 6) {
        LL_GPIO_ResetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin); // CLK low
        LL_GPIO_ResetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin); // SDA low
        LL_GPIO_SetOutputPin(TM1637_SCL_GPIO_Port, TM1637_SCL_Pin);   // CLK high
        LL_GPIO_SetOutputPin(TM1637_SDA_GPIO_Port, TM1637_SDA_Pin);   // SDA high (Physical STOP)
      }

      tm_pipeline_index++;
      if (tm_pipeline_index < 7) {
        // If the next byte continues the current frame block (Addresses & Display Segments), 
        // skip the START toggle logic and stream the next byte out directly.
        if (tm_pipeline_index >= 2 && tm_pipeline_index <= 5) {
          tm_state = TM_SEND_BYTE;
        } else {
          tm_state = TM_START; // Transition to next explicit command group block
        }
        tm_tx_byte = tm_tx_pipeline[tm_pipeline_index];
        tm_bit_index = 0;
      } else {
        tm_state = TM_IDLE; // Total pipeline sequence successfully dispatched
      }
      break;
  }
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
