/* USER CODE BEGIN Header */
/**
 * @file           : main.cpp
 * @brief          : Main program body with CAN parsing, Moving Average, and UART Injection
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "orion2_can_comms.hpp"
#include "stm32l4xx_hal_can.h"
#include "ws_can_comms.hpp"
#include <cinttypes>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- 1. GLOBAL OBJECTS ---
WaveSculptor ws(&hcan1);
Orion2 orion(&hcan1);

// --- 2. MOVING AVERAGE CLASS ---
template <uint8_t SIZE> class MovingAverage {
private:
  float buffer[SIZE] = {0};
  uint8_t index = 0;
  float sum = 0;

public:
  void add(float val) {
    sum -= buffer[index];
    buffer[index] = val;
    sum += val;
    index = (index + 1) % SIZE;
  }
  float get() { return sum / SIZE; }
};

// 10-sample moving average buffers
MovingAverage<10> avgSOC;
MovingAverage<10> avgSpeed;
MovingAverage<10> avgBattTemp;
MovingAverage<10> avgMCTemp;
MovingAverage<10> avgMotorTemp;
MovingAverage<10> avgPowerIn;
MovingAverage<10> avgPowerOut;

// --- 3. UART TERMINAL CAN INJECTOR ---
uint8_t uartRxByte;
char uartRxBuffer[100];
uint8_t uartRxIndex = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Printf(const char *fmt, ...);
void processTerminalCommand();
void sendGenieObject(uint8_t object, uint8_t index, uint16_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- Custom UART Print Wrapper (Bypasses stdout linker error) ---
void UART_Printf(const char *fmt, ...) {
  char buffer[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
}

void processTerminalCommand() {
  uint32_t id;
  uint32_t d[8];

  // Check if the user typed a CAN injection command
  // Uses SCNx32 to guarantee proper 32-bit hex reading on ARM architectures
  if (sscanf(uartRxBuffer, "CAN %" SCNx32 " %" SCNx32 " %" SCNx32 " %" SCNx32 " %" SCNx32 " %" SCNx32 " %" SCNx32 " %" SCNx32 " %" SCNx32, &id, &d[0],
             &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7]) == 9) {
    uint8_t rxData[8];
    for (int i = 0; i < 8; i++)
      rxData[i] = (uint8_t)d[i];

    // Route injected CAN message to the appropriate parser
    if (orion.isOrion2Message(id)) {
      uint16_t offset = (id == 0x36) ? 0x36 : (id - orion.getBaseAddr());
      orion.parseMeasurement(static_cast<Orion2::MessageID>(offset), rxData);
      UART_Printf("Injected to Orion2: ID %" PRIX32 "\r\n", id);
    } else if (ws.isWaveSculptorMessage(id)) {
      ws.parseMeasurement(static_cast<WaveSculptor::MessageID>(id - ws.getBaseAddr()), rxData);
      UART_Printf("Injected to WaveSculptor: ID %" PRIX32 "\r\n", id);
    } else {
      UART_Printf("Ignored ID %" PRIX32 "\r\n", id);
    }
  } else {
    UART_Printf("Invalid Command. Use: CAN <ID> <B0>..<B7> in HEX\r\n");
  }
}

// UART Interrupt Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (uartRxByte == '\r' || uartRxByte == '\n') {
      uartRxBuffer[uartRxIndex] = '\0'; // Null terminate
      if (uartRxIndex > 0)
        processTerminalCommand();
      uartRxIndex = 0; // Reset buffer
    } else if (uartRxIndex < sizeof(uartRxBuffer) - 1) {
      uartRxBuffer[uartRxIndex++] = uartRxByte;
    }
    // Re-arm interrupt
    HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);
  }
}

// --- 4. CAN HARDWARE INTERRUPT ---
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    uint32_t id = rxHeader.StdId;

    // Route real CAN message to parsers
    if (orion.isOrion2Message(id)) {
      uint16_t offset = (id == 0x36) ? 0x36 : (id - orion.getBaseAddr());
      orion.parseMeasurement(static_cast<Orion2::MessageID>(offset), rxData);
    } else if (ws.isWaveSculptorMessage(id)) {
      ws.parseMeasurement(static_cast<WaveSculptor::MessageID>(id - ws.getBaseAddr()), rxData);
    }
  }
}

// Display transmit helper
void sendGenieObject(uint8_t object, uint8_t index, uint16_t data) {
  uint8_t message[6];
  message[0] = 0x01;
  message[1] = object;
  message[2] = index;
  message[3] = (data >> 8) & 0xFF;
  message[4] = data & 0xFF;
  message[5] = message[0] ^ message[1] ^ message[2] ^ message[3] ^ message[4];
  HAL_UART_Transmit(&huart1, message, 6, 100);
  HAL_Delay(2); // Prevent buffer overrun on display
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 1. Configure CAN Filter to accept all standard IDs
  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  // 2. Start CAN and activate RX Interrupts
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // Init WaveSculptor object with initialized CAN handle
  ws.init(&hcan1, 0x400, 0x500);

  // 3. Start UART Terminal listener
  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  // Give 4D Display time to boot and mount SD Card
  UART_Printf("Booting Display...\r\n");
  HAL_Delay(3500);
  UART_Printf("System Live.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastSampleTime = 0;
  uint32_t lastDisplayTime = 0;

  while (1) {
    uint32_t currentMillis = HAL_GetTick();

    // --- 100 Hz (10ms) SAMPLING LOOP ---
    if (currentMillis - lastSampleTime >= 10) {
      lastSampleTime = currentMillis;

      // Sample direct data
      avgSOC.add(orion.getPackSOC());
      avgSpeed.add(ws.getVehicleVelocity());
      avgBattTemp.add(orion.getIntakeTemp());
      avgMCTemp.add(ws.getDSPBoardTemp());
      avgMotorTemp.add(ws.getMotorTemp());

      // Calculate Power (Watts)
      float voltage = ws.getBusVoltage();
      float current = ws.getBusCurrent();
      float powerWatts = voltage * current;

      if (powerWatts >= 0) {
        avgPowerOut.add(powerWatts);
        avgPowerIn.add(0);
      } else {
        avgPowerOut.add(0);
        avgPowerIn.add(powerWatts * -1.0f); // Make positive for display
      }
    }

    // --- 10 Hz (100ms) DISPLAY UPDATE LOOP ---
    if (currentMillis - lastDisplayTime >= 100) {
      lastDisplayTime = currentMillis;

      // Fetch smoothed values and cast to integers for display
      uint16_t dispSOC = (uint16_t)avgSOC.get();
      uint16_t dispSpeed = (uint16_t)avgSpeed.get();
      uint16_t dispPwrIn = (uint16_t)avgPowerIn.get();
      uint16_t dispPwrOut = (uint16_t)avgPowerOut.get();
      uint16_t dispBattT = (uint16_t)avgBattTemp.get();
      uint16_t dispMCT = (uint16_t)avgMCTemp.get();
      uint16_t dispMotorT = (uint16_t)avgMotorTemp.get();

      // Update Gauges
      sendGenieObject(0x08, 0, dispSpeed);
      sendGenieObject(0x04, 0, dispSOC);

      // Update Text Digits
      sendGenieObject(0x0F, 0, dispPwrIn);  // TextDigits1
      sendGenieObject(0x0F, 1, dispPwrOut); // TextDigits2
      sendGenieObject(0x0F, 2, dispBattT);  // TextDigits3
      sendGenieObject(0x0F, 3, dispMCT);    // TextDigits4
      sendGenieObject(0x0F, 4, dispMotorT); // TextDigits5

      // Heartbeat
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
   */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
