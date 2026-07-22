/* USER CODE BEGIN Header */
/**
 * @file           : main.cpp
 * @brief          : Production Solar Dashboard - V7 (huart2 Debug Restored + FIFO Drain)
 */
/* USER CODE END Header */

#include "main.h"
#include "orion2_can_comms.hpp"
#include "stm32l4xx_hal_can.h"
#include "ws_can_comms.hpp"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <cmath>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
WaveSculptor ws(&hcan1);
Orion2 orion(&hcan1);

// The WaveSculptor internally calculates Vehicle Velocity in m/s.
// 1 m/s = 2.23694 Miles Per Hour
const float MS_TO_MPH_CONVERSION = 2.23694f;

// Orion's coulomb-counted SOC was corrupted by a low-voltage BPS test, so SOC
// is estimated from pack open-circuit voltage instead until it's recalibrated.
const float PACK_VOLTAGE_FULL = 134.4f;  // 100% SOC
const float PACK_VOLTAGE_EMPTY = 83.0f;  // 0% SOC

float estimateSOCFromVoltage(float openCircuitVoltage) {
  float soc = (openCircuitVoltage - PACK_VOLTAGE_EMPTY) / (PACK_VOLTAGE_FULL - PACK_VOLTAGE_EMPTY) * 100.0f;
  if (soc < 0.0f) soc = 0.0f;
  if (soc > 100.0f) soc = 100.0f;
  return soc;
}

// Rewritten Moving Average to completely eliminate floating-point drift
template <uint8_t SIZE> class MovingAverage {
private:
  float buffer[SIZE] = {0};
  uint8_t index = 0;
public:
  void add(float val) {
    if (std::isnan(val) || std::isinf(val)) return; 
    buffer[index] = val;
    index = (index + 1) % SIZE;
  }
  float get() {
    float sum = 0;
    for(int i = 0; i < SIZE; i++) sum += buffer[i];
    return sum / SIZE;
  }
};

MovingAverage<10> avgSOC, avgSpeed, avgBattTemp, avgMCTemp, avgMotorTemp, avgPowerIn, avgPowerOut;

// Terminal variables restored
uint8_t uartRxByte;
char uartRxBuffer[100];
uint8_t uartRxIndex = 0;
/* USER CODE END PV */

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
void UART_Printf(const char *fmt, ...);
void processTerminalCommand();
void sendGenieObject(uint8_t object, uint8_t index, uint16_t data);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Custom UART Print Wrapper for Terminal Debugging
void UART_Printf(const char *fmt, ...) {
  char buffer[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
}

// Allows manual CAN testing via Serial Monitor
void processTerminalCommand() {
  unsigned int id = 0; unsigned int d[8] = {0};
  if (strncmp(uartRxBuffer, "CAN ", 4) == 0) {
    int parsedArgs = sscanf(uartRxBuffer + 4, "%x %x %x %x %x %x %x %x %x", &id, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7]);
    if (parsedArgs == 9) {
      uint8_t rxData[8];
      for (int i = 0; i < 8; i++) rxData[i] = static_cast<uint8_t>(d[i]);
      if (orion.isOrion2Message(id)) {
        orion.parseMeasurement(static_cast<Orion2::MessageID>((id == 0x36) ? 0x36 : (id - orion.getBaseAddr())), rxData);
      } else if (ws.isWaveSculptorMessage(id)) {
        ws.parseMeasurement(static_cast<WaveSculptor::MessageID>(id - ws.getBaseAddr()), rxData);
      }
    }
  }
}

// UART Receive Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (uartRxByte == '\r' || uartRxByte == '\n') {
      uartRxBuffer[uartRxIndex] = '\0';
      if (uartRxIndex > 0) processTerminalCommand();
      uartRxIndex = 0;
    } else if (uartRxIndex < sizeof(uartRxBuffer) - 1) {
      uartRxBuffer[uartRxIndex++] = uartRxByte;
    }
    HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);
  }
}

// CAN Hardware Interrupt
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  
  // Drain the ENTIRE FIFO to prevent buffer overflows under heavy load
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
      uint32_t id = rxHeader.StdId;
      if (orion.isOrion2Message(id)) {
        orion.parseMeasurement(static_cast<Orion2::MessageID>((id == 0x36) ? 0x36 : (id - orion.getBaseAddr())), rxData);
      } else if (ws.isWaveSculptorMessage(id)) {
        ws.parseMeasurement(static_cast<WaveSculptor::MessageID>(id - ws.getBaseAddr()), rxData);
      }
    }
  }
}

// Display Transmit
void sendGenieObject(uint8_t object, uint8_t index, uint16_t data) {
  uint8_t message[6];
  message[0] = 0x01; message[1] = object; message[2] = index;
  message[3] = (data >> 8) & 0xFF; message[4] = data & 0xFF;
  message[5] = message[0] ^ message[1] ^ message[2] ^ message[3] ^ message[4];
  HAL_UART_Transmit(&huart1, message, 6, 100);
  HAL_Delay(5); 
}
/* USER CODE END 0 */

int main(void) {
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // Pull-up on PA15 to prevent the floating pin interrupt crash
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  CAN_FilterTypeDef canFilterConfig = {0};
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  ws.init(&hcan1, 0x400, 0x500);

  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  HAL_Delay(3000); // Give the 4D Display time to boot
  UART_Printf("Dashboard Ready. huart2 Restored.\r\n");
  
  uint32_t lastSampleTime = 0;
  uint32_t lastDisplayTime = 0;
  /* USER CODE END 2 */

  while (1) {
    uint32_t currentMillis = HAL_GetTick();

    // --- 100 Hz Sampling Loop ---
    if (currentMillis - lastSampleTime >= 10) {
      lastSampleTime = currentMillis;

      avgBattTemp.add(orion.getIntakeTemp());
      avgMCTemp.add(ws.getDSPBoardTemp()); 
      avgMotorTemp.add(ws.getMotorTemp()); 

      // Native m/s directly to absolute MPH
      float velocityMPH = fabsf(ws.getVehicleVelocity() * MS_TO_MPH_CONVERSION);
      // Hard clamp incoming CAN errors so the gauge never vanishes
      if (velocityMPH > 99.0f) velocityMPH = 99.0f;
      avgSpeed.add(velocityMPH);

      // BMS Power Calculation
      float bmsVolts = orion.getPackInstVoltage(); 
      float bmsAmps  = orion.getPackCurrent();     
      float bmsPowerWatts = bmsVolts * bmsAmps;

      avgSOC.add(estimateSOCFromVoltage(orion.getPackOpenVoltage()));

      if (bmsAmps >= 0) {
          avgPowerOut.add(fabsf(bmsPowerWatts)); 
          avgPowerIn.add(0);
      } else {
          avgPowerIn.add(fabsf(bmsPowerWatts));  
          avgPowerOut.add(0);
      }
    }

    // --- 10 Hz Display Loop ---
    if (currentMillis - lastDisplayTime >= 100) {
      lastDisplayTime = currentMillis;

      uint16_t dispBattT  = (uint16_t)(avgBattTemp.get() * 10.0f);
      uint16_t dispMCT    = (uint16_t)(avgMCTemp.get() * 10.0f);
      uint16_t dispMotorT = (uint16_t)(avgMotorTemp.get() * 10.0f);

      // Clamp power to 9999 to guarantee a clean 4-digit fit
      uint16_t dispPwrIn  = (uint16_t)fminf(avgPowerIn.get(), 9999.0f);
      uint16_t dispPwrOut = (uint16_t)fminf(avgPowerOut.get(), 9999.0f);
      uint16_t dispSpeed  = (uint16_t)avgSpeed.get();

      sendGenieObject(0x08, 0, dispSpeed);                   // Circular Gauge
      sendGenieObject(0x04, 0, (uint16_t)avgSOC.get());      // Linear Gauge
      sendGenieObject(0x0F, 0, dispBattT);                   // iTextDigits1
      sendGenieObject(0x0F, 1, dispMCT);                     // iTextDigits2
      sendGenieObject(0x0F, 2, dispMotorT);                  // iTextDigits3
      sendGenieObject(0x0F, 3, dispPwrIn);                   // iTextDigits4
      sendGenieObject(0x0F, 4, dispPwrOut);                  // iTextDigits5

      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); // Alive LED
    }
  }
}

/** * Peripheral Inits Below (Standard CubeMX generation) 
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_CAN1_Init(void) {
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
  if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void) {
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
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }