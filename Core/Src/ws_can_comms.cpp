/**
 * @file ws_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of WaveSculptor CAN communications class based on
 * Prohelion's documentation at
 * https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html
 * @version 0.2
 * @date 2026-03-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ws_can_comms.hpp"
#include "uart_guard.hpp"
#include <cstdio>
#include <cstring>

// UPDATE: Constructor uses CAN_HandleTypeDef
WaveSculptor::WaveSculptor(CAN_HandleTypeDef *hcan, uint16_t baseAddr, uint16_t dcuBaseAddr)
    : hcan_(hcan), baseAddr_(baseAddr), dcuBaseAddr_(dcuBaseAddr) {}

void WaveSculptor::init(CAN_HandleTypeDef *hcan, uint16_t baseAddr, uint16_t dcuBaseAddr) {
  hcan_ = hcan;
  baseAddr_ = baseAddr;
  dcuBaseAddr_ = dcuBaseAddr;
}

// ============================================================================
// Drive Control Commands
// ============================================================================

HAL_StatusTypeDef WaveSculptor::sendMotorDrive(float motorCurrent, float motorRPM) {
  if (!isInitialized()) {
    lastError_ = HAL_ERROR;
    return lastError_;
  }
#if WS_DEBUG_ENABLED
  {
    UartGuard guard;
    printf("WS: Sending Motor Drive - Current: %.2f %%, RPM: %.2f\n", motorCurrent, motorRPM);
  }
#endif
  CAN_TxHeaderTypeDef txHeader = dcuBaseTxHeader_(DCUMessageID::MotorDrive);

  uint8_t txData[8] = {0};
  memcpy(&txData[4], &motorCurrent, sizeof(motorCurrent));
  memcpy(&txData[0], &motorRPM, sizeof(motorRPM));

  uint32_t txMailbox;
  lastError_ = HAL_CAN_AddTxMessage(hcan_, &txHeader, txData, &txMailbox);
  return lastError_;
}

HAL_StatusTypeDef WaveSculptor::sendMotorPower(float busCurrent) {
  if (!isInitialized()) {
    lastError_ = HAL_ERROR;
    return lastError_;
  }
#if WS_DEBUG_ENABLED
  {
    UartGuard guard;
    printf("WS: Sending Motor Power - Bus Current: %.2f %%\n", busCurrent);
  }
#endif
  CAN_TxHeaderTypeDef txHeader = dcuBaseTxHeader_(DCUMessageID::MotorPower);

  uint8_t txData[8] = {0};
  memcpy(&txData[4], &busCurrent, sizeof(busCurrent));

  uint32_t txMailbox;
  lastError_ = HAL_CAN_AddTxMessage(hcan_, &txHeader, txData, &txMailbox);
  return lastError_;
}

HAL_StatusTypeDef WaveSculptor::sendReset() {
  if (!isInitialized()) {
    lastError_ = HAL_ERROR;
    return lastError_;
  }
#if WS_DEBUG_ENABLED
  {
    UartGuard guard;
    printf("WS: Sending Reset Command\n");
  }
#endif
  CAN_TxHeaderTypeDef txHeader = dcuBaseTxHeader_(DCUMessageID::Reset);

  uint8_t txData[8] = {0};

  uint32_t txMailbox;
  lastError_ = HAL_CAN_AddTxMessage(hcan_, &txHeader, txData, &txMailbox);
  return lastError_;
}

// ============================================================================
// Measurement Parsing
// ============================================================================

HAL_StatusTypeDef WaveSculptor::parseMeasurement(WaveSculptor::MessageID id, const uint8_t *rxData) {
  if (rxData == nullptr) {
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  lastError_ = HAL_OK;

  switch (id) {
  case MessageID::Identification: {
    memcpy(&serialNumber_, &rxData[0], sizeof(serialNumber_));
    memcpy(&deviceID_, &rxData[4], sizeof(deviceID_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Identification - SN=0x%08X, DevID=0x%08X\n", serialNumber_, deviceID_);
    }
#endif
    break;
  }
  case MessageID::Status: {
    memcpy(&receiveErrorCount_, &rxData[0], sizeof(receiveErrorCount_));
    memcpy(&transmitErrorCount_, &rxData[1], sizeof(transmitErrorCount_));
    memcpy(&activeMotor_, &rxData[2], sizeof(activeMotor_));
    memcpy(&errorFlags_, &rxData[4], sizeof(errorFlags_));
    memcpy(&limitFlags_, &rxData[6], sizeof(limitFlags_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Status - RxErr=%u, TxErr=%u, Active=%u, Errors=0x%04X, Limits=0x%04X\n", receiveErrorCount_, transmitErrorCount_, activeMotor_,
             errorFlags_, limitFlags_);
    }
#endif
    break;
  }
  case MessageID::Bus: {
    memcpy(&busCurrent_, &rxData[0], sizeof(busCurrent_));
    memcpy(&busVoltage_, &rxData[4], sizeof(busVoltage_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Bus - Current=%.2f A, Voltage=%.2f V\n", busCurrent_, busVoltage_);
    }
#endif
    break;
  }
  case MessageID::Velocity: {
    memcpy(&vehicleVelocity_, &rxData[0], sizeof(vehicleVelocity_));
    memcpy(&motorVelocity_, &rxData[4], sizeof(motorVelocity_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Velocity - Vehicle=%.2f m/s, Motor=%.2f RPM\n", vehicleVelocity_, motorVelocity_);
    }
#endif
    break;
  }
  case MessageID::PhaseCurrent: {
    memcpy(&phaseBCurrent_, &rxData[0], sizeof(phaseBCurrent_));
    memcpy(&phaseCCurrent_, &rxData[4], sizeof(phaseCCurrent_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Phase Current - B=%.2f A, C=%.2f A\n", phaseBCurrent_, phaseCCurrent_);
    }
#endif
    break;
  }
  case MessageID::MotorVoltageVector: {
    memcpy(&dVoltage_, &rxData[0], sizeof(dVoltage_));
    memcpy(&qVoltage_, &rxData[4], sizeof(qVoltage_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Motor Voltage - D=%.2f V, Q=%.2f V\n", dVoltage_, qVoltage_);
    }
#endif
    break;
  }
  case MessageID::MotorCurrentVector: {
    memcpy(&dCurrent_, &rxData[0], sizeof(dCurrent_));
    memcpy(&qCurrent_, &rxData[4], sizeof(qCurrent_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Motor Current - D=%.2f A, Q=%.2f A\n", dCurrent_, qCurrent_);
    }
#endif
    break;
  }
  case MessageID::MotorBackEMF: {
    memcpy(&dBackEMF_, &rxData[0], sizeof(dBackEMF_));
    memcpy(&qBackEMF_, &rxData[4], sizeof(qBackEMF_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Back-EMF - D=%.2f V, Q=%.2f V\n", dBackEMF_, qBackEMF_);
    }
#endif
    break;
  }
  case MessageID::VoltageRail15V: {
    memcpy(&measured15VSupply_, &rxData[0], sizeof(measured15VSupply_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: 15V Rail - %.2f V\n", measured15VSupply_);
    }
#endif
    break;
  }
  case MessageID::VoltageRail3V3_1V9: {
    memcpy(&measured3V3Supply_, &rxData[0], sizeof(measured3V3Supply_));
    memcpy(&measured1V9Supply_, &rxData[4], sizeof(measured1V9Supply_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Power Rails - 3V3=%.2f V, 1V9=%.2f V\n", measured3V3Supply_, measured1V9Supply_);
    }
#endif
    break;
  }
  case MessageID::HeatSinkMotorTemp: {
    memcpy(&heatsinkTemp_, &rxData[0], sizeof(heatsinkTemp_));
    memcpy(&motorTemp_, &rxData[4], sizeof(motorTemp_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Temperature - Heatsink=%.2f C, Motor=%.2f C\n", heatsinkTemp_, motorTemp_);
    }
#endif
    break;
  }
  case MessageID::DSPTemp: {
    memcpy(&dspBoardTemp_, &rxData[0], sizeof(dspBoardTemp_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: DSP Temp - %.2f C\n", dspBoardTemp_);
    }
#endif
    break;
  }
  case MessageID::OdometerBusAh: {
    memcpy(&dcBusAh_, &rxData[0], sizeof(dcBusAh_));
    memcpy(&odometer_, &rxData[4], sizeof(odometer_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Energy/Distance - BusAh=%.2f Ah, Odometer=%.2f km\n", dcBusAh_, odometer_);
    }
#endif
    break;
  }
  case MessageID::SlipSpeed: {
    memcpy(&slipSpeed_, &rxData[0], sizeof(slipSpeed_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Slip Speed - %.2f m/s\n", slipSpeed_);
    }
#endif
    break;
  }
  default:
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  return HAL_OK;
}
