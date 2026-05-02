/**
 * @file ws_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of WaveSculptor CAN communications class based on
 * raw DBC definitions for accurate byte mapping.
 * @version 0.3
 * @date 2026-05-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ws_can_comms.hpp"
#include "uart_guard.hpp"
#include <cstdio>
#include <cstring>

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
// Measurement Parsing (Corrected to DBC Specification)
// ============================================================================

HAL_StatusTypeDef WaveSculptor::parseMeasurement(WaveSculptor::MessageID id, const uint8_t *rxData) {
  if (rxData == nullptr) {
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  lastError_ = HAL_OK;

  switch (id) {
  case MessageID::Identification: {
    // DBC: TritiumID 0|32, SerialNumber 32|32
    memcpy(&deviceID_, &rxData[0], sizeof(deviceID_));
    memcpy(&serialNumber_, &rxData[4], sizeof(serialNumber_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Identification - SN=0x%08X, DevID=0x%08X\n", serialNumber_, deviceID_);
    }
#endif
    break;
  }
  case MessageID::Status: {
    // DBC: Limits 0|16, Errors 16|16, ActiveMotor 32|16, TxErr 48|8, RxErr 56|8
    memcpy(&limitFlags_, &rxData[0], sizeof(limitFlags_));
    memcpy(&errorFlags_, &rxData[2], sizeof(errorFlags_));
    memcpy(&activeMotor_, &rxData[4], sizeof(activeMotor_));
    transmitErrorCount_ = rxData[6];
    receiveErrorCount_ = rxData[7];
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
    // DBC: BusVoltage 0|32, BusCurrent 32|32
    memcpy(&busVoltage_, &rxData[0], sizeof(busVoltage_));
    memcpy(&busCurrent_, &rxData[4], sizeof(busCurrent_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Bus - Voltage=%.2f V, Current=%.2f A\n", busVoltage_, busCurrent_);
    }
#endif
    break;
  }
  case MessageID::Velocity: {
    // DBC: MotorVelocity 0|32, VehicleVelocity 32|32
    memcpy(&motorVelocity_, &rxData[0], sizeof(motorVelocity_));
    memcpy(&vehicleVelocity_, &rxData[4], sizeof(vehicleVelocity_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Velocity - Motor=%.2f RPM, Vehicle=%.2f m/s\n", motorVelocity_, vehicleVelocity_);
    }
#endif
    break;
  }
  case MessageID::PhaseCurrent: {
    // DBC: PhaseCurrentB 0|32, PhaseCurrentC 32|32
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
    // DBC: Vq 0|32, Vd 32|32
    memcpy(&qVoltage_, &rxData[0], sizeof(qVoltage_));
    memcpy(&dVoltage_, &rxData[4], sizeof(dVoltage_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Motor Voltage - Q=%.2f V, D=%.2f V\n", qVoltage_, dVoltage_);
    }
#endif
    break;
  }
  case MessageID::MotorCurrentVector: {
    // DBC: Iq 0|32, Id 32|32
    memcpy(&qCurrent_, &rxData[0], sizeof(qCurrent_));
    memcpy(&dCurrent_, &rxData[4], sizeof(dCurrent_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Motor Current - Q=%.2f A, D=%.2f A\n", qCurrent_, dCurrent_);
    }
#endif
    break;
  }
  case MessageID::MotorBackEMF: {
    // DBC: BEMFq 0|32, BEMFd 32|32
    memcpy(&qBackEMF_, &rxData[0], sizeof(qBackEMF_));
    memcpy(&dBackEMF_, &rxData[4], sizeof(dBackEMF_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Back-EMF - Q=%.2f V, D=%.2f V\n", qBackEMF_, dBackEMF_);
    }
#endif
    break;
  }
  case MessageID::VoltageRail15V: {
    // DBC: Reserved 0|32, Supply15V 32|32
    memcpy(&measured15VSupply_, &rxData[4], sizeof(measured15VSupply_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: 15V Rail - %.2f V\n", measured15VSupply_);
    }
#endif
    break;
  }
  case MessageID::VoltageRail3V3_1V9: {
    // DBC: Supply1V9 0|32, Supply3V3 32|32
    memcpy(&measured1V9Supply_, &rxData[0], sizeof(measured1V9Supply_));
    memcpy(&measured3V3Supply_, &rxData[4], sizeof(measured3V3Supply_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Power Rails - 1V9=%.2f V, 3V3=%.2f V\n", measured1V9Supply_, measured3V3Supply_);
    }
#endif
    break;
  }
  case MessageID::HeatSinkMotorTemp: {
    // DBC: MotorTemp 0|32, HeatsinkTemp 32|32
    memcpy(&motorTemp_, &rxData[0], sizeof(motorTemp_));
    memcpy(&heatsinkTemp_, &rxData[4], sizeof(heatsinkTemp_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Temperature - Motor=%.2f C, Heatsink=%.2f C\n", motorTemp_, heatsinkTemp_);
    }
#endif
    break;
  }
  case MessageID::DSPTemp: {
    // DBC: DspBoardTemp 0|32, Reserved 32|32
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
    // DBC: Odometer 0|32, DCBusAh 32|32
    memcpy(&odometer_, &rxData[0], sizeof(odometer_));
    memcpy(&dcBusAh_, &rxData[4], sizeof(dcBusAh_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Energy/Distance - Odometer=%.2f m, BusAh=%.2f Ah\n", odometer_, dcBusAh_);
    }
#endif
    break;
  }
  case MessageID::SlipSpeed: {
    // DBC: SlipSpeed 0|32, Reserved 32|32
    memcpy(&slipSpeed_, &rxData[0], sizeof(slipSpeed_));
#if WS_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("WS: Slip Speed - %.2f Hz\n", slipSpeed_);
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