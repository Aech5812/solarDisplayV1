/**
 * @file ws_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for WaveSculptor motor controller CAN communications class
 * @version 0.2
 * @date 2026-03-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef WS_CAN_COMMS_HPP
#define WS_CAN_COMMS_HPP

#include "stm32l4xx_hal.h"
#include <cstdio>

// Debug toggle - set to true to enable debug prints, false to disable
#define WS_DEBUG_ENABLED false

/**
 * @brief Class for WaveSculptor motor controller CAN communications
 *
 * Provides non-blocking (queue-based) transmission and synchronous parsing of received CAN messages.
 * Measurements are updated when new CAN data is received via parseMeasurement().
 */
class WaveSculptor {
public:
  static constexpr uint16_t DEFAULT_BASE_ADDR = 0x400;
  static constexpr uint16_t DEFAULT_DCU_BASE_ADDR = 0x500;

  /**
   * @brief Enum for WaveSculptor measurement message IDs (offsets from base address)
   */
  enum class MessageID : uint16_t {
    Identification = 0x00,
    Status = 0x01,
    Bus = 0x02,
    Velocity = 0x03,
    PhaseCurrent = 0x04,
    MotorVoltageVector = 0x05,
    MotorCurrentVector = 0x06,
    MotorBackEMF = 0x07,
    VoltageRail15V = 0x08,
    VoltageRail3V3_1V9 = 0x09,
    HeatSinkMotorTemp = 0x0B,
    DSPTemp = 0x0C,
    OdometerBusAh = 0x0E,
    SlipSpeed = 0x17
  };

  /**
   * @brief Enum for WaveSculptor Driver Control Unit (DCU) command message IDs
   */
  enum class DCUMessageID : uint16_t { MotorDrive = 0x01, MotorPower = 0x02, Reset = 0x02 };

private:
  CAN_HandleTypeDef *hcan_ = nullptr;

  // Base addresses
  uint16_t baseAddr_ = DEFAULT_BASE_ADDR;
  uint16_t dcuBaseAddr_ = DEFAULT_DCU_BASE_ADDR;

  // Identification and Status Data
  uint32_t serialNumber_ = 0;
  uint32_t deviceID_ = 0;
  uint8_t receiveErrorCount_ = 0;
  uint8_t transmitErrorCount_ = 0;
  uint16_t activeMotor_ = 0;
  uint16_t errorFlags_ = 0;
  uint16_t limitFlags_ = 0;

  // Electrical Measurements (in SI units)
  float busCurrent_ = 0.0f;      // Amperes
  float busVoltage_ = 0.0f;      // Volts
  float vehicleVelocity_ = 0.0f; // m/s
  float motorVelocity_ = 0.0f;   // RPM
  float phaseCCurrent_ = 0.0f;   // Amperes
  float phaseBCurrent_ = 0.0f;   // Amperes
  float dVoltage_ = 0.0f;        // Volts (D-axis)
  float qVoltage_ = 0.0f;        // Volts (Q-axis)
  float dCurrent_ = 0.0f;        // Amperes (D-axis)
  float qCurrent_ = 0.0f;        // Amperes (Q-axis)
  float dBackEMF_ = 0.0f;        // Volts (D-axis)
  float qBackEMF_ = 0.0f;        // Volts (Q-axis)

  // Power Supply Measurements
  float measured15VSupply_ = 0.0f; // Volts
  float measured3V3Supply_ = 0.0f; // Volts
  float measured1V9Supply_ = 0.0f; // Volts

  // Temperature Measurements
  float heatsinkTemp_ = 0.0f; // Celsius
  float motorTemp_ = 0.0f;    // Celsius
  float dspBoardTemp_ = 0.0f; // Celsius

  // Energy/Distance Measurements
  float dcBusAh_ = 0.0f;   // Ampere-hours
  float odometer_ = 0.0f;  // Kilometers
  float slipSpeed_ = 0.0f; // m/s

  // Error tracking
  HAL_StatusTypeDef lastError_ = HAL_OK;

  /**
   * @brief Helper to construct CAN transmit header for DCU commands
   */
  CAN_TxHeaderTypeDef dcuBaseTxHeader_(WaveSculptor::DCUMessageID id) const {
    return CAN_TxHeaderTypeDef{.StdId = static_cast<uint32_t>(dcuBaseAddr_ + static_cast<uint16_t>(id)),
                               .ExtId = 0,
                               .IDE = CAN_ID_STD,
                               .RTR = CAN_RTR_DATA,
                               .DLC = 8,
                               .TransmitGlobalTime = DISABLE};
  }

  /**
   * @brief Helper to construct CAN transmit header for measurement requests
   */
  CAN_TxHeaderTypeDef requestBaseTxHeader_(WaveSculptor::MessageID id) const {
    return CAN_TxHeaderTypeDef{.StdId = static_cast<uint32_t>(baseAddr_ + static_cast<uint16_t>(id)),
                               .ExtId = 0,
                               .IDE = CAN_ID_STD,
                               .RTR = CAN_RTR_REMOTE,
                               .DLC = 0,
                               .TransmitGlobalTime = DISABLE};
  }

public:
  /**
   * @brief Construct a new WaveSculptor motor controller interface
   *
   * @param hcan Pointer to FDCAN handle
   * @param baseAddr Base CAN ID for measurement messages (default 0x400)
   * @param dcuBaseAddr Base CAN ID for DCU command messages (default 0x500)
   */
  WaveSculptor(CAN_HandleTypeDef *hcan, uint16_t baseAddr = DEFAULT_BASE_ADDR, uint16_t dcuBaseAddr = DEFAULT_DCU_BASE_ADDR);

  /**
   * @brief Initialize or reinitialize the CAN handle and base addresses.
   *
   * Only must be called if the `WaveSculptor` object is initialized before the CAN peripheral.
   *
   * @param hcan Pointer to FDCAN handle
   * @param baseAddr Base CAN ID for measurement messages (default 0x400)
   * @param dcuBaseAddr Base CAN ID for DCU command messages (default 0x500)
   */
  void init(CAN_HandleTypeDef *hcan, uint16_t baseAddr = DEFAULT_BASE_ADDR, uint16_t dcuBaseAddr = DEFAULT_DCU_BASE_ADDR);

  /**
   * @brief Send motor drive command (motor current and RPM setpoint)
   *
   * @param motorCurrent Motor current command as percentage [0-100]
   * @param motorRPM Motor RPM setpoint
   * @return HAL_StatusTypeDef - HAL_OK if message queued successfully
   */
  HAL_StatusTypeDef sendMotorDrive(float motorCurrent, float motorRPM);

  /**
   * @brief Send motor power command (bus current limit)
   *
   * @param busCurrent Bus current command as percentage [0-100]
   * @return HAL_StatusTypeDef - HAL_OK if message queued successfully
   */
  HAL_StatusTypeDef sendMotorPower(float busCurrent);

  /**
   * @brief Send reset command to motor controller
   *
   * @return HAL_StatusTypeDef - HAL_OK if message queued successfully
   */
  HAL_StatusTypeDef sendReset();

  /**
   * @brief Helper template to send measurement requests (non-blocking, queues to TX FIFO)
   *
   * Usage: requestMeasurement<WaveSculptor::MessageID::Identification>()
   * or:    requestMeasurement<WaveSculptor::MessageID::Status>()
   *
   * Available MessageID values: Identification, Status, Bus, Velocity, PhaseCurrent,
   * MotorVoltageVector, MotorCurrentVector, MotorBackEMF, VoltageRail15V, VoltageRail3V3_1V9,
   * HeatSinkMotorTemp, DSPTemp, OdometerBusAh, SlipSpeed
   *
   * @tparam MsgID Message ID to request
   * @return HAL_StatusTypeDef - HAL_OK if request queued successfully
   */
  template <WaveSculptor::MessageID MsgID> HAL_StatusTypeDef requestMeasurement() {
    if (!isInitialized()) {
      lastError_ = HAL_ERROR;
      return lastError_;
    }
#if WS_DEBUG_ENABLED
    printf("WS: Requesting measurement 0x%02X\n", baseAddr_ + static_cast<uint8_t>(MsgID));
#endif
    CAN_TxHeaderTypeDef txHeader = requestBaseTxHeader_(MsgID);
    uint8_t txData[0] = {};
    uint32_t pTxMailbox;
    lastError_ = HAL_CAN_AddTxMessage(hcan_, &txHeader, txData, &pTxMailbox);
    return lastError_;
  }

  /**
   * @brief Parse received CAN measurement message and update internal state
   *
   * @param id CAN message ID offset as enum
   * @param rxData Pointer to received 8-byte CAN data
   * @return HAL_StatusTypeDef - HAL_OK on successful parse, HAL_ERROR if invalid
   */
  HAL_StatusTypeDef parseMeasurement(WaveSculptor::MessageID id, const uint8_t *rxData);

  uint32_t getSerialNumber() const { return serialNumber_; }
  uint32_t getDeviceID() const { return deviceID_; }
  uint8_t getReceiveErrorCount() const { return receiveErrorCount_; }
  uint8_t getTransmitErrorCount() const { return transmitErrorCount_; }
  uint16_t getActiveMotor() const { return activeMotor_; }
  uint16_t getErrorFlags() const { return errorFlags_; }
  uint16_t getLimitFlags() const { return limitFlags_; }

  float getBusCurrent() const { return busCurrent_; }
  float getBusVoltage() const { return busVoltage_; }
  float getVehicleVelocity() const { return vehicleVelocity_; }
  float getMotorVelocity() const { return motorVelocity_; }
  float getPhaseBCurrent() const { return phaseBCurrent_; }
  float getPhaseCCurrent() const { return phaseCCurrent_; }
  float getDVoltage() const { return dVoltage_; }
  float getQVoltage() const { return qVoltage_; }
  float getDCurrent() const { return dCurrent_; }
  float getQCurrent() const { return qCurrent_; }
  float getDBackEMF() const { return dBackEMF_; }
  float getQBackEMF() const { return qBackEMF_; }

  float getMeasured15VSupply() const { return measured15VSupply_; }
  float getMeasured3V3Supply() const { return measured3V3Supply_; }
  float getMeasured1V9Supply() const { return measured1V9Supply_; }

  float getHeatsinkTemp() const { return heatsinkTemp_; }
  float getMotorTemp() const { return motorTemp_; }
  float getDSPBoardTemp() const { return dspBoardTemp_; }

  float getDCBusAh() const { return dcBusAh_; }
  float getOdometer() const { return odometer_; }
  float getSlipSpeed() const { return slipSpeed_; }

  uint16_t getBaseAddr() const { return baseAddr_; }
  uint16_t getDcuBaseAddr() const { return dcuBaseAddr_; }
  HAL_StatusTypeDef getLastError() const { return lastError_; }
  bool isInitialized() const { return hcan_ != nullptr; }

  /**
   * @brief Check if a CAN ID is within the WaveSculptor message range
   *
   * @param id CAN message ID
   * @return true if ID is within WaveSculptor base address range
   */
  bool isWaveSculptorMessage(uint32_t id) const {
    return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(WaveSculptor::MessageID::SlipSpeed);
  }

  /**
   * @brief Check if a CAN ID is within the WaveSculptor DCU command range
   *
   * @param id CAN message ID
   * @return true if ID is within WaveSculptor DCU address range
   */
  bool isWaveSculptorDCUMessage(uint32_t id) const { return id >= dcuBaseAddr_ && id <= dcuBaseAddr_ + static_cast<uint16_t>(DCUMessageID::Reset); }
};

#endif /* WS_CAN_COMMS_HPP */
