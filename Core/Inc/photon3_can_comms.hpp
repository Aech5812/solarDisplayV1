/**
 * @file photon3_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for Photon3 CAN communications class
 * @version 0.2
 * @date 2026-03-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef PHOTON3_CAN_COMMS_HPP
#define PHOTON3_CAN_COMMS_HPP

#include "stm32l4xx_hal.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define PHOTON3_DEBUG_ENABLED false

/**
 * @brief Class for Photon3 Solar MPPT CAN Communications
 *
 * Provides synchronous parsing of received CAN messages with state storage.
 * Measurements are updated when new CAN data is received via parseMeasurement().
 */
class Photon3 {
public:
  static constexpr uint16_t DEFAULT_BASE_ADDR = 0x600;

  /**
   * @brief Enum for Photon3 Message IDs (offsets from base address)
   */
  enum class MessageID : uint16_t { Channel1 = 0x00, Channel2 = 0x01 };

  /**
   * @brief Construct a new Photon3 object
   *
   * @param hfdcan Pointer to FDCAN handle
   * @param baseAddr Base address for Photon3 messages (default 0x600)
   */
  Photon3(CAN_HandleTypeDef *hfdcan, uint16_t baseAddr = DEFAULT_BASE_ADDR);

  /**
   * @brief Parse received CAN measurement data and update internal state
   *
   * @param id CAN message ID offset as enum
   * @param rxData Pointer to received 8-byte CAN data
   * @return HAL_StatusTypeDef - HAL_OK on successful parse
   */
  HAL_StatusTypeDef parseMeasurement(Photon3::MessageID id, const uint8_t *rxData);

  /**
   * @brief Check if a CAN ID is within the Photon3 message range
   *
   * @param id CAN message ID
   * @return true if ID is within Photon3 range
   */
  bool isPhoton3Message(uint32_t id) const { return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(Photon3::MessageID::Channel2); }

  // Channel 1 Getters (10mV/LSB, 1mA/LSB, 10m°C/LSB)

  //
  uint16_t getCh1ArrayVoltage() const { return ch1ArrayVoltage_; }
  uint16_t getCh1ArrayCurrent() const { return ch1ArrayCurrent_; }
  uint16_t getCh1BatteryVoltage() const { return ch1BatteryVoltage_; }
  uint16_t getCh1UnitTemp() const { return ch1UnitTemp_; }

  // Channel 2 Getters (10mV/LSB, 1mA/LSB, 10m°C/LSB)

  //
  uint16_t getCh2ArrayVoltage() const { return ch2ArrayVoltage_; }
  uint16_t getCh2ArrayCurrent() const { return ch2ArrayCurrent_; }
  uint16_t getCh2BatteryVoltage() const { return ch2BatteryVoltage_; }
  uint16_t getCh2UnitTemp() const { return ch2UnitTemp_; }

  uint16_t getBaseAddr() const { return baseAddr_; }
  HAL_StatusTypeDef getLastError() const { return lastError_; }

private:
  [[maybe_unused]] CAN_HandleTypeDef *hfdcan_; // Kept for potential future use
  uint16_t baseAddr_;                            // Used for message range checking

  // Measurement Data - Channel 1
  uint16_t ch1ArrayVoltage_ = 0;   // 10mV per unit
  uint16_t ch1ArrayCurrent_ = 0;   // 1mA per unit
  uint16_t ch1BatteryVoltage_ = 0; // 10mV per unit
  uint16_t ch1UnitTemp_ = 0;       // 10m°C per unit

  // Measurement Data - Channel 2
  uint16_t ch2ArrayVoltage_ = 0;   // 10mV per unit
  uint16_t ch2ArrayCurrent_ = 0;   // 1mA per unit
  uint16_t ch2BatteryVoltage_ = 0; // 10mV per unit
  uint16_t ch2UnitTemp_ = 0;       // 10m°C per unit

  // Error tracking
  HAL_StatusTypeDef lastError_ = HAL_OK;
};

#endif /* PHOTON3_CAN_COMMS_HPP */
