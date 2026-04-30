/**
 * @file orion2_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for Orion2 BMS CAN communications class
 * @version 0.1
 * @date 2026-03-31
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ORION2_CAN_COMMS_HPP
#define ORION2_CAN_COMMS_HPP

#include "stm32l4xx_hal.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define ORION2_DEBUG_ENABLED false

/**
 * @brief Class for Orion2 BMS CAN Communications
 *
 * Provides synchronous parsing of received CAN messages with state storage.
 * Measurements are updated when new CAN data is received via parseMeasurement().
 */
class Orion2 {
public:
  static constexpr uint16_t DEFAULT_BASE_ADDR = 0x6B0;

  /**
   * @brief Enum for Orion2 Message IDs (offsets from base address, except CellVoltage)
   */
  enum class MessageID : uint16_t {
    PackMeasurements = 0x02, // 0x6B2 - Pack voltage, current, and SOC
    PackExtended = 0x03,     // 0x6B3 - Pack capacity, power, temperature, fan speed
    CellVoltage = 0x36       // 0x36 - Cell voltage broadcast (separate address, not an offset)
  };

  /**
   * @brief Construct a new Orion2 object
   *
   * @param hfdcan Pointer to FDCAN handle
   * @param baseAddr Base address for Orion2 messages (default 0x6B0)
   */
  Orion2(CAN_HandleTypeDef *hfdcan, uint16_t baseAddr = DEFAULT_BASE_ADDR);

  /**
   * @brief Parse received CAN measurement data and update internal state
   *
   * @param id CAN message ID offset as enum
   * @param rxData Pointer to received 8-byte CAN data
   * @return HAL_StatusTypeDef - HAL_OK on successful parse
   */
  HAL_StatusTypeDef parseMeasurement(Orion2::MessageID id, const uint8_t *rxData);

  /**
   * @brief Check if a CAN ID is within the Orion2 message range
   *
   * @param id CAN message ID
   * @return true if ID is within Orion2 range
   */
  bool isOrion2Message(uint32_t id) const {
    // Check offset-based messages (0x6B2, 0x6B3)
    if (id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(Orion2::MessageID::PackExtended)) {
      return true;
    }
    // Check cell voltage message (0x36 - separate address)
    return id == 0x36;
  }

  // Pack Measurements Getters (scaled float values)
  float getPackCurrent() const { return packCurrent_; }         // Amperes
  float getPackInstVoltage() const { return packInstVoltage_; } // Volts
  float getPackOpenVoltage() const { return packOpenVoltage_; } // Volts
  float getPackSOC() const { return packSOC_; }                 // Percent

  // Pack Extended Measurements Getters (scaled float values)
  float getPackCapacity() const { return totalPackCapacity_; } // Ampere-hours
  float getPackPower() const { return packKWPower_; }          // Kilowatts
  float getIntakeTemp() const { return intakeTemperature_; }   // Celsius
  float getFanVoltage() const { return fanVoltage_; }          // Volts

  // Cell Voltage Broadcast Getters
  uint8_t getCellID() const { return cellID_; }                    // Cell ID
  float getCellVoltage() const { return cellVoltage_; }            // Volts
  float getCellResistance() const { return cellResistance_; }      // milliOhms
  float getCellOpenVoltage() const { return cellOpenVoltage_; }    // Volts
  uint8_t getCellBalancing() const { return cellBalancing_; }      // Boolean flag
  uint8_t getChecksum() const { return checksum_; }                // CRC checksum

  uint16_t getBaseAddr() const { return baseAddr_; }
  HAL_StatusTypeDef getLastError() const { return lastError_; }

private:
  [[maybe_unused]] CAN_HandleTypeDef *hfdcan_; // Kept for potential future use
  uint16_t baseAddr_;                            // Used for message range checking

  // Pack Measurements (MessageID::PackMeasurements - 0x6B2)
  float packCurrent_ = 0.0f;     // Amperes (scaled)
  float packInstVoltage_ = 0.0f; // Volts (scaled)
  float packOpenVoltage_ = 0.0f; // Volts (scaled)
  float packSOC_ = 0.0f;         // Percent (scaled)

  // Pack Extended Measurements (MessageID::PackExtended - 0x6B3)
  float totalPackCapacity_ = 0.0f; // Ampere-hours (scaled)
  float packKWPower_ = 0.0f;       // Kilowatts (scaled)
  float intakeTemperature_ = 0.0f; // Celsius (scaled)
  float fanVoltage_ = 0.0f;        // Volts (scaled)

  // Cell Voltage Broadcast (MessageID::CellVoltage - 0x36)
  uint8_t cellID_ = 0;           // Cell ID (0-255)
  float cellVoltage_ = 0.0f;     // Volts (scaled)
  float cellResistance_ = 0.0f;  // milliOhms (scaled)
  float cellOpenVoltage_ = 0.0f; // Volts (scaled)
  uint8_t cellBalancing_ = 0;    // Boolean flag (0 or 1)
  uint8_t checksum_ = 0;         // CRC checksum

  // Error tracking
  HAL_StatusTypeDef lastError_ = HAL_OK;
};

#endif /* ORION2_CAN_COMMS_HPP */
