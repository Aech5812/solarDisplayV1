/**
 * @file orion2_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of Orion2 BMS CAN communications class
 * @version 0.1
 * @date 2026-03-31
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "orion2_can_comms.hpp"
#include "uart_guard.hpp"
#include <cstdio>
#include <cstring>

Orion2::Orion2(CAN_HandleTypeDef *hfdcan, uint16_t baseAddr) : hfdcan_(hfdcan), baseAddr_(baseAddr) {}

HAL_StatusTypeDef Orion2::parseMeasurement(Orion2::MessageID id, const uint8_t *rxData) {
  if (rxData == nullptr) {
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  lastError_ = HAL_OK;

  switch (id) {
  case MessageID::PackMeasurements: {
    // Extract raw mixed-width little-endian values and scale to floats
    int16_t rawCurrent;
    uint16_t rawInstVoltage, rawOpenVoltage;
    uint8_t rawSOC;

    memcpy(&rawCurrent, &rxData[0], sizeof(rawCurrent));
    memcpy(&rawInstVoltage, &rxData[2], sizeof(rawInstVoltage));
    memcpy(&rawOpenVoltage, &rxData[4], sizeof(rawOpenVoltage));
    memcpy(&rawSOC, &rxData[6], sizeof(rawSOC));

    // Scale raw values to floats
    packCurrent_ = rawCurrent * 0.1f;
    packInstVoltage_ = rawInstVoltage * 0.1f;
    packOpenVoltage_ = rawOpenVoltage * 0.1f;
    packSOC_ = rawSOC * 0.5f;

#if ORION2_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("Orion2: Pack Measurements - Current: %.1f A, InstVolt: %.1f V, OpenVolt: %.1f V, SOC: %.1f %%\n", packCurrent_, packInstVoltage_,
             packOpenVoltage_, packSOC_);
    }
#endif
    break;
  }
  case MessageID::PackExtended: {
    // Extract raw 16-bit little-endian values and scale to floats
    uint16_t rawCapacity, rawPower, rawTemp, rawFanVoltage;

    memcpy(&rawCapacity, &rxData[0], sizeof(rawCapacity));
    memcpy(&rawPower, &rxData[2], sizeof(rawPower));
    memcpy(&rawTemp, &rxData[4], sizeof(rawTemp));
    memcpy(&rawFanVoltage, &rxData[6], sizeof(rawFanVoltage));

    // Scale raw values to floats
    totalPackCapacity_ = rawCapacity * 0.1f;
    packKWPower_ = rawPower * 0.1f;
    intakeTemperature_ = rawTemp * 1.0f; // Direct conversion to float
    fanVoltage_ = rawFanVoltage * 0.01f;

#if ORION2_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("Orion2: Pack Extended - Capacity: %.1f Ah, Power: %.1f kW, Intake Temp: %.1f °C, Fan Voltage: %.2f V\n", totalPackCapacity_,
             packKWPower_, intakeTemperature_, fanVoltage_);
    }
#endif
    break;
  }
  case MessageID::CellVoltage: {
    // Extract cell voltage broadcast data
    // Note: DBC uses motorola/intel byte order for different signals
    uint16_t rawCellVoltage, rawCellResistance, rawCellOpenVoltage;

    cellID_ = rxData[0];                                                 // Byte 0: Cell ID
    memcpy(&rawCellVoltage, &rxData[1], sizeof(rawCellVoltage));         // Bytes 1-2: Cell Voltage
    memcpy(&rawCellResistance, &rxData[3], sizeof(rawCellResistance));   // Bytes 3-4: Cell Resistance
    memcpy(&rawCellOpenVoltage, &rxData[5], sizeof(rawCellOpenVoltage)); // Bytes 5-6: Cell Open Voltage
    cellBalancing_ = (rxData[4] >> 7) & 0x01;                            // Bit 7 of byte 4: Cell Balancing flag
    checksum_ = rxData[7];                                               // Byte 7: Checksum

    // Scale raw values to floats
    cellVoltage_ = rawCellVoltage * 0.0001f;
    cellResistance_ = rawCellResistance * 0.01f;
    cellOpenVoltage_ = rawCellOpenVoltage * 0.0001f;

#if ORION2_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("Orion2: Cell Voltage - CellID: %u, Voltage: %.4f V, Resistance: %.2f mOhm, OpenVolt: %.4f V, Balancing: %u, CRC: 0x%02X\n", cellID_,
             cellVoltage_, cellResistance_, cellOpenVoltage_, cellBalancing_, checksum_);
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