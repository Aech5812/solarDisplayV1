/**
 * @file photon3_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of Photon3 CAN communications class based on
 * dilithium power systems' documentation at
 * https://www.dilithiumpower.com/products/photon-3#h.p_8MVrI1wk4RlD
 * @version 0.2
 * @date 2026-03-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "photon3_can_comms.hpp"
#include "uart_guard.hpp"
#include <cstdio>
#include <cstring>

Photon3::Photon3(CAN_HandleTypeDef *hfdcan, uint16_t baseAddr) : hfdcan_(hfdcan), baseAddr_(baseAddr) {}

HAL_StatusTypeDef Photon3::parseMeasurement(Photon3::MessageID id, const uint8_t *rxData) {
  if (rxData == nullptr) {
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  lastError_ = HAL_OK;

  switch (id) {
  case MessageID::Channel1: {
    // Extract 16-bit little-endian values from received data
    memcpy(&ch1ArrayVoltage_, &rxData[0], sizeof(ch1ArrayVoltage_));     // 10mV per unit
    memcpy(&ch1ArrayCurrent_, &rxData[2], sizeof(ch1ArrayCurrent_));     // 1mA per unit
    memcpy(&ch1BatteryVoltage_, &rxData[4], sizeof(ch1BatteryVoltage_)); // 10mV per unit
    memcpy(&ch1UnitTemp_, &rxData[6], sizeof(ch1UnitTemp_));             // 10m°C per unit
#if PHOTON3_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("Photon3: Channel 1 - ArrayVolts: %.2f V, ArrayCurrent: %.2f A, BatVolts: %.2f V, Temp: %.2f °C\n", ch1ArrayVoltage_ * 0.01f,
             ch1ArrayCurrent_ * 0.001f, ch1BatteryVoltage_ * 0.01f, ch1UnitTemp_ * 0.01f);
    }
#endif
    break;
  }
  case MessageID::Channel2: {
    // Extract 16-bit little-endian values from received data
    memcpy(&ch2ArrayVoltage_, &rxData[0], sizeof(ch2ArrayVoltage_));     // 10mV per unit
    memcpy(&ch2ArrayCurrent_, &rxData[2], sizeof(ch2ArrayCurrent_));     // 1mA per unit
    memcpy(&ch2BatteryVoltage_, &rxData[4], sizeof(ch2BatteryVoltage_)); // 10mV per unit
    memcpy(&ch2UnitTemp_, &rxData[6], sizeof(ch2UnitTemp_));             // 10m°C per unit
#if PHOTON3_DEBUG_ENABLED
    {
      UartGuard guard;
      printf("Photon3: Channel 2 - ArrayVolts: %.2f V, ArrayCurrent: %.2f A, BatVolts: %.2f V, Temp: %.2f °C\n", ch2ArrayVoltage_ * 0.01f,
             ch2ArrayCurrent_ * 0.001f, ch2BatteryVoltage_ * 0.01f, ch2UnitTemp_ * 0.01f);
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
