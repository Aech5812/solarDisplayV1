#ifndef WS_CAN_COMMS_HPP
#define WS_CAN_COMMS_HPP

#include "stm32l4xx_hal.h"
#include <cstdio>

// Debug toggle - set to true to enable debug prints, false to disable
#define WS_DEBUG_ENABLED false

class WaveSculptor {
public:
  static constexpr uint16_t DEFAULT_BASE_ADDR = 0x400;
  static constexpr uint16_t DEFAULT_DCU_BASE_ADDR = 0x500;

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

  enum class DCUMessageID : uint16_t { MotorDrive = 0x01, MotorPower = 0x02, Reset = 0x02 };

private:
  CAN_HandleTypeDef *hcan_ = nullptr;
  uint16_t baseAddr_ = DEFAULT_BASE_ADDR;
  uint16_t dcuBaseAddr_ = DEFAULT_DCU_BASE_ADDR;

  uint32_t serialNumber_ = 0;
  uint32_t deviceID_ = 0;
  uint8_t receiveErrorCount_ = 0;
  uint8_t transmitErrorCount_ = 0;
  uint16_t activeMotor_ = 0;
  uint16_t errorFlags_ = 0;
  uint16_t limitFlags_ = 0;

  float busCurrent_ = 0.0f;
  float busVoltage_ = 0.0f;
  float vehicleVelocity_ = 0.0f;
  float motorVelocity_ = 0.0f;
  float phaseCCurrent_ = 0.0f;
  float phaseBCurrent_ = 0.0f;
  float dVoltage_ = 0.0f;
  float qVoltage_ = 0.0f;
  float dCurrent_ = 0.0f;
  float qCurrent_ = 0.0f;
  float dBackEMF_ = 0.0f;
  float qBackEMF_ = 0.0f;
  float measured15VSupply_ = 0.0f;
  float measured3V3Supply_ = 0.0f;
  float measured1V9Supply_ = 0.0f;
  float heatsinkTemp_ = 0.0f;
  float motorTemp_ = 0.0f;
  float dspBoardTemp_ = 0.0f;
  float dcBusAh_ = 0.0f;
  float odometer_ = 0.0f;
  float slipSpeed_ = 0.0f;

  HAL_StatusTypeDef lastError_ = HAL_OK;

  CAN_TxHeaderTypeDef dcuBaseTxHeader_(WaveSculptor::DCUMessageID id) const {
    return CAN_TxHeaderTypeDef{.StdId = static_cast<uint32_t>(dcuBaseAddr_ + static_cast<uint16_t>(id)),
                               .ExtId = 0,
                               .IDE = CAN_ID_STD,
                               .RTR = CAN_RTR_DATA,
                               .DLC = 8,
                               .TransmitGlobalTime = DISABLE};
  }

  CAN_TxHeaderTypeDef requestBaseTxHeader_(WaveSculptor::MessageID id) const {
    return CAN_TxHeaderTypeDef{.StdId = static_cast<uint32_t>(baseAddr_ + static_cast<uint16_t>(id)),
                               .ExtId = 0,
                               .IDE = CAN_ID_STD,
                               .RTR = CAN_RTR_REMOTE,
                               .DLC = 0,
                               .TransmitGlobalTime = DISABLE};
  }

public:
  WaveSculptor(CAN_HandleTypeDef *hcan, uint16_t baseAddr = DEFAULT_BASE_ADDR, uint16_t dcuBaseAddr = DEFAULT_DCU_BASE_ADDR);
  void init(CAN_HandleTypeDef *hcan, uint16_t baseAddr = DEFAULT_BASE_ADDR, uint16_t dcuBaseAddr = DEFAULT_DCU_BASE_ADDR);
  HAL_StatusTypeDef sendMotorDrive(float motorCurrent, float motorRPM);
  HAL_StatusTypeDef sendMotorPower(float busCurrent);
  HAL_StatusTypeDef sendReset();

  template <WaveSculptor::MessageID MsgID> HAL_StatusTypeDef requestMeasurement() {
    if (!isInitialized()) {
      lastError_ = HAL_ERROR;
      return lastError_;
    }
    CAN_TxHeaderTypeDef txHeader = requestBaseTxHeader_(MsgID);
    uint8_t txData[0] = {};
    uint32_t pTxMailbox;
    lastError_ = HAL_CAN_AddTxMessage(hcan_, &txHeader, txData, &pTxMailbox);
    return lastError_;
  }

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

  bool isWaveSculptorMessage(uint32_t id) const {
    return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(WaveSculptor::MessageID::SlipSpeed);
  }

  bool isWaveSculptorDCUMessage(uint32_t id) const { return id >= dcuBaseAddr_ && id <= dcuBaseAddr_ + static_cast<uint16_t>(DCUMessageID::Reset); }
};

#endif /* WS_CAN_COMMS_HPP */