/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{
/**
 * @brief Constructor
 * @param payload_size Specified payload size
 **/
VescFrame::VescFrame(const int16_t payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256)
  {
    // single byte payload size
    frame_.resize(VESC_MIN_FRAME_SIZE + payload_size);
    *(frame_.begin()) = 2;
    *(frame_.begin() + 1) = payload_size;
    payload_end_.first = frame_.begin() + 2;
  }
  else
  {
    // two byte payload size
    frame_.resize(VESC_MIN_FRAME_SIZE + 1 + payload_size);
    *(frame_.begin()) = 3;
    *(frame_.begin() + 1) = payload_size >> 8;
    *(frame_.begin() + 2) = payload_size & 0xFF;
    payload_end_.first = frame_.begin() + 3;
  }

  payload_end_.second = payload_end_.first + payload_size;
  *(frame_.end() - 1) = 3;
}

/**
 * @brief Constructor
 * @param frame Reference of a buffer with constant range
 * @param payload_size Specified payload size
 **/
VescFrame::VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload)
{
  /* VescPacketFactory::createPacket() should make sure that
   *  the input is valid, but run a few cheap checks anyway */
  assert(boost::distance(frame) >= VESC_MIN_FRAME_SIZE);
  assert(boost::distance(frame) <= VESC_MAX_FRAME_SIZE);
  assert(boost::distance(payload) <= VESC_MAX_PAYLOAD_SIZE);
  assert(std::distance(frame.first, payload.first) > 0 && std::distance(payload.second, frame.second) > 0);

  frame_.resize(std::distance(boost::begin(frame), boost::end(frame)));
  frame_.assign(boost::begin(frame), boost::end(frame));
  payload_end_.first = frame_.begin() + std::distance(frame.first, payload.first);
  payload_end_.second = frame_.begin() + std::distance(frame.first, payload.second);
}


uint16_t VescFrame::payload_size() const{
  return std::distance(payload_end_.first, payload_end_.second);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param name Packet name
 * @param payload_size Specified payload size
 * @param payload_id ID of payload
 **/
VescPacket::VescPacket(const std::string& name, const int16_t payload_size, const int16_t payload_id)
  : VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(boost::distance(payload_end_) > 0);
  *payload_end_.first = payload_id;
}

/**
 * @brief Constructor
 * @param name Packet name
 * @param raw Pointer of a frame
 **/
VescPacket::VescPacket(const std::string& name, std::shared_ptr<VescFrame> raw) : VescFrame(*raw), name_(name)
{
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == frame_.front())
  {
    // payload size field is one byte
    payload_end_.first = frame_.begin() + 2;
    payload_end_.second = payload_end_.first + *(frame_.begin() + 1);
  }
  else
  {
    assert(VescFrame::VESC_SOF_VAL_LARGE_FRAME == frame_.front());
    // payload size field is two bytes
    payload_end_.first = frame_.begin() + 3;
    payload_end_.second = payload_end_.first + (*(frame_.begin() + 1) << 8) + *(frame_.begin() + 2);
  }
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param raw Pointer of VescFrame
 **/
VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw) : VescPacket("FWVersion", raw)
{
}

/**
 * @brief Gets major farmware version
 * @return Major farmware version
 **/
int16_t VescPacketFWVersion::fwMajor() const
{
  return *(payload_end_.first + 1);
}

/**
 * @brief Gets minor farmware version
 * @return Minor farmware version
 **/
int16_t VescPacketFWVersion::fwMinor() const
{
  return *(payload_end_.first + 2);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestFWVersion::VescPacketRequestFWVersion() : VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw) : VescPacket("Values", raw)
{
}

/**
 * @brief Gets temperature of MOSFETs
 * @return Temperature of MOSFETs
 **/
double VescPacketValues::getMosTemp() const
{
  return readBuffer(TEMP_MOS, 2) / 10.0;
}

/**
 * @brief Gets temperature of the motor
 * @return Temperature of the motor
 **/
double VescPacketValues::getMotorTemp() const
{
  return readBuffer(TEMP_MOTOR, 2) / 10.0;
}

/**
 * @brief Gets motor current
 * @return Motor current
 **/
double VescPacketValues::getMotorCurrent() const
{
  return readBuffer(CURRENT_MOTOR, 4) / 100.0;
}

/**
 * @brief Gets input current
 * @return Input current
 **/
double VescPacketValues::getInputCurrent() const
{
  return readBuffer(CURRENT_IN, 4) / 100.0;
}

/**
 * @brief Gets input current
 * @return Input current
 **/
double VescPacketValues::getDirectAxisCurrent() const
{
  return readBuffer(ID, 4) / 100.0;
}

/**
 * @brief Gets input current
 * @return Input current
 **/
double VescPacketValues::getQuadratureAxisCurrent() const
{
  return readBuffer(IQ, 4) / 100.0;
}

/**
 * @brief Gets the current duty value
 * @return The current duty value
 **/
double VescPacketValues::getDuty() const
{
  int16_t duty_raw = static_cast<int32_t>(readBuffer(DUTY_NOW, 2));

  // inverts to derive a negative value
  if (duty_raw > 1000)
  {
    duty_raw = !duty_raw;
  }

  return static_cast<double>(duty_raw) / 1000.0;
}

/**
 * @brief Gets the current angular velocity
 * @return The current angular velocity
 **/
double VescPacketValues::getVelocityERPM() const
{
  return readBuffer(ERPM, 4);
}

/**
 * @brief Gets input voltage
 * @return Input voltage
 **/
double VescPacketValues::getInputVoltage() const
{
  return readBuffer(VOLTAGE_IN, 2) / 10.0;
}

/**
 * @brief Gets consumed charge
 * @return Consumed charge
 **/
double VescPacketValues::getConsumedCharge() const
{
  return readBuffer(AMP_HOURS, 4) / 10000.0;
}

/**
 * @brief Gets input charge
 * @return Input charge
 **/
double VescPacketValues::getInputCharge() const
{
  return readBuffer(AMP_HOURS_CHARGED, 4) / 10000.0;
}

/**
 * @brief Gets consumed power
 * @return The amount of watt hours drawn. / Consumed power.
 **/
double VescPacketValues::getConsumedPower() const
{
  return readBuffer(WATT_HOURS, 4) / 10000.0;
}

/**
 * @brief Gets input power
 * @return The amount of watt hours fed back.
 **/
double VescPacketValues::getInputPower() const
{
  return readBuffer(WATT_HOURS_CHARGED, 4) / 10000.0;
}

/**
 * @brief Gets the current position
 * @return The current position
 **/
double VescPacketValues::getPosition() const
{
  return readBuffer(TACHOMETER, 4);
}

/**
 * @brief Gets absolute displacement
 * @return Absolute displacement
 **/
double VescPacketValues::getDisplacement() const
{
  return readBuffer(TACHOMETER_ABS, 4);
}

/**
 * @brief Gets fault code
 * @return Fault code
 **/
int VescPacketValues::getFaultCode() const
{
  return static_cast<int32_t>(*(payload_end_.first + FAULT_CODE));
}


/**
 * @brief Gets the position of pid, in float degrees.
 * @return The position of pid
 **/
double VescPacketValues::getPIDPosNow() const
{
  return readBuffer(PID_POS_NOW, 4) / 1000000.0;
}


/**
 * @brief Gets VESC ID where the packet came from
 * @return uint8_t vesc id
 **/
uint8_t VescPacketValues::getVescID() const
{
  return static_cast<uint8_t>(*(payload_end_.first + VESC_ID));
}

/**
 * @brief Gets NTC temperature of mosfet 1.
 * @return The NTC temperature of mosfet 1
 **/
double VescPacketValues::getTempMos1() const
{
  return readBuffer(TEMP_MOS1, 4) / 10.0;
}

/**
 * @brief Gets NTC temperature of mosfet 2.
 * @return The NTC temperature of mosfet 2
 **/
double VescPacketValues::getTempMos2() const
{
  return readBuffer(TEMP_MOS2, 4) / 10.0;
}

/**
 * @brief Gets NTC temperature of mosfet 3.
 * @return The NTC temperature of mosfet 3
 **/
double VescPacketValues::getTempMos3() const
{
  return readBuffer(TEMP_MOS3, 4) / 10.0;
}

/**
 * @brief Gets the average direct axis motor voltage. (FOC only)
 * @return The average D axis voltage.
 **/
double VescPacketValues::getAveDirectAxisVoltage() const
{
  return readBuffer(VD_AVE, 4) / 1000.0;
}

/**
 * @brief Gets the average quadrature axis motor voltage. (FOC only)
 * @return The average Q axis voltage.
 **/
double VescPacketValues::getAveQuadratureAxisVoltage() const
{
  return readBuffer(VQ_AVE, 4) / 1000.0;
}

/**
 * @brief Reads a value from the buffer
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacketValues::readBuffer(const uint8_t map_id, const uint8_t size) const
{
  int32_t value = 0;
  switch (size)
  {
    case 2:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1));
      break;
    case 4:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 24);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1) << 16);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 2) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 3));
      break;
  }

  return static_cast<double>(value);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestValues::VescPacketRequestValues() : VescPacket("RequestFWVersion", 1, COMM_GET_VALUES)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetDuty::VescPacketSetDuty(double duty) : VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  // checks the range of duty
  if (duty > 1.0)
  {
    duty = 1.0;
  }
  else if (duty < -1.0)
  {
    duty = -1.0;
  }

  const int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetCurrent::VescPacketSetCurrent(double current) : VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  const int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake)
  : VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  const int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetVelocityERPM::VescPacketSetVelocityERPM(double vel_erpm) : VescPacket("SetERPM", 5, COMM_SET_ERPM)
{
  const int32_t v = static_cast<int32_t>(vel_erpm);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetPos::VescPacketSetPos(double pos) : VescPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  const int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos) : VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  uint16_t v = static_cast<uint16_t>(servo_pos * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}


VescPacketFwdToCAN::VescPacketFwdToCAN(const VescPacket &packet, uint8_t can_addr) : 
  VescPacket("ForwardCAN", packet.payload_size() + 2, COMM_FORWARD_CAN)
{
  uint16_t orig_pl_size = boost::distance(packet.payload_end_);

  // add can address, then copy contents from original payload
  *(payload_end_.first + 1) = static_cast<uint8_t>(can_addr);

  Buffer::iterator new_payload_start = payload_end_.first + 2;    // location in new payload to place the old payload in

  for (uint i = 0; i < orig_pl_size ; i++){
    *(new_payload_start + i) = *(packet.payload_end_.first + i);
  }


  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);


  // // RANDEL: DEBUG printout
  // std::cout << "************** OLD:\n";
  // for(auto it = packet.payload_end_.first; it != packet.payload_end_.second;  ++it){
  //   std::cout << unsigned(*it) << ", ";
  // }
  // std::cout << std::endl;
  // std::cout << "**** NEW:\n";
  // for(auto it = payload_end_.first; it != payload_end_.second;  ++it){
  //   std::cout << unsigned(*it) << ", ";
  // }
  // std::cout << "\n\n" << std::endl;
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor - Request app conf from uart connected vesc
 **/
VescGetAppConf::VescGetAppConf() : VescPacket("GetAppConf", 1, COMM_GET_APPCONF)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/
/**
 * @brief Constructor - VESC reply frame for its app conf
 * @param raw Pointer of VescFrame
 **/
VescPacketAppConf::VescPacketAppConf(std::shared_ptr<VescFrame> raw) : VescPacket("AppConf", raw)
{
  // #if DEBUG_RANDEL == 1
  //   std::cout << "############## RAW frame:" << std::endl;
  //   for(auto it = raw->getFrame().begin(); it != raw->getFrame().end();  ++it){
  //     std::cout << unsigned(*it) << ", ";
  //   }
  //   std::cout << "\n" << std::endl;

  //   // std::cout << "^^^^^^^^^^^^^^ RAW payload:" << std::endl;
  //   // for(auto it = raw->payload_end_.first; it != raw->payload_end_.second;  ++it){
  //   //   std::cout << unsigned(*it) << ", ";
  //   // }
  //   // std::cout << "\n\n" << std::endl;

  //   std::cout << "&&&&&&&&&&&&&& PROCESSED frame:" << std::endl;
  //   for(auto it = getFrame().begin(); it != getFrame().end();  ++it){
  //     std::cout << unsigned(*it) << ", ";
  //   }
  //   std::cout << "\n" << std::endl;

  //   std::cout << ">>>>>>>>>>>>>> PROCESSED payload:" << std::endl;
  //   for(auto it = payload_end_.first; it != payload_end_.second;  ++it){
  //     std::cout << unsigned(*it) << ", ";
  //   }
  //   std::cout << "\n\n" << std::endl;
  // #endif // DEBUG_RANDEL
}

uint8_t VescPacketAppConf::getVescID() const
{
  return static_cast<uint8_t>(*(payload_end_.first + 5));
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor - Packet for requesting retrun packets
 * @param raw Pointer of VescFrame
 **/
VescPingVescCanIDs::VescPingVescCanIDs() : VescPacket("CmdPingCanIDs", 1, COMM_PING_CAN)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}


/*------------------------------------------------------------------*/

/**
 * @brief Constructor - Reply packet of VESC that contains pinged VESC IDs/can addresses
 * @param raw Pointer of VescFrame
 **/
VescPacketPingedCanIDs::VescPacketPingedCanIDs(std::shared_ptr<VescFrame> raw) : VescPacket("PingedCanIDs", raw)
{
  int _count = payload_size() - 1;
  vesc_ids.reserve(_count);
  for(auto it = payload_end_.first + 1 ; it != payload_end_.second; ++it){
    vesc_ids.push_back(*it);
  }

  
}

// uint8_t VescPacketPingedCanIDs::getVescID() const
// {
//   return static_cast<uint8_t>(*(payload_end_.first + 5));
// }

/*------------------------------------------------------------------*/

/*
VescPacketSetDetect::VescPacketSetDetect(uint8_t mode) :
  VescPacket("SetDetect", 3, COMM_SET_DETECT)
{
  *(payload_end_.first + 1) = mode;

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}
*/

/*------------------------------------------------------------------*/
/**
 * @brief Constructor
 **/
VescPacketGeneric::VescPacketGeneric(std::shared_ptr<VescFrame> raw) : VescPacket("GenericPacket", raw)
{
  
}

/*------------------------------------------------------------------*//**
 * @brief Constructor
 **/
LegReqAllPos::LegReqAllPos() : VescPacket("LegAllPos", 1, COMM_LEG_ALL_POS)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}


/*------------------------------------------------------------------*//**
 * @brief Constructor
 **/
LegReqAllPosStatus::LegReqAllPosStatus() : VescPacket("LegAllPos", 1, COMM_LEG_ALL_POS_STATUS)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

}  // namespace vesc_driver
