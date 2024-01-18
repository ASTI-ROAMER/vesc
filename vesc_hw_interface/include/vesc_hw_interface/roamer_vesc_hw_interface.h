/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#ifndef XR1_HW_INTERFACE_VESC_HW_INTERFACE_H_
#define XR1_HW_INTERFACE_VESC_HW_INTERFACE_H_

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <set>
#include <array>
#include <string>

#include <angles/angles.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <urdf_model/types.h>
#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.hpp>
#include <boost/thread/thread.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_packet_factory.h"
#include "vesc_hw_interface/vesc_servo_controller.h"
#include "vesc_hw_interface/vesc_wheel_controller.h"

#include "vesc_driver/data_map_leg_encoder.h"

// #define DEBUG_RANDEL 0

namespace vesc_hw_interface
{
using vesc_driver::VescInterface;
using vesc_driver::VescPacket;
using vesc_driver::VescPacketValues;
using vesc_driver::VescPacketPingedCanIDs;
using vesc_driver::VescPacketAppConf;


class xr1PoweredMotor
{
public:
  xr1PoweredMotor(){};
  xr1PoweredMotor(std::string joint_name, std::string command_mode, uint8_t vesc_id, int local=false, int mock=false): 
      joint_type_(urdf::Joint::CONTINUOUS), joint_name_(joint_name), command_mode_(command_mode), vesc_id_(vesc_id), is_local_(local), is_mock_(mock),
      cmd(0.0), pos(0.0),  vel(0.0), eff(0.0),
      num_rotor_poles_(14), num_hall_sensors_(3), gear_ratio_(1.0), torque_const_(1.0), screw_lead_(1.0) {};
  ~xr1PoweredMotor(){};

  int joint_type_;
  std::string joint_name_, command_mode_;
  uint8_t vesc_id_;                    // also the can bus id
  
  int is_local_, is_mock_;

  // stuff for commands for ros control
  double cmd, pos, vel, eff;          // command, position, velocity, effort

  int num_rotor_poles_;               // the number of rotor poles
  int num_hall_sensors_;              // the number of hall sensors
  double gear_ratio_, torque_const_;  // physical params
  double screw_lead_;                 // linear distance (m) of 1 revolution

  joint_limits_interface::JointLimits joint_limits_;
  joint_limits_interface::PositionJointSaturationInterface limit_position_interface_;
  joint_limits_interface::VelocityJointSaturationInterface limit_velocity_interface_;
  joint_limits_interface::EffortJointSaturationInterface limit_effort_interface_;


  uint8_t vesc_id() const { return vesc_id_; }


};


class xr1JointSensor
{
public:
  static const int MAX_ENCODER_VAL=4095;
  xr1JointSensor(){};
  xr1JointSensor(std::string joint_name, uint16_t enc_zero_val_=0, bool reversed_=false): 
      joint_type_(urdf::Joint::CONTINUOUS), joint_name_(joint_name),
      dir_val(reversed_ ? -1 : 1), enc_zero_val(enc_zero_val_),enc_val(enc_zero_val), isEncUpToDate(0),
      pos(0.0),  vel(0.0), eff(0.0),
      gear_ratio_(1.0) {};
  ~xr1JointSensor(){};

  int joint_type_;
  std::string joint_name_;
  


  // stuff for commands for ros control
  int16_t dir_val;
  uint16_t enc_zero_val, enc_val;               // 0 radians encoder value, 12-bit encoder value
  uint8_t isEncUpToDate;                 // 0 if not updated, due to unreadable spi values
  double pos, vel, eff;               // command, position, velocity, effort

  double gear_ratio_;

  void setReversedDir(){
    dir_val = -1;
  }
  void setForwardDir(){
    dir_val = 1;
  }

  void setEncoderZeroValue(int zero_val){
    enc_zero_val = (uint16_t)zero_val;
  }

  int16_t zeroed_enc_val(){
    int16_t temp = (int16_t)enc_val - (int16_t)enc_zero_val;
    return temp > 0 ? temp : (int16_t)(MAX_ENCODER_VAL+temp);   // temp in false is negative
  };

  // zeroed and dir corrected encoder value
  int16_t zeroed_dir_enc_val(){
    return zeroed_enc_val() * dir_val;
  };

  // zeroed and normalized angle (pos) in radians
  double final_pos_rad(){
    return angles::normalize_angle(zeroed_dir_enc_val() * 2 * M_PI / MAX_ENCODER_VAL);
  };

  void get_final_pos_from_enc(){
    pos = final_pos_rad();
  };

  // joint_limits_interface::JointLimits joint_limits_;
};


class XR1VescHwInterface : public hardware_interface::RobotHW
{
public:
  XR1VescHwInterface();
  ~XR1VescHwInterface();

  bool init(ros::NodeHandle&, ros::NodeHandle&);
  void read(const ros::Time&, const ros::Duration&);
  void write(const ros::Time&, const ros::Duration&);
  ros::Time getTime() const;

private:
  VescInterface vesc_interface_;
  VescInterface leg_interface_;
  VescServoController servo_controller_;
  VescWheelController wheel_controller_;

  int direct_vesc_id_;
  std::vector<uint8_t> detected_vesc_ids;     // index 0 of this SHOULD BE THE direct_vesc_id_
  

  const std::string motor_names[4] = {"wheel_front_left_joint",
                                      "wheel_rear_left_joint",
                                      "wheel_front_right_joint",
                                      "wheel_rear_right_joint"};
  std::array<uint8_t, 4> motor_vesc_ids = {0};
  

  const std::string passive_w_names[2] = {"wheel_middle_left_joint",
                                          "wheel_middle_right_joint"};

  std::string rb_names[4] = {   "rocker_left_joint",
                                "bogie_left_joint",
                                "rocker_right_joint",
                                "bogie_right_joint"};
  // 4 actuators: (0)front-left, (1)rear-left, (2)front-right, (3)rear-right
  xr1PoweredMotor motors[4];
  xr1PoweredMotor passive_wheels[2];

  std::map<uint8_t, xr1PoweredMotor*> idTomotor_ptr_map;

  // leg position sensor: (0)rocker left, (1)bogie left, (2)rocker right, (3)bogie right
  xr1JointSensor rb[4];
  bool use_only_valid_leg_encoder_values;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;
  

  void packetCallback(const std::shared_ptr<VescPacket const>&);
  void errorCallback(const std::string&);

  void legPacketCallback(const std::shared_ptr<VescPacket const>&);
  void legErrorCallback(const std::string&);

  void registerControlInterfaces(ros::NodeHandle& nh_root, ros::NodeHandle& nh);

  uint8_t verifyVescID(int _in, uint8_t _default);

};






}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_HW_INTERFACE_H_
