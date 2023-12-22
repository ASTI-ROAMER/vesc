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

#include "vesc_hw_interface/roamer_vesc_hw_interface.h"

namespace vesc_hw_interface
{
XR1VescHwInterface::XR1VescHwInterface()
  : vesc_interface_(std::string(), std::bind(&XR1VescHwInterface::packetCallback, this, std::placeholders::_1),
                    std::bind(&XR1VescHwInterface::errorCallback, this, std::placeholders::_1)), direct_vesc_id_(-1)
{
  detected_vesc_ids.reserve(4);
}

XR1VescHwInterface::~XR1VescHwInterface()
{
}

bool XR1VescHwInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
{
  // reads a port name to open
  std::string port;
  if (!nh.getParam("port", port))
  {
    ROS_FATAL("[XR1_VESC] VESC communication port parameter required.");
    ros::shutdown();
    return false;
  }

  // attempts to open the serial port
  try
  {
    vesc_interface_.connect(port);
  }
  catch (serial::SerialException exception)
  {
    ROS_FATAL("[XR1_VESC] Failed to connect to the VESC, %s.", exception.what());
    ros::shutdown();
    return false;
  }

  // Get VESC ID of direct uart connected VESC, refer to [packetCallback()] for placing the value
  ROS_INFO("[XR1_VESC] Requesting AppConf to get direct VESC ID...");
  vesc_interface_.send(vesc_driver::VescGetAppConf(), -1);

  unsigned int _count = 0;
  int ms_sleep = 2, _timeout_ms = 3000;
  while(direct_vesc_id_ == -1 ){
    boost::this_thread::sleep( boost::posix_time::milliseconds(ms_sleep) );
    if(ms_sleep * _count++ > _timeout_ms){
      ROS_FATAL("[XR1_VESC] Failed to get to VESC ID in %d ms.",_timeout_ms);
      ros::shutdown();
      return false;
    }
  }

  // Get VESC IDs/ CANBUS IDs of connected VESCs, refer to [packetCallback()] for placing the value
  ROS_INFO("[XR1_VESC] Requesting CANBUS connected VESC IDs...");
  vesc_interface_.send(vesc_driver::VescPingVescCanIDs(), -1);

  while(detected_vesc_ids.size() <= 1){
    boost::this_thread::sleep( boost::posix_time::milliseconds(ms_sleep) );
    if(ms_sleep * _count++ > _timeout_ms){
      ROS_INFO("[XR1_VESC] Failed to scan CANBUS ONLY connected VESCs in %d ms.",_timeout_ms);
      break;
    }
  }


  // **** RANDEL: START new implementation ****************
  // Get VESC IDs from parameter server
  int id0, id1, id2, id3;
  nh.param<int>("id_front_left", id0, 115);
  nh.param<int>("id_rear_left", id1, 23);
  nh.param<int>("id_front_right", id2, 127);
  nh.param<int>("id_rear_right", id3, 67);

  motor_vesc_ids[0] = verifyVescID(id0, 115);
  motor_vesc_ids[1] = verifyVescID(id1, 23);
  motor_vesc_ids[2] = verifyVescID(id2, 127);
  motor_vesc_ids[3] = verifyVescID(id3, 67);

  // Make sure the VESC IDs given by the user are unique
  std::set<uint8_t> check_unique_ids(motor_vesc_ids.begin(), motor_vesc_ids.end());
  if (check_unique_ids.size() != motor_vesc_ids.size()){
    ROS_FATAL("[XR1_VESC] The given VESC IDs for the wheels are not unique.");
    ros::shutdown();
    return false;
  }
  

  // create a main motor directly connected to UART
  for (int i=0; i < 4; i++){
    // No local vesc for now
    motors[i] = vesc_hw_interface::xr1PoweredMotor(motor_names[i], "velocity", uint8_t(motor_vesc_ids[i]), false, false);

    motors[i].joint_limits_.has_velocity_limits = true;
    motors[i].joint_limits_.max_velocity = 1000.0;
    motors[i].joint_limits_.has_effort_limits = true; 
    motors[i].joint_limits_.max_effort = 100.0;

    idTomotor_ptr_map[motors[i].vesc_id_] = &motors[i];   // insert to map
    // Create map for motors

    // checking motors 
    if (motors[0].num_rotor_poles_ % 2 != 0){
      ROS_FATAL("[XR1_VESC] There should be even number of rotor poles");
      ros::shutdown();
      return false;
    }
    if (motors[0].joint_type_ == urdf::Joint::UNKNOWN){
      ROS_FATAL("[XR1_VESC] Verify your joint type");
      ros::shutdown();
      return false;
    }
    // // reads system parameters
    // ROS_INFO("Gear ratio is set to %f", motors[i].gear_ratio_);
    // ROS_INFO("Torque constant is set to %f", motors[i].torque_const_);
    // ROS_INFO("The number of hall sensors is set to %d", motors[i].num_hall_sensors_);
    // ROS_INFO("Screw lead is set to %f", motors[i].screw_lead_);
    // ROS_INFO("The number of rotor poles is set to %d", motors[i].num_rotor_poles_);
    // ROS_INFO("mode: %s", motors[i].command_mode_.data());
    // ROS_INFO("joint type: %s", motors[i].joint_type_);
  }

  // Find the local motor
  try{
    auto &m = *idTomotor_ptr_map.at(uint8_t(direct_vesc_id_));
    m.is_local_ = true;
  } catch(const std::out_of_range oor){
    ROS_FATAL("[XR1_VESC] UART connected VESC not found in the known devices!");
    ros::shutdown();
    return false;
  }

  // If we are strict, all given IDs should be found with CANBUS/UART
  bool _strict_all_ids;
  nh.param<bool>("strict_motors_check", _strict_all_ids, false);
  for (auto &m : motors){
    if(std::find(detected_vesc_ids.begin(), detected_vesc_ids.end(), m.vesc_id_) != detected_vesc_ids.end()) {
        ROS_INFO("[XR1_VESC] Motor ID [%d] is connected", m.vesc_id_);
    } else {
      if(_strict_all_ids){
        ROS_FATAL("[XR1_VESC] Using {strict_all_ids}, ID [%d] was not found! EXITING...", m.vesc_id_);
        ros::shutdown();
        return false;
      }
      ROS_INFO("[XR1_VESC] Motor ID [%d] was NOT found! Setting it as MOCK (not sending CANBUS commands to it) ...", m.vesc_id_);
      m.is_mock_ = true;
    }
  }
  

  // TODO: change this!!
  // Create interfaces for leg sensors
  for (int i=0; i < 4; i++){
    rb[i] = xr1JointSensor(rb_names[i], false, true);
  }

  // Create passive wheels 
  for (int i=0; i < 2; i++){
    passive_wheels[i] = xr1PoweredMotor(passive_w_names[i], "velocity", i+10, false, true);
  }
  

  registerControlInterfaces(nh_root, nh);
  // for (int i=0; i < 4; i++){
  //   fprintf(stderr, "Motor[%d] id: %d\n", i, motors[i].vesc_id_);
  // }
  // **** RANDEL: END new implementation ****************


  return true;
}

void XR1VescHwInterface::registerControlInterfaces(ros::NodeHandle& nh_root, ros::NodeHandle& nh){
  // Get relevant vars
  bool flag_middle_wheel_mimics_front, flag_register_passive_wheels;
  nh.param<bool>("middle_wheel_mimics_front", flag_middle_wheel_mimics_front, false);
  nh.param<bool>("register_passive_wheels", flag_register_passive_wheels, true);


  // REGISTER MOTORS
  for (auto &m : motors){
    // Registers motor to the state interface (for read)
    ROS_INFO("[XR1_VESC] Registering motor id[%d](%s) to ros_control", m.vesc_id_, m.joint_name_.c_str());
    hardware_interface::JointStateHandle state_handle(m.joint_name_, &(m.pos), &(m.vel), &(m.eff));
    joint_state_interface_.registerHandle(state_handle);

    // Registers motor to the velocity interface (for write)
    hardware_interface::JointHandle velocity_handle(joint_state_interface_.getHandle(m.joint_name_.c_str()), &(m.cmd));
    joint_velocity_interface_.registerHandle(velocity_handle);
    
    // if (m.command_mode_ == "velocity_duty")
    // {
    //   wheel_controller_.init(nh, &vesc_interface_);
    //   wheel_controller_.setGearRatio(m.gear_ratio_);
    //   wheel_controller_.setTorqueConst(m.torque_const_);
    //   wheel_controller_.setRotorPoles(m.num_rotor_poles_);
    //   wheel_controller_.setHallSensors(m.num_hall_sensors_);
    // }

    joint_limits_interface::VelocityJointSaturationHandle limit_handle(velocity_handle, m.joint_limits_);
    m.limit_velocity_interface_.registerHandle(limit_handle);
    // registerInterface(&joint_state_interface_);
    // registerInterface(&joint_velocity_interface_);
  }

  // REGISTER LEG SENSORS
  for (auto &s : rb){
    hardware_interface::JointStateHandle _state_handle(s.joint_name_.c_str(), &(s.pos), &(s.vel), &(s.vel));
    joint_state_interface_.registerHandle(_state_handle);
  }


  // **** Register passive wheels
  if (flag_register_passive_wheels){
    // use states of front wheel when [flag_middle_wheel_mimics_front] is true
    xr1PoweredMotor &left_middle = flag_middle_wheel_mimics_front ? motors[0] : passive_wheels[0];
    xr1PoweredMotor &right_middle = flag_middle_wheel_mimics_front ? motors[2] : passive_wheels[1];

    hardware_interface::JointStateHandle state_handle_p1(passive_wheels[0].joint_name_, &(left_middle.pos), &(left_middle.vel), &(left_middle.eff));
    joint_state_interface_.registerHandle(state_handle_p1);
    hardware_interface::JointHandle velocity_handle_p1(joint_state_interface_.getHandle(passive_wheels[0].joint_name_.c_str()), &(passive_wheels[0].cmd));
    joint_velocity_interface_.registerHandle(velocity_handle_p1);

    hardware_interface::JointStateHandle state_handle_p2(passive_wheels[1].joint_name_, &(right_middle.pos), &(right_middle.vel), &(right_middle.eff));
    joint_state_interface_.registerHandle(state_handle_p2);
    hardware_interface::JointHandle velocity_handle_p2(joint_state_interface_.getHandle(passive_wheels[1].joint_name_.c_str()), &(passive_wheels[1].cmd));
    joint_velocity_interface_.registerHandle(velocity_handle_p2);
  }

  

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_velocity_interface_);
}


void XR1VescHwInterface::read(const ros::Time& time, const ros::Duration& period)
{
  // requests joint states
  // function `packetCallback` will be called after receiving return packets
  // ROS_INFO("***RANDEL: Readi9ng");
  // vesc_interface_.requestState();

  for (auto &m : motors){
    if (m.is_mock_){
      // Don't read from mock motors
      continue;
    }
    if (m.command_mode_ == "velocity")
    {
      if (m.is_local_){
        // vesc_interface_.requestState();
        #if DEBUG_RANDEL == 1
          fprintf(stderr, "@@@@@@ REQUEST LOCAL state[%d]***\n", m.vesc_id_);
        #endif // DEBUG_RANDEL
        vesc_interface_.send(vesc_driver::VescPacketRequestValues(), -1);
      } else{
        #if DEBUG_RANDEL == 1
          fprintf(stderr, "@@@@@@ REQUEST CANBUS state[%d] ***\n", m.vesc_id_);
        #endif // DEBUG_RANDEL
        vesc_interface_.send(vesc_driver::VescPacketRequestValues(), m.vesc_id_);
      }
    }
  }

  // fprintf(stderr, "@@@@@@ GET APPCONF ***\n");
  // vesc_interface_.send(vesc_driver::VescGetAppConf(), -1);

  // fprintf(stderr, "@@@@@@ CMD PING CAN IDs ***\n");
  // vesc_interface_.send(vesc_driver::VescPingVescCanIDs(), -1);



  return;
}

void XR1VescHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // sends commands
  // auto &m = motors[1];
  for (auto &m : motors){
    #if DEBUG_RANDEL == 1
      fprintf(stderr, "^^^^^^^^^^^^^^ TRYING TO WRITE FOR[%d]***\n", m.vesc_id_);
    #endif // DEBUG_RANDEL
    if (m.is_mock_){
      // Don't write to mock motors
      continue;
    }
    if (m.command_mode_ == "velocity")
    {
      m.limit_velocity_interface_.enforceLimits(period);

      // converts the velocity unit: rad/s or m/s -> rpm -> erpm
      const double command_rpm = m.cmd * 60.0 / 2.0 / M_PI / m.gear_ratio_;
      const double command_erpm = command_rpm * static_cast<double>(m.num_rotor_poles_) / 2;


      // fprintf(stderr, "****CMDDDD: %f, %f\n", command_, command_erpm);
      // sends a reference velocity command
      if (m.is_local_){
        // vesc_interface_.setSpeed(command_erpm);
        #if DEBUG_RANDEL == 1
          fprintf(stderr, "$$$$$$ SEND LOCAL[%d]***: %f\n", m.vesc_id_, command_erpm);
        #endif // DEBUG_RANDEL
        vesc_interface_.send(vesc_driver::VescPacketSetVelocityERPM(command_erpm), -1);
      } else{
        #if DEBUG_RANDEL == 1
          fprintf(stderr, "$$$$$$ SEND CANBUS[%d] ***: %f\n", m.vesc_id_, command_erpm);
        #endif // DEBUG_RANDEL
        vesc_interface_.send(vesc_driver::VescPacketSetVelocityERPM(command_erpm), m.vesc_id_);
      }
    }
  }
  /*
  else if (command_mode_ == "velocity_duty")
  {
    limit_velocity_interface_.enforceLimits(period);

    // executes PID control
    wheel_controller_.setTargetVelocity(cmds_[0]);
  }
  else if (command_mode_ == "effort")
  {
    limit_effort_interface_.enforceLimits(period);

    // converts the command unit: Nm or N -> A
    const double command_current = cmds_[0] * gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_.setCurrent(command_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    cmds_[0] = std::max(-1.0, cmds_[0]);
    cmds_[0] = std::min(1.0, cmds_[0]);

    // sends a  duty command
    vesc_interface_.setDutyCycle(cmds_[0]);
  }*/
  return;
}

ros::Time XR1VescHwInterface::getTime() const
{
  return ros::Time::now();
}

void XR1VescHwInterface::packetCallback(const std::shared_ptr<VescPacket const>& packet)
{
  // ROS_INFO("***RANDEL: packetCallback CALLED!!!");
  if (!vesc_interface_.isRxDataUpdated())
  {
    ROS_WARN("[XR1VescHwInterface::packetCallback]packetCallcack called, but no packet received");
  }

  // RANDEL: DEBUG printout
  #if DEBUG_RANDEL == 1
    std::cout << "############## RAW REPLY("<< packet->getName() << "):\n";
    for(auto it = packet->getFrame().begin(); it != packet->getFrame().end();  ++it){
      std::cout << unsigned(*it) << ", ";
    }
    std::cout << "\n\n" << std::endl;
  #endif // DEBUG_RANDEL

  if (packet->getName() == "Values")
  { 
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);
    
    const uint8_t vesc_id = values->getVescID();
    auto map_it = idTomotor_ptr_map.find(vesc_id);
    if (map_it == idTomotor_ptr_map.end()) {
      // ROS_INFO("VESC ID in reply packet did not match any motors!");
      fprintf(stderr, "VESC ID[%d] in reply packet did not match any motors!", vesc_id);
    } else {
      // found
      xr1PoweredMotor &motor = *(map_it->second);
      const double current = values->getMotorCurrent();
      const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(motor.num_rotor_poles_ / 2);
      const double steps = values->getPosition();

      motor.pos = angles::normalize_angle(steps / (motor.num_hall_sensors_ * motor.num_rotor_poles_) * motor.gear_ratio_);  // unit: rad or m
      motor.vel = velocity_rpm / 60.0 * 2.0 * M_PI * motor.gear_ratio_;                 // unit: rad/s or m/s
      motor.eff = current * motor.torque_const_ / motor.gear_ratio_;                    // unit: Nm or N

      // if(motor.joint_name_ == "wheel_rear_left_joint"){
      //   std::cout << "******* pvRe:" << motor.pos << ", " << motor.vel << ", " << values->getVelocityERPM()<< ", " << motor.eff << ", " << std::endl;
      // }

    }
  } 
  else if (packet->getName() == "PingedCanIDs"){  
    std::shared_ptr<VescPacketPingedCanIDs const> values = std::dynamic_pointer_cast<VescPacketPingedCanIDs const>(packet);

    std::stringstream sstrm;
    sstrm << "[XR1_VESC] CANBUS-only pinged (" << values->vesc_ids.size() <<") VESC IDs: ";
    std::copy(values->vesc_ids.begin(), values->vesc_ids.end(), std::ostream_iterator<double>(sstrm, " "));
    sstrm << std::endl;
    ROS_INFO_STREAM(sstrm.str());

    assert(detected_vesc_ids.size() > 0);             // detected_vesc_ids should AT LEAST contain direct_vesc_id_ at this point
    // clear detected_vesc_ids from 2nd element to end (1st elem should be direct_vesc_id_)
    if(detected_vesc_ids.size() > 1){
      detected_vesc_ids.erase(detected_vesc_ids.begin() + 1, detected_vesc_ids.end());
    } else if (detected_vesc_ids.size() == 0){
      ROS_ERROR("[XR1_VESC] Vector detected_vesc_ids[] should AT LEAST contain direct_vesc_id_ !!!");
    }

    for(auto it = values->vesc_ids.begin(); it != values->vesc_ids.end(); it++){
      detected_vesc_ids.push_back(*it);
    }

  }
  else if (packet->getName() == "AppConf"){
    std::shared_ptr<VescPacketAppConf const> values = std::dynamic_pointer_cast<VescPacketAppConf const>(packet);
    direct_vesc_id_ = values->getVescID();
    ROS_INFO("[XR1_VESC] Direct UART VESC ID: [%d]", direct_vesc_id_);

    // Place that id in detected_vesc_ids[]
    if (detected_vesc_ids.size() == 0){
      detected_vesc_ids.push_back(uint8_t(direct_vesc_id_));
    } else {
      detected_vesc_ids[0] = uint8_t(direct_vesc_id_);
    }
  }
  

  return;
}

void XR1VescHwInterface::errorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
  return;
}

uint8_t XR1VescHwInterface::verifyVescID(int _in, uint8_t _default){
  if (_in <= std::numeric_limits<std::int8_t>::max()
      && _in >= std::numeric_limits<std::int8_t>::min()){
    return uint8_t(_in);
  } 
  else{
    return _default;
  }
}

}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::XR1VescHwInterface, hardware_interface::RobotHW)
