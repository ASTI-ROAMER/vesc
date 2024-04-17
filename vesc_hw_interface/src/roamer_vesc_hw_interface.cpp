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
                    std::bind(&XR1VescHwInterface::errorCallback, this, std::placeholders::_1)), 
    leg_interface_(std::string(), std::bind(&XR1VescHwInterface::legPacketCallback, this, std::placeholders::_1),
                    std::bind(&XR1VescHwInterface::legErrorCallback, this, std::placeholders::_1)),
    direct_vesc_id_(-1), use_only_valid_leg_encoder_values(true)
{
  detected_vesc_ids.reserve(4);
}

XR1VescHwInterface::~XR1VescHwInterface()
{
}

bool XR1VescHwInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
{
  nh.param<bool>("debug_disable_motors", disable_motors, false);
  nh.param<bool>("debug_disable_encoders", disable_encoders, false);
  nh.param<bool>("debug_print_enc_vals", print_enc_vals, false);
  
  if(!disable_motors){              // no uart devices connected
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
    catch (vesc_driver::SerialException exception)
    {
      ROS_FATAL("[XR1_VESC] Failed to connect to the VESC [%s].\n\n%s.", port.c_str(), exception.what());
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

  // Get gear ratios (motor gear teeth / wheel gear teeth)
  double gear_ratios[4];
  nh.param<double>("gear_ratio_front_left", gear_ratios[0], 0.5);
  nh.param<double>("gear_ratio_rear_left", gear_ratios[1], 0.5);
  nh.param<double>("gear_ratio_front_right", gear_ratios[2], 0.5);
  nh.param<double>("gear_ratio_rear_right", gear_ratios[3], 0.5);

  // fprintf(stderr, "\n\n****** GR: %f, %f, %f, %f\n\n", gear_ratios[0], gear_ratios[1], gear_ratios[2], gear_ratios[3] );

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
    motors[i] = vesc_hw_interface::xr1PoweredMotor(motor_names[i], "velocity", uint8_t(motor_vesc_ids[i]), false, false, gear_ratios[i]);

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

  if(!disable_motors){              // no uart devices connected
    // Find the local motor
    try{
      auto &m = *idTomotor_ptr_map.at(uint8_t(direct_vesc_id_));
      m.is_local_ = true;
    } catch(const std::out_of_range oor){
      ROS_FATAL("[XR1_VESC] UART connected VESC not found in the known devices!");
      ros::shutdown();
      return false;
    }
  }

  // If we are strict, all given IDs should be found with CANBUS/UART
  bool _strict_all_ids;
  nh.param<bool>("strict_motors_check", _strict_all_ids, false);
  for (auto &m : motors){
    if(std::find(detected_vesc_ids.begin(), detected_vesc_ids.end(), m.vesc_id_) != detected_vesc_ids.end()) {
        ROS_INFO("[XR1_VESC] Motor ID [%d] is connected", m.vesc_id_);
    } else {
      if(_strict_all_ids && !disable_motors){
        ROS_FATAL("[XR1_VESC] Using {strict_all_ids}, ID [%d] was not found! EXITING...", m.vesc_id_);
        ros::shutdown();
        return false;
      }
      ROS_INFO("[XR1_VESC] Motor ID [%d] was NOT found! Setting it as MOCK (not sending CANBUS commands to it) ...", m.vesc_id_);
      m.is_mock_ = true;
    }
  }

  // Create passive wheels 
  for (int i=0; i < 2; i++){
    passive_wheels[i] = xr1PoweredMotor(passive_w_names[i], "velocity", i+10, false, true);
  }
  // **** RANDEL: END new implementation ****************


  // *************** RANDEL: leg initialization ****************
  // reads a port name to open
  std::string leg_port;
  if (!nh.getParam("leg_port", leg_port))
  {
    ROS_FATAL("[XR1_VESC] Leg Encoders communication port parameter required.");
    ros::shutdown();
    return false;
  }

  // attempts to open the serial port
  if(!disable_encoders){
    try
    {
      leg_interface_.connect(leg_port);
    }
    catch (vesc_driver::SerialException exception)
    {
      ROS_FATAL("[XR1_VESC] Failed to connect to the Leg Encoders [%s].\n\n%s.", leg_port.c_str(), exception.what());
      ros::shutdown();
      return false;
    }
  }
  
  nh.param<bool>("use_only_valid_leg_encoder_values", use_only_valid_leg_encoder_values, true);


  // Setup rocker joints (complementary joints, only one encoder value )
  std::string temp_joint_name, temp_joint_name2;
  int temp_zero_val=0, temp_zero_val2=0;
  bool temp_reversed=false, temp_reversed2=false;

  nh.param<std::string>("rocker_left_joint_name", temp_joint_name, DEFAULT_COMP_JOINT_NAMES[0]);
  nh.param<std::string>("rocker_right_joint_name", temp_joint_name2, DEFAULT_COMP_JOINT_NAMES[1]);
  nh.param<int>("rocker_zero_enc_val", temp_zero_val, 0);
  nh.param<bool>("rocker_reverse_dir", temp_reversed, false);
  if(temp_zero_val < 0 || temp_zero_val > xr1JCompelemtaryJointSensor::MAX_ENCODER_VAL){
    ROS_FATAL("[XR1_VESC] Parameter [%s] must be between [0, %d], value=%d", "comp_leg1_joint_name", xr1JCompelemtaryJointSensor::MAX_ENCODER_VAL, temp_zero_val);
    ros::shutdown();
    return false;
  }
  ROS_INFO("[XR1_VESC] Rocker joints: joint_names=[%s, %s] enc_zero_val=%d, dir=%s", temp_joint_name.c_str(), temp_joint_name2.c_str(), temp_zero_val, temp_reversed ? "REVERSED" : "FORWARDS");
  comp_joints_ = xr1JCompelemtaryJointSensor(temp_joint_name, temp_joint_name2, temp_zero_val, temp_reversed);

  // Setup bogie joints (normal joints, one encoder value for each joint)
  nh.param<std::string>("bogie_left_joint_name", temp_joint_name, DEFAULT_NORM_JOINT_NAMES[0]);
  nh.param<int>("bogie_left_zero_enc_val", temp_zero_val, 0);
  nh.param<bool>("bogie_left_reverse_dir", temp_reversed, false);
  nh.param<std::string>("bogie_right_joint_name", temp_joint_name2, DEFAULT_NORM_JOINT_NAMES[1]);
  nh.param<int>("bogie_right_zero_enc_val", temp_zero_val2, 0);
  nh.param<bool>("bogie_right_reverse_dir", temp_reversed2, false);
  if(temp_zero_val < 0 || temp_zero_val > xr1JCompelemtaryJointSensor::MAX_ENCODER_VAL ||
      temp_zero_val2 < 0 || temp_zero_val2 > xr1JCompelemtaryJointSensor::MAX_ENCODER_VAL){
    ROS_FATAL("[XR1_VESC] Parameter [%s] must be between [0, %d], values=(%d, %d)", "bogie_right/left_zero_enc_val", xr1JCompelemtaryJointSensor::MAX_ENCODER_VAL, temp_zero_val, temp_zero_val2);
    ros::shutdown();
    return false;
  }
  ROS_INFO("[XR1_VESC] Bogie left joint: joint_name=[%s] enc_zero_val=%d, dir=%s", temp_joint_name.c_str(), temp_zero_val, temp_reversed ? "REVERSED" : "FORWARDS");
  ROS_INFO("[XR1_VESC] Bogie right joint: joint_name=[%s] enc_zero_val=%d, dir=%s", temp_joint_name2.c_str(), temp_zero_val2, temp_reversed2 ? "REVERSED" : "FORWARDS");
  normal_joints_[0] = xr1JointSensor(temp_joint_name, temp_zero_val, temp_reversed);
  normal_joints_[1] = xr1JointSensor(temp_joint_name2, temp_zero_val2, temp_reversed2);

  initDiagnostics();

  // **** register interfaces
  registerControlInterfaces(nh_root, nh);
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
  // For rocker
  hardware_interface::JointStateHandle _state_handle(comp_joints_.joint_name_.c_str(), &(comp_joints_.pos), &(comp_joints_.vel), &(comp_joints_.eff));
  joint_state_interface_.registerHandle(_state_handle);
  hardware_interface::JointStateHandle _state_handle2(comp_joints_.joint_name_2.c_str(), &(comp_joints_.pos2), &(comp_joints_.vel2), &(comp_joints_.eff2));
  joint_state_interface_.registerHandle(_state_handle2);
  // For bogie
  for (auto &norm_joint : normal_joints_){
    hardware_interface::JointStateHandle _state_handle(norm_joint.joint_name_.c_str(), &(norm_joint.pos), &(norm_joint.vel), &(norm_joint.eff));
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

  if(!disable_motors){              // no uart devices connected
    #if DEBUG_RANDEL == 1
      fprintf(stderr, "@@@@@@ requesting VESC motor stats ***\n");
    #endif // DEBUG_RANDEL
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
  }

  if(!disable_encoders){
    #if DEBUG_RANDEL == 1
      fprintf(stderr, "@@@@@@ requesting leg enc ***\n");
    #endif // DEBUG_RANDEL
    if (use_only_valid_leg_encoder_values){
      leg_interface_.send(vesc_driver::LegReqAllPosStatus(), -1);
    } else{
      leg_interface_.send(vesc_driver::LegReqAllPos(), -1);
    }
  }

  // fprintf(stderr, "@@@@@@ GET APPCONF ***\n");
  // vesc_interface_.send(vesc_driver::VescGetAppConf(), -1);

  // fprintf(stderr, "@@@@@@ CMD PING CAN IDs ***\n");
  // vesc_interface_.send(vesc_driver::VescPingVescCanIDs(), -1);

  diagnostic_updater_.update();

  return;
}

void XR1VescHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // sends commands
  // auto &m = motors[1];

  if(!disable_motors){
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


        // fprintf(stderr, "****CMD[%d]: %f, %f, %f\n", m.vesc_id_, m.cmd, command_rpm, command_erpm);
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
  }
  // fprintf(stderr, "--------------------\n\n");
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

  if(!disable_motors){
    if (!vesc_interface_.isRxDataUpdated())
    {
      ROS_WARN("[XR1VescHwInterface::packetCallback]packetCallcack called, but no packet received");
    }
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
      motor.extract_packet_values(values);
      motor.update_pos_vel_eff_from_pckt_vals();

      // const double current = values->getMotorCurrent();
      // const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(motor.num_rotor_poles_ / 2);
      // const double steps = values->getPosition();

      // motor.pos = angles::normalize_angle(steps / (motor.num_hall_sensors_ * motor.num_rotor_poles_) * motor.gear_ratio_);  // unit: rad or m
      // motor.vel = velocity_rpm / 60.0 * 2.0 * M_PI * motor.gear_ratio_;                 // unit: rad/s or m/s
      // motor.eff = current * motor.torque_const_ / motor.gear_ratio_;                    // unit: Nm or N


      // PRINTING stats
      

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


void XR1VescHwInterface::legPacketCallback(const std::shared_ptr<VescPacket const>& packet)
{
  // ROS_INFO("***RANDEL: legpacketCallback CALLED!!!");
  if(!disable_encoders){
    if (!leg_interface_.isRxDataUpdated())
    {
      ROS_WARN("[XR1VescHwInterface::legPacketCallback] called, but no packet received");
    }
  }

  // RANDEL: DEBUG printout
  #if DEBUG_RANDEL == 1
    std::cout << "############## RAW REPLY("<< packet->getName() << "):\n";
    for(auto it = packet->getFrame().begin(); it != packet->getFrame().end();  ++it){
      std::cout << unsigned(*it) << ", ";
    }
    std::cout << "\n\n" << std::endl;
  #endif // DEBUG_RANDEL

  if (*(packet->payload_end_.first) == COMM_LEG_ALL_POS){
    #if DEBUG_RANDEL == 1
      fprintf(stderr, "** LEG rcv frame: \n");
      for(auto it = packet->frame_.begin() ; it != packet->frame_.end() ;  ++it){
        fprintf(stderr, " %02x", (unsigned char)(*it));
      }
      fprintf(stderr, "\n\n");
    #endif // DEBUG_RANDEL
    
    // IMPORTANT! Arduino outputs 3 data chunks. These 3 chunks maps to our 4 leg joint as:
    // *** rocker joint is only 1 object but contains both left and right rocker joints, as it only needs 1 encoder value for 2 joint positions
    //  rocker_joints = arduino_chunk[0]        
    //  bogie_left_joint = arduino_chunk[1]
    //  bogie_right_joint = arduino_chunk[2]

    xr1JointSensor* temp_j_ptrs[] = {static_cast<xr1JointSensor*>(&comp_joints_), &(normal_joints_[0]), &(normal_joints_[1])};
    for(int i=0; i < 3; i++){
      const uint8_t hbyte = (uint8_t)(*(packet->payload_end_.first + 1 + (2 *i)));
      const uint8_t lbyte = (uint8_t)(*(packet->payload_end_.first + 2 + (2 *i)));
      const uint16_t enc_val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

      temp_j_ptrs[i]->isEncUpToDate = 1;
      temp_j_ptrs[i]->enc_val = enc_val;
      temp_j_ptrs[i]->update_final_pos_from_enc();
      
      #if DEBUG_RANDEL == 1
        double rad_a = angles::normalize_angle(enc_val * 2 * M_PI / 4095.0);
        fprintf(stderr, "------\n");
        fprintf(stderr, "[%d]hbyte: %02x\n", i, hbyte);
        fprintf(stderr, "[%d]lbyte: %02x\n", i, lbyte);
        fprintf(stderr, "[%d]raw_encval: %04x (%d dec) (%f rad)\n", i, enc_val, enc_val, rad_a);
        fprintf(stderr, "[%d]processed(z %d): zeroed: %d,  zd: %d,  final: %frad\n", i, temp_j_ptrs[i]->enc_zero_val, temp_j_ptrs[i]->zeroed_enc_val(), temp_j_ptrs[i]->zeroed_dir_enc_val(), temp_j_ptrs[i]->final_pos_rad());
      #endif // DEBUG_RANDEL
    } 

  } else if (*(packet->payload_end_.first) == COMM_LEG_ALL_POS_STATUS){
    #if DEBUG_RANDEL == 1
      fprintf(stderr, "** LEG rcv frame: \n");
      for(auto it = packet->frame_.begin() ; it != packet->frame_.end() ;  ++it){
        fprintf(stderr, " %02x", (unsigned char)(*it));
      }
      fprintf(stderr, "\n\n");
    #endif // DEBUG_RANDEL

    // IMPORTANT! Arduino outputs 3 data chunks. These 3 chunks maps to our 4 leg joint as:
    // *** rocker joint is only 1 object but contains both left and right rocker joints, as it only needs 1 encoder value for 2 joint positions
    //  rocker_joints = arduino_chunk[0]        
    //  bogie_left_joint = arduino_chunk[1]
    //  bogie_right_joint = arduino_chunk[2]

    xr1JointSensor* temp_j_ptrs[] = {static_cast<xr1JointSensor*>(&comp_joints_), &(normal_joints_[0]), &(normal_joints_[1])};
    for(int i=0; i < 3; i++){
      const uint8_t hbyte = (uint8_t)(*(packet->payload_end_.first + 1 + (3 *i)));
      const uint8_t lbyte = (uint8_t)(*(packet->payload_end_.first + 2 + (3 *i)));
      const uint8_t status = (uint8_t)(*(packet->payload_end_.first + 3 + (3 *i)));
      const uint16_t enc_val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

      temp_j_ptrs[i]->isEncUpToDate = status;
      temp_j_ptrs[i]->enc_val = enc_val;
      if (temp_j_ptrs[i]->isEncUpToDate){
        temp_j_ptrs[i]->update_final_pos_from_enc();
      }
      // if(i==0)
      // fprintf(stderr, "*** [%d]processed(zv %d): zeroed: %d,  zd: %d,  final: %frad, r:%d, s:%d, rr:%d\n", i, temp_j_ptrs[i]->enc_zero_val, temp_j_ptrs[i]->zeroed_enc_val(), temp_j_ptrs[i]->zeroed_dir_enc_val(), temp_j_ptrs[i]->final_pos_rad(), temp_j_ptrs[i]->enc_val, temp_j_ptrs[i]->isEncUpToDate, enc_val);
      #if DEBUG_RANDEL == 1
        double rad_a = angles::normalize_angle(enc_val * 2 * M_PI / 4095.0);
        fprintf(stderr, "------\n");
        fprintf(stderr, "[%d]hbyte: %02x\n", i, hbyte);
        fprintf(stderr, "[%d]lbyte: %02x\n", i, lbyte);
        fprintf(stderr, "[%d]status: %02x, isEncUpToDate: %d\n", i, status, temp_j_ptrs[i]->isEncUpToDate);
        fprintf(stderr, "[%d]raw_encval: %04x (%d dec) (%f rad)\n", i, enc_val, enc_val, rad_a);
        fprintf(stderr, "[%d]processed(z %d): zeroed: %d,  zd: %d,  final: %frad\n", i, temp_j_ptrs[i]->enc_zero_val, temp_j_ptrs[i]->zeroed_enc_val(), temp_j_ptrs[i]->zeroed_dir_enc_val(), temp_j_ptrs[i]->final_pos_rad());
      #endif // DEBUG_RANDEL
      if(print_enc_vals){
        fprintf(stderr, "enc[%d]: %u\n", i, (unsigned int)enc_val);
      }
    }
  }
  

  return;
}

void XR1VescHwInterface::legErrorCallback(const std::string& error)
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


// ==============================================================================================
// ===================================== Diagnostics ============================================
void XR1VescHwInterface::initDiagnostics(){
  diagnostic_updater_.setHardwareID("XR11");
  diagnostic_updater_.add("system_status", this, &XR1VescHwInterface::xr1_hw_system_updater);
  diagnostic_updater_.add("motors_status", this, &XR1VescHwInterface::xr1_hw_motors_updater);
  diagnostic_updater_.add("encoder_status", this, &XR1VescHwInterface::xr1_hw_encoders_updater);
  diagnostic_updater_.add("power_status", this, &XR1VescHwInterface::xr1_power_system_updater);
  diagnostic_updater_.add("safety_status", this, &XR1VescHwInterface::xr1_safety_system_updater);

  diagnostic_updater_.force_update();
}


void XR1VescHwInterface::xr1_hw_system_updater(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System Status OK!");

  double total_vesc_cur_ = motors[0].mc_input_current_ + motors[1].mc_input_current_ + motors[2].mc_input_current_ + motors[3].mc_input_current_;
  double left_vesc_cur_ = motors[0].mc_input_current_ + motors[1].mc_input_current_;
  double right_vesc_cur_ = motors[2].mc_input_current_ + motors[3].mc_input_current_;
  
  stat.add("VESC UART port:", vesc_interface_.port_name);
  stat.add("Encoder UART port:", leg_interface_.port_name);
  stat.add("Total VESC Drivers Current", total_vesc_cur_);
  stat.add("Left VESC Dual Driver Current", left_vesc_cur_);
  stat.add("Right VESC Dual Driver Current", right_vesc_cur_);

  if(!disable_motors){
    if(!vesc_interface_.isConnected()){
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "VESC UART port not connected!!");
    }
  
    if((left_vesc_cur_ > xr1PoweredMotor::MAX_DUAL_VESC_CURRENT) 
      || (right_vesc_cur_ > xr1PoweredMotor::MAX_DUAL_VESC_CURRENT)){
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "VESC dual driver max fuse current reached!!");
    }

    if(total_vesc_cur_ > xr1PoweredMotor::MAX_TOTAL_VESC_CURRENT) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Total VESC driver max fuse current reached!!");
    }
  }

  if(!disable_encoders){
    if(!leg_interface_.isConnected()){
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Leg encoder UART port not connected!!");
    }

    if (!(comp_joints_.isEncUpToDate && normal_joints_[0].isEncUpToDate && normal_joints_[1].isEncUpToDate)){
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "At least 1 of the leg joint encoders is not updated!");
    }
  }

  
}


void XR1VescHwInterface::xr1_hw_motors_updater(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motors System OK!");
  
  stat.add("F Left Motor Current (A)", motors[0].mc_motor_current_);
  stat.add("R Left Motor Current (A)", motors[1].mc_motor_current_);
  stat.add("F Right Motor Current (A)", motors[2].mc_motor_current_);
  stat.add("R Right Motor Current (A)", motors[3].mc_motor_current_);

  stat.add("F Left Input Current (A)", motors[0].mc_input_current_);
  stat.add("R Left Input Current (A)", motors[1].mc_input_current_);
  stat.add("F Right Input Current (A)", motors[2].mc_input_current_);
  stat.add("R Right Input Current (A)", motors[3].mc_input_current_);

  stat.add("F Left Motor Input Voltage (V)", motors[0].mc_voltage_in_);
  stat.add("R Left Motor Input Voltage (V)", motors[1].mc_voltage_in_);
  stat.add("F Right Motor Input Voltage (V)", motors[2].mc_voltage_in_);
  stat.add("R Right Motor Input Voltage (V)", motors[3].mc_voltage_in_);
  
  stat.add("F Left Motor Temp (C)", motors[0].mc_temp_motor_filtered_);
  stat.add("R Left Motor Temp (C)", motors[1].mc_temp_motor_filtered_);
  stat.add("F Right Motor Temp (C)", motors[2].mc_temp_motor_filtered_);
  stat.add("R Right Motor Temp (C)", motors[3].mc_temp_motor_filtered_);

  stat.add("F Left Fault Code", motors[0].mc_fault_code_);
  stat.add("R Left Fault Code", motors[1].mc_fault_code_);
  stat.add("F Right Fault Code", motors[2].mc_fault_code_);
  stat.add("R Right Fault Code", motors[3].mc_fault_code_);

  stat.add("F Left FET Temp (C)", motors[0].mc_temp_fet_filtered_);
  stat.add("R Left FET Temp (C)", motors[1].mc_temp_fet_filtered_);
  stat.add("F Right FET Temp (C)", motors[2].mc_temp_fet_filtered_);
  stat.add("R Right FET Temp (C)", motors[3].mc_temp_fet_filtered_);

  for (auto &m : motors){
    if(m.mc_motor_current_ > 20.0){
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "A motor driver reached max current!");
      break;
    }

    if(m.mc_fault_code_){
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "A motor driver has a fault!");
      break;
    }

    if(m.mc_temp_motor_filtered_ > 85.0){
      if(m.mc_temp_motor_filtered_ > 1000.0){
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Erratically high motor temp. Possibly a disconnected hall sensor cable.");
        break;
      }
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "A motor driver reached max temperature!");
      break;
    }
  }
}


void XR1VescHwInterface::xr1_hw_encoders_updater(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Encoders System OK!");

  stat.add("Middle Encoder (L+R Rocker) Status", bool(comp_joints_.isEncUpToDate));
  stat.add("Left Bogie Status", bool(normal_joints_[0].isEncUpToDate));
  stat.add("Right Bogie Status", bool(normal_joints_[1].isEncUpToDate));
  
  if(!disable_encoders){
    if (!(comp_joints_.isEncUpToDate && normal_joints_[0].isEncUpToDate && normal_joints_[1].isEncUpToDate)){
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "At least 1 of the leg joint encoders is not updated (Possibly disconnected cable)!");
    }
  }
}

void XR1VescHwInterface::xr1_power_system_updater(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Power System OK!");
  double total_AH_drawn = motors[0].mc_amp_hours_ + motors[1].mc_amp_hours_ + motors[2].mc_amp_hours_ + motors[3].mc_amp_hours_;
  double total_WH_drawn = motors[0].mc_watt_hours_ + motors[1].mc_watt_hours_ + motors[2].mc_watt_hours_ + motors[3].mc_watt_hours_;
  stat.add("Total Amp-hours Drawn (AH)", total_AH_drawn);
  stat.add("Total Watt-hours Drawn (WH)", total_WH_drawn);

}

void XR1VescHwInterface::xr1_safety_system_updater(diagnostic_updater::DiagnosticStatusWrapper &stat){
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safety System OK!");
  stat.add("DUMMY Tineout", true);
  stat.add("DUMMY Lockout", true);
  stat.add("DUMMY Emergency Stop", true);
  stat.add("DUMMY No Battery", true);
  stat.add("DUMMY Current Limit", false);

}


}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::XR1VescHwInterface, hardware_interface::RobotHW)
