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
                    std::bind(&XR1VescHwInterface::errorCallback, this, std::placeholders::_1))
{
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
    ROS_FATAL("VESC communication port parameter required.");
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
    ROS_FATAL("Failed to connect to the VESC, %s.", exception.what());
    ros::shutdown();
    return false;
  }


  // **** RANDEL: START new implementation ****************
  // create a main motor directly connected to UART
  for (int i=0; i < 4; i++){
    if (i==0){
      motors[i] = vesc_hw_interface::xr1PoweredMotor(motor_names[i], "velocity", motor_vesc_ids[i], true, false);
    } else{
      motors[i] = vesc_hw_interface::xr1PoweredMotor(motor_names[i], "velocity", motor_vesc_ids[i] ,false, true);
    }
    motors[i].joint_limits_.has_velocity_limits = true;
    motors[i].joint_limits_.max_velocity = 1000.0;
    motors[i].joint_limits_.has_effort_limits = true;
    motors[i].joint_limits_.max_effort = 100.0;

    idTomotor_ptr_map[motors[i].vesc_id_] = &motors[i];   // insert to map
    // Create map for motors

    // checking motors 
    if (motors[0].num_rotor_poles_ % 2 != 0){
      ROS_ERROR("There should be even number of rotor poles");
      ros::shutdown();
      return false;
    }
    if (motors[0].joint_type_ == urdf::Joint::UNKNOWN){
      ROS_ERROR("Verify your joint type");
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

  // Create interfaces for leg sensors
  for (int i=0; i < 4; i++){
    rb[i] = xr1JointSensor(rb_names[i], false, true);
  }

  // Create passive wheels 
  for (int i=0; i < 2; i++){
    passive_wheels[i] = xr1PoweredMotor(passive_w_names[i], "velocity", i+10, false, true);
  }
  

  registerControlInterfaces(nh_root, nh);
  for (int i=0; i < 4; i++){
    fprintf(stderr, "Motor[%d] id: %d\n", i, motors[i].vesc_id_);
  }
  // **** RANDEL: END new implementation ****************


  return true;
}

void XR1VescHwInterface::registerControlInterfaces(ros::NodeHandle& nh_root, ros::NodeHandle& nh){
  // REGISTER MOTORS
  for (auto &m : motors){
    // Registers motor to the state interface (for read)
    fprintf(stderr, "^^^^ Registering motor id[%d]\n", m.vesc_id_);
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
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_velocity_interface_);
  }

  // REGISTER LEG SENSORS
  for (auto &s : rb){
    hardware_interface::JointStateHandle _state_handle(s.joint_name_.c_str(), &(s.pos), &(s.vel), &(s.vel));
    joint_state_interface_.registerHandle(_state_handle);
  }

  // **** Register passive wheels as MOCK (copy front wheel states) ****** reads from actual powered motor
  hardware_interface::JointStateHandle state_handle_p1(passive_wheels[0].joint_name_, &(motors[0].pos), &(motors[0].vel), &(motors[0].eff));
  joint_state_interface_.registerHandle(state_handle_p1);
  hardware_interface::JointHandle velocity_handle_p1(joint_state_interface_.getHandle(passive_wheels[0].joint_name_.c_str()), &(passive_wheels[0].cmd));
  joint_velocity_interface_.registerHandle(velocity_handle_p1);

  hardware_interface::JointStateHandle state_handle_p2(passive_wheels[1].joint_name_, &(motors[2].pos), &(motors[2].vel), &(motors[2].eff));
  joint_state_interface_.registerHandle(state_handle_p2);
  hardware_interface::JointHandle velocity_handle_p2(joint_state_interface_.getHandle(passive_wheels[1].joint_name_.c_str()), &(passive_wheels[1].cmd));
  joint_velocity_interface_.registerHandle(velocity_handle_p2);

  

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


  /*if (command_mode_ == "position")
  {
    // For PID control, request packets are automatically sent in the control cycle.
    // The latest data is read in this function.
    position_ = servo_controller_.getPositionSens();
    velocity_ = servo_controller_.getVelocitySens();
    effort_ = servo_controller_.getEffortSens();
  }
  else if (command_mode_ == "velocity_duty")
  {
    p_wheel_pos_[0] = wheel_controller_.getPositionSens();
    p_wheel_vel_[0] = wheel_controller_.getVelocitySens();
    p_wheel_eff_[0] = wheel_controller_.getEffortSens();
  }
  else
  {
    // ROS_INFO("***RANDEL: requesting state!");
    vesc_interface_.requestState();
  }*/

  // ROS_INFO("***RANDEL: joint: %d", joint_type_);
  // if (joint_type_ == urdf::Joint::REVOLUTE || joint_type_ == urdf::Joint::CONTINUOUS)
  // {
  //   p_wheel_pos_[0] = angles::normalize_angle(p_wheel_pos_[0]);
  //   // ROS_INFO("***RANDEL: uuuuuu %f", position_);
  // }

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
    std::cout << "############## REPLY:\n";
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

    }
  }

  return;
}

void XR1VescHwInterface::errorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::XR1VescHwInterface, hardware_interface::RobotHW)
