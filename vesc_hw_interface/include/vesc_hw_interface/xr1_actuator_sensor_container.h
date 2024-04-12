#ifndef XR1_ACTUATOR_SENSOR_CONTAINER_HW_INTERFACE_H_
#define XR1_ACTUATOR_SENSOR_CONTAINER_HW_INTERFACE_H_

#include <string>

#include <ros/ros.h>
#include <urdf_model/types.h>
#include <urdf_parser/urdf_parser.h>
#include <angles/angles.h>
#include <joint_limits_interface/joint_limits_interface.h>

// #include "vesc_driver/data_map_leg_encoder.h"
#include "vesc_driver/vesc_packet.h"


namespace vesc_hw_interface
{
using vesc_driver::VescPacketValues;


class xr1PoweredMotor
{
public:
  static constexpr double MAX_MOTOR_CURRENT=95.0;               // per motor/driver current (from flipsky specs), ** check motor current
  static constexpr double MAX_DUAL_VESC_CURRENT=40.0;           // per dual vesc controller (a dual vesc controller has 2 drivers, related to fuse used), ** check input current
  static constexpr double MAX_TOTAL_VESC_CURRENT=80.0;          // total current draw for all motors drivers (related to fuse used), ** check input current
  xr1PoweredMotor(){};
  xr1PoweredMotor(std::string joint_name, std::string command_mode, uint8_t vesc_id, int local=false, int mock=false, double gear_ratio=1.0): 
      joint_type_(urdf::Joint::CONTINUOUS), joint_name_(joint_name), command_mode_(command_mode), vesc_id_(vesc_id), is_local_(local), is_mock_(mock),
      cmd(0.0), pos(0.0),  vel(0.0), eff(0.0),
      num_rotor_poles_(14), num_hall_sensors_(3), gear_ratio_(gear_ratio), torque_const_(1.0), screw_lead_(1.0),
      mc_last_update_time(0.0) {};
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

  // motor stats
  ros::Time mc_last_update_time;

  double mc_temp_fet_filtered_ = 0.0;                 // filtered MOSFET temperature
  double mc_temp_motor_filtered_ = 0.0;               // filtered motor temperature
  double mc_motor_current_ = 0.0;                     // average motor current
  double mc_input_current_ = 0.0;                     // average input current
  double mc_current_direct_axis_ = 0.0;               // average D axis current
  double mc_current_quadrature_axis_ = 0.0;           // average Q axis current
  double mc_duty_cycle_now_ = 0.0;                    // the duty cycle now
  double mc_erpm_ = 0.0;                              // current ERPM (different from RPM!!)
  double mc_voltage_in_ = 0.0;                        // input voltage
  double mc_amp_hours_ = 0.0;                         // the amount of amp hours drawn from the input source
  double mc_amp_hours_charged_ = 0.0;                 // the amount of amp hours fed back into the input source
  double mc_watt_hours_ = 0.0;                        // the amount of watt hours drawn from the input source
  double mc_watt_hours_charged_ = 0.0;                // the amount of watt hours fed back into the input source
  int mc_tachometer_ = 0.0;                             // tachometer value / position
  int mc_tachometer_abs_ = 0.0;                         // abs tachometer value / displacement
  int mc_fault_code_ = 0;                             // fault code
  double mc_pid_pos_now_ = 0.0;                       // the position of pid, in float degrees
  uint8_t mc_controller_vesc_id_ = 0;                 // the controller vesc ID where the values CAME FROM
  double mc_ntc_temp_mos1_ = 0.0;                     // The NTC temperature of mosfet 1
  double mc_ntc_temp_mos2_ = 0.0;                     // The NTC temperature of mosfet 2
  double mc_ntc_temp_mos3_ = 0.0;                     // The NTC temperature of mosfet 2
  double mc_avg_vd_ = 0.0;                            // The average Direct axis voltage.
  double mc_avg_vq_ = 0.0;                            // The average Quadrature axis voltage.

  void extract_packet_values(std::shared_ptr<VescPacketValues const> &pckt_vals);
  void update_pos_vel_eff_from_pckt_vals();

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

  void setReversedDir();
  void setForwardDir();
  void setEncoderZeroValue(int zero_val);
  int16_t zeroed_enc_val();

  // zeroed and dir corrected encoder value
  int16_t zeroed_dir_enc_val();

  // zeroed and normalized angle (pos) in radians
  double final_pos_rad();

  virtual void update_final_pos_from_enc();

  // joint_limits_interface::JointLimits joint_limits_;
};


class xr1JCompelemtaryJointSensor : public xr1JointSensor
{
  public:
  xr1JCompelemtaryJointSensor() : xr1JointSensor() {};
  xr1JCompelemtaryJointSensor(std::string joint_name, std::string joint_name2, uint16_t enc_zero_val_=0, bool reversed_=false): 
      xr1JointSensor(joint_name, enc_zero_val_, reversed_),
      joint_name_2(joint_name2),
      pos2(0.0),  vel2(0.0), eff2(0.0) {};
  ~xr1JCompelemtaryJointSensor(){};

  std::string joint_name_2;
  double pos2, vel2, eff2;    // negative version of pos, vel, eff

  void update_final_pos_from_enc() override;
};

} // namespace vesc_hw_interface

#endif  // XR1_ACTUATOR_SENSOR_CONTAINER_HW_INTERFACE_H_