
#include "vesc_hw_interface/xr1_actuator_sensor_container.h"
#include <cmath>

namespace vesc_hw_interface
{
  void xr1PoweredMotor::extract_packet_values(std::shared_ptr<VescPacketValues const> &pckt_vals){
    mc_temp_fet_filtered_ = pckt_vals->getMosTemp();
    mc_temp_motor_filtered_ = pckt_vals->getMotorTemp();
    mc_motor_current_ = fabs(pckt_vals->getMotorCurrent());
    mc_input_current_ = pckt_vals->getInputCurrent();
    mc_current_direct_axis_ = pckt_vals->getDirectAxisCurrent();
    mc_current_quadrature_axis_ = pckt_vals->getQuadratureAxisCurrent();
    mc_duty_cycle_now_ = pckt_vals->getDuty();
    mc_erpm_ = pckt_vals->getVelocityERPM();
    mc_voltage_in_ = pckt_vals->getInputVoltage();
    mc_amp_hours_ = pckt_vals->getConsumedCharge();
    mc_amp_hours_charged_ = pckt_vals->getInputCharge();
    mc_watt_hours_ = pckt_vals->getConsumedPower();
    mc_watt_hours_charged_ = pckt_vals->getInputPower();
    mc_tachometer_ = pckt_vals->getPosition();
    mc_tachometer_abs_ = pckt_vals->getDisplacement();
    mc_fault_code_ = pckt_vals->getFaultCode();
    mc_pid_pos_now_ = pckt_vals->getPIDPosNow();
    mc_controller_vesc_id_ = pckt_vals->getVescID();
    mc_ntc_temp_mos1_ = pckt_vals->getTempMos1();
    mc_ntc_temp_mos2_ = pckt_vals->getTempMos2();
    mc_ntc_temp_mos3_ = pckt_vals->getTempMos3();
    mc_avg_vd_ = pckt_vals->getAveDirectAxisVoltage();
    mc_avg_vq_ = pckt_vals->getAveQuadratureAxisVoltage();
    
    mc_last_update_time = ros::Time::now();
  }

  void xr1PoweredMotor::update_pos_vel_eff_from_pckt_vals(){
    pos = angles::normalize_angle(mc_tachometer_ *2.0*M_PI / (num_hall_sensors_ * num_rotor_poles_) * gear_ratio_);  // unit: rad or m
    vel = mc_erpm_ / 60.0 * 2.0 * M_PI * gear_ratio_;                         // unit: rad/s or m/s
    eff = mc_motor_current_ * torque_const_ / gear_ratio_;                    // unit: Nm or N

    // std::cout << "**********v"<< int(vesc_id_) <<":" << vel << std::endl;
  }



  void xr1JointSensor::setReversedDir(){
    dir_val = -1;
  }
  void xr1JointSensor::setForwardDir(){
    dir_val = 1;
  }

  void xr1JointSensor::setEncoderZeroValue(int zero_val){
    enc_zero_val = (uint16_t)zero_val;
  }

  int16_t xr1JointSensor::zeroed_enc_val(){
    int16_t temp = (int16_t)enc_val - (int16_t)enc_zero_val;
    return temp > 0 ? temp : (int16_t)(MAX_ENCODER_VAL+temp);   // temp in false is negative
  }

  // zeroed and dir corrected encoder value
  int16_t xr1JointSensor::zeroed_dir_enc_val(){
    return zeroed_enc_val() * dir_val;
  }

  // zeroed and normalized angle (pos) in radians
  double xr1JointSensor::final_pos_rad(){
    return angles::normalize_angle(zeroed_dir_enc_val() * 2 * M_PI / MAX_ENCODER_VAL);
  }

  void xr1JointSensor::update_final_pos_from_enc(){
    pos = final_pos_rad();
  }



  void xr1JCompelemtaryJointSensor::update_final_pos_from_enc(){
    pos = final_pos_rad();
    pos2 = -pos;
  }
}