// Copyright 2023 Pixmoving, Inc. 
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PIX_ROBOBUS_DRIVER__CONTROL_CONVERTER_HPP_
#define PIX_ROBOBUS_DRIVER__CONTROL_CONVERTER_HPP_

#include <memory>
#include <string>
//ros
#include <rclcpp/rclcpp.hpp>
// autoware
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

// pix control
#include <pix_robobus_driver_msgs/msg/throttle_command.hpp>
#include <pix_robobus_driver_msgs/msg/gear_command.hpp>
#include <pix_robobus_driver_msgs/msg/steering_command.hpp>
#include <pix_robobus_driver_msgs/msg/brake_command.hpp>
#include <pix_robobus_driver_msgs/msg/park_command.hpp>
#include <pix_robobus_driver_msgs/msg/vehicle_mode_command.hpp>
#include <pix_robobus_driver_msgs/msg/auto_remote_ctrl_msg.hpp>
#include <pix_robobus_driver_msgs/msg/vcu_report.hpp>
// #include <pix_robobus_driver_msgs/msg/a2v_wheel_ctrl.hpp>
// pix report
#include <pix_robobus_driver_msgs/msg/gear_report.hpp>

namespace pix_robobus_driver
{
namespace control_converter
{
using ThrottleCommand = pix_robobus_driver_msgs::msg::ThrottleCommand;
using GearCommand = pix_robobus_driver_msgs::msg::GearCommand;
using SteeringCommand = pix_robobus_driver_msgs::msg::SteeringCommand;
using BrakeCommand = pix_robobus_driver_msgs::msg::BrakeCommand;
using ParkCommand = pix_robobus_driver_msgs::msg::ParkCommand;
using VehicleModeCommand = pix_robobus_driver_msgs::msg::VehicleModeCommand;
using GearReport = pix_robobus_driver_msgs::msg::GearReport;
using AutoRemote = pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg;
// using A2vWheelCtrl = pix_robobus_driver_msgs::msg::A2vWheelCtrl;

//chassis = enable control
enum { DISABLE, ENABLE };
//chassis drive mode control
enum {
  DIRVE_ENCTRL_THROTTLE_PADDLE,
  DIRVE_ENCTRL_SPEED
};
// chassis gear control and report
enum {
  GEAR_INVALID,
  GEAR_PARK,
  GEAR_REVERSE,
  GEAR_NEUTRAL,
  GEAR_DRIVE
};

// chassi steer mode control
enum {
  STEER_STANDARD,
  STEER_NON_DIRECTION,
  STEER_SYNC_DIRECTION
};

/**
 * @brief node parameter
 * @param loop_rate loop rate of publisher
 * @param max_steerig_angle max steering angle in radians
 * @param steering_factor the rate to convert steering angle to steering command signal value
 * @param autoware_control_command_timeout control command timeout threshold in ms
 */
struct Param
{
  double loop_rate;                     // hz
  double max_steering_angle;            // radians
  double steering_factor;               //
  int autoware_control_command_timeout; // ms
};

class ControlConverter : public rclcpp::Node
{
private:
  // parameters
  Param param_;
  bool engage_cmd_;

  // shared msgs
  GearReport::ConstSharedPtr gear_report_ptr_;
  autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_command_ptr_;
  tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr actuation_command_ptr_;
  tier4_api_msgs::msg::DoorStatus::ConstSharedPtr door_status_ptr_;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_ptr_;

  // timestamps
  rclcpp::Time gear_report_received_time_;
  rclcpp::Time gear_command_received_time_;
  rclcpp::Time actuation_command_received_time_;

  // subscribers
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::ConstSharedPtr
    actuation_command_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::ConstSharedPtr
    gear_command_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::GearReport>::ConstSharedPtr
    gear_feedback_sub_;
  rclcpp::Subscription<AutoRemote>::ConstSharedPtr auto_remote_ctrl_command_sub_;
  rclcpp::Subscription<pix_robobus_driver_msgs::msg::VcuReport>::ConstSharedPtr vcu_report_sub_;

  // need to be done
  // emergency command
  // hazard lights command
  // turn indicators command
  // operation mode
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::ConstSharedPtr
    operation_mode_sub_;

  // services
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server_;

  // publishers
  rclcpp::Publisher<ThrottleCommand>::SharedPtr throttle_ctrl_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_ctrl_pub_;
  rclcpp::Publisher<SteeringCommand>::SharedPtr steer_ctrl_pub_;
  rclcpp::Publisher<BrakeCommand>::SharedPtr brake_ctrl_pub_;
  rclcpp::Publisher<ParkCommand>::SharedPtr park_ctrl_pub_;
  rclcpp::Publisher<VehicleModeCommand>::SharedPtr vehicle_ctrl_pub_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  //remote control flag
  int remote_require;
  int current_velocity;

  double parking_brake;

public:
  /**
   * @brief Construct a new Control Converter object
   * 
   */
  ControlConverter();
  /**
   * @brief callback function of actuation command, in order to get accel pedal, brake pedal, steer command
   * 
   * @param msg input message
   */
  void callbackActuationCommand(
    const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr & msg);
  /**
   * @brief callback function of actuation command, in order to get the gear command
   * 
   * @param msg input message
   */
  void callbackGearCommand(
    const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr & msg);
  /**
   * @brief callback function of v2a drive status feedback, in order to get the current gear of vehicle
   * 
   * @param msg input message
   */
  void callbackGearReport(
    const pix_robobus_driver_msgs::msg::GearReport::ConstSharedPtr & msg);

  void callbackOperationMode(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr & msg);

  /**
   * @brief request function to modify control mode AUTO/MANUAL
   * 
   * @param request 
   * @param response 
   */
  void onControlModeRequest(
    const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
    const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);
  /**
   * @brief timer callback function, to evaluate whether if msgs are timeout, than publish control msgs to pix driver control command node
   * 
   */
  void timerCallback();

  void callbackVcuReport(const pix_robobus_driver_msgs::msg::VcuReport::ConstSharedPtr & msg);
  void callbackAutoRemoteControlCommand(const pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg::ConstSharedPtr & msg);
};

} // namespace control_converter
} // namespace pix_robobus_driver

#endif // PIX_ROBOBUS_DRIVER__CONTROL_CONVERTER_HPP_