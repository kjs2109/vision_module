#include <pix_robobus_driver/control_command.hpp>

namespace pix_robobus_driver
{
namespace control_command
{
ControlCommand::ControlCommand() : Node("control_command")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.command_timeout_ms = declare_parameter("command_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // initialize msg received time, make reservation of data size
  // example brake_command_received_time_ = this->now();
  throttle_command_received_time_ = this->now();
  brake_command_received_time_ = this->now();
  steering_command_received_time_ = this->now();
  gear_command_received_time_ = this->now();
  park_command_received_time_ = this->now();
  vehicle_mode_command_received_time_ = this->now();
  auto_remote_ctrl_received_time_ = this->now();

  is_engage_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from robobus driver autoware interface
    /**
    a2v_brake_ctrl_sub_ = create_subscription<A2vBrakeCtrl>(
      "/pix_robobus/a2v_brakectrl_131", 1, std::bind(&ControlCommand::callbackBrakeCtrl, this, _1));
    **/
    throttle_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::ThrottleCommand>("/pix_robobus/throttle_command", 1, std::bind(&ControlCommand::callbackThrottleCommand, this, _1));
    brake_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::BrakeCommand>("/pix_robobus/brake_command", 1, std::bind(&ControlCommand::callbackBrakeCommand, this, _1));
    steering_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::SteeringCommand>("/pix_robobus/steering_command", 1, std::bind(&ControlCommand::callbackSteeringCommand, this, _1));
    gear_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::GearCommand>("/pix_robobus/gear_command", 1, std::bind(&ControlCommand::callbackGearCommand, this, _1));
    park_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::ParkCommand>("/pix_robobus/park_command", 1, std::bind(&ControlCommand::callbackParkCommand, this, _1));
    vehicle_mode_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::VehicleModeCommand>("/pix_robobus/vehicle_mode_command", 1, std::bind(&ControlCommand::callbackVehicleModeCommand, this, _1));
    auto_remote_ctrl_command_sub_ = create_subscription<pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg>("/pix_robobus/auto_remote_ctrl_msg", 1, std::bind(&ControlCommand::callbackAutoRemoteControlCommand, this, _1));

    // engage
    // engage_ctrl_sub_ = create_subscription<std_msgs::msg::Bool>(
    //   "input/engage", 1, std::bind(&ControlCommand::callbackEngage, this, _1));
  }
  /* publisher */
  {
    // to socketcan drivier
    can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("output/can_tx", rclcpp::QoS{1});
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ControlCommand::timerCallback, this));
  }
}

void ControlCommand::callbackAutoRemoteControlCommand(const pix_robobus_driver_msgs::msg::AutoRemoteCtrlMsg::ConstSharedPtr & msg)
{
    auto_remote_ctrl_received_time_ = this->now();
    auto_remote_ctrl_command_ptr_ = msg;
    remote_status = auto_remote_ctrl_command_ptr_->auto_remote_drive_ctrl_mode;
    RCLCPP_INFO(get_logger(), "remote_require_flag: %d", remote_status);
    auto_ctrl_command_entity_.Reset();
    auto_ctrl_command_entity_.set_auto_drive_ctrl_mode(remote_status);

    can_msgs::msg::Frame auto_control_can_msg;
    auto_control_can_msg.header.stamp = msg->header.stamp;
    auto_control_can_msg.dlc = 8;
    auto_control_can_msg.id = auto_ctrl_command_entity_.ID;
    auto_control_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = auto_ctrl_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        auto_control_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    auto_control_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(auto_control_can_msg);
}

// calback functions
/** example
void ControlCommand::callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg)
{
  brake_command_received_time_ = this->now();
  brake_ctrl_ptr_ = msg;
  a2v_brakectrl_131_entity_.Reset();
  a2v_brakectrl_131_entity_.UpdateData(
    msg->acu_chassis_brake_en, msg->acu_chassis_aeb_ctrl, msg->acu_chassis_brake_pdl_target,
    msg->acu_chassis_epb_ctrl, msg->acu_brake_life_sig, msg->acu_check_sum_131);
  can_msgs::msg::Frame brake_ctrl_can_msg;
  brake_ctrl_can_msg.header.stamp = msg->header.stamp;
  brake_ctrl_can_msg.dlc = 8;
  brake_ctrl_can_msg.id = a2v_brakectrl_131_entity_.ID;
  brake_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_brakectrl_131_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    brake_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  brake_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(brake_ctrl_can_msg);
}
**/


void ControlCommand::callbackThrottleCommand(const pix_robobus_driver_msgs::msg::ThrottleCommand::ConstSharedPtr & msg)
{
    throttle_command_received_time_ = this->now();
    throttle_command_ptr_ = msg;
    throttle_command_entity_.Reset();
    throttle_command_entity_.UpdateData(msg->dirve_speed_target, msg->dirve_acc, msg->check_sum100, msg->dirve_throttle_pedal_target, msg->dirve_en_ctrl);
    can_msgs::msg::Frame throttle_command_can_msg;
    throttle_command_can_msg.header.stamp = msg->header.stamp;
    throttle_command_can_msg.dlc = 8;
    throttle_command_can_msg.id = throttle_command_entity_.ID;
    throttle_command_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = throttle_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        throttle_command_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    throttle_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(throttle_command_can_msg);
}

    

void ControlCommand::callbackBrakeCommand(const pix_robobus_driver_msgs::msg::BrakeCommand::ConstSharedPtr & msg)
{
    brake_command_received_time_ = this->now();
    brake_command_ptr_ = msg;
    brake_command_entity_.Reset();
    brake_command_entity_.UpdateData(msg->aeb_en_ctrl, msg->brake_dec, msg->check_sum101, msg->brake_pedal_target, msg->brake_en_ctrl);
    can_msgs::msg::Frame brake_command_can_msg;
    brake_command_can_msg.header.stamp = msg->header.stamp;
    brake_command_can_msg.dlc = 8;
    brake_command_can_msg.id = brake_command_entity_.ID;
    brake_command_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = brake_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        brake_command_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    brake_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(brake_command_can_msg);

    
}



void ControlCommand::callbackSteeringCommand(const pix_robobus_driver_msgs::msg::SteeringCommand::ConstSharedPtr & msg)
{
    steering_command_received_time_ = this->now();
    steering_command_ptr_ = msg;
    steering_command_entity_.Reset();
    steering_command_entity_.UpdateData(msg->steer_en_ctrl, msg->steer_angle_target, msg->steer_angle_speed, msg->check_sum102);
    can_msgs::msg::Frame steering_command_can_msg;
    steering_command_can_msg.header.stamp = msg->header.stamp;
    steering_command_can_msg.dlc = 8;
    steering_command_can_msg.id = steering_command_entity_.ID;
    steering_command_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = steering_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        steering_command_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    steering_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(steering_command_can_msg);
}



void ControlCommand::callbackGearCommand(const pix_robobus_driver_msgs::msg::GearCommand::ConstSharedPtr & msg)
{
    gear_command_received_time_ = this->now();
    gear_command_ptr_ = msg;
    gear_command_entity_.Reset();
    gear_command_entity_.UpdateData(msg->gear_target, msg->gear_en_ctrl, msg->check_sum103);
    can_msgs::msg::Frame gear_command_can_msg;
    gear_command_can_msg.header.stamp = msg->header.stamp;
    gear_command_can_msg.dlc = 8;
    gear_command_can_msg.id = gear_command_entity_.ID;
    gear_command_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = gear_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        gear_command_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    gear_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(gear_command_can_msg);
}



void ControlCommand::callbackParkCommand(const pix_robobus_driver_msgs::msg::ParkCommand::ConstSharedPtr & msg)
{
    park_command_received_time_ = this->now();
    park_command_ptr_ = msg;
    park_command_entity_.Reset();
    park_command_entity_.UpdateData(msg->check_sum104, msg->park_target, msg->park_en_ctrl);
    can_msgs::msg::Frame park_command_can_msg;
    park_command_can_msg.header.stamp = msg->header.stamp;
    park_command_can_msg.dlc = 8;
    park_command_can_msg.id = park_command_entity_.ID;
    park_command_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = park_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        park_command_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    park_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(park_command_can_msg);
}



void ControlCommand::callbackVehicleModeCommand(const pix_robobus_driver_msgs::msg::VehicleModeCommand::ConstSharedPtr & msg)
{
    vehicle_mode_command_received_time_ = this->now();
    vehicle_mode_command_ptr_ = msg;
    vehicle_mode_command_entity_.Reset();
    vehicle_mode_command_entity_.UpdateData(msg->auto_prompts, msg->clearance_lamp_ctrl, msg->turn_right_prompts, msg->turn_left_prompts, msg->headlight_ctrl, msg->back_up_prompts, msg->vehicle_door_ctrl, msg->check_sum105, msg->turn_light_ctrl, msg->vehicle_vin_req, msg->drive_mode_ctrl, msg->steer_mode_ctrl);
    can_msgs::msg::Frame vehicle_mode_command_can_msg;
    vehicle_mode_command_can_msg.header.stamp = msg->header.stamp;
    vehicle_mode_command_can_msg.dlc = 8;
    vehicle_mode_command_can_msg.id = vehicle_mode_command_entity_.ID;
    vehicle_mode_command_can_msg.is_extended = false;
    uint8_t *signal_bits;
    signal_bits = vehicle_mode_command_entity_.get_data();
    for (int i = 0; i < 8; i++)
    {
        vehicle_mode_command_can_msg.data[i] = *signal_bits;
        signal_bits += 1;
    }
    vehicle_mode_command_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(vehicle_mode_command_can_msg);
}

    

void ControlCommand::callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_engage_ = msg->data;
}

void ControlCommand::timerCallback()
{
    if (!is_engage_) return;
    const rclcpp::Time current_time = this->now();

    // remote control engage 시에는 control 메시지 publish하지 않음
    if (remote_status != 2)
    {
        checkAndPublishCan(throttle_command_received_time_, throttle_command_can_ptr_, "throttle_command");

        // 주석 해제 시 추가 가능
        checkAndPublishCan(brake_command_received_time_, brake_command_can_ptr_, "brake_command");
        checkAndPublishCan(steering_command_received_time_, steering_command_can_ptr_, "steering_command");
        checkAndPublishCan(gear_command_received_time_, gear_command_can_ptr_, "gear_command");
        checkAndPublishCan(park_command_received_time_, park_command_can_ptr_, "park_command");
        checkAndPublishCan(vehicle_mode_command_received_time_, vehicle_mode_command_can_ptr_, "vehicle_mode_command");
    }

    checkAndPublishCan(auto_remote_ctrl_received_time_, auto_control_command_can_ptr_, "remote_control_require");
}

template <typename MsgType>
void ControlCommand::checkAndPublishCan(
    const rclcpp::Time &received_time,
    const std::shared_ptr<const MsgType> &msg_ptr,
    const std::string &msg_name)
{
    const rclcpp::Time current_time = this->now();
    const double delta_time_ms = (current_time - received_time).seconds() * 1000.0;

    if (delta_time_ms > param_.command_timeout_ms || msg_ptr == nullptr)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
            "%s timeout = %f ms.", msg_name.c_str(), delta_time_ms);
        return;
    }

    can_frame_pub_->publish(*msg_ptr);
}

} // namespace control_command
} // namespace pix_robobus_driver
