#include <pix_robobus_driver/control_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pix_robobus_driver::control_command::ControlCommand>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}