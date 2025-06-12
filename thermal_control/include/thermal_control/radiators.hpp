#ifndef THERMAL_CONTROL__RADIATOR_CONTROLLER_HPP_
#define THERMAL_CONTROL__RADIATOR_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "thermal_control/srv/vent_heat.hpp"

namespace thermal_control
{

class RadiatorController : public rclcpp::Node
{
public:
  RadiatorController();

private:
  void handle_vent_request(
    const std::shared_ptr<thermal_control::srv::VentHeat::Request>,
    std::shared_ptr<thermal_control::srv::VentHeat::Response>);

  rclcpp::Service<thermal_control::srv::VentHeat>::SharedPtr vent_service_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
};

}  // namespace thermal_control

#endif  // THERMAL_CONTROL__RADIATOR_CONTROLLER_HPP_
