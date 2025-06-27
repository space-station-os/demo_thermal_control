#ifndef THERMAL_CONTROL__EXTERNAL_LOOP_HPP_
#define THERMAL_CONTROL__EXTERNAL_LOOP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "thermal_control/srv/coolant_flow.hpp"
#include "thermal_control/msg/internal_loop_status.hpp"
#include "thermal_control/msg/external_loop_status.hpp"
#include "thermal_control/srv/vent_heat.hpp"

namespace thermal_control
{

class ExternalLoopA : public rclcpp::Node
{
public:
  ExternalLoopA();

private:
  // State variables
  double ammonia_temp_;
  bool ammonia_filled_;
  bool awaiting_ammonia_response_;

  // ROS interfaces
  rclcpp::Client<thermal_control::srv::CoolantFlow>::SharedPtr ammonia_client_;
  rclcpp::Publisher<thermal_control::msg::ExternalLoopStatus>::SharedPtr loop_status_pub_;
  rclcpp::Subscription<thermal_control::msg::InternalLoopStatus>::SharedPtr internal_sub_;
  rclcpp::Client<thermal_control::srv::VentHeat>::SharedPtr radiator_client_;

  rclcpp::TimerBase::SharedPtr retry_timer_;

  // Core methods
  void try_ammonia_refill();
  void interface_heat_exchanger(const thermal_control::msg::InternalLoopStatus::SharedPtr msg);
};

}  // namespace thermal_control

#endif  // THERMAL_CONTROL__EXTERNAL_LOOP_HPP_
