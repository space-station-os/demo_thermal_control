#ifndef THERMAL_CONTROL__COOLANT_MANAGER_HPP_
#define THERMAL_CONTROL__COOLANT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include "space_station_eclss/srv/clean_water.hpp"
#include "thermal_control/srv/coolant_flow.hpp"
#include "thermal_control/srv/internal_loop.hpp"

#include "thermal_control/msg/tank_status.hpp"
#include "thermal_control/msg/internal_loop_status.hpp"
#include "thermal_control/msg/external_loop_status.hpp"

namespace thermal_control
{

class CoolantManager : public rclcpp::Node
{
public:
  CoolantManager();

private:
  // === Water logic ===
  void request_water();
  void handle_fill_loops(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response>);

  // === Ammonia logic ===
  void handle_ammonia(
    const std::shared_ptr<thermal_control::srv::CoolantFlow::Request>,
    std::shared_ptr<thermal_control::srv::CoolantFlow::Response>);

  // === Internal coolant loop state logic ===
  void handle_thermal_state_request(
    const std::shared_ptr<thermal_control::srv::InternalLoop::Request>,
    std::shared_ptr<thermal_control::srv::InternalLoop::Response>);
  void publish_loop_temperatures();
  void apply_heat_reduction(
    const thermal_control::msg::ExternalLoopStatus::SharedPtr);

  // === Periodic updates ===
  void control_cycle();

  // === ROS interfaces ===
  rclcpp::Client<space_station_eclss::srv::CleanWater>::SharedPtr water_client_;
  rclcpp::Service<thermal_control::srv::CoolantFlow>::SharedPtr ammonia_server_;
  rclcpp::Service<thermal_control::srv::InternalLoop>::SharedPtr thermal_state_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fill_loops_server_;

  rclcpp::Publisher<thermal_control::msg::InternalLoopStatus>::SharedPtr loop_temp_pub_;
  rclcpp::Subscription<thermal_control::msg::ExternalLoopStatus>::SharedPtr loop_temp_sub_;
  rclcpp::Publisher<thermal_control::msg::TankStatus>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // === System state ===
  double tank_capacity_;
  double ammonia_volume_;
  double ammonia_temp_;
  double ammonia_pressure_;
  bool heater_on_;
  bool heater_logged_;  // To prevent heater log spamming

  double loop_mass_kg_;
  double initial_temperature_;
  double current_temperature_;
};

}  // namespace thermal_control

#endif  // THERMAL_CONTROL__COOLANT_MANAGER_HPP_
