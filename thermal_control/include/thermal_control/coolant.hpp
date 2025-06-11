#ifndef THERMAL_CONTROL__COOLANT_MANAGER_HPP_
#define THERMAL_CONTROL__COOLANT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "space_station_eclss/srv/clean_water.hpp"
#include "thermal_control/srv/coolant_flow.hpp"
#include "thermal_control/msg/tank_status.hpp"
#include "thermal_control/srv/internal_loop.hpp"
#include "thermal_control/msg/internal_loop_status.hpp"


namespace thermal_control
{

class CoolantManager : public rclcpp::Node
{
public:
  CoolantManager();

private:
  // Tank states
  double tank_capacity_;     // 90.0 kg total
  double ammonia_temp_;      // in °C
  double ammonia_pressure_;  // in Pa
  bool heater_on_;

  double loop_mass_kg_ = 25.0;
  double initial_temperature_ = 10.0;
  double current_temperature_ = 15.0;  // gets updated from simulation
  double ammonia_volume_ = 90.0;


  // ROS Interfaces
  rclcpp::Client<space_station_eclss::srv::CleanWater>::SharedPtr water_client_;
  rclcpp::Service<thermal_control::srv::CoolantFlow>::SharedPtr ammonia_server_;
  rclcpp::Publisher<thermal_control::msg::InternalLoopStatus>::SharedPtr loop_temp_pub_;

  rclcpp::Publisher<thermal_control::msg::TankStatus>::SharedPtr status_pub_;
  rclcpp::Service<thermal_control::srv::InternalLoop>::SharedPtr thermal_state_server_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr loop_temp_timer_;
  // Methods
  void request_water();
  void handle_ammonia(
    const std::shared_ptr<thermal_control::srv::CoolantFlow::Request> request,
    std::shared_ptr<thermal_control::srv::CoolantFlow::Response> response);
  void control_cycle();
  void handle_thermal_state_request(
  const std::shared_ptr<thermal_control::srv::InternalLoop::Request> request,
  std::shared_ptr<thermal_control::srv::InternalLoop::Response> response);
  void publish_loop_temperatures();

};

}  // namespace thermal_control

#endif  // THERMAL_CONTROL__COOLANT_MANAGER_HPP_
