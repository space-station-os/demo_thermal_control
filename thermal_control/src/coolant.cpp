#include "thermal_control/coolant.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace thermal_control
{

CoolantManager::CoolantManager()
: Node("coolant_manager"),
  tank_capacity_(90.0),
  ammonia_volume_(90.0),
  ammonia_temp_(-20.0),
  ammonia_pressure_(101325.0),
  heater_on_(false),
  heater_logged_(false),
  loop_mass_kg_(25.0),
  initial_temperature_(10.0),
  current_temperature_(15.0)
{
  water_client_ = this->create_client<space_station_eclss::srv::CleanWater>("/wpa/dispense_water");

  ammonia_server_ = this->create_service<thermal_control::srv::CoolantFlow>(
    "/tcs/request_ammonia", std::bind(&CoolantManager::handle_ammonia, this, _1, _2));

  thermal_state_server_ = this->create_service<thermal_control::srv::InternalLoop>(
    "/tcs/loop_a/thermal_state", std::bind(&CoolantManager::handle_thermal_state_request, this, _1, _2));

  fill_loops_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/tcs/fill_loops", std::bind(&CoolantManager::handle_fill_loops, this, _1, _2));

  loop_temp_pub_ = this->create_publisher<thermal_control::msg::InternalLoopStatus>("/tcs/internal_loop_heat", 10);

  loop_temp_sub_ = this->create_subscription<thermal_control::msg::ExternalLoopStatus>(
    "/tcs/external_loop_a/status", 10, std::bind(&CoolantManager::apply_heat_reduction, this, _1));

  status_pub_ = this->create_publisher<thermal_control::msg::TankStatus>("/tcs/ammonia_status", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&CoolantManager::control_cycle, this));

  publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&CoolantManager::publish_loop_temperatures, this));

  request_water();  // Initial fill
}

void CoolantManager::handle_fill_loops(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  request_water();
  response->success = true;
  response->message = "Internal loops filled with water.";
}

void CoolantManager::request_water()
{
  if (!water_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "[FILL] Water tank service not available.");
    return;
  }

  auto req = std::make_shared<space_station_eclss::srv::CleanWater::Request>();
  req->water = 100.0;
  req->iodine_level = 0.2;

  auto future = water_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto resp = future.get();
    if (resp->success && resp->delivered_volume > 0.0) {
      double half = resp->delivered_volume / 2.0;
      current_temperature_ += 2.5;

      RCLCPP_INFO(this->get_logger(),
        "[LOOP FILL] %.2fL received | %.2fL to Loop A | %.2fL to Loop B | Temp: %.2f°C",
        resp->delivered_volume, half, half, current_temperature_);
    } else {
      RCLCPP_WARN(this->get_logger(), "[FILL] Water request failed: %s", resp->message.c_str());
    }
  }
}

void CoolantManager::apply_heat_reduction(
  const thermal_control::msg::ExternalLoopStatus::SharedPtr msg)
{
  double deltaT = msg->loop_inlet_temp - msg->loop_outlet_temp;
  double before = current_temperature_;
  current_temperature_ = std::max(current_temperature_ - deltaT, 10.0);

  double loop_a_before = before + 1.0;
  double loop_b_before = before + 0.5;
  double loop_a_after = current_temperature_ + 1.0;
  double loop_b_after = current_temperature_ + 0.5;

  RCLCPP_INFO(this->get_logger(),
    "[EXCHANGE] ΔT=%.2f°C | Avg: %.2f→%.2f°C | A: %.2f→%.2f°C | B: %.2f→%.2f°C",
    deltaT, before, current_temperature_,
    loop_a_before, loop_a_after,
    loop_b_before, loop_b_after);
}

void CoolantManager::handle_ammonia(
  const std::shared_ptr<thermal_control::srv::CoolantFlow::Request> request,
  std::shared_ptr<thermal_control::srv::CoolantFlow::Response> response)
{
  double req = request->requested_volume;
  if (ammonia_volume_ >= req) {
    ammonia_volume_ -= req;
    response->granted = true;
    response->status_msg = "Ammonia granted.";
  } else {
    response->granted = false;
    response->status_msg = "Insufficient ammonia.";
  }
  response->current_temperature = ammonia_temp_;
}

void CoolantManager::handle_thermal_state_request(
  const std::shared_ptr<thermal_control::srv::InternalLoop::Request> request,
  std::shared_ptr<thermal_control::srv::InternalLoop::Response> response)
{
  if (!request->status) {
    response->loop_capacity = 0.0;
    response->temperature = 0.0;
    response->heat_transferred = 0.0;
    return;
  }

  const double Cp = 4.186;
  double delta_T = current_temperature_ - initial_temperature_;
  delta_T = std::max(0.0, delta_T);
  double Q = loop_mass_kg_ * Cp * delta_T;

  response->loop_capacity = loop_mass_kg_;
  response->temperature = current_temperature_;
  response->heat_transferred = Q;
}

void CoolantManager::publish_loop_temperatures()
{
  double loop_a_temp = current_temperature_ + 1.0;
  double loop_b_temp = current_temperature_ + 0.5;
  rclcpp::Time now = this->now();

  thermal_control::msg::InternalLoopStatus msg;
  msg.loop_a.temperature = loop_a_temp;
  msg.loop_a.variance = 0.0;
  msg.loop_a.header.stamp = now;

  msg.loop_b.temperature = loop_b_temp;
  msg.loop_b.variance = 0.0;
  msg.loop_b.header.stamp = now;

  loop_temp_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "[TEMP] A: %.2f°C | B: %.2f°C | Avg: %.2f°C",
    loop_a_temp, loop_b_temp, current_temperature_);
}

void CoolantManager::control_cycle()
{
  // Simulate heat gain in internal loop
  current_temperature_ += 0.9;

  // Heater hysteresis logic
  if (ammonia_temp_ < -10.0) {
    if (!heater_on_) {
      heater_on_ = true;
      RCLCPP_INFO(this->get_logger(), "[HEATER] Turning heater ON (%.2f°C)", ammonia_temp_);
    }
  } else if (ammonia_temp_ > -5.0) {
    if (heater_on_) {
      heater_on_ = false;
      RCLCPP_INFO(this->get_logger(), "[HEATER] Turning heater OFF (%.2f°C)", ammonia_temp_);
    }
  }

  // While heater is on, increase ammonia temp
  if (heater_on_) {
    ammonia_temp_ += 1.5;
    RCLCPP_DEBUG(this->get_logger(), "[HEATER] Heating ammonia → %.2f°C", ammonia_temp_);
  }

  // Update pressure based on ammonia temperature
  ammonia_pressure_ = 101325.0 + (ammonia_temp_ + 20.0) * 1500.0;

  // Publish tank status
  thermal_control::msg::TankStatus status;
  status.tank_capacity = tank_capacity_;

  status.tank_temperature.temperature = ammonia_temp_;
  status.tank_temperature.variance = 0.0;
  status.tank_temperature.header.stamp = this->now();

  status.tank_pressure.fluid_pressure = ammonia_pressure_;
  status.tank_pressure.variance = 0.0;
  status.tank_pressure.header.stamp = this->now();

  status.tank_heater_on = heater_on_;
  status_pub_->publish(status);
}


}  // namespace thermal_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thermal_control::CoolantManager>());
  rclcpp::shutdown();
  return 0;
}
