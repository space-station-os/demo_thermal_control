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
    loop_mass_kg_(25.0),
    initial_temperature_(10.0),
    current_temperature_(15.0)
{
  water_client_ = this->create_client<space_station_eclss::srv::CleanWater>("/wpa/dispense_water");

  ammonia_server_ = this->create_service<thermal_control::srv::CoolantFlow>(
    "/tcs/request_ammonia", std::bind(&CoolantManager::handle_ammonia, this, _1, _2));

  status_pub_ = this->create_publisher<thermal_control::msg::TankStatus>("/tcs/ammonia_status", 10);

  thermal_state_server_ = this->create_service<thermal_control::srv::InternalLoop>(
    "/tcs/loop_a/thermal_state",
    std::bind(&CoolantManager::handle_thermal_state_request, this, _1, _2));

  control_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&CoolantManager::control_cycle, this));

  loop_temp_pub_ = this->create_publisher<thermal_control::msg::InternalLoopStatus>("/tcs/internal_loop_heat", 10);
  loop_temp_timer_ = this->create_wall_timer(
  std::chrono::seconds(5), std::bind(&CoolantManager::publish_loop_temperatures, this));

  request_water();
}

void CoolantManager::request_water()
{
  if (!water_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Water tank service not available.");
    return;
  }

  auto req = std::make_shared<space_station_eclss::srv::CleanWater::Request>();
  req->water = 100.0;
  req->iodine_level = 0.2;  // safety threshold

  auto future = water_client_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto resp = future.get();
    if (resp->success && resp->delivered_volume > 0.0) {
      double half = resp->delivered_volume / 2.0;
      RCLCPP_INFO(this->get_logger(), "[LOOP FILL] %.2fL received | %.2fL to Loop A | %.2fL to Loop B",
                  resp->delivered_volume, half, half);

      // Simulate that the internal water has picked up heat
      current_temperature_ += 2.5;

      // TODO: simulate loop A and loop B updates with publish or log
    } else {
      RCLCPP_WARN(this->get_logger(), "Water request failed: %s", resp->message.c_str());
    }
  }
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

  RCLCPP_INFO(this->get_logger(), "Ammonia requested: %.2f kg → %s | Remaining: %.2f kg",
              req, response->status_msg.c_str(), ammonia_volume_);
}

void CoolantManager::handle_thermal_state_request(
  const std::shared_ptr<thermal_control::srv::InternalLoop::Request> request,
  std::shared_ptr<thermal_control::srv::InternalLoop::Response> response)
{
  if (!request->status) {
    RCLCPP_WARN(this->get_logger(), "[LOOP A] Thermal state request denied (status=false).");
    response->loop_capacity = 0.0;
    response->temperature = 0.0;
    response->heat_transferred = 0.0;
    return;
  }

  // Simulate water delivery and split
  const double total_water = 100.0;
  const double half = total_water / 2.0;
  const double Cp = 4.186;
  rclcpp::Time now = this->now();

  // Simulate rising temps based on circulation
  current_temperature_ += 2.0;  // Simulated gain
  double loop_a_temp = current_temperature_ + 1.0;
  double loop_b_temp = current_temperature_ + 0.5;

  // Calculate delta T and Q
  double delta_T = current_temperature_ - initial_temperature_;
  delta_T = std::max(0.0, delta_T);
  double Q = loop_mass_kg_ * Cp * delta_T;

  // Fill service response
  response->loop_capacity = loop_mass_kg_;
  response->temperature = current_temperature_;
  response->heat_transferred = Q;

  // Publish loop water status
  thermal_control::msg::InternalLoopStatus temp_msg;
  temp_msg.loop_a.temperature = loop_a_temp;
  temp_msg.loop_a.variance = 0.0;
  temp_msg.loop_a.header.stamp = now;

  temp_msg.loop_b.temperature = loop_b_temp;
  temp_msg.loop_b.variance = 0.0;
  temp_msg.loop_b.header.stamp = now;

  loop_temp_pub_->publish(temp_msg);

  RCLCPP_INFO(this->get_logger(),
    "[LOOP A] Q = %.2f kJ | %.2f kg @ ΔT=%.2f°C | Temps: A=%.2f, B=%.2f",
    Q, loop_mass_kg_, delta_T, loop_a_temp, loop_b_temp);
}

void CoolantManager::publish_loop_temperatures()
{
  // Simulate dynamic temp rise every 5s
  current_temperature_ += 0.8;

  double loop_a_temp = current_temperature_ + 1.0;
  double loop_b_temp = current_temperature_ + 0.5;
  rclcpp::Time now = this->now();

  thermal_control::msg::InternalLoopStatus temp_msg;
  temp_msg.loop_a.temperature = loop_a_temp;
  temp_msg.loop_a.variance = 0.0;
  temp_msg.loop_a.header.stamp = now;

  temp_msg.loop_b.temperature = loop_b_temp;
  temp_msg.loop_b.variance = 0.0;
  temp_msg.loop_b.header.stamp = now;

  loop_temp_pub_->publish(temp_msg);

  RCLCPP_INFO(this->get_logger(), "[TEMP] A: %.2f°C | B: %.2f°C", loop_a_temp, loop_b_temp);
}


void CoolantManager::control_cycle()
{
  // Heater logic
  if (ammonia_temp_ < -10.0) {
    heater_on_ = true;
    ammonia_temp_ += 1.5;
  } else if (ammonia_temp_ > -5.0) {
    heater_on_ = false;
  }

  // Simulate pressure based on temp (simple model)
  ammonia_pressure_ = 101325.0 + (ammonia_temp_ + 20.0) * 1500.0;

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