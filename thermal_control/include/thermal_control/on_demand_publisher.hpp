#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thermal_control/msg/thermal_node_data_array.hpp>
#include <thermal_control/msg/thermal_node_data.hpp>
#include <thermal_control/srv/get_sub_topic.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace thermal_control{
class FocusNodeMonitor : public rclcpp::Node
{
public:
  FocusNodeMonitor();

private:
  void thermal_node_callback(const thermal_control::msg::ThermalNodeDataArray::SharedPtr msg);
  void handle_focus_request(
    const std::shared_ptr<thermal_control::srv::GetSubTopic::Request> request,
    std::shared_ptr<thermal_control::srv::GetSubTopic::Response> response);

  rclcpp::Subscription<thermal_control::msg::ThermalNodeDataArray>::SharedPtr node_sub_;
  rclcpp::Service<thermal_control::srv::GetSubTopic>::SharedPtr service_;

  std::unordered_map<std::string, thermal_control::msg::ThermalNodeData> latest_nodes_;
  std::unordered_map<std::string, rclcpp::Publisher<thermal_control::msg::ThermalNodeData>::SharedPtr> focused_publishers_;
  std::unordered_set<std::string> focused_names_;
};
}
