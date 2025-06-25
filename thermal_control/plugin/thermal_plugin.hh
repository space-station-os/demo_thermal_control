#ifndef SPACESTATION__THERMAL_PLUGIN_HH_
#define SPACESTATION__THERMAL_PLUGIN_HH_

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <google/protobuf/text_format.h>

#include <thermal_controller/ThermalNodeData.pb.h>
#include <thermal_controller/ThermalLinkFlow.pb.h>

#include "rclcpp/rclcpp.hpp"
#include "thermal_control/msg/thermal_node_data_array.hpp"
#include "thermal_control/msg/thermal_link_flows_array.hpp"

namespace spacestation {

struct ThermalNode {
  std::string name;
  double temperature = 293.0;
  double heat_capacity = 1000.0;
  double internal_power = 0.0;
};

struct ThermalLink {
  std::string from;
  std::string to;
  double conductance = 0.1;
};

class ThermalPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ThermalPlugin();

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;
  ~ThermalPlugin() {
      if (ros_spin_thread_.joinable())
        ros_spin_thread_.join();
      if (rclcpp::ok())
        rclcpp::shutdown();
    }


private:
  void rk4_step(double dt);
  std::thread ros_spin_thread_;

  gz::transport::Node gz_node_;
  std::shared_ptr<gz::transport::Node::Publisher> node_pub_;
  std::shared_ptr<gz::transport::Node::Publisher> link_pub_;

  std::vector<ThermalNode> nodes_;
  std::vector<ThermalLink> links_;
  std::unordered_map<std::string, size_t> node_index_;

  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<thermal_control::msg::ThermalNodeDataArray>::SharedPtr ros_node_pub_;
  rclcpp::Publisher<thermal_control::msg::ThermalLinkFlowsArray>::SharedPtr ros_link_pub_;


  double timestep_ = 0.1;
  double last_time_ = 0.0;
};

}  // namespace spacestation

#endif  // SPACESTATION__THERMAL_PLUGIN_HH_
