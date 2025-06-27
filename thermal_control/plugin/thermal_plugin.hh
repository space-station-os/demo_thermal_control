#ifndef SPACESTATION__THERMAL_PLUGIN_HH_
#define SPACESTATION__THERMAL_PLUGIN_HH_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>

#include <gz/sim/System.hh>


#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/Util.hh>

#include "rclcpp/rclcpp.hpp"
#include "thermal_control/msg/thermal_node_data_array.hpp"
#include "thermal_control/msg/thermal_link_flows_array.hpp"

#include <thermal_controller/ThermalNodeData.pb.h>
#include <thermal_controller/ThermalLinkFlow.pb.h>

namespace spacestation {

// Struct representing a thermal node (link)
struct ThermalNode
{
  std::string name;
  double temperature = 293.0;     // Kelvin
  double heat_capacity = 1000.0;  // J/K
  double internal_power = 0.0;    // Watts
};

// Struct representing a thermal link (conductance between nodes)
struct ThermalLink
{
  std::string from;
  std::string to;
  double conductance = 0.1;       // W/K
};

class ThermalPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ThermalPlugin();
  ~ThermalPlugin() override;

  // Configure plugin (called once)
  void Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr) override;

  // Called every simulation iteration before physics update
  void PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override;

private:
  // Perform RK4 integration step for thermal simulation
  void rk4_step(double dt);

  struct ThermalPluginPrivate;
  std::unique_ptr<ThermalPluginPrivate> dataPtr;

  // Gazebo transport node and publishers
  gz::transport::Node gz_node_;
  std::shared_ptr<gz::transport::Node::Publisher> node_pub_;
  std::shared_ptr<gz::transport::Node::Publisher> link_pub_;

  // ROS2 interface: node and publishers
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<thermal_control::msg::ThermalNodeDataArray>::SharedPtr ros_node_pub_;
  rclcpp::Publisher<thermal_control::msg::ThermalLinkFlowsArray>::SharedPtr ros_link_pub_;

  std::thread ros_spin_thread_;

  double timestep_ = 0.1;
  double last_time_ = 0.0;
};

}  // namespace spacestation

#endif  // SPACESTATION__THERMAL_PLUGIN_HH_
