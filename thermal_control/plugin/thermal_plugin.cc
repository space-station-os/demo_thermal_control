#include "thermal_plugin.hh"

#include <mutex>
#include <string>
#include <optional>
#include <chrono>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace spacestation;
using namespace gz;
using namespace gz::sim;

struct ThermalPlugin::ThermalPluginPrivate
{
  Model model{gz::sim::kNullEntity};

  // Map of link name -> node index in nodes_ vector
  std::unordered_map<std::string, size_t> node_index_;

  // Store thermal nodes and links
  std::vector<ThermalNode> nodes_;
  std::vector<ThermalLink> links_;
};

ThermalPlugin::ThermalPlugin()
  : dataPtr(std::make_unique<ThermalPluginPrivate>())
{
  gzdbg << "[ThermalPlugin] Constructor called." << std::endl;
}

ThermalPlugin::~ThermalPlugin()
{
  gzdbg << "[ThermalPlugin] Destructor called." << std::endl;

  if (ros_node_)
    rclcpp::shutdown();

  if (ros_spin_thread_.joinable())
    ros_spin_thread_.join();
}

void ThermalPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  gzdbg << "[ThermalPlugin] Configure started." << std::endl;

  // Initialize ROS2 node and publishers
  ros_node_ = std::make_shared<rclcpp::Node>("thermal_plugin_node");
  ros_node_pub_ = ros_node_->create_publisher<thermal_control::msg::ThermalNodeDataArray>(
      "/thermal/nodes/state", 10);
  ros_link_pub_ = ros_node_->create_publisher<thermal_control::msg::ThermalLinkFlowsArray>(
      "/thermal/links/flux", 10);

  ros_spin_thread_ = std::thread([this]() {
    gzdbg << "[ThermalPlugin] ROS2 spin thread started." << std::endl;
    rclcpp::spin(this->ros_node_);
    gzdbg << "[ThermalPlugin] ROS2 spin thread exited." << std::endl;
  });

  // Store model entity
  dataPtr->model = Model(_entity);
  if (!dataPtr->model.Valid(_ecm))
  {
    gzerr << "[ThermalPlugin] Model is invalid. Aborting." << std::endl;
    return;
  }

  // Optional: default params from root SDF
  double default_initial_temp = 293.15;
  double default_heat_capacity = 1000.0;
  double default_internal_power = 0.0;
  double default_conductance = 0.1;

  if (_sdf->HasElement("default_initial_temperature"))
    default_initial_temp = _sdf->Get<double>("default_initial_temperature");
  if (_sdf->HasElement("default_heat_capacity"))
    default_heat_capacity = _sdf->Get<double>("default_heat_capacity");
  if (_sdf->HasElement("default_internal_power"))
    default_internal_power = _sdf->Get<double>("default_internal_power");
  if (_sdf->HasElement("default_conductance"))
    default_conductance = _sdf->Get<double>("default_conductance");

  // --- Now read joint_name from this plugin instance ---
  if (!_sdf->HasElement("joint_name"))
  {
    gzerr << "[ThermalPlugin] Missing required <joint_name> element. Aborting." << std::endl;
    return;
  }
  std::string jointName = _sdf->Get<std::string>("joint_name");

  Entity jointEntity = dataPtr->model.JointByName(_ecm, jointName);
  if (jointEntity == kNullEntity)
  {
    gzerr << "[ThermalPlugin] Joint [" << jointName << "] not found in model. Aborting." << std::endl;
    return;
  }

  // Get child link name
  auto childLinkComp = _ecm.Component<components::ChildLinkName>(jointEntity);
  if (!childLinkComp)
  {
    gzerr << "[ThermalPlugin] Joint [" << jointName << "] missing ChildLinkName component. Aborting." << std::endl;
    return;
  }
  std::string childLinkName = childLinkComp->Data();

  // Get parent link name
  Entity parentLinkEntity = _ecm.ParentEntity(jointEntity);
  if (parentLinkEntity == kNullEntity)
  {
    gzerr << "[ThermalPlugin] Joint [" << jointName << "] missing parent entity. Aborting." << std::endl;
    return;
  }
  std::string parentLinkName = scopedName(parentLinkEntity, _ecm, "/", false);

  // Helper lambda: add thermal node if missing, with optional overrides from this plugin's sdf element
  auto addNodeIfMissing = [&](const std::string &linkName) {
    if (dataPtr->node_index_.count(linkName) == 0)
    {
      ThermalNode node;
      node.name = jointName;
      node.temperature = default_initial_temp;
      node.heat_capacity = default_heat_capacity;
      node.internal_power = default_internal_power;

      // Per-node overrides from this plugin's sdf nested elements, e.g. <initial_temperature_linkName>, etc.
      std::string tempParam = "initial_temperature_" + jointName;
      if (_sdf->HasElement(tempParam))
        node.temperature = _sdf->Get<double>(tempParam);

      std::string heatCapParam = "heat_capacity_" + jointName;
      if (_sdf->HasElement(heatCapParam))
        node.heat_capacity = _sdf->Get<double>(heatCapParam);

      std::string internalPowerParam = "internal_power_" + jointName;
      if (_sdf->HasElement(internalPowerParam))
        node.internal_power = _sdf->Get<double>(internalPowerParam);

      dataPtr->node_index_[linkName] = dataPtr->nodes_.size();
      dataPtr->nodes_.push_back(std::move(node));
    }
  };

  // Add parent and child nodes if missing
  addNodeIfMissing(parentLinkName);
  addNodeIfMissing(childLinkName);

  // Create thermal link from parent to child with per-plugin conductance or default
  ThermalLink tlink;
  tlink.from = childLinkName;
  tlink.to = parentLinkName;

  if (_sdf->HasElement("conductance"))
    tlink.conductance = _sdf->Get<double>("conductance");
  else
    tlink.conductance = default_conductance;

  dataPtr->links_.push_back(std::move(tlink));

  // Setup Gazebo transport publishers
  node_pub_ = std::make_shared<gz::transport::Node::Publisher>(
      gz_node_.Advertise<thermal_controller::ThermalNodeData_V>("/thermal/nodes/state"));

  link_pub_ = std::make_shared<gz::transport::Node::Publisher>(
      gz_node_.Advertise<thermal_controller::ThermalLinkFlow_V>("/thermal/links/flux"));

  last_time_ = 0.0;
  timestep_ = 0.1;

  std::cout << "[ThermalPlugin] Configured with " << dataPtr->nodes_.size()
            << " thermal nodes and " << dataPtr->links_.size() << " thermal links." << std::endl;
}

void ThermalPlugin::rk4_step(double dt)
{
  // RK4 step on all node temperatures, heat flows on links
  std::vector<double> k1(dataPtr->nodes_.size(), 0.0);
  std::vector<double> k2(dataPtr->nodes_.size(), 0.0);
  std::vector<double> k3(dataPtr->nodes_.size(), 0.0);
  std::vector<double> k4(dataPtr->nodes_.size(), 0.0);

  auto compute_derivative = [&](std::vector<double> &dT, const std::vector<double> &temp) {
    for (auto &v : dT) v = 0.0;

    for (const auto &link : dataPtr->links_)
    {
      size_t i = dataPtr->node_index_.at(link.from);
      size_t j = dataPtr->node_index_.at(link.to);
      double deltaT = temp[i] - temp[j];
      double q = link.conductance * deltaT;

      dT[i] -= q / dataPtr->nodes_[i].heat_capacity;
      dT[j] += q / dataPtr->nodes_[j].heat_capacity;
    }

    for (size_t i = 0; i < dataPtr->nodes_.size(); ++i)
      dT[i] += dataPtr->nodes_[i].internal_power / dataPtr->nodes_[i].heat_capacity;
  };

  std::vector<double> temp0(dataPtr->nodes_.size());
  for (size_t i = 0; i < dataPtr->nodes_.size(); ++i)
    temp0[i] = dataPtr->nodes_[i].temperature;

  compute_derivative(k1, temp0);
  std::vector<double> temp1 = temp0;
  for (size_t i = 0; i < temp1.size(); ++i)
    temp1[i] += 0.5 * dt * k1[i];

  compute_derivative(k2, temp1);
  std::vector<double> temp2 = temp0;
  for (size_t i = 0; i < temp2.size(); ++i)
    temp2[i] += 0.5 * dt * k2[i];

  compute_derivative(k3, temp2);
  std::vector<double> temp3 = temp0;
  for (size_t i = 0; i < temp3.size(); ++i)
    temp3[i] += dt * k3[i];

  compute_derivative(k4, temp3);

  for (size_t i = 0; i < dataPtr->nodes_.size(); ++i)
    dataPtr->nodes_[i].temperature += (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
}

void ThermalPlugin::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager & /*ecm*/)
{
  double sim_time = std::chrono::duration<double>(info.simTime).count();
  if (sim_time - last_time_ < timestep_)
    return;

  rk4_step(timestep_);
  last_time_ = sim_time;

  gzdbg << "[ThermalPlugin] Publishing thermal node and link states at sim time: " << sim_time << std::endl;

  // Gazebo transport messages
  thermal_controller::ThermalNodeData_V nodes_msg;
  for (const auto &node : dataPtr->nodes_)
  {
    auto *entry = nodes_msg.add_states();
    entry->set_name(node.name);
    entry->set_temperature(node.temperature);
    entry->set_heat_capacity(node.heat_capacity);
    entry->set_internal_power(node.internal_power);
  }
  node_pub_->Publish(nodes_msg);

  thermal_controller::ThermalLinkFlow_V links_msg;
  for (const auto &link : dataPtr->links_)
  {
    size_t i = dataPtr->node_index_.at(link.from);
    size_t j = dataPtr->node_index_.at(link.to);
    double q = link.conductance * (dataPtr->nodes_[i].temperature - dataPtr->nodes_[j].temperature);

    auto *flow = links_msg.add_flows();
    flow->set_from(link.from);
    flow->set_to(link.to);
    flow->set_conductance(link.conductance);
    flow->set_heat_flow(q);
  }
  link_pub_->Publish(links_msg);

  // ROS2 messages
  thermal_control::msg::ThermalNodeDataArray ros_nodes_msg;
  for (const auto &node : dataPtr->nodes_)
  {
    thermal_control::msg::ThermalNodeData entry;
    entry.name = node.name;
    entry.temperature = node.temperature;
    entry.heat_capacity = node.heat_capacity;
    entry.internal_power = node.internal_power;
    ros_nodes_msg.nodes.push_back(entry);
  }
  ros_node_pub_->publish(ros_nodes_msg);

  thermal_control::msg::ThermalLinkFlowsArray ros_links_msg;
  for (const auto &link : dataPtr->links_)
  {
    size_t i = dataPtr->node_index_.at(link.from);
    size_t j = dataPtr->node_index_.at(link.to);
    double q = link.conductance * (dataPtr->nodes_[i].temperature - dataPtr->nodes_[j].temperature);

    thermal_control::msg::ThermalLinkFlows flow;
    flow.node_a = link.from;
    flow.node_b = link.to;
    flow.conductance = link.conductance;
    flow.heat_flow = q;
    ros_links_msg.links.push_back(flow);
  }
  ros_link_pub_->publish(ros_links_msg);
}

GZ_ADD_PLUGIN(
  spacestation::ThermalPlugin,
  gz::sim::System,
  spacestation::ThermalPlugin::ISystemConfigure,
  spacestation::ThermalPlugin::ISystemPreUpdate
)
