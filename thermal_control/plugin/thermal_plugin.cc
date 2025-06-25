#include "thermal_plugin.hh"



using namespace spacestation;

ThermalPlugin::ThermalPlugin() {}

void ThermalPlugin::Configure(
  const gz::sim::Entity &entity,
  const std::shared_ptr<const sdf::Element> & /*sdf*/,
  gz::sim::EntityComponentManager &ecm,
  gz::sim::EventManager & /*eventMgr*/)
{
  gz::sim::Model model(entity);
  if (!model.Valid(ecm)) {
    std::cerr << "[ThermalPlugin] Invalid model entity." << std::endl;
    return;
  }

  auto links = model.Links(ecm);
  for (const auto &link : links) {
    auto name = gz::sim::scopedName(link, ecm, "/", false);

    ThermalNode node;
    node.name = name;
    node.temperature = 290.0 + static_cast<double>(rand() % 1000) / 100.0;
    node.heat_capacity = 1000.0;
    node.internal_power = 0.0;

    node_index_[name] = nodes_.size();
    nodes_.push_back(node);
  }

  for (size_t i = 0; i < nodes_.size(); ++i) {
    for (size_t j = i + 1; j < nodes_.size(); ++j) {
      ThermalLink link;
      link.from = nodes_[i].name;
      link.to = nodes_[j].name;
      link.conductance = 0.05;
      links_.push_back(link);
    }
  }

  node_pub_ = std::make_shared<gz::transport::Node::Publisher>(
    gz_node_.Advertise<thermal_controller::ThermalNodeData_V>("/thermal/nodes/state"));

  link_pub_ = std::make_shared<gz::transport::Node::Publisher>(
    gz_node_.Advertise<thermal_controller::ThermalLinkFlow_V>("/thermal/links/flux"));
  

  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("thermal_plugin_node");

  ros_node_pub_ = ros_node_->create_publisher<thermal_control::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10);
  ros_link_pub_ = ros_node_->create_publisher<thermal_control::msg::ThermalLinkFlowsArray>(
    "/thermal/links/flux", 10);

  ros_spin_thread_ = std::thread([this]() {
      rclcpp::spin(this->ros_node_);
    });


  std::cout << "[ThermalPlugin] Configured with " << nodes_.size() << " nodes and " << links_.size() << " links.\n";
}

void ThermalPlugin::rk4_step(double dt)
{
  std::vector<double> k1(nodes_.size(), 0.0);
  std::vector<double> k2(nodes_.size(), 0.0);
  std::vector<double> k3(nodes_.size(), 0.0);
  std::vector<double> k4(nodes_.size(), 0.0);

  auto compute_derivative = [&](std::vector<double> &dT, const std::vector<double> &temp) {
    for (auto &v : dT) v = 0.0;

    for (const auto &link : links_) {
      size_t i = node_index_[link.from];
      size_t j = node_index_[link.to];
      double deltaT = temp[i] - temp[j];
      double q = link.conductance * deltaT;

      dT[i] -= q / nodes_[i].heat_capacity;
      dT[j] += q / nodes_[j].heat_capacity;
    }

    for (size_t i = 0; i < nodes_.size(); ++i)
      dT[i] += nodes_[i].internal_power / nodes_[i].heat_capacity;
  };

  std::vector<double> temp0(nodes_.size());
  for (size_t i = 0; i < nodes_.size(); ++i)
    temp0[i] = nodes_[i].temperature;

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

  for (size_t i = 0; i < nodes_.size(); ++i)
    nodes_[i].temperature += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
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

  // Publish node states
  thermal_controller::ThermalNodeData_V node_msg;
  for (const auto &node : nodes_) {
    auto *entry = node_msg.add_states();
    entry->set_name(node.name);
    entry->set_temperature(node.temperature);
    entry->set_heat_capacity(node.heat_capacity);
    entry->set_internal_power(node.internal_power);
  }
  node_pub_->Publish(node_msg);

  // Publish heat flow between links
  thermal_controller::ThermalLinkFlow_V link_msg;
  for (const auto &link : links_) {
    size_t i = node_index_[link.from];
    size_t j = node_index_[link.to];
    double q = link.conductance * (nodes_[i].temperature - nodes_[j].temperature);

    auto *flow = link_msg.add_flows();
    flow->set_from(link.from);
    flow->set_to(link.to);
    flow->set_conductance(link.conductance);
    flow->set_heat_flow(q);
  }
  link_pub_->Publish(link_msg);


  thermal_control::msg::ThermalNodeDataArray ros_node_msg;
  for (const auto &node : nodes_) {
    thermal_control::msg::ThermalNodeData entry;
    entry.name = node.name;
    entry.temperature = node.temperature;
    entry.heat_capacity = node.heat_capacity;
    entry.internal_power = node.internal_power;
    ros_node_msg.nodes.push_back(entry);
  }
  ros_node_pub_->publish(ros_node_msg);

  thermal_control::msg::ThermalLinkFlowsArray ros_link_msg;
  for (const auto &link : links_) {
    size_t i = node_index_[link.from];
    size_t j = node_index_[link.to];
    double q = link.conductance * (nodes_[i].temperature - nodes_[j].temperature);

    thermal_control::msg::ThermalLinkFlows flow;
    flow.node_a = link.from;
    flow.node_b = link.to;
    flow.conductance = link.conductance;
    flow.heat_flow = q;
    ros_link_msg.links.push_back(flow);
  }
  ros_link_pub_->publish(ros_link_msg);

}

GZ_ADD_PLUGIN(
  spacestation::ThermalPlugin,
  gz::sim::System,
  spacestation::ThermalPlugin::ISystemConfigure,
  spacestation::ThermalPlugin::ISystemPreUpdate
)
