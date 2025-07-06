# 🌡️ THERMAL CONTROL SYSTEM — *SSOS DEMO MODULE*

## Overview

This demo implements the **Active Thermal Control System (ATCS)** and **Passive Solar Thermal Modeling** for the HAVEN-2 space station configuration (inspired by VAST). It simulates thermal conduction between spacecraft joints, heat absorption by internal coolant loops, and rejection via radiators (currently visualized via solar array motion).

> Thermal nodes = spacecraft joints
> Thermal links = conduction paths between joint-connected links
> Heat transfer = simulated using physical RK4 integration and control logic

---

##  Installation

### Prerequisites

* ROS 2 Humble
* `colcon`, `tmux`
* Recommended: Gazebo Harmonic and RViz

### 1. Clone and Build Thermal Control

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src
git clone -b v1 https://github.com/space-station-os/demo_thermal_control.git
cd ..
colcon build --symlink-install
```

### 2. Add Required Dependencies

If not already present:

```bash
cd ~/ssos_ws/src
git clone --recurse-submodules -b v0.8.4 https://github.com/space-station-os/space_station_os.git
cd ..
colcon build --symlink-install
```

---

##  Launching the Thermal Control Demo

This demo consists of:

* A **thermal solver node** that parses joints from the URDF and simulates heat conduction
* **Coolant control loops** (2 internal, 1 external) that remove heat when thresholds are crossed
* **Sun vector and solar heat input nodes** for passive solar heating

### 1. Start the Space Station Model

```bash
tmux new -s space_station -d "ros2 launch space_station_description display.launch.py"
```

### 2. Launch Thermal Network with Internal and External Coolant Loops

```bash
tmux new -s thermal_network -d "ros2 launch thermal_control thermals.launch.py"
```

---

##  Sun Vector + Solar Heat Input Nodes

The following nodes simulate **solar flux** based on spacecraft position and orientation, and apply **solar thermal input** to specific panels:

### 3. Launch GNC and Sun Vector System

```bash
tmux new -s demo1c -d "ros2 run space_station_gnc demo1c_small_incident"
tmux new -s gnc_core -d "ros2 launch space_station_gnc gnc_core.launch.py"
tmux new -s sun_nodes -d "ros2 launch thermal_control solar_absorbitivity.launch.py"
```

### Node Behavior

* **`demo1c_small_incident`** publishes spacecraft position and attitude (used for sun vector calc)
* **`gnc_core.launch.py`** includes full dynamics and control system nodes
* **`solar_absorbitivity.launch.py`**:

  * Computes solar incidence angle
  * Applies solar power as heat input to thermal nodes connected to `lsa_*` and `rsa_*` solar panels
  * Will later contribute to radiator heating logic and thermal budget in mission control

---

##  System Workflow

1. **Thermal plugin** reads joints as thermal nodes and creates a conduction network between links.
2. If the **average node temperature** exceeds 1300K:

   * Internal coolant loops (loop A & B) absorb heat from nodes.
3. When internal loop water reaches a threshold:

   * It calls the **external loop** to reject heat via ammonia and radiator panels.
4. Radiators are visually animated by rotating solar arrays (to be replaced with heat indicators).(NOTE FOR VISUALIZATION YOU NEED GAZEBO)
5. **Sun vector and flux** computation simulates solar heating based on orientation.

> This simulates the full Active Thermal Control System like on the ISS, with future plans for heater activation, fault injection, and mission dashboard visualization.

---

## Coming Soon

* Integration with **Open MCT** mission control dashboard


