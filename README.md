
# Thermal Control System (TCS)

**`thermal_control/`** is a ROS 2 C++ package that simulates the **Active Thermal Control System** (ATCS) onboard a space station. It models internal water coolant loops, external ammonia loops, radiator venting, and temperature regulation with heat transfer and hysteresis logic.

---

##  Features

* Internal coolant loop management with real-time temperature simulation
* External ammonia heat exchange logic
* Service-based coolant request and loop temperature reporting
* Heater control using hysteresis to prevent freezing
* Radiator venting service with solar panel visual feedback (via joint rotation)
* Live temperature publishing for dashboard visualization

---

##  Nodes

### 1. `coolant`

Handles:

* Initial loop fill from ECLSS product water
* Temperature simulation with heat gain
* Ammonia coolant tank modeling (temperature, pressure, heater)
* Heat transfer from external loop

**Publishes:**

* `/tcs/ammonia_status` → `TankStatus`
* `/tcs/internal_loop_heat` → `InternalLoopStatus`

**Services:**

* `/tcs/fill_loops` → `std_srvs/Trigger`
* `/tcs/request_ammonia` → `CoolantFlow`
* `/tcs/loop_a/thermal_state` → `InternalLoop`

**Subscribes:**

* `/tcs/external_loop_a/status` → `ExternalLoopStatus`

---

### 2. `external_loop`

Simulates external ammonia loop logic and interaction with the radiator.

**Publishes:**

* `/tcs/external_loop_a/status` → `ExternalLoopStatus`

**Client:**

* Calls `/tcs/request_ammonia` for coolant supply

---

### 3. `radiator_controller`

Listens for radiator venting requests and visually rotates solar panel joints.

**Subscribes:**

* N/A

**Publishes:**

* `/solar_controller/commands` → `std_msgs/Float64MultiArray`

**Services:**

* `/tcs/radiator_a/vent_heat` → `VentHeat`

---

## 📦 Messages & Services

### Custom Messages

* `TankStatus.msg` – Tank temperature, pressure, heater state
* `InternalLoopStatus.msg` – Loop A and B temperatures
* `ExternalLoopStatus.msg` – Inlet/outlet temperatures of external ammonia loop

### Custom Services

* `CoolantFlow.srv` – Request ammonia from tank
* `InternalLoop.srv` – Query internal loop thermal state
* `VentHeat.srv` – Trigger radiator venting action

---

## 🛠️ How to Build

```bash
cd ~/ssos_ws
colcon build --packages-select thermal_control
source install/setup.bash
```

---

## 🚀 Running

**Launch all nodes manually:**

```bash
ros2 launch thermal_control thermals.launch.py
```

