# THERMAL CONTROLS

### INSTALLATION 

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src 
git clone -b v1 https://github.com/space-station-os/demo_thermal_control.git
cd ..
colcon build --symlink-install 
```
> Please ensure you have both space_station_eclss and space_station_description files before building. if not follow the instructions below

```bash
cd ~/ssos_ws/src
git clone --recurse-submodules -b v0.8.4 https://github.com/space-station-os/space_station_os.git
cd ..
colcon build --symlink-install 
```

### DEMONSTRATION 

Current Thermal controls consists of thermal nodes and links which it reads from the urdf file. While the other coolant nodes simulate the heat release from these nodes to the radiators (Currently for visuals, we are rotating the solar arrays, but will later indicate with something TBD).

##### LAUNCH 

```
sudo apt-get install tmux
tmux new -s gz -d "ros2 launch space_station_description gazebo.launch.py" 
```

This should give you two topics 

```bash
/thermal/links/flux
/thermal/nodes/state
```




