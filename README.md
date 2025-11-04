# CNI_Custom_LIO-SLAM-COLOUR_isaacsim

![image](docs/lio_sam_output1.gif)

## 🧠 Prerequisites

Ensure the following are properly installed:
- [NVIDIA IsaacSim](https://developer.nvidia.com/isaac-sim)
- [ROS2 Humble](https://docs.ros.org/en/humble/)

## References 
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)
- [LIO-SAM (Color version)](https://github.com/leo-drive/LIO-SAM-COLOR)
- [isaac_sim_pointcloud_tool](https://github.com/liuxiao916/isaac_sim_pointcloud_tool)

---


## 🚀 1. Launch IsaacSim

```bash
cd ~/isaacsim/
./isaac-sim.selector.sh
```

- Click **START**
- Open the scene:  
  ```
  /CNI_Custom_LIO-SLAM-COLOUR_isaacsim/test_env1.usd
  ```
- Press the **Play** button to begin simulation.


## 🔄 2. Build and Source the Project 

```bash
cd ~/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/
source /opt/ros/humble/setup.bash
colcon build && source install/setup.bash
```


## 🛰️ 3. Run LIO-SAM (Color)

```bash
ros2 launch lio_sam full_lio_sam.launch.py
```


## 🕹️ 4. Move the Robot in the Environment

### Publish velocity command:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Or use keyboard control:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Or use controller:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_controller robot_teleop.launch.py
```

## 🌐 5. Save the map

When mapping process is finished, run following command and save slam map to specified path. Be sure you are in LIO-SAM directory.
```bash
source install/setup.bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /Downloads/service_LOAM}"
```

---

## 🐞 Debugging / Optional Steps

## 🔄 6. Convert PointCloud Format to Velodyne (in terminal 1)

```bash
cd ~/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run isaac_sim_pointcloud_tool converter
```


## 🛰️ 7. Run LIO-SAM (Color) (in terminal 2)

```bash
cd ~/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lio_sam run.launch.py
```


## 🌐 8. View the Transform Tree

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_tools view_frames
```

This generates a PDF of the TF tree.

---
