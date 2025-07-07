# CNI_Custom_LIO-SLAM-COLOUR_isaacsim

## 🧠 Prerequisites

Ensure the following are properly installed:
- [NVIDIA IsaacSim](https://developer.nvidia.com/isaac-sim)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)
- [LIO-SAM (Color version)](https://github.com/leo-drive/LIO-SAM-COLOR)
- [isaac_sim_pointcloud_tool](https://github.com/liuxiao916/isaac_sim_pointcloud_tool)

---

### 🚀 1. Launch IsaacSim

```bash
cd ~/isaacsim/
./isaac-sim.selector.sh
```

- Click **START**
- Open the scene:  
  ```
  /CNI_Custom_LIO-SLAM-COLOUR_isaacsim/test.usd
  ```
- Press the **Play** button to begin simulation.

---

## 🔄 2. Convert PointCloud Format to Velodyne

```bash
cd ~/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run isaac_sim_pointcloud_tool converter
```

---

## 🛰️ 3. Run LIO-SAM (Color)

```bash
cd ~/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lio_sam run.launch.py
```

---

## 🕹️ 4. Move the Robot in the Environment

### Publish velocity command:

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Or use keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🌐 5. View the Transform Tree

```bash
source /opt/ros/humble/setup.bash

ros2 run tf2_tools view_frames
```

This generates a PDF of the TF tree.

---
