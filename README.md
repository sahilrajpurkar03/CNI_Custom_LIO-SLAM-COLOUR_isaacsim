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
## Usage Note

Currently, the code includes a patch to handle **Isaac Sim (Omniverse) simulation**, where camera images are upside-down relative to ROS optical frame.  

**Important:** When using a **real robot**, remember to **swap back to the original patch**.

**Reason:** On a real robot, the camera images already match ROS camera optical frame, so the Isaac Sim vertical flip is unnecessary and will invert the colors.


### Original Patch for Real Robot (LIO-SAM)

**File:** `src/mapOptmization.cpp`  
**Section:** Update ONLY the sampling section in both loops (corner points and surface points).

```cpp
// Project point to camera image and get color
double x = point3d_transformed_camera[0] / point3d_transformed_camera[2];
double y = point3d_transformed_camera[1] / point3d_transformed_camera[2];

int px = static_cast<int>(std::round(x));
int py = static_cast<int>(std::round(y));

int width  = pair.second->get_image_width();
int height = pair.second->get_image_height();

if (px >= 0 && px < width &&
    py >= 0 && py < height &&
    point3d_transformed_camera[2] > 0)
{
    cv::Vec3b color =
        pair.second->get_cv_image().at<cv::Vec3b>(cv::Point(px, py));

    if (pair.second->get_image_msg()->encoding == "rgb8") 
    {
        pointRGB.r = color[0];
        pointRGB.g = color[1];
        pointRGB.b = color[2];
    }
    else if (pair.second->get_image_msg()->encoding == "bgr8")
    {
        pointRGB.r = color[2];
        pointRGB.g = color[1];
        pointRGB.b = color[0];
    }
    else
    {
        pointRGB.r = color[0];
        pointRGB.g = color[1];
        pointRGB.b = color[2];
    }
}
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
