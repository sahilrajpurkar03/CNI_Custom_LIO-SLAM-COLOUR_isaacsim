# Mapping and Localization Status Report

## 🚧 Current Issues

> **Note:** Firstly, the problems need to be solved with the single camera setup before addressing the issues with all four cameras.

### Single Camera Setup
- ⬜ Re-registered point clouds make the global map representation unclear.  
- ⬜ Point cloud registration process stops unexpectedly after running for some time.  

### Four Camera Setup
- ⬜ Re-registration of point clouds results in a degraded color global map compared to the single-camera setup.  
- ⬜ Overlapping regions from multiple cameras introduce noise in the generated map.  


---

## ✅ Issues Resolved

- ✅ **Incorrect point cloud alignment during robot rotation (single camera)**  
  **Cause:** Orientation mismatch in Isaac Sim.  
  **Solution:** Corrected orientation and updated corresponding logic in `mapOptimization.cpp`.  

- ✅ **Improper robot movement in the simulation environment**  
  **Cause:** The robot was controlled using a differential drive controller, which is unsuitable for the Carter robot.  
  **Solution:** Implemented an Ackermann steering controller for accurate motion behavior.  

- ✅ **Incorrect camera TFs (upside-down and rotated 180° about Y-axis)**  
  **Cause:** Mismatch between Isaac Sim’s USD + OpenGL coordinate convention and ROS camera optical frame.  
  **Coordinate Conventions:**  
  - *Isaac Sim:* X → right, Y → up, Z → backward (into the screen)  
  - *ROS:* X → right, Y → down, Z → forward  
  **Solution:** Adjusted camera orientation settings in Isaac Sim to match ROS conventions.  

- ✅ **Low feature extraction**  
  **Cause:** Camera misalignment.  
  **Solution:** Corrected camera alignment, improving feature detection.  

- ✅ **Camera images not used for projection and coloring**  
  **Cause:** Incorrect camera alignment.  
  **Solution:** Fixed alignment to enable accurate color projection.  

- ✅ **Incomplete point cloud registration**  
  **Cause:** Misalignment between camera and camera link frames.  
  **Solution:** Corrected TF alignment to ensure all point clouds register properly.  

- ✅ **Low LiDAR publish rate (~2 Hz)**  
  **Solution:** Increased LiDAR publishing frequency to achieve optimal mapping performance.  

- ✅ **Unstable mapping due to odometry drift**  
  **Solution:** Adjusted odometry and mapping parameters to stabilize pose estimation.  

- ✅ **TF tree issue (`lidar_link` not connected to `base_link`)**  
  **Solution:** Fixed TF connections to ensure proper hierarchy in the transform tree.  

- ✅ **Camera position and orientation not updating**  
  **Solution:** Corrected TF broadcasting to ensure real-time camera pose updates.  
