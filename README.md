# SC-NDT-mapping
## Overview
This is the extraction of the mapping pacakge from the [Autoware](https://github.com/autowarefoundation/autoware).
The license is in accordance with Autoware.
The loop-closure method implemented in this package referes to [SC-LEGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM). 

这是个基于NDT mapping（参考 Autoware）和 Scan Context 回环检测 （参照 SC-LEGO-LOAM）的建图包。
NDT mapping 原理可参考 [ndt_mapping 算法与代码对应及运行](https://www.notion.so/ndt_mapping-6c8e00266fd1441bab2026fb474b66f6)。
Scan Context 原理可参考大神博客：https://www.guyuehome.com/37852

## Dependency
- [PCL](https://pointclouds.org/)
- [GSTAM](https://gtsam.org/get_started/)

## Usage
 ### Input
  - Point Cloud (/velodyne_points)
  - Odom (optional: used for initial pose guess)
 ### Output
  - /ndt_map
  - /current_pose
 ### Run the packge
 `` roslaunch lidar_localizer ndt_mapping.launch  ``
 
 change rosbag name in launch file
