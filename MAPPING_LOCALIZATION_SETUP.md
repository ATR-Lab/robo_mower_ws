# SLAM Setup Guide: Mapping & Localization

This guide explains how to set up and run Simultaneous Localization and Mapping (**SLAM**) using ROS 2. It also includes common troubleshooting tips.

---

## 1. Dependencies

You'll need these packages installed for a basic SLAM setup:

- **ROS 2 Humble**: The robotic operating system (for Ubuntu 22.04).
- **SLAM Package**: `slam_toolbox` is a robust and widely used choice.
- **Visualization**: `rviz2` is essential for viewing the map and robot pose.
- **Navigation Tools**: `nav2_bringup` and `nav2_map_server` are needed for a full navigation stack and map saving.

To install everything, run the following command:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-rviz2
````

-----

## 2\. Running SLAM

Before running any ROS 2 commands, always **source your ROS 2 environment**:

```bash
source /opt/ros/humble/setup.bash
```

### To build a map (mapping)

Run this command while driving the robot around in an unknown environment.

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### To use an existing map (localization)

This command localizes your robot within a pre-built map. Replace `/path/to/map.yaml` with the location of your map file.

```bash
ros2 launch slam_toolbox localization_launch.py map_file:=/path/to/map.yaml
```

-----

## 3\. Saving Maps

Once you've built a satisfactory map, save it using the `map_saver_cli` tool.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This command will create two files in the `~/maps/` directory:

  - `my_map.pgm`: An image of the occupancy grid map.
  - `my_map.yaml`: A metadata file containing information like the map's origin and resolution.

-----

## 4\. Troubleshooting Common Issues

### ❌ TF Errors (Missing Transforms)

**Problem:** "No transform from `odom` to `base_link`"

**Fix:** Ensure your robot is publishing the required transformations (`odom` → `base_link` → `laser`). You can check this with the `tf2_echo` tool.

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

### ❌ Map Not Showing in RViz

**Problem:** The map display is blank.

**Fix:** Verify that the `/map` topic is being published.

```bash
ros2 topic list | grep map
```

### ❌ Map Saving Failed

**Problem:** `[ERROR] [map_saver_cli]: Failed to save map`

**Fix:** Make sure the destination folder exists and you have write permissions. The default save location is `~/maps`.

```bash
mkdir -p ~/maps
chmod a+rw ~/maps
```

### ❌ High CPU Usage

**Problem:** SLAM runs slowly, and your system becomes unresponsive.

**Fix:** Reduce the frequency of laser scans or downsample the scans in your launch file to decrease the computational load.

### ❌ Localization Drift

**Problem:** The robot's estimated position slowly drifts from its true location.

**Fix:** While `slam_toolbox` can handle localization, for robust, long-term localization with a static map, consider using the **Adaptive Monte Carlo Localization** (`amcl`) package, which is part of the `nav2` stack.

-----

**Note:** Once a map is saved, it can be reused later for localization without needing to rebuild it. This is the core concept of separating mapping from localization.

```
```
