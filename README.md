# stereo_depth_ros2

A ROS2 stereo depth pipeline built around the KITTI stereo dataset.

The pipeline is split into two nodes:
- **`kitti_loader`** reads stereo image pairs from disk and publishes them as ROS2 image topics at a configurable rate, simulating a live stereo camera feed.
- **`stereo_depth_SGBM`** subscribes to the left/right image topics, synchronises matching frame pairs, and runs OpenCV's Semi-Global Block Matching (SGBM) algorithm to compute a disparity map, which it publishes as a ROS2 image topic.

This mirrors a typical real-time stereo depth setup: a camera driver (here, the KITTI loader standing in for one) feeds synchronised stereo frames to a depth estimation node, with each stage decoupled via ROS2 topics.

## Build

```bash
# Only run this if setting up the package for the first time (not needed after git clone)
# ros2 pkg create stereo_depth_ros2 --build-type ament_cmake --dependencies rclcpp sensor_msgs cv_bridge image_transport message_filters

source /opt/ros/jazzy/setup.bash
cd ~/workspace
colcon build
source install/setup.bash
```

## Nodes

### kitti_loader

Loads stereo image pairs from the KITTI dataset and publishes them as ROS2 topics.

**Publishes:**
- `/kitti/left/image_raw` — `sensor_msgs/msg/Image`
- `/kitti/right/image_raw` — `sensor_msgs/msg/Image`

**Run:**
```bash
ros2 run stereo_depth_ros2 kitti_loader --ros-args --params-file src/stereo_depth_ros2/config/params.yaml
```

**Parameters (`config/params.yaml`):**
```yaml
kitti_loader:
  ros__parameters:
    left_image_folder_path: /path/to/kitti/image_2
    right_image_folder_path: /path/to/kitti/image_3
    publish_rate_hz: 10.0
```

---

### stereo_depth_SGBM

Subscribes to stereo image pairs, computes disparity and depth via OpenCV SGBM, and publishes results.

**Subscribes:**
- `/kitti/left/image_raw`
- `/kitti/right/image_raw`

**Publishes:**
- `/stereo/disparity` — `sensor_msgs/msg/Image`

**Run:**
```bash
ros2 run stereo_depth_ros2 stereo_depth_SGBM
```
