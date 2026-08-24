# p03_slam_ros

ROS 2 integration for the ROS-independent `p03_slam_core` library.

The package provides two nodes:

- `wheel_imu_odometry_node` uses wheel travel for translation and IMU yaw rate
  for rotation.
- `p03_slam_node` synchronizes odometry and laser scans, runs likelihood-field
  scan matching, updates the occupancy grid, and publishes TF.

## Build and test

Run from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to p03_slam_ros --symlink-install
source install/setup.bash

colcon test --packages-select p03_slam_core p03_slam_ros
colcon test-result --verbose
```

The test result should report zero errors and zero failures.

## Indoor simulation

Start the simulator in terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor rviz:=False
```

Start odometry and SLAM in terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch p03_slam_ros indoor_slam.launch.py
```

The launch file applies the `P1_robot0` namespace. Therefore, relative names
such as `lidar_2d` resolve to `/P1_robot0/lidar_2d`.

Start keyboard control in terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -r cmd_vel:=/P1_robot0/cmd_vel
```

Stop all terminals with `Ctrl+C`. Restart Gazebo after each short test run if
its memory usage continues to grow.

## Runtime checks

Check the laser type and rate:

```bash
ros2 topic type /P1_robot0/lidar_2d
ros2 topic hz /P1_robot0/lidar_2d
```

Expected result: `sensor_msgs/msg/LaserScan` at approximately 10 Hz.

Check the fused odometry pose and rate:

```bash
ros2 topic echo /P1_robot0/wheel_imu_odom --once --field pose.pose
ros2 topic hz /P1_robot0/wheel_imu_odom
```

Compare fused odometry with simulation ground truth during debugging:

```bash
ros2 topic echo /P1_robot0/wheel_imu_odom --once --field pose.pose
ros2 topic echo /P1_robot0/odom_gt --once --field pose.pose
```

`odom_gt` is only a simulator diagnostic reference. The SLAM node does not use
it.

Inspect the resolved subscriptions and publishers:

```bash
ros2 node info /P1_robot0/wheel_imu_odometry
ros2 node info /P1_robot0/p03_slam
```

Check the static base-to-lidar transform:

```bash
ros2 run tf2_ros tf2_echo \
  P1_robot0/base_link \
  P1_robot0/lidar2d_link
```

Expected translation:

```text
x = 0.175 m
y = 0.000 m
z = 0.231 m
```

Save the current occupancy grid for visual inspection:

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /P1_robot0/map \
  -f /tmp/p03_slam_map
```

This creates `/tmp/p03_slam_map.pgm` and `/tmp/p03_slam_map.yaml`.
