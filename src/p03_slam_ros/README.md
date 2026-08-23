# p03_slam_ros

ROS 2 integration for the ROS-independent `p03_slam_core` library.

The current node subscribes to a planar laser scan, converts each ROS message to
`p03_slam_core::LaserScan2D`, and reports the number of received beams.

## Build

Run from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to p03_slam_ros --symlink-install
source install/setup.bash
```

Confirm that ROS 2 can find the executable:

```bash
ros2 pkg executables p03_slam_ros
```

Expected executable:

```text
p03_slam_ros p03_slam_node
```

## Test

```bash
colcon test --packages-select p03_slam_ros
colcon test-result --verbose
```

The result should report zero errors and zero failures.

## Indoor lidar smoke test

Start the indoor simulation in terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor rviz:=False
```

Inspect the lidar topic in terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic type /P1_robot0/lidar_2d
ros2 topic info /P1_robot0/lidar_2d --verbose
ros2 topic hz /P1_robot0/lidar_2d
```

Expected type and rate:

```text
sensor_msgs/msg/LaserScan
approximately 10 Hz
```

Run the SLAM node in terminal 3:

Because I didn’t write /P1_robot0/lidar_2d directly when creating the subscription and used the relative topic name lidar_2d instead, ROS 2 performs name resolution. Therefore, we need to specify the namespace as /P1_robot0. 
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run p03_slam_ros p03_slam_node --ros-args -r __ns:=/P1_robot0
```

Expected callback output:

```text
Received laser scan with 1024 beams.
```

Inspect the resolved node and subscription names:

```bash
ros2 node info /P1_robot0/p03_slam
```

The relative topic `lidar_2d` should resolve to
`/P1_robot0/lidar_2d`. Stop the node and simulation with `Ctrl+C`.

--------------------------------------------------------------------------------------------------------------------------------------------------------------

TF check :
```bash
ros2 run tf2_ros tf2_echo \
  P1_robot0/base_link \
  P1_robot0/lidar2d_link
```

```text
Matrix:
  1.000  0.000  0.000  0.175
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.231
  0.000  0.000  0.000  1.000

  Translation:
x = 0.175 m
y = 0.000 m
z = 0.231 m

Rotation:
yaw = 0.000 rad
```
----------------------------
ros2 topic echo \
  /P1_robot0/lidar_2d \
  --once \
  --field header
------------------------------
ros2 topic echo \
  /P1_robot0/wheel_odom \
  --once \
  --field header
-----------------------------
ros2 topic echo \
  /P1_robot0/wheel_odom \
  --once \
  --field child_frame_id
---------------------------
ros2 topic echo \
  /P1_robot0/wheel_odom \
  --once \
  --field pose.pose


  johnny@JohnnyPC:~/ws/ros2_slam/ros2_masterproject$ ros2 topic echo \
  /P1_robot0/wheel_odom \
  --once \
  --field pose.pose
position:
  x: -1.4454768199028135e-17
  y: -3.705215409675186e-33
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 1.0097514522709812e-16
  w: 1.0