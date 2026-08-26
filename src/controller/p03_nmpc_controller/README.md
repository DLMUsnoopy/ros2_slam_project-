# controller

A ROS 2 waypoint-following controller for a differential-drive robot. It subscribes to odometry and a planned route, then drives the robot through each waypoint in sequence using a decoupled PID control strategy.

---

## Quick Start

**Launch (recommended)**
```bash
ros2 launch controller launch.py
```

**Run the node directly**
```bash
ros2 run controller pid_controller
```

---

## Topics

| Direction | Topic | Message Type | Description |
|-----------|-------|--------------|-------------|
| Subscribed | `/P1_robot0/odom_gt` | `nav_msgs/msg/Odometry` | Robot pose and orientation |
| Subscribed | `/planned_route_poses` | `geometry_msgs/msg/PoseArray` | Ordered list of waypoints to follow |
| Published  | `/P1_robot0/cmd_vel`   | `geometry_msgs/msg/TwistStamped` | Linear and angular velocity commands |

---

## Input / Output Data

### Input — Odometry (`nav_msgs/msg/Odometry`)
The node reads:
- `pose.pose.position.x / .y` — robot's 2D position in the world frame
- `pose.pose.orientation` (quaternion) — converted internally to a yaw angle (heading) using `tf2::Matrix3x3::getRPY`

### Input — Planned Route (`geometry_msgs/msg/PoseArray`)
An ordered array of poses. Only the `position.x` and `position.y` of each pose are used; orientation within the array is ignored. The node walks through these waypoints sequentially.

### Output — Velocity Command (`geometry_msgs/msg/TwistStamped`)
- `twist.linear.x` — forward speed (m/s)
- `twist.angular.z` — yaw rate (rad/s)

At any given moment **only one of these is non-zero** (see control logic below).

---

## Control Logic

### Waypoint Switching
The node maintains an index `w_idx` into the received `PoseArray`. After each control tick it computes the Euclidean distance to the current target waypoint. When that distance drops below **2 cm**, the index is incremented and the robot begins tracking the next waypoint. The robot stays on the final waypoint once the end of the list is reached.

### Decoupled Angular / Linear PID

Rather than sending simultaneous angular and linear commands, the controller separates rotation and forward motion into two mutually exclusive modes. This avoids the lateral drift that occurs when a robot turns and drives at the same time — drift that would complicate any upstream planner (e.g. MPC).

**Step 1 — Rotate to face the waypoint (angular mode)**

The controller computes the ideal heading:
```
ideal_yaw = atan2(dy, dx)
d_yaw     = ideal_yaw − current_yaw        # normalised to (−π, π]
```

If the heading error exceeds **≈ 4.6°** (`|d_yaw| > 0.08 rad`) *and* the robot is not already at the waypoint, a PID loop drives `angular_z` to correct it. `linear_x` is zero during this phase.

**Step 2 — Drive forward (linear mode)**

Once the heading error is within that same threshold, the angular output is suppressed and a second PID loop drives `linear_x` proportional to the remaining distance `d_dist`. The robot moves in a straight line toward the waypoint. `angular_z` is zero during this phase.

This alternation continues — correct heading, then drive, then correct heading again if the path curves — until the waypoint is reached.

### PID Parameters

| Controller | Kp  | Ki  | Kd  |
|------------|-----|-----|-----|
| Linear     | 0.5 | 0.0 | 0.1 |
| Angular    | 0.5 | 0.0 | 0.1 |

Both integrals and previous-error terms are `static` variables inside the PID function, so they persist across the 50 Hz timer ticks. **Note:** gains are marked for tuning once the full system stack is running.

### Control Rate
The timer fires at **50 Hz** (every 20 ms). `dt = 0.05 s` is used as the fixed time step for integral accumulation and derivative calculation.

---

## Logging

On startup the node creates `/tmp/pid_log.csv` and appends one row per control tick:

```
time,d_dist
1700000000.123,0.341
...
```

This can be used to plot convergence behaviour and tune the PID gains.

---

## Dependencies

- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `tf2`