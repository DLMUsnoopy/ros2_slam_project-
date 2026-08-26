# RViz Hand-Drawn, A*, And RRT Planners

This package contains three route planners:

- `p03_rviz_route_planner`: a hand-drawn planner that records points clicked in RViz.
- `astar_global_planner`: an A* planner that calculates a route from the robot pose to an RViz goal.
- `rrt_global_planner`: an RRT planner that samples the map and calculates a feasible route from the robot pose to an RViz goal.

All planners publish to the same route topics by default. Do not run multiple planners at the same time unless the output topics are remapped.

The normal workflow in this repository uses the saved static map published on:

```text
/map
```

The live occupancy grid on `/P1_robot0/occupancy_grid` is still available for mapping experiments, but the examples below use the saved map published by `occu_map_creation`.

## Build

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
colcon build --packages-select occu_map_creation p03_rviz_route_planner
source install/setup.bash
```

## Common Setup

### 1. Start Simulation

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor
```

### 2. Start Static Map

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch occu_map_creation occu.launch.py \
  start_live_mapping:=false \
  publish_saved_map:=true \
  use_sim_time:=true
```

This publishes the saved static map on `/map`. Useful checks:

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field header.frame_id
```

Expected output:

```text
active [3]
map
```

### 3. RViz Map Display

In RViz, set:

```text
Global Options > Fixed Frame: map
```

Add the saved map display:

```text
/map -> Map
```

Use this QoS for `/map` in RViz:

```text
Reliability Policy: Reliable
Durability Policy: Transient Local
```

# Part 1: Hand-Drawn Route Planner

The hand-drawn planner creates a route from points selected manually in RViz. When the route changes, it publishes the full current route.

It uses the saved static map on `/map` for RViz visualization. The planner itself does not calculate obstacle-free paths; it only records the points that you click and republishes them as a route in the `map` frame.

## Start Static Map

Start the simulation first:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor
```

Start the saved static map:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch occu_map_creation occu.launch.py \
  start_live_mapping:=false \
  publish_saved_map:=true \
  use_sim_time:=true
```

Check that `/map` is active:

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field header.frame_id
```

Expected output:

```text
active [3]
map
```

## Start Hand-Drawn Planner

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch p03_rviz_route_planner rviz_route_planner.launch.py
```

The launch defaults are equivalent to:

```bash
ros2 launch p03_rviz_route_planner rviz_route_planner.launch.py \
  clicked_point_topic:=/clicked_point \
  default_frame_id:=map \
  use_sim_time:=true
```

If RViz publishes a clicked point in another frame, the planner transforms that point into `map` with TF before storing it.

## Use In RViz

1. Set `Global Options > Fixed Frame` to `map`.
2. Add `/map -> Map`.
3. Add `/planned_route_marker -> Marker`.
4. Add `/planned_route -> Path`.
5. Select `Publish Point`.
6. Click at least two points on reachable free space.

The route line appears after at least two clicked points.

## Hand-Drawn Planner Topics

### Subscribed Topics


| Topic            | Type                             | Description               |
| ---------------- | -------------------------------- | ------------------------- |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz`Publish Point` input |

### Published Topics


| Topic                   | Type                            | Description         |
| ----------------------- | ------------------------------- | ------------------- |
| `/planned_route`        | `nav_msgs/msg/Path`             | Full route as path  |
| `/planned_route_poses`  | `geometry_msgs/msg/PoseArray`   | Full route as poses |
| `/planned_route_marker` | `visualization_msgs/msg/Marker` | Route line for RViz |

`/planned_route` publishes when the route changes. It does not continuously stream the same old points. If `ros2 topic echo /planned_route` starts after the last click, it may not show anything until another point is added.

## Hand-Drawn Planner Services


| Service             | Type                   | Description              |
| ------------------- | ---------------------- | ------------------------ |
| `/clear_route`      | `std_srvs/srv/Trigger` | Clear all waypoints      |
| `/undo_route_point` | `std_srvs/srv/Trigger` | Remove the last waypoint |

```bash
ros2 service call /undo_route_point std_srvs/srv/Trigger {}
ros2 service call /clear_route std_srvs/srv/Trigger {}
```

## Manual Test

```bash
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: \"map\"}, point: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: \"map\"}, point: {x: 2.0, y: 1.0, z: 0.0}}"

ros2 topic echo /planned_route --once
```

# Part 2: A* Global Planner

The A* planner calculates a route automatically from the current robot pose to a goal selected in RViz.

The default A* setup uses:

```text
/map                    Static occupancy map from nav2_map_server
/P1_robot0/odom_gt      Gazebo ground-truth robot pose
/P1_robot0/goal_pose    RViz 2D Goal Pose input
```

`/P1_robot0/odom_gt` is published with frame `/P1_robot0/odom`, but its x/y coordinates are treated as saved-map coordinates by default through `pose_frame_override:=map`.

## Start A* Planner With Static Map And Ground Truth

Start the simulation first:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor
```

Start the saved static map:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch occu_map_creation occu.launch.py \
  start_live_mapping:=false \
  publish_saved_map:=true \
  use_sim_time:=true
```

Check the map server:

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field header.frame_id
```

Expected output:

```text
active [3]
map
```

Start A*:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch p03_rviz_route_planner astar_global_planner.launch.py
```

The launch defaults are equivalent to:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py \
  map_topic:=/map \
  pose_topic:=/P1_robot0/odom_gt \
  pose_frame_override:=map \
  goal_pose_topic:=/P1_robot0/goal_pose \
  inflation_radius:=0.30 \
  reduce_waypoints:=true
```

## Use In RViz

1. Set `Global Options > Fixed Frame` to `map`.
2. Add `/map -> Map`.
3. Add `/planned_route -> Path`.
4. Add `/planned_route_marker -> Marker`.
5. Select `2D Goal Pose`.
6. Drag a goal on a reachable free cell.

If RViz publishes the goal in `P1_robot0/base_link`, the planner transforms that goal into `map` with TF before planning. The planner log should show:

```text
Transformed goal from frame 'P1_robot0/base_link' to map frame 'map'.
Published A* route from ...:
waypoint_reduction=on, raw_poses=..., published_poses=...
```

If the route is very short, check the raw goal frame and position:

```bash
ros2 topic echo /P1_robot0/goal_pose --once
```

A `P1_robot0/base_link` goal is local to the robot. For a map-frame global goal test, publish one manually:

```bash
ros2 topic pub --once /P1_robot0/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.64, y: 7.71, z: 0.0}, orientation: {w: 1.0}}}"
```

## Compare Raw And Reduced A*

A* normally returns a dense grid path. By default, this package reduces those waypoints with line-of-sight shortcuts before publishing. To compare both outputs with the same start and goal, run:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py reduce_waypoints:=false
```

Then run the same goal again with reduction enabled:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py reduce_waypoints:=true
```

The planner log prints `raw_poses`, `raw_length`, `published_poses`, and `published_length`.

## A* Planner Topics

### Subscribed Topics


| Topic                  | Type                            | Description                                  |
| ---------------------- | ------------------------------- | -------------------------------------------- |
| `/map`                 | `nav_msgs/msg/OccupancyGrid`    | Static saved occupancy map input             |
| `/P1_robot0/odom_gt`   | `nav_msgs/msg/Odometry`         | Gazebo ground-truth robot pose used as start |
| `/P1_robot0/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz`2D Goal Pose` input used as goal        |

### Published Topics


| Topic                   | Type                            | Description                |
| ----------------------- | ------------------------------- | -------------------------- |
| `/planned_route`        | `nav_msgs/msg/Path`             | Calculated A* route        |
| `/planned_route_poses`  | `geometry_msgs/msg/PoseArray`   | Calculated A* route poses  |
| `/planned_route_marker` | `visualization_msgs/msg/Marker` | Calculated A* route marker |

## A* Planner Parameters


| Parameter             | Default                | Description                                              |
| --------------------- | ---------------------- | -------------------------------------------------------- |
| `map_topic`           | `/map`                 | Occupancy grid used by A*                                |
| `pose_topic`          | `/P1_robot0/odom_gt`   | Robot start pose topic                                   |
| `pose_frame_override` | `map`                  | Frame used to interpret`pose_topic` coordinates          |
| `goal_pose_topic`     | `/P1_robot0/goal_pose` | RViz goal topic                                          |
| `inflation_radius`    | `0.30`                 | Obstacle inflation radius in meters                      |
| `allow_unknown`       | `false`                | Whether A* may plan through unknown occupancy-grid cells |
| `reduce_waypoints`    | `true`                 | Reduce dense A* waypoints with line-of-sight shortcuts   |
| `use_sim_time`        | `true`                 | Use simulation clock                                     |

# Part 3: RRT Global Planner

The RRT planner calculates a collision-free route automatically from the current robot pose to a goal selected in RViz. It uses the same saved static map, robot pose, goal topic, and output topics as the A* planner.

The default RRT setup uses:

```text
/map                    Static occupancy map from nav2_map_server
/P1_robot0/odom_gt      Gazebo ground-truth robot pose
/P1_robot0/goal_pose    RViz 2D Goal Pose input
```

`/P1_robot0/odom_gt` is published with frame `/P1_robot0/odom`, but its x/y coordinates are treated as saved-map coordinates by default through `pose_frame_override:=map`.

## Start RRT Planner With Static Map And Ground Truth

Start the simulation first:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor
```

Start the saved static map:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch occu_map_creation occu.launch.py \
  start_live_mapping:=false \
  publish_saved_map:=true \
  use_sim_time:=true
```

Check the map server:

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --field header.frame_id
```

Expected output:

```text
active [3]
map
```

Start RRT:

```bash
cd ~/ws/p03-find-track-and-follow-your-target-new
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch p03_rviz_route_planner rrt_global_planner.launch.py
```

The launch defaults are equivalent to:

```bash
ros2 launch p03_rviz_route_planner rrt_global_planner.launch.py \
  map_topic:=/map \
  pose_topic:=/P1_robot0/odom_gt \
  pose_frame_override:=map \
  goal_pose_topic:=/P1_robot0/goal_pose \
  inflation_radius:=0.30 \
  max_iterations:=3000 \
  step_size:=0.50 \
  goal_sample_rate:=0.10 \
  smooth_path:=true \
  smoothing_iterations:=100
```

## Use RRT In RViz

1. Set `Global Options > Fixed Frame` to `map`.
2. Add `/map -> Map`.
3. Add `/planned_route -> Path`.
4. Add `/planned_route_marker -> Marker`.
5. Select `2D Goal Pose`.
6. Drag a goal on a reachable free cell.

When RRT succeeds, the planner log should show:

```text
Published RRT route from ...:
smoothing=on, raw_poses=..., published_poses=...
```

If planning fails, try increasing the sample count or reducing the obstacle inflation:

```bash
ros2 launch p03_rviz_route_planner rrt_global_planner.launch.py \
  max_iterations:=8000 \
  inflation_radius:=0.30
```

For debugging only, use a smaller radius:

```bash
ros2 launch p03_rviz_route_planner rrt_global_planner.launch.py inflation_radius:=0.02
```

To compare raw RRT with the optimized smoothed output, run the same start and goal twice:

```bash
ros2 launch p03_rviz_route_planner rrt_global_planner.launch.py smooth_path:=false
```

```bash
ros2 launch p03_rviz_route_planner rrt_global_planner.launch.py \
  smooth_path:=true \
  smoothing_iterations:=100
```

The planner log prints both the raw path length and the published path length, so the improvement can be reported with waypoint count and route length.

## RRT Planner Topics

### Subscribed Topics


| Topic                  | Type                            | Description                                  |
| ---------------------- | ------------------------------- | -------------------------------------------- |
| `/map`                 | `nav_msgs/msg/OccupancyGrid`    | Static saved occupancy map input             |
| `/P1_robot0/odom_gt`   | `nav_msgs/msg/Odometry`         | Gazebo ground-truth robot pose used as start |
| `/P1_robot0/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz`2D Goal Pose` input used as goal        |

### Published Topics


| Topic                   | Type                            | Description                 |
| ----------------------- | ------------------------------- | --------------------------- |
| `/planned_route`        | `nav_msgs/msg/Path`             | Calculated RRT route        |
| `/planned_route_poses`  | `geometry_msgs/msg/PoseArray`   | Calculated RRT route poses  |
| `/planned_route_marker` | `visualization_msgs/msg/Marker` | Calculated RRT route marker |

## RRT Planner Parameters


| Parameter              | Default                | Description                                     |
| ---------------------- | ---------------------- | ----------------------------------------------- |
| `map_topic`            | `/map`                 | Occupancy grid used by RRT                      |
| `pose_topic`           | `/P1_robot0/odom_gt`   | Robot start pose topic                          |
| `pose_frame_override`  | `map`                  | Frame used to interpret`pose_topic` coordinates |
| `goal_pose_topic`      | `/P1_robot0/goal_pose` | RViz goal topic                                 |
| `inflation_radius`     | `0.30`                 | Obstacle inflation radius in meters             |
| `max_iterations`       | `3000`                 | Maximum number of RRT samples                   |
| `step_size`            | `0.50`                 | Maximum tree extension distance in meters       |
| `goal_sample_rate`     | `0.10`                 | Probability of sampling the goal directly       |
| `smooth_path`          | `true`                 | Enable shortcut smoothing before publishing     |
| `smoothing_iterations` | `100`                  | Number of shortcut attempts for path smoothing  |
| `use_sim_time`         | `true`                 | Use simulation clock                            |

## Obstacle Inflation And Footprint

The planner uses `inflation_radius` as a circular safety radius around occupied cells.

For the Jackal model, the approximate footprint including wheels is:

```text
chassis length ~= 0.42 m
width including wheels ~= 0.42 m
circular footprint radius ~= 0.30 m
```

The default planner setting is `0.30 m`, which keeps the robot away from obstacles while still leaving most indoor doors and corridors connected. A more conservative Jackal setting is:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py inflation_radius:=0.35
```

If the map is too narrow or planning fails because passages close after inflation, try:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py inflation_radius:=0.25
```

For debugging only, use a smaller radius and allow unknown cells:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py \
  inflation_radius:=0.02 \
  allow_unknown:=true
```

## Live Map Option

To use the live occupancy grid instead of the saved map:

```bash
ros2 launch p03_rviz_route_planner astar_global_planner.launch.py \
  map_topic:=/P1_robot0/occupancy_grid \
  pose_topic:=/P1_robot0/esmt_pose \
  pose_frame_override:=
```

The live-map option uses the localization estimate `/P1_robot0/esmt_pose`, which is already published in the `map` frame.

## Frame Notes

Use `map` as the RViz fixed frame and planner frame.

Expected robot TF chain:

```text
map -> P1_robot0/odom -> P1_robot0/base_link -> robot links
```

If `RobotModel` is red but `Map`, `Marker`, and `Path` work, the planner can still be tested. That warning usually means a robot model transform is missing.

## Expected Result

For the hand-drawn planner:

- RViz shows the saved static map on `/map`.
- RViz shows a route line after at least two clicked points.
- `/planned_route` publishes the full route when the route changes.
- Undo and clear services work.

For the A* planner:

- RViz shows the saved static map on `/map`.
- RViz shows a route after selecting a reachable `2D Goal Pose`.
- `/planned_route` publishes the calculated path.
- `/planned_route_marker` publishes the route marker.

For the RRT planner:

- RViz shows the saved static map on `/map`.
- RViz shows a route after selecting a reachable `2D Goal Pose`.
- `/planned_route` publishes the sampled feasible path.
- `/planned_route_marker` publishes the route marker.
