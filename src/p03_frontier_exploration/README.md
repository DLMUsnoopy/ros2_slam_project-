# Frontier Exploration

This package selects automatic exploration goals from a live occupancy grid and
publishes them to the existing A* planner goal topic.

## Idea

- Subscribe to `/map` as a real-time `nav_msgs/msg/OccupancyGrid`.
- Treat a frontier as a free cell that touches at least one unknown cell.
- Cluster neighboring frontier cells.
- Score each cluster with travel distance and information gain, measured as nearby unknown cells.
- Pick a representative cell from the best-scoring cluster.
- Publish that target as `geometry_msgs/msg/PoseStamped` on `/P1_robot0/goal_pose`.
- Let `p03_rviz_route_planner/astar_global_planner` compute the actual path.

The default cluster score is `distance_weight * distance - information_gain_weight * information_gain`.
The goal is intentionally placed on a known free cell at the edge of unknown
space, so A* can keep `allow_unknown:=false`.

## Run

Build the new package:

```bash
colcon build --packages-select p03_frontier_exploration
source install/setup.bash
```

Run only the frontier goal selector:

```bash
ros2 launch p03_frontier_exploration frontier_explorer.launch.py
```

Run frontier selection together with the existing A* planner:

```bash
ros2 launch p03_frontier_exploration frontier_astar_exploration.launch.py
```

Useful RViz topics:

- `/frontier_clusters`
- `/frontier_goal_marker`
- `/planned_route`
- `/planned_route_marker`
