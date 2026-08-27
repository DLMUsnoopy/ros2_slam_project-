# p03_slam_evaluation

Compares `/P1_robot0/slam_pose` with the simulator ground truth
`/P1_robot0/odom_gt`.

The node aligns both trajectories at the first synchronized sample. Ground
truth is interpolated at every SLAM timestamp before the error is calculated.
The terminal reports current, RMSE, and maximum translation and yaw errors.
The launch file also opens live line graphs for these metrics with `rqt_plot`.

## Run

Start the simulator and SLAM first. Then run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch p03_slam_evaluation evaluation.launch.py
```

Two graph windows open by default:

- translation error in metres: current, RMSE, and maximum;
- yaw error in degrees: current, RMSE, and maximum.

To run without graph windows:

```bash
ros2 launch p03_slam_evaluation evaluation.launch.py plot_errors:=false
```

The graph data is also available on these topic trees:

```text
/P1_robot0/evaluation/errors/translation/{current,rmse,max}
/P1_robot0/evaluation/errors/yaw/{current_deg,rmse_deg,max_deg}
```

Example output:

```text
Evaluation: samples=100 | translation current=0.031 m, RMSE=0.024 m, max=0.061 m | yaw current=0.42 deg, RMSE=0.38 deg, max=1.12 deg.
```

## RViz

Set the fixed frame to `map` and add two `Path` displays:

- `/P1_robot0/evaluation/estimated_path`
- `/P1_robot0/evaluation/ground_truth_path`

The paths should remain close after the robot moves and rotates. Ground truth
is used only by this evaluation node and is never sent to the SLAM pipeline.
