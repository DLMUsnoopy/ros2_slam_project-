"""Select exploration goals from frontiers in a live occupancy grid.

The node subscribes to a nav_msgs/OccupancyGrid and robot odometry. It finds
free cells next to unknown cells, clusters them into frontiers, chooses a target
cell on the best cluster, and publishes that cell as a PoseStamped goal. The
published goal can feed the existing A* planner through its goal_pose_topic.
"""

from collections import deque
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


CARDINAL_NEIGHBORS = ((1, 0), (-1, 0), (0, 1), (0, -1))
EIGHT_NEIGHBORS = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
)


def parameter_as_bool(value):
    """Parse ROS parameter values that may arrive as bools or strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def quaternion_to_yaw(quaternion):
    """Extract the planar yaw angle from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_grid(x, y, map_info):
    """Convert a map-frame point in meters into an occupancy-grid cell."""
    origin = map_info.origin.position
    yaw = quaternion_to_yaw(map_info.origin.orientation)
    dx = x - origin.x
    dy = y - origin.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return (
        math.floor(local_x / map_info.resolution),
        math.floor(local_y / map_info.resolution),
    )


def grid_to_world(cell, map_info):
    """Convert an occupancy-grid cell into the map-frame center point."""
    grid_x, grid_y = cell
    local_x = (grid_x + 0.5) * map_info.resolution
    local_y = (grid_y + 0.5) * map_info.resolution
    origin = map_info.origin.position
    yaw = quaternion_to_yaw(map_info.origin.orientation)
    return (
        origin.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        origin.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def in_bounds(cell, width, height):
    """Return whether a cell lies inside the map."""
    return 0 <= cell[0] < width and 0 <= cell[1] < height


def cell_index(cell, width):
    """Return the row-major OccupancyGrid index for a cell."""
    return cell[1] * width + cell[0]


class FrontierExplorer(Node):
    """Publish automatic A* goals from occupancy-grid frontier clusters."""

    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/P1_robot0/odom_gt")
        self.declare_parameter("pose_frame_override", "map")
        self.declare_parameter("goal_pose_topic", "/P1_robot0/goal_pose")
        self.declare_parameter("frontier_marker_topic", "frontier_clusters")
        self.declare_parameter("goal_marker_topic", "frontier_goal_marker")
        self.declare_parameter("update_rate", 1.0)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("robot_clearance_radius", 0.25)
        self.declare_parameter("start_search_radius", 0.50)
        self.declare_parameter("goal_reached_distance", 0.45)
        self.declare_parameter("goal_change_distance", 1.50)
        self.declare_parameter("min_goal_hold_time", 20.0)
        self.declare_parameter("goal_switch_score_margin", 25.0)
        self.declare_parameter("goal_valid_frontier_radius", 0.60)
        self.declare_parameter("republish_goal_period", 2.0)
        self.declare_parameter("selection_strategy", "information_gain")
        self.declare_parameter("distance_weight", 1.0)
        self.declare_parameter("frontier_size_weight", 0.08)
        self.declare_parameter("information_gain_radius", 1.0)
        self.declare_parameter("information_gain_weight", 0.05)
        self.declare_parameter("visualize", True)

        gp = lambda name: self.get_parameter(name).value
        self.map_topic = gp("map_topic")
        self.pose_topic = gp("pose_topic")
        self.pose_frame_override = str(gp("pose_frame_override")).strip()
        self.goal_pose_topic = gp("goal_pose_topic")
        self.frontier_marker_topic = gp("frontier_marker_topic")
        self.goal_marker_topic = gp("goal_marker_topic")
        self.update_rate = max(0.1, float(gp("update_rate")))
        self.occupied_threshold = int(gp("occupied_threshold"))
        self.min_frontier_size = max(1, int(gp("min_frontier_size")))
        self.robot_clearance_radius = max(0.0, float(gp("robot_clearance_radius")))
        self.start_search_radius = max(0.0, float(gp("start_search_radius")))
        self.goal_reached_distance = max(0.0, float(gp("goal_reached_distance")))
        self.goal_change_distance = max(0.0, float(gp("goal_change_distance")))
        self.min_goal_hold_time = max(0.0, float(gp("min_goal_hold_time")))
        self.goal_switch_score_margin = max(0.0, float(gp("goal_switch_score_margin")))
        self.goal_valid_frontier_radius = max(0.0, float(gp("goal_valid_frontier_radius")))
        self.republish_goal_period = max(0.0, float(gp("republish_goal_period")))
        self.selection_strategy = str(gp("selection_strategy")).strip().lower()
        self.distance_weight = float(gp("distance_weight"))
        self.frontier_size_weight = float(gp("frontier_size_weight"))
        self.information_gain_radius = max(0.0, float(gp("information_gain_radius")))
        self.information_gain_weight = float(gp("information_gain_weight"))
        self.visualize = parameter_as_bool(gp("visualize"))

        self.map_msg = None
        self.robot_pose = None
        self.current_goal_cell = None
        self.current_goal_world = None
        self.current_goal_score = math.inf
        self.current_goal_selected_time = 0.0
        self.last_goal_publish_time = 0.0
        self.last_selection_log_time = 0.0

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(Odometry, self.pose_topic, self.on_pose, 20)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)
        self.frontier_marker_pub = self.create_publisher(Marker, self.frontier_marker_topic, 10)
        self.goal_marker_pub = self.create_publisher(Marker, self.goal_marker_topic, 10)
        self.create_timer(1.0 / self.update_rate, self.on_timer)

        pose_frame_note = (
            f" interpreted as '{self.pose_frame_override}'"
            if self.pose_frame_override
            else ""
        )
        self.get_logger().info(
            f"Ready. Watching map '{self.map_topic}', pose '{self.pose_topic}'"
            f"{pose_frame_note}, publishing frontier goals to '{self.goal_pose_topic}'."
        )

    def on_map(self, msg):
        """Cache the latest occupancy grid."""
        self.map_msg = msg

    def on_pose(self, msg):
        """Cache the latest robot pose."""
        self.robot_pose = msg

    def on_timer(self):
        """Periodically select and publish an exploration goal."""
        if self.map_msg is None or self.robot_pose is None:
            return

        map_frame = self.map_msg.header.frame_id or "map"
        raw_pose_frame = self.robot_pose.header.frame_id
        pose_frame = self.pose_frame_override or raw_pose_frame
        if pose_frame and pose_frame != map_frame:
            self.get_logger().warn(
                f"Robot pose frame '{pose_frame}' does not match map frame '{map_frame}'.",
                throttle_duration_sec=5.0,
            )
            return

        info = self.map_msg.info
        robot_position = self.robot_pose.pose.pose.position
        robot_cell = world_to_grid(robot_position.x, robot_position.y, info)
        if not in_bounds(robot_cell, info.width, info.height):
            self.get_logger().warn("Robot pose is outside the occupancy grid.", throttle_duration_sec=5.0)
            return

        clusters = self.find_frontier_clusters(self.map_msg, robot_cell)
        if self.visualize:
            self.publish_frontier_markers(clusters, map_frame)

        if not clusters:
            self.clear_goal_marker(map_frame)
            self.get_logger().info("No frontier clusters found.", throttle_duration_sec=5.0)
            return

        goal_cell, cluster, information_gain, candidate_score = self.choose_goal_cell(
            clusters, robot_cell, self.map_msg
        )
        goal_world = grid_to_world(goal_cell, info)
        distance_to_goal = math.hypot(robot_position.x - goal_world[0], robot_position.y - goal_world[1])

        now = time.monotonic()
        should_pick_new_goal = self.current_goal_cell is None
        switch_reason = "initial"
        if self.current_goal_cell is not None:
            current_goal_distance = math.hypot(
                robot_position.x - self.current_goal_world[0],
                robot_position.y - self.current_goal_world[1],
            )
            current_goal_is_frontier = self.goal_is_still_frontier(
                self.current_goal_cell, clusters, info.resolution
            )
            held_time = now - self.current_goal_selected_time
            candidate_shift = math.hypot(
                goal_world[0] - self.current_goal_world[0],
                goal_world[1] - self.current_goal_world[1],
            )
            better_score = candidate_score + self.goal_switch_score_margin < self.current_goal_score

            if current_goal_distance <= self.goal_reached_distance:
                should_pick_new_goal = True
                switch_reason = "reached"
            elif not current_goal_is_frontier:
                should_pick_new_goal = True
                switch_reason = "frontier_finished"
            elif (
                held_time >= self.min_goal_hold_time
                and candidate_shift >= self.goal_change_distance
                and better_score
            ):
                should_pick_new_goal = True
                switch_reason = "better_information_gain"
            else:
                should_pick_new_goal = False

        if should_pick_new_goal:
            self.current_goal_cell = goal_cell
            self.current_goal_world = goal_world
            self.current_goal_score = candidate_score
            self.current_goal_selected_time = now
            self.publish_goal(map_frame)
            if now - self.last_selection_log_time > 1.0:
                self.last_selection_log_time = now
                self.get_logger().info(
                    f"Selected frontier goal ({goal_world[0]:.2f}, {goal_world[1]:.2f}) "
                    f"reason={switch_reason}, cluster_size={len(cluster)}, "
                    f"information_gain={information_gain}, score={candidate_score:.2f}, "
                    f"robot_distance={distance_to_goal:.2f} m."
                )
            return

        if self.republish_goal_period > 0.0:
            if time.monotonic() - self.last_goal_publish_time >= self.republish_goal_period:
                self.publish_goal(map_frame)

    def find_frontier_clusters(self, map_msg, robot_cell):
        """Return connected components of reachable frontier cells."""
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        clearance_cells = math.ceil(self.robot_clearance_radius / map_msg.info.resolution)
        inflated_obstacles = self.make_inflated_obstacles(data, width, height, clearance_cells)
        start_cell = self.find_nearest_free_cell(
            robot_cell, data, width, height, inflated_obstacles, map_msg.info.resolution
        )
        if start_cell is None:
            self.get_logger().warn(
                "No free map cell found near the robot for frontier selection.",
                throttle_duration_sec=5.0,
            )
            return []
        if start_cell != robot_cell:
            self.get_logger().warn(
                f"Robot cell {robot_cell} is not free; using nearby free cell {start_cell}.",
                throttle_duration_sec=5.0,
            )
        
        # Using BFS to find all reachable free cells from the start_cell, avoiding inflated obstacles and occupied cells.
        reachable_cells = self.find_reachable_free_cells(
            start_cell, data, width, height, inflated_obstacles
        )

        if not reachable_cells:
            self.get_logger().warn(
                "No reachable free-space component found for frontier selection.",
                throttle_duration_sec=5.0,
            )
            return []

        frontier_cells = set()
        for cell in reachable_cells:
            if self.has_unknown_neighbor(cell, data, width, height):
                frontier_cells.add(cell)

        clusters = []
        visited = set()
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = self.grow_cluster(cell, frontier_cells, visited)
            if len(cluster) < self.min_frontier_size:
                continue
            clusters.append(cluster)

        clusters.sort(key=lambda cluster: self.cluster_distance(cluster, robot_cell))
        return clusters

    def find_nearest_free_cell(self, center, data, width, height, inflated_obstacles, resolution):
        """Find a nearby free cell when the robot pose falls on unknown/occupied map data."""
        max_radius_cells = max(0, math.ceil(self.start_search_radius / resolution))
        best_cell = None
        best_distance_sq = math.inf
        for radius in range(max_radius_cells + 1):
            found_at_radius = False
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (center[0] + dx, center[1] + dy)
                    if not in_bounds(candidate, width, height):
                        continue
                    if candidate in inflated_obstacles:
                        continue
                    if not self.is_free_value(data[cell_index(candidate, width)]):
                        continue
                    distance_sq = dx * dx + dy * dy
                    if distance_sq < best_distance_sq:
                        best_cell = candidate
                        best_distance_sq = distance_sq
                        found_at_radius = True
            if found_at_radius:
                return best_cell
        return None

    def find_reachable_free_cells(self, start, data, width, height, inflated_obstacles):
        """Flood-fill free cells that are connected to the robot's current cell."""
        if (
            not in_bounds(start, width, height)
            or not self.is_free_value(data[cell_index(start, width)])
            or start in inflated_obstacles
        ):
            return set()

        reachable = {start}
        queue = deque([start])
        while queue:
            cell = queue.popleft()
            for dx, dy in CARDINAL_NEIGHBORS:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor in reachable or not in_bounds(neighbor, width, height):
                    continue
                if neighbor in inflated_obstacles:
                    continue
                if not self.is_free_value(data[cell_index(neighbor, width)]):
                    continue
                reachable.add(neighbor)
                queue.append(neighbor)
        return reachable

    def grow_cluster(self, start, frontier_cells, visited):
        """Flood-fill one frontier connected component."""
        cluster = []
        queue = deque([start])
        visited.add(start)
        while queue:
            cell = queue.popleft()
            cluster.append(cell)
            for dx, dy in EIGHT_NEIGHBORS:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor in frontier_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        return cluster

    def choose_goal_cell(self, clusters, robot_cell, map_msg):
        """Choose a representative cell from the best frontier cluster."""
        best_cluster = None
        best_information_gain = 0
        best_score = math.inf
        for cluster in clusters:
            distance = self.cluster_distance(cluster, robot_cell)
            if self.selection_strategy == "nearest":
                score = distance
                information_gain = self.cluster_information_gain(cluster, map_msg)
            elif self.selection_strategy == "cost":
                information_gain = len(cluster)
                score = self.distance_weight * distance - self.frontier_size_weight * information_gain
            else:
                information_gain = self.cluster_information_gain(cluster, map_msg)
                score = self.distance_weight * distance - self.information_gain_weight * information_gain
            if score < best_score:
                best_score = score
                best_cluster = cluster
                best_information_gain = information_gain

        centroid_x = sum(cell[0] for cell in best_cluster) / len(best_cluster)
        centroid_y = sum(cell[1] for cell in best_cluster) / len(best_cluster)
        goal_cell = min(
            best_cluster,
            key=lambda cell: (cell[0] - centroid_x) ** 2 + (cell[1] - centroid_y) ** 2,
        )
        return goal_cell, best_cluster, best_information_gain, best_score

    def goal_is_still_frontier(self, goal_cell, clusters, resolution):
        """Return whether the current goal still lies near an active frontier."""
        max_distance_cells = max(1, math.ceil(self.goal_valid_frontier_radius / resolution))
        max_distance_sq = max_distance_cells * max_distance_cells
        for cluster in clusters:
            for cell in cluster:
                dx = cell[0] - goal_cell[0]
                dy = cell[1] - goal_cell[1]
                if dx * dx + dy * dy <= max_distance_sq:
                    return True
        return False

    def cluster_distance(self, cluster, robot_cell):
        """Return the nearest-cell distance from the robot to a cluster."""
        return min(math.hypot(cell[0] - robot_cell[0], cell[1] - robot_cell[1]) for cell in cluster)

    def cluster_information_gain(self, cluster, map_msg):
        """Count unknown cells visible around a frontier cluster."""
        info = map_msg.info
        radius_cells = math.ceil(self.information_gain_radius / info.resolution)
        unknown_cells = set()

        offsets = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    offsets.append((dx, dy))

        for cell in cluster:
            for dx, dy in offsets:
                candidate = (cell[0] + dx, cell[1] + dy)
                if not in_bounds(candidate, info.width, info.height):
                    continue
                if map_msg.data[cell_index(candidate, info.width)] < 0:
                    unknown_cells.add(candidate)

        return len(unknown_cells)

    def is_free_value(self, value):
        """Return True for cells A* can use as target cells."""
        return 0 <= value < self.occupied_threshold

    def has_unknown_neighbor(self, cell, data, width, height):
        """Return True if a free cell touches unknown space."""
        for dx, dy in CARDINAL_NEIGHBORS:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if in_bounds(neighbor, width, height) and data[cell_index(neighbor, width)] < 0:
                return True
        return False

    def make_inflated_obstacles(self, data, width, height, radius):
        """Return occupied cells expanded by the configured robot clearance."""
        obstacles = []
        for index, value in enumerate(data):
            if value >= self.occupied_threshold:
                obstacles.append((index % width, index // width))

        if radius <= 0:
            return set(obstacles)

        inflated = set(obstacles)
        offsets = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    offsets.append((dx, dy))

        for x, y in obstacles:
            for dx, dy in offsets:
                candidate = (x + dx, y + dy)
                if in_bounds(candidate, width, height):
                    inflated.add(candidate)
        return inflated

    def publish_goal(self, map_frame):
        """Publish the current frontier as a PoseStamped goal for A*."""
        if self.current_goal_world is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_frame
        msg.pose.position.x = self.current_goal_world[0]
        msg.pose.position.y = self.current_goal_world[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.last_goal_publish_time = time.monotonic()
        if self.visualize:
            self.publish_goal_marker(map_frame)

    def publish_frontier_markers(self, clusters, map_frame):
        """Publish frontier cells as RViz points."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = map_frame
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD if clusters else Marker.DELETE
        marker.scale.x = max(0.04, self.map_msg.info.resolution * 1.4)
        marker.scale.y = marker.scale.x
        marker.color.r = 0.1
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.points = []
        if clusters:
            for cluster in clusters:
                for cell in cluster:
                    x, y = grid_to_world(cell, self.map_msg.info)
                    marker.points.append(Point(x=x, y=y, z=0.08))
        self.frontier_marker_pub.publish(marker)

    def publish_goal_marker(self, map_frame):
        """Publish the selected frontier goal with high-contrast RViz markers."""
        stamp = self.get_clock().now().to_msg()
        x, y = self.current_goal_world

        sphere = Marker()
        sphere.header.stamp = stamp
        sphere.header.frame_id = map_frame
        sphere.ns = "frontier_goal"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.20
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.45
        sphere.scale.y = 0.45
        sphere.scale.z = 0.22
        sphere.color.r = 1.0
        sphere.color.g = 0.08
        sphere.color.b = 0.02
        sphere.color.a = 1.0
        self.goal_marker_pub.publish(sphere)

        pillar = Marker()
        pillar.header.stamp = stamp
        pillar.header.frame_id = map_frame
        pillar.ns = "frontier_goal"
        pillar.id = 1
        pillar.type = Marker.CYLINDER
        pillar.action = Marker.ADD
        pillar.pose.position.x = x
        pillar.pose.position.y = y
        pillar.pose.position.z = 0.60
        pillar.pose.orientation.w = 1.0
        pillar.scale.x = 0.10
        pillar.scale.y = 0.10
        pillar.scale.z = 1.00
        pillar.color.r = 1.0
        pillar.color.g = 0.05
        pillar.color.b = 0.02
        pillar.color.a = 0.85
        self.goal_marker_pub.publish(pillar)

        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = map_frame
        label.ns = "frontier_goal"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = 1.25
        label.pose.orientation.w = 1.0
        label.scale.z = 0.34
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = "FRONTIER GOAL"
        self.goal_marker_pub.publish(label)

    def clear_goal_marker(self, map_frame):
        """Delete the selected-goal marker when exploration is complete."""
        for marker_id in (0, 1, 2):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = map_frame
            marker.ns = "frontier_goal"
            marker.id = marker_id
            marker.action = Marker.DELETE
            self.goal_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
