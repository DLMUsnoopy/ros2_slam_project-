"""Calculate and publish a global route with the A* search algorithm.

The planner consumes a nav_msgs/OccupancyGrid, the robot pose in the map frame,
and a goal selected with RViz's "2D Goal Pose" tool. It converts world
coordinates to grid cells, expands occupied cells by a configurable safety
radius, searches the grid with A*, and publishes the resulting route for RViz
and downstream waypoint-following nodes.

This is a standalone global planner, not a Nav2 planner plugin. It calculates a
complete map-level route but does not command robot velocity or perform local
dynamic-obstacle avoidance.
"""

import heapq
import itertools
import math
import time

import numpy as np
import rclpy
from scipy.ndimage import distance_transform_edt
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


# A* works on grid cells, not directly on metric coordinates.
# Moving horizontally or vertically changes one grid coordinate by one cell, so
# its cost is 1. Moving diagonally changes x and y at the same time, so the
# Euclidean distance is sqrt(2). Keeping these costs different makes the path
# length closer to the real geometric distance on the map.
CARDINAL_COST = 1.0
DIAGONAL_COST = math.sqrt(2.0)


def parameter_as_bool(value):
    """Parse ROS parameter values that may arrive as bools or strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def quaternion_to_yaw(quaternion):
    """Extract the planar yaw angle from a geometry_msgs Quaternion."""
    # ROS poses store orientation as a quaternion. For a 2D occupancy grid we
    # only care about yaw, meaning rotation around the z axis. Roll and pitch
    # are ignored because the planner assumes a flat ground plane.
    # Occupancy-grid origins are usually axis-aligned, but ROS allows the map
    # origin to contain a rotation. Supporting yaw here keeps the coordinate
    # conversion correct for both the common and the rotated-map cases.
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)



def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a 3D vector by a geometry_msgs Quaternion."""
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w
    vx, vy, vz = vector

    # v' = v + 2w(q x v) + 2(q x (q x v))
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx

    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


def transform_point(point, transform):
    """Apply a geometry_msgs TransformStamped to a point."""
    rotated = rotate_vector_by_quaternion(
        (point.x, point.y, point.z), transform.transform.rotation
    )
    translation = transform.transform.translation
    return Point(
        x=rotated[0] + translation.x,
        y=rotated[1] + translation.y,
        z=rotated[2] + translation.z,
    )

def world_to_grid(x, y, map_info):
    """Convert a point in map/world coordinates into an integer grid cell."""
    # RViz goals and robot odometry are in metric map coordinates, for example
    # x=1.2 meters and y=-0.7 meters. A* needs integer cells, for example
    # (224, 186). This function performs that conversion using the map origin,
    # map resolution, and optional map rotation.
    origin = map_info.origin.position
    yaw = quaternion_to_yaw(map_info.origin.orientation)
    dx = x - origin.x
    dy = y - origin.y

    # Translate into the map origin and rotate by -yaw. The resulting local
    # coordinates align with the OccupancyGrid x/y axes.
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

    # floor() intentionally selects the cell containing the world point. A
    # point below the map origin becomes negative and is rejected later by the
    # bounds check instead of being incorrectly truncated to cell zero.
    return (
        math.floor(local_x / map_info.resolution),
        math.floor(local_y / map_info.resolution),
    )


def grid_to_world(cell, map_info):
    """Convert a grid cell into the world coordinates of its center point."""
    # A* returns cells such as (10, 15), but ROS Path messages should contain
    # metric positions. This is the reverse of world_to_grid(). The returned
    # point is placed at the center of the cell so the route does not hug cell
    # boundaries unnecessarily.
    grid_x, grid_y = cell

    # Publish waypoints at cell centers, not cell corners. This gives the robot
    # the maximum available clearance from neighboring obstacle cells.
    local_x = (grid_x + 0.5) * map_info.resolution
    local_y = (grid_y + 0.5) * map_info.resolution
    origin = map_info.origin.position
    yaw = quaternion_to_yaw(map_info.origin.orientation)

    # Rotate by +yaw and translate back into the global map frame.
    return (
        origin.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        origin.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def in_bounds(cell, width, height):
    """Return whether a cell lies inside the rectangular occupancy grid."""
    return 0 <= cell[0] < width and 0 <= cell[1] < height


class GridCellMask:
    """Provide set-like blocked-cell lookup backed by a compact boolean grid."""

    def __init__(self, mask):
        self.mask = np.asarray(mask, dtype=np.bool_)
        self.count = int(np.count_nonzero(self.mask))

    def __contains__(self, cell):
        x, y = cell
        return (
            0 <= y < self.mask.shape[0]
            and 0 <= x < self.mask.shape[1]
            and bool(self.mask[y, x])
        )

    def __len__(self):
        return self.count


class GridPenaltyMap:
    """Provide dict-like obstacle-cost lookup backed by a float grid."""

    def __init__(self, values):
        self.values = np.asarray(values, dtype=np.float32)
        self.count = int(np.count_nonzero(self.values))

    def get(self, cell, default=0.0):
        x, y = cell
        if 0 <= y < self.values.shape[0] and 0 <= x < self.values.shape[1]:
            return float(self.values[y, x])
        return default

    def __len__(self):
        return self.count


def make_blocked_cells(data, width, height, occupied_threshold, allow_unknown, inflation_cells):
    """Build the set of cells that A* must not enter.

    OccupancyGrid stores cells as a flat row-major array. Values from 0 to 100
    indicate occupancy probability and -1 indicates unknown space. Unknown
    cells are treated as blocked by default because driving into unexplored
    space is unsafe unless the caller explicitly enables it.
    """
    blocked = set()
    occupied_obstacles = set()
    for index, value in enumerate(data):
        cell = (index % width, index // width)
        if value >= occupied_threshold:
            blocked.add(cell)
            occupied_obstacles.add(cell)
        elif value < 0 and not allow_unknown:
            blocked.add(cell)

    if inflation_cells <= 0:
        return blocked

    # Inflate true occupied obstacles only. Unknown cells stay blocked, but they
    # are not inflated; otherwise every frontier goal, which intentionally sits
    # next to unknown space, becomes unreachable.
    for x, y in occupied_obstacles:
        for dx in range(-inflation_cells, inflation_cells + 1):
            for dy in range(-inflation_cells, inflation_cells + 1):
                if dx * dx + dy * dy > inflation_cells * inflation_cells:
                    continue
                candidate = (x + dx, y + dy)
                if in_bounds(candidate, width, height):
                    blocked.add(candidate)
    return blocked


def make_obstacle_penalties(data, width, height, occupied_threshold, radius_cells, weight):
    """Return extra traversal costs for cells near true occupied obstacles."""
    if radius_cells <= 0 or weight <= 0.0:
        return {}

    occupied = []
    for index, value in enumerate(data):
        if value >= occupied_threshold:
            occupied.append((index % width, index // width))

    best_distance_sq = {}
    for x, y in occupied:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                distance_sq = dx * dx + dy * dy
                if distance_sq == 0 or distance_sq > radius_cells * radius_cells:
                    continue
                candidate = (x + dx, y + dy)
                if not in_bounds(candidate, width, height):
                    continue
                if distance_sq < best_distance_sq.get(candidate, math.inf):
                    best_distance_sq[candidate] = distance_sq

    penalties = {}
    radius = float(radius_cells)
    for cell, distance_sq in best_distance_sq.items():
        distance = math.sqrt(distance_sq)
        normalized_closeness = max(0.0, (radius - distance + 1.0) / radius)
        penalties[cell] = weight * normalized_closeness * normalized_closeness
    return penalties


def make_navigation_costs(
    data,
    width,
    height,
    occupied_threshold,
    allow_unknown,
    inflation_cells,
    obstacle_cost_cells,
    obstacle_cost_weight,
):
    """Build blocked cells and clearance penalties with one distance transform."""
    grid = np.asarray(data, dtype=np.int16).reshape((height, width))
    occupied = grid >= occupied_threshold

    if np.any(occupied):
        obstacle_distance = distance_transform_edt(~occupied)
    else:
        obstacle_distance = np.full((height, width), np.inf, dtype=np.float64)

    blocked_mask = occupied.copy()
    if not allow_unknown:
        blocked_mask |= grid < 0
    if inflation_cells > 0:
        blocked_mask |= obstacle_distance <= float(inflation_cells)

    blocked = GridCellMask(blocked_mask)

    penalty_values = np.zeros_like(obstacle_distance, dtype=np.float32)
    if obstacle_cost_cells > 0 and obstacle_cost_weight > 0.0:
        penalty_mask = (
            (~blocked_mask)
            & (obstacle_distance > 0.0)
            & (obstacle_distance <= float(obstacle_cost_cells))
        )
        distances = obstacle_distance[penalty_mask]
        radius = float(obstacle_cost_cells)
        penalty_values[penalty_mask] = obstacle_cost_weight * np.square(
            np.maximum(0.0, (radius - distances + 1.0) / radius)
        )

    return blocked, GridPenaltyMap(penalty_values)


def line_cells(start, end):
    """Return grid cells touched by a line segment using Bresenham traversal."""
    x0, y0 = start
    x1, y1 = end
    cells = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy

    x = x0
    y = y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * error
        if e2 > -dy:
            error -= dy
            x += sx
        if e2 < dx:
            error += dx
            y += sy
    return cells


def line_is_free(start, end, width, height, blocked, obstacle_penalties=None, max_penalty=math.inf):
    """Return True when a line is free and does not pass too near obstacles."""
    obstacle_penalties = obstacle_penalties or {}
    for cell in line_cells(start, end):
        if not in_bounds(cell, width, height) or cell in blocked:
            return False
        if obstacle_penalties.get(cell, 0.0) > max_penalty:
            return False
    return True



def reduce_waypoints_by_line_of_sight(
    cells,
    width,
    height,
    blocked,
    obstacle_penalties=None,
    max_shortcut_penalty=math.inf,
):
    """Compress a grid path without shortcutting through low-clearance cells."""
    if len(cells) <= 2:
        return list(cells)

    reduced = [cells[0]]
    current_index = 0
    while current_index < len(cells) - 1:
        next_index = len(cells) - 1
        while next_index > current_index + 1:
            if line_is_free(
                cells[current_index],
                cells[next_index],
                width,
                height,
                blocked,
                obstacle_penalties,
                max_shortcut_penalty,
            ):
                break
            next_index -= 1

        reduced.append(cells[next_index])
        current_index = next_index

    return reduced


def octile_distance(first, second):
    """Return an admissible A* heuristic for an eight-connected grid."""
    # A heuristic estimates the remaining cost from a cell to the goal. A* is
    # optimal when this estimate never overestimates the real cheapest cost.
    # Octile distance is the correct choice when diagonal moves are allowed.
    # The shortest obstacle-free route contains min(dx, dy) diagonal moves and
    # the remaining difference as cardinal moves. This estimates cost more
    # precisely than Manhattan distance when diagonal movement is enabled.
    dx = abs(first[0] - second[0])
    dy = abs(first[1] - second[1])
    return min(dx, dy) * DIAGONAL_COST + abs(dx - dy) * CARDINAL_COST


def heuristic_distance(cell, goal, heuristic_type, heuristic_weight=1.0, use_diagonal=True):
    """Compute the selected A* heuristic in grid-cell cost units."""
    dx = abs(cell[0] - goal[0])
    dy = abs(cell[1] - goal[1])
    heuristic_type = str(heuristic_type).strip().lower()

    if heuristic_type in ("zero", "none", "dijkstra"):
        value = 0.0
    elif heuristic_type in ("euclidean", "l2"):
        value = math.hypot(dx, dy)
    elif heuristic_type in ("diagonal", "octile"):
        value = octile_distance(cell, goal)
    elif heuristic_type in ("weighted_diagonal", "weighted_octile"):
        value = heuristic_weight * octile_distance(cell, goal)
    elif not use_diagonal:
        value = dx + dy
    else:
        value = octile_distance(cell, goal) if use_diagonal else dx + dy

    return value


def path_length_cells(cells):
    """Return the geometric length of a grid-cell path in cell units."""
    return sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(cells, cells[1:]))


def reconstruct_path(came_from, current):
    """Follow parent pointers from the goal back to the start cell."""
    # During A*, came_from[child] = parent records the best known predecessor
    # for each visited cell. Once the goal is reached, following these links
    # backwards reconstructs the full route from goal to start. The final
    # reverse() changes it into start-to-goal order for publishing.
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path



def astar_search(
    start,
    goal,
    width,
    height,
    blocked,
    use_diagonal=True,
    heuristic_type="diagonal",
    heuristic_weight=1.0,
    obstacle_penalties=None,
):
    """Search for a lowest-cost grid route from start to goal with A*."""
    # Core idea of A*:
    # - g(n): cost already paid from the start cell to cell n.
    # - h(n): estimated remaining cost from n to the goal.
    # - f(n) = g(n) + h(n): priority used to decide which cell to expand next.
    # The algorithm always expands the currently most promising cell. Compared
    # with Dijkstra, the heuristic pulls the search toward the goal instead of
    # spreading equally in every direction.
    # Fail early for invalid clicks, robot poses outside the map, and endpoints
    # rejected by occupancy or obstacle inflation.
    if not in_bounds(start, width, height) or not in_bounds(goal, width, height):
        return [], []
    if start in blocked or goal in blocked:
        return [], []
    obstacle_penalties = obstacle_penalties or {}

    cardinal_steps = [
        (-1, 0, CARDINAL_COST),
        (1, 0, CARDINAL_COST),
        (0, -1, CARDINAL_COST),
        (0, 1, CARDINAL_COST),
    ]
    diagonal_steps = [
        (-1, -1, DIAGONAL_COST),
        (-1, 1, DIAGONAL_COST),
        (1, -1, DIAGONAL_COST),
        (1, 1, DIAGONAL_COST),
    ]
    steps = cardinal_steps + diagonal_steps if use_diagonal else cardinal_steps

    # heapq provides the priority queue used as A*'s open set. Queue entries are
    # ordered first by f(n) = g(n) + h(n), then by h(n). Preferring the smaller
    # h-cost when f-costs tie directs the search toward the goal instead of
    # allowing grid-coordinate tuple ordering to decide the expansion order.
    # A monotonic sequence number gives the heap a deterministic final tie-break.
    #
    # The queued g-cost is retained as well. A cell can be inserted more than
    # once when a cheaper route is discovered before its older entry is popped.
    # Comparing queued_g with cost_so_far lets us discard that stale entry.
    # frontier is the open set: cells discovered but not fully expanded yet.
    # heapq keeps the cell with the smallest f-cost at the front.
    # came_from stores the path tree for reconstruction.
    # cost_so_far stores g(n), the cheapest known cost from start to each cell.
    sequence = itertools.count()
    start_h = heuristic_distance(
        start, goal, heuristic_type, heuristic_weight, use_diagonal
    )
    frontier = [(start_h, start_h, next(sequence), start, 0.0)]
    came_from = {}
    cost_so_far = {start: 0.0}
    explored = []
    closed_set = set()

    while frontier:
        # Pick the cell with the lowest estimated total route cost. If it is
        # the goal, the cheapest route has been found and can be reconstructed.
        _, _, _, current, queued_g = heapq.heappop(frontier)
        if queued_g != cost_so_far.get(current):
            continue
        if current in closed_set:
            continue

        closed_set.add(current)
        explored.append(current)
        if current == goal:
            return reconstruct_path(came_from, current), explored

        # Expand all valid neighboring cells. With diagonal movement enabled,
        # each cell has up to eight neighbors; otherwise it has four.
        for dx, dy, step_cost in steps:
            neighbor = (current[0] + dx, current[1] + dy)
            if not in_bounds(neighbor, width, height) or neighbor in blocked:
                continue

            if neighbor in closed_set:
                continue

            # A diagonal move is only valid if both adjacent cardinal cells are
            # free. Without this rule, a point-sized path could cut through the
            # touching corner of two obstacles even though a real robot cannot.
            if dx != 0 and dy != 0:
                first_side = (current[0] + dx, current[1])
                second_side = (current[0], current[1] + dy)
                if first_side in blocked or second_side in blocked:
                    continue

            new_cost = cost_so_far[current] + step_cost + obstacle_penalties.get(neighbor, 0.0)

            # Only update a cell when this route is strictly better than the
            # best route discovered previously. This bounds repeated work and
            # preserves the lowest-cost parent for path reconstruction.
            if new_cost >= cost_so_far.get(neighbor, math.inf):
                continue

            cost_so_far[neighbor] = new_cost
            came_from[neighbor] = current
            # Push the neighbor into the open set with its f-cost. If the same
            # cell is discovered again later with a worse cost, the check above
            # will ignore that worse route.
            heuristic = heuristic_distance(
                neighbor, goal, heuristic_type, heuristic_weight, use_diagonal
            )
            heapq.heappush(
                frontier,
                (
                    new_cost + heuristic,
                    heuristic,
                    next(sequence),
                    neighbor,
                    new_cost,
                ),
            )

    # Exhausting the frontier means the free-space graph does not connect the
    # selected start and goal cells.
    return [], explored


def astar(start, goal, width, height, blocked, use_diagonal=True):
    """Backward-compatible A* wrapper returning only the path."""
    cells, _ = astar_search(start, goal, width, height, blocked, use_diagonal)
    return cells


class AStarGlobalPlanner(Node):
    """ROS node that runs A* when RViz supplies a new global goal pose."""

    def __init__(self):
        """Declare parameters, initialize state, and create ROS interfaces."""
        super().__init__("astar_global_planner")

        # The SLAM occupancy grid and TF tree are the authoritative navigation
        # state. A topic pose source is kept only as a backwards-compatible
        # option for experiments that do not publish map -> base TF.
        self.declare_parameter("map_topic", "/P1_robot0/map") # change the map source to /P1_robot0/map
        self.declare_parameter("pose_source", "tf")  # Select TF or topic pose input.
        self.declare_parameter("pose_topic", "/P1_robot0/odom_gt")
        self.declare_parameter("pose_frame_override", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "P1_robot0/base_link")
        self.declare_parameter("tf_lookup_rate", 20.0)
        self.declare_parameter("goal_pose_topic", "/P1_robot0/goal_pose")
        self.declare_parameter("goal_update_distance", 0.01)

        # Output topics intentionally match the hand-drawn planner so either
        # planner can feed the same visualization and waypoint-following nodes.
        # Do not run both planners simultaneously unless their outputs are
        # remapped, because they would publish competing routes.
        self.declare_parameter("path_topic", "planned_route")
        self.declare_parameter("pose_array_topic", "planned_route_poses")
        self.declare_parameter("marker_topic", "planned_route_marker")
        self.declare_parameter("search_marker_topic", "astar_search_cells")

        # Occupancy values greater than or equal to this threshold are blocked.
        # Unknown cells are blocked unless allow_unknown is explicitly enabled.
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("allow_unknown", False)

        # Expand obstacles before planning so the route respects approximate
        # robot size. The radius is expressed in meters and converted into grid
        # cells after a map is received because map resolution is map-specific.
        self.declare_parameter("inflation_radius", 0.30)
        self.declare_parameter("obstacle_cost_radius", 0.45)
        self.declare_parameter("obstacle_cost_weight", 4.0)
        self.declare_parameter("endpoint_search_radius", 0.0)
        self.declare_parameter("use_diagonal", True)
        self.declare_parameter("reduce_waypoints", True)
        self.declare_parameter("max_shortcut_obstacle_cost", 1.0)
        self.declare_parameter("heuristic", "diagonal")
        self.declare_parameter("heuristic_weight", 1.5)
        self.declare_parameter("visualize_search", True)
        self.declare_parameter("max_visualized_search_cells", 2500)

        # A saved map rarely changes, so replanning on every map update is off
        # by default. Enable this when using a live occupancy grid if route
        # updates should follow newly observed static obstacles.
        self.declare_parameter("replan_on_map_update", False)
        self.declare_parameter("replan_rate", 5.0)
        self.declare_parameter("map_preprocess_rate", 2.0)
        self.declare_parameter("freeze_prefix_waypoints", 2)
        self.declare_parameter("waypoint_reached_threshold", 0.10)
        self.declare_parameter("route_progress_max_distance", 1.0)
        self.declare_parameter("waypoint_spacing", 0.20)
        self.declare_parameter("route_z_offset", 0.05)

        # Resolve parameters once at startup. Dynamic parameter updates are not
        # implemented by this compact standalone planner.
        gp = lambda name: self.get_parameter(name).value
        self.map_topic = gp("map_topic")
        self.pose_source = str(gp("pose_source")).strip().lower()
        self.pose_topic = gp("pose_topic")
        self.pose_frame_override = str(gp("pose_frame_override")).strip()
        self.map_frame = str(gp("map_frame")).strip().lstrip("/")
        self.base_frame = str(gp("base_frame")).strip().lstrip("/")
        self.tf_lookup_rate = max(0.1, float(gp("tf_lookup_rate")))
        self.goal_pose_topic = gp("goal_pose_topic")
        self.goal_update_distance = max(0.0, float(gp("goal_update_distance")))
        self.path_topic = gp("path_topic")
        self.pose_array_topic = gp("pose_array_topic")
        self.marker_topic = gp("marker_topic")
        self.search_marker_topic = gp("search_marker_topic")
        self.occupied_threshold = int(gp("occupied_threshold"))
        self.allow_unknown = parameter_as_bool(gp("allow_unknown"))
        self.inflation_radius = float(gp("inflation_radius"))
        self.obstacle_cost_radius = max(0.0, float(gp("obstacle_cost_radius")))
        self.obstacle_cost_weight = max(0.0, float(gp("obstacle_cost_weight")))
        self.endpoint_search_radius = max(0.0, float(gp("endpoint_search_radius")))
        self.use_diagonal = parameter_as_bool(gp("use_diagonal"))
        self.reduce_waypoints = parameter_as_bool(gp("reduce_waypoints"))
        self.max_shortcut_obstacle_cost = max(0.0, float(gp("max_shortcut_obstacle_cost")))
        self.heuristic = str(gp("heuristic")).strip().lower()
        self.heuristic_weight = float(gp("heuristic_weight"))
        self.visualize_search = parameter_as_bool(gp("visualize_search"))
        self.max_visualized_search_cells = max(1, int(gp("max_visualized_search_cells")))
        self.replan_on_map_update = parameter_as_bool(gp("replan_on_map_update"))
        self.replan_rate = max(0.0, float(gp("replan_rate")))
        self.map_preprocess_rate = max(0.0, float(gp("map_preprocess_rate")))
        self.freeze_prefix_waypoints = max(0, int(gp("freeze_prefix_waypoints")))
        self.waypoint_reached_threshold = max(0.0, float(gp("waypoint_reached_threshold")))
        self.route_progress_max_distance = max(0.0, float(gp("route_progress_max_distance")))
        self.waypoint_spacing = max(0.0, float(gp("waypoint_spacing")))
        self.route_z_offset = float(gp("route_z_offset"))

        # Cache the latest inputs. Goals may arrive before the map or pose, so
        # retaining all three messages lets the planner run as soon as every
        # required input becomes available.
        self.map_msg = None
        self.blocked_cells = None
        self.obstacle_penalties = {}
        self.inflation_cells = 0
        self.obstacle_cost_cells = 0
        self.map_preprocess_time_ms = 0.0
        self.last_map_preprocess_time = -math.inf
        self.robot_pose = None
        self.goal_pose = None
        self.last_route_points = []
        self.waiting_for_inputs = False

        # nav2_map_server publishes a latched-style transient-local map. Matching
        # that durability allows a planner started later to receive the latest
        # saved map immediately instead of waiting for another publication.
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        if self.pose_source == "topic":
            self.create_subscription(Odometry, self.pose_topic, self.on_pose, 10)
            self.tf_pose_timer = None
        elif self.pose_source == "tf":
            self.tf_pose_timer = self.create_timer(
                1.0 / self.tf_lookup_rate, self.update_robot_pose_from_tf
            )
        else:
            raise ValueError("pose_source must be 'tf' or 'topic'")
        self.create_subscription(PoseStamped, self.goal_pose_topic, self.on_goal_pose, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.pose_array_pub = self.create_publisher(PoseArray, self.pose_array_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.search_marker_pub = self.create_publisher(Marker, self.search_marker_topic, 10)
        self.replan_timer = None
        if self.replan_rate > 0.0:
            self.replan_timer = self.create_timer(1.0 / self.replan_rate, self.on_replan_timer)

        if self.pose_source == "tf":
            pose_description = f"TF '{self.map_frame}' -> '{self.base_frame}'"
        else:
            pose_frame_note = (
                f" interpreted as frame '{self.pose_frame_override}'"
                if self.pose_frame_override else ""
            )
            pose_description = f"pose '{self.pose_topic}'{pose_frame_note}"
        self.get_logger().info(
            f"Ready. Waiting for map '{self.map_topic}', {pose_description}, "
            f"and RViz goals on '{self.goal_pose_topic}'. "
            f"replan_rate={self.replan_rate:.2f} Hz, "
            f"freeze_prefix_waypoints={self.freeze_prefix_waypoints}, "
            f"waypoint_reached_threshold={self.waypoint_reached_threshold:.2f} m."
        )

    def on_map(self, msg):
        """Cache a new occupancy grid and its inflated obstacle cells."""
        now = time.monotonic()
        preprocess_period = (
            1.0 / self.map_preprocess_rate if self.map_preprocess_rate > 0.0 else 0.0
        )
        if self.blocked_cells is not None and now - self.last_map_preprocess_time < preprocess_period:
            return

        self.map_msg = msg
        preprocess_start = time.perf_counter()
        self.inflation_cells = math.ceil(self.inflation_radius / msg.info.resolution)
        self.obstacle_cost_cells = math.ceil(self.obstacle_cost_radius / msg.info.resolution)
        self.blocked_cells, self.obstacle_penalties = make_navigation_costs(
            msg.data,
            msg.info.width,
            msg.info.height,
            self.occupied_threshold,
            self.allow_unknown,
            self.inflation_cells,
            self.obstacle_cost_cells,
            self.obstacle_cost_weight,
        )
        self.last_map_preprocess_time = time.monotonic()
        self.map_preprocess_time_ms = (time.perf_counter() - preprocess_start) * 1000.0
        self.get_logger().info(
            f"Cached inflated A* map: size={msg.info.width}x{msg.info.height}, "
            f"inflation_radius={self.inflation_radius:.2f} m, "
            f"inflation_cells={self.inflation_cells}, "
            f"obstacle_cost_radius={self.obstacle_cost_radius:.2f} m, "
            f"obstacle_penalty_cells={len(self.obstacle_penalties)}, "
            f"blocked_cells={len(self.blocked_cells)}, "
            f"map_preprocess_time={self.map_preprocess_time_ms:.2f} ms."
        )
        if self.goal_pose is not None and (self.waiting_for_inputs or self.replan_on_map_update):
            self.plan(freeze_prefix=bool(self.last_route_points))

    def on_pose(self, msg):
        """Cache the newest robot pose and plan if an earlier goal is waiting."""
        self.robot_pose = msg
        if self.last_route_points:
            self.prune_reached_waypoints(msg.pose.pose.position)
        if self.goal_pose is not None and self.waiting_for_inputs:
            self.plan()

    def update_robot_pose_from_tf(self):
        """Read the current SLAM pose from map -> base TF."""
        map_frame = (
            self.map_msg.header.frame_id.lstrip("/")
            if self.map_msg is not None and self.map_msg.header.frame_id
            else self.map_frame
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Waiting for TF '{map_frame}' -> '{self.base_frame}': {exc}",
                throttle_duration_sec=5.0,
            )
            return

        pose = Odometry()
        pose.header = transform.header
        pose.header.frame_id = map_frame
        pose.child_frame_id = self.base_frame
        pose.pose.pose.position.x = transform.transform.translation.x
        pose.pose.pose.position.y = transform.transform.translation.y
        pose.pose.pose.position.z = transform.transform.translation.z
        pose.pose.pose.orientation = transform.transform.rotation
        self.on_pose(pose)

    def on_replan_timer(self):
        """Periodically replan from the frozen NMPC prefix anchor."""
        if self.goal_pose is None:
            return
        self.plan(freeze_prefix=True)

    def on_goal_pose(self, msg):
        """Replan immediately only when the goal position genuinely changes."""
        goal_changed = True
        if self.goal_pose is not None:
            previous = self.goal_pose.pose.position
            incoming = msg.pose.position
            shift = math.hypot(incoming.x - previous.x, incoming.y - previous.y)
            goal_changed = shift >= self.goal_update_distance

        self.goal_pose = msg
        if goal_changed or not self.last_route_points:
            self.plan(freeze_prefix=bool(self.last_route_points))

    def prune_reached_waypoints(self, robot_position):
        """Keep only route waypoints that are still ahead of the robot."""
        if not self.last_route_points:
            return

        removed = 0
        if len(self.last_route_points) >= 2 and self.route_progress_max_distance > 0.0:
            best_distance = math.inf
            best_segment = 0
            best_progress = 0.0
            for index, (first, second) in enumerate(
                zip(self.last_route_points, self.last_route_points[1:])
            ):
                dx = second.x - first.x
                dy = second.y - first.y
                length_sq = dx * dx + dy * dy
                if length_sq <= 1e-12:
                    progress = 0.0
                else:
                    progress = max(
                        0.0,
                        min(
                            1.0,
                            ((robot_position.x - first.x) * dx + (robot_position.y - first.y) * dy)
                            / length_sq,
                        ),
                    )
                projected_x = first.x + progress * dx
                projected_y = first.y + progress * dy
                distance = math.hypot(
                    robot_position.x - projected_x, robot_position.y - projected_y
                )
                if distance < best_distance:
                    best_distance = distance
                    best_segment = index
                    best_progress = progress

            if best_distance <= self.route_progress_max_distance:
                removed = best_segment + (1 if best_progress > 0.05 else 0)
                if removed:
                    del self.last_route_points[:removed]

        if self.waypoint_reached_threshold <= 0.0:
            return
        while self.last_route_points:
            waypoint = self.last_route_points[0]
            distance = math.hypot(robot_position.x - waypoint.x, robot_position.y - waypoint.y)
            if distance > self.waypoint_reached_threshold:
                break
            self.last_route_points.pop(0)
            removed += 1
        if removed:
            self.get_logger().info(
                f"Pruned {removed} reached waypoint(s); {len(self.last_route_points)} remain cached."
            )

    def plan(self, freeze_prefix=False):
        """Validate inputs, execute A*, and publish the resulting route."""
        # This method is the bridge between ROS and the pure A* function above.
        # It gathers the latest map, robot pose, and RViz goal; converts metric
        # coordinates into grid cells; runs A*; then converts the cell path back
        # into ROS messages for RViz and downstream nodes.
        # A user may click before the map server or localization node is ready.
        # Store the goal and defer work instead of discarding the request.
        total_start = time.perf_counter()

        missing = []
        if self.map_msg is None or self.blocked_cells is None:
            missing.append("map")
        if self.robot_pose is None:
            missing.append("robot pose")
        if missing:
            self.waiting_for_inputs = True
            self.get_logger().warning(f"Goal stored; waiting for {' and '.join(missing)}.")
            return

        self.waiting_for_inputs = False
        map_frame = self.map_msg.header.frame_id or "map"
        raw_pose_frame = self.robot_pose.header.frame_id
        pose_frame = self.pose_frame_override or raw_pose_frame
        goal_frame = self.goal_pose.header.frame_id

        # Robot pose frame mismatches are reported clearly unless
        # pose_frame_override is set, which is useful for Gazebo ground truth
        # coordinates that are known to use the saved map coordinate system but
        # carry an odom frame_id. RViz goals may arrive in another frame, so
        # those are transformed into the map frame through TF before planning.
        if raw_pose_frame and self.pose_frame_override and raw_pose_frame != self.pose_frame_override:
            self.get_logger().warn(
                f"Interpreting robot pose frame '{raw_pose_frame}' as "
                f"'{self.pose_frame_override}' because pose_frame_override is set.",
                throttle_duration_sec=5.0,
            )
        if pose_frame and pose_frame != map_frame:
            self.fail_plan(f"Robot pose frame '{pose_frame}' does not match map frame '{map_frame}'.")
            return
        info = self.map_msg.info
        start_position = self.robot_pose.pose.pose.position
        goal_position = self.goal_pose.pose.position
        if goal_frame and goal_frame != map_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    map_frame.lstrip("/"), goal_frame.lstrip("/"), rclpy.time.Time()
                )
                goal_position = transform_point(goal_position, transform)
                self.get_logger().warn(
                    f"Transformed goal from frame '{goal_frame}' to map frame '{map_frame}'.",
                    throttle_duration_sec=5.0,
                )
            except Exception as exc:
                self.fail_plan(
                    f"Goal frame '{goal_frame}' does not match map frame '{map_frame}', "
                    f"and TF transform failed: {exc}"
                )
                return
        # Convert robot start and goal from meters into occupancy-grid indices.
        # Periodic replanning keeps the first N cached route points unchanged and
        # runs A* from the Nth cached waypoint to the latest goal.
        self.prune_reached_waypoints(start_position)
        prefix_points = []
        frozen_prefix_count = 0
        if (
            freeze_prefix
            and self.freeze_prefix_waypoints > 0
            and len(self.last_route_points) >= self.freeze_prefix_waypoints
        ):
            prefix_points = list(self.last_route_points[: self.freeze_prefix_waypoints])
            frozen_prefix_count = len(prefix_points)
            anchor = prefix_points[-1]
            start = world_to_grid(anchor.x, anchor.y, info)
            start_source = f"waypoint_{frozen_prefix_count}"
        else:
            start = world_to_grid(start_position.x, start_position.y, info)
            start_source = "robot_pose"
        goal = world_to_grid(goal_position.x, goal_position.y, info)

        # Reuse the inflated obstacle map cached in on_map(). For a static map,
        # obstacle inflation is preprocessing and is not counted in per-goal
        # planning time.
        blocked = self.blocked_cells
        original_start = start
        original_goal = goal
        start = self.nearest_unblocked_cell(start, info, blocked)
        planned_goal = self.nearest_unblocked_cell(goal, info, blocked)
        if start is None or planned_goal is None:
            self.fail_plan(
                f"Could not find unblocked planning endpoint near start {original_start} "
                f"or goal {original_goal}; endpoint_search_radius={self.endpoint_search_radius:.2f} m."
            )
            return
        if start != original_start:
            self.get_logger().warn(
                f"A* start cell {original_start} is blocked; using nearby free cell {start}.",
                throttle_duration_sec=5.0,
            )
        if planned_goal != original_goal:
            self.get_logger().warn(
                f"A* goal cell {original_goal} is blocked; using nearby free cell {planned_goal}.",
                throttle_duration_sec=5.0,
            )

        search_start = time.perf_counter()
        cells, explored_cells = astar_search(
            start,
            planned_goal,
            info.width,
            info.height,
            blocked,
            self.use_diagonal,
            self.heuristic,
            self.heuristic_weight,
            self.obstacle_penalties,
        )
        search_time_ms = (time.perf_counter() - search_start) * 1000.0
        if not cells:
            message = (
                f"No route found from grid cell {start} to {planned_goal} "
                f"(requested {original_start} to {original_goal}). "
                "Check map bounds, obstacles, unknown cells, and inflation_radius."
            )
            if prefix_points:
                self.get_logger().warn(message + " Retrying from the current robot pose.")
                prefix_points = []
                frozen_prefix_count = 0
                start_source = "robot_pose_fallback"
                original_start = world_to_grid(start_position.x, start_position.y, info)
                start = self.nearest_unblocked_cell(original_start, info, blocked)
                if start is None:
                    self.fail_plan(
                        f"Could not find unblocked fallback start near robot cell {original_start}; "
                        f"endpoint_search_radius={self.endpoint_search_radius:.2f} m."
                    )
                    return

                search_start = time.perf_counter()
                cells, explored_cells = astar_search(
                    start,
                    planned_goal,
                    info.width,
                    info.height,
                    blocked,
                    self.use_diagonal,
                    self.heuristic,
                    self.heuristic_weight,
                    self.obstacle_penalties,
                )
                search_time_ms = (time.perf_counter() - search_start) * 1000.0
                if not cells:
                    self.fail_plan(message + " Fallback from the current robot pose also failed.")
                    return
            else:
                self.fail_plan(message)
                return

        raw_count = len(cells)
        raw_length = path_length_cells(cells) * info.resolution
        if self.reduce_waypoints:
            cells = reduce_waypoints_by_line_of_sight(
                cells,
                info.width,
                info.height,
                blocked,
                self.obstacle_penalties,
                self.max_shortcut_obstacle_cost,
            )

        tail_length = path_length_cells(cells) * info.resolution

        # A* returns integer cells. Publish route poses at their world-space
        # centers and slightly above the map plane for clear RViz rendering.
        planned_points = [
            Point(x=x, y=y, z=self.route_z_offset)
            for x, y in (grid_to_world(cell, info) for cell in cells)
        ]
        planned_points = self.resample_waypoints(planned_points, self.waypoint_spacing)
        if prefix_points and planned_points:
            # The first planned point is the frozen anchor, so avoid publishing it twice.
            points = prefix_points + planned_points[1:]
        else:
            points = planned_points
        reduction_state = "on" if self.reduce_waypoints else "off"
        published_length = sum(
            math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(points, points[1:])
        )
        self.publish_route(points, map_frame)
        self.last_route_points = list(points)
        if self.visualize_search:
            self.publish_search_cells(
                explored_cells,
                info,
                map_frame,
                delete_marker=False,
                max_cells=self.max_visualized_search_cells,
            )

        total_planning_time_ms = (time.perf_counter() - total_start) * 1000.0
        effective_heuristic_weight = (
            self.heuristic_weight
            if self.heuristic in ("weighted_diagonal", "weighted_octile")
            else 1.0
        )
        self.get_logger().info(
            f"Published A* route from {start} to {planned_goal}: "
            f"heuristic={self.heuristic}, heuristic_weight={effective_heuristic_weight:.2f}, "
            f"start_source={start_source}, frozen_waypoints={frozen_prefix_count}, "
            f"search_time={search_time_ms:.2f} ms, "
            f"total_planning_time={total_planning_time_ms:.2f} ms, "
            f"expanded_cells={len(explored_cells)}, "
            f"waypoint_reduction={reduction_state}, "
            f"max_shortcut_obstacle_cost={self.max_shortcut_obstacle_cost:.2f}, "
            f"raw_poses={raw_count}, raw_length={raw_length:.2f} m, "
            f"tail_length={tail_length:.2f} m, "
            f"published_poses={len(points)}, published_length={published_length:.2f} m, "
            f"waypoint_spacing={self.waypoint_spacing:.2f} m, "
            f"inflation_radius={self.inflation_radius:.2f} m, "
            f"cached_map_preprocess_time={self.map_preprocess_time_ms:.2f} ms."
        )

    @staticmethod
    def resample_waypoints(points, spacing):
        """Return path points at a stable maximum spacing for NMPC consumers."""
        if spacing <= 0.0 or len(points) <= 1:
            return list(points)

        result = [points[0]]
        for target in points[1:]:
            start = result[-1]
            distance = math.hypot(target.x - start.x, target.y - start.y)
            while distance > spacing:
                ratio = spacing / distance
                start = Point(
                    x=start.x + ratio * (target.x - start.x),
                    y=start.y + ratio * (target.y - start.y),
                    z=start.z + ratio * (target.z - start.z),
                )
                result.append(start)
                distance = math.hypot(target.x - start.x, target.y - start.y)
            if distance > 1e-6:
                result.append(target)
        return result


    def fail_plan(self, message):
        """Report a planning failure and clear any previously published route."""
        self.get_logger().error(message)
        self.last_route_points = []
        if self.map_msg is not None:
            self.publish_search_cells(
                [],
                self.map_msg.info,
                self.map_msg.header.frame_id or "map",
                delete_marker=True,
            )

        # Clearing stale output matters: leaving the last successful route on
        # screen could suggest that it is still valid for the new goal.
        self.publish_route([], self.map_msg.header.frame_id or "map", delete_marker=True)

    def nearest_unblocked_cell(self, cell, map_info, blocked):
        """Return cell itself or the closest nearby cell A* can enter."""
        if in_bounds(cell, map_info.width, map_info.height) and cell not in blocked:
            return cell

        max_radius = math.ceil(self.endpoint_search_radius / map_info.resolution)
        for radius in range(1, max_radius + 1):
            best = None
            best_distance_sq = math.inf
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if not in_bounds(candidate, map_info.width, map_info.height):
                        continue
                    if candidate in blocked:
                        continue
                    distance_sq = dx * dx + dy * dy
                    if distance_sq < best_distance_sq:
                        best = candidate
                        best_distance_sq = distance_sq
            if best is not None:
                return best
        return None

    def publish_route(self, points, frame_id, delete_marker=False):
        """Publish an A* route as Path, PoseArray, and RViz LINE_STRIP messages."""
        # The same route is published in three formats:
        # - Path: standard navigation output, best for planners/controllers.
        # - PoseArray: simple list of poses, useful for custom consumers.
        # - Marker: visual line strip so the route is easy to see in RViz.
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame_id
        pose_array = PoseArray()
        pose_array.header = path.header

        # Use one derived orientation per cell so a future waypoint follower can
        # consume the published path directly without recomputing headings.
        for index, point in enumerate(points):
            pose = Pose()
            pose.position = point
            pose.orientation = self.orientation_for_point(points, index)
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)
            pose_array.poses.append(pose)

        self.path_pub.publish(path)
        self.pose_array_pub.publish(pose_array)

        marker = Marker()
        marker.header = path.header
        marker.ns = "astar_route"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE if delete_marker else Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = points
        self.marker_pub.publish(marker)

        if delete_marker or not points:
            for marker_id in (1, 2):
                endpoint = Marker()
                endpoint.header = path.header
                endpoint.ns = "astar_route_endpoint"
                endpoint.id = marker_id
                endpoint.action = Marker.DELETE
                self.marker_pub.publish(endpoint)
            return

        start_marker = self.route_endpoint_marker(path.header, points[0], 1, 0.1, 0.95, 0.25)
        goal_marker = self.route_endpoint_marker(path.header, points[-1], 2, 1.0, 0.15, 0.1)
        self.marker_pub.publish(start_marker)
        self.marker_pub.publish(goal_marker)

    def publish_search_cells(self, cells, map_info, frame_id, delete_marker=False, max_cells=None):
        """Publish explored A* cells as RViz POINTS for search-process visualization."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = "astar_search"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.DELETE if delete_marker else Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(map_info.resolution * 0.75, 0.035)
        marker.scale.y = max(map_info.resolution * 0.75, 0.035)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.85
        marker.color.a = 0.30
        if max_cells is not None and len(cells) > max_cells:
            stride = max(1, math.ceil(len(cells) / max_cells))
            cells = cells[::stride][:max_cells]
        marker.points = [
            Point(x=x, y=y, z=self.route_z_offset * 0.5)
            for x, y in (grid_to_world(cell, map_info) for cell in cells)
        ]
        self.search_marker_pub.publish(marker)

    @staticmethod
    def route_endpoint_marker(header, point, marker_id, red, green, blue):
        """Return a sphere marker for the route start or goal endpoint."""
        marker = Marker()
        marker.header = header
        marker.ns = "astar_route_endpoint"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z + 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.16
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        return marker

    @staticmethod
    def orientation_for_point(points, index):
        """Derive the planar heading of a route point from its neighbors."""
        if len(points) < 2:
            return Quaternion(w=1.0)

        current = points[index]
        target = points[index + 1] if index < len(points) - 1 else points[index - 1]
        yaw = math.atan2(target.y - current.y, target.x - current.x)

        # For the final point, target points backward. Add pi so its heading
        # remains aligned with the route's forward travel direction.
        if index == len(points) - 1:
            yaw += math.pi
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main(args=None):
    """Start the planner and keep processing ROS callbacks until shutdown."""
    rclpy.init(args=args)
    node = AStarGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
