"""Calculate and publish a global route with the RRT algorithm.

This planner is a standalone ROS 2 node like astar_global_planner.py. It uses
an occupancy grid, the robot pose, and an RViz goal pose. The coordinate-frame
handling follows the A* planner: robot poses can be interpreted through
pose_frame_override, and RViz goals are transformed into the map frame with TF
when needed.

RRT* is sampling-based: instead of expanding cells in cost order like A*, it
randomly samples free cells, grows a tree from the robot start, and rewires
nearby nodes to improve the path cost after a collision-free route is found.
"""

import math
import time
import random

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Reuse the coordinate-conversion, obstacle inflation, and TF point-transform
# helpers from the A* planner. This keeps both planners consistent: they use the
# same interpretation of the map origin, map resolution, blocked cells, and RViz
# goal frame conversion.
from p03_rviz_route_planner.astar_global_planner import (
    grid_to_world,
    in_bounds,
    make_blocked_cells,
    quaternion_to_yaw,
    transform_point,
    world_to_grid,
)


def parameter_as_bool(value):
    """Parse ROS parameter values that may arrive as bools or strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def euclidean_points(first, second):
    """Return Euclidean distance between two map-frame points in meters."""
    return math.hypot(first[0] - second[0], first[1] - second[1])


def line_cells_for_goal_adjustment(start, end):
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


def free_goal_candidates_toward_start(start, goal, width, height, blocked):
    """Yield free candidate goals from closest-to-goal toward the start cell."""
    if not in_bounds(start, width, height) or start in blocked:
        return

    previous = None
    for cell in line_cells_for_goal_adjustment(goal, start):
        if cell == start:
            return
        if cell == previous:
            continue
        previous = cell
        if in_bounds(cell, width, height) and cell not in blocked:
            yield cell


def reachable_paths_from_start(start, width, height, blocked, use_diagonal=True):
    """Explore reachable free cells from start for RRT adjusted-goal fallback."""
    if not in_bounds(start, width, height) or start in blocked:
        return set(), {}, []

    cardinal_steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    diagonal_steps = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    steps = cardinal_steps + diagonal_steps if use_diagonal else cardinal_steps

    reachable = {start}
    came_from = {}
    explored = []
    queue = [start]
    queue_index = 0
    while queue_index < len(queue):
        current = queue[queue_index]
        queue_index += 1
        explored.append(current)
        for dx, dy in steps:
            neighbor = (current[0] + dx, current[1] + dy)
            if not in_bounds(neighbor, width, height) or neighbor in blocked:
                continue
            if neighbor in reachable:
                continue
            if dx != 0 and dy != 0:
                first_side = (current[0] + dx, current[1])
                second_side = (current[0], current[1] + dy)
                if first_side in blocked or second_side in blocked:
                    continue
            reachable.add(neighbor)
            came_from[neighbor] = current
            queue.append(neighbor)

    return reachable, came_from, explored


def steer_toward(start, target, step_size):
    """Move from start toward target by at most step_size meters."""
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance <= step_size:
        return target
    scale = step_size / distance
    return (start[0] + dx * scale, start[1] + dy * scale)


def grid_to_world_point(cell, map_info):
    """Convert an integer grid cell to the world point at its center."""
    return grid_to_world(cell, map_info)


def grid_local_to_world(local_x, local_y, map_info):
    """Convert continuous grid-local meters into map-frame world coordinates."""
    origin = map_info.origin.position
    yaw = quaternion_to_yaw(map_info.origin.orientation)
    return (
        origin.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        origin.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def world_to_continuous_grid(point, map_info):
    """Convert a map-frame point to continuous occupancy-grid coordinates."""
    origin = map_info.origin.position
    yaw = quaternion_to_yaw(map_info.origin.orientation)
    dx = point[0] - origin.x
    dy = point[1] - origin.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return (local_x / map_info.resolution, local_y / map_info.resolution)


def segment_grid_cells(start, end, map_info):
    """Return every grid cell touched by a continuous segment (supercover DDA)."""
    x0, y0 = world_to_continuous_grid(start, map_info)
    x1, y1 = world_to_continuous_grid(end, map_info)
    cell_x = math.floor(x0)
    cell_y = math.floor(y0)
    end_x = math.floor(x1)
    end_y = math.floor(y1)
    cells = [(cell_x, cell_y)]
    seen = {(cell_x, cell_y)}

    dx = x1 - x0
    dy = y1 - y0
    step_x = 1 if dx > 0.0 else -1 if dx < 0.0 else 0
    step_y = 1 if dy > 0.0 else -1 if dy < 0.0 else 0
    t_delta_x = abs(1.0 / dx) if step_x else math.inf
    t_delta_y = abs(1.0 / dy) if step_y else math.inf
    next_x = cell_x + 1.0 if step_x > 0 else float(cell_x)
    next_y = cell_y + 1.0 if step_y > 0 else float(cell_y)
    t_max_x = (next_x - x0) / dx if step_x else math.inf
    t_max_y = (next_y - y0) / dy if step_y else math.inf

    def add(cell):
        if cell not in seen:
            seen.add(cell)
            cells.append(cell)

    while cell_x != end_x or cell_y != end_y:
        if t_max_x < t_max_y and not math.isclose(t_max_x, t_max_y, abs_tol=1e-12):
            cell_x += step_x
            t_max_x += t_delta_x
            add((cell_x, cell_y))
        elif t_max_y < t_max_x and not math.isclose(t_max_x, t_max_y, abs_tol=1e-12):
            cell_y += step_y
            t_max_y += t_delta_y
            add((cell_x, cell_y))
        else:
            # At an exact grid corner, conservatively include both side cells
            # as well as the diagonal cell so corner grazing cannot be missed.
            add((cell_x + step_x, cell_y))
            add((cell_x, cell_y + step_y))
            cell_x += step_x
            cell_y += step_y
            t_max_x += t_delta_x
            t_max_y += t_delta_y
            add((cell_x, cell_y))

    return cells


def add_spatial_node(buckets, node, bucket_size):
    """Insert one RRT node into a spatial hash used for nearest lookup."""
    key = (math.floor(node[0] / bucket_size), math.floor(node[1] / bucket_size))
    buckets.setdefault(key, []).append(node)


def nearest_spatial_node(sample, buckets, bucket_size):
    """Return the exact nearest node while avoiding a full-tree scan."""
    center_x = math.floor(sample[0] / bucket_size)
    center_y = math.floor(sample[1] / bucket_size)
    best_node = None
    best_distance = math.inf
    ring = 0

    while True:
        for bucket_x in range(center_x - ring, center_x + ring + 1):
            for bucket_y in range(center_y - ring, center_y + ring + 1):
                if ring and (abs(bucket_x - center_x) != ring and abs(bucket_y - center_y) != ring):
                    continue
                for node in buckets.get((bucket_x, bucket_y), ()):
                    distance = euclidean_points(node, sample)
                    if distance < best_distance:
                        best_node = node
                        best_distance = distance

        left = (center_x - ring) * bucket_size
        right = (center_x + ring + 1) * bucket_size
        bottom = (center_y - ring) * bucket_size
        top = (center_y + ring + 1) * bucket_size
        distance_to_outside = min(
            sample[0] - left, right - sample[0], sample[1] - bottom, top - sample[1]
        )
        if best_node is not None and best_distance <= distance_to_outside:
            return best_node
        ring += 1


def point_is_free(point, map_info, width, height, blocked):
    """Return whether a continuous map-frame point lies in an unblocked cell."""
    cell = world_to_grid(point[0], point[1], map_info)
    return in_bounds(cell, width, height) and cell not in blocked


def segment_is_free(start, end, map_info, width, height, blocked):
    """Return whether every occupancy-grid cell touched by a segment is free."""
    for cell in segment_grid_cells(start, end, map_info):
        if not in_bounds(cell, width, height) or cell in blocked:
            return False
    return True


def random_free_point(map_info, width, height, blocked, rng):
    """Sample a continuous free point inside the occupancy-grid bounds."""
    max_x = width * map_info.resolution
    max_y = height * map_info.resolution
    for _ in range(1000):
        point = grid_local_to_world(rng.uniform(0.0, max_x), rng.uniform(0.0, max_y), map_info)
        if point_is_free(point, map_info, width, height, blocked):
            return point
    return None


def sample_in_disc(center, radius, rng):
    """Uniformly sample a continuous point inside a circle."""
    angle = rng.uniform(-math.pi, math.pi)
    distance = radius * math.sqrt(rng.random())
    return (
        center[0] + distance * math.cos(angle),
        center[1] + distance * math.sin(angle),
    )


def record_trace_event(trace_events, max_trace_events, event):
    """Append one bounded sampling event without affecting planning behavior."""
    if trace_events is None or len(trace_events) >= max_trace_events:
        return
    trace_events.append(event)


def empty_rrt_stats():
    """Create counters used to diagnose RRT/RRT* runtime and rejection causes."""
    return {
        "iterations_used": 0,
        "global_samples": 0,
        "local_samples_attempted": 0,
        "radius_shrinks": 0,
        "collision_rejections": 0,
        "invalid_point_rejections": 0,
        "zero_length_rejections": 0,
        "duplicate_rejections": 0,
        "nodes_added": 0,
        "goal_connection_checks": 0,
        "goal_connection_rejections": 0,
        "parent_candidates": 0,
        "rewire_checks": 0,
        "rewires": 0,
        "best_cost": math.inf,
    }


def format_rrt_stats(label, stats):
    """Format compact RRT diagnostics for planner logs."""
    return (
        f"{label}[iterations={stats['iterations_used']}, "
        f"local_samples={stats['local_samples_attempted']}, "
        f"radius_shrinks={stats['radius_shrinks']}, "
        f"collision_rejections={stats['collision_rejections']}, "
        f"invalid_points={stats['invalid_point_rejections']}, "
        f"duplicates={stats['duplicate_rejections']}, "
        f"nodes_added={stats['nodes_added']}, "
        f"goal_checks={stats['goal_connection_checks']}, "
        f"goal_blocked={stats['goal_connection_rejections']}, "
        f"parent_candidates={stats['parent_candidates']}, "
        f"rewire_checks={stats['rewire_checks']}, "
        f"rewires={stats['rewires']}, "
        f"best_cost={stats['best_cost']:.2f}m]"
    )


def adaptive_radius_extension(
    parent,
    target,
    map_info,
    width,
    height,
    blocked,
    maximum_radius,
    minimum_radius,
    radius_decay,
    samples_per_radius,
    rng,
    trace_events=None,
    max_trace_events=0,
):
    """Find a valid local extension, shrinking the sampling radius on failure."""
    radius = maximum_radius
    stats = {
        "local_samples_attempted": 0,
        "radius_shrinks": 0,
        "collision_rejections": 0,
        "invalid_point_rejections": 0,
        "zero_length_rejections": 0,
    }

    while radius + 1e-12 >= minimum_radius:
        for attempt in range(samples_per_radius):
            # First try at every radius remains directed toward the global/goal
            # sample. Further attempts explore the whole local sampling circle.
            candidate = (
                steer_toward(parent, target, radius)
                if attempt == 0
                else sample_in_disc(parent, radius, rng)
            )
            stats["local_samples_attempted"] += 1

            if euclidean_points(candidate, parent) < 1e-9:
                stats["zero_length_rejections"] += 1
                record_trace_event(trace_events, max_trace_events,
                                   ("rejected_zero", candidate, parent, radius))
                continue
            if not point_is_free(candidate, map_info, width, height, blocked):
                stats["invalid_point_rejections"] += 1
                record_trace_event(trace_events, max_trace_events,
                                   ("rejected_invalid", candidate, parent, radius))
                continue
            if not segment_is_free(parent, candidate, map_info, width, height, blocked):
                stats["collision_rejections"] += 1
                record_trace_event(trace_events, max_trace_events,
                                   ("rejected_collision", candidate, parent, radius))
                continue
            record_trace_event(trace_events, max_trace_events,
                               ("local_accepted", candidate, parent, radius))
            return candidate, radius, stats

        radius *= radius_decay
        if radius + 1e-12 >= minimum_radius:
            stats["radius_shrinks"] += 1
            record_trace_event(trace_events, max_trace_events,
                               ("radius_shrink", parent, parent, radius))

    return None, None, stats


def reconstruct_rrt_path(parents, current):
    """Reconstruct a start-to-goal path from RRT parent pointers."""
    path = [current]
    while current in parents:
        current = parents[current]
        path.append(current)
    path.reverse()
    return path


def nearby_nodes(point, nodes, radius):
    """Return all existing tree nodes within radius of point."""
    radius = max(0.0, float(radius))
    return [node for node in nodes if euclidean_points(point, node) <= radius]


def set_parent(child, parent, parents, children):
    """Assign a parent pointer while keeping child adjacency in sync."""
    old_parent = parents.get(child)
    if old_parent is not None and old_parent in children:
        children[old_parent].discard(child)
    parents[child] = parent
    children.setdefault(parent, set()).add(child)
    children.setdefault(child, set())


def update_descendant_costs(root, parents, children, costs):
    """Refresh subtree costs after one node has been rewired."""
    stack = list(children.get(root, ()))
    while stack:
        node = stack.pop()
        parent = parents[node]
        costs[node] = costs[parent] + euclidean_points(parent, node)
        stack.extend(children.get(node, ()))


def tree_edges_from_parents(parents):
    """Build the current tree edge list from parent pointers."""
    return [(parent, child) for child, parent in parents.items()]


def is_ancestor(candidate, node, parents):
    """Return whether candidate is already on node's path to the root."""
    current = node
    while current in parents:
        current = parents[current]
        if current == candidate:
            return True
    return False


def path_length_points(points):
    """Return the geometric length of a continuous path in meters."""
    return sum(euclidean_points(a, b) for a, b in zip(points, points[1:]))


def shortcut_smooth_path(points, map_info, width, height, blocked, iterations=100, rng=None):
    """Remove unnecessary RRT waypoints with continuous collision checks."""
    if rng is None:
        rng = random.Random()
    if len(points) <= 2:
        return list(points)

    smoothed = list(points)
    iterations = max(0, int(iterations))
    for _ in range(iterations):
        if len(smoothed) <= 2:
            break

        first = rng.randrange(0, len(smoothed) - 1)
        second = rng.randrange(first + 1, len(smoothed))
        if second <= first + 1:
            continue

        if segment_is_free(smoothed[first], smoothed[second], map_info, width, height, blocked):
            smoothed = smoothed[: first + 1] + smoothed[second:]

    return smoothed


def sparsify_waypoints(points, map_info, width, height, blocked, min_spacing):
    """Publish fewer global waypoints while preserving collision-free segments."""
    min_spacing = max(0.0, float(min_spacing))
    if min_spacing <= 0.0 or len(points) <= 2:
        return list(points)

    sparse = [points[0]]
    anchor_index = 0
    final_index = len(points) - 1

    while anchor_index < final_index:
        anchor = points[anchor_index]
        selected_index = None

        for candidate_index in range(final_index, anchor_index, -1):
            candidate = points[candidate_index]
            is_goal = candidate_index == final_index
            if not is_goal and euclidean_points(anchor, candidate) < min_spacing:
                continue
            if segment_is_free(anchor, candidate, map_info, width, height, blocked):
                selected_index = candidate_index
                break

        if selected_index is None:
            selected_index = anchor_index + 1

        sparse.append(points[selected_index])
        anchor_index = selected_index

    return sparse


def rrt_to_any_goal(
    start,
    goals,
    map_info,
    width,
    height,
    blocked,
    max_iterations=20000,
    step_size=0.20,
    goal_sample_rate=0.10,
    rng=None,
    goal_tolerance=0.05,
    minimum_sampling_radius=0.05,
    radius_decay=0.5,
    samples_per_radius=6,
    trace_events=None,
    max_trace_events=0,
    rewire_radius=None,
):
    """Grow one continuous-space RRT* tree to the best ordered goal candidate."""
    if rng is None:
        rng = random.Random()
    stats = empty_rrt_stats()

    if not point_is_free(start, map_info, width, height, blocked):
        return [], None, [], stats

    goal_candidates = []
    seen = set()
    for goal in goals:
        key = (round(goal[0], 6), round(goal[1], 6))
        if key in seen:
            continue
        seen.add(key)
        if point_is_free(goal, map_info, width, height, blocked):
            goal_candidates.append(goal)
    if not goal_candidates:
        return [], None, [], stats

    goal_tolerance = max(0.0, float(goal_tolerance))
    step_size = max(1e-3, float(step_size))
    minimum_sampling_radius = min(
        step_size, max(1e-3, float(minimum_sampling_radius))
    )
    radius_decay = min(0.95, max(0.05, float(radius_decay)))
    samples_per_radius = max(1, int(samples_per_radius))
    max_iterations = max(1, int(max_iterations))
    goal_sample_rate = min(1.0, max(0.0, float(goal_sample_rate)))
    if rewire_radius is None:
        rewire_radius = max(step_size * 2.5, map_info.resolution * 2.0)
    rewire_radius = max(map_info.resolution, float(rewire_radius))

    best_index = None
    best_goal = None
    best_path = []
    best_goal_edge = None
    best_goal_parent = None
    best_goal_cost = math.inf

    # If the preferred goal is directly visible, the straight segment is already
    # optimal under Euclidean path length. Lower-priority direct goals are kept
    # only as an initial incumbent while the tree continues seeking better goals.
    for index, goal in enumerate(goal_candidates):
        if euclidean_points(start, goal) <= goal_tolerance:
            best_index = index
            best_goal = goal
            best_path = [start]
            best_goal_cost = 0.0
            break
        stats["goal_connection_checks"] += 1
        if segment_is_free(start, goal, map_info, width, height, blocked):
            cost = euclidean_points(start, goal)
            best_index = index
            best_goal = goal
            best_path = [start, goal]
            best_goal_edge = (start, goal)
            best_goal_cost = cost
            stats["best_cost"] = best_goal_cost
            break
        stats["goal_connection_rejections"] += 1
    if best_index == 0:
        return best_path, best_goal, ([best_goal_edge] if best_goal_edge else []), stats

    nodes = [start]
    node_set = {start}
    parents = {}
    children = {start: set()}
    costs = {start: 0.0}
    spatial_buckets = {}
    bucket_size = max(step_size, map_info.resolution)
    add_spatial_node(spatial_buckets, start, bucket_size)

    for iteration in range(1, max_iterations + 1):
        stats["iterations_used"] = iteration
        stats["global_samples"] += 1
        if best_index is None:
            active_goals = goal_candidates
        else:
            active_goals = goal_candidates[: best_index + 1]
        if rng.random() < goal_sample_rate and active_goals:
            sample = rng.choice(active_goals)
        else:
            sample = random_free_point(map_info, width, height, blocked, rng)
            if sample is None:
                continue
        record_trace_event(trace_events, max_trace_events,
                           ("global_sample", sample, sample, 0.0))

        nearest = (
            min(nodes, key=lambda node: euclidean_points(node, sample))
            if len(nodes) < 256
            else nearest_spatial_node(sample, spatial_buckets, bucket_size)
        )
        new_node, _, extension_stats = adaptive_radius_extension(
            nearest,
            sample,
            map_info,
            width,
            height,
            blocked,
            step_size,
            minimum_sampling_radius,
            radius_decay,
            samples_per_radius,
            rng,
            trace_events,
            max_trace_events,
        )
        for key, value in extension_stats.items():
            stats[key] += value
        if new_node is None:
            continue
        if new_node in node_set:
            stats["duplicate_rejections"] += 1
            continue

        near_nodes = nearby_nodes(new_node, nodes, rewire_radius)
        if nearest not in near_nodes:
            near_nodes.append(nearest)
        stats["parent_candidates"] += len(near_nodes)

        best_parent = nearest
        best_node_cost = costs[nearest] + euclidean_points(nearest, new_node)
        for candidate_parent in near_nodes:
            candidate_cost = costs[candidate_parent] + euclidean_points(candidate_parent, new_node)
            if candidate_cost + 1e-9 >= best_node_cost:
                continue
            if not segment_is_free(candidate_parent, new_node, map_info, width, height, blocked):
                continue
            best_parent = candidate_parent
            best_node_cost = candidate_cost

        nodes.append(new_node)
        node_set.add(new_node)
        costs[new_node] = best_node_cost
        set_parent(new_node, best_parent, parents, children)
        add_spatial_node(spatial_buckets, new_node, bucket_size)
        stats["nodes_added"] += 1
        record_trace_event(trace_events, max_trace_events,
                           ("edge_accepted", new_node, best_parent, 0.0))

        for node in near_nodes:
            if node == best_parent or node == start or node == new_node:
                continue
            if is_ancestor(node, new_node, parents):
                continue
            stats["rewire_checks"] += 1
            rewired_cost = costs[new_node] + euclidean_points(new_node, node)
            if rewired_cost + 1e-9 >= costs[node]:
                continue
            if not segment_is_free(new_node, node, map_info, width, height, blocked):
                continue
            set_parent(node, new_node, parents, children)
            costs[node] = rewired_cost
            update_descendant_costs(node, parents, children, costs)
            stats["rewires"] += 1
            record_trace_event(trace_events, max_trace_events,
                               ("edge_accepted", node, new_node, 0.0))

        # Goal candidates are ordered by preference. RRT* keeps improving the
        # current best candidate instead of returning on the first connection.
        for index, goal in enumerate(goal_candidates):
            if best_index is not None and index > best_index:
                break
            stats["goal_connection_checks"] += 1
            if not segment_is_free(new_node, goal, map_info, width, height, blocked):
                stats["goal_connection_rejections"] += 1
                continue

            path_to_goal = reconstruct_rrt_path(parents, new_node)
            goal_edge = None
            goal_cost = costs[new_node]
            if euclidean_points(path_to_goal[-1], goal) > goal_tolerance:
                path_to_goal.append(goal)
                goal_edge = (new_node, goal)
                goal_cost += euclidean_points(new_node, goal)

            is_better_priority = best_index is None or index < best_index
            is_better_cost = index == best_index and goal_cost + 1e-9 < best_goal_cost
            if is_better_priority or is_better_cost:
                best_index = index
                best_goal = goal
                best_path = path_to_goal
                best_goal_edge = goal_edge
                best_goal_parent = new_node
                best_goal_cost = goal_cost
                stats["best_cost"] = best_goal_cost

    if best_goal_parent is not None and best_goal is not None:
        best_path = reconstruct_rrt_path(parents, best_goal_parent)
        if euclidean_points(best_path[-1], best_goal) > goal_tolerance:
            best_path.append(best_goal)
            best_goal_edge = (best_goal_parent, best_goal)
        else:
            best_goal_edge = None

    edges = tree_edges_from_parents(parents) + ([best_goal_edge] if best_goal_edge else [])
    return best_path, best_goal, edges, stats

def rrt(
    start,
    goal,
    map_info,
    width,
    height,
    blocked,
    max_iterations=20000,
    step_size=0.20,
    goal_sample_rate=0.10,
    rng=None,
    goal_tolerance=0.05,
    minimum_sampling_radius=0.05,
    radius_decay=0.5,
    samples_per_radius=6,
    trace_events=None,
    max_trace_events=0,
    rewire_radius=None,
):
    """Search for a collision-free continuous path to one goal with RRT*."""
    points, _, _, _ = rrt_to_any_goal(
        start,
        [goal],
        map_info,
        width,
        height,
        blocked,
        max_iterations,
        step_size,
        goal_sample_rate,
        rng,
        goal_tolerance,
        minimum_sampling_radius,
        radius_decay,
        samples_per_radius,
        trace_events,
        max_trace_events,
        rewire_radius,
    )
    return points


class RRTGlobalPlanner(Node):
    """ROS node that runs RRT when RViz supplies a new global goal pose."""

    def __init__(self):
        super().__init__("rrt_global_planner")

        # Inputs follow the A* planner defaults. The saved map is the default
        # map source; pose_frame_override lets Gazebo ground truth coordinates be
        # interpreted as map-frame coordinates when that is how the saved map was
        # generated. RViz goals can still be transformed with TF if needed.
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/P1_robot0/odom_gt")
        self.declare_parameter("pose_frame_override", "map")
        self.declare_parameter("goal_pose_topic", "/P1_robot0/goal_pose")

        self.declare_parameter("path_topic", "planned_route")
        self.declare_parameter("pose_array_topic", "planned_route_poses")
        self.declare_parameter("marker_topic", "planned_route_marker")
        self.declare_parameter("tree_marker_topic", "rrt_tree_edges")
        self.declare_parameter("sampling_process_topic", "rrt_sampling_process")

        # Map safety parameters match A*: occupied and unknown cells become
        # blocked, and obstacle inflation keeps the tree away from walls.
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("inflation_radius", 0.30)

        # RRT-specific tuning:
        # - max_iterations bounds how long sampling may continue.
        # - step_size is the initial/maximum local sampling-circle radius.
        # - failed local expansions shrink that radius exponentially.
        # - goal_sample_rate controls how often the global target sample is a goal.
        # - rewire_radius controls the neighborhood used for RRT* parent selection.
        self.declare_parameter("max_iterations", 5000)
        self.declare_parameter("step_size", 0.30)
        self.declare_parameter("rewire_radius", 1.00)
        self.declare_parameter("minimum_sampling_radius", 0.075)
        self.declare_parameter("radius_decay", 0.5)
        self.declare_parameter("samples_per_radius", 3)
        self.declare_parameter("goal_sample_rate", 0.15)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("smooth_path", False)
        self.declare_parameter("smoothing_iterations", 100)
        self.declare_parameter("min_waypoint_spacing", 2.0)
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("adjust_goal_if_blocked", True)
        self.declare_parameter("fallback_on_rrt_timeout", True)
        self.declare_parameter("max_adjusted_goal_candidates", 250)
        self.declare_parameter("adjusted_goal_attempts", 40)
        self.declare_parameter("adjusted_goal_iterations", 8000)
        self.declare_parameter("visualize_tree", True)
        self.declare_parameter("visualize_sampling_process", True)
        self.declare_parameter("replay_interval", 0.03)
        self.declare_parameter("replay_events_per_tick", 20)
        self.declare_parameter("max_visualized_samples", 10000)
        self.declare_parameter("replan_on_map_update", False)
        self.declare_parameter("route_z_offset", 0.05)

        gp = lambda name: self.get_parameter(name).value
        self.map_topic = gp("map_topic")
        self.pose_topic = gp("pose_topic")
        self.pose_frame_override = str(gp("pose_frame_override")).strip()
        self.goal_pose_topic = gp("goal_pose_topic")
        self.path_topic = gp("path_topic")
        self.pose_array_topic = gp("pose_array_topic")
        self.marker_topic = gp("marker_topic")
        self.tree_marker_topic = gp("tree_marker_topic")
        self.sampling_process_topic = gp("sampling_process_topic")
        self.occupied_threshold = int(gp("occupied_threshold"))
        self.allow_unknown = parameter_as_bool(gp("allow_unknown"))
        self.inflation_radius = max(0.0, float(gp("inflation_radius")))
        self.max_iterations = max(1, int(gp("max_iterations")))
        self.step_size = max(1e-3, float(gp("step_size")))
        self.rewire_radius = max(self.step_size, float(gp("rewire_radius")))
        self.minimum_sampling_radius = min(
            self.step_size, max(1e-3, float(gp("minimum_sampling_radius")))
        )
        self.radius_decay = min(0.95, max(0.05, float(gp("radius_decay"))))
        self.samples_per_radius = max(1, int(gp("samples_per_radius")))
        self.goal_sample_rate = min(1.0, max(0.0, float(gp("goal_sample_rate"))))
        self.goal_tolerance = max(0.0, float(gp("goal_tolerance")))
        self.smooth_path = parameter_as_bool(gp("smooth_path"))
        self.smoothing_iterations = max(0, int(gp("smoothing_iterations")))
        self.min_waypoint_spacing = max(0.0, float(gp("min_waypoint_spacing")))
        self.random_seed = int(gp("random_seed"))
        self.adjust_goal_if_blocked = parameter_as_bool(gp("adjust_goal_if_blocked"))
        self.fallback_on_rrt_timeout = parameter_as_bool(gp("fallback_on_rrt_timeout"))
        self.max_adjusted_goal_candidates = max(1, int(gp("max_adjusted_goal_candidates")))
        self.adjusted_goal_attempts = max(1, int(gp("adjusted_goal_attempts")))
        self.adjusted_goal_iterations = max(1, int(gp("adjusted_goal_iterations")))
        self.visualize_tree = parameter_as_bool(gp("visualize_tree"))
        self.visualize_sampling_process = parameter_as_bool(gp("visualize_sampling_process"))
        self.replay_interval = max(0.001, float(gp("replay_interval")))
        self.replay_events_per_tick = max(1, int(gp("replay_events_per_tick")))
        self.max_visualized_samples = max(1, int(gp("max_visualized_samples")))
        self.replan_on_map_update = parameter_as_bool(gp("replan_on_map_update"))
        self.route_z_offset = float(gp("route_z_offset"))

        # Cache the latest ROS inputs. A goal may arrive before map or robot
        # pose, so plan() can defer work until all required inputs are available.
        self.map_msg = None
        self.robot_pose = None
        self.goal_pose = None
        self.waiting_for_inputs = False

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # TF is used only for goal poses whose frame differs from the map frame.
        # Robot pose follows the same pose_frame_override behavior as A*.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(Odometry, self.pose_topic, self.on_pose, 10)
        self.create_subscription(PoseStamped, self.goal_pose_topic, self.on_goal_pose, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.pose_array_pub = self.create_publisher(PoseArray, self.pose_array_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.tree_marker_pub = self.create_publisher(Marker, self.tree_marker_topic, 10)
        self.sampling_process_pub = self.create_publisher(
            MarkerArray, self.sampling_process_topic, 10
        )
        self.replay_timer = None
        self.replay_events = []
        self.replay_index = 0
        self.replay_frame_id = "map"
        self.replay_global_points = []
        self.replay_accepted_points = []
        self.replay_collision_points = []
        self.replay_invalid_points = []
        self.replay_edges = []
        self.replay_radius_center = None
        self.replay_radius = 0.0

        pose_frame_note = (
            f" interpreted as frame '{self.pose_frame_override}'"
            if self.pose_frame_override else ""
        )
        self.get_logger().info(
            f"Ready. Waiting for map '{self.map_topic}', pose '{self.pose_topic}'"
            f"{pose_frame_note}, and RViz goals on '{self.goal_pose_topic}'."
        )

    def on_map(self, msg):
        """Cache a new occupancy grid and optionally recalculate the route."""
        self.map_msg = msg
        if self.goal_pose is not None and (self.waiting_for_inputs or self.replan_on_map_update):
            self.plan()

    def on_pose(self, msg):
        """Cache the newest robot pose and plan if an earlier goal is waiting."""
        self.robot_pose = msg
        if self.goal_pose is not None and self.waiting_for_inputs:
            self.plan()

    def on_goal_pose(self, msg):
        """Store a newly selected RViz goal and calculate an RRT route."""
        self.goal_pose = msg
        self.plan()

    def plan(self):
        """Validate inputs, execute RRT, and publish the resulting route."""
        # plan() is the ROS-to-algorithm bridge: validate cached inputs, convert
        # map-frame metric coordinates into grid cells, build blocked cells,
        # run RRT*, then convert the result back to ROS path messages.
        missing = []
        if self.map_msg is None:
            missing.append("map")
        if self.robot_pose is None:
            missing.append("robot pose")
        if missing:
            self.waiting_for_inputs = True
            self.get_logger().warning(f"Goal stored; waiting for {' and '.join(missing)}.")
            return

        self.waiting_for_inputs = False
        self.stop_sampling_replay(clear_markers=True)
        map_frame = self.map_msg.header.frame_id or "map"
        raw_pose_frame = self.robot_pose.header.frame_id
        pose_frame = self.pose_frame_override or raw_pose_frame
        goal_frame = self.goal_pose.header.frame_id

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

        # RViz goals can be published in any available frame. If the goal is not
        # already in the map frame, transform its position into map coordinates
        # before converting it to a grid cell.
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

        # RRT samples continuous map-frame points. Grid cells are used only for
        # occupancy/collision checks and for adjusted-goal candidate generation.
        start = (start_position.x, start_position.y)
        goal = (goal_position.x, goal_position.y)
        start_cell = world_to_grid(start_position.x, start_position.y, info)
        goal_cell = world_to_grid(goal_position.x, goal_position.y, info)

        # Build the same obstacle model as A*: occupied cells, unknown cells when
        # allow_unknown is false, plus inflated obstacle margins.
        inflation_cells = math.ceil(self.inflation_radius / info.resolution)
        blocked = make_blocked_cells(
            self.map_msg.data,
            info.width,
            info.height,
            self.occupied_threshold,
            self.allow_unknown,
            inflation_cells,
        )
        planned_goal = goal
        adjusted_goal = None
        goal_is_free = point_is_free(goal, info, info.width, info.height, blocked)
        planning_start = time.perf_counter()
        trace_events = [] if self.visualize_sampling_process else None
        search_rng = random.Random(self.random_seed if self.random_seed else None)
        smoothing_seed = self.random_seed + 1 if self.random_seed else None
        fallback_seed = self.random_seed + 2 if self.random_seed else None
        smoothing_rng = random.Random(smoothing_seed)
        fallback_rng = random.Random(fallback_seed)
        route_points, _, tree_edges, main_stats = rrt_to_any_goal(
            start,
            [planned_goal],
            info,
            info.width,
            info.height,
            blocked,
            self.max_iterations,
            self.step_size,
            self.goal_sample_rate,
            search_rng,
            self.goal_tolerance,
            self.minimum_sampling_radius,
            self.radius_decay,
            self.samples_per_radius,
            trace_events,
            self.max_visualized_samples,
            self.rewire_radius,
        )
        used_fallback = False
        fallback_stats = empty_rrt_stats()
        can_use_adjusted_goal = self.adjust_goal_if_blocked and (
            not goal_is_free or self.fallback_on_rrt_timeout
        )
        if not route_points and can_use_adjusted_goal:
            reachable_cells, _, _ = reachable_paths_from_start(
                start_cell, info.width, info.height, blocked, use_diagonal=True
            )
            candidate_cells = []
            checked_candidates = 0
            for candidate_cell in free_goal_candidates_toward_start(
                start_cell, goal_cell, info.width, info.height, blocked
            ):
                checked_candidates += 1
                if candidate_cell in reachable_cells:
                    candidate_cells.append(candidate_cell)
                    if len(candidate_cells) >= self.adjusted_goal_attempts:
                        break
                if checked_candidates >= self.max_adjusted_goal_candidates:
                    break

            candidate_goals = [grid_to_world_point(cell, info) for cell in candidate_cells]
            candidate_points = []
            candidate_edges = []
            reached_goal = None
            if candidate_goals:
                candidate_points, reached_goal, candidate_edges, fallback_stats = rrt_to_any_goal(
                    start,
                    candidate_goals,
                    info,
                    info.width,
                    info.height,
                    blocked,
                    self.adjusted_goal_iterations,
                    self.step_size,
                    self.goal_sample_rate,
                    fallback_rng,
                    self.goal_tolerance,
                    self.minimum_sampling_radius,
                    self.radius_decay,
                    self.samples_per_radius,
                    trace_events,
                    self.max_visualized_samples,
                    self.rewire_radius,
                )
                tree_edges = candidate_edges

            if candidate_points and reached_goal is not None:
                reached_index = candidate_goals.index(reached_goal)
                adjusted_goal_cell = candidate_cells[reached_index]
                route_points = candidate_points
                adjusted_goal = reached_goal
                planned_goal = reached_goal
                used_fallback = True
                self.get_logger().info(
                    f"Adjusted goal fallback reached candidate {reached_index + 1}/"
                    f"{len(candidate_cells)} at cell {adjusted_goal_cell} using one shared RRT tree."
                )

            if not route_points:
                if candidate_cells:
                    self.get_logger().warn(
                        f"Tried one shared RRT tree for {len(candidate_cells)} reachable adjusted goal candidates "
                        f"from {checked_candidates} checked cells, but RRT did not connect. "
                        "Increase adjusted_goal_iterations, adjusted_goal_attempts, or reduce step_size."
                    )
                else:
                    self.get_logger().warn(
                        f"No reachable adjusted goal found among {checked_candidates} candidates. "
                        "Increase max_adjusted_goal_candidates or check map/inflation settings."
                    )
        elif not route_points and self.adjust_goal_if_blocked and goal_is_free:
            self.get_logger().warn(
                "Original RRT goal is free but was not reached within max_iterations; "
                "not switching to an adjusted goal because fallback_on_rrt_timeout is false. "
                "Increase max_iterations/goal_sample_rate or set fallback_on_rrt_timeout:=true "
                "if you want timeout failures to use adjusted goals."
            )

        planning_time_ms = (time.perf_counter() - planning_start) * 1000.0
        diagnostic_note = format_rrt_stats("main", main_stats)
        if fallback_stats["iterations_used"] or fallback_stats["goal_connection_checks"]:
            diagnostic_note += ", " + format_rrt_stats("fallback", fallback_stats)
        if self.visualize_sampling_process:
            self.start_sampling_replay(trace_events or [], map_frame)
        if self.visualize_tree:
            self.publish_tree_edges(tree_edges, map_frame, delete_marker=False)

        if not route_points:
            self.get_logger().warn(f"RRT diagnostics: {diagnostic_note}")
            self.fail_plan(
                f"No RRT route found from start {start} to goal {goal}. "
                "Try increasing max_iterations, reducing inflation_radius, or checking blocked/unknown cells."
            )
            return

        raw_count = len(route_points)
        raw_length = path_length_points(route_points)
        if self.smooth_path:
            route_points = shortcut_smooth_path(
                route_points,
                info,
                info.width,
                info.height,
                blocked,
                self.smoothing_iterations,
                smoothing_rng,
            )

        smoothed_count = len(route_points)
        smoothed_length = path_length_points(route_points)
        route_points = sparsify_waypoints(
            route_points,
            info,
            info.width,
            info.height,
            blocked,
            self.min_waypoint_spacing,
        )
        published_length = path_length_points(route_points)

        points = [Point(x=x, y=y, z=self.route_z_offset) for x, y in route_points]
        self.publish_route(points, map_frame)
        smoothing_state = "on" if self.smooth_path else "off"
        goal_note = (
            f"adjusted_goal={planned_goal} original_goal={goal}, "
            if adjusted_goal is not None and planned_goal == adjusted_goal
            else ""
        )
        self.get_logger().info(
            f"Published continuous-space RRT route from {start} to {planned_goal}: "
            f"{goal_note}"
            f"fallback={used_fallback}, planning_time={planning_time_ms:.2f} ms, "
            f"tree_edges={len(tree_edges)}, adaptive_radius={self.step_size:.2f}->"
            f"{self.minimum_sampling_radius:.2f} m, rewire_radius={self.rewire_radius:.2f} m, "
            f"radius_decay={self.radius_decay:.2f}, "
            f"samples_per_radius={self.samples_per_radius}, goal_tolerance={self.goal_tolerance:.2f} m, "
            f"random_seed={self.random_seed}, replay_events={len(trace_events or [])}, "
            f"{diagnostic_note}, "
            f"smoothing={smoothing_state}, iterations={self.smoothing_iterations}, "
            f"min_waypoint_spacing={self.min_waypoint_spacing:.2f} m, "
            f"raw_poses={raw_count}, raw_length={raw_length:.2f} m, "
            f"smoothed_poses={smoothed_count}, smoothed_length={smoothed_length:.2f} m, "
            f"published_poses={len(route_points)}, published_length={published_length:.2f} m, "
            f"inflation_radius={self.inflation_radius:.2f} m."
        )

    def fail_plan(self, message):
        """Report a planning failure and clear any previously published route."""
        self.get_logger().error(message)
        frame_id = self.map_msg.header.frame_id if self.map_msg is not None else "map"
        self.publish_route([], frame_id or "map", delete_marker=True)
        if self.visualize_tree:
            self.publish_tree_edges([], frame_id or "map", delete_marker=True)

    def publish_route(self, points, frame_id, delete_marker=False):
        """Publish an RRT route as Path, PoseArray, and RViz LINE_STRIP messages."""
        # Keep the output contract identical to the A* and hand-drawn planners:
        # Path for navigation consumers, PoseArray for simple waypoint consumers,
        # and Marker for RViz visualization.
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame_id
        pose_array = PoseArray()
        pose_array.header = path.header

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
        marker.ns = "rrt_route"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE if delete_marker else Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.11
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.15
        marker.color.a = 1.0
        marker.points = points
        self.marker_pub.publish(marker)

    def stop_sampling_replay(self, clear_markers=False):
        """Stop an active replay and optionally remove its RViz markers."""
        if self.replay_timer is not None:
            self.replay_timer.cancel()
            self.destroy_timer(self.replay_timer)
            self.replay_timer = None
        if clear_markers:
            marker = Marker()
            marker.action = Marker.DELETEALL
            self.sampling_process_pub.publish(MarkerArray(markers=[marker]))

    def start_sampling_replay(self, events, frame_id):
        """Replay recorded RRT sampling events without affecting planning time."""
        self.stop_sampling_replay(clear_markers=True)
        self.replay_events = list(events)
        self.replay_index = 0
        self.replay_frame_id = frame_id
        self.replay_global_points = []
        self.replay_accepted_points = []
        self.replay_collision_points = []
        self.replay_invalid_points = []
        self.replay_edges = []
        self.replay_radius_center = None
        self.replay_radius = 0.0
        if not self.replay_events:
            return
        self.replay_timer = self.create_timer(
            self.replay_interval, self.on_sampling_replay_tick
        )
        self.get_logger().info(
            f"Replaying {len(self.replay_events)} RRT sampling events on "
            f"\x27{self.sampling_process_topic}\x27."
        )

    def on_sampling_replay_tick(self):
        """Advance the recorded RRT process by one display batch."""
        end = min(
            len(self.replay_events),
            self.replay_index + self.replay_events_per_tick,
        )
        for event_type, point, parent, radius in self.replay_events[self.replay_index:end]:
            if event_type == "global_sample":
                self.replay_global_points.append(point)
            elif event_type == "edge_accepted":
                self.replay_accepted_points.append(point)
                self.replay_edges.append((parent, point))
            elif event_type == "rejected_collision":
                self.replay_collision_points.append(point)
            elif event_type.startswith("rejected_"):
                self.replay_invalid_points.append(point)

            if event_type in (
                "local_accepted",
                "rejected_collision",
                "rejected_invalid",
                "rejected_zero",
                "radius_shrink",
            ):
                self.replay_radius_center = parent
                self.replay_radius = radius

        self.replay_index = end
        self.publish_sampling_replay_state()
        if self.replay_index >= len(self.replay_events):
            if self.replay_timer is not None:
                self.replay_timer.cancel()
            self.get_logger().info("RRT sampling-process replay finished.")

    def sampling_marker(self, marker_id, namespace, marker_type, scale, color, points):
        """Create one consistently styled marker for sampling-process replay."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.replay_frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [
            Point(x=point[0], y=point[1], z=self.route_z_offset * 0.6)
            for point in points
        ]
        return marker

    def publish_sampling_replay_state(self):
        """Publish accumulated replay points, accepted edges, and current radius."""
        markers = [
            self.sampling_marker(
                0, "rrt_global_samples", Marker.POINTS, 0.055,
                (0.35, 0.10, 1.0, 1.0), self.replay_global_points,
            ),
            self.sampling_marker(
                1, "rrt_accepted_nodes", Marker.POINTS, 0.065,
                (1.0, 0.05, 0.65, 1.0), self.replay_accepted_points,
            ),
            self.sampling_marker(
                2, "rrt_collision_rejections", Marker.POINTS, 0.060,
                (1.0, 0.0, 0.0, 0.90), self.replay_collision_points,
            ),
            self.sampling_marker(
                3, "rrt_invalid_rejections", Marker.POINTS, 0.055,
                (1.0, 0.45, 0.0, 0.85), self.replay_invalid_points,
            ),
        ]

        edge_points = []
        for first, second in self.replay_edges:
            edge_points.extend((first, second))
        markers.append(
            self.sampling_marker(
                4, "rrt_replay_tree", Marker.LINE_LIST, 0.025,
                (0.0, 0.9, 1.0, 0.80), edge_points,
            )
        )

        radius_points = []
        if self.replay_radius_center is not None and self.replay_radius > 0.0:
            segments = 48
            for index in range(segments + 1):
                angle = 2.0 * math.pi * index / segments
                radius_points.append((
                    self.replay_radius_center[0] + self.replay_radius * math.cos(angle),
                    self.replay_radius_center[1] + self.replay_radius * math.sin(angle),
                ))
        markers.append(
            self.sampling_marker(
                5, "rrt_sampling_radius", Marker.LINE_STRIP, 0.025,
                (0.2, 0.45, 1.0, 0.95), radius_points,
            )
        )
        self.sampling_process_pub.publish(MarkerArray(markers=markers))

    def publish_tree_edges(self, edges, frame_id, delete_marker=False):
        """Publish RRT tree edges as RViz LINE_LIST for process visualization."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.DELETE if delete_marker else Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 0.90
        for first, second in edges:
            marker.points.append(Point(x=first[0], y=first[1], z=self.route_z_offset * 0.8))
            marker.points.append(Point(x=second[0], y=second[1], z=self.route_z_offset * 0.8))
        self.tree_marker_pub.publish(marker)

    @staticmethod
    def orientation_for_point(points, index):
        """Derive the planar heading of a route point from its neighbors."""
        if len(points) < 2:
            return Quaternion(w=1.0)
        current = points[index]
        target = points[index + 1] if index < len(points) - 1 else points[index - 1]
        yaw = math.atan2(target.y - current.y, target.x - current.x)
        if index == len(points) - 1:
            yaw += math.pi
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main(args=None):
    """Start the planner and keep processing ROS callbacks until shutdown."""
    rclpy.init(args=args)
    node = RRTGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
