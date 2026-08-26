"""Publish a route assembled manually from waypoints selected in RViz.

This node is intentionally simple: it does not inspect the occupancy map and it
never calculates a collision-free path. Instead, it collects points selected by
the user and republishes them in several ROS message formats. This makes it a
useful visualization tool and a convenient source of test routes for downstream
controllers.
"""

import math

import rclpy
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
    Quaternion,
)
from nav_msgs.msg import Path
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a 3D vector by a geometry_msgs Quaternion."""
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w
    vx, vy, vz = vector

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


class RvizRoutePlanner(Node):
    """Collect RViz waypoints and publish them as a hand-drawn route."""

    def __init__(self):
        """Configure ROS interfaces and initialize the in-memory waypoint list."""
        super().__init__("p03_rviz_route_planner")

        # Keep all topic names configurable so the node can be reused under a
        # namespace or connected to a different RViz configuration without code
        # changes. Relative topic names follow the node namespace automatically.
        self.declare_parameter("clicked_point_topic", "clicked_point")
        self.declare_parameter("path_topic", "planned_route")
        self.declare_parameter("pose_array_topic", "planned_route_poses")
        self.declare_parameter("marker_topic", "planned_route_marker")
        self.declare_parameter("default_frame_id", "map")

        # Ignore almost identical consecutive clicks. Besides making the route
        # cleaner, this avoids publishing zero-length path segments that do not
        # provide a useful heading for downstream consumers.
        self.declare_parameter("min_point_distance", 0.15)

        # RViz can render a line exactly on the map plane poorly. Lift the route
        # marker slightly above z=0 so that it remains clearly visible.
        self.declare_parameter("route_z_offset", 0.05)

        # Resolve parameters once during startup. The current node does not
        # support changing them dynamically while it is running.
        self.clicked_point_topic = self.get_parameter("clicked_point_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.pose_array_topic = self.get_parameter("pose_array_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.default_frame_id = self.get_parameter("default_frame_id").value
        self.min_point_distance = float(self.get_parameter("min_point_distance").value)
        self.route_z_offset = float(self.get_parameter("route_z_offset").value)

        # Waypoints are stored in click order and published in the configured
        # map frame so they overlay the saved static map consistently.
        self.points = []
        self.frame_id = self.default_frame_id

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # "Publish Point" in RViz emits PointStamped messages. Each click is
        # stored as the next waypoint in the hand-drawn route.
        self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self.on_clicked_point,
            10,
        )

        # Publish the same logical route in three forms:
        # - Path is the conventional navigation representation.
        # - PoseArray is convenient for simple waypoint consumers.
        # - Marker draws a clearly visible line in RViz.
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.pose_array_pub = self.create_publisher(PoseArray, self.pose_array_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        # Services provide lightweight editing without restarting the node.
        self.create_service(Trigger, "clear_route", self.clear_route)
        self.create_service(Trigger, "undo_route_point", self.undo_route_point)

        self.get_logger().info(
            "Ready. In RViz use 'Publish Point' to add waypoints. "
            f"Publishing route on '{self.path_topic}'."
        )

    def on_clicked_point(self, msg):
        """Convert a PointStamped RViz click into a route waypoint."""
        point = self.point_in_map_frame(msg.point, msg.header.frame_id)
        if point is None:
            return
        self.add_waypoint(self.make_point(point.x, point.y, point.z))

    def point_in_map_frame(self, point, source_frame):
        """Return a point expressed in the configured map frame."""
        frame_id = source_frame or self.default_frame_id
        if frame_id == self.default_frame_id:
            return point

        try:
            transform = self.tf_buffer.lookup_transform(
                self.default_frame_id.lstrip("/"), frame_id.lstrip("/"), rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().error(
                f"Point frame '{frame_id}' does not match route frame "
                f"'{self.default_frame_id}', and TF transform failed: {exc}"
            )
            return None

        self.get_logger().warn(
            f"Transformed waypoint from frame '{frame_id}' to "
            f"'{self.default_frame_id}'.",
            throttle_duration_sec=5.0,
        )
        return transform_point(point, transform)

    def add_waypoint(self, point):
        """Append a sufficiently distinct waypoint and republish the route."""
        if (
            self.points
            and self.distance(self.points[-1], point) < self.min_point_distance
        ):
            self.get_logger().debug("Ignoring clicked point too close to previous point.")
            return

        self.frame_id = self.default_frame_id
        self.points.append(point)
        self.publish_route()

        self.get_logger().info(
            f"Added waypoint {len(self.points)}: "
            f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f}) in frame '{self.frame_id}'"
        )

    def make_point(self, x, y, z):
        """Create a waypoint with a small visualization offset above the map."""
        return Point(x=x, y=y, z=z + self.route_z_offset)

    def clear_route(self, request, response):
        """Remove every stored waypoint and delete the RViz marker."""
        del request
        self.points.clear()

        # Publishing empty Path and PoseArray messages informs route consumers
        # that the previous route is no longer valid. DELETE explicitly removes
        # the previous LINE_STRIP marker from RViz.
        self.publish_route(delete_marker=True)

        response.success = True
        response.message = "Route cleared."
        self.get_logger().info(response.message)
        return response

    def undo_route_point(self, request, response):
        """Remove only the most recently added waypoint, if one exists."""
        del request
        if not self.points:
            response.success = False
            response.message = "Route has no points to remove."
            return response

        removed = self.points.pop()

        # If the final point has been removed, delete the old marker instead of
        # publishing a zero-point LINE_STRIP that may leave stale graphics.
        self.publish_route(delete_marker=not self.points)

        response.success = True
        response.message = (
            f"Removed waypoint ({removed.x:.2f}, {removed.y:.2f}, {removed.z:.2f})."
        )
        self.get_logger().info(response.message)
        return response

    def publish_route(self, delete_marker=False):
        """Publish the current waypoint list in all supported ROS formats."""
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id

        pose_array = PoseArray()
        pose_array.header = path.header

        # A Path stores stamped poses, while a PoseArray stores bare poses. Use
        # the same Pose data in both outputs so all consumers see an identical
        # route and derived heading at each waypoint.
        for index, point in enumerate(self.points):
            pose = Pose()
            pose.position = point
            pose.orientation = self.orientation_for_point(index)

            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose = pose

            path.poses.append(pose_stamped)
            pose_array.poses.append(pose)

        self.path_pub.publish(path)
        self.pose_array_pub.publish(pose_array)
        self.marker_pub.publish(self.make_route_marker(stamp, delete_marker))

    def make_route_marker(self, stamp, delete_marker=False):
        """Build the RViz LINE_STRIP marker used to visualize the route."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "hand_drawn_route"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE if delete_marker else Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08

        # Use a bright cyan line so the manually selected route is easy to
        # distinguish from the orange route published by the A* planner.
        marker.color.r = 0.1
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = list(self.points)
        return marker

    def orientation_for_point(self, index):
        """Derive a planar heading quaternion from adjacent route waypoints."""
        if len(self.points) < 2:
            # A single point has no path direction. The identity quaternion is
            # a valid neutral orientation for this special case.
            return Quaternion(w=1.0)

        current = self.points[index]
        if index < len(self.points) - 1:
            # For normal points, face toward the next waypoint.
            target = self.points[index + 1]
        else:
            # The last point has no successor, so infer the same forward
            # direction by looking backward and correcting the angle below.
            target = self.points[index - 1]

        yaw = math.atan2(target.y - current.y, target.x - current.x)
        if index == len(self.points) - 1:
            yaw += math.pi

        # A planar yaw rotation has only z and w quaternion components.
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    @staticmethod
    def distance(first, second):
        """Return the Euclidean distance between two geometry_msgs Points."""
        return math.sqrt(
            (first.x - second.x) ** 2
            + (first.y - second.y) ** 2
            + (first.z - second.z) ** 2
        )


def main(args=None):
    """Start the node and keep processing ROS callbacks until shutdown."""
    rclpy.init(args=args)
    node = RvizRoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
