#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class StaticFootprintPublisher(Node):
    """Publish a fixed navigation footprint independently of Nav2 costmaps."""

    def __init__(self):
        super().__init__("static_footprint_publisher")

        self.declare_parameter("topic", "/g1_nav/static_footprint")
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter(
            "points",
            [0.23, 0.38, 0.23, -0.38, -0.23, -0.38, -0.23, 0.38],
        )

        self._topic = self.get_parameter("topic").value
        self._frame_id = self.get_parameter("frame_id").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        coordinates = list(self.get_parameter("points").value)

        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if len(coordinates) < 6 or len(coordinates) % 2 != 0:
            raise ValueError("points must contain at least three x/y coordinate pairs")

        self._points = []
        for index in range(0, len(coordinates), 2):
            point = Point32()
            point.x = float(coordinates[index])
            point.y = float(coordinates[index + 1])
            self._points.append(point)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(PolygonStamped, self._topic, qos)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

        self._publish()
        self.get_logger().info(
            f"Publishing fixed footprint on {self._topic} in {self._frame_id} "
            f"at {publish_rate:.1f} Hz"
        )

    def _publish(self):
        message = PolygonStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.polygon.points = self._points
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = StaticFootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
