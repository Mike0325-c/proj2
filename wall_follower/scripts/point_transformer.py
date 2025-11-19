#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
import math

from wall_follower.landmark import marker_type, max_markers, Landmark


class PointTransformer(Node):

    def __init__(self):
        super().__init__('point_transformer')

        self.start_x = None
        self.start_y = None
        self.start_heading = None

        self.start_pose_timer = self.create_timer(0.5, self.record_start_pose)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.point_subscriber = self.create_subscription(
            PointStamped,
            '/marker_position',
            self.point_callback,
            10
        )

        self.marker_publisher_ = self.create_publisher(
            MarkerArray,
            'visualization_marker_array',
            10
        )

        self.marker_array = MarkerArray()
        self.marker_array.markers = []
        self.marker_position = []

        for i in range(max_markers):
            self.marker_position.append(Landmark(i, self.marker_array.markers))

    def record_start_pose(self):
        if self.start_x is not None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception:
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.start_x = tx
        self.start_y = ty
        self.start_heading = yaw

        self.get_logger().info(
            f"Recorded start pose: x={tx:.3f}, y={ty:.3f}, heading={yaw:.3f}"
        )

    def saveLandmarks(self):
        if self.start_x is None:
            sx, sy, sh = 0.0, 0.0, 0.0
        else:
            sx, sy, sh = self.start_x, self.start_y, self.start_heading

        filename = 'markers.txt'
        self.get_logger().info(f"Saving markers to {filename}")

        with open(filename, 'w') as f:
            f.write(f"{sx:.6f}, {sy:.6f}, {sh:.6f}\n")

            for idx, lm in enumerate(self.marker_position):
                if lm.top_marker is None:
                    continue

                x = lm.top_marker.pose.position.x
                y = lm.top_marker.pose.position.y

                if idx < len(marker_type):
                    type_str = marker_type[idx]
                else:
                    type_str = str(idx)

                f.write(f"{x:.6f}, {y:.6f}, {type_str}\n")

        self.get_logger().info("Markers saved successfully.")

    def point_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return

        which_marker = int(msg.point.z)
        msg.point.z = 0.0

        map_point = tf2_geometry_msgs.do_transform_point(msg, transform)

        self.marker_position[which_marker].update_position(map_point.point)
        self.marker_publisher_.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.saveLandmarks()
        exit()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
