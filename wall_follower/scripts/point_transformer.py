#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2024.10.14

# ROS 2 program to subscribe to update a MarkerArray subscribing to 
# PointStamed topic, assuming it's published by a vision node.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import csv
import math

from wall_follower.landmark import marker_type, max_markers, Landmark





class PointTransformer(Node):

	def __init__(self):
		super().__init__('point_transformer')
		
   		# 记录起点位姿（start_x, start_y, heading），用 map->base_footprint 的 TF
        self.start_heading = None
        self.start_x = None
        
		self.start_y = None

        # 周期性尝试记录起点（刚开始 SLAM 还没收敛可能会失败）
        self.start_pose_timer = self.create_timer(0.5, self.record_start_pose)

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.point_subscriber = self.create_subscription(PointStamped, '/marker_position', self.point_callback, 10)
		self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

		self.marker_position = []
		self.marker_array = MarkerArray()			
		self.marker_array.markers = []
		

		for i in range(max_markers):
			self.marker_position.append(Landmark(i, self.marker_array.markers))
   
        
	def record_start_pose(self):
            """从 TF 中记录起点位姿，只记录一次。"""
        if self.start_x is not None:
            # 已经记录过了
            return

        try:
            # 从 map 到 base_footprint 的变换（也可以用 base_link，看你 TF 树）
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception as e:
            # SLAM 还没准备好，过一会儿 timer 会再试
            # self.get_logger().warn(f"Waiting for start pose TF: {e}")
            return

        # 平移
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        # 旋转（四元数 -> yaw）
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.start_x = tx
        self.start_y = ty
        self.start_heading = yaw

        self.get_logger().info(
            f"Recorded start pose: x={self.start_x:.3f}, y={self.start_y:.3f}, heading={self.start_heading:.3f}"
        )

	    def saveLandmarks(self):
            """按老师要求的格式保存到 markers.txt。"""
        if self.start_x is None:
            self.get_logger().warn(
                "Start pose not recorded, using (0,0,0) as fallback in markers.txt"
            )
            sx, sy, sh = 0.0, 0.0, 0.0
        else:
            sx, sy, sh = self.start_x, self.start_y, self.start_heading

        filename = 'markers.txt'
        self.get_logger().info(f"Saving markers to {filename}")

        with open(filename, 'w') as f:
            # 第一行：start_x, start_y, start_heading
            f.write(f"{sx:.6f}, {sy:.6f}, {sh:.6f}\n")

            # 后面每一行：marker_x, marker_y, marker_type（字符串）
            for idx, lm in enumerate(self.marker_position):
                if lm.top_marker is None:
                    continue

                # Landmark 里有一个 RViz 的 Marker 存在 top_marker
                x = lm.top_marker.pose.position.x
                y = lm.top_marker.pose.position.y

                # marker_type 来自 wall_follower.landmark.marker_type
                # 比如 ["pink_green", "blue_yellow", ...]
                if idx < len(marker_type):
                    type_str = marker_type[idx]
                else:
                    type_str = str(idx)

                f.write(f"{x:.6f}, {y:.6f}, {type_str}\n")

        self.get_logger().info("Markers saved, shutting down.")



	def point_callback(self, msg):
		try:
			# Lookup the transform from the camera_rgb_optical_frame to the map frame
			transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
		except tf2_ros.LookupException as e:
			self.get_logger().error('Transform lookup failed: %s' % str(e))
			return

		which_marker = int(msg.point.z)
		m = marker_type[which_marker]
		msg.point.z = 0.0

		# Transform the point from camera_rgb_optical_frame to map frame
		map_point = tf2_geometry_msgs.do_transform_point(msg, transform)

		# Print the transformed point in the map frame
#		self.get_logger().info(f'Mapped {m} marker to /map frame: x={map_point.point.x}, y={map_point.point.y}, z={map_point.point.z}')

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

