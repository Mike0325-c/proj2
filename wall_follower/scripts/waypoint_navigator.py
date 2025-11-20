#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import csv
import math
import os
import time

CSV_PATH = "/home/pi/turtlebot3_ws/src/wall_follower/landmarks.csv"
MAP_FRAME = "map"


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("waypoint_navigator started")
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        points = []
        if not os.path.exists(CSV_PATH):
            self.get_logger().error(f"CSV file not found: {CSV_PATH}")
            return points

        with open(CSV_PATH, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                except ValueError:
                    # 跳过可能的表头等非数字行
                    self.get_logger().warn(f"Skip invalid row in csv: {row}")
                    continue
                points.append((x, y))

        self.get_logger().info(f"Loaded {len(points)} waypoints from csv")
        return points

    def make_goal(self, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # yaw → quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose
        return goal

    def go_all(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded, nothing to do.")
            return

        # 等 nav2 action server ready（阻塞等就行）
        self.get_logger().info("Waiting for Nav2 action server...")
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Nav2 action server not available, abort.")
            return
        self.get_logger().info("Nav2 action server ready, start navigating...")

        for i, (x, y) in enumerate(self.waypoints):
            self.get_logger().info(f"Going to waypoint {i+1}: ({x:.2f}, {y:.2f})")

            goal_msg = self.make_goal(x, y, 0.0)
            send_future = self.client.send_goal_async(goal_msg)

            # 等待服务器接收 goal（这里会自己帮你 spin）
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f"Goal {i+1} rejected")
                continue

            self.get_logger().info(f"Goal {i+1} accepted, waiting for result...")

            # 等导航结束
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result is None:
                self.get_logger().warn(f"Goal {i+1} result is None (maybe canceled/aborted)")
            else:
                self.get_logger().info(
                    f"Reached waypoint {i+1}, status: {result.status}"
                )

            # 稍微停一下再去下一个点
            time.sleep(1.0)

        self.get_logger().info("Finished all waypoints.")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        node.go_all()   # 这里会自己 spin_until_future_complete 多次
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
