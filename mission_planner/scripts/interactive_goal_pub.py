#!/usr/bin/env python
"""
Interactive Goal Publisher for SUPER Planner

Lets you send 3D goals to the drone in real-time by typing coordinates.
The drone will plan and fly to each goal as you enter it.

Usage:
  rosrun mission_planner interactive_goal_pub.py

Commands:
  x y z         - Send goal to (x, y, z)
  x y z yaw     - Send goal with yaw (degrees)
  status        - Print current drone position
  help          - Show this help
  quit / q      - Exit

Examples:
  0 0 10        - Fly to center at 10m height
  -20 30 40     - Fly to (-20, 30, 40)
  10 0 5 90     - Fly to (10, 0, 5) facing 90 degrees
"""

import rospy
import sys
import math
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class InteractiveGoalPublisher:
    def __init__(self):
        rospy.init_node('interactive_goal_pub', anonymous=True)

        self.goal_pub = rospy.Publisher('/planning/click_goal', PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/lidar_slam/odom', Odometry, self.odom_callback)

        self.current_pos = None
        self.current_yaw = None
        self.goal_count = 0

    def odom_callback(self, msg):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi

    def publish_goal(self, x, y, z, yaw_deg=None):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        if yaw_deg is not None:
            yaw_rad = yaw_deg * math.pi / 180.0
            q = quaternion_from_euler(0, 0, yaw_rad)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
        else:
            # NaN quaternion = let planner choose yaw
            goal.pose.orientation.w = float('nan')

        self.goal_pub.publish(goal)
        self.goal_count += 1
        yaw_str = f", yaw={yaw_deg:.0f}deg" if yaw_deg is not None else ""
        print(f"  [Goal #{self.goal_count}] Sent: ({x:.1f}, {y:.1f}, {z:.1f}{yaw_str})")
        if self.current_pos:
            dist = math.sqrt(
                (x - self.current_pos[0])**2 +
                (y - self.current_pos[1])**2 +
                (z - self.current_pos[2])**2
            )
            print(f"  Distance from current pos: {dist:.1f}m")

    def print_status(self):
        if self.current_pos:
            print(f"  Drone at: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {self.current_pos[2]:.1f})")
            print(f"  Yaw: {self.current_yaw:.1f} deg")
        else:
            print("  No odom received yet")

    def run(self):
        print("=" * 60)
        print("  SUPER Interactive Goal Publisher")
        print("=" * 60)
        print("  Type 'x y z' to send a goal (e.g. '0 0 10')")
        print("  Type 'status' to see drone position")
        print("  Type 'help' for all commands")
        print("  Type 'quit' to exit")
        print("=" * 60)

        # Wait for odom
        print("\n  Waiting for drone odom...", end='', flush=True)
        timeout = rospy.Time.now() + rospy.Duration(10)
        while self.current_pos is None and rospy.Time.now() < timeout and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if self.current_pos:
            print(f" OK! Drone at ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {self.current_pos[2]:.1f})")
        else:
            print(" WARNING: no odom yet, the drone sim might not be running")

        print()
        while not rospy.is_shutdown():
            try:
                line = input("goal> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting...")
                break

            if not line:
                continue

            if line.lower() in ('quit', 'q', 'exit'):
                print("Exiting...")
                break

            if line.lower() == 'help':
                print(__doc__)
                continue

            if line.lower() == 'status':
                self.print_status()
                continue

            # Parse coordinates
            parts = line.split()
            try:
                if len(parts) == 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    self.publish_goal(x, y, z)
                elif len(parts) == 4:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    yaw = float(parts[3])
                    self.publish_goal(x, y, z, yaw)
                else:
                    print("  Usage: x y z [yaw_deg]  (e.g. '0 0 10' or '10 0 5 90')")
            except ValueError:
                print("  Invalid input. Type numbers: x y z [yaw_deg]")


if __name__ == '__main__':
    try:
        pub = InteractiveGoalPublisher()
        pub.run()
    except rospy.ROSInterruptException:
        pass
