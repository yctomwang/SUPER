#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import time

class ExplorationManager:
    def __init__(self):
        rospy.init_node('exploration_manager')
        
        self.frontier_sub = rospy.Subscriber('/rog_map/frontier', PointCloud2, self.frontier_cb)
        self.odom_sub = rospy.Subscriber('/lidar_slam/odom', Odometry, self.odom_cb)
        self.goal_pub = rospy.Publisher('/planning/click_goal', PoseStamped, queue_size=1)
        
        self.robot_pos = None
        self.frontiers = []
        self.current_goal = None
        self.last_goal_time = 0
        self.stuck_timeout = 15.0 # Seconds to wait before giving up on a goal
        self.reached_dist = 3.0   # Distance to consider goal reached
        
        # Wait for connections
        time.sleep(1.0)
        
        self.timer = rospy.Timer(rospy.Duration(2.0), self.control_loop)
        
        print("[Exploration] Initialized. Waiting for frontiers...")

    def odom_cb(self, msg):
        self.robot_pos = np.array([msg.pose.pose.position.x, 
                                   msg.pose.pose.position.y, 
                                   msg.pose.pose.position.z])

    def frontier_cb(self, msg):
        # Parse PointCloud2
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # Convert to numpy array immediately
        pts = list(gen)
        if len(pts) > 0:
            self.frontiers = np.array(pts)
        else:
            self.frontiers = np.empty((0,3))

    def control_loop(self, event):
        if self.robot_pos is None:
            return

        if len(self.frontiers) == 0:
            # print("[Exploration] No frontiers detected yet.")
            return

        # Check if we reached the goal or timed out
        if self.current_goal is not None:
            dist = np.linalg.norm(self.robot_pos - self.current_goal)
            
            # If closer than threshold, mark reached
            if dist < self.reached_dist:
                print(f"[Exploration] Goal reached (dist {dist:.2f}m < {self.reached_dist}m). Picking new goal.")
                self.current_goal = None
            
            # If timed out
            elif time.time() - self.last_goal_time > self.stuck_timeout:
                 print(f"[Exploration] Goal timeout ({self.stuck_timeout}s). Picking new goal.")
                 self.current_goal = None
        
        # If no goal, select one
        if self.current_goal is None:
            self.select_next_goal()

    def select_next_goal(self):
        if len(self.frontiers) == 0:
            return

        # Strategy: Cluster frontiers to find significant unknown areas
        # 1. Discretize frontiers to large voxels (e.g. 3m) to group them
        resolution = 3.0
        rounded = np.floor(self.frontiers / resolution) * resolution
        
        # 2. Count points in each voxel
        # Use a structured array or convert to void view for np.unique on rows
        # But for simplicity/speed with float arrays:
        # We can map to string keys or just use lexical sort
        
        # Simple clustering:
        # Find unique voxel centers and their counts
        # This is a bit heavy if points are many, but frontier points usually aren't millions
        
        # Faster way: Lexicographical sort
        # View as void
        dtype = np.dtype((np.void, rounded.dtype.itemsize * rounded.shape[1]))
        b = np.ascontiguousarray(rounded).view(dtype)
        unique_b, counts = np.unique(b, return_counts=True)
        unique_centers = unique_b.view(rounded.dtype).reshape(-1, rounded.shape[1])
        
        # Filter noise clusters (too few points)
        min_points = 3
        valid_mask = counts >= min_points
        
        candidates = unique_centers[valid_mask]
        candidate_counts = counts[valid_mask]
        
        if len(candidates) == 0:
            print("[Exploration] Only small noise frontiers found.")
            return
            
        # Score candidates: 
        # Score = Size * 1.0 - Distance * 0.5
        # We want BIG clusters that are somewhat CLOSE
        
        best_score = -float('inf')
        best_goal = None
        
        for i, center in enumerate(candidates):
            # Center of the voxel + half resolution to be in middle
            goal_pt = center + resolution / 2.0
            
            dist = np.linalg.norm(goal_pt - self.robot_pos)
            
            if dist < 2.0: continue # Too close to be useful
            
            # Simple utility function
            # Prefer larger clusters, penalize distance
            score = candidate_counts[i] * 1.0 - dist * 0.5
            
            if score > best_score:
                best_score = score
                best_goal = goal_pt

        if best_goal is not None:
            self.publish_goal(best_goal)
        else:
            print("[Exploration] No suitable goal found.")

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        
        # Orient towards goal? Optional. Identity for now.
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)
        self.current_goal = goal
        self.last_goal_time = time.time()
        print(f"[Exploration] >>> Sending Goal: [{goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f}]")

if __name__ == '__main__':
    try:
        ExplorationManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
