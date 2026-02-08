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
        self.frontiers = None
        self.current_goal = None
        self.last_goal_time = 0
        self.stuck_timeout = 20.0 # Seconds to wait before giving up on a goal
        self.reached_dist = 2.0   # Distance to consider goal reached
        self.initial_nudge_done = False
        self.start_time = time.time()
        
        # Wait for connections
        time.sleep(1.0)
        
        self.timer = rospy.Timer(rospy.Duration(1.0), self.control_loop)
        
        print("[Exploration] Initialized. Waiting for frontiers...")

    def odom_cb(self, msg):
        self.robot_pos = np.array([msg.pose.pose.position.x, 
                                   msg.pose.pose.position.y, 
                                   msg.pose.pose.position.z])

    def frontier_cb(self, msg):
        # Parse PointCloud2
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = list(gen)
        if len(pts) > 0:
            self.frontiers = np.array(pts)
        else:
            self.frontiers = np.empty((0,3))

    def control_loop(self, event):
        if self.robot_pos is None:
            return

        # Initial Nudge Logic: Retry until frontiers appear
        if self.frontiers is None or len(self.frontiers) == 0:
            # If 5 seconds passed since start, try nudging
            if time.time() - self.start_time > 5.0:
                # If we haven't nudged recently (every 5s)
                if time.time() - self.last_goal_time > 5.0:
                    print("[Exploration] No frontiers yet. Sending Initial Nudge (Up 5m).")
                    nudge_goal = self.robot_pos + np.array([0, 0, 5.0])
                    self.publish_goal(nudge_goal)
                    # Don't set initial_nudge_done flag to stop retrying, 
                    # instead rely on frontiers appearing to break this loop.
            return

        # Check if we reached the goal or timed out
        if self.current_goal is not None:
            dist = np.linalg.norm(self.robot_pos - self.current_goal)
            
            if dist < self.reached_dist:
                print(f"[Exploration] Goal reached (dist {dist:.2f}m). Picking new goal.")
                self.current_goal = None
            elif time.time() - self.last_goal_time > self.stuck_timeout:
                 print(f"[Exploration] Goal timeout ({self.stuck_timeout}s). Picking new goal.")
                 self.current_goal = None
        
        # If no goal, select one
        if self.current_goal is None:
            self.select_next_goal()

    def select_next_goal(self):
        if self.frontiers is None or len(self.frontiers) == 0:
            return

        print(f"[Exploration] Processing {len(self.frontiers)} frontier points...")

        # 1. Discretize frontiers to large voxels (3m)
        resolution = 3.0
        # Use simple integer grid coordinates
        grid_indices = np.floor(self.frontiers / resolution).astype(int)
        
        # 2. Count points in each voxel
        unique_indices, counts = np.unique(grid_indices, axis=0, return_counts=True)
        
        # 3. Filter noise
        min_points = 5
        valid_mask = counts >= min_points
        
        candidates_indices = unique_indices[valid_mask]
        candidates_counts = counts[valid_mask]
        
        if len(candidates_indices) == 0:
            print("[Exploration] Only small noise frontiers found (all < 5 points).")
            return
            
        # Convert grid indices back to world coordinates (center of voxel)
        candidates_pos = candidates_indices * resolution + resolution/2.0
        
        best_score = -float('inf')
        best_goal = None
        
        for i, center in enumerate(candidates_pos):
            # 1. Hard Constraints
            if center[2] > 65.0: continue # Too high (near ceiling)
            if center[2] < 1.0: continue  # Too low (floor)

            dist = np.linalg.norm(center - self.robot_pos)
            
            if dist < 2.0: 
                # print(f"  Reject: Too close ({dist:.1f}m)")
                continue 
            
            # 2. Score Calculation
            # Preference: Large Clusters > Nearby > Horizontal
            
            # Calculate vertical distance
            z_diff = abs(center[2] - self.robot_pos[2])
            
            # Penalize layer change (vertical jump > 5m)
            layer_penalty = 0.0
            if z_diff > 5.0:
                layer_penalty = 20.0 
            
            # Score = Size * 1.0 - Distance * 0.5 - LayerPenalty
            score = candidates_counts[i] * 1.0 - dist * 0.5 - layer_penalty
            
            if score > best_score:
                best_score = score
                best_goal = center

        if best_goal is not None:
            print(f"[Exploration] Selected goal with score {best_score:.1f} (Size: {candidates_counts[i]}, Dist: {dist:.1f}m)")
            self.publish_goal(best_goal)
        else:
            print("[Exploration] No suitable goal found (all too close?).")

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        
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
