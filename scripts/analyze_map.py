#!/usr/bin/env python3
import numpy as np
import sys
import os

def analyze_pcd(file_path):
    print(f"Analyzing {file_path}...")
    
    # Simple ASCII PCD parser
    points = []
    header_passed = False
    
    try:
        with open(file_path, 'r') as f:
            for line in f:
                if not header_passed:
                    if line.startswith('DATA ascii'):
                        header_passed = True
                    continue
                
                parts = line.strip().split()
                if len(parts) >= 3:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    pts = np.array(points)
    print(f"Loaded {len(pts)} points.")
    
    if len(pts) == 0:
        return

    print(f"Bounds:")
    print(f"  X: [{np.min(pts[:,0]):.2f}, {np.max(pts[:,0]):.2f}]")
    print(f"  Y: [{np.min(pts[:,1]):.2f}, {np.max(pts[:,1]):.2f}]")
    print(f"  Z: [{np.min(pts[:,2]):.2f}, {np.max(pts[:,2]):.2f}]")

    # Check start position (0,0,3)
    start_pos = np.array([0, 0, 3.0])
    dists = np.linalg.norm(pts - start_pos, axis=1)
    min_dist = np.min(dists)
    print(f"\nStart Position (0,0,3):")
    print(f"  Distance to nearest wall: {min_dist:.2f}m")
    
    if min_dist < 1.0:
        print("  WARNING: Start position is VERY close to a wall!")
    else:
        print("  Start position seems safe.")

    # Slice analysis at Z=3
    print("\nSlice analysis at Z=3.0 +/- 1.0m:")
    slice_mask = np.abs(pts[:,2] - 3.0) < 1.0
    slice_pts = pts[slice_mask]
    
    if len(slice_pts) > 0:
        print(f"  Points in slice: {len(slice_pts)}")
        print(f"  Slice X range: [{np.min(slice_pts[:,0]):.2f}, {np.max(slice_pts[:,0]):.2f}]")
        print(f"  Slice Y range: [{np.min(slice_pts[:,1]):.2f}, {np.max(slice_pts[:,1]):.2f}]")
        
        # Check if 0,0 is inside the bounds of the slice
        if 0 < np.min(slice_pts[:,0]) or 0 > np.max(slice_pts[:,0]) or \
           0 < np.min(slice_pts[:,1]) or 0 > np.max(slice_pts[:,1]):
            print("  WARNING: (0,0) is OUTSIDE the bounding box of the slice!")
        else:
            print("  (0,0) is inside the bounding box of the slice.")
    else:
        print("  No points found in this slice.")

if __name__ == "__main__":
    pcd_file = os.path.join(os.path.dirname(__file__), '../mars_uav_sim/perfect_drone_sim/pcd/real_stope.pcd')
    analyze_pcd(pcd_file)
