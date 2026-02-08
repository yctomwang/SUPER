#!/usr/bin/env python3
"""
Generate a zig-zag scanning waypoint mission for the real stope.
Real stope dimensions (centered):
  X: [-36.7, 36.7]  (73.4m span)
  Y: [-75.4, 75.4]  (150.7m span)
  Z: [0, 68.4]      (68.4m height)

We'll fly a pattern that covers the stope in layers,
staying a few meters inside the walls.
"""
import os

def generate_mission(filename):
    waypoints = []

    # Stay inside the walls with margin
    margin = 5.0
    min_x, max_x = -30, 30       # ~60m range in X
    min_y, max_y = -70, 70       # ~140m range in Y

    # Z layers from bottom to top
    z_levels = list(range(5, 66, 8))  # 5, 13, 21, 29, 37, 45, 53, 61

    # Zig-zag pattern: alternate Y direction at each X pass, alternate X direction at each Z level
    y_positions = [min_y, max_y]

    for zi, z in enumerate(z_levels):
        if zi % 2 == 0:
            # Left to right in X
            x_positions = list(range(min_x, max_x + 1, 15))  # every 15m
        else:
            # Right to left in X
            x_positions = list(range(max_x, min_x - 1, -15))

        for xi, x in enumerate(x_positions):
            if xi % 2 == 0:
                waypoints.append([x, min_y, z, 1])
                waypoints.append([x, max_y, z, 1])
            else:
                waypoints.append([x, max_y, z, 1])
                waypoints.append([x, min_y, z, 1])

    # Return to start area
    waypoints.append([0, 0, 3, 1])

    # Write waypoints (NO count line - parser reads every line as a waypoint)
    with open(filename, 'w') as f:
        for wp in waypoints:
            f.write(f"{wp[0]} {wp[1]} {wp[2]} {wp[3]}\n")

    print(f"Generated {len(waypoints)} waypoints for real stope mission")
    print(f"Z levels: {z_levels}")
    print(f"Written to: {filename}")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(script_dir)
    output = os.path.join(repo_dir, "mission_planner", "data", "real_stope_coverage.txt")
    generate_mission(output)
