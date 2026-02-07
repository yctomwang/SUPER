import numpy as np
import random

def generate_mine_stope_pcd(filename):
    points = []
    
    # Dimensions
    length = 30.0
    width = 6.0
    height = 70.0
    
    # Steps for density
    step = 0.2
    
    # Walls
    # X walls (at -15 and +15)
    for y in np.arange(-width/2, width/2, step):
        for z in np.arange(0, height, step):
            points.append([-length/2, y, z])
            points.append([length/2, y, z])
            
    # Y walls (at -3 and +3)
    for x in np.arange(-length/2, length/2, step):
        for z in np.arange(0, height, step):
            points.append([x, -width/2, z])
            points.append([x, width/2, z])
            
    # Floor and Ceiling
    for x in np.arange(-length/2, length/2, step):
        for y in np.arange(-width/2, width/2, step):
            points.append([x, y, 0])
            points.append([x, y, height])

    # Obstacles (Random pillars/blocks inside)
    num_obstacles = 20
    for _ in range(num_obstacles):
        # Random center
        cx = random.uniform(-length/2 + 2, length/2 - 2)
        cy = random.uniform(-width/2 + 1, width/2 - 1)
        cz = random.uniform(5, height - 5)
        
        # Random size
        sx = random.uniform(0.5, 2.0)
        sy = random.uniform(0.5, 2.0)
        sz = random.uniform(0.5, 10.0)
        
        # Fill obstacle
        for x in np.arange(cx - sx/2, cx + sx/2, 0.2):
            for y in np.arange(cy - sy/2, cy + sy/2, 0.2):
                for z in np.arange(cz - sz/2, cz + sz/2, 0.2):
                    points.append([x, y, z])

    # Write PCD
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")
            
    print(f"Generated {len(points)} points in {filename}")

if __name__ == "__main__":
    generate_mine_stope_pcd("mine_stope.pcd")
