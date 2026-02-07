def generate_mine_mission(filename):
    waypoints = []
    
    z_levels = range(5, 75, 10) # 5, 15, ..., 65
    
    min_x, max_x = -13, 13
    min_y, max_y = -1.5, 1.5
    
    for i, z in enumerate(z_levels):
        if i % 2 == 0:
            # Clockwise / zigzag 1
            waypoints.append([min_x, min_y, z, 1])
            waypoints.append([max_x, min_y, z, 1])
            waypoints.append([max_x, max_y, z, 1])
            waypoints.append([min_x, max_y, z, 1])
        else:
            # Reverse / zigzag 2
            waypoints.append([min_x, max_y, z, 1])
            waypoints.append([max_x, max_y, z, 1])
            waypoints.append([max_x, min_y, z, 1])
            waypoints.append([min_x, min_y, z, 1])
            
    # Return to base
    waypoints.append([0, 0, 1.5, 1])

    with open(filename, 'w') as f:
        for p in waypoints:
            f.write(f"{p[0]} {p[1]} {p[2]} {p[3]}\n")
            
    print(f"Generated {len(waypoints)} waypoints in {filename}")

if __name__ == "__main__":
    generate_mine_mission("mine_coverage.txt")
