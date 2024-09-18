import numpy as np
import picar_4wd as fc
import matplotlib.pyplot as plt
import os
from datetime import datetime

MAX_ANGLE = 90
MIN_ANGLE = -90
STEP_ANGLE = 5
ANGLE_LIST = list(range(MIN_ANGLE, MAX_ANGLE + STEP_ANGLE, STEP_ANGLE))
MAX_MAPPING_DIST = 50  # in cm
OUTPUT_FOLDER = "maps"
VISUALIZE_MAP = False

def scan_dist(direct=0):
    angles = ANGLE_LIST if direct else ANGLE_LIST[::-1] 
    ret = [fc.get_distance_at(a) for a in angles] 
    return ret

def interpolate(mapping, point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    dx = x2 - x1
    dy = y2 - y1
    
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return

    x_i = dx / steps
    y_i = dy / steps

    for step in range(1, steps):
        x = int(round(x1 + step * x_i))
        y = int(round(y1 + step * y_i))
        if 0 <= x < mapping.shape[1] and 0 <= y < mapping.shape[0]:
            mapping[y][x] = 1  # row x column

def map_obj(mapping, car_pos, dist, visualize = False):
    dist_clip = np.clip(dist, a_min=0, a_max=MAX_MAPPING_DIST)

    angles_in_rad = np.deg2rad(np.array(ANGLE_LIST))
    
    obj_pos = dist_clip * np.array([np.sin(angles_in_rad), np.cos(angles_in_rad)])
    obj_xy = np.int32(np.round(obj_pos, 0)).T

    car_x, car_y = car_pos
    
    obstacle_points = []
    
    height, width = mapping.shape
        
    for point in obj_xy:
        x = point[0] + car_x
        y = car_y + point[1] # changed to + to place obstacles in front of the car
        if 0 <= x < width and 0 <= y < height:
            mapping[y][x] = 1  # row x column
            obstacle_points.append((x, y))

    for i in range(1, len(obstacle_points)):
        point1 = obstacle_points[i - 1]
        point2 = obstacle_points[i]
        interpolate(mapping, point1, point2)
        
    if visualize: visualize_map(mapping, car_pos, obj_xy)
    
    return mapping

def visualize_map(mapping, car_pos, obj_xy):
    
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
        
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    filename = f"{OUTPUT_FOLDER}/map_{timestamp}.png"
    
    x = np.array([i[0] for i in obj_xy])
    y = np.array([i[1] for i in obj_xy])
    
    plt.figure(figsize=(10, 10))
    
    plt.imshow(mapping, cmap='Greys', origin='lower')
    
    plt.scatter(car_pos[0] + x, car_pos[1] + y, c='red', s=50)  # plot obstacles
    plt.scatter(car_pos[0], car_pos[1], c='blue', s=1000, marker='^')  # car position
    
    plt.title(f"Environment Map - {timestamp}")
    plt.xlabel("X-axis (cm)")
    plt.ylabel("Y-axis (cm)")
    plt.grid(True)
    plt.axis('equal')
    plt.xlim(0, MAX_MAPPING_DIST * 2)
    plt.ylim(0, MAX_MAPPING_DIST * 2)
    
    plt.savefig(filename)
    plt.close()
    print(f"Map saved as {filename}")

def main():
    mapping = np.zeros((MAX_MAPPING_DIST * 2, MAX_MAPPING_DIST * 2), dtype=int)
    
    # Car at bottom center, facing up
    car_pos = [MAX_MAPPING_DIST, 0]
    
    dist = np.array(scan_dist(direct=0))
    
    mapping = map_obj(mapping, car_pos, dist, VISUALIZE_MAP)

    print("Distance Readings:", list(dist))
    
    for row in mapping:
        print(' '.join(map(str, row)))

if __name__ == "__main__":
    main()
