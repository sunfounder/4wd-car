import numpy as np
import picar_4wd as fc

MAX_ANGLE = 90
MIN_ANGLE = -90
STEP_ANGLE = 5
ANGLE_LIST = list(range(MIN_ANGLE, MAX_ANGLE + STEP_ANGLE, STEP_ANGLE))[-1::-1]
MAX_MAPPING_DIST = 20

def scan_dist(direct=0):
    if direct: # left to right
        angles = iter(ANGLE_LIST)
    else:      # right to left  
        angles = iter(ANGLE_LIST[-1::-1])
    
    ret = list()
    for a in angles:
        ret.append(fc.get_distance_at(a))
    assert(len(ret) == len(ANGLE_LIST))
    
    if direct == 0:
        ret.reverse()  # always return dist from left to right
    
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
        if 0 <= x < mapping.shape[0] and 0 <= y < mapping.shape[1]:
            mapping[x][y] = 1  

def map_obj(mapping, car_position, dist):
    dist_clip = np.clip(dist, a_min=0, a_max=2**31-1)  

    angles_in_rad = np.deg2rad(np.array(ANGLE_LIST))  
    
    obj_pos = dist_clip * np.array([-np.sin(angles_in_rad), np.cos(angles_in_rad)])  
    obj_xy = np.int32(np.round(obj_pos, 0)).T  

    car_x, car_y = car_position
    
    obstacle_points = []
    
    # Map each obstacle point
    for point in obj_xy:
        x = point[0] + car_x
        y = car_y - point[1]  
        if 0 <= x < mapping.shape[0] and 0 <= y < mapping.shape[1] and not (x == car_x and y == car_y):
            mapping[x][y] = 1 # mark as obstacle
            obstacle_points.append((x, y))

    for i in range(1, len(obstacle_points)):
        point1 = obstacle_points[i - 1]
        point2 = obstacle_points[i]
        interpolate(mapping, point1, point2)

    return mapping

def main():
    mapping = np.zeros((MAX_MAPPING_DIST + 1, 2 * MAX_MAPPING_DIST + 1), dtype=int)
    car_pos = (MAX_MAPPING_DIST, MAX_MAPPING_DIST)  

    dist = np.array(scan_dist(direct=1))  
    mapping = map_obj(mapping, car_pos, dist)  

    print("Distance Readings:", list(dist))

    for i in range(MAX_MAPPING_DIST):
        print(list(mapping[i]))

if __name__ == "__main__":
    main()
