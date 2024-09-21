import numpy as np
import picar_4wd as fc
from a_star import *


MAX_ANGLE = 90
MIN_ANGLE = -90
STEP_ANGLE = 5
SERVO_ANGLE_ERROR = -5  # servo is 5 degree to the left when reset to 0 degree
ANGLE_LIST = list(range(MIN_ANGLE, MAX_ANGLE+STEP_ANGLE, STEP_ANGLE))[-1::-1]
MAX_MAPPING_DIST = 50

def scan_dist(direct=0):
	if direct:  # left to right
		angles = iter(ANGLE_LIST)
		#print(ANGLE_LIST)
	else:       # right to left
		angles = iter(ANGLE_LIST[-1::-1])
	#print(list(angles))
	ret = list()
	for a in angles:
		ret.append(fc.get_distance_at(a + SERVO_ANGLE_ERROR))
	assert(len(ret) == len(ANGLE_LIST))
	
	if direct == 0:
		ret.reverse()  # always return dist from left to right
	
	return ret
	
def map_obj(dist):
	dist_clip = np.clip(dist, a_min=0, a_max=2**31-1)  # processing sensor data, prune -1 and -2, cap to int32 max

	angles_in_rad = np.array(ANGLE_LIST)*np.pi/180

	obj_pos = dist_clip * np.array([np.sin(angles_in_rad), np.cos(angles_in_rad)])  # car pos is (0, 0)
	obj_xy = np.int32(np.round(obj_pos, 0))

	#mapping = np.zeros((MAX_MAPPING_DIST+1, 2*MAX_MAPPING_DIST+1))  # car pos is (0, MAX_MAPPING_DIST)
	mapping = list()

	for i in range(len(ANGLE_LIST)):
		x = obj_xy[0][i]
		y = obj_xy[1][i]
		if  np.abs(x)<=MAX_MAPPING_DIST and y<=MAX_MAPPING_DIST and not (x ==0 and y == 0):
			mapping.append((x+MAX_MAPPING_DIST, y))
			#mapping[MAX_MAPPING_DIST-y][x+MAX_MAPPING_DIST] = 1
			#print(f"x={x+MAX_MAPPING_DIST}, y={MAX_MAPPING_DIST-y}")
			
	return mapping
    

if __name__ == "__main__":
    start, goal = (5, 0), (10, 10)
    diagram = GridWithWeights(11, 11)
    
    dist = np.array(scan_dist(1))
    mapping = map_obj(dist)

    walls = list()
    for m in mapping:
        wall = (m[0]//10, m[1]//10)
        if wall in walls:
            continue
        walls.append(wall)

    diagram.walls = walls
    came_from, cost_so_far = a_star_search(diagram, start, goal)
    path = reconstruct_path(came_from, start=start, goal=goal)
    draw_grid(diagram, point_to=came_from, start=start, goal=goal)
    print()
    draw_grid(diagram, path=reconstruct_path(came_from, start=start, goal=goal))
    
