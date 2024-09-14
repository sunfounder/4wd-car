import sys
import time

import numpy as np
import picar_4wd as fc


MAX_ANGLE = 90
MIN_ANGLE = -90
STEP_ANGLE = 5
ANGLE_LIST = list(range(MIN_ANGLE, MAX_ANGLE+STEP_ANGLE, STEP_ANGLE))[-1::-1]
MAX_MAPPING_DIST = 20

def scan_dist(direct=0):
	if direct:  # left to right
		angles = iter(ANGLE_LIST)
	else:       # right to left
		angles = iter(ANGLE_LIST[-1::-1])
	
	ret = list()
	for a in angles:
		ret.append(fc.get_distance_at(a))
	assert(len(ret) == len(ANGLE_LIST))
	
	if direct == 0:
		ret.reverse()  # always return dist from left to right
	
	return ret
	
def map_obj(dist):
	dist_clip = np.clip(dist, a_min=0, a_max=2**31-1)  # processing sensor data, prune -1 and -2, cap to int32 max

	angles_in_rad = np.array(ANGLE_LIST)*np.pi/180

	obj_pos = dist_clip * np.array([-np.sin(angles_in_rad), np.cos(angles_in_rad)])  # car pos is (0, 0)
	obj_xy = np.int32(np.round(obj_pos, 0))

	mapping = np.zeros((MAX_MAPPING_DIST+1, 2*MAX_MAPPING_DIST+1))  # car pos is (MAX_MAPPING_DIST, MAX_MAPPING_DIST)

	for i in range(len(ANGLE_LIST)):
		x = obj_xy[0][i]
		y = obj_xy[1][i]
		if  np.abs(x)<=MAX_MAPPING_DIST and y<=MAX_MAPPING_DIST and not (x ==0 and y == 0):
			mapping[MAX_MAPPING_DIST-y][x+MAX_MAPPING_DIST] = 1
			
	return mapping


dist = np.array(scan_dist())
mapping = map_obj(dist)

print(list(dist))

for i in range(MAX_MAPPING_DIST):
	print(list(mapping[i]))
	
# ~ fc.get_distance_at(-90)

# ~ while True:
# ~ for _ in range(2):
	# ~ for angle in angle_list:
		# ~ fc.get_distance_at(angle)
		# ~ time.sleep(0.01)
	# ~ angle_list.reverse()
	
# ~ print(angle_list)
# ~ print(dist_list)
# ~ print(scan_dist(0))
