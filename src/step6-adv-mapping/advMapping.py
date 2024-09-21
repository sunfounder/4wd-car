import numpy as np
import picar_4wd as fc
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pyhere # type: ignore
import math
from typing import List
import time
import signal
import sys

MAX_ANGLE = 90
MIN_ANGLE = -90
STEP_ANGLE = 3
MAX_MAPPING_DIST = 50 # in cm
OUTPUT_FOLDER = pyhere.here("src/maps")
VISUALIZE_MAP = True
DEBUG_MODE = False
INTERPOLATE = False
OBSTACLE_THRESHOLD = 20  # cm

FORWARD_TIME = 0.5
TURN_TIME = 0.5

class SimpleSLAM:
    def __init__(self, map_size, resolution):
        self.map = np.zeros((map_size, map_size), dtype=float)
        self.resolution = resolution
        self.position = [map_size // 2, 0, 0] # x, y, theta
        self.map_size = map_size

    def update_position(self, distance, angle):
        self.position[0] += distance * math.cos(self.position[2] + angle)
        self.position[1] += distance * math.sin(self.position[2] + angle)
        self.position[2] += angle
        self.position[2] %= 2 * math.pi 

    def update_map(self, scan_data, scan_angles, visualize=False):
        x, y, theta = self.position
        angles_in_rad = np.deg2rad(np.array(scan_angles))
        dist_clip = np.clip(scan_data, a_min=0, a_max=MAX_MAPPING_DIST)
        
        obj_pos = dist_clip * np.array([np.sin(angles_in_rad + theta), np.cos(angles_in_rad + theta)])
        obj_xy = np.int32(np.round(obj_pos, 0)).T
        
        obstacle_points = []
        
        for point in obj_xy:
            map_x = int(x + point[0])
            map_y = int(y + point[1])
            
            if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
                self.map[map_y, map_x] += 0.1
                self.map[map_y, map_x] = min(self.map[map_y, map_x], 1.0)
                obstacle_points.append((map_x, map_y))

        if INTERPOLATE:
            for i in range(1, len(obstacle_points)):
                self.interpolate_points(obstacle_points[i-1], obstacle_points[i])

        if visualize:
            self.visualize_map(obj_xy)

        return obstacle_points

    def interpolate_points(self, point1, point2):
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
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                self.map[y, x] += 0.1
                self.map[y, x] = min(self.map[y, x], 1.0)

    def visualize_map(self, obj_xy):
        if not os.path.exists(OUTPUT_FOLDER):
            os.makedirs(OUTPUT_FOLDER)
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"{OUTPUT_FOLDER}/map_{timestamp}.png"
        
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='Greys', origin='lower')
        plt.scatter(self.position[0] + obj_xy[:, 0], self.position[1] + obj_xy[:, 1], c='red', s=50, edgecolor='none')
        plt.scatter(self.position[0], self.position[1], c='blue', s=1000, marker='^', edgecolor='none')
        
        plt.title(f"Environment Map - {timestamp}")
        plt.xlabel("X-axis (cm)")
        plt.ylabel("Y-axis (cm)")
        plt.grid(True)
        plt.axis('equal')
        plt.xlim(0, self.map_size)
        plt.ylim(0, self.map_size)
        
        plt.savefig(filename)
        plt.close()
        print(f"Map saved as {filename}")

    def get_map(self) -> np.ndarray:
        return self.map

    def get_position(self) -> List[float]:
        return self.position

def scan_dist(direct=0, debug=False, step_angle=STEP_ANGLE):
    if debug:
        angles = list(range(MIN_ANGLE, MAX_ANGLE + 1, 1))  # 1 degree steps
    else:
        angle_list = list(range(MIN_ANGLE, MAX_ANGLE + step_angle, step_angle))
        angles = angle_list if direct else angle_list[::-1]
    ret = [fc.get_distance_at(a) for a in angles]
    return ret, angles

def main():
    setup_interrupt_handler()
    slam = SimpleSLAM(map_size=MAX_MAPPING_DIST * 2, resolution=1)
    start_time = time.time()
    
    try:
        while time.time() - start_time < 30:  
            dist, angles = scan_dist(direct=0, debug=DEBUG_MODE, step_angle=STEP_ANGLE)
            slam.update_map(dist, angles, visualize=VISUALIZE_MAP)
            
            # clip the distance to the max mapping distance 
            dist = np.clip(dist, a_min=0, a_max=MAX_MAPPING_DIST)
            
            # get the distance of the left, forward, and right sectors
            left_sector = dist[:len(dist)//3]
            forward_sector = dist[len(dist)//3:2*len(dist)//3]
            right_sector = dist[2*len(dist)//3:]
            
            # get the average distance of the left, forward, and right sectors
            avg_left = sum(left_sector) / len(left_sector)
            avg_forward = sum(forward_sector) / len(forward_sector)
            avg_right = sum(right_sector) / len(right_sector)
            
            if avg_forward < OBSTACLE_THRESHOLD:
                if avg_left > avg_right:
                    print("Obstacle ahead, turning left...")
                    fc.turn_left(30)
                    time.sleep(TURN_TIME)
                    fc.stop()
                    slam.update_position(0, math.pi/4)
                else:
                    print("Obstacle ahead, turning right...")
                    fc.turn_right(30)
                    time.sleep(TURN_TIME)
                    fc.stop()
                    slam.update_position(0, -math.pi/4) 
            else:
                print("Moving forward")
                fc.forward(30)
                time.sleep(FORWARD_TIME)
                fc.stop()
                slam.update_position(10, 0)      
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        fc.stop()
        print("Test completed.")

def setup_interrupt_handler():
    signal.signal(signal.SIGINT, signal_handler)
    print('Press Ctrl+C to stop the program')

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    fc.stop()
    sys.exit(0)

if __name__ == "__main__":
    main()
