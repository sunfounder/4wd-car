import time
import numpy as np
import picar_4wd as fc

from advMapping import scan_dist, map_obj
from a_star import *

DIRECTIONS = ['W', 'N', 'E', 'S']
FORWARD_SPEED = 8
FORWARD_TIME = 0.4  # Time to move one grid unit forward
TURN_SPEED = 30
TURN_TIME = 1      # Time to turn 90 degrees

def get_direction_distance(start, end):
    ew = end[0] - start[0]
    ns = end[1] - start[1]
    if ns != 0 and ew == 0:
        return ('S', ns) if ns > 0 else ('N', ns)
    elif ew != 0 and ns == 0:
        return ('E', ew) if ew > 0 else ('W', ew)
    else:
        raise Exception(f"Unexpected start {start} and end {end}")

class Picar:
    def __init__(self, loc):
        self.loc = loc  # Actual location (for simulation)
        self.direction = 'S'  # Actual direction
        self.est_loc = loc.copy()  # Estimated location
        self.est_direction = self.direction  # Estimated direction

    def move_to(self, dest):
        print(f"At {self.loc}, moving to {dest}")
        direction, dist = get_direction_distance(self.loc, dest)
        
        self.turn_to(direction)
        self.update_estimated_direction(direction)

        print(f"Moving {dist} units forward")
        fc.forward(FORWARD_SPEED)
        time.sleep(abs(dist) * FORWARD_TIME)
        fc.stop()

        # Update actual location
        if direction in ['W', 'E']:
            self.loc[0] += dist
        else:
            self.loc[1] += dist

        # Update estimated location
        self.update_estimated_location(dist)
        print(f"Estimated Location: {self.est_loc}, Estimated Direction: {self.est_direction}")

        assert self.loc[0] == dest[0] and self.loc[1] == dest[1], "Mismatch in actual and expected location"

    def turn_to(self, direction):
        if self.direction == direction:
            return
        
        cur_dir_idx = DIRECTIONS.index(self.direction)
        target_dir_idx = DIRECTIONS.index(direction)
        turn = (target_dir_idx - cur_dir_idx) % 4

        if turn == 1:
            print('Turning right')
            fc.turn_right(TURN_SPEED)
            time.sleep(TURN_TIME)
            fc.stop()
        elif turn == 3:
            print('Turning left')
            fc.turn_left(TURN_SPEED)
            time.sleep(TURN_TIME)
            fc.stop()
        elif turn == 2:
            print('Turning around (180 degrees)')
            fc.turn_right(TURN_SPEED)
            time.sleep(2 * TURN_TIME)
            fc.stop()
        else:
            raise Exception(f"Cannot turn from {self.direction} to {direction}")

        self.direction = direction

    def update_estimated_direction(self, new_direction):
        self.est_direction = new_direction

    def update_estimated_location(self, dist):
        # Update the estimated location based on the estimated direction and distance
        if self.est_direction == 'N':
            self.est_loc[1] -= dist
        elif self.est_direction == 'S':
            self.est_loc[1] += dist
        elif self.est_direction == 'E':
            self.est_loc[0] += dist
        elif self.est_direction == 'W':
            self.est_loc[0] -= dist

    # Optional method for sensor-based correction
    def sensor_correction(self):
        # Implement sensor-based correction here
        # For example, using ultrasonic sensors to detect walls and adjust position
        pass

if __name__ == '__main__':
    diagram5 = GridWithWeights(10, 10)
    diagram5.walls = scan_dist(direct=0)

    start, goal = (4, 0), (3, 9)

    car = Picar(list(start))

    came_from, cost_so_far = a_star_search(diagram5, start, goal)
    draw_grid(diagram5, point_to=came_from, start=start, goal=goal)
    print()
    draw_grid(diagram5, path=reconstruct_path(came_from, start=start, goal=goal))

    # Convert path to command
    path = reconstruct_path(came_from, start=start, goal=goal)
    for spot in path[1:]:
        car.move_to(spot)
        # Optional: Apply sensor correction after each move
        # car.sensor_correction()
