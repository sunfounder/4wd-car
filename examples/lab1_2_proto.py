import time
import numpy as np
import picar_4wd as fc

# ~ from adv-mapping import scan_dist, map_obj
from implementation import *


DIRECITONS = ['W', 'N', 'E', 'S']
FORWARD_SPEED = 8
FORWARD_TIME = 0.4
TURN_SPEED = 30
TURN_TIME = 1


def get_direction_distance(start, end):
    ew = end[0] - start[0]
    ns = end[1] - start[1]
    if ns and ew == 0:
        return ('S', ns) if ns > 0 else ('N', ns)
    elif ew and ns == 0:
        return ('E', ew) if ew > 0 else ('W', ew)
    else:
        raise Exception(f"Unexpected start {start} and end {end}")


def refine_path(path, car_loc, car_direction):
    ret = [car_loc]
    start_loc = car_loc
    cur_loc = car_loc
    cur_direction = car_direction
    for next_loc in path:
        direction, dist = get_direction_distance(cur_loc, next_loc)
        if direction == cur_direction:
            cur_loc = next_loc
            next
        else:
            ret += [next_loc]
            cur_direction = direction
            cur_loc = next_loc
            start_loc = next_loc
    return ret


class Picar:
    def __init__(self, loc):
        self.loc = loc
        self.direction = 'S'

    def move_to(self, dest):
        print(f"at {self.loc}, moving to {dest}")
        direction, dist = get_direction_distance(self.loc, dest)
        
        self.turn_to(direction)

        print(f"move {dist}")
        fc.forward(FORWARD_SPEED)
        time.sleep(abs(dist)*FORWARD_TIME)
        fc.stop()
        
        if direction in ['W', 'E']:
            self.loc[0] += dist
        else:
            self.loc[1] += dist

        assert(self.loc[0] == dest[0] and self.loc[1] == dest[1])

    def turn_to(self, direction):
        if self.direction == direction: return
        
        cur_dir_idx = DIRECITONS.index(self.direction)

        if DIRECITONS[(cur_dir_idx+1)%4] == direction:
            print('turn right')
            fc.turn_right(TURN_SPEED)
            time.sleep(TURN_TIME)
            fc.stop()
        elif DIRECITONS[cur_dir_idx-1] == direction:
            print('turn left')
            fc.turn_left(TURN_SPEED)
            time.sleep(TURN_TIME)
            fc.stop()
            
        else:
            raise Exception(f"cannot turn from {self.direction} to {direction}")

        self.direction = direction
    


diagram5 = GridWithWeights(10, 10)
#diagram5.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram5.walls = [(3, 2), (3, 3), (3, 4), (3, 5),
                  (4, 2), (4, 3), (4, 4), (4, 5)]

start, goal = (4, 0), (3, 9)

car = Picar(list(start))

came_from, cost_so_far = a_star_search(diagram5, start, goal)
draw_grid(diagram5, point_to=came_from, start=start, goal=goal)
print()
draw_grid(diagram5, path=reconstruct_path(came_from, start=start, goal=goal))


# convert path to command
path = reconstruct_path(came_from, start=start, goal=goal)
for spot in path[1:]:
    car.move_to(spot)

