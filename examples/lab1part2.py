import picar_4wd as fc
import time
import numpy as np
import sys
import math
from enum import Enum

# Define enum for holding driving direction, in relation to the destination, which can be defined
# as infinity in the direction of the starting position of the car.
class DrivingDirection (Enum):
    towards_destination = 1
    right = 2
    left =3
    away_from_destination = 4

# Set speed of car
speed = 20

# Set starting direction of car as toward destination
direction = DrivingDirection.towards_destination

# Initialise counter for measuring distance
distance_counter = 0
forward_timer = 0

# Initialise array representing 20 * 20 occupany squares of approx 20cm 
# when 7 represents unmapped areas, 0 represents clear and 1 represents obstacle
array_shape = (25, 25)
fill_value = 7
map = np.full(array_shape, fill_value)

# Intialise car location in array
car_position = [24, 10]


# Execute turn of car
def turn(turning_direction):

    global distance_counter

    # Set time for turning action for a period in seconds which gives a 90 degree turn angle.
    # Different timers needed for left and right turns to maintain consistent turning angle
    turn_left_timer = 0.9
    turn_right_timer = 1

    # Execute turn in direction received in function call and wait for specific time 
    # before stopping
    if turning_direction == 'right':
        fc.turn_right(speed)
        time.sleep(turn_right_timer)

    else:
        fc.turn_left(speed)
        time.sleep(turn_left_timer)
    
    # Stop turn
    fc.stop()
    #updatePositionTurning(turning_direction)

    # Reset distance counter as this is used to ensure car moves a certain distance
    # forward after each turn before attempting a turn towards destination
    distance_counter = 0
    return

def updatePositionMovingForward(distance):
    global direction
    global car_position
    if direction == DrivingDirection.towards_destination:
        car_position[0] = car_position[0] - distance

    elif direction == DrivingDirection.right:
        car_position[1] = car_position[1] + distance

    elif direction == DrivingDirection.left:
        car_position[1] = car_position[1] - distance
    
    else:
        car_position[0] = car_position[0] + distance

"""def updatePositionTurning(turning_direction):
    global direction
    global car_position

    if direction == DrivingDirection.towards_destination:
        if turning_direction == 'right':
            map[car_position[0], car_position[1]] = "4"
        else:
            map[car_position[0], car_position[1]] = "4"
            

    elif direction == DrivingDirection.right:
        if turning_direction == 'right':
            map[car_position[0], car_position[1]] = "4"
        else:
            map[car_position[0], car_position[1]] = "4"

    elif direction == DrivingDirection.left:
        if turning_direction == 'right':
            map[car_position[0], car_position[1]] = "4"
        else:
            map[car_position[0], car_position[1]] = "4"

    else:
        if turning_direction == 'right':
            map[car_position[0], car_position[1]] = "4"
        else:
            map[car_position[0], car_position[1]] = "4"

"""

# Move car forward and update distance counter each time function is called
def move_forward():
    global distance_counter
    fc.forward(speed)
    # Sleep while car travels 5 cm
    #time.sleep(0.16)
    distance_counter += 1
    print("Distance counter ", distance_counter)
    # Car moves 16cm forward each time
    updatePositionMovingForward(1)
    return

# Check ultrasonic scan. Assessing left, centre and right parts of scan for obstacles
# return blocked state object
def check_scan(scan_list, blocked_state):
    if scan_list[0:3] != [ 2, 2, 2]:
        blocked_state['left'] = True
        print("Blocked left")
    else:
        blocked_state['left'] = False

    if scan_list[3:7] != [2, 2, 2, 2]:
        blocked_state['centre'] = True
        print("Blocked centre")
    else:
        blocked_state['centre'] = False
    
    if scan_list[7:10] != [2, 2, 2]:
        blocked_state['right'] = True
        print ("Blocked right")
    else:
        blocked_state['right'] = False
    return blocked_state

# Decide on the action based on the blocked state and the direction of the car
# in relation to the destination
def decide_on_action(blocked_state):

    # Use global variable
    global direction

    # Inital filter on driving direction
    if direction == DrivingDirection.towards_destination:

        if blocked_state['centre']:

            # Check if left or right appears to be clear and
            # if so attempt turn in that direction
            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.left
                print("Turning left")
                return

            elif not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.right
                print("Turning right")
                return
        
            else:
                # If no preferred direction then attempt to turn left
                turn('left')
                direction = DrivingDirection.left
                print("Turning left - default action")
                return

        else:
            #If clear in front then drive forward
            move_forward()


    elif direction == DrivingDirection.right:

        # Want to turn left towards destination if possible and car has travelled at least 
        # about a car length forward
        if not blocked_state['left'] and distance_counter >= 2:
            turn('left')
            direction = DrivingDirection.towards_destination
            print("Turning left")
            return
        
        # If cannot turn left continue if possible and if not then turn
        if blocked_state['centre']:

            if not blocked_state['right']:
                turn('right')
                direction = DrivingDirection.away_from_destination
                print("Turning right")
                return
        
            else:
                turn('left')
                direction = DrivingDirection.towards_destination
                print("Turning left - default action")
                return

        else:
            #If clear in front then drive forward
            move_forward()

    elif direction == DrivingDirection.left:
        print("In driving direction left")

        # Want to turn right towards destination if possible and car has travelled at least 
        # about a car length forward
        if not blocked_state['right'] and distance_counter >= 2:
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Turning right")
            return

        # If cannot turn right continue if possible and if not then turn
        if blocked_state['centre']:

            if not blocked_state['left']:
                turn('left')
                direction = DrivingDirection.away_from_destination
                print("Turning left")
                return
        
            else:
                turn('right')
                direction = DrivingDirection.towards_destination
                print("Turning right - default action")
                return

        else:
            #If clear in front then drive forward
            move_forward()

    elif direction == DrivingDirection.away_from_destination:
        print("In driving direction - away from destination")

        # Want to turn right or left back towards destination if possible and car has travelled at least 
        # about a car length forward
        if not blocked_state['right'] and distance_counter >= 2:
            turn('right')
            direction = DrivingDirection.left
            print("Turning right")
            return

        if not blocked_state['left'] and distance_counter >= 2:
            turn('left')
            direction = DrivingDirection.right
            print("Turning left")
            return

        # If cannot turn right continue if possible and if not then turn
        if blocked_state['centre']:
            turn('right')
            direction = DrivingDirection.towards_destination
            print("Turning right - default action")
            return

        else:
            #If clear in front then drive forward
            move_forward()
    else:
        return 
        

def updateMap(blocked_state):
    global direction
    global map
    global car_position

    if map[car_position[0], car_position[1]] != 1:
        map[car_position[0], car_position[1]] = 0

    # Inital filter on driving direction
    if direction == DrivingDirection.towards_destination:

        if blocked_state['left']:
            map[car_position[0],car_position[1]-1] = 1
        else:
            if map[car_position[0],car_position[1]-1] != 1:
                map[car_position[0],car_position[1]-1] = 0

        if blocked_state['centre']:
            map[car_position[0]-1, car_position[1]] = 1
        else:
            if map[car_position[0]-1, car_position[1]] != 1:
                map[car_position[0]-1, car_position[1]] = 0

        if blocked_state['right']:
            map[car_position[0], car_position[1]+1] = 1
        else:
            if map[car_position[0], car_position[1]+1] != 1:
                map[car_position[0], car_position[1]+1] = 0

    elif direction == DrivingDirection.right:

        if blocked_state['left']:
            map[car_position[0]-1,car_position[1]] = 1
        else:
            if map[car_position[0]-1,car_position[1]] != 1:
                map[car_position[0]-1,car_position[1]] = 0

        if blocked_state['centre']:
            map[car_position[0], car_position[1]+1] = 1
        else:
            if map[car_position[0], car_position[1]+1] != 1:
                map[car_position[0], car_position[1]+1] = 0

        if blocked_state['right']:
            map[car_position[0]+1, car_position[1]] = 1
        else:
            if map[car_position[0]+1, car_position[1]] != 1:
                map[car_position[0]+1, car_position[1]] = 0
    
    elif direction == DrivingDirection.left:

        if blocked_state['left']:
            map[car_position[0]+1,car_position[1]] = 1
        else:
            if map[car_position[0]+1,car_position[1]] != 1:
                map[car_position[0]+1,car_position[1]] = 0

        if blocked_state['centre']:
            map[car_position[0], car_position[1]-1] = 1
        else:
            if map[car_position[0], car_position[1]-1] != 1:
                map[car_position[0], car_position[1]-1] = 0

        if blocked_state['right']:
            map[car_position[0]-1, car_position[1]] = 1
        else:
            if map[car_position[0]-1, car_position[1]] != 1:
                map[car_position[0]-1, car_position[1]] = 0

    else: # If direction away frpm destination

        if blocked_state['left']:
            map[car_position[0],car_position[1]+1] = 1
        else:
            if map[car_position[0],car_position[1]+1] != 1:
                map[car_position[0],car_position[1]+1] = 0

        if blocked_state['centre']:
            map[car_position[0]-1, car_position[1]] = 1
        else:
            if map[car_position[0]-1, car_position[1]] != 1:
                map[car_position[0]-1, car_position[1]] = 0

        if blocked_state['right']:
            map[car_position[0], car_position[1]-1] = 1
        else:
            if map[car_position[0], car_position[1]-1] != 1:
                map[car_position[0], car_position[1]-1] = 0

                

def main():

    print("Sys max size ", sys.maxsize)
    np.set_printoptions(threshold=sys.maxsize)
    #np.set_printoptions(threshold=220)


    # Initate dictionary to hold detected obstacle location in front of car
    blocked_state = {
        'left': False,
        'centre': False,
        'right': False
    }

    # Start loop to perform scan and take respective actions
    while True:
        # Get ultrasonic scan input 
        scan_list = fc.scan_step(35)
        print(scan_list)
        if not scan_list:
            continue
        # Wait for full scan to be received from the sensor
        if len(scan_list) != 10:
            continue

        
        # Check for obstacles
        blocked_state = check_scan(scan_list, blocked_state)
        print(scan_list)
        print ("Blocked state ", blocked_state)

        # Update map
        updateMap(blocked_state)

        print("Car position ", car_position)
        #Decide on actions based on obstacles and current driving direction
        decide_on_action(blocked_state)


        
        print(map)
        #print(map[car_position[0]-25:car_position[0]+25, car_position[1]-25:car_position[1]+25])
        #plt.imshow(map, interpolation='nearest')
        #plt.show()      
        



if __name__ == "__main__":

    print("If you want to quit.Please press q")

    try: 
        main()
    finally: 
        fc.stop()