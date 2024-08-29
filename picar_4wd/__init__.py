#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from .pwm import PWM
from .adc import ADC
from .pin import Pin
from .motor import Motor
from .servo import Servo
from .ultrasonic import Ultrasonic 
from .speed import Speed
from .filedb import FileDB  
from .utils import *
import time
from .version import __version__

soft_reset()
time.sleep(0.2)

# Config File:
config = FileDB("config")
left_front_reverse = config.get('left_front_reverse', default_value = False)
right_front_reverse = config.get('right_front_reverse', default_value = False)
left_rear_reverse = config.get('left_rear_reverse', default_value = False)
right_rear_reverse = config.get('right_rear_reverse', default_value = False)    
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 

# Init motors
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=left_front_reverse) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=right_front_reverse) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=left_rear_reverse) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=right_rear_reverse) # motor 4

# left_front_speed = Speed(12)
# right_front_speed = Speed(16)
left_rear_speed = Speed(25)
right_rear_speed = Speed(4)  

# Init Greyscale
gs0 = ADC('A5')
gs1 = ADC('A6')
gs2 = ADC('A7')

# Init Ultrasonic
us = Ultrasonic(Pin('D8'), Pin('D9'))

# Init Servo
# print("Init Servo: %s" % ultrasonic_servo_offset)

servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

def start_speed_thread():
    # left_front_speed.start()
    # right_front_speed.start()
    left_rear_speed.start()
    right_rear_speed.start()

##################################################################
# Grayscale 
def get_grayscale_list():
    adc_value_list = []
    adc_value_list.append(gs0.read())
    adc_value_list.append(gs1.read())
    adc_value_list.append(gs2.read())
    return adc_value_list

def is_on_edge(ref, gs_list):
    ref = int(ref)
    if gs_list[2] <= ref or gs_list[1] <= ref or gs_list[0] <= ref:  
        return True
    else:
        return False

def get_line_status(ref,fl_list):#170<x<300
    ref = int(ref)
    if fl_list[1] <= ref:
        return 0
    
    elif fl_list[0] <= ref:
        return -1

    elif fl_list[2] <= ref:
        return 1

########################################################
# Ultrasonic
ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []

errors = []

def run_command(cmd=""):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    # print(result)
    # print(status)
    return status, result


def do(msg="", cmd=""):
    print(" - %s..." % (msg), end='\r')
    print(" - %s... " % (msg), end='')
    status, result = eval(cmd)
    # print(status, result)
    if status == 0 or status == None or result == "":
        print('Done')
    else:
        print('Error')
        errors.append("%s error:\n  Status:%s\n  Error:%s" %
                      (msg, status, result))

def get_distance_at(angle):
    global angle_distance
    servo.set_angle(angle)
    time.sleep(0.04)
    distance = us.get_distance()
    angle_distance = [angle, distance]
    return distance

def get_status_at(angle, ref1=35, ref2=10):
    dist = get_distance_at(angle)
    if dist > ref1 or dist == -2:
        return 2
    elif dist > ref2:
        return 1
    else:
        return 0

def scan_step(ref):
    global scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    status = get_status_at(current_angle, ref1=ref)#ref1

    scan_list.append(status)
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            # print("reverse")
            scan_list.reverse()
        # print(scan_list)
        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False

########################################################
# Motors
def forward(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(power)
    right_rear.set_power(power)

def backward(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

def turn_left(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(power)
    right_rear.set_power(power)

def turn_right(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

def stop():
    left_front.set_power(0)
    left_rear.set_power(0)
    right_front.set_power(0)
    right_rear.set_power(0)

def set_motor_power(motor, power):
    if motor == 1:
        left_front.set_power(power)
    elif motor == 2:
        right_front.set_power(power)
    elif motor == 3:
        left_rear.set_power(power)
    elif motor == 4:
        right_rear.set_power(power)

# def speed_val(*arg):
#     if len(arg) == 0:
#         return (left_front_speed() + left_rear_speed() + right_front_speed() + right_rear_speed()) / 4
#     elif arg[0] == 1:
#         return left_front_speed()
#     elif arg[0] == 2:
#         return right_front_speed()
#     elif arg[0] == 3:
#         return left_rear_speed()
#     elif arg[0] == 4:
#         return right_rear_speed()

def speed_val():
    return (left_rear_speed() + right_rear_speed()) / 2.0

######################################################## 
if __name__ == '__main__':
    start_speed_thread()
    while 1:
        forward(1)
        time.sleep(0.1)
        print(speed_val())
