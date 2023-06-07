#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from math import pi
from PyPID import *

# Parametri fisici robot
WHEEL_DIAMETER = 55

WALL_P_GAIN = .8
WALL_D_GAIN = .3

MAX_OUTPUT = 130
MAX_FRONT_DISTANCE = 200

SPEED = 200
TIME_DELTA = 10

motors_speed = (SPEED/(WHEEL_DIAMETER * pi))*360

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
left_distance_sensor = UltrasonicSensor(Port.S1)
front_distance_sensor = UltrasonicSensor(Port.S3)

wall_following_control = PropDer(WALL_P_GAIN, WALL_D_GAIN)

def run_motors(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def get_wall_cte(left_distance, right_distance):
    return right_distance - left_distance

right_distance = 0
left_distance = 0
front_distance = 0
current_wall_cte = 0
current_output = 0

while True:
    right_distance = right_distance_sensor.distance()
    left_distance = left_distance_sensor.distance()
    front_distance = front_distance_sensor.distance()

    current_wall_cte = get_wall_cte(left_distance, right_distance)

    wall_following_control.update(current_wall_cte)

    current_output = min(max(wall_following_control.output * 120 / (max(-MAX_FRONT_DISTANCE, min(front_distance, MAX_FRONT_DISTANCE))), -MAX_OUTPUT), MAX_OUTPUT)

    run_motors(motors_speed + current_output, motors_speed - current_output)

    wait(TIME_DELTA)
