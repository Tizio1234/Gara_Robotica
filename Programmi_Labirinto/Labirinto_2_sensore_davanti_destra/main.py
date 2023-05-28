#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from PyPID import PropIntDer
from math import pi

# Parametri fisici robot o strada
FRONT_TRIGGER_DISTANCE = 150
RIGHT_TRIGGER_DISTANCE = 150
TARGET_DISTANCE = 70
WHEEL_DIAMETER = 55
AXLE_TRACK = 100
ROAD_DIAMETER = 300

# Parametri comportamento robot
WALL_P_GAIN = 1.0
WALL_D_GAIN = 0.2
WALL_I_GAIN = 0.1
SPEED = (250/(WHEEL_DIAMETER * pi)) * 360
TURNING_SPEED = 50
TIME_DELTA = 10
RIGHT_ANGLE_ANGLE = 90

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S4)
front_distance_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
robot.stop()

wall_following_control = PropIntDer(WALL_P_GAIN, WALL_D_GAIN, WALL_I_GAIN, 300)

def distance_cte(distance):
    return TARGET_DISTANCE - distance

def run_motors(left, right):
    left_motor.run(left)
    right_motor.run(right)

front_distance = 0
right_distance = 0

while True:
    right_distance = right_distance_sensor.distance()
    wall_cte = -distance_cte(right_distance)

    if front_distance <= FRONT_TRIGGER_DISTANCE:
        if right_distance <= RIGHT_TRIGGER_DISTANCE:

        else:
    
    else:
        wall_following_control.update(wall_cte)
        run_motors(SPEED + wall_following_control.output, SPEED - wall_following_control.output)
    
    wait(TIME_DELTA)
