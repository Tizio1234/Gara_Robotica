#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from PyPID import PropIntDer

# Define motor and sensor objects
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, 55, 92)
right_distance_sensor = UltrasonicSensor(Port.S4)

TARGET_DISTANCE = 150
PROPORTIONAL_GAIN = 0.7
DERIVATIVE_GAIN = 0.3
INTEGRAL_GAIN = 0.1
SPEED = 250
TIME_DELTA = 10

control = PropIntDer(PROPORTIONAL_GAIN, INTEGRAL_GAIN, DERIVATIVE_GAIN, 300)

while True:
    cte = min(right_distance_sensor.distance() - TARGET_DISTANCE, 150)

    control.update(cte)

    robot.drive(SPEED, control.output)

    wait(TIME_DELTA)
