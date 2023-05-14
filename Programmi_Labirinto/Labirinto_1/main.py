#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from math import cos, radians

# Define motor and sensor objects
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, 55, 92)
right_distance_sensor = UltrasonicSensor(Port.S2)
front_distance_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)

TARGET_DISTANCE = 100
PROPORTIONAL_GAIN = 1.0
DERIVATIVE_GAIN = 0.0
INTEGRAL_GAIN = 0.0
SPEED = 200
TIME_DELTA = 10

current_cte = 0
last_cte = 0
current_cter = 0
current_sse = 0
current_turning_rate = 0

gyro_sensor.reset_angle(0)

while True:
    right_distance = right_distance_sensor.distance() * cos(radians(abs(gyro_sensor.angle())))

    current_cte = right_distance - TARGET_DISTANCE

    current_cter = current_cte - last_cte

    current_turning_rate = PROPORTIONAL_GAIN * current_cte + DERIVATIVE_GAIN * current_cter

    robot.drive(SPEED, current_turning_rate)

    wait(TIME_DELTA)
