#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

FRONT_TRIGGER_DISTANCE = 35
RIGHT_TRIGGER_DISTANCE = 120
RIGHT_MAX_DISTANCE = 180
TARGET_DISTANCE = 70
P_GAIN = .5
SPEED = 200
ROTATING_SPEED = 100
RIGHT_ANGLE_ANGLE = 90
TIME_DELTA = 10


FORWARD = 0
ROTATING_TO_RIGHT = 1
ROTATING_TO_LEFT = 2
BACKWARDS = 3
STOP = 4

# Create your objects here.
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
front_distance_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, 55, 100)
robot.stop()

right_distance = 0
front_distance = 0
current_state = STOP

def motors_speed(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def motor_H(parameter):
    if parameter == FORWARD:
        motors_speed(SPEED, SPEED)
    elif parameter == BACKWARDS:
        motors_speed(-SPEED, -SPEED)
    elif parameter == ROTATING_TO_RIGHT:
        motors_speed(ROTATING_SPEED, -ROTATING_SPEED)
    elif parameter == ROTATING_TO_LEFT:
        motors_speed(-ROTATING_SPEED, ROTATING_SPEED)
    elif parameter == STOP:
        left_motor.stop()
        right_motor.stop()
    else:
        return
    current_state = parameter

def wait_for_angle(angle):
    starting_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - starting_angle) < abs(angle):
        pass

def measure():
    return (right_distance_sensor.distance(), front_distance_sensor.distance())

while True:
    right_distance, front_distance = measure()
    if front_distance <= FRONT_TRIGGER_DISTANCE:
        robot.stop()
        motor_H(STOP)
        if right_distance <= RIGHT_TRIGGER_DISTANCE:
            motor_H(ROTATING_TO_LEFT)
            wait_for_angle(RIGHT_ANGLE_ANGLE)
            motor_H(STOP)
        else:
            motor_H(ROTATING_TO_RIGHT)
            wait_for_angle(RIGHT_ANGLE_ANGLE)
            motor_H(STOP)
    else:
        if right_distance < RIGHT_MAX_DISTANCE:
            robot.drive(SPEED, (right_distance - TARGET_DISTANCE) * P_GAIN)
        else:
            robot.drive(SPEED, 0)
    wait(TIME_DELTA)