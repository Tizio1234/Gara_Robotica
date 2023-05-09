#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

TRIGGER_DISTANCE_FRONT = 70
TRIGGER_DISTANCE_RIGHT = 100

# Create your objects here.
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
front_distance_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, 55, 100)

def rotate(angle):
    starting_angle = gyro_sensor.angle()
    robot.drive(0, -90 if angle < 0 else 90)
    while abs(gyro_sensor.angle()) < abs(starting_angle) + abs(angle):
        pass
    robot.stop()

# Write your program here.

while True:
    right_distance = right_distance_sensor.distance()
    front_distance = front_distance_sensor.distance()
    if front_distance <= TRIGGER_DISTANCE_FRONT:
        if right_distance < TRIGGER_DISTANCE_RIGHT:
            rotate(-90)
        else:
            rotate(90)
    else:
        robot.drive(100, 0)