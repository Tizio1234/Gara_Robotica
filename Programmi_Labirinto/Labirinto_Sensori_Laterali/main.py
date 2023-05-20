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

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
left_distance_sensor = UltrasonicSensor(Port.S2)
right_distance_sensor = UltrasonicSensor(Port.S1)

robot = DriveBase(left_motor, right_motor, 55, 100)
robot.stop()

P_GAIN = .8
D_GAIN = .2
MAX_TURN_RATE = 90
SPEED = 230
TIME_DELTA = 5

def cte():
    return right_distance_sensor.distance() - left_distance_sensor.distance()

min_turn_rate = -abs(MAX_TURN_RATE)
max_turn_rate = abs(MAX_TURN_RATE)

current_turn_rate = 0
current_cte = 0
last_cte = 0
current_cter = 0

while True:
    current_cte = cte()
    current_cter = current_cte - last_cte
    current_turn_rate = min(max_turn_rate, max(current_cte * P_GAIN + current_cter * D_GAIN, min_turn_rate))
    robot.drive(SPEED, current_turn_rate)
    wait(TIME_DELTA)
