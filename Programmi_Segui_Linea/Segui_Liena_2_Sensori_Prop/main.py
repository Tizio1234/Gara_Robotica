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

WHEEL_DIAMETER = 55
AXLE_TRACK = 214
PROPORTIONAL_GAIN = 1
SPEED = 100
DELTA = 10

# Create your objects here.
ev3 = EV3Brick()

motor_L = Motor(Port.D, Direction.COUNTERCLOCKWISE)
motor_R = Motor(Port.A)
robot = DriveBase(motor_R, motor_L, WHEEL_DIAMETER, AXLE_TRACK)

sensor_R = ColorSensor(Port.S1)
sensor_L = ColorSensor(Port.S2)

def CTE(ref_left:int, ref_right:int):
    return ref_left - ref_right

# Write your program here.
ev3.speaker.beep()

current_cte = 0

while True:
    current_cte = CTE(int(sensor_L.reflection()*1.607), sensor_R.reflection())
    robot.drive(SPEED, PROPORTIONAL_GAIN*current_cte)
    print(current_cte)
    wait(DELTA)