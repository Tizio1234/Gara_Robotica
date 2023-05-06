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

# Motors/Robot physical Parameters
LEFT_MOTOR_PORT = Port.D
LEFT_MOTOR_INVERTED = False
RIGHT_MOTOR_PORT = Port.A
RIGHT_MOTOR_INVERTED = False
SENSORS_PORTS = [Port.S1, Port.S2, Port.S3, Port.S4] # Sensors from left to right
SENSORS_ENVIRONMENT_VALUES = [50, 50, 50, 50]
SENSORS_LINE_VALUES = [7, 7, 7, 7]
WHEEL_DIAMETER = 55 # mm
AXLE_TRACK = 188 # mm
SPEED = 200 # mm/S

# Create your objects here.
#ev3 = EV3Brick()

motor_L = Motor(Port.D, Direction.CLOCKWISE if not LEFT_MOTOR_INVERTED else Direction.COUNTERCLOCKWISE)
motor_R = Motor(Port.A, Direction.CLOCKWISE if not LEFT_MOTOR_INVERTED else Direction.COUNTERCLOCKWISE)
robot = DriveBase(motor_R, motor_L, WHEEL_DIAMETER, AXLE_TRACK)

robot_sensors = [ColorSensor(sensor_port) for sensor_port in SENSORS_PORTS]

# Write your program here.
#ev3.speaker.beep()
