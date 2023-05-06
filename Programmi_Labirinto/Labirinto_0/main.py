#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from robot_labirinto import RobotLabirinto
from sys import exit


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
maze_robot = RobotLabirinto(100,
                            55,
                            Port.A,
                            Port.D,
                            Port.S2,
                            Port.S1,
                            Port.S3,
                            200,
                            100,
                            True,
                            True
                            )

# Write your program here.

maze_robot.rotate(90)
