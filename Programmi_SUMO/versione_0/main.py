#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import UltrasonicSensor, ColorSensor, Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# sensori
front_distance_sensor = UltrasonicSensor(Port.S1)
left_color_sensor = ColorSensor(Port.S2)
right_color_sensor = ColorSensor(Port.S3)

# motori
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
