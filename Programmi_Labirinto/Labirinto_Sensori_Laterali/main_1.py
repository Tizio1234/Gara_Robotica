#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from general_pid import PropDer


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
left_distance_sensor = UltrasonicSensor(Port.S1)
right_distance_sensor = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, 55, 90)

P_GAIN = .55
D_GAIN = .1
MAX_TURN_RATE = 120
SPEED = 250
TIME_DELTA = 10

control = PropDer(P_GAIN, D_GAIN)

def cte():
    return right_distance_sensor.distance() - left_distance_sensor.distance()

min_turn_rate = -abs(MAX_TURN_RATE)
max_turn_rate = abs(MAX_TURN_RATE)

current_turn_rate = 0

while True:
    control.update(cte())
    #try:
    #    current_speed = SPEED * 10 / int(abs(current_turn_rate))
    #except ZeroDivisionError:
    #    current_speed = SPEED
    current_turn_rate = min(max(control.output, min_turn_rate), max_turn_rate)
    robot.drive(SPEED, current_turn_rate)
    wait(TIME_DELTA)
