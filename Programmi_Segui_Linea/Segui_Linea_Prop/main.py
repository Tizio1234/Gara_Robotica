#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

WHEEL_DIAMETER = 55
AXLE_TRACK = 188
LINE_REFLECTION = 6
ENVIRONMENT_REFLECTION = 47
PROPORTIONAL_GAIN = 150
DERIVATIVE_GAIN = 90
INTEGRAL_GAIN = 70
SPEED = 200
DELTA = 5
CTE_LIST_LEN = 400

# Create your objects here.
ev3 = EV3Brick()

motor_L = Motor(Port.D, Direction.COUNTERCLOCKWISE)
motor_R = Motor(Port.A)
robot = DriveBase(motor_R, motor_L, WHEEL_DIAMETER, AXLE_TRACK)

sensor = ColorSensor(Port.S1)

CTE_COEFFICIENT = 2/(ENVIRONMENT_REFLECTION - LINE_REFLECTION)
CTE_OFFSET = 1 - CTE_COEFFICIENT*ENVIRONMENT_REFLECTION

def CTE(reflection:int):
    return max(min(reflection*CTE_COEFFICIENT + CTE_OFFSET, 1), -1)

def CTER(cte_0, cte_1):
    return cte_1-cte_0

# Write your program here.
ev3.speaker.beep()

current_cte = 0
last_cte = 0
current_cter = 0
current_sse = 0
current_turn_rate = 0
current_speed = 0
cte_list = [0] * CTE_LIST_LEN
i = 0

while True:
    last_cte = current_cte
    current_cte = CTE(sensor.reflection())
    cte_list[i] = current_cte
    current_cter = CTER(current_cte, last_cte)
    current_sse = sum(cte_list)/CTE_LIST_LEN
    i += 1
    if i == CTE_LIST_LEN: i = 0
    current_turn_rate = PROPORTIONAL_GAIN*current_cte + DERIVATIVE_GAIN*current_cter + INTEGRAL_GAIN*current_sse
    current_speed = SPEED*(1 - abs(current_sse))
    #print(current_sse)
    robot.drive(current_speed, current_turn_rate)
    wait(DELTA)
