#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Constants
WHEEL_DIAMETER = 55
AXLE_TRACK = 214
LINE_REFLECTION = 6
ENVIRONMENT_REFLECTION = 54
PROPORTIONAL_GAIN = 1.2
DERIVATIVE_GAIN = 1
INTEGRAL_GAIN = 1
SPEED_GAIN = 30
MAX_SPEED = 100
MAX_CTE_COUNT = 20

# Create your objects here.
ev3 = EV3Brick()

motor_L = Motor(Port.A)
motor_R = Motor(Port.B)
robot = DriveBase(motor_L, motor_R, WHEEL_DIAMETER, AXLE_TRACK)

sensor = ColorSensor(Port.S1)

watch = StopWatch()

# Configuration
CTE_COEFFICIENT = 2/(ENVIRONMENT_REFLECTION - LINE_REFLECTION)
CTE_OFFSET = 1 - CTE_COEFFICIENT*ENVIRONMENT_REFLECTION

# Functions
def CTE(reflection:int):
    return reflection*CTE_COEFFICIENT + CTE_OFFSET

def CTER(cte_0, cte_1, delta):
    return (cte_1-cte_0)/delta

def SSE(cte_list:list):
    return sum(cte_list)/len(cte_list)

def SPEED(turn_rate:int):
    return (MAX_SPEED*SPEED_GAIN)/turn_rate

# Write your program here.
ev3.speaker.beep()

current_cte = 0
current_cte_time = 0
current_cter = 0
current_sse = 0
current_speed = 0
current_turn_rate = 0

cte_list = []
for i in range(MAX_CTE_COUNT):
    cte_list.append({"cte":0, "time":0})

index = 1

watch.reset()

while True:
    current_cte = CTE(sensor.reflection())
    current_cte_time = watch.time()
    cte_list[index] = {"cte":current_cte, "time":current_cte_time}
    current_cter = CTER(cte_list[index - 1], cte_list[index], cte_list[index]["time"] - cte_list[index - 1]["time"])
    current_sse = SSE(cte_list)
    current_turn_rate = PROPORTIONAL_GAIN*current_cte + DERIVATIVE_GAIN*current_cter + INTEGRAL_GAIN*current_sse
    current_speed = max(MAX_SPEED, SPEED(current_turn_rate))
    robot.drive(current_speed, current_turn_rate)
