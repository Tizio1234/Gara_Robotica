#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from general_pid import Prop


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
left_distance_sensor = UltrasonicSensor(Port.S4)
right_distance_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, 45, 80)
robot.stop()

angular_speed_control = Prop(-1.0)

P_GAIN = .6
D_GAIN = .2
MAX_TURN_RATE = 120
SPEED = 260
TIME_DELTA = 3

def cte():
    return right_distance_sensor.distance() - left_distance_sensor.distance()

min_turn_rate = -abs(MAX_TURN_RATE)
max_turn_rate = abs(MAX_TURN_RATE)

current_turn_rate = 0
current_cte = 0
last_cte = 0
current_cter = 0
current_speed = 0

while True:
    current_cte = cte()
    current_cter = current_cte - last_cte
    current_turn_rate = min(max_turn_rate, max(current_cte * P_GAIN + current_cter * D_GAIN, min_turn_rate))
    try:
        current_speed = SPEED * 10 / int(abs(current_turn_rate))
    except ZeroDivisionError:
        current_speed = SPEED
    robot.drive(SPEED, current_turn_rate)
    wait(TIME_DELTA)
