#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from PyPID import *
from math import pi


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
left_distance_sensor = UltrasonicSensor(Port.S1)
right_distance_sensor = UltrasonicSensor(Port.S2)
gyro_sensor = GyroSensor(Port.S3, Direction.COUNTERCLOCKWISE)

P_GAIN = .8
D_GAIN = .2
A_P_GAIN = 1.5
A_D_GAIN = .4
MAX_TURN_RATE = 190
SPEED = 200
WHEEL_DIAMETER = 55
TIME_DELTA = 10
TRIGGER_DISTANCE = 200

motors_speed = (SPEED/(WHEEL_DIAMETER * pi)) * 360

control = PropDer(P_GAIN, D_GAIN)

ang_speed_control = Prop(A_P_GAIN)

def wall_cte(right_distance, left_distance):
    return right_distance - left_distance

def ang_speed_cte(current_speed, target_speed):
    return target_speed - current_speed

def run_motors(left, right):
    left_motor.run(left)
    right_motor.run(right)

def turn_of(angle):
    if not angle: return
    starting_angle = gyro_sensor.angle()
    target_angle = starting_angle + angle
    if angle > 0:
        run_motors(motors_speed/2, -motors_speed/2)
        while gyro_sensor.angle() < target_angle:
            pass
    else:
        run_motors(-motors_speed/2, motors_speed/2)
        while gyro_sensor.angle() > target_angle:
            pass
    run_motors(0, 0)
    control.reset()

waiting_time = 100/SPEED

current_turn_rate = 0

while True:
    left_distance = left_distance_sensor.distance()
    right_distance = right_distance_sensor.distance()
    if left_distance_sensor.distance() >= TRIGGER_DISTANCE:
        wait(waiting_time)
        print("turn")
        turn_of(-60)
        run_motors(SPEED, SPEED)
        while left_distance >= TRIGGER_DISTANCE:
            pass
    elif right_distance_sensor.distance() >= TRIGGER_DISTANCE:
        wait(waiting_time)
        print("turn")
        turn_of(60)
        run_motors(SPEED, SPEED)
        while right_distance >= TRIGGER_DISTANCE:
            pass
    else:
        control.update(wall_cte(right_distance, left_distance))
        current_turn_rate = max(min(control.output, MAX_TURN_RATE), -MAX_TURN_RATE)
        ang_speed_control.update(ang_speed_cte(gyro_sensor.speed(), current_turn_rate))
        run_motors(motors_speed + ang_speed_control.output, motors_speed - ang_speed_control.output)
        wait(TIME_DELTA)
 