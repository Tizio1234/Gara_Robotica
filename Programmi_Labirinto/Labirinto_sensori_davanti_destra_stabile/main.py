#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from math import pi, cos, radians
from PyPID import *


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

FRONT_TRIGGER_DISTANCE = 60
RIGHT_TRIGGER_DISTANCE = 500
TARGET_DISTANCE = 70
WHEEL_DIAMETER = 55

WALL_P_GAIN = .4
WALL_D_GAIN = .2

SPEED = 200
RIGHT_ANGLE_ANGLE = 80
TIME_DELTA = 20
ROTATING_SPEED = 50

motors_speed = (SPEED/(WHEEL_DIAMETER * pi))*360

rotating_motors_speed = (ROTATING_SPEED/(WHEEL_DIAMETER * pi))*360

# Create your objects here.
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
front_distance_sensor = UltrasonicSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)

wall_following_control = PropDer(WALL_P_GAIN, WALL_D_GAIN)

ang_speed_control = Prop(1.0)

def run_motors(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def wall_cte(current_distance, target_distance):
    return current_distance - target_distance

def ang_cte(current_ang_speed, target_ang_speed):
    return target_ang_speed - current_ang_speed

def turn_of(angle):
    run_motors(0, 0)
    wait(500)
    print("turn")
    if not angle: return
    gyro_sensor.reset_angle(0)
    starting_angle = gyro_sensor.angle()
    target_angle = starting_angle + angle
    if angle > 0:
        run_motors(rotating_motors_speed, -rotating_motors_speed)
        while gyro_sensor.angle() < target_angle:
            pass
        run_motors(0, 0)
    else:
        run_motors(-rotating_motors_speed, rotating_motors_speed)
        while gyro_sensor.angle() > target_angle:
            pass
        run_motors(0, 0)
    wall_following_control.reset()

front_distance = 0
right_distance = 0

while True:
    front_distance = front_distance_sensor.distance()
    right_distance = right_distance_sensor.distance()

    if front_distance <= FRONT_TRIGGER_DISTANCE:
        if right_distance <= RIGHT_TRIGGER_DISTANCE:
            turn_of(-RIGHT_ANGLE_ANGLE)
        else:
            turn_of(RIGHT_ANGLE_ANGLE)
    else:
        if right_distance < RIGHT_TRIGGER_DISTANCE:
            cte = wall_cte(right_distance, TARGET_DISTANCE)
            wall_following_control.update(cte)
            ang_speed_control.update(ang_cte(gyro_sensor.speed(), wall_following_control.output))
            run_motors(motors_speed + wall_following_control.output, motors_speed - wall_following_control.output)
            wait(TIME_DELTA)
        else:
            run_motors(motors_speed, motors_speed)

    wait(TIME_DELTA)
