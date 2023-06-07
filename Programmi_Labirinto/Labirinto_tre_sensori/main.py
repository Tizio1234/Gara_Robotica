#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from math import pi
from PyPID import *

# Parametri fisici robot
FRONT_TRIGGER_DISTANCE = 60
LATERAL_MAX_TRIGGER_DISTANCE = 300
WHEEL_DIAMETER = 55

WALL_P_GAIN = .4
WALL_D_GAIN = .2

SINGLE_WALL_P_GAIN = .2

ANG_P_GAIN = 1.0

# Offsets errori sensori vari
ANG_SPEED_OFFSET = 1
RIGHT_ANGLE_ANGLE = 70

SPEED = 200
TIME_DELTA = 10
ROTATING_SPEED = 100

TARGET_DISTANCE = 75

motors_speed = (SPEED/(WHEEL_DIAMETER * pi))*360

rotating_motors_speed = (ROTATING_SPEED/(WHEEL_DIAMETER * pi))*360

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
left_distance_sensor = UltrasonicSensor(Port.S1)
front_distance_sensor = UltrasonicSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)

wall_following_control = PropDer(WALL_P_GAIN, WALL_D_GAIN)

single_wall_following_control = Prop()

ang_speed_control = Prop(ANG_P_GAIN)

def run_motors(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def get_wall_cte(left_distance, right_distance):
    return right_distance - left_distance

def get_ang_speed_cte(current_ang_speed, target_ang_speed):
    return target_ang_speed - current_ang_speed

def turn_of(angle):
    if not angle: return
    starting_angle = gyro_sensor.angle()
    target_angle = starting_angle + angle
    if angle > 0:
        run_motors(rotating_motors_speed, -rotating_motors_speed)
        while gyro_sensor.angle() < target_angle:
            pass
    else:
        run_motors(-rotating_motors_speed, rotating_motors_speed)
        while gyro_sensor.angle() > target_angle:
            pass
    run_motors(0, 0)
    wall_following_control.reset()
    wait(50)
    #wall_following_control.reset()

front_distance = 0
right_distance = 0
left_distance = 0
ang_speed = 0
wall_cte = 0
single_wall_cte = 0
ang_speed_cte = 0

while True:
    front_distance = front_distance_sensor.distance()
    right_distance = right_distance_sensor.distance()
    left_distance = left_distance_sensor.distance()
    ang_speed = gyro_sensor.speed()

    if front_distance <= FRONT_TRIGGER_DISTANCE:
        if right_distance <= LATERAL_MAX_TRIGGER_DISTANCE:
            turn_of(-RIGHT_ANGLE_ANGLE)
        else:
            turn_of(RIGHT_ANGLE_ANGLE)
    else:
        if right_distance < LATERAL_MAX_TRIGGER_DISTANCE and left_distance < LATERAL_MAX_TRIGGER_DISTANCE: # wall following control
            wall_cte = get_wall_cte(left_distance, right_distance)
            wall_following_control.update(wall_cte)
            run_motors(motors_speed + wall_following_control.output, motors_speed - wall_following_control.output)
        elif right_distance < LATERAL_MAX_TRIGGER_DISTANCE:
            #ang_speed_cte = get_ang_speed_cte(ang_speed, ANG_SPEED_OFFSET)
            #ang_speed_control.update(ang_speed_cte)
            #run_motors(motors_speed + ang_speed_control.output, motors_speed - ang_speed_control.output)
            single_wall_cte = get_wall_cte(TARGET_DISTANCE, right_distance)
            single_wall_following_control.update()
            run_motors(motors_speed + single_wall_following_control.output,motors_speed - single_wall_following_control.output)
        elif left_distance < LATERAL_MAX_TRIGGER_DISTANCE:
            single_wall_cte = get_wall_cte(left_distance, TARGET_DISTANCE)
            single_wall_following_control.update()
            run_motors(motors_speed + single_wall_following_control.output,motors_speed - single_wall_following_control.output)
        else:
            ang_speed_cte = get_ang_speed_cte(ang_speed, ANG_SPEED_OFFSET)
            ang_speed_control.update(ang_speed_cte)
            run_motors(motors_speed + ang_speed_control.output, motors_speed - ang_speed_control.output)
