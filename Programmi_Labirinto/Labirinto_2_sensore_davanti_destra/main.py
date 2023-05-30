#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from PyPID import PropIntDer, PropDer
from math import pi

# Parametri fisici robot o strada
FRONT_TRIGGER_DISTANCE = 130
RIGHT_TRIGGER_DISTANCE = 150
TARGET_DISTANCE = 70
WHEEL_DIAMETER = 55
AXLE_TRACK = 100
ROAD_DIAMETER = 100

# Parametri comportamento robot
WALL_P_GAIN = 1.0
WALL_D_GAIN = 0.2
WALL_I_GAIN = 0.1
ANG_SPEED_P_GAIN = 1.0
ANG_SPEED_D_GAIN = .5
ANG_SPEED_I_GAIN = 8.0
CTE_LIST_LENGTH = 300
SPEED = 200
TURNING_SPEED = 50
TIME_DELTA = 10
GYRO_ANG_SPEED_COEFF = 0.9
RIGHT_ANGLE = 60

RIGHT_TURN = 0
LEFT_TURN = 1

time_delta_seconds = TIME_DELTA / 1000

ev3 = EV3Brick()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_distance_sensor = UltrasonicSensor(Port.S2)
front_distance_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3, Direction.COUNTERCLOCKWISE)

robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
robot.stop()

wall_following_control = PropDer(WALL_P_GAIN, WALL_D_GAIN)

motors_speed = (SPEED/(WHEEL_DIAMETER * pi)) * 360
target_ang_speed = (360 * SPEED) / (pi * ROAD_DIAMETER)

def distance_cte(distance):
    return TARGET_DISTANCE - distance

def ang_speed_cte(current_ang_speed, target_ang_speed):
    return target_ang_speed - current_ang_speed

def run_motors(left, right):
    left_motor.run(left)
    right_motor.run(right)

def get_ang_speed():
    return gyro_sensor.speed() * GYRO_ANG_SPEED_COEFF

def turn_to(parameter):
    angle = 0
    ang_speed = 0
    time = 0
    watch = StopWatch()
    if parameter == RIGHT_TURN:
        ang_speed_control = PropIntDer(ANG_SPEED_P_GAIN, ANG_SPEED_D_GAIN, ANG_SPEED_I_GAIN, CTE_LIST_LENGTH)
        while angle < RIGHT_ANGLE:
            while watch.time() < time + TIME_DELTA:
                pass
            time = watch.time()
            ang_speed = get_ang_speed()
            angle += ang_speed * time_delta_seconds
            ang_speed_control.update(ang_speed_cte(ang_speed, target_ang_speed))
            run_motors(motors_speed + ang_speed_control.output, motors_speed - ang_speed_control.output)
    elif parameter == LEFT_TURN:
        ang_speed_control = PropIntDer(ANG_SPEED_P_GAIN, ANG_SPEED_D_GAIN, ANG_SPEED_I_GAIN, CTE_LIST_LENGTH)
        while angle > -RIGHT_ANGLE:
            while watch.time() < time + TIME_DELTA:
                pass
            time = watch.time()
            ang_speed = get_ang_speed()
            angle += ang_speed * time_delta_seconds
            ang_speed_control.update(ang_speed_cte(ang_speed, -target_ang_speed))
            run_motors(motors_speed + ang_speed_control.output, motors_speed - ang_speed_control.output)

wall_ang_speed_control = PropIntDer(ANG_SPEED_P_GAIN, ANG_SPEED_D_GAIN, ANG_SPEED_I_GAIN, CTE_LIST_LENGTH)

while True:
    right_distance = right_distance_sensor.distance()
    front_distance = front_distance_sensor.distance()
    current_ang_speed = get_ang_speed()

    if front_distance <= FRONT_TRIGGER_DISTANCE:
        if right_distance <= RIGHT_TRIGGER_DISTANCE:
            turn_to(LEFT_TURN)
        else:
            turn_to(RIGHT_TURN)
    else:
        wall_following_control.update(distance_cte(right_distance))
        #wall_ang_speed_control.update(ang_speed_cte(current_ang_speed, wall_following_control.output))
        run_motors(motors_speed - wall_following_control.output, motors_speed + wall_following_control.output)
    
    wait(TIME_DELTA)
