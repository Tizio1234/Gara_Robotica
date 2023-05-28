from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from PyPID import PropIntDer
from math import pi

# Parametri fisici robot o strada
FRONT_TRIGGER_DISTANCE = 150
RIGHT_TRIGGER_DISTANCE = 150
TARGET_DISTANCE = 70
WHEEL_DIAMETER = 55
AXLE_TRACK = 100
ROAD_DIAMETER = 300

# Parametri comportamento robot
ANG_SPEED_P_GAIN = 1.0
ANG_SPEED_D_GAIN = 0.2
ANG_SPEED_I_GAIN = 0.1
SPEED = 100
TURNING_SPEED = 50
CTE_LIST_LENGTH = 200
TIME_DELTA = 10
RIGHT_ANGLE_ANGLE = 90

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
gyro_sensor = GyroSensor(Port.S2)

motors_speed = (SPEED/(WHEEL_DIAMETER * pi)) * 360

left_motor_speed = (SPEED * (ROAD_DIAMETER - AXLE_TRACK)) / ROAD_DIAMETER
right_motor_speed = (SPEED * (ROAD_DIAMETER + AXLE_TRACK)) / ROAD_DIAMETER

ang_speed_control = PropIntDer(ANG_SPEED_P_GAIN, ANG_SPEED_D_GAIN, ANG_SPEED_I_GAIN, CTE_LIST_LENGTH)

def ang_speed_cte(current_ang_speed, target_ang_speed):
    return target_ang_speed - current_ang_speed

def run_motors(left, right):
    left_motor.run(left)
    right_motor.run(right)

run_motors(left_motor_speed, right_motor_speed)
