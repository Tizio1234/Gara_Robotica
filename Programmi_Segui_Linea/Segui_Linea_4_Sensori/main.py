#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor,)
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from PyPID import PropDer


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Motors/Robot physical Parameters
LEFT_MOTOR_PORT = Port.D
LEFT_MOTOR_INVERTED = False
RIGHT_MOTOR_PORT = Port.A
RIGHT_MOTOR_INVERTED = False
SENSORS_PORTS = [Port.S1, Port.S2, Port.S3, Port.S4] # Sensors from left to right
SENSORS_ENVIRONMENT_VALUES = [50, 50, 50, 50]
SENSORS_LINE_VALUES = [7, 7, 7, 7]
WHEEL_DIAMETER = 55 # mm
AXLE_TRACK = 188 # mm
SPEED = 100 # mm/S
P_GAIN = 1.0
D_GAIN = 0.3

line_following_control = PropDer(P_GAIN, D_GAIN)

# Create your objects here.
#ev3 = EV3Brick()

motor_L = Motor(Port.A, Direction.CLOCKWISE if not LEFT_MOTOR_INVERTED else Direction.COUNTERCLOCKWISE)
motor_R = Motor(Port.C, Direction.CLOCKWISE if not RIGHT_MOTOR_INVERTED else Direction.COUNTERCLOCKWISE)
robot = DriveBase(motor_R, motor_L, WHEEL_DIAMETER, AXLE_TRACK)

SENSORS = [ColorSensor(sensor_port) for sensor_port in SENSORS_PORTS]

COEFFICIENTS = [1/(sensor_line - sensor_env) for sensor_env, sensor_line in zip(SENSORS_ENVIRONMENT_VALUES, SENSORS_LINE_VALUES)]
OFFSETS = [-coeff * sensor_env for coeff, sensor_env in zip(COEFFICIENTS, SENSORS_ENVIRONMENT_VALUES)]

while True:
    sensors_cte = -1.5 * SENSORS[0].reflection() - 0.5 * SENSORS[1].reflection() + 0.5 * SENSORS[2].reflection() + 1.5 * SENSORS[3].reflection()
    line_following_control.update(sensors_cte)

    robot.drive(SPEED, line_following_control.output)
