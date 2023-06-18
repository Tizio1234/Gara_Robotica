#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import UltrasonicSensor, ColorSensor, Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase

ROBOT_SPEED = 200
ROTATING_SPEED = 90
FRONT_TRIGGER_DISTANCE = 500

TIME_DELTA = 30

REFLECTION_THRESHOLD = 30
def on_edge(reflection:int):
    return False #reflection > REFLECTION_THRESHOLD

# sensori
front_distance_sensor = UltrasonicSensor(Port.S2)
right_distance_sensor = UltrasonicSensor(Port.S3)
left_color_sensor = ColorSensor(Port.S1)
right_color_sensor = ColorSensor(Port.S4)

# motori
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

# oggetti controllo
robot = DriveBase(left_motor, right_motor, 55, 160)

last_active_sensor = True

front_distance = 0
right_distance = 0
front_sensor_state = False
right_sensor_state = False
rotating_speed_coefficient = 0

while True:
    front_distance = front_distance_sensor.distance()
    right_distance = right_distance_sensor.distance()

    front_sensor_state = front_distance < FRONT_TRIGGER_DISTANCE
    right_sensor_state = right_distance < FRONT_TRIGGER_DISTANCE

    if not front_sensor_state and not right_sensor_state:
        if last_active_sensor:
            rotating_speed_coefficient = 1
        else:
            rotating_speed_coefficient = -1
    else:
        if not front_sensor_state and right_sensor_state:
            rotating_speed_coefficient = 1
            last_active_sensor = True
        elif front_sensor_state and not right_sensor_state:
            rotating_speed_coefficient = 0
            last_active_sensor = False
        else:
            rotating_speed_coefficient = 0.3
    
    robot.drive(ROBOT_SPEED, ROTATING_SPEED * rotating_speed_coefficient)

    wait(TIME_DELTA)
